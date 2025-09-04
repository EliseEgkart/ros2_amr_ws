import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Header, Bool
from nav_msgs.msg import Odometry

import sys
from rclpy.executors import MultiThreadedExecutor
from concurrent.futures import Future

from tf_transformations import quaternion_from_euler,euler_from_quaternion

import numpy as np
from scipy.linalg import solve_discrete_are as dare 

from sensor_msgs.msg import LaserScan, PointCloud2
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sklearn.cluster import DBSCAN
from sklearn.linear_model import RANSACRegressor
import warnings
from sklearn.exceptions import UndefinedMetricWarning


import sensor_msgs_py.point_cloud2 as pc2
import struct

from amr_msgs.msg import Errormsg
import serial  ###5/5 update (STM)
import os
import json

from amr_msgs.msg import WheelMotor

amr_number = os.getenv('AMR_NUMBER', '0')

class Nodelet1(Node):
    def __init__(self):
        super().__init__('check_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        self.lidar_scan = self.create_subscription(LaserScan,'/scan',self.lidar_callback,qos_profile_sensor_data)
        self.point_cloud_pub_ = self.create_publisher(PointCloud2, '/clustered_points', qos_profile)
        self.error_point_pub_ = self.create_publisher(Errormsg,'/error',10)

        self.fail_point_pub_ = self.create_publisher(String,'/fail_point_pub',10)
        


        
         # from control_center
        self.control_from_center = self.create_subscription(String, '/control_from_center', self.control_from_center_callback, 10)

        # to control_center
        self.control_from_amr = self.create_publisher(String, '/control_from_amr', 10)
        
        self.lifecycle = False

        self.lift_leg_distance = 0.45   # 전체 길이 50cm - 다리 두께 1.5 *2 (다리 두께 절반이 1.5cm)
        self.threshold_distance = 10   # 첫번째 군집시 군집 안의 최대 거리
        self.threshold_between_leg_and_wall_distance = 0.2 ## 양 끝에 벽이 있을 경우 ( 가림막 )
        self.check_range = 1.5   # 1.5m 안에 책상 다리가 존재하도록 해야함

        self.e1 = None
        self.e1_old = None
        self.delta_e1 = None
        self.e1_threshold = 0.25
        self.cnt = 0
        self.cnt_outlier = 0
        self.initial_e1 = None
        self.initial_e1_set = []
        self.max_range_distance = 2.0

        self.number_set = 20 # 벽으로 인식하기 위한 최소 라이다 갯수
        self.min_point_leg = 2 # 다리로 인식하기 위한 최소 라이다 갯수 : 노이즈를 없애기 위한 것

        self.laser1XOff = -0.143555
        self.laser1YOff = 0.24698

        self.robot_x = 0.4
        self.robot_y = 0.6

    def generate_colors(self, num_colors):
        np.random.seed(0)
        colors = np.random.randint(0, 255, size=(num_colors, 3))
        return colors



    def control_from_center_callback(self, data):
        message = data.data
        if message == 'lqr_start':
            self.lifecycle = True
            msg = String()
            msg.data = 'lqr_start'
            self.control_from_amr.publish(msg)
        elif message == 'stop':
            self.lifecycle = False
            msg = String()
            msg.data = 'stop'
            self.control_from_amr.publish(msg)

        elif message == 'kill':
            self.shutdown_node()

    def shutdown_node(self):
        self.get_logger().info('Shutting down lqr_node1')
        self.lifecycle = False
        rclpy.shutdown()
        sys.exit(0)


    def statistical_outlier_removal(self, points, mean_k=2, std_dev_mul_thresh=1):
        if len(points) < mean_k:
            return np.array(points)

        # k-Nearest Neighbors를 사용하여 각 포인트의 이웃 평균 거리 계산
        distances = []
        for i in range(len(points)):
            point = points[i]
            diff = points - point
            dist = np.linalg.norm(diff[:, :2], axis=1)
            dist.sort()
            # mean_k까지의 거리 평균
            mean_distance = np.mean(dist[1:mean_k+1])
            distances.append(mean_distance)

        distances = np.array(distances)
        mean_distance = np.mean(distances)
        std_distance = np.std(distances)

        filtered_points = []
        unfiltered_points = []
        for i, distance in enumerate(distances):
            if abs(distance - mean_distance) < std_dev_mul_thresh * std_distance:
                filtered_points.append(points[i])

            else:
                unfiltered_points.append(points[i])

        return (np.array(filtered_points),np.array(unfiltered_points))

    def remove_angle_outliers(self, cluster, remove_fraction=0.1):
        """
        클러스터에서 angle 값의 양쪽 끝부분을 제거하는 함수

        :param cluster: numpy 배열, shape (N, 4), 클러스터 포인트 (x, y, z, angle)
        :param remove_fraction: float, 제거할 비율 (양쪽 끝에서 각각 remove_fraction 비율만큼 제거)
        :return: tuple, (filtered_points, unfiltered_points)
        """
        if len(cluster) == 0:
            return np.array(cluster), np.array(cluster)
        
        # 제거할 개수 계산
        remove_count = int(len(cluster) * remove_fraction)

        if remove_count == 0:
            return np.array(cluster), np.array([])

        # 양쪽 끝부분 제거
        filtered_cluster = cluster[remove_count:-remove_count]
        unfiltered_cluster = np.vstack((cluster[:remove_count], cluster[-remove_count:]))

        return filtered_cluster, unfiltered_cluster


    
    def lidar_callback(self, msg):
        if self.lifecycle == True:
            clustered_points = []

            error_msg = Errormsg()
            pub_fail = String()

            points = []
            min_angle = 15.0 * (np.pi / 180)  # 60도
            max_angle = 165.0 * (np.pi / 180)  # 120도

            for i in range(len(msg.ranges)):
                angle = msg.angle_min + i * msg.angle_increment
                distance = msg.ranges[i]
                if min_angle <= angle <= max_angle and msg.range_min < distance < msg.range_max:
                    x = distance * np.cos(angle)
                    y = distance * np.sin(angle)
                    z = 0 
                    points.append([x, y, z, angle])

    
            if not points:
                self.get_logger().warn('No valid points found in the scan data')
                return

            points = np.array(points)

            # for point in points:
            #     x, y, z, _ = point
            #     r, g, b = 0,255,0
            #     a = 255  # 알파 값
            #     rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
            #     clustered_points.append((x, y, z, rgb))


            
            clustering2 = DBSCAN(eps=0.03, min_samples=2).fit(points[:, :3])   ## 세분화 시켜서 다리를 분리, 다리같은 경우는 점들이 모여 있으므로 sample이 3개 이상은 있을 것이다.
            labels2 = clustering2.labels_

            unique_labels2 = set(labels2)

            colors = self.generate_colors(len(unique_labels2))
            cluster_means = []

            secondary_points = []

            for k2 in unique_labels2:
                if k2 == -1:
                    continue
                class_member_mask2 = (labels2 == k2)
                cluster2 = points[class_member_mask2]
                color = colors[k2]


                # for point in cluster2:
                #     x, y, z, _ = point
                #     r, g, b = color
                #     a = 255  # 알파 값
                #     rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                #     clustered_points.append((x, y, z, rgb)) ## 여기는 클러스터링 된 집단


                min_x, min_y = np.min(cluster2[:, :2], axis=0)
                max_x, max_y = np.max(cluster2[:, :2], axis=0)

                distance_between_point = np.sqrt((max_x-min_x)**2+(max_y-min_y)**2)


                if len(cluster2) >= self.number_set and distance_between_point > self.threshold_between_leg_and_wall_distance: 
                    cluster2 = self.filter_and_ransac(cluster2, self.number_set)

                secondary_points.append([cluster2])



                

            flattened_points = [item for sublist in secondary_points for item in sublist]

            # 리스트의 배열을 수직으로 쌓아 하나의 배열로 만듭니다
            try:
                secondary_points = np.vstack(flattened_points)
            except ValueError as e:
                self.get_logger().error(f"No secondary_points")
                return


            # for point in secondary_points:
            #     x, y, z, _  = point
            #     r, g, b = 0,0,255
            #     a = 255  # 알파 값
            #     rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
            #     clustered_points.append((x, y, z, rgb)) ## 여기는 클러스터링 된 집단     




            filtered_cluster2 = DBSCAN(eps=0.01, min_samples=3).fit(secondary_points[:, :3]) 

            labels3 = filtered_cluster2.labels_

            unique_labels3 = set(labels3)

            colors2 = self.generate_colors(len(unique_labels3))


            for k3 in unique_labels3:
                if k3 == -1:
                    continue
                class_member_mask3 = (labels3 == k3)
                filtered_cluster3 = secondary_points[class_member_mask3]
                color2 = colors2[k3]

                    

                if len(filtered_cluster3) > self.min_point_leg:
                    mean_x = np.mean(filtered_cluster3[:, 0])
                    mean_y = np.mean(filtered_cluster3[:, 1])
                    mean_z = np.mean(filtered_cluster3[:, 2])
                    angle = np.mean(filtered_cluster3[:, 3])
                    cluster_means.append([mean_x, mean_y, mean_z,angle,len(filtered_cluster3)])

                    
                    # x, y, z = mean_x,mean_y,mean_z
                    # r, g, b = 0,255,0
                    # a = 255  # 알파 값
                    # rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                    # clustered_points.append((x, y, z, rgb))  ## 클러스터링에서 선 지우고 남은 거의 평균 점

                    # for point in filtered_cluster3:
                    #     # x, y, z, _ = point
                    #     # # r, g, b = color2
                    #     # # a = 255  # 알파 값
                    #     # # rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                    #     # # clustered_points.append((x, y, z, rgb)) ## 클러스터링에서 선 지우고 남은 거
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "laser"

            fields = [
                pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='rgb', offset=12, datatype=pc2.PointField.UINT32, count=1),
            ]


            if cluster_means is None:
                self.get_logger().info("cluster_means is None.")
                return

            


            for index, mean in enumerate(cluster_means[:3]):
                distance = np.linalg.norm(mean)  # 로봇이 원점(0, 0, 0)에 있다고 가정

            min_two_leg_accurate = 10
            min_between_leg_and_robot = 100
            two_index = [0.0, 0.0, 0.0, 0.0]

            point_index1 = 0
            point_index2 = 0

            point_index_list = []
            
            mean_set = []

            for i in range(len(cluster_means)):
                for j in range(i + 1, len(cluster_means)):
                    mean1 = cluster_means[i]
                    mean2 = cluster_means[j]

                    self.amr_length = 0.45
                    self.roi_x = 0.541 + 0.3
                    self.roi_y = 0.665 + 0.15

                    if (mean1[1] > self.amr_length / 2.0 and
                        mean1[1] < self.amr_length / 2.0 + self.roi_y and
                        mean2[1] > self.amr_length / 2.0 and
                        mean2[1] < self.amr_length / 2.0 + self.roi_y and
                        mean1[0] > -1 * self.roi_x / 2.0 and
                        mean1[0] < self.roi_x / 2.0 and
                        mean2[0] > -1 * self.roi_x / 2.0 and
                        mean2[0] < self.roi_x / 2.0):
                        
                        dx = mean2[0] - mean1[0]
                        dy = mean2[1] - mean1[1]

                        if dx != 0:
                            slope = np.fabs(dy / dx)
                        else:
                            slope = float('inf')

                        if slope < 1.732:
                            distance_between_leg = np.sqrt(dx**2+dy**2)
                            error = (distance_between_leg-self.lift_leg_distance)**2
                            mean_set.append((i, j, error))
                            # self.get_logger().info(f'{error}')  

            if mean_set:
                i, j, min_error = min(mean_set, key=lambda x: x[2])
                
                if min_error < 0.0025:
                    if self.cnt < 10:
                        self.cnt += 1
                    else:
                        point_index_list.append((i, j))
            

            point_indices_1_result = []
            point_indices_2_result = []
            if cluster_means and point_index_list:
                for point_indices in point_index_list:
                    
                    x1, y1, z1 , _ , _= cluster_means[point_indices[0]]
                    x2, y2, z2 , _ , _= cluster_means[point_indices[1]]
                    
                    # distance_between_leg_and_robot = np.sqrt((x1+x2)**2+(y1+y2)**2)/2.0   # 얻은 포인트와 로봇 사이 거리
                    # if distance_between_leg_and_robot < self.check_range and distance_between_leg_and_robot < min_between_leg_and_robot:
                    #     min_between_leg_and_robot = distance_between_leg_and_robot
                    point_indices_1_result = point_indices[0]
                    point_indices_2_result = point_indices[1]

            
            

            try:
                x1, y1, z1, _ , _ = cluster_means[point_indices_1_result]
                r, g, b = 255, 0, 0  # 빨간색
                a = 255  # 알파 값
                rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                clustered_points.append((x1, y1, z1, rgb))      
                p1 = [x1,y1]

                x2, y2, z2 , _ , _= cluster_means[point_indices_2_result]
                r, g, b = 255, 0, 0 # 빨간색
                a = 255  # 알파 값
                rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                clustered_points.append((x2, y2, z2, rgb))   
                p2 = [x2,y2]               
                error_array = self.error_cal(p1,p2)
                
                error_msg.e1 = error_array[0]
                self.e1 = error_array[0]
                error_msg.e2 = error_array[1]
                error_msg.e3 = error_array[2]

                error_msg.x1 = x1
                error_msg.x2 = x2
                error_msg.y1 = y1
                error_msg.y2 = y2


                self.error_point_pub_.publish(error_msg)

                pub_fail.data = 'Success'
                self.fail_point_pub_.publish(pub_fail)

                # self.get_logger().info(f'e1:{error_array[0]} , e2 : {error_array[1]}, e3 : {error_array[2]}')

            except:
                # cluster_means가 비어 있을 때는 아무것도 하지 않음
                # self.get_logger().info("cluster_means is empty, no action taken.")

                pub_fail.data = 'Fail'
                self.fail_point_pub_.publish(pub_fail)

                
            pc2_msg = pc2.create_cloud(header, fields, clustered_points)
            self.point_cloud_pub_.publish(pc2_msg) 

    def filter_and_ransac(self, cluster, number_set):
        if len(cluster) < number_set:  # RANSAC을 적용할 최소 샘플 수
            return cluster

        X = cluster[:, 1].reshape(-1, 1)
        y = cluster[:, 0]
        
        # if number_set < 8:
        #     number_set = 8

        ransac = RANSACRegressor(
            max_trials=10,        # 최대 반복 횟수
            min_samples= int(number_set/4),        # 모델을 추정하는 데 필요한 최소 샘플 수
            residual_threshold=0.01 # 잔차의 최대 허용 값
        )
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", category=UndefinedMetricWarning)
            ransac.fit(X, y)
        inlier_mask = ransac.inlier_mask_

        filtered_cluster = cluster[~inlier_mask]

        return filtered_cluster      

    def cos_2_distance(self, r1, r2, theta):
        return np.sqrt(r1**2+r2**2-2*r1*r2*np.cos(theta))
    
    def error_cal(self, point1, point2):
        ## 왼쪽 다리가 b 오른쪽 다리가 a
        if np.arctan2(point1[1],point1[0]) > np.arctan2(point2[1],point2[0]):
            bx = point1[0]  ## arctan이 큰 쪽이 왼쪽 --> point1
            by = point1[1]
            ax = point2[0]
            ay = point2[1]
        else:
            bx = point2[0]  ## arctan이 큰 쪽이 왼쪽 --> point2
            by = point2[1]
            ax = point1[0]
            ay = point1[1]
        # 중심의 좌표
        xm = (ax + bx) / 2
        ym = (ay + by) / 2
        # 수선의 발의 좌표
        xp = ((bx - ax) * (ax + bx) + ((by - ay) * (ay + by)) / 2) / ((by - ay)**2 + (bx - ax)**2)
        yp = ((by - ay) * xp) / (bx - ax)
        # 중심과 수선의 발의 거리
        e3 = np.sqrt((xm-xp)**2+(ym-yp)**2)

        return [-((bx**2+by**2)-(ax**2+ay**2))/2.0/np.sqrt((bx-ax)**2+(by-ay)**2) , np.arcsin((by-ay)/np.sqrt((bx-ax)**2+(by-ay)**2)), e3]
    

class EKF_SLAM:
    def __init__(self, state_dim, landmark_dim, dt, V, init_landmark ,init_pose_x, init_pose_y, init_yaw):
        self.state_dim = state_dim
        self.landmark_dim = landmark_dim
        self.dt = dt
        self.V = V 
        self.init_landmark = init_landmark
        self.odom_init_x = init_pose_x
        self.odom_init_y = init_pose_y
        self.odom_init_yaw = init_yaw


        self.P_hat = np.eye((self.state_dim + 2*self.landmark_dim))   ## x,y,psi,V, landmark ==> dim + 2 8 land_dim
        self.P_hat[0,0] = 0
        self.P_hat[1,1] = 0
        self.P_hat[2,2] = 0
        self.P_hat[3,3] = 0
        self.P_hat[4,4] = 0
        self.P_hat[5,5] = 0
        self.P_hat[6,6] = 0
        self.P_hat[7,7] = 0

        self.Qe = np.zeros(((self.state_dim + 2*self.landmark_dim),(self.state_dim + 2*self.landmark_dim)))
        self.Re = np.eye((2 + 2*self.landmark_dim))  ## psi,v,landmark ==> 2 + 2* land_dim

        self.Qe[0,0] = 0
        self.Qe[1,1] = 0
        self.Qe[2,2] = 0
        self.Qe[3,3] = 0
        
        self.Qe[4,4] = 0.01
        self.Qe[5,5] = 0.01
        self.Qe[6,6] = 0.01
        self.Qe[7,7] = 0.01

        self.Re[0,0] = 0.001
        self.Re[1,1] = 0.001

        angle_noise = 0.01
        dist_noise = 0.01
          
        for i in range(self.landmark_dim):
            self.Re[2*i+2,2*i+2] = angle_noise
            self.Re[2*i+3,2*i+3] = dist_noise   
        
        self.x_hat = np.zeros((self.state_dim + 2*self.landmark_dim, 1))

        self.x_hat[0][0] = 0
        self.x_hat[1][0] = 0
        self.x_hat[2][0] = np.pi/2
        self.x_hat[3][0] = self.V
        self.x_hat[4][0] = self.init_landmark[0][0]  # landmark1 x
        self.x_hat[5][0] = self.init_landmark[0][1]  # landmark1 y
        self.x_hat[6][0] = self.init_landmark[1][0]  # landmark2 x
        self.x_hat[7][0] = self.init_landmark[1][1]  # landmark2 y


        self.x_ = np.zeros((3, 1))
        self.x_[0][0] = 0
        self.x_[1][0] = 0
        self.x_[2][0] = np.pi/2

          


    def EKF_propagate(self, x_hat, P_hat, Qe, dt, step):

        A = np.zeros((self.state_dim + 2*self.landmark_dim, self.state_dim + 2*self.landmark_dim))
        A[0, 2] = -x_hat[3][0] * np.sin(x_hat[2][0])
        A[0, 3] = np.cos(x_hat[2][0])
        A[1, 2] = x_hat[3][0] * np.cos(x_hat[2][0])
        A[1, 3] = np.sin(x_hat[2][0])
        A = np.eye(self.state_dim + 2*self.landmark_dim) + A * dt
        P_hat = np.dot(A, np.dot(P_hat, A.T)) + Qe  # 8*8


        if(step == 3 or step == 4):
            P_hat[4:8, :] = 0
            P_hat[:, 4:8] = 0

        return x_hat, P_hat

    def propagate_x(self, x, dt, input):
        xdot = np.zeros((self.state_dim + 2*self.landmark_dim, 1))
        
        xdot[0][0] = input[1] * np.cos(x[2][0])  # xdot
        xdot[1][0] = input[1] * np.sin(x[2][0])  # ydot
        xdot[2][0] = input[0]  # yaw rate
        xdot[3][0] = 0.0  # 가속도
        x = x + xdot * dt  # time integration (Euler scheme)
        x[3][0] = input[1]  # 속도

        return x
    
    def odom_x(self, x_hat, pose_x, pose_y, yaw, v_x):

        (x_hat[0][0],x_hat[1][0]) = self.transformation([self.odom_init_x,self.odom_init_y],[pose_x, pose_y],self.odom_init_yaw - np.pi/2)

        x_hat[2][0] = (yaw - self.odom_init_yaw + np.pi/2 + np.pi) % (2 * np.pi) - np.pi
        x_hat[3][0] = v_x

        return x_hat
    
    def transformation(self, init, before_tran, yaw):

        a_x = init[0]
        a_y = init[1]
        psi = yaw

        b_x = before_tran[0]
        b_y = before_tran[1]

        diff = np.array([[b_x - a_x],[b_y - a_y]])

        rotation_matrix = np.array([[np.cos(psi), np.sin(psi)],[-np.sin(psi), np.cos(psi)]]) 
        trans = np.dot(rotation_matrix, diff)
        return (trans[0][0],trans[1][0])
       

    def EKF_update(self, x_hat, z, P_hat, Re, step):
        Hx = []
        Hx = np.array([[x_hat[2][0]], [x_hat[3][0]]])


        # 반복문을 통해 Hx 확장
        for j in range(2, (len(x_hat) - 2) // 2 + 1):
            angle_diff = np.arctan2(x_hat[2 * j + 1] - x_hat[1], x_hat[2 * j] - x_hat[0]) - x_hat[2]
            distance = np.sqrt((x_hat[0] - x_hat[2 * j]) ** 2 + (x_hat[1] - x_hat[2 * j + 1]) ** 2)
            
            # 새로운 값을 2차원 배열로 변환하여 Hx에 세로로 추가
            Hx = np.vstack([Hx, [[angle_diff.item()]], [[distance.item()]]])

        
        H_ = np.array([[0, 0, 1, 0] + [0] * 2 * self.landmark_dim, [0, 0, 0, 1] + [0] * 2 * self.landmark_dim])

        for j in range(2, (len(x_hat) - 2) // 2 + 1):
            dist3 = np.sqrt((x_hat[2 * j] - x_hat[0]) ** 2 + (x_hat[2 * j + 1] - x_hat[1]) ** 2)
            dist4 = np.sqrt((x_hat[0] - x_hat[2 * j]) ** 2 + (x_hat[1] - x_hat[2 * j + 1]) ** 2)
            

            H3_ = [(x_hat[2 * j + 1][0] - x_hat[1][0]) / (dist3[0] ** 2), -(x_hat[2 * j][0] - x_hat[0][0]) / (dist3[0] ** 2), -1, 0] + \
                [0] * (j - 2) * 2 + [-(x_hat[2 * j + 1][0] - x_hat[1][0]) / (dist3[0] ** 2), (x_hat[2 * j][0] - x_hat[0][0]) / (dist3[0] ** 2)] + \
                [0] * (len(x_hat) - 2 - j * 2)

            H4_ = [(x_hat[0][0] - x_hat[2 * j][0]) / dist4[0], (x_hat[1][0] - x_hat[2 * j + 1][0]) / dist4[0], 0, 0] + \
                [0] * (j - 2) * 2 + [-(x_hat[0][0] - x_hat[2 * j][0]) / dist4[0], -(x_hat[1][0] - x_hat[2 * j + 1][0]) / dist4[0]] + \
                [0] * (len(x_hat) - 2 - j * 2)
            
            H_ = np.vstack([H_, H3_, H4_])



        K_ = np.dot(np.dot(P_hat, H_.T), np.linalg.inv(np.dot(H_, np.dot(P_hat, H_.T)) + Re))  # 


        x_hat = x_hat + np.dot(K_, (z - Hx))
        P_hat = np.dot(np.eye(self.state_dim + 2 * self.landmark_dim) - np.dot(K_, H_), P_hat)

        if(step == 3 or step == 4):
            P_hat[4:8, :] = 0
            P_hat[:, 4:8] = 0
            
        return x_hat, P_hat
 
        
    def generate_z(self, x, x1, y1, x2, y2, landmark_dim):
        # DTR = np.pi / 180
        z = np.zeros((2 + 2*landmark_dim,1))
        z[0][0] = x[2][0] #+ 3 * DTR * np.random.randn()
        z[1][0] = x[3][0] #+ 0.2 * np.random.randn()
        z[2][0] = np.arctan2(y1,x1) - np.pi/2
        z[3][0] = np.sqrt(x1**2 + y1**2)
        z[4][0] = np.arctan2(y2,x2) - np.pi/2
        z[5][0] = np.sqrt(x2**2 + y2**2)

        return z
    

class Nodelet2(Node):
    def __init__(self):
        super().__init__('parking_lqr')

        self.lqr_publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lift_pub = self.create_publisher(Bool, '/lift' ,10)
        self.error_sub_ = self.create_subscription(Errormsg,'/error',self.lqr_command,10)

        self.stm_pub = self.create_publisher(String, '/stm_command', 10)
        self.fail_point_pub = self.create_subscription(String,'/fail_point_pub',self.fail_point,10)
        


        self.control_from_center = self.create_subscription(String, '/control_from_center', self.control_from_center_callback, 10)
        # to control_center
        self.control_from_amr = self.create_publisher(String, '/control_from_amr', 10)


        self.joy_auto_sub = self.create_subscription(String, '/joy_auto', self.joy_auto_callback, 10)
        self.joy = True

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )     
        
        self.point_cloud_pub_2 = self.create_publisher(PointCloud2, '/clustered_points_filter', qos_profile)

        self.odom_sub = self.create_subscription(Odometry, '/odom',self.vel_sub_callback, 10)


        self.lifecycle = False

        self.Ts = 0.05 # Sampling time
        self.V = 0.15
        self.V1 = 0.15
        self.timer = self.create_timer(self.Ts, self.timer_callback)

        self.A = np.array([[1, self.V*self.Ts],  
                    [0, 1]]) 
        self.B = np.array([[0],  
                    [self.Ts]]) 
        
        self.R = np.array([1]) # Penalty for angular velocity effort 클수록 input이 작아짐

        self.Q = np.array([[2.0, 0], # Penalize X position error 클수록 error가 빨리 작아짐
                    [0, 0.1]])  # Penalize YAW ANGLE heading error 

        self.e1 = 0
        self.e2 = 0
        self.e3 = 0

        self.e1_hat = 0
        self.e2_hat = 0
        self.e3_hat = 1000  # 초기값이 정해지기 전에 step넘어가는 것을 방지하기 위해 큰 값

        self.e3_old = 0
        self.e3_threshold = 0.6
        self.step = 0
        self.distance_center = 0.70
        self.distance_stop_center = 0.35
        self.gain = 1.5

        self.tmp = 0
        self.count_stop = 0
        
        self.count_wait_between_1_2 = 0

        

        self.e3_count = 0
        
        self.x1 = 0
        self.y1 = 0
        self.x2 = 0
        self.y2 = 0

        self.x1_1 = 0
        self.y1_1 = 0
        self.x2_1 = 0
        self.y2_1 = 0
        self.x1_old1 = 0
        self.y1_old1 = 0
        self.x2_old1 = 0
        self.y2_old1 = 0
        
        self.x1_ = 0
        self.y1_ = 0
        self.x2_ = 0
        self.y2_ = 0
        self.input = np.zeros(2)

        self.is_lifted = Bool()
        self.is_lifted.data = False


        self.first_loop_detect_leg = True
        self.ekf_slam = None
        self.v_x = 0.0
        self.w_z = 0.0

        self.last_time = self.get_clock().now()
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.yaw = 0.0
        self.count_step1 = 0
        self.count_step11 = 0
        self.count_step2 = 0
        self.count_step3 = 0
        self.x1_old = 0
        self.y1_old = 0
        self.x2_old = 0
        self.y2_old = 0
        self.step_1_1 = False
        self.step_1_2 = False
        self.step_2_ready = False
        self.step_3_ready = False


    def control_from_center_callback(self, data):
        message = data.data
        if message == 'lqr_start':
            self.lifecycle = True
        elif message == 'stop':
            self.lifecycle = False

        elif message == 'kill':
            self.shutdown_node()


    def shutdown_node(self):
        self.get_logger().info('Shutting down lqr_node2')
        self.lifecycle = False
        self.future.set_result(True)  # Future 객체 완료
        rclpy.shutdown()
        sys.exit(0)


    def transformation(self, robot_pos, landmark_xy):
        robot_pos_x = robot_pos[0]
        robot_pos_y = robot_pos[1]
        robot_pos_psi = robot_pos[2] - np.pi/2


        landmark_x = landmark_xy[0]
        landmark_y = landmark_xy[1]

        diff = np.array([[landmark_x - robot_pos_x],[landmark_y - robot_pos_y]])

        rotation_matrix = np.array([[np.cos(robot_pos_psi), np.sin(robot_pos_psi)],[-np.sin(robot_pos_psi), np.cos(robot_pos_psi)]]) 
        trans = np.dot(rotation_matrix, diff)
        return (trans[0][0],trans[1][0])
    

    def joy_auto_callback(self, data):
        message = data.data

        if message == 'joy':
            self.joy = True

        elif message == 'auto':
            self.joy = False
            
    def error_cal(self, point1, point2):
        ## 왼쪽 다리가 b 오른쪽 다리가 a
        if np.arctan2(point1[1],point1[0]) > np.arctan2(point2[1],point2[0]):
            bx = point1[0]  ## arctan이 큰 쪽이 왼쪽 --> point1
            by = point1[1]
            ax = point2[0]
            ay = point2[1]
        else:
            bx = point2[0]  ## arctan이 큰 쪽이 왼쪽 --> point2
            by = point2[1]
            ax = point1[0]
            ay = point1[1]
        # 중심의 좌표
        xm = (ax + bx) / 2
        ym = (ay + by) / 2
        # 수선의 발의 좌표
        xp = ((bx - ax) * (ax + bx) + ((by - ay) * (ay + by)) / 2) / ((by - ay)**2 + (bx - ax)**2)
        yp = ((by - ay) * xp) / (bx - ax)
        # 중심과 수선의 발의 거리
        e3 = np.sqrt((xm-xp)**2+(ym-yp)**2)

        return [-((bx**2+by**2)-(ax**2+ay**2))/2.0/np.sqrt((bx-ax)**2+(by-ay)**2) , np.arcsin((by-ay)/np.sqrt((bx-ax)**2+(by-ay)**2)), e3]
    

    def timer_callback(self):

        if self.lifecycle == True:
            clustered_points = []

            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "laser"

            fields = [
                pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='rgb', offset=12, datatype=pc2.PointField.UINT32, count=1),
            ]


            if self.e3 != 0.0 :
                self.e3_count += 1
            
            if self.e3_count == 11 :
                self.step = 1
                # self.step_1_1 = True  # 뒷다리로 바뀌기 위한 첫번째 조건 (앞다리 인식 후)

            if self.step_1_1:
                if self.e3_hat < self.e3_threshold:    # 앞다리 가까워졌을 때 : 두번째 조건
                    # print('step_1_1:',self.e3_hat,self.e3_threshold)
                    self.step_1_2 = True
            
            if self.step_1_2:
                if self.e3_hat > self.distance_center:
                    self.step_2_ready = True

            # self.get_logger().info(f'{self.step_1_1},{self.step_1_2},{self.step_2_ready},{self.e3_hat}')
            


            if self.ekf_slam is not None and self.step_2_ready and self.step == 1:
                self.step = 2
                
                # x_l1,y_l1 = self.transformation((self.ekf_slam.x_hat[0][0],self.ekf_slam.x_hat[1][0],self.ekf_slam.x_hat[2][0]), (self.ekf_slam.x_hat[4][0],self.ekf_slam.x_hat[5][0])) 

            # self.get_logger().info(f'STEP: {self.step}')


            if self.ekf_slam is not None:
                if (self.joy == True):
                    self.input[0] = self.w_z
                    self.input[1] = self.v_x


                if self.read_point == False:
                    self.ekf_slam.Qe[4,4] = 0
                    self.ekf_slam.Qe[5,5] = 0
                    self.ekf_slam.Qe[6,6] = 0
                    self.ekf_slam.Qe[7,7] = 0

                else:
                    self.ekf_slam.Qe[4,4] = 0.01
                    self.ekf_slam.Qe[5,5] = 0.01
                    self.ekf_slam.Qe[6,6] = 0.01
                    self.ekf_slam.Qe[7,7] = 0.01                
                
                

                #self.ekf_slam.x_hat = self.ekf_slam.propagate_x(self.ekf_slam.x_hat, self.Ts, self.input)  # true state propagation

                self.ekf_slam.x_hat = self.ekf_slam.odom_x(self.ekf_slam.x_hat, self.pose_x, self.pose_y, self.yaw, self.v_x)

                z = self.ekf_slam.generate_z(self.ekf_slam.x_hat, self.x1, self.y1, self.x2, self.y2, self.ekf_slam.landmark_dim)
                self.ekf_slam.x_hat, self.ekf_slam.P_hat = self.ekf_slam.EKF_propagate(self.ekf_slam.x_hat, self.ekf_slam.P_hat, self.ekf_slam.Qe, self.Ts, self.step)  # state propagation
                self.ekf_slam.x_hat, self.ekf_slam.P_hat = self.ekf_slam.EKF_update(self.ekf_slam.x_hat, z, self.ekf_slam.P_hat, self.ekf_slam.Re, self.step)  # measurement update
                

                x_l1,y_l1 = self.transformation((self.ekf_slam.x_hat[0][0],self.ekf_slam.x_hat[1][0],self.ekf_slam.x_hat[2][0]), (self.ekf_slam.x_hat[4][0],self.ekf_slam.x_hat[5][0]))   
                r, g, b = 0, 255 , 0
                a = 255  # 알파 값
                rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                clustered_points.append((x_l1, y_l1, 0.0, rgb))


                x_l2, y_l2   = self.transformation((self.ekf_slam.x_hat[0][0],self.ekf_slam.x_hat[1][0],self.ekf_slam.x_hat[2][0]), (self.ekf_slam.x_hat[6][0],self.ekf_slam.x_hat[7][0]))
                r, g, b = 0, 255 , 0
                a = 255  # 알파 값
                rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                clustered_points.append((x_l2, y_l2, 0.0, rgb))



                # x_,y_ = self.transformation((0,0,-np.pi),(self.ekf_slam.x_hat[6][0],self.ekf_slam.x_hat[7][0])) #

                pc2_msg = pc2.create_cloud(header, fields, clustered_points)
                self.point_cloud_pub_2.publish(pc2_msg)


                error_array_ = self.error_cal([x_l1,y_l1],[x_l2,y_l2])
            
                self.e1_hat = error_array_[0]
                self.e2_hat = error_array_[1]
                self.e3_hat = error_array_[2]               

                




            if(self.first_loop_detect_leg and self.e1 != 0 and self.e2 != 0):
                
                if (abs((self.x1_1+self.x2_1)/2) < 0.12 and 0.55 <((self.y1_1+self.y2_1)/2) < 0.92):
                    
                    # self.get_logger().info('legs are detected! step : 3')
                    self.get_logger().info(f'legs are detected! step : 1 / found count : {self.count_wait_between_0_1}')

                    self.count_wait_between_0_1 += 1
                    
                    # self.get_logger().info(f'{(self.x1_+self.x2_)/2}, {(self.y1_+self.y2_)/2}')
                    
                else:
                    self.count_wait_between_0_1 = 0
                    self.count_step1 = 0
                    self.get_logger().info('legs are not detected.. wait 3 seconds')
                
                if self.count_step1 > 30:
                    self.step_1_1 = True 

                    self.ekf_slam = None  # 기존거 할당 해제
                    self.first_loop_detect_leg = False
                    init_landmark =  [[self.x1_1, self.y1_1], [self.x2_1, self.y2_1]]
                    self.ekf_slam = EKF_SLAM(4, 2, self.Ts, 0 , init_landmark, self.pose_x, self.pose_y, self.yaw)   ## 초기화  self.ekf_slam = EKF_SLAM( 4, 2, self.Ts, 0.05 ,init_landmark) 
                    self.get_logger().info('Legs are detected')
                    

                    
                    
                    
                




            if self.first_loop_detect_leg is not True and self.step == 1:
                P = dare(self.A, self.B, self.Q, self.R)
                K = np.linalg.inv(self.B.T @ P @ self.B + self.R) @ (self.B.T @ P @ self.A)

                x_error = np.array([self.e1_hat,self.e2_hat])
                # x_error = np.array([self.e1,self.e2])
                u = -K @ x_error
                self.input[0] = self.w_z
                self.input[1] = self.v_x
                # print(u)

                command = Twist()
                command.linear.x = self.V1
                command.angular.z = u[0]  # 각속도 (rad/s)        
                self.lqr_publisher_.publish(command)

            elif self.step == 2:


                

                # if (self.read_point):
                    
                    # self.get_logger().info(f'{(self.x1 + self.x2)/2.0},{(self.y1 + self.y2)/2.0}')
                # print(self.count_wait_between_1_2)
                
                if (abs((self.x1_+self.x2_)/2) < 0.12 and 0.6 <((self.y1_+self.y2_)/2) < 0.95):
                    
                    # self.get_logger().info('legs are detected! step : 3')
                    self.get_logger().info(f'legs are detected! step : 3 / found count : {self.count_wait_between_1_2}')

                    self.count_wait_between_1_2 += 1
                    
                    # self.get_logger().info(f'{(self.x1_+self.x2_)/2}, {(self.y1_+self.y2_)/2}')
                    
                else:
                    self.count_wait_between_1_2 = 0
                    self.count_step2 = 0
                    self.get_logger().info('legs are not detected.. wait 3 seconds')
                
                if self.count_step2 > 30:
                    self.step = 3

                    self.ekf_slam = None  # 기존거 할당 해제
                    init_landmark =  [[self.x1_, self.y1_], [self.x2_, self.y2_]]
                    self.ekf_slam = EKF_SLAM(4, 2, self.Ts, 0 , init_landmark, self.pose_x, self.pose_y, self.yaw)   ## 초기화  self.ekf_slam = EKF_SLAM( 4, 2, self.Ts, 0.05 ,init_landmark) 
                    print('초기 3번!!!!',self.ekf_slam.x_hat[4][0], self.ekf_slam.x_hat[5][0])
                    print('4번!!!!',self.ekf_slam.x_hat[6][0], self.ekf_slam.x_hat[7][0])



            elif self.step == 3:
        

                if self.e3_hat > self.distance_stop_center:
                    P = dare(self.A, self.B, self.Q, self.R)
                    K = np.linalg.inv(self.B.T @ P @ self.B + self.R) @ (self.B.T @ P @ self.A)

                    x_error = np.array([self.e1_hat,self.e2_hat])
                    u = -K @ x_error
                    self.input[0] = self.w_z
                    self.input[1] = self.v_x
                    # print(u)

                    command = Twist()
                    command.linear.x = self.V*(self.gain * (self.e3_hat - (self.distance_stop_center-0.1))/ (0.9-self.distance_stop_center))  # 입력은 설정속도 *gain * (목표지점 오차)/정규화
                    # print(command.linear.x, self.e3_hat - (self.distance_stop_center))
                    command.angular.z = u[0]  # 각속도 (rad/s)        
                    self.lqr_publisher_.publish(command)
                elif self.e3_hat < self.distance_stop_center:
                    command = Twist()
                    command.linear.x = 0.0  # 선형 속도 (m/s)
                    command.angular.z = 0.0  # 각속도 (rad/s)  
                    self.input[0] = 0.0
                    self.input[1] = 0.0  
                    self.lqr_publisher_.publish(command)

                    self.step = 4


            elif self.step == 4:
                if(self.count_stop == 50):


                    self.get_logger().info(f'도착-정지' )
                    msg = String()   
                    msg1 = 'UU'
                    msg.data = json.dumps(msg1)
                    self.stm_pub.publish(msg)
                
                    msg1 = 'red_blinking'
                    msg.data  = json.dumps(msg1)
                    self.stm_pub.publish(msg)
                    self.is_lifted.data = True
                    self.lift_pub.publish(self.is_lifted)
                    
                    self.input[0] = 0.0
                    self.input[1] = 0.0  

                if(self.count_stop == 150):
                    msg = String()
                    msg.data = 'lqr_done'
                    self.control_from_amr.publish(msg)
                    self.lifecycle = False
                    

                    
                    self.input[0] = 0.0
                    self.input[1] = 0.0  
                    self.shutdown_node()

                self.count_stop += 1
            
            

            # print(self.e3_hat, 'nonononono'if (self.e3_hat - self.distance_center) > 0 else 'yes~')


    def vel_sub_callback(self,msg):

        self.pose_x = msg.pose.pose.position.x  # slam 에서는 y가 전진 방향, odom은 x가 전진 방향
        self.pose_y = msg.pose.pose.position.y
        self.w_z = msg.twist.twist.angular.z
        self.v_x = msg.twist.twist.linear.x
        q = msg.pose.pose.orientation
        (roll, pitch, self.yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

        
    def lqr_command(self,msg):
        [self.e1,self.e2,self.e3,self.x1,self.x2,self.y1,self.y2] = [msg.e1,msg.e2,msg.e3,msg.x1,msg.x2,msg.y1,msg.y2]
        
        if (self.step == 1):
            self.count_step11 += 1            
            if(self.count_step11 > 10): 
                #초기 몇개는 무시!
                
                # self.get_logger().info(f'{(self.x1_+self.x2_)/2}, {(self.y1_+self.y2_)/2}')
                

                if (abs((self.x1+self.x2)/2) < 0.12 and 0.55 <((self.y1+self.y2)/2) < 0.92):
                    self.count_step1 = self.count_step1 + 1

                    if (self.count_step1 >= 2):
                        
                        self.x1_1 = (self.x1 + (self.count_step1-1) * self.x1_old1) / (self.count_step1)
                        self.y1_1 = (self.y1 + (self.count_step1-1) * self.y1_old1) / (self.count_step1)
                        self.x2_1 = (self.x2 + (self.count_step1-1) * self.x2_old1) / (self.count_step1)
                        self.y2_1 = (self.y2 + (self.count_step1-1) * self.y2_old1) / (self.count_step1)
                        
                        if(self.count_wait_between_0_1 < 30):
                            self.get_logger().info(f'다리 사이 추정 위치 :{(self.x1_1+self.x2_1)/2}, {(self.y1_1+self.y2_1)/2}')

                        # print('추정 (x,y) :',self.x1_, self.y1_, self.x2_, self.y2_)

                    self.x1_old1 = self.x1_1
                    self.y1_old1 = self.y1_1
                    self.x2_old1 = self.x2_1
                    self.y2_old1 = self.y2_1

                if (self.count_step1 == 1):
                    self.x1_old1 = self.x1
                    self.y1_old1 = self.y1
                    self.x2_old1 = self.x2
                    self.y2_old1 = self.y2
                    self.x1_1 = self.x1
                    self.y1_1 = self.y1
                    self.x2_1 = self.x2
                    self.y2_1 = self.y2                   
                    
        if (self.step == 2):
            self.count_step3 += 1            
            if(self.count_step3 > 10): 
                #초기 몇개는 무시!
                
                # print((self.x1+self.x2)/2, (self.y1+self.y2)/2)
                
                # print(self.count_step2)
                # self.get_logger().info(f'{(self.x1_+self.x2_)/2}, {(self.y1_+self.y2_)/2}')
                

                if (abs((self.x1+self.x2)/2) < 0.12 and 0.6 <((self.y1+self.y2)/2) < 0.95):
                    self.count_step2 = self.count_step2 + 1
                    # self.get_logger().info(f'count step :{self.count_step2}')
                    # self.get_logger().info(f'{(self.x1_+self.x2_)/2}, {(self.y1_+self.y2_)/2}')
                    # print('XXXXXXXXXXXXXX',self.count_step2)
                    if (self.count_step2 >= 2):
                        
                        self.x1_ = (self.x1 + (self.count_step2-1) * self.x1_old) / (self.count_step2)
                        self.y1_ = (self.y1 + (self.count_step2-1) * self.y1_old) / (self.count_step2)
                        self.x2_ = (self.x2 + (self.count_step2-1) * self.x2_old) / (self.count_step2)
                        self.y2_ = (self.y2 + (self.count_step2-1) * self.y2_old) / (self.count_step2)
                        self.get_logger().info(f'다리 사이 추정 위치 :{(self.x1_1+self.x2_1)/2}, {(self.y1_1+self.y2_1)/2}')

                        # print('추정 (x,y) :',self.x1_, self.y1_, self.x2_, self.y2_)

                    self.x1_old = self.x1_
                    self.y1_old = self.y1_
                    self.x2_old = self.x2_
                    self.y2_old = self.y2_

                if (self.count_step2 == 1):
                    self.x1_old = self.x1
                    self.y1_old = self.y1
                    self.x2_old = self.x2
                    self.y2_old = self.y2
                    self.x1_ = self.x1
                    self.y1_ = self.y1
                    self.x2_ = self.x2
                    self.y2_ = self.y2                   
                # print('aaaaaaaaaaaaaaa',(self.x1_+self.x2_)/2, (self.y1_+self.y2_)/2)
                    
                
                    
            
                
            # self.get_logger().info(f'{(self.x1_+self.x2_)/2}, {(self.y1_+self.y2_)/2}')

    def fail_point(self,msg):

        if msg.data == 'Fail':
            self.read_point = False
        elif msg.data == 'Success':
            self.read_point = True



def main():
    rclpy.init()

    node1 = Nodelet1()
    node2 = Nodelet2()

    # Future 객체 생성
    future = Future()

    # 노드에 Future 객체 전달
    node2.future = future

    executor = MultiThreadedExecutor()
    executor.add_node(node1)
    executor.add_node(node2)

    
    try:
        executor.spin_until_future_complete(future)
    finally:
        node1.destroy_node()
        node2.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
