import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Header, Bool

import numpy as np
import sympy as sym
from scipy.signal import cont2discrete
from scipy.linalg import solve_discrete_are as dare 
from scipy.integrate import odeint 

from sensor_msgs.msg import LaserScan, PointCloud2
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sklearn.cluster import DBSCAN
import sensor_msgs_py.point_cloud2 as pc2
import struct

from amr_msgs.msg import Errormsg
import serial  ###5/5 update (STM)

class Nodelet1(Node):
    def __init__(self):
        super().__init__('check_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )     
        
        self.lidar_scan = self.create_subscription(LaserScan,'/scan',self.lidar_callback,qos_profile_sensor_data)
        self.point_cloud_pub_ = self.create_publisher(PointCloud2, 'clustered_points', qos_profile)
        self.error_point_pub_ = self.create_publisher(Errormsg,'/error',10)


         # from control_center
        self.control_from_center = self.create_subscription(String, '/control_from_center', self.control_from_center_callback, 10)
        # to control_center
        self.control_from_amr = self.create_publisher(String, '/control_from_amr', 10)
        
        self.lifecycle = False

        self.lift_leg_distance = 0.47   # 전체 길이 50cm - 다리 두께 1.5 *2 (다리 두께 절반이 1.5cm)
        self.threshold_distance = 0.3   # 첫번째 군집시 군집 안의 최대 거리
        self.check_range = 1.5   # 1.5m 안에 책상 다리가 존재하도록 해야함

        self.e1 = None
        self.e1_old = None
        self.delta_e1 = None
        self.e1_threshold = 0.25
        self.cnt = 0
        self.cnt_outlier = 0
        self.initial_e1 = None
        self.initial_e1_set = []
        

        

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



    def lidar_callback(self, msg):
        if self.lifecycle == True:

            error_msg = Errormsg()

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

            clustering = DBSCAN(eps=0.3, min_samples=2).fit(points[:,:3])    ## 첫번째 클러스트링 : 군집을 크게 나눠서 붙어 있는 점들을 하나로 봄 --> 벽 같은 놈들은 하나로 취급됨
            labels1 = clustering.labels_

            clustered_points = []
            cluster_means = []
            unique_labels1 = set(labels1)

    ##################################################
            secondary_points = []


            colors = self.generate_colors(len(unique_labels1))

            for k1 in unique_labels1:
                if k1 == -1:
                    continue
                class_member_mask1 = (labels1 == k1)
                cluster1 = points[class_member_mask1]
                color = colors[k1]


                min_x, min_y = np.min(cluster1[:, :2], axis=0)
                max_x, max_y = np.max(cluster1[:, :2], axis=0)

                distance_between_point = np.sqrt((max_x-min_x)**2+(max_y-min_y)**2)

                if 1 <= len(cluster1) <= 100 and distance_between_point < self.threshold_distance:   ## 군집 갯수가 적절하고, 군집의 가장 먼 거리가 threshold 내에 존재 해야함 --> 우리가 들어야 하는 다리는 공간이 있기 때문에 주위에 없을 가능성이 큼.
                    secondary_points.append([cluster1])
                    for point in cluster1:
                        x, y, z, _ = point
                        # r, g, b = color
                        # a = 255  # 알파 값
                        # rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                        # clustered_points.append((x, y, z, rgb))

            flattened_points = [item for sublist in secondary_points for item in sublist]

            # 리스트의 배열을 수직으로 쌓아 하나의 배열로 만듭니다
            try:
                secondary_points = np.vstack(flattened_points)
            except ValueError as e:
                self.get_logger().error(f"No secondary_points")
                return
            # clustered_points = []

            # 두 번째 클러스터링
            clustering2 = DBSCAN(eps=0.02, min_samples=2).fit(secondary_points[:, :3])   ## 세분화 시켜서 다리를 분리, 다리같은 경우는 점들이 모여 있으므로 sample이 3개 이상은 있을 것이다.
            labels2 = clustering2.labels_

            unique_labels2 = set(labels2)

            colors = self.generate_colors(len(unique_labels2))

            for k2 in unique_labels2:
                if k2 == -1:
                    continue
                class_member_mask2 = (labels2 == k2)
                cluster2 = secondary_points[class_member_mask2]
                color = colors[k2]

                if 1 <= len(cluster2) <= 30:
                    mean_x = np.mean(cluster2[:, 0])
                    mean_y = np.mean(cluster2[:, 1])
                    mean_z = np.mean(cluster2[:, 2])
                    angle = np.mean(cluster2[:, 3])
                    cluster_means.append([mean_x, mean_y, mean_z,angle,len(cluster2)])
                    for point in cluster2:
                        x, y, z, _ = point
                        # r, g, b = color
                        # a = 255  # 알파 값
                        # rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                        # clustered_points.append((x, y, z, rgb))


    ##################################################


            # colors = self.generate_colors(len(unique_labels))
            
            # aspect_ratio = 0
            # for k in unique_labels:
            #     if k == -1:
            #         continue
            #     class_member_mask = (labels == k)
            #     cluster = points[class_member_mask]
            #     color = colors[k]

            #     # min_x, min_y = np.min(cluster[:, :2], axis=0)
            #     # max_x, max_y = np.max(cluster[:, :2], axis=0)
            #     # width = max_x - min_x
            #     # height = max_y - min_y
            #     # aspect_ratio = max(width / height, height / width)

            #     # 비율을 기준으로 필터링 (예: 길이 너비 비율이 2 이하인 군집만 선택)
            #     # if aspect_ratio <= 2:
            #     #     mean_x = np.mean(cluster[:, 0])
            #     #     mean_y = np.mean(cluster[:, 1])
            #     #     mean_z = np.mean(cluster[:, 2])
            #     #     angle = np.mean(cluster[:, 3])
            #     #     cluster_means.append([mean_x, mean_y, mean_z,angle])
            #     #     for point in cluster:
            #     #         x, y, z , _ = point
            #     #         r, g, b = 255, 255, 0  # 노란색
            #     #         a = 255  # 알파 값
            #     #         rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
            #     #         clustered_points.append((x, y, z, rgb))                

            #     if 3 <= len(cluster) <= 30:  
            #         mean_x = np.mean(cluster[:, 0])
            #         mean_y = np.mean(cluster[:, 1])
            #         mean_z = np.mean(cluster[:, 2])
            #         angle = np.mean(cluster[:, 3])
            #         cluster_means.append([mean_x, mean_y, mean_z,angle,len(cluster)])
            #         for point in cluster:
            #             x, y, z, _ = point
            #             r, g, b = color
            #             a = 255  # 알파 값
            #             rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
            #             clustered_points.append((x, y, z, rgb))



            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = msg.header.frame_id

            fields = [
                pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='rgb', offset=12, datatype=pc2.PointField.UINT32, count=1),
            ]




            for index, mean in enumerate(cluster_means[:3]):
                distance = np.linalg.norm(mean)  # 로봇이 원점(0, 0, 0)에 있다고 가정

            min_two_leg_accurate = 10
            min_between_leg_and_robot = 100
            two_index = [0.0, 0.0, 0.0, 0.0]

            point_index1 = 0
            point_index2 = 0

            point_index_list = []

            for i in range(len(cluster_means)):
                for j in range(i + 1, len(cluster_means)):
                    mean1 = cluster_means[i]
                    mean2 = cluster_means[j]
                    distance1 = np.linalg.norm(mean1[:3])
                    distance2 = np.linalg.norm(mean2[:3])
                    angle1 = mean1[3]
                    angle2 = mean2[3]

                    self.amr_length = 0.45
                    self.roi_x = 0.541 + 0.3
                    self.roi_y = 0.665

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

                            distance_between_leg = self.cos_2_distance(distance1,distance2,np.fabs(angle1-angle2))
                            error = (distance_between_leg-self.lift_leg_distance)**2

                            if error < 1e-3:
                                #self.get_logger().info(f'{error}')
                                # two_index = [i, j, angle1,angle2, distance1, distance2]
                                # point_index1 = i
                                # point_index2 = j
                                e1_, e2_, e3_ = self.error_cal(cluster_means[i], cluster_means[j])
                                if self.cnt < 10:
                                    self.cnt += 1
                                    self.initial_e1_set.append(e1_)
                                else:
                                    self.initial_e1 = sum(self.initial_e1_set) / len(self.initial_e1_set)
                                    if self.cnt == 10:
                                        self.e1_old = self.initial_e1
                                        self.cnt += 1
                                    self.e1 = e1_
                                    self.delta_e1 = self.e1 - self.e1_old
                                    self.e1_old = self.e1

                                    if abs(self.delta_e1) > self.e1_threshold:
                                        self.cnt_outlier += 1

                                    if self.cnt_outlier % 2 == 0:
                                        point_index_list.append((i,j))
                                        # point_index_list.append((point_index1,point_index2))

            point_indices_1_result = []
            point_indices_2_result = []
            if cluster_means and point_index_list:
                for point_indices in point_index_list:
                    
                    x1, y1, z1 , _ , _= cluster_means[point_indices[0]]
                    x2, y2, z2 , _ , _= cluster_means[point_indices[1]]
                    
                    distance_between_leg_and_robot = np.sqrt((x1+x2)**2+(y1+y2)**2)/2.0   # 얻은 포인트와 로봇 사이 거리
                    if distance_between_leg_and_robot < self.check_range and distance_between_leg_and_robot < min_between_leg_and_robot:
                        min_between_leg_and_robot = distance_between_leg_and_robot
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
                self.error_point_pub_.publish(error_msg)

                
                self.get_logger().info(f'e1:{error_array[0]} , e2 : {error_array[1]}, e3 : {error_array[2]}')

            except:
                # cluster_means가 비어 있을 때는 아무것도 하지 않음
                self.get_logger().info("cluster_means is empty, no action taken.")

            pc2_msg = pc2.create_cloud(header, fields, clustered_points)
            self.point_cloud_pub_.publish(pc2_msg) 

             

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

class Nodelet2(Node):
    def __init__(self):
        super().__init__('parking_lqr')

        self.lqr_publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lift_pub = self.create_publisher(Bool, '/lift' ,10)
        self.error_sub_ = self.create_subscription(Errormsg,'/error',self.lqr_command,10)
        # to control_center
        self.control_from_amr = self.create_publisher(String, '/control_from_amr', 10)


        self.Ts = 0.03 # Sampling time
        self.V = 0.05

        self.timer = self.create_timer(self.Ts, self.timer_callback)

        self.A = np.array([[1, self.V*self.Ts],  
                    [0, 1]]) 
        self.B = np.array([[0],  
                    [self.Ts]]) 
        
        self.R = np.array([1]) # Penalty for angular velocity effort 클수록 input이 작아짐

        self.Q = np.array([[2.0, 0], # Penalize X position error 클수록 error가 빨리 작아짐
                    [0, 0.1]])  # Penalize YAW ANGLE heading error 

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.e1 = 0
        self.e2 = 0
        self.e3 = 0
        self.e3_old = 0
        self.e3_threshold = 0.40
        self.step = 0
        self.distance_center = 0.37
        self.tmp = 0
        self.count_stop = 0
        self.e3_count = 0

        self.is_lifted = Bool()
        self.is_lifted.data = False

        


         ############## 5/5 UPDATE (STM) ##############
        self.serial_port = serial.Serial(
            port='/dev/ttyACM1',  # STM32가 연결된 포트
            baudrate=115200,      # 통신 속도
            timeout=1            # 타임아웃 설정
        )
        # self.timer_stm = self.create_timer(self.dt, self.check_serial)  
        # ##############################################

    def send_data(self, data): # for lift
        self.get_logger().info(f'send data check: {data}' )
        self.serial_port.write(data.encode())  # 데이터 인코딩 후 전송
    

    def timer_callback(self):

        if self.e3 != 0.0 :
            self.e3_count += 1
        
        if self.e3_count == 11 :
            self.step = 1

        if self.tmp == 0 and self.e3 != 0:
            self.tmp = 1
            self.e3_old = self.e3
        delta_e3 = self.e3 - self.e3_old
        self.e3_old = self.e3
        self.get_logger().info(f'delta: {delta_e3}')
        if delta_e3 > self.e3_threshold:
            self.step = 2
        elif delta_e3 < -1 * self.e3_threshold:
            self.step = 1
        self.get_logger().info(f'STEP: {self.step}')

        if self.step == 1:
            P = dare(self.A, self.B, self.Q, self.R)
            K = np.linalg.inv(self.B.T @ P @ self.B + self.R) @ (self.B.T @ P @ self.A)

            x_error = np.array([self.e1,self.e2])
            u = -K @ x_error
            print(u)

            command = Twist()
            command.linear.x = 0.05  # 선형 속도 (m/s)
            command.angular.z = u[0]  # 각속도 (rad/s)        
            self.lqr_publisher_.publish(command)

        elif self.step == 2:
            if self.e3 > self.distance_center:
                P = dare(self.A, self.B, self.Q, self.R)
                K = np.linalg.inv(self.B.T @ P @ self.B + self.R) @ (self.B.T @ P @ self.A)

                x_error = np.array([self.e1,self.e2])
                u = -K @ x_error
                print(u)

                command = Twist()
                command.linear.x = 0.05  # 선형 속도 (m/s)
                command.angular.z = u[0]  # 각속도 (rad/s)        
                self.lqr_publisher_.publish(command)
            else:
                command = Twist()
                command.linear.x = 0.0  # 선형 속도 (m/s)
                command.angular.z = 0.0  # 각속도 (rad/s)        
                self.lqr_publisher_.publish(command)
                self.step = 3

        elif self.step == 3:
            if(self.count_stop == 50):

                self.get_logger().info(f'도착-정지' )
                self.send_data("UU")
                self.is_lifted.data = True
                self.lift_pub.publish(self.is_lifted)
                
            if(self.count_stop == 150):
                msg = String()
                msg.data = 'lqr_done'
                self.control_from_amr.publish(msg)
                self.lifecycle = False
                
                    
            self.count_stop +=1


    def lqr_command(self,msg):
        [self.e1,self.e2,self.e3] = [msg.e1,msg.e2,msg.e3]


    # def getB(yaw, dt):

    #     B = np.array([[np.cos(yaw)*dt, 0],
    #                     [np.sin(yaw)*dt, 0],
    #                     [0, dt]])
    #     return B
    

    # def f(self,X, u):
    #     theta = X[2]
    #     v, w = u
    #     dx = sym.cos(theta)*v
    #     dy = sym.sin(theta)*v
    #     dtheta = w
    #     return dx, dy, dtheta
    
    # def calc_A(self,theta_d, v_d):
    #     A = np.array([[0, 0, -np.sin(theta_d)*v_d],
    #                     [0, 0, np.cos(theta_d)*v_d],
    #                     [0, 0, 0]])
    #     return A

    # # Input matrix
    # def calc_B(self,theta_d):
    #     B = np.array([[np.cos(theta_d), 0],
    #                     [np.sin(theta_d), 0],
    #                     [0, 1]])
    #     return B

    # def discrete(theta_d, v_d, T):
    #     A = self.calc_A(theta_d, v_d)
    #     Ad = T*A + np.eye(3)

    #     B = self.calc_B(theta_d)
    #     Bd = B*T

    #     return Ad, Bd


    # def dlqr(self,A, B, Q, R):
    #     '''Discrete time LTI LQR'''
    #     # Solve discrete Ricatti equation (DARE)
    #     P = dare(A, B, Q, R)
    #     # Compute the LQR gain
    #     K = np.linalg.inv(B.T @ P @ B + R) @ (B.T @ P @ A)
    #     return K, P
def main():
    rclpy.init()

    node1 = Nodelet1()
    node2 = Nodelet2()

    executor = MultiThreadedExecutor()
    executor.add_node(node1)
    executor.add_node(node2)

    
    try:
        executor.spin()
    finally:
        node1.destroy_node()
        node2.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
