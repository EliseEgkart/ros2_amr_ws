import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Header, String
from sensor_msgs.msg import LaserScan, PointCloud2
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from sklearn.linear_model import RANSACRegressor
import struct


class ChargeNode(Node):
    def __init__(self):
        super().__init__('charge_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )     
        self.lidar_scan = self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos_profile_sensor_data)
        self.point_cloud_pub_ = self.create_publisher(PointCloud2, 'clustered_points', qos_profile)
        self.line_info_pub_ = self.create_publisher(String, 'line_info', 10)

        self.min_points_per_line = 10  # Minimum number of points required to consider a line
        self.distance_threshold = 0.1  # Threshold for segmenting points based on distance

        self.jump_point_distance = 0.3
        self.adjant_point_distance_consider_noise = 0.03
        self.angle_increasment = 0.0019344649044796824


        self.dtheta = 0.01
        self.min_points_per_segment = 3  # 각 구간 내 최소 점 개수
        self.tolerance = 0.02  # 허용 오차

    def lidar_callback(self, msg):
        points = []
        min_angle = 45.0 * (np.pi / 180)  # 45 degrees
        max_angle = 135.0 * (np.pi / 180)  # 135 degrees


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
        all_line_points = []
        feature_point_index = []
        feature_points = []

        feature_points_min = []
        feature_points_max = []
        
        previous_local_min = None
        previous_local_max = None
        # 구간 나누기
        segments = np.arange(min_angle, max_angle, self.dtheta)
      
        for seg_start in segments:
            seg_end = seg_start + self.dtheta
            seg_points = points[(points[:, 3] >= seg_start) & (points[:, 3] < seg_end)]
            if len(seg_points) < self.min_points_per_segment:
                continue

            # 로컬 최소값과 최대값 찾기
            y_values = seg_points[:, 1]
            local_min_idx = np.argmin(y_values)
            local_max_idx = np.argmax(y_values)

            local_min = seg_points[local_min_idx]
            local_max = seg_points[local_max_idx]
            
            # x, y, z, _ = local_min
            # r, g, b = 0, 255, 0
            # a = 255  # 알파 값
            # rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
            # feature_points_min.append((x, y, z, rgb))
            # feature_point_index.append(local_min_idx)
            # x, y, z, _ = local_max
            # r, g, b = 255, 0, 0
            # a = 255  # 알파 값
            # rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
            # feature_points_max.append((x, y, z, rgb))
            # feature_point_index.append(local_max_idx)
     

            if previous_local_max is None:
                x, y, z, _ = local_max
                r, g, b = 255, 0, 0
                a = 255  # 알파 값
                rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                feature_points_max.append([x, y, z, rgb,0])

            if previous_local_min is None:
                x, y, z, _ = local_min
                r, g, b = 255, 0, 0
                a = 255  # 알파 값
                rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                feature_points_min.append([x, y, z, rgb,0])



            if previous_local_max is not None and abs(local_min[1] - previous_local_max[1]) < self.tolerance:
                feature_points_max[-1][4] = 1


            if previous_local_min is not None and abs(local_max[1] - previous_local_min[1]) < self.tolerance:
                feature_points_min[-1][4] = 1


            




            # if previous_local_max is not None and abs(local_min[1] - previous_local_max[1]) < self.tolerance:
            #     feature_points_min.pop()

            #     if abs(local_max[1] - local_min[1]) < self.jump_point_distance:
            #         feature_points_max.pop(-2) 
            #     a =1


            # if previous_local_min is not None and abs(local_max[1] - previous_local_min[1]) < self.tolerance:
            #     feature_points_max.pop()
            #     if abs(local_max[1] - local_min[1]) < self.jump_point_distance:
            #         feature_points_min.pop(-2) 
            #     a=1



            # # 로컬 최소값 추가 (이전 구간의 극대값과 비교)
            # if previous_local_max is None or abs(local_min[1] - previous_local_max[1]) > self.tolerance:
            #     x, y, z, _ = local_min
            #     r, g, b = 0, 255, 0
            #     a = 255  # 알파 값
            #     rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
            #     feature_points.append((x, y, z, rgb))
            #     feature_point_index.append(local_min_idx)

            # # 로컬 최대값 추가 (이전 구간의 극소값과 비교)
            # if previous_local_min is None or abs(local_max[1] - previous_local_min[1]) > self.tolerance:
            #     x, y, z, _ = local_max
            #     r, g, b = 255, 0, 0
            #     a = 255  # 알파 값
            #     rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
            #     feature_points.append((x, y, z, rgb))
            #     feature_point_index.append(local_max_idx)

            # if  previous_local_max is not None and abs(local_max[1] - previous_local_max[1]) > self.jump_point_distance:
            #     x, y, z, _ = previous_local_max
            #     r, g, b = 255, 255, 0
            #     a = 255  # 알파 값
            #     rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
            #     feature_points.append((x, y, z, rgb))
            #     feature_point_index.append(previous_local_max_idx)   
    

            # if  previous_local_min is not None and abs(local_min[1] - previous_local_min[1]) > self.jump_point_distance:
            #     x, y, z, _ = previous_local_min
            #     r, g, b = 255, 255, 255
            #     a = 255  # 알파 값
            #     rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
            #     feature_points.append((x, y, z, rgb))
            #     feature_point_index.append(previous_local_min_idx)   

            previous_local_max = local_max
            previous_local_min = local_min

            previous_local_max_idx = local_min_idx
            previous_local_min_idx = local_min_idx
        # points = np.array(points)
        # all_line_points = []
        # start_idx = 0

        # point_N = 3 #round(self.dtheta / self.angle_increasment)

        # point_y = points[0][1]  # start point
        # feature_point_index = []
        # feature_points = []
        # increase = 0
        # decrease = 0

        # x, y, z, _ = points[0]
        # r, g, b = 255,255,255
        # a = 255  # 알파 값
        # rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
        # feature_points.append((x, y, z, rgb))


        # print(points[:,1])

        # print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1")
        # for i in range(point_N, len(points), point_N):

        #     if (point_y < points[i,1] and abs(points[i,1] - point_y) > self.adjant_point_distance_consider_noise and increase == 0):
        #         feature_point_index.append(i)
        #         increase = 1
        #         decrease = 0


        #         x, y, z, _ = points[i]
            
        #         r, g, b = 0,255,0
        #         a = 255  # 알파 값
        #         rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
        #         feature_points.append((x, y, z, rgb))
              
        #         print(point_y , points[i,1], abs(points[i,1] - point_y) )
                

        #     elif (point_y > points[i,1] and abs(points[i,1] - point_y) > self.adjant_point_distance_consider_noise and decrease == 0):
        #         feature_point_index.append(i)
        #         increase = 0
        #         decrease = 1
        #         x, y, z, _ = points[i]
        #         r, g, b = 255,0,0
        #         a = 255  # 알파 값
        #         rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
        #         feature_points.append((x, y, z, rgb))

        #     elif abs(points[i,1] - point_y) > self.jump_point_distance:
        #         if(i>=1):
        #             feature_point_index.append(i-1)
        #             x, y, z, _ = points[i-1]
        #             r, g, b = 0,0,255
        #             a = 255  # 알파 값
        #             rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
        #             feature_points.append((x, y, z, rgb))
        #         feature_point_index.append(i)
        #         x, y, z, _ = points[i]
        #         r, g, b = 0,0,255
        #         a = 255  # 알파 값
        #         rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
        #         feature_points.append((x, y, z, rgb))

        #     point_y = points[i,1]


        # for i in range(len(feature_point_index)):

        #     x, y, z, _ = points[feature_point_index[i]]
        #     r, g, b = 0,0,0
        #     a = 255  # 알파 값
        #     rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
        #     feature_points.append((x, y, z, rgb))

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = msg.header.frame_id
        fields = [
            pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='rgb', offset=12, datatype=pc2.PointField.UINT32, count=1),
        ]


        # feature_points_min = np.array(feature_points_min)
        # feature_points_max = np.array(feature_points_max)

        feature_points = np.vstack([feature_points_min,feature_points_max])

        feature_points = feature_points[:,:4]
        feature_points = feature_points.tolist()


        pc2_msg = pc2.create_cloud(header, fields, feature_points)
        self.point_cloud_pub_.publish(pc2_msg)       

        # # Segment points based on distance threshold
        # for i in range(1, len(points)):
        #     if np.linalg.norm(points[i, :2] - points[i - 1, :2]) > self.distance_threshold:
        #         seg_points = points[start_idx:i]
        #         start_idx = i
        #         if len(seg_points) > self.min_points_per_line:
        #             self.process_segment(seg_points, all_line_points)
        
        # # Process last segment
        # if start_idx < len(points):
        #     seg_points = points[start_idx:]
        #     if len(seg_points) > self.min_points_per_line:
        #         self.process_segment(seg_points, all_line_points)

        # if not all_line_points:
        #     self.get_logger().warn('No lines detected in the scan data')
        #     return

        # # Publish line information and point clouds
        # for i, line_points in enumerate(all_line_points):
        #     start_point = line_points[0, :2]
        #     end_point = line_points[-1, :2]
        #     line_length = np.linalg.norm(end_point - start_point)
        #     line_angle = np.arctan2(end_point[1] - start_point[1], end_point[0] - start_point[0])

        #     line_info = f'Line {i+1} - Length: {line_length:.2f}, Angle: {np.degrees(line_angle):.2f} degrees'
        #     self.get_logger().info(line_info)

        #     # Publish line information
        #     self.line_info_pub_.publish(String(data=line_info))

        #     # Publish line points as PointCloud2
        #     header = Header()
        #     header.stamp = self.get_clock().now().to_msg()
        #     header.frame_id = msg.header.frame_id

        #     line_points_cloud = pc2.create_cloud_xyz32(header, line_points[:, :3].tolist())
        #     self.point_cloud_pub_.publish(line_points_cloud)


    def process_segment(self, seg_points, all_line_points):
        # RANSAC for line fitting in each segment
        remaining_points = seg_points[:, 0:2]
        ransac = RANSACRegressor(min_samples=self.min_points_per_line)
        ransac.fit(remaining_points, np.zeros(remaining_points.shape[0]))  # Fit the model

        inlier_mask = ransac.inlier_mask_
        line_points = seg_points[inlier_mask]

        if len(line_points) > self.min_points_per_line:
            all_line_points.append(line_points)

def main(args=None):
    rclpy.init(args=args)
    node = ChargeNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
