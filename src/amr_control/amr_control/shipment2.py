import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import sys
from rclpy.executors import MultiThreadedExecutor
from concurrent.futures import Future
import json

class ShipmentNode2(Node):
    def __init__(self):
        super().__init__('shipment2_node')

        
        self.robot_number = 2  # 변수로 설정 -> 노드마다 수정필요!!

        self.publisher_ = self.create_publisher(String, '/control_from_center', 10)
        self.publisher_waypoint = self.create_publisher(Float32MultiArray, '/destination', 10)
        self.timer = self.create_timer(1, self.timer_callback)
        self.get_logger().info('AMR Control Node has been started.')

        self.subscription_from_amr = self.create_subscription(
            String,
            '/control_from_amr',
            self.listener_callback,
            10
        )



        self.completion_pub = self.create_publisher(String, '/node_completion', 10)   #작업에 변수가 다 설정되고, 실행될때 data_manager, Task_scheduler에 보고용
                                                                                      #작업이 모두 끝나면 task_schedular에 보고용.

        self.waypoint1_sub = self.create_subscription(Float32MultiArray, f'/shipment{self.robot_number}_node/waypoint1', self.waypoint1_callback, 10)
        self.waypoint2_sub = self.create_subscription(Float32MultiArray, f'/shipment{self.robot_number}_node/waypoint2', self.waypoint2_callback, 10)
        self.waypoint3_sub = self.create_subscription(Float32MultiArray, f'/shipment{self.robot_number}_node/waypoint3', self.waypoint3_callback, 10)
        self.item_info_sub = self.create_subscription(String, f'/shipment{self.robot_number}_node/item_info', self.item_info_callback, 10)
        self.destination_info_sub = self.create_subscription(String, f'/shipment{self.robot_number}_node/destination_info', self.destination_info_callback, 10)

        # 각 라인 노드들에 보내주는 정보 토픽
        self.ch1_pub = self.create_publisher(String, f'amr{self.robot_number}/ch1_set_param_from_ship', 10)
        self.ch2_pub = self.create_publisher(String, f'amr{self.robot_number}/ch2_set_param_from_ship', 10)
        self.lu1_pub = self.create_publisher(String, f'amr{self.robot_number}/lu1_set_param_from_ship', 10)
        self.lu2_pub = self.create_publisher(String, f'amr{self.robot_number}/lu2_set_param_from_ship', 10)
        self.ld1_pub = self.create_publisher(String, f'amr{self.robot_number}/ld1_set_param_from_ship', 10)

        # 각 라인 노드들 세팅 완료 정보 구독 토픽
        self.ch1_sub = self.create_subscription(String, f'amr{self.robot_number}/ch1_set_param_from_line', self.ch1_callback,10)
        self.ch2_sub = self.create_subscription(String, f'amr{self.robot_number}/ch2_set_param_from_line', self.ch2_callback,10)
        self.lu1_sub = self.create_subscription(String, f'amr{self.robot_number}/lu1_set_param_from_line', self.lu1_callback,10)
        self.lu2_sub = self.create_subscription(String, f'amr{self.robot_number}/lu2_set_param_from_line', self.lu2_callback,10)
        self.ld1_sub = self.create_subscription(String, f'amr{self.robot_number}/ld1_set_param_from_line', self.ld1_callback,10)
        

        self.main_lifecycle = True

        # 각 라인 노드들 세팅 완료 플래그
        self.ch2_set_done = False
        self.ch1_set_done = False
        self.lu1_set_done = False
        self.lu2_set_done = False
        self.ld1_set_done = False
        
        self.waypoint1 = None
        self.waypoint2 = None
        self.waypoint3 = None
        self.item_info = None
        self.destination_info = None
        self.firstloop = True

        self.shipment_start = True
        self.ch2_start = False   #charge out
        self.d2_start = False   #destination
        self.lu1_start = False  #lift up in
        self.lqr_start = False  # lqr
        self.lu2_start = False  #lift up out
        self.d3_start = False   
        self.ld1_start = False  #lift down 
        self.d1_start = False  
        self.ch1_start = False  #charge in

        
        self.shipment_done = False
        self.ch2_done = False
        self.d2_done = False
        self.lu1_done = False
        self.lqr_done = False
        self.lu2_done = False
        self.d3_done = False
        self.ld1_done = False
        self.d1_done = False
        self.ch1_done = False
        
        self.received_msg = None

    def ch1_callback(self, msg):
        if msg.data == 'True':
            self.ch1_set_done = True
            self.get_logger().info(f'ch1_set_done')
        
    def ch2_callback(self, msg):
        if msg.data == 'True':
            self.ch2_set_done = True
            self.get_logger().info(f'ch2_set_done')
        
    def lu1_callback(self, msg):
        if msg.data == 'True':
            self.lu1_set_done = True
            self.get_logger().info(f'lu1_set_done')

    def lu2_callback(self, msg):
        if msg.data == 'True':
            self.lu2_set_done = True
            self.get_logger().info(f'lu2_set_done')

    def ld1_callback(self, msg):
        if msg.data == 'True':
            self.ld1_set_done = True
            self.get_logger().info(f'ld1_set_done')

    def waypoint1_callback(self, msg):
        self.waypoint1 = msg
        self.get_logger().info(f'Waypoint1 received: {self.waypoint1.data}')

    def waypoint2_callback(self, msg):
        self.waypoint2 = msg
        self.get_logger().info(f'Waypoint2 received: {self.waypoint2.data}')

    def waypoint3_callback(self, msg):
        self.waypoint3 = msg
        self.get_logger().info(f'Waypoint3 received: {self.waypoint3.data}')

    def item_info_callback(self, msg):
        self.item_info = json.loads(msg.data)
        self.get_logger().info(f'Item Info received: {self.item_info}')

    def destination_info_callback(self, msg):
        self.destination_info = json.loads(msg.data)
        self.get_logger().info(f'Destination Info received: {self.destination_info}')



    #라인 세팅 콜백
    def waypoint1_callback(self, msg):
        self.waypoint1 = msg
        self.get_logger().info(f'Waypoint1 received: {self.waypoint1.data}')
    
    def shutdown_node(self):
        self.get_logger().info(f'Shutting down shipment{self.robot_number}_node')
        self.main_lifecycle = False
        self.destroy_node()
        self.get_logger().info('Node shutdown complete')


    def timer_callback(self):
        if self.firstloop == True:
            if self.waypoint1 is not None and self.waypoint2 is not None and self.waypoint3 is not None and self.item_info is not None and self.destination_info is not None:
                
                completion_msg = String()
                completion_msg.data = f'shipment{self.robot_number}_node_start'
                self.completion_pub.publish(completion_msg)
                self.firstloop = False

        else : #값설정이 모두 끝난후에 shipment실행

            if not self.shipment_done and self.shipment_start:
                msg = String()
                msg.data = 'shipment_start'
                self.publisher_.publish(msg)
                self.get_logger().info('shipment_start published')


            if not self.ch2_done and self.ch2_start:

                if self.ch2_set_done == False: 
                    msg_ch2 = String()
                    if self.robot_number % 2 == 0 : #오른쪽 충전소 -> 나갈때 우회전.
                        msg_ch2.data = 'R'
                    else : #왼쪽 충전소-> 나갈때 좌회전
                        msg_ch2.data = 'L'
                    
                    self.ch2_pub.publish(msg_ch2)
                    self.get_logger().info('ch2_parameter published')


                elif self.ch2_set_done == True:
                    msg = String()
                    msg.data = 'ch2_start'
                    self.publisher_.publish(msg)
                    self.get_logger().info('ch2_start published')
                    if self.received_msg == 'ch2_start':
                        self.get_logger().info('ch2 is running')
                        self.ch2_start = False

            
            
            if not self.d2_done and self.d2_start:
                msg = Float32MultiArray()
                msg.data = self.waypoint1.data  # 전달받은 웨이포인트 찍어줌.
                # msg.data = [7.64, -0.46, 0.0, 0.0, 0.0, 0.06, 0.998,
                #         11.769, -1.409, 0.0, 0.0, 0.0, -0.9946, 0.1028]
                self.publisher_waypoint.publish(msg)
                self.get_logger().info('d2_start published')
                if self.received_msg == 'driving_start':
                    self.get_logger().info('d2 is running')
                    self.d2_start = False


            if not self.lu1_done and self.lu1_start:

                if not self.lu1_set_done: 
                    msg_lu1 = String()
                    direction = ""
                    if self.item_info[0] % 2 == 0:  # 오른쪽 셀
                        if self.item_info[1] == 1:
                            direction = 'L'
                        elif self.item_info[1] == 2: 
                            direction = 'R'
                        msg_lu1.data = f'{direction}{9-self.item_info[2]}'
                    else:  # 왼쪽셀
                        if self.item_info[1] == 1:
                            direction = 'R'
                        elif self.item_info[1] == 2: 
                            direction = 'L'
                        msg_lu1.data = f'{direction}{self.item_info[2]}'

                    self.lu1_pub.publish(msg_lu1)
                    self.get_logger().info(f'lu1 parameter published: {msg_lu1.data}')


                elif self.lu1_set_done:
                    msg = String()
                    msg.data = 'lu1_start'
                    self.publisher_.publish(msg)
                    self.get_logger().info('lu1_start published')
                    if self.received_msg == 'lu1_start':
                        self.get_logger().info('lu1 is running')
                        self.lu1_start = False



            if not self.lqr_done and self.lqr_start:
                msg = String()
                msg.data = 'lqr_start'
                self.publisher_.publish(msg)
                self.get_logger().info('lqr_start published')
                if self.received_msg == 'lqr_start':
                    self.get_logger().info('lqr is running')
                    self.lqr_start = False


            if not self.lu2_done and self.lu2_start:
                if not self.lu2_set_done: 
                    msg_lu2 = String()
                    direction = ""
                    # escape_dir = ""
                    
                    if self.item_info[0] % 2 == 0:  # 오른쪽 셀
                        if self.item_info[1] == 1:
                            direction = 'R'
                        elif self.item_info[1] == 2: 
                            direction = 'L'
                        msg_lu2.data = f'{direction}{self.item_info[2]}'
                    else:  # 왼쪽셀
                        if self.item_info[1] == 1:
                            direction = 'L'
                        elif self.item_info[1] == 2: 
                            direction = 'R'
                        msg_lu2.data = f'{direction}{10-self.item_info[2]}'
                        

                    self.lu2_pub.publish(msg_lu2)
                    self.get_logger().info(f'lu2 parameter published: {msg_lu2.data}')


                elif self.lu2_set_done:
                    msg = String()
                    msg.data = 'lu2_start'
                    self.publisher_.publish(msg)
                    self.get_logger().info('lu2_start published')
                    if self.received_msg == 'lu2_start':
                        self.get_logger().info('lu2 is running')
                        self.lu2_start = False

            if not self.d3_done and self.d3_start:
                msg = Float32MultiArray()
                # msg.data = [13.957017221667478,-3.118006990215112,0.0 ,0.0 ,0.0 ,0.0544632, 0.998515]
                msg.data = self.waypoint2.data
                self.publisher_waypoint.publish(msg)
                self.get_logger().info('d3_start published')
                if self.received_msg == 'driving_start':
                    self.get_logger().info('d3 is running')
                    self.d3_start = False

            if not self.ld1_done and self.ld1_start:
                if not self.ld1_set_done: 
                    msg_ld1 = String()
                    direction = ""
                    if self.destination_info[0] % 2 == 0:  # 오른쪽 셀
                        if self.destination_info[1] == 1:
                            direction = 'R'
                        elif self.destination_info[1] == 2: 
                            direction = 'L'
                        msg_ld1.data = f'{direction}{self.destination_info[2]}'
                    else:  # 왼쪽셀
                        if self.destination_info[1] == 1:
                            direction = 'L'
                        elif self.destination_info[1] == 2: 
                            direction = 'R'
                        msg_ld1.data = f'{direction}{7-self.destination_info[2]}'

                    self.ld1_pub.publish(msg_ld1)
                    self.get_logger().info(f'ld1 parameter published: {msg_ld1.data}')

                elif self.ld1_set_done:
                    msg = String()
                    msg.data = 'ld1_start'
                    self.publisher_.publish(msg)
                    self.get_logger().info('ld1_start published')
                    if self.received_msg == 'ld1_start':
                        self.get_logger().info('ld1 is running')
                        self.ld1_start = False

            if not self.d1_done and self.d1_start:
                msg = Float32MultiArray()
                # msg.data = [3.883242, 0.0704, 0.0 , 0.0 ,0.0 ,-0.6249009942792362, 0.7807040075142576]
                msg.data = self.waypoint3.data
                self.publisher_waypoint.publish(msg)
                self.get_logger().info('d1_start published')
                if self.received_msg == 'driving_start':
                    self.get_logger().info('d1 is running')
                    self.d1_start = False

            if not self.ch1_done and self.ch1_start:
                if self.ch1_set_done == False: 
                    msg_ch1 = String()
                    direction = ""
                    
                    if self.robot_number % 2 == 0 : #오른쪽 충전소 -> 나갈때 우회전.
                        direction = 'L'
                    else : #왼쪽 충전소-> 나갈때 좌회전
                        direction = 'R'
                    
                    if self.robot_number == 6 or 7:
                        target_charge = 1
                    elif self.robot_number == 4 or 5:
                        target_charge = 2
                    elif self.robot_number == 2 or 3:
                        target_charge = 3
                    elif self.robot_number == 1:
                        target_charge = 4
                    
                    
                    msg_ch1.data = f'{direction}{target_charge}'

                    self.ch1_pub.publish(msg_ch1)
                    self.get_logger().info('ch1_parameter published')


                elif self.ch1_set_done == True:
                    msg = String()
                    msg.data = 'ch1_start'
                    self.publisher_.publish(msg)
                    self.get_logger().info('ch1_start published')
                    if self.received_msg == 'ch1_start':
                        self.get_logger().info('ch1 is running')
                        self.ch1_start = False



    def listener_callback(self, msg):
        self.get_logger().info('Received : "%s"' % msg.data)
        self.received_msg = msg.data

        if msg.data == 'shipment_done':
            self.shipment_done = True
            self.ch2_start = True

        if msg.data == 'ch2_done':
            self.ch2_done = True
            self.d2_start = True

        if msg.data == 'driving_done' and self.ch2_done == True and self.lu1_start == False:
            self.d2_done = True
            self.lu1_start = True

        if msg.data == 'lu1_done':
            self.lu1_done = True
            self.lqr_start = True

        if msg.data == 'lqr_done':
            self.lqr_done = True
            self.lu2_start = True

        if msg.data == 'lu2_done':
            self.lu2_done = True
            self.d3_start = True            

        if msg.data == 'driving_done' and self.lu2_done == True and self.ld1_start == False:
            self.d3_done = True
            self.ld1_start = True

        if msg.data == 'ld1_done':
            self.ld1_done = True
            self.d1_start = True

        if msg.data == 'driving_done' and self.ld1_done == True and self.ch1_start == False:
            self.d1_done = True
            self.ch1_start = True

        if msg.data == 'ch1_done':
            self.ch1_done = True
            completion_msg = String()
            completion_msg.data = f'shipment{self.robot_number}_node_done'
            self.completion_pub.publish(completion_msg)
            self.shutdown_node()
        



def main(args=None):
    rclpy.init(args=args)
    node = ShipmentNode2()

    try:
        while rclpy.ok() and node.main_lifecycle:
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_node()
        node.get_logger().info('rclpy has been shut down')