import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import sys
from rclpy.executors import MultiThreadedExecutor
from concurrent.futures import Future

class AMRControlNode(Node):
    def __init__(self):
        super().__init__('amr_control_node')
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

        self.ch2_start = True   #charge out
        self.d2_start = False   #destination
        self.lu1_start = False  #lift up in
        self.lqr_start = False  # lqr
        self.lu2_start = False  #lift up out
        self.d3_start = False   
        self.ld1_start = False  #lift down 
        self.d1_start = False  
        self.ch1_start = False  #charge in

        
        
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
    
    def shutdown_node(self):
        self.get_logger().info('Shutting down shipment_node')
        self.future.set_result(True)  # Future 객체 완료
        rclpy.shutdown()
        sys.exit(0)

    def timer_callback(self):

        if not self.ch2_done and self.ch2_start:
            msg = String()
            msg.data = 'ch2_start'
            self.publisher_.publish(msg)
            self.get_logger().info('ch2_start published')
            if self.received_msg == 'ch2_start':
                self.get_logger().info('ch2 is running')
                self.ch2_start = False

        
        
        if not self.d2_done and self.d2_start:
            msg = Float32MultiArray()
            msg.data = [7.64, -0.46, 0.0, 0.0, 0.0, 0.06, 0.998,
                    11.769, -1.409, 0.0, 0.0, 0.0, -0.9946, 0.1028]
            self.publisher_waypoint.publish(msg)
            self.get_logger().info('d2_start published')
            if self.received_msg == 'driving_start':
                self.get_logger().info('d2 is running')
                self.d2_start = False


        if not self.lu1_done and self.lu1_start:
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
            msg = String()
            msg.data = 'lu2_start'
            self.publisher_.publish(msg)

            self.get_logger().info('lu2_start published')
            if self.received_msg == 'lu2_start':
                self.get_logger().info('lu2 is running')
                self.lu2_start = False

        if not self.d3_done and self.d3_start:
            msg = Float32MultiArray()
            msg.data = [13.957017221667478,-3.118006990215112,0.0 ,0.0 ,0.0 ,0.0544632, 0.998515]
            self.publisher_waypoint.publish(msg)
            self.get_logger().info('d3_start published')
            if self.received_msg == 'driving_start':
                self.get_logger().info('d3 is running')
                self.d3_start = False

        if not self.ld1_done and self.ld1_start:
            msg = String()
            msg.data = 'ld1_start'
            self.publisher_.publish(msg)
            self.get_logger().info('ld1_start published')
            if self.received_msg == 'ld1_start':
                self.get_logger().info('ld1 is running')
                self.ld1_start = False

        if not self.d1_done and self.d1_start:
            msg = Float32MultiArray()
            msg.data = [3.883242, 0.0704, 0.0 , 0.0 ,0.0 ,-0.6249009942792362, 0.7807040075142576]
            self.publisher_waypoint.publish(msg)
            self.get_logger().info('d1_start published')
            if self.received_msg == 'driving_start':
                self.get_logger().info('d1 is running')
                self.d1_start = False

        if not self.ch1_done and self.ch1_start:
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
            self.shutdown_node()
        



def main(args=None):
    rclpy.init(args=args)
    node = AMRControlNode()
    # Future 객체 생성
    future = Future()

    # 노드에 Future 객체 전달
    node.future = future
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    
    try:
        executor.spin_until_future_complete(future)
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()