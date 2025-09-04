import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

amr_number = os.getenv('AMR_NUMBER', '0')

class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_pose = PoseStamped()

    def send_goal(self, pose):
        self.goal_pose = pose
        self.client.wait_for_server()
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.goal_pose

        self._send_goal_future = self.client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        try:
            result = future.result()
            status = future.result().status

            if status == 4:  # STATUS_SUCCEEDED
                self.get_logger().info('Goal reached successfully.')
                self.next_action()
            else:
                self.get_logger().info(f'Goal failed with status: {status}')
                self.handle_failure(status)
        except Exception as e:
            self.get_logger().error(f'Exception in get_result_callback: {e}')
            self.handle_failure(e)
    
    def handle_failure(self, reason):
        self.get_logger().info(f'실패함 ㅠ: {reason}')

    def next_action(self):
        self.get_logger().info('Performing next action...')
        # 여기서 다음 동작을 수행합니다.
        # 예를 들어, 특정 토픽에 메시지를 퍼블리시하거나 다른 서비스를 호출할 수 있습니다.
        # 예시: 다음 동작으로 특정 토픽에 메시지 퍼블리시
        next_action_publisher = self.create_publisher(String, 'next_action_topic', 10)
        next_action_msg = String()
        next_action_msg.data = 'Next action performed'
        next_action_publisher.publish(next_action_msg)

def main(args=None):
    rclpy.init(args=args)
    navigation_client = NavigationClient()
    
    pose = PoseStamped()
    # 여기에 목표 지점 설정
    pose.header.frame_id = 'map'
    pose.header.stamp = navigation_client.get_clock().now().to_msg()
    pose.pose.position.x = 2.8832424844533313
    pose.pose.position.y = 0.0704704581537811
    pose.pose.position.z = 0.0

    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = -0.6249009942792362
    pose.pose.orientation.w = 0.7807040075142576

    navigation_client.send_goal(pose)
    rclpy.spin(navigation_client)
    
    navigation_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


class AMRControlNode(Node):
    def __init__(self):
        super().__init__('amr_control_node')
        self.publisher_ = self.create_publisher(String, f'/amr{amr_number}/control_from_center', 10)
        self.timer = self.create_timer(1, self.timer_callback)
        self.get_logger().info('AMR Control Node has been started.')

        self.subscription_from_amr = self.create_subscription(
            String,
            '/control_from_amr',
            self.listener_callback,
            10
        )
        self.lu1_start = True
        self.lqr_start = False
        self.lu2_start = False

        self.lu1_done = False
        self.lqr_done = False
        self.lu2_done = False

        self.received_msg = None
        

    def timer_callback(self):
        
        if not self.lu1_done and self.lu1_start:
            msg = String()
            msg.data = 'lu1_start'
            self.publisher_.publish(msg)
            if self.received_msg == 'lu1_start':
                self.get_logger().info('lu1 is running')
                self.lu1_start = False


        if not self.lqr_done and self.lqr_start:
            msg = String()
            msg.data = 'lqr_start'
            self.publisher_.publish(msg)

        if not self.lu2_done and self.lu2_start:
            msg = String()
            msg.data = 'lu2_start'
            self.publisher_.publish(msg)


    def listener_callback(self, msg):
        self.get_logger().info('Received : "%s"' % msg.data)
        self.received_msg = msg.data

        if msg.data == 'lu1_done':
            self.lu1_done = True
            self.lqr_start = True

        if msg.data == 'lqr_done':
            self.lqr_done = True
            self.lu2_start = True

        if msg.data == 'lu2_done':
            self.lu2_done = True
            # self.lqr_start = True            