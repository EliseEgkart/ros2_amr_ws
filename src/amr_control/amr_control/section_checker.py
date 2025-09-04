from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node

class SectionChecker(Node):

    def __init__(self):
        super().__init__('section_checker')
        
        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.listener_callback_amcl_pose,
            10
        )

    def section1(self, x, y):
        return 10.25 <= x < 12.35 and 5.5 <= y < 6.8

    def section2(self, x, y):
        return 10.25 <= x < 12.35 and -6.6 <= y < -5.2

    def section3(self, x, y):
        return 7.65 <= x < 10.25 and 5.5 <= y < 6.8

    def section4(self, x, y):
        return 7.65 <= x < 10.25 and -6.6 <= y < -5.2

    def section5(self, x, y):
        return 5.08 <= x < 7.65 and 5.5 <= y < 6.8

    def section6(self, x, y):
        return 5.08 <= x < 7.65 and -6.6 <= y < -5.2

    def section7(self, x, y):
        return 3.15 <= x < 5.08 and 5.5 <= y < 6.8

    def section8(self, x, y):
        return 3.15 <= x < 5.08 and -6.6 <= y < -5.2

    def section9(self, x, y):
        return 10.25 <= x < 12.35 and 0.5 <= y < 5.5

    def section10(self, x, y):
        return 10.25 <= x < 12.35 and -5.2 <= y < -0.76

    def section11(self, x, y):
        return 7.65 <= x < 10.25 and 0.5 <= y < 5.5

    def section12(self, x, y):
        return 7.65 <= x < 10.25 and -5.2 <= y < -0.76

    def section13(self, x, y):
        return 5.08 <= x < 7.65 and 0.5 <= y < 5.5

    def section14(self, x, y):
        return 5.08 <= x < 7.65 and -5.2 <= y < -0.76

    def section15(self, x, y):
        return 3.15 <= x < 5.08 and 0.5 <= y < 5.5

    def section16(self, x, y):
        return 3.15 <= x < 5.08 and -5.2 <= y < -0.76

    def section17(self, x, y):
        return 1.23 <= x < 3.15 and 0.5 <= y < 3.94

    def section18(self, x, y):
        return 1.23 <= x < 3.15 and -4.2 <= y < -0.76

    def section19(self, x, y):
        return -2.35 <= x < 1.23 and 0.5 <= y < 3.94

    def section20(self, x, y):
        return -2.35 <= x < 1.23 and -4.2 <= y < -0.76

    def section21(self, x, y):
        return 0.6 <= x < 3.15 and 3.94 <= y < 5.5

    def section22(self, x, y):
        return 0.6 <= x < 3.15 and -5.2 <= y < -4.2

    def section23(self, x, y):
        return -2.35 <= x < 0.6 and 3.94 <= y < 5.5

    def section24(self, x, y):
        return -2.35 <= x < 0.6 and -5.2 <= y < -4.2

    def section25(self, x, y):
        return -2.35 <= x < 3.15 and 5.1 <= y < 6.1

    def section26(self, x, y):
        return -2.35 <= x < 3.15 and -5.75 <= y < -5.1

    def section27(self, x, y):
        return -2.35 <= x < 12.35 and -0.76 <= y < 0.5

    def section29(self, x, y):
        return 1.93 <= x < 3.5 and 0.5 <= y < 5.5

    def section30(self, x, y):
        return 1.93 <= x < 3.5 and -5.2 <= y < -0.76

    def listener_callback_amcl_pose(self, msg):
        # 매번 콜백이 호출될 때마다 sections 리스트를 초기화
        sections = []
        
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y

        # Check which sections the robot is in
        if self.section1(x, y):
            sections.append('Section 1')
        if self.section2(x, y):
            sections.append('Section 2')
        if self.section3(x, y):
            sections.append('Section 3')
        if self.section4(x, y):
            sections.append('Section 4')
        if self.section5(x, y):
            sections.append('Section 5')
        if self.section6(x, y):
            sections.append('Section 6')
        if self.section7(x, y):
            sections.append('Section 7')
        if self.section8(x, y):
            sections.append('Section 8')
        if self.section9(x, y):
            sections.append('Section 9')
        if self.section10(x, y):
            sections.append('Section 10')
        if self.section11(x, y):
            sections.append('Section 11')
        if self.section12(x, y):
            sections.append('Section 12')
        if self.section13(x, y):
            sections.append('Section 13')
        if self.section14(x, y):
            sections.append('Section 14')
        if self.section15(x, y):
            sections.append('Section 15')
        if self.section16(x, y):
            sections.append('Section 16')
        if self.section17(x, y):
            sections.append('Section 17')
        if self.section18(x, y):
            sections.append('Section 18')
        if self.section19(x, y):
            sections.append('Section 19')
        if self.section20(x, y):
            sections.append('Section 20')
        if self.section21(x, y):
            sections.append('Section 21')
        if self.section22(x, y):
            sections.append('Section 22')
        if self.section23(x, y):
            sections.append('Section 23')
        if self.section24(x, y):
            sections.append('Section 24')
        if self.section25(x, y):
            sections.append('Section 25')
        if self.section26(x, y):
            sections.append('Section 26')
        if self.section27(x, y):
            sections.append('Section 27')
        if self.section29(x, y):
            sections.append('Section 29')
        if self.section30(x, y):
            sections.append('Section 30')

        if sections:
            self.get_logger().info(f'in {", ".join(sections)} ----------------- x={x}, y={y}')

        else:
            self.get_logger().info(f'The robot is not in any defined section')

def main(args=None):
    rclpy.init(args=args)
    section_checker = SectionChecker()
    rclpy.spin(section_checker)
    section_checker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
