import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import serial 
import json

class STM_node(Node):
    def __init__(self):
        super().__init__('stm_node')
        self.get_logger().info('STM_node is running')

        self.sub_stm_manger = self.create_subscription(String, '/stm_command', self.stm_callback, 100)

        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.serial_port = serial.Serial(
            port='/dev/ttyACM_STM',  # STM32가 연결된 포트
            baudrate=115200,      # 통신 속도
            timeout=1            # 타임아웃 설정
        )
        self.on_off = False
        self.blinking_active = False


    def stm_callback(self, msg):
        
        data = json.loads(msg.data)

        if(data == 'UU'):
            self.linear_up()

        elif(data == 'DD'):
            self.linear_down()


        elif(data == 'red'):
            self.blinking_active = False

            self.led_control(0x00,[0xFF,0x00,0x00],0x2D)



        elif(data == 'green'): ## 완충
            self.blinking_active = False
            self.led_control(0x00,[0x00,0xFF,0x00],0x2D)

        elif(data == 'blue'):  ##
            self.blinking_active = False
            self.led_control(0x00,[0x00,0xFF,0xFF],0x2D)


        elif(data == 'off'):  ##
            self.blinking_active = False
            self.led_control(0x00,[0xFF,0xFF,0xFF],0x01)



        elif(data == 'red_blinking'):
            self.blinking_active = True
            self.color = [0xFF,0x00,0x00]



    def timer_callback(self):
        if self.blinking_active:
            self.on_off = not self.on_off
            self.led_blinking(0x00, self.color, 0x2D, self.on_off)


    def led_blinking(self, id, rgb, brigtness, on_off):
        self.get_logger().info('blinking')
        if (on_off):
            tx_data = [0xaa, 0x02, id, rgb[0], rgb[1], rgb[2], brigtness, 0xbb]

        else:
            tx_data = [0xaa, 0x02, id, rgb[0], rgb[1], rgb[2], 0x00, 0xbb]

        self.send_data(tx_data)

    def send_data(self, data):
        self.serial_port.write(bytearray(data)) 
        # self.serial_port.write(data.encode())  # 데이터 인코딩 후 전송


    def linear_up(self):
        self.get_logger().info('linear_up')
        tx_data = [0xaa,0x01,0x00,0x01,0x00,0x00,0x00,0xbb]
        self.send_data(tx_data)


    def linear_down(self):
        self.get_logger().info('linear_down')
        tx_data = [0xaa,0x01,0x00,0x02,0x00,0x00,0x00,0xbb]
        self.send_data(tx_data)


    def led_control(self, id, rgb, brigtness):
        self.get_logger().info('light')
        tx_data = [0xaa, 0x02, id, rgb[0], rgb[1], rgb[2], brigtness, 0xbb]
        self.send_data(tx_data)






def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = STM_node()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()