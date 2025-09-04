import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from amr_msgs.msg import Batterymsg

import serial
import json
import subprocess

import time
import psutil

# sudo gedit /etc/udev/rules.d/99-usb-serial.rules  --> 여기 설정가능  설정시 mode 0777 → 자동 권한 부여
# ex) ATTRS{idProduct}=="6001", ATTRS{idVendor}=="0403", ATTRS{manufacturer}=="FTDI", ATTRS{serial}=="B003IDMH", MODE="0777", SYMLINK+="ttyUSB_RS485"
# 이거는 모터 관련된 시리얼 포트


# 1. ls /dev/ttyUSB*   → 포트 연결됐는지 확인
# 2. lsusb → ATTRS{idProduct}  찾는법
# ex) ATTRS{idProduct}=="ea60", ATTRS{idVendor}=="10c4", ATTRS{manufacturer}=="Silicon Labs", ATTRS{serial}=="02713caffb0aee11b4ea4b02f59e3369", MODE="0777", SYMLINK+="ttyUSB_LIDAR2"
# 여기서 새로운 라이다 교체시 ATTRS{serial} 만 바꿔주면 됨
 
# 3. udevadm info -a -n /dev/ttyUSB1  → 포트 정보 확인    or   sudo dmesg | grep ttyUSB
# 4. sudo udevadm trigger
# 5. sudo udevadm control --reload-rules
# → udev 규칙 재적용

# 6. 새로운 포트에 연결시 권한 부여 ( 로그아웃 필요 )   
# sudo usermod -aG dialout $USER
# ==> 권한 처음 부여시 로그아웃 필요



class Battery_node(Node):

    def __init__(self):
        super().__init__('battery_node')
        self.get_logger().info('My Node is running')

        self.ser = self.init_serial()
        self.publisher_battery_info = self.create_publisher(String, '/battery_info', 10)
        self.stm_pub = self.create_publisher(String, '/stm_command' , 10)

        self.publisher_battery_charge_info = self.create_publisher(Bool, '/battery_charge_info', 10)
        

        self.publisher_battery_info_integrate = self.create_publisher(Batterymsg, '/battery_info_integrate', 10)

        self.publisher_pc_temperature_info = self.create_publisher(String, '/pc_temperature', 10)
        

        self.dt = 1
        self.retry_interval = 5  # 재시도 간격을 5초로 설정
        self.last_retry_time = 0  # 마지막 재시도 시간을 기록
        self.timer_ = self.create_timer(self.dt, self.timer_bat_callback)

    def init_serial(self):
        try:
            return serial.Serial('/dev/ttyUSB_battery', 115200, timeout=0.3)
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to /dev/ttyUSB_battery: {e}")
            self.trigger_udev()
            return None

    def trigger_udev(self):
        try:
            result = subprocess.run(['sudo', 'udevadm', 'trigger'], check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            self.get_logger().info("sudo udevadm trigger done")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"An error occurred while executing udevadm trigger: {e.stderr.decode()}")

    def timer_bat_callback(self):
        if not self.ser or not self.ser.is_open:
            current_time = time.time()
            if current_time - self.last_retry_time > self.retry_interval:
                self.last_retry_time = current_time
                self.ser = self.init_serial()
            if not self.ser:
                return
        packet = [
            0xAA, 0x00,          # Start Sequence
            0X00,      # Device ID
            0x53,             # Command
            0x00,         # Data Length
        ]

        checksum = 0
        for byte in packet[2:]:
            checksum ^= byte
        packet.append(checksum)   # Checksum
        packet.extend([0x00, 0xAA])  # End Sequence

       
        try:
            self.ser.write(packet)
            readbytes = self.ser.read(size=28)
        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication error: {e}")
            self.ser.close()
            self.ser = None
            return
        
        # self.get_logger().info(f"Received message (hex): {' '.join(f'{byte:02X}' for byte in readbytes)}")

        if len(readbytes) < 20:
            self.get_logger().error("Received data is too short.")
            return

        # 배터리 잔량 계산
        battery_capacity_high = readbytes[6]
        battery_capacity_low  = readbytes[7]
        battery_capacity = (battery_capacity_high << 8) + battery_capacity_low
        battery_capacity_percentage = (battery_capacity / 100)

        # 배터리 소모 전류 계산
        battery_current_high = readbytes[13]
        battery_current_low  = readbytes[14]
        battery_current = (battery_current_high << 8) + battery_current_low

        # 배터리팩 충전 여부
        battery_charge_status = readbytes[12]  # Assuming this is the correct byte for charging status
        charging_status = True if battery_charge_status == 1 else False

        # 배터리팩 온도
        battery_temperature = readbytes[17]  # Assuming this is the correct byte for charging status


        if charging_status and battery_capacity_percentage > 98.:
            msg = String()   
            msg1 = 'green'
            msg.data = json.dumps(msg1)
            self.stm_pub.publish(msg)
            
            
        if charging_status and battery_capacity_percentage <= 98.:
            msg = String()   
            msg1 = 'red'
            msg.data = json.dumps(msg1)
            self.stm_pub.publish(msg)

        # 정보 로깅
        # self.get_logger().info(f"Battery Capacity: {battery_capacity_percentage:.2f}%")
        # self.get_logger().info(f"Battery Pack Charging Status: {charging_status}")
        # self.get_logger().info(f"Battery Current: {battery_current}")
        # self.get_logger().info(f"Battery Temperature: {battery_temperature}")

        battery_info = f"{battery_capacity_percentage:.2f}"
        battery_current_info = f"{battery_current*0.001:.3f}"
        battery_temperature_info = f"{battery_temperature}"

        # 퍼블리시
        battery_info_msg = String()
        battery_info_msg.data = battery_info
        self.publisher_battery_info.publish(battery_info_msg)

        battery_charge_info_msg = Bool()
        battery_charge_info_msg.data = charging_status
        self.publisher_battery_charge_info.publish(battery_charge_info_msg)

        battery_info_msg_integr = Batterymsg()
        battery_info_msg_integr.charge = battery_info
        battery_info_msg_integr.current = battery_current_info
        battery_info_msg_integr.temperature = battery_temperature_info

        self.publisher_battery_info_integrate.publish(battery_info_msg_integr)


        ## PC 온도
        temps = psutil.sensors_temperatures()
        cpu_temp = temps.get('coretemp', [])[0].current if 'coretemp' in temps else None

        if cpu_temp is not None:
            msg = String()
            msg.data = f"{cpu_temp}"

            self.publisher_pc_temperature_info.publish(msg)
            # self.get_logger().info(f'Published CPU Temperature: {cpu_temp} °C')
        else:
            self.get_logger().warning('CPU temperature not available')

def main(args=None):
    rclpy.init(args=args)
    node = Battery_node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()