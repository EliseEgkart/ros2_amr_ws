import rclpy
from rclpy.node import Node
from std_msgs.msg import String , Float32MultiArray
import subprocess
import signal
import time
import json

class TaskScheduler(Node):
    def __init__(self):
        super().__init__('task_scheduler')
        
        self.create_subscription(String, '/command_from_data_manager', self.command_from_data_callback, 10)
        self.create_subscription(String, '/node_completion', self.node_completion_callback, 10)
        self.get_logger().info('task_scheduler node started')
        self.pub_for_data_manager = self.create_publisher(String, '/command_from_task_scheduler', 10)

        self.processes = set()
        self.command = None
        self.item_info = None   # 물류창고위치(cell,row,column)
        self.destination_info = None    # 출하장위치(cell,row,column)
        self.assigned_robot = None    # 배정된 로봇
        self.waypoints1 = None
        self.waypoints2 = None
        self.waypoints3 = None

        # Timer for publishing waypoints and info
        self.publish_timer = None

    def command_from_data_callback(self, msg):
        data = json.loads(msg.data)
        self.command = data.get("command")
        item_info = data.get("item_info", {})
        destination_info = data.get("destination_info", {})

        self.item_info = [item_info.get("cell"), item_info.get("row"), item_info.get("column")]
        self.destination_info = [destination_info.get("cell"), destination_info.get("row"), destination_info.get("column")]

        self.get_logger().info(f'Command: {self.command}')
        self.get_logger().info(f'Item Info: {self.item_info}')
        self.get_logger().info(f'Destination Info: {self.destination_info}')

        self.robot_assignment(self.item_info[0], self.item_info[1])
        self.set_waypoints(self.item_info[0], self.item_info[1], self.destination_info[0], self.destination_info[1])
        
        if self.command is not None and self.assigned_robot is not None and self.waypoints1 is not None and self.waypoints2 is not None and self.waypoints3 is not None and self.item_info is not None and self.destination_info is not None:
        # shipment 또는 receivement를 실행시키기 위한 모든 변수가 설정되면 실행
            self.start_nodes()


    def set_waypoints(self, cell, row, cell2, row2):   # cell,row -> 물류창고, cell2,row2 -> 입출하장
        waypoints_dict = {
            (1, 1): [3.912690195097347, 11.71825588121582, 0.0, 0.0, 0.0, -0.197731896147338, 0.9802561385913268],
            (1, 2): [1.4140762850110322, 11.666793736605127, 0.0, 0.0, 0.0, -0.10937657704721537, 0.9940003845036653],
            (2, 1): [4.320604998732083, -0.14324658305728066, 0.0, 0.0, 0.0, 0.2565932538316344, 0.9665194783800761],
            (2, 2): [1.7860168870106632, -0.4141938147996854, 0.0, 0.0, 0.0, 0.15350051557093916, 0.988148567635179],
            (3, 1): [1.4140762850110322, 11.666793736605127, 0.0, 0.0, 0.0, -0.10937657704721537, 0.9940003845036653],
            (3, 2): [-1.0921673351654437, 11.603673769447363, 0.0, 0.0, 0.0, -0.1420383976054733, 0.9898611486494808],
            (4, 1): [1.7860168870106632, -0.4141938147996854, 0.0, 0.0, 0.0, 0.15350051557093916, 0.988148567635179],
            (4, 2): [-0.6773224787475235, -0.4813707250550307, 0.0, 0.0, 0.0, 0.20521854184410376, 0.9787161744261611],
            (5, 1): [-1.0921673351654437, 11.603673769447363, 0.0, 0.0, 0.0, -0.1420383976054733, 0.9898611486494808],
            (5, 2): [-3.7021931388637173, 11.146616287042802, 0.0, 0.0, 0.0, -0.06575903362075076, 0.9978355322883952],
            (6, 1): [-0.6773224787475235, -0.4813707250550307, 0.0, 0.0, 0.0, 0.20521854184410376, 0.9787161744261611],
            (6, 2): [-3.1157422271423525, -0.5067201040327088, 0.0, 0.0, 0.0, 0.14894112858744396, 0.9888460649742702],
            (7, 1): [-4.062710604008178, 5.075429304773304, 0.0, 0.0, 0.0, 0.9936572727072835, 0.11245098663828255],
            (7, 2): [-6.341296234454732, 4.8047263865163545, 0.0, 0.0, 0.0, 0.9862166986832325, 0.16545882641414508],
            (8, 1): [-4.043739784085927, 5.056988691203313, 0.0, 0.0, 0.0, -0.9724548569389564, 0.2330912937366686],
            (8, 2): [-6.290015250823851, 4.836760131511661, 0.0, 0.0, 0.0, -0.9912231471276579, 0.13219936686059236],
            (9, 1): [11.769, -1.409, 0.0, 0.0, 0.0, -0.9946, 0.1028],
            (0, 2): [-8.315030904832255, 10.13183768752143, 0.0, 0.0, 0.0, 0.6595287034617422, 0.7516793793300927],
            (0, 1): [-7.763658314859935, -0.07959956833345853, 0.0, 0.0, 0.0, -0.5503567085872294, 0.8349296337494744],
            (0, 3): [11.769, -1.409, 0.0, 0.0, 0.0, -0.9946, 0.1028],
            (0, 4): [3.883242, 0.0704, 0.0, 0.0, 0.0, -0.6249009942792362, 0.7807040075142576],
        }

        if self.command == 'shipment' :
            waypoint_data1 = waypoints_dict.get((cell, row), None)
            if waypoint_data1:
                self.waypoint1= Float32MultiArray(data=waypoint_data1)
                self.get_logger().info(f'Waypoint1 set for item cell {cell}, row {row} : {self.waypoint1.data}')
            else:
                self.get_logger().warning(f'No waypoint1 found for cell {cell}, row {row}')
            
            waypoint_data2 = waypoints_dict.get((cell2, row2), None)
            if waypoint_data2:
                self.waypoint2= Float32MultiArray(data=waypoint_data2)
                self.get_logger().info(f'Waypoint2 set for destination cell {cell2}, row {row2}: {self.waypoint2.data}')
            else:
                self.get_logger().warning(f'No waypoint2 found for cell {cell2}, row {row2}')
            
            if self.assigned_robot == 1 or 3 or 5 or 7 :
                waypoint_data3 = waypoints_dict.get((0, 1), None)
                if waypoint_data3:
                    self.waypoint3= Float32MultiArray(data=waypoint_data3)
                    self.get_logger().info(f'Waypoint3 set for charge_station1: {self.waypoint3.data}')
                else:
                    self.get_logger().warning(f'No waypoint3 found for charge_station1')

            elif self.assigned_robot == 2 or 4 or 6 :
                waypoint_data3 = waypoints_dict.get((0, 2), None)
                if waypoint_data3:
                    self.waypoint3= Float32MultiArray(data=waypoint_data3)
                    self.get_logger().info(f'Waypoint3 set for charge_station2: {self.waypoint3.data}')
                else:
                    self.get_logger().warning(f'No waypoint3 found for charge_station2')

        elif self.command == 'receivement' :
            waypoint_data1 = waypoints_dict.get((cell2, row2), None)
            if waypoint_data1:
                self.waypoint1= Float32MultiArray(data=waypoint_data1)
                self.get_logger().info(f'Waypoint1 set for destination cell {cell2}, row {row2} : {self.waypoint1.data}')
            else:
                self.get_logger().warning(f'No waypoint1 found for cell {cell2}, row {row2}')
            
            waypoint_data2 = waypoints_dict.get((cell, row), None)
            if waypoint_data2:
                self.waypoint2= Float32MultiArray(data=waypoint_data2)
                self.get_logger().info(f'Waypoint2 set for destination cell {cell}, row {row}: {self.waypoint2.data}')
            else:
                self.get_logger().warning(f'No waypoint2 found for cell {cell}, row {row}')
            
            if self.assigned_robot == 1 or 3 or 5 or 7 :
                waypoint_data3 = waypoints_dict.get((0, 3), None)
                if waypoint_data3:
                    self.waypoint3= Float32MultiArray(data=waypoint_data3)
                    self.get_logger().info(f'Waypoint3 set for charge_station1: {self.waypoint3.data}')
                else:
                    self.get_logger().warning(f'No waypoint3 found for charge_station1')

            elif self.assigned_robot == 2 or 4 or 6 :
                waypoint_data3 = waypoints_dict.get((0, 4), None)
                if waypoint_data3:
                    self.waypoint3= Float32MultiArray(data=waypoint_data3)
                    self.get_logger().info(f'Waypoint3 set for charge_station2: {self.waypoint3.data}')
                else:
                    self.get_logger().warning(f'No waypoint3 found for charge_station2')


    def robot_assignment(self, cell ,row):
        if cell == 1 and row == 1:
            self.assigned_robot = 1
        
        elif (cell == 1 and row == 2) or (cell == 3 and row == 1):
            self.assigned_robot = 3

        elif cell == 2 and row == 1:
            self.assigned_robot = 2

        elif (cell == 2 and row == 2) or (cell == 4 and row == 1):
            self.assigned_robot = 4

        elif (cell == 3 and row == 2) or (cell == 5 and row == 1):
            self.assigned_robot = 5

        elif (cell == 4 and row == 2) or (cell == 6 and row == 1) or (cell == 6 and row == 2):
            self.assigned_robot = 6

        elif (cell == 5 and row == 2) :
            self.assigned_robot = 7

    def start_nodes(self):
        self.get_logger().info('start_node 실행')
        self.get_logger().info(f'self.processses 목록:  {self.processes}')
        
        if self.command in ['shipment', 'receivement']:
            for i in range(1, 8):  # assigned_robot 값을 1부터 7까지 반복
                if self.assigned_robot == i:
                    node_name = f'{self.command}{i}_node'
                    # if node_name not in self.processes or self.processes[node_name].poll() is not None:
                    if node_name not in self.processes:
                        self.processes.add(node_name)
                        subprocess.Popen([
                            'ros2', 'run', 'amr_control', node_name
                        ])
                        self.get_logger().info(f'{node_name} started')
                        
                        # 타이머를 설정하여 주기적으로 데이터를 퍼블리시
                        self.publish_timer = self.create_timer(1.0, self.publish_waypoints_and_info)

                        break  # 노드를 시작하면 반복문 종료

    def publish_waypoints_and_info(self):
        # 데이터를 ROS 2 메시지로 변환하여 발행
        waypoint1_msg = Float32MultiArray(data=self.waypoint1.data)
        waypoint2_msg = Float32MultiArray(data=self.waypoint2.data)
        waypoint3_msg = Float32MultiArray(data=self.waypoint3.data)
        item_info_msg = String(data=json.dumps(self.item_info))
        destination_info_msg = String(data=json.dumps(self.destination_info))

        node_name = f'{self.command}{self.assigned_robot}_node'

        waypoint1_pub = self.create_publisher(Float32MultiArray, f'/{node_name}/waypoint1', 10)
        waypoint2_pub = self.create_publisher(Float32MultiArray, f'/{node_name}/waypoint2', 10)
        waypoint3_pub = self.create_publisher(Float32MultiArray, f'/{node_name}/waypoint3', 10)
        item_info_pub = self.create_publisher(String, f'/{node_name}/item_info', 10)
        destination_info_pub = self.create_publisher(String, f'/{node_name}/destination_info', 10)

        self.get_logger().info(f'Publishing waypoints and info to {node_name}')
        waypoint1_pub.publish(waypoint1_msg)
        waypoint2_pub.publish(waypoint2_msg)
        waypoint3_pub.publish(waypoint3_msg)
        item_info_pub.publish(item_info_msg)
        destination_info_pub.publish(destination_info_msg)

    def node_completion_callback(self, msg):   #작업 배치가 완료되면(세부 파라미터 설정이 완료되면) 실행되는 콜백
        self.get_logger().info(f'Node completion received: {msg.data}')
        if msg.data == f'{self.command}{self.assigned_robot}_node_start':
            # 배치되어 실행시작한 작업에 대한 결과 데이터 매니저로 전송
            data_for_manager = {
                "AMR_number": self.assigned_robot,
                "command": self.command,
                "item_info": {
                    "cell": self.item_info[0],
                    "row": self.item_info[1],
                    "column": self.item_info[2]
                },
                "destination_info": {
                    "cell": self.destination_info[0],
                    "row": self.destination_info[1],
                    "column": self.destination_info[2]
                }
            }
            json_data = json.dumps(data_for_manager)
            for_data_manager = String()
            for_data_manager.data = json_data
            
            for i in range(5):
                self.pub_for_data_manager.publish(for_data_manager)
                # self.get_logger().info(f'data_for_manager {json_data}')

            # 퍼블리싱 타이머를 중지하고 변수들을 초기화
            if self.publish_timer is not None:
                self.publish_timer.cancel()
            self.command = None
            self.item_info = None
            self.destination_info = None
            self.assigned_robot = None
            self.waypoint1 = None
            self.waypoint2 = None
            self.waypoint3 = None
            self.get_logger().info('Publishing stopped and variables reset')
        
        else: #shipment_node_done 이런형식으로 들어옴.
        # 작업이 끝난 shipment 또는 receivement 노드를 서브프로세스에서 제거
            node_name = msg.data.replace("_done", "")
            if node_name in self.processes:
                self.processes.remove(node_name)  # 프로세스를 셋에서 제거
                self.get_logger().info(f'{node_name} has been removed from processes')


    def shutdown_nodes(self):
        if self.processes:
            for name, process in list(self.processes.items()):
                if process.poll() is None:  # 프로세스가 실행 중인지 확인
                    self.get_logger().info(f'Shutting down {name}')
                    process.send_signal(signal.SIGINT)  # SIGINT 신호 보내기
                    try:
                        process.wait(timeout=5)  # 최대 5초 대기
                    except subprocess.TimeoutExpired:
                        self.get_logger().warning(f'{name} did not terminate in time, killing it')
                        process.kill()  # 강제 종료
                        process.wait()
                    self.processes.pop(name)  # 프로세스를 딕셔너리에서 제거
            self.get_logger().info('All nodes shut down')
            time.sleep(2)  # 노드를 완전히 종료한 후 잠시 대기
        self.processes = {}
        print('11111' ,self.processes)

    

def main(args=None):
    rclpy.init(args=args)
    node = TaskScheduler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
