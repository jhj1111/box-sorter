import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time
import json
import threading

class ConveyorController(Node):
    def __init__(self):
        super().__init__('conveyor_controller_node')

        # 아두이노 연결 설정
        self.arduino_port = "/dev/ttyACM0"  # 리눅스: "/dev/ttyUSB0", 윈도우: "COM3"
        self.baud_rate = 115200  # 시리얼 통신 속도
        self.ser = None  # 시리얼 객체 초기화
        self.current_status = None  # 현재 상태 저장
        self.running = True  # 쓰레드 실행 상태

        # ROS 2 퍼블리셔 & 서브스크라이버 설정
        self.status_publisher = self.create_publisher(String, '/conveyor/status', 10)
        self.command_subscriber = self.create_subscription(String, '/conveyor/control', self.send_command, 10)

        # 아두이노 연결 시도
        self.connect_arduino()

        # 상태 읽기 쓰레드 시작
        if self.ser:
            self.status_thread = threading.Thread(target=self.read_status, daemon=True)
            self.status_thread.start()
        else:
            self.running = False

    def connect_arduino(self):
        """아두이노와의 연결을 시도"""
        try:
            self.ser = serial.Serial(self.arduino_port, self.baud_rate, timeout=1)
            time.sleep(2)  # 아두이노 초기화 시간
            self.get_logger().info("아두이노와 연결되었습니다.")
        except serial.SerialException:
            self.get_logger().error(f"오류: {self.arduino_port} 포트에 연결할 수 없습니다.")
            self.ser = None
        except FileNotFoundError:
            self.get_logger().error(f"오류: {self.arduino_port} 포트를 찾을 수 없습니다.")
            self.ser = None
        except OSError as e:
            self.get_logger().error(f"시스템 오류 발생: {e}")
            self.ser = None
        except serial.SerialTimeoutException:
            self.get_logger().error("오류: 아두이노 응답 없음.")
            self.ser = None

    def read_status(self):
        """아두이노 상태를 읽어 '/conveyor/status' 토픽으로 퍼블리시"""
        while self.running:
            if self.ser and self.ser.in_waiting > 0:
                received_data = self.ser.read(1).decode().strip()
                if received_data:
                    status_msg = String()
                    if received_data == '.' and self.current_status != "READY":
                        self.current_status = "READY"
                        status_msg.data = "READY"
                    elif received_data == '_' and self.current_status != "RUN":
                        self.current_status = "RUN"
                        status_msg.data = "RUN"

                    if status_msg.data:
                        self.status_publisher.publish(status_msg)
                        self.get_logger().info(f"Publishing '/conveyor/status': {status_msg.data}")

            #time.sleep(0.1)  # CPU 점유율 방지

    def send_command(self, msg):
        """'/conveyor/command' 토픽을 구독하고, 받은 JSON 데이터를 아두이노에 전송"""
        try:
            command_data = json.loads(msg.data)  # JSON 문자열을 파싱
            command_data['control'] = command_data.get('control', 'stop')
            command_data['distance.mm'] = float(command_data.get('distance.mm', 1.0))
            json_data = json.dumps(command_data)  # 다시 JSON 문자열로 변환 (안전성 확보)
            if self.ser:
                self.ser.write(f"{json_data}\n".encode())
                self.get_logger().info(f"Sent to Arduino: {json_data}")
            else:
                self.get_logger().warn("아두이노가 연결되지 않아 데이터를 전송할 수 없습니다.")
        except json.JSONDecodeError:
            self.get_logger().error("JSON 파싱 오류: 잘못된 형식의 데이터를 수신했습니다.")

    def disconnect_arduino(self):
        """아두이노 연결 종료"""
        self.running = False
        if self.ser:
            if hasattr(self, "status_thread") and self.status_thread.is_alive():
                self.status_thread.join()
            self.ser.close()
            self.get_logger().info("아두이노 연결이 종료되었습니다.")

    def __del__(self):
        """객체가 삭제될 때 실행 (ROS 종료 시)"""
        self.disconnect_arduino()

def main(args=None):
    rclpy.init(args=args)
    node = ConveyorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("노드 종료 중...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
