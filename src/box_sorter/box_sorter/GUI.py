import rclpy
from rclpy.node import Node

import sys
import serial
import time
import threading
from PySide2.QtWidgets import QApplication, QMainWindow, QTextBrowser, QPushButton, QVBoxLayout, QWidget, QLineEdit

# 아두이노 시리얼 포트 설정
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

class ArduinoSerialNode(Node):
    def __init__(self):
        super().__init__('arduino_serial_node')

class GUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.arduino = None  # 시리얼 객체 초기화
        self.setupUi()
        self.connect_serial()
        self.current_status = None

    def setupUi(self):
        self.setWindowTitle("Control GUI")
        self.setGeometry(100, 100, 400, 300)

        self.centralwidget = QWidget(self)
        self.setCentralWidget(self.centralwidget)

        self.layout = QVBoxLayout(self.centralwidget)

        self.textBrowser = QTextBrowser(self.centralwidget)
        self.layout.addWidget(self.textBrowser)

        self.distance_input = QLineEdit(self.centralwidget)
        self.distance_input.setPlaceholderText("이동할 거리 입력")
        self.layout.addWidget(self.distance_input)

        self.start_button = QPushButton("시작", self.centralwidget)
        self.start_button.clicked.connect(self.start_action)
        self.layout.addWidget(self.start_button)

        self.stop_button = QPushButton("정지", self.centralwidget)
        self.stop_button.clicked.connect(self.stop_action)
        self.layout.addWidget(self.stop_button)

    def connect_serial(self):
        """ 아두이노 시리얼 연결 """
        try:
            self.arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            time.sleep(2)  # 연결 안정화 대기
            self.textBrowser.append("Arduino 연결 성공")

            # 상태 읽기 쓰레드 실행
            status_thread = threading.Thread(target=self.read_status, daemon=True)
            status_thread.start()
        except serial.SerialException:
            self.textBrowser.append("시리얼 포트를 찾을 수 없습니다. 포트 설정을 확인하세요.")

    def read_status(self):
        """아두이노에서 지속적으로 상태를 읽어 GUI에 출력"""
        while self.arduino:
            try:
                response = self.arduino.read(1).decode().strip()
                if response == '.' and self.current_status != "READY":
                    self.textBrowser.append("'conveyor_status': READY")
                    self.current_status = "READY"
                elif response == '_' and self.current_status != "RUN":
                    self.textBrowser.append("'conveyor_status': RUN")
                    self.current_status = "RUN"
            except serial.SerialException:
                self.textBrowser.append("⚠ 시리얼 통신 오류")
                break
            except UnicodeDecodeError:
                continue  # 데이터 오류 시 무시하고 계속 읽기

    def start_action(self):
        """ 시작 버튼 클릭 시 아두이노에 이동 명령 전송 """
        distance_text = self.distance_input.text()
        try:
            distance_mm = float(distance_text)
            if self.arduino:
                self.textBrowser.append(f"{distance_mm} 이동 요청")
                self.arduino.write(f"{distance_mm}\n".encode())
            else:
                self.textBrowser.append("Arduino 연결 없음")
        except ValueError:
            self.textBrowser.append("유효한 숫자를 입력하세요.")

    def stop_action(self):
            """ 정지 버튼 클릭 시 동작 """
            distance_mm = 1
            if self.arduino:
                self.textBrowser.append("정지")
                self.arduino.write(f"{distance_mm}\n".encode())

def main(args=None):
    rclpy.init(args=args)
    arduino_serial_node = ArduinoSerialNode()

    app = QApplication(sys.argv)
    gui = GUI()
    gui.show()

    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        pass
    finally:
        arduino_serial_node.close_serial()
        arduino_serial_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()