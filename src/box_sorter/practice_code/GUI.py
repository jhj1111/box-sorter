import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

import os
import cv2
import sys
import time
import json
import serial
import imutils
import threading
import numpy as np
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QImage, QPixmap

# 아두이노 시리얼 포트 설정
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = "/usr/lib/qt/plugins" #적절한 경로로 수정 필요
os.environ["QT_QPA_PLATFORM"] = "xcb" #기본 플랫폼 설정(본인이 사용하려는 플랫폼으로 설정)

class ImageSubscriber(Node):
    def __init__(self, gui):
        super().__init__('image_subscriber')
        self.gui = gui
        self.subscription_rgb = self.create_subscription(
            CompressedImage,
            'yolo/compressed',
            self.image_callback,
            10)
        self.image_np = None

    def image_callback(self, msg):
        print('listener_callback_rgb...')
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # Decode to color image
        if self.image_np is not None:
            self.gui.image_show(self.image_np)

class GUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.arduino = None  # 시리얼 객체 초기화
        self.setupUi()
        self.connect_serial()
        self.current_status = None

    def setupUi(self):
        self.setWindowTitle("Control GUI")
        self.setGeometry(100, 100, 800, 600)

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

        self.control_layout = QHBoxLayout()

        self.parts_combo = QComboBox(self.centralwidget)
        self.parts_combo.addItems(["Red", "Blue"])
        self.control_layout.addWidget(self.parts_combo)

        self.count_combo = QComboBox(self.centralwidget)
        self.count_combo.addItems(["1", "2"])
        self.control_layout.addWidget(self.count_combo)

        self.goal_combo = QComboBox(self.centralwidget)
        self.goal_combo.addItems(["1", "2", "3"])
        self.control_layout.addWidget(self.goal_combo)

        self.send_button = QPushButton("Send", self.centralwidget)
        self.send_button.clicked.connect(self.send_job)
        self.control_layout.addWidget(self.send_button)

        # 전체 레이아웃에 추가
        self.layout.addLayout(self.control_layout)

        self.label_1 = QLabel(self.centralwidget)  # QLabel for displaying the image
        self.layout.addWidget(self.label_1)

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
            distance_mm = float(distance_text) * 10.24
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

    def send_job(self):
        selected_part = self.parts_combo.currentText()
        selected_count = self.count_combo.currentText()
        selected_goal = self.goal_combo.currentText()

        self.textBrowser.append(f"Selected Part: {selected_part}, Count: {selected_count}, Goal: {selected_goal}")

    def image_show(self, image_np):
        image = self.cvimage_to_label(image_np)
        self.label_1.setPixmap(QPixmap.fromImage(image))

    def cvimage_to_label(self, image):
        image = imutils.resize(image, width=640)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = QImage(image,
                       image.shape[1],
                       image.shape[0],
                       QImage.Format_RGB888)
        return image

def start_node(gui):
    #rclpy.init()

    node = ImageSubscriber(gui)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    
    gui = GUI()
    gui.show()
    #arduino_serial_node = ImageSubscriber(gui)  # Pass GUI instance to ArduinoSerialNode

    # ROS 2 노드를 별도의 스레드에서 실행
    ros2_thread = threading.Thread(target=start_node, args=(gui,))
    ros2_thread.start()

    # GUI 실행
    sys.exit(app.exec_())

    arduino_serial_node.destroy_node()
    rclpy.shutdown()
    ros2_thread.join()

if __name__ == "__main__":
    main()