import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor

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

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
    QLabel, QTextBrowser, QLineEdit, QRadioButton, QButtonGroup, QGroupBox
)

import lib as lib

# 아두이노 시리얼 포트 설정
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = "/usr/lib/qt/plugins" #적절한 경로로 수정 필요
os.environ["QT_QPA_PLATFORM"] = "xcb" #기본 플랫폼 설정(본인이 사용하려는 플랫폼으로 설정)

class JobPublisher(Node):
    def __init__(self):
        super().__init__('job_publisher')
        self.publisher_ = self.create_publisher(String, 'job_topic', 10)

    def send_json_job(self, red, blue, goal):
        data = {
            "red": red,
            "blue": blue,
            "goal": goal
        }
        job_data = json.dumps(data)
        msg = String()
        msg.data = job_data
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published Job: {job_data}")

class GUI(QMainWindow):
    def __init__(self, job_publisher):
        super().__init__()
        self.arduino = None  # 시리얼 객체 초기화
        self.setupUi()
        self.current_status = None
        self.image_np = None
        self.node = job_publisher

        self.subscription_rgb = self.node.create_subscription(
            CompressedImage,
            'yolo/compressed',
            self.image_callback,
            10)
        
        self.subscription_status = self.node.create_subscription(
            String,
            '/conveyor/status',
            self.read_status,
            10)
        
        self.conveyor_publisher_ = self.node.create_publisher(String, '/conveyor/control', 10)

    def image_callback(self, msg):
        print('listener_callback_rgb...')
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # Decode to color image
        if self.image_np is not None:
            self.image_show(self.image_np)

    def setupUi(self):
        self.setWindowTitle("Control GUI")
        self.setGeometry(100, 100, 800, 600)

        self.centralwidget = QWidget(self)
        self.setCentralWidget(self.centralwidget)
        self.layout = QVBoxLayout(self.centralwidget) 

        # 출력 창
        self.textBrowser = QTextBrowser(self.centralwidget)
        self.layout.addWidget(self.textBrowser)

        # 이동 거리 입력
        self.distance_input = QLineEdit(self.centralwidget)
        self.distance_input.setPlaceholderText("이동할 거리 입력")
        self.layout.addWidget(self.distance_input)

        # 시작 & 정지 버튼
        self.button_layout = QHBoxLayout()
        self.start_button = QPushButton("시작", self.centralwidget)
        self.start_button.clicked.connect(self.start_action)
        self.button_layout.addWidget(self.start_button)

        self.stop_button = QPushButton("정지", self.centralwidget)
        self.stop_button.clicked.connect(self.stop_action)
        self.button_layout.addWidget(self.stop_button)

        self.layout.addLayout(self.button_layout)

        # 컨트롤 그룹 (Red & Blue 개수, Goal 설정)
        self.control_group = QGroupBox("Control Settings")
        self.control_layout = QVBoxLayout(self.control_group)

        # Red & Blue 개수 선택
        self.count_group = QGroupBox("Red & Blue Count")
        self.count_layout = QHBoxLayout()

        self.red_label = QLabel("Red:")
        self.count_layout.addWidget(self.red_label)

        self.red_count_0 = QRadioButton("0", self)
        self.red_count_1 = QRadioButton("1", self)
        self.red_count_2 = QRadioButton("2", self)
        self.red_count_1.setChecked(True)

        self.red_count_group = QButtonGroup(self)
        self.red_count_group.addButton(self.red_count_0)
        self.red_count_group.addButton(self.red_count_1)
        self.red_count_group.addButton(self.red_count_2)

        self.count_layout.addWidget(self.red_count_0)
        self.count_layout.addWidget(self.red_count_1)
        self.count_layout.addWidget(self.red_count_2)

        self.blue_label = QLabel("Blue:")
        self.count_layout.addWidget(self.blue_label)

        self.blue_count_0 = QRadioButton("0", self)
        self.blue_count_1 = QRadioButton("1", self)
        self.blue_count_2 = QRadioButton("2", self)
        self.blue_count_1.setChecked(True)

        self.blue_count_group = QButtonGroup(self)
        self.blue_count_group.addButton(self.blue_count_0)
        self.blue_count_group.addButton(self.blue_count_1)
        self.blue_count_group.addButton(self.blue_count_2)

        self.count_layout.addWidget(self.blue_count_0)
        self.count_layout.addWidget(self.blue_count_1)
        self.count_layout.addWidget(self.blue_count_2)

        self.count_group.setLayout(self.count_layout)
        self.control_layout.addWidget(self.count_group)

        # Goal 설정
        self.goal_group = QGroupBox("Select Goal")
        self.goal_layout = QHBoxLayout()

        self.goal_1 = QRadioButton("1", self)
        self.goal_2 = QRadioButton("2", self)
        self.goal_3 = QRadioButton("3", self)
        self.goal_1.setChecked(True)

        self.goal_button_group = QButtonGroup(self)
        self.goal_button_group.addButton(self.goal_1)
        self.goal_button_group.addButton(self.goal_2)
        self.goal_button_group.addButton(self.goal_3)

        self.goal_layout.addWidget(self.goal_1)
        self.goal_layout.addWidget(self.goal_2)
        self.goal_layout.addWidget(self.goal_3)

        self.goal_group.setLayout(self.goal_layout)
        self.control_layout.addWidget(self.goal_group)

        # Send 버튼 추가
        self.send_button = QPushButton("Send", self.centralwidget)
        self.send_button.clicked.connect(self.send_job)
        self.control_layout.addWidget(self.send_button)

        self.layout.addWidget(self.control_group)

        # 이미지 표시 QLabel
        self.label_1 = QLabel(self.centralwidget)
        self.layout.addWidget(self.label_1)

        self.setLayout(self.layout)

    def read_status(self, msg):
        """아두이노에서 지속적으로 상태를 읽어 GUI에 출력"""
        self.arduino = True
        response = msg.data

        if response != self.current_status:
            self.textBrowser.append(f"'conveyor_status': {response}")
            self.current_status = response


    def start_action(self):
        """ 시작 버튼 클릭 시 아두이노에 이동 명령 전송 """
        command = 'go'
        distance_text = self.distance_input.text()
        try:
            distance_mm = float(distance_text)
            if self.arduino:
                self.textBrowser.append(f"{distance_mm} 이동 요청")
                json_data = lib.create_json_msg(['control', 'distance.mm'], [command, 10.24 * distance_mm])    #?->mm 변환
                self.conveyor_publisher_.publish(json_data)
            else:
                self.textBrowser.append("Arduino 연결 없음")
        except ValueError:
            self.textBrowser.append("유효한 숫자를 입력하세요.")

    def stop_action(self):
        """ 정지 버튼 클릭 시 동작 """
        command = 'stop'
        distance_mm = 1
        if self.arduino:
            self.textBrowser.append("정지")
            json_data = lib.create_json_msg(['control'], [command])
            self.conveyor_publisher_.publish(json_data)
            #self.arduino.write(f"{distance_mm}\n".encode())

    def send_job(self):
        """Red & Blue 개수와 Goal을 선택 후 한 번에 Send 실행"""
        selected_red_count = next(btn.text() for btn in self.red_count_group.buttons() if btn.isChecked())
        selected_blue_count = next(btn.text() for btn in self.blue_count_group.buttons() if btn.isChecked())
        selected_goal = next(btn.text() for btn in self.goal_button_group.buttons() if btn.isChecked())

        # ROS 2 메시지 전송
        self.node.send_json_job(selected_red_count, selected_blue_count, selected_goal)
        
        # 결과 출력
        self.textBrowser.append(f"Red: {selected_red_count}, Blue: {selected_blue_count}, Goal: {selected_goal}")

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

def start_node(node):
    #rclpy.init()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
def main(args=None):
    """ GUI + ROS 2 노드 실행 """

    rclpy.init(args=args)
    
    app = QApplication(sys.argv)

    job_publisher = JobPublisher()

    # Qt GUI 실행
    gui = GUI(job_publisher)
    gui.show()

     # ROS 2 노드를 별도의 스레드에서 실행
    ros2_thread = threading.Thread(target=start_node, args=(job_publisher,))
    ros2_thread.start() 

    # GUI 실행 (여기서 프로그램이 멈춰 있음)
    app.exec_()

    # GUI가 종료되면 ROS 2 노드 정리
    rclpy.shutdown()
    ros2_thread.join()

if __name__ == "__main__":
    main()