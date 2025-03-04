import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2, sys

class CameraPublisher(Node):
    def __init__(self, cam):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(CompressedImage, 'image_raw/compressed', 10)
        self.timer = self.create_timer(0.2, self.timer_callback)  # 10 Hz
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(int(cam))
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FPS, 5)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
        
        print(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH), self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    def timer_callback(self):
        # 카메라에서 한 프레임 읽기
        ret, frame = self.cap.read()

        if ret:
            # OpenCV 이미지 (BGR)을 JPEG로 압축
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 30]  # 30은 압축 품질
            _, compressed_image = cv2.imencode('.jpg', frame, encode_param)

            # 압축된 이미지를 CompressedImage 메시지로 변환
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()  # 타임스탬프 추가
            msg.header.frame_id = "camera"  # 프레임 ID 설정
            msg.format = "jpeg"  # 압축 형식 설정
            msg.data = compressed_image.tobytes()  # 압축된 이미지 데이터

            # CompressedImage 퍼블리시
            self.publisher_.publish(msg)

            # encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 10]  # 90은 압축 품질
            # _, compressed_image_low = cv2.imencode('.jpg', frame, encode_param)
            # msg.data = compressed_image_low.tobytes()  # 압축된 이미지 데이터
            # self.publisher_low_.publish(msg)

            # self.get_logger().info('Publishing compressed image... {%d}, {%d}' % (len(compressed_image), len(compressed_image_low)))
            self.get_logger().info('Publishing compressed image... {%d}' % (len(compressed_image),))

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        cam = '2'
    else : cam = sys.argv[1]

    node = CameraPublisher(cam)
    rclpy.spin(node)

    # 종료 시 자원 정리
    node.cap.release()
    rclpy.shutdown()

if __name__ == '__main__':
    main()