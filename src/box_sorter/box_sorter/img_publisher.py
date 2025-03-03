import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(CompressedImage, 'compressed_low', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)  # 첫 번째 카메라 사용

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # 이미지를 압축된 형식으로 변환
            compressed_img_msg = self.bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpeg')
            self.publisher_.publish(compressed_img_msg)
            self.get_logger().info('Publishing compressed image.')

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    rclpy.spin(node)

    # 종료 시 자원 정리
    node.cap.release()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
