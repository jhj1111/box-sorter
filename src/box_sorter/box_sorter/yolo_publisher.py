import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np

class YOLOPublisher(Node):
    def __init__(self):
        super().__init__('Yolo_publisher')
        self.iamge_subscriber = self.create_subscription(CompressedImage, '/image_raw/compressed', self.image_callback, 10)
        self.publisher_ = self.create_publisher(CompressedImage, '/yolo/compressed', 10)
        self.iamge_subscriber

        # YOLO 모델 로드
        self.yolo_model = YOLO("runs/detect/yolov8_train/weights/best.pt")

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        mtx = np.array([[1.35745561e+03, 0.00000000e+00, 7.31862038e+02],
        [0.00000000e+00, 1.35721160e+03, 4.39804697e+02],
        [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

        dist = np.array([[ 2.87362321e-02,  1.17548402e-01, -1.08876899e-05,
          2.27235180e-03, -5.09375304e-01]])
        
        h, w = frame.shape[:2]

        new_frame, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

        dst = cv2.undistort(frame, mtx, dist, None, new_frame)

        # YOLO 예측 실행
        results = self.yolo_model.predict(dst, conf=0.5)
        dst = results[0].plot()

        new_dst = cv2.resize(dst, (640, 360))
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 30]  # 30은 압축 품질
        _, compressed_image = cv2.imencode('.jpg', new_dst, encode_param)

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()  # 타임스탬프 추가
        msg.header.frame_id = "camera"  # 프레임 ID 설정
        msg.format = "jpeg"  # 압축 형식 설정
        msg.data = compressed_image.tobytes()  # 압축된 이미지 데이터

        # CompressedImage 퍼블리시
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing compressed image with bounding boxes.')

def main(args=None):
    rclpy.init(args=args)
    node = YOLOPublisher()
    rclpy.spin(node)

    # 종료 시 자원 정리
    node.cap.release()
    rclpy.shutdown()

if __name__ == '__main__':
    main()