import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(CompressedImage, 'compressed_low', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FPS, 25)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        
        print(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH), self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        # YOLO 모델 로드
        self.yolo_model = YOLO("runs/detect/yolov8_train/weights/best.pt")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to capture frame")
            return

        # YOLO 예측 실행
        results = self.yolo_model.predict(frame, conf=0.5)

        # 바운딩 박스 그리기
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # 바운딩 박스 좌표
                confidence = float(box.conf[0])         # 신뢰도
                class_id = int(box.cls[0])              # 클래스 ID
                label = self.yolo_model.names[class_id]  # 클래스 이름

                # 바운딩 박스 그리기
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"{label} {confidence:.2f}", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                self.get_logger().info(f"Detected: {label} ({confidence:.2f}) at ({x1},{y1})-({x2},{y2})")

        # 이미지를 압축된 형식으로 변환 후 전송
        compressed_img_msg = self.bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpeg')
        self.publisher_.publish(compressed_img_msg)
        self.get_logger().info('Publishing compressed image with bounding boxes.')

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    rclpy.spin(node)

    # 종료 시 자원 정리
    node.cap.release()
    rclpy.shutdown()

if __name__ == '__main__':
    main()