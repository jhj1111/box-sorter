import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2, os
import numpy as np
from ament_index_python.packages import get_package_share_directory

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('yolo_subscriber')
        self.subscription_rgb = self.create_subscription(
            CompressedImage,
            'image_raw/compressed',
            self.listener_callback_rgb,
            10)
        
        self.publisher_ = self.create_publisher(CompressedImage, 'yolo/compressed', 10)

        self.subscription_rgb  # prevent unused variable warning

        # YOLO 모델 로드
        # config_dir = os.path.join(get_package_share_directory('box_sorter'), 'config')
        # yolo_bt = 'best.pt'
        # yolo_path = os.path.join(config_dir, yolo_bt)
        self.yolo_model = YOLO("runs/detect/yolov8_train/weights/best.pt")
        #self.yolo_model = YOLO(yolo_path)

    def listener_callback_rgb(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # Decode to color image

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
        self.get_logger().info('Publishing compressed image with bounding boxes.')

        # cv2.imshow('RGB Image', frame)
        # cv2.waitKey(1)  # Display the image until a keypress

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    rclpy.spin(node)

    # 종료 시 자원 정리
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()