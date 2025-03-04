import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2, os
import numpy as np
from ament_index_python.packages import get_package_share_directory

class YOLOPublisher(Node):
    def __init__(self):
        super().__init__('Yolo_publisher')
        self.iamge_subscriber = self.create_subscription(CompressedImage, '/image_raw/compressed', self.image_callback, 10)
        self.publisher_ = self.create_publisher(CompressedImage, '/yolo/compressed', 10)
        self.yolo_info_publisher = self.create_publisher(String, 'yolo/detected_info', 10)
        self.iamge_subscriber

        # YOLO 모델 로드
        config_dir = os.path.join(get_package_share_directory('box_sorter'), 'config')
        yolo_file = os.path.join(config_dir, 'best.pt')
        # self.yolo_model = YOLO("runs/detect/yolov8_train/weights/best.pt")
        self.yolo_model = YOLO(yolo_file)

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

        shp = dst.shape
        dst = cv2.line(dst, (0, int(shp[0]/2)), (shp[1], int(shp[0]/2)), (0, 255, 255), 3)
        dst = cv2.line(dst, (int(shp[1]/2), 0), (int(shp[1]/2), shp[0]-1), (0, 255, 255), 3)

        # 객체 감지 결과 처리
        detected_info = []
        for result in results:
            for box in result.boxes:
                class_id = int(box.cls.item())  # 감지된 객체 클래스 ID
                x_center = (box.xywh[0][0] - (w / 2)) / (w / 2)  # x 좌표 정규화 (-1 ~ 1)
                y_center = ((h / 2) - box.xywh[0][1]) / (h / 2)  # y 좌표 정규화 (-1 ~ 1)
                detected_info.append([class_id, x_center, y_center, 0])  # 위치 정보 추가 (추후 업데이트)

        # 감지된 정보를 String 메시지로 변환
        detected_info_msg = String()
        detected_info_msg.data = str(detected_info)
        self.yolo_info_publisher.publish(detected_info_msg)
        self.get_logger().info(f'Publishing detected info: {detected_info}')

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

    def convert_box_center_2_mm(self, results, img_size, pix_2_mm):
        cls = results[0].boxes.cls.tolist()
        centers = []
        centers_img = []
        
        for i, r in enumerate(results[0].boxes.xywh):
            d = r.tolist()
            centers.append( (int(cls[i]), (d[0] - img_size[0]/2)*pix_2_mm, (d[1] - img_size[1]/2)*pix_2_mm) )
            centers_img.append( (d[0], d[1]) )
        
        return centers, centers_img

    def inference(self, img_color, pix_2_mm):
        results = self.model(img_color)
        plots = results[0].plot()
        shp = img_color.shape

        plots = cv2.line(plots, (0, int(shp[0]/2)), (shp[1], int(shp[0]/2)), (0, 255, 255), 3)
        plots = cv2.line(plots, (int(shp[1]/2), 0), (int(shp[1]/2), shp[0]-1), (0, 255, 255), 3)
        info, info_img = self.convert_box_center_2_mm(results, (shp[1], shp[0]), pix_2_mm)

        font = cv2.FONT_ITALIC
        font_scale = 1
        font_thickness = 2
        for i, (x, y) in enumerate(info_img):
            x = int(x)
            y = int(y)
            text = "%d, %d" % (info[i][1]*1000, info[i][2]*1000)

            text_size, _ = cv2.getTextSize(text, font, font_scale, font_thickness)
            text_w, text_h = text_size
            org = (x, y)

            cv2.rectangle(plots, (x - 5, y - 5), (x + text_w + 5, y + text_h + 5), (0, 0, 0), -1)
            cv2.putText(plots, text, (x, y + text_h + font_scale - 1), font, font_scale, (0, 255, 0), font_thickness)

        return plots, info

def main(args=None):
    rclpy.init(args=args)
    node = YOLOPublisher()
    rclpy.spin(node)

    # 종료 시 자원 정리
    node.cap.release()
    rclpy.shutdown()

if __name__ == '__main__':
    main()