import cv2
from ultralytics import YOLO
import numpy as np

# 모델 로드
model = YOLO("runs/detect/yolov8_train/weights/best.pt")

# 클래스 이름 가져오기
class_names = model.names  # YOLO 모델에 저장된 클래스 목록

# 테스트 이미지로 예측 수행
image_path = "data/dataset/test/images/blue_saved_image_001.jpg" 
results = model.predict(image_path, line_width=2, conf=0.5)

# NMS 파라미터
confidence_threshold = 0.5
nms_threshold = 0.4

# 이미지 불러오기
res_img = cv2.imread(image_path)

# 인식된 객체 결과 확인 및 NMS 적용
for result in results:
    boxes = []
    confidences = []
    class_ids = []

    # 객체 탐지 결과 수집
    for box in result.boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0])  # 바운딩 박스 좌표
        confidence = float(box.conf[0])         # 신뢰도
        class_id = int(box.cls[0])              # 클래스 ID

        if confidence >= confidence_threshold:
            boxes.append([x1, y1, x2 - x1, y2 - y1])  # (x, y, width, height)
            confidences.append(confidence)
            class_ids.append(class_id)

    # OpenCV NMS 적용
    indices = cv2.dnn.NMSBoxes(
        bboxes=boxes,
        scores=confidences,
        score_threshold=confidence_threshold,
        nms_threshold=nms_threshold
    )

    # 결과 시각화
    if isinstance(indices, np.ndarray):  # NMS 결과가 배열일 경우
        indices = indices.flatten().tolist()  # 리스트로 변환

    for i in indices:  # NMS 적용된 인덱스 리스트 순회
        x, y, w, h = boxes[i]
        class_id = class_ids[i]
        class_name = class_names[class_id]  # 클래스 ID를 이름으로 변환
        conf = confidences[i]

        # 바운딩 박스 그리기
        cv2.rectangle(res_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(res_img, f"{class_name} {conf:.2f}", (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # 결과 출력
        print(f"Class: {class_name}, Confidence: {conf:.2f}, Box: ({x}, {y}), ({x + w}, {y + h})")

# 이미지 표시 (로컬 환경)
cv2.imshow("Detection Result", res_img)
cv2.waitKey(0)  # 아무 키나 누를 때까지 창 유지
cv2.destroyAllWindows()  # 창 닫기
