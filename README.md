# Box Sorter ROS2 Package

## 개요
`box_sorter` 패키지는 ROS2를 기반으로 동작하는 박스 분류 시스템입니다. 본 패키지는 박스를 인식하고, YOLO를 이용한 객체 검출을 수행하며, 컨베이어 벨트의 상태를 관리합니다.
~~gpt가 헛소리한다~~

## 설치 및 실행 방법

### 1. 패키지 빌드
```bash
colcon build --packages-select box_sorter
source install/setup.bash
```

### 2. 노드 실행

#### 2.1. Job Publisher (GUI 실행)
```bash
ros2 run box_sorter job_publisher
```
- GUI를 실행하여 작업을 관리합니다.

#### 2.2. Image Publisher (카메라 이미지 전송)
```bash
ros2 run box_sorter img_publisher
```
- 카메라에서 촬영한 이미지를 퍼블리시합니다.
- 토픽: `/image_raw/compressed` (`sensor_msgs/CompressedImage`)

#### 2.3. YOLO Publisher (YOLO 객체 검출)
```bash
ros2 run box_sorter yolo_publisher
```
- YOLO를 이용하여 박스를 검출하고 결과를 퍼블리시합니다.
- 퍼블리시 토픽:
  - `/yolo/compressed` (`sensor_msgs/CompressedImage`) : YOLO 처리된 이미지
  - `/yolo/detected_info` (`std_msgs/String`) : YOLO 검출 정보

#### 2.4. Conveyor (컨베이어 벨트 상태 관리)
```bash
ros2 run box_sorter conveyor
```
- 컨베이어 벨트 상태를 퍼블리시합니다.
- 퍼블리시 토픽: `/conveyor/status` (`std_msgs/String`)
- 아두이노와 연결하여 컨베이어 벨트를 제어합니다.

## 로봇 실행
```bash
ros2 launch turtlebot3_manipulation_bringup hardware.launch.py
```
```bash
ros2 launch aruco_yolo aruco_yolo.launch.py
```

## 의존성 패키지
- ROS2 Humble
- OpenCV
- YOLOv5 또는 YOLOv8
- Arduino 통신을 위한 `rosserial` 또는 `ros2serial` (필요 시)

## 라이선스
MIT License

