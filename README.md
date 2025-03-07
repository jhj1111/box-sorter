# 📦 Box Sorter Package

## 📌 Requirements

### 🔹 TurtleBot3 Manipulation Packages
- `turtlebot3_manipulation_bringup`
- `turtlebot3_manipulation_cartographer`
- `turtlebot3_manipulation_description`
- `turtlebot3_manipulation_hardware`
- `turtlebot3_manipulation_moveit_config`
- `turtlebot3_manipulation_navigation2`
- `turtlebot3_manipulation_teleop`

### 🔹 Additional Packages
- `turtlebot_cosmo_interface`
- `turtlebot_moveit`

---

## 🚀 실행 방법 (How to Run)

### 🖥️ 1. TurtleBot3에서 실행 (SSH 접속 필요)

#### 1.1 TurtleBot3 Bringup 실행
```bash
ros2 launch turtlebot3_manipulation_bringup hardware.launch.py
```
- TurtleBot3의 하드웨어 제어 노드 실행

#### 1.2 YOLO & ArUco Detector 실행
```bash
ros2 launch aruco_yolo aruco_yolo.launch.py
```
- **YOLO 노드**: Red, Blue, Purple 박스 클래스 및 중심 검출
- **ArUco 노드**: ArUco 마커 종류 및 거리 데이터 측정

---

### 💻 2. PC에서 실행

#### 2.1 경로 설정 노드 실행
```bash
ros2 launch box_sorter_manipulator moveit_core.launch.py
```
- MoveIt과 연동하여 Arm 및 Gripper의 경로를 설정

#### 2.2 Arm Controller 실행
```bash
ros2 run turtlebot_moveit turtlebot_arm_controller
```
- Arm Manipulator의 조인트 이동 명령 실행

#### 2.3 Manager 노드 실행
```bash
ros2 run box_sorter simple_manager_node
```
- 상태(status)에 따른 실행 방법 정의

#### 2.4 GUI & Conveyor 노드 실행
```bash
ros2 launch box_sorter GUI_conveyor.launch.py
```

##### ✅ GUI 노드 실행
```bash
ros2 run box_sorter job_publisher
```
- 이동 명령 실행 및 상태 출력
- **Publisher**:
  - `/job_topic` (`std_msgs/String`)
  - `/conveyor/control` (`std_msgs/String`)
- **Subscriber**:
  - `/yolo/compressed` (`sensor_msgs/CompressedImage`)
  - `/conveyor/status` (`std_msgs/String`)

##### ✅ Conveyor 노드 실행
```bash
ros2 run box_sorter conveyor
```
- 컨베이어 벨트 상태를 퍼블리시함
- **Publisher**: 
  - `/conveyor/control` (`std_msgs/String`)
  - `/conveyor/status` (`std_msgs/String`)
- 아두이노와 연결하여 컨베이어 벨트를 제어함

---

## 📜 추가 정보 (Additional Information)
- ROS 2 Humble 기반에서 테스트됨
- ArUco 및 YOLO를 활용한 물체 인식 기능 포함
- MoveIt을 활용한 Arm Manipulator 제어

📩 문의 사항이 있으면 개발팀에 연락하세요! 🚀
*~쓸데없는거 넣지 마라~*