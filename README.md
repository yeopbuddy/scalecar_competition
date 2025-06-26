# 🚗 ScaleCar Autonomous Driving - ROS Project

ROS 기반 스케일카 자율주행 경진대회 프로젝트입니다.  
Lane Keeping, Object Detection, Stop Line Detection 등 다양한 자율주행 요소들을 통합하여 구현하였습니다.

---

## 📁 프로젝트 구조

```
src/
├── ar_track_alvar/        # AR 마커 인식
├── darknet_ros/           # YOLOv3 기반 객체 탐지 노드
├── detection_msgs/        # 커스텀 메시지 정의
├── fiducials/             # Fiducial 마커 인식 노드
├── image_pipeline/        # 카메라 보정 및 이미지 전처리 파이프라인
├── obstacle_detector/     # Lidar 기반 다중 정적 장애물 탐지
├── racecar/               # 차량 하드웨어 설정 및 모델 관련 패키지
├── razor_imu_9dof/        # IMU 센서 통신 및 상태 파악
├── rplidar_ros/           # RPLidar 드라이버 및 데이터 수집
├── usb_cam/               # USB 카메라 입력 노드
├── vesc/                  # VESC 기반 모터 제어 노드
├── wego/                  # 상위 통합 로직 및 런치 관리
└── yolov5_ros/            # YOLOv5 기반 객체 탐지 노드
```

---

## ⚙️ 주요 기능

| 기능             | 설명 |
|------------------|------|
| 🚘 차선 유지 (Lane Keeping) | OpenCV 기반 차선 추적, PID 제어 |
| 🛑 정지선 감지 (Stop Line Detection) | ROI 내 수평 라인 감지 알고리즘 |
| 🎯 객체 탐지 (Object Detection) | YOLOv3/YOLOv5 + Custom Dataset 활용 |
| 🔧 차량 제어 | VESC 기반 모터 속도 및 조향 각도 제어 |
| 🌐 센서 통합 | Lidar, IMU, 카메라, 마커 기반 위치 인식 융합 |
| 🧪 시뮬레이션 및 시각화 | RViz, rosbag, rqt 등을 통한 테스트 및 분석 |

---

## 🛠️ 사용 기술

- **ROS Noetic (Ubuntu 20.04)**
- **Python, C++**
- **OpenCV, NumPy**
- **YOLOv3 / YOLOv5** (darknet_ros / yolov5_ros)
- **RPLidar, Razor IMU, USB Camera**
- **VESC 모터 컨트롤러**
- **RViz, rqt, rosbag**

---

## 관련 자료

- 대회 정보: AutoRace 2024 스케일카 자율주행 경진대회

---

