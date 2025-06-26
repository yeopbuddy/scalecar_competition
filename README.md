# 🚗 ScaleCar Autonomous Driving - ROS Project

ROS 기반 스케일카 자율주행 경진대회 프로젝트입니다.
Lane Keeping, Object Detection, Stop Line Detection 등 다양한 자율주행 요소들을 통합해서 구현하였습니다.

---

## 포맷 구조

```
wego_ws/
└── src/
    ├── lane_pkg/              # 차선 인식 및 주행 알고리즘
    ├── object_pkg/            # YOLO 기반 객체 탐지
    ├── stopline_pkg/          # 정지선 감지 알고리즘
    ├── serial_pkg/            # 차반 제어용 직렬 통신 노드
    └── ...                    # 기타 ROS 노드
```

---

## 기본 기능

| 기능             | 설명 |
|------------------|------|
| 🚘 차선 유지 (Lane Keeping) | OpenCV 기반 차선 추적, PID 제어 |
| 🚩 정지선 감지 (Stop Line Detection) | ROI 내 수평 라인 감지 알고리버즘 |
| 🎯 객체 탐지 (Object Detection) | YOLOv5-tiny + Custom Dataset |
| 🔧 제어 | 직렬 통신으로 모터 속도 및 조향 각도 제어 |
| 🧪 시뮬레이션 | 실차 기반 테스트 및 ROS 시각화 (RViz 등) |

---

## 사용 기술

- **ROS Noetic (Ubuntu 20.04)**
- **Python, C++**
- **OpenCV, NumPy**
- **YOLOv5-tiny** (Custom 학습)
- **RViz, rqt, rosbag**

---

## 관련 자료

- 대회 정보: AutoRace 2024 스케일카 자율주행 경진대회

---
