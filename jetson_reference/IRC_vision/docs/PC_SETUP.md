# PC 개발 환경 설정 및 실행 가이드

## 1. 프로젝트 개요

이 문서는 2026 IRC 휴머노이드 지능형 로봇 대회 비전 프로젝트의 PC 개발 환경 설정과 실행 방법을 정리한다.

주요 환경:

- Ubuntu Linux
- ROS 2 Humble
- Intel RealSense
- OpenCV
- YOLO26 ONNX
- ONNX Runtime
- Python 3.10
- NVIDIA Jetson Orin Nano로 최종 이전 예정

현재 개발 PC에서는 YOLO26을 CPUExecutionProvider로 실행한다.

---

## 2. 프로젝트 경로

기본 프로젝트 경로:

```bash
~/my_cv
```

주요 구조:

```text
my_cv/
├── archive/
│   └── legacy_opencv/     # 현재 사용하지 않는 초기 OpenCV 실험
├── docs/
│   ├── PC_SETUP.md
│   └── JETSON_SETUP.md
│
├── src/
│   ├── step/                # 카메라, YOLO, analyzer, 객체별 planner
│   │   ├── setup.py
│   │   ├── package.xml
│   │   ├── models/
│   │   │   └── best.onnx
│   │   └── step/
│   │       ├── yolo26_detector.py
│   │       └── unified_vision_node.py
│   └── mission_control/     # 미션 우선순위와 단일 모션 명령
│       ├── package.xml
│       └── mission_control/
│           └── motion_decision_node.py
│
├── build/
├── install/
└── log/
```

---

## 3. RealSense 실행

첫 번째 터미널에서 RealSense 카메라 노드를 실행한다.

```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
```

정상적으로 실행되면 다음과 같은 이미지 토픽을 확인할 수 있다.

```bash
ros2 topic list | grep camera
```

대표적인 컬러 이미지 토픽:

```text
/camera/color/image_raw
```

현재 비전 노드들은 기본적으로 이 컬러 이미지 토픽을 구독한다.

---

## 4. 프로젝트 빌드

새 Python 노드를 추가하거나 `setup.py`의 `entry_points`를 수정한 경우 다시 빌드한다.

```bash
cd ~/my_cv
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

특정 패키지만 빌드하려면:

```bash
cd ~/my_cv
source /opt/ros/humble/setup.bash
colcon build --packages-select step mission_control --symlink-install
source install/setup.bash
```

---

## 5. 기존 OpenCV 비전 노드 보관

초기 `look_ground`, `look_gground`, `find_direct`, `find_ddirect`는 흰색 테이프를 threshold, contour, ROI로 처리하던 프로토타입입니다. 현재는 `archive/legacy_opencv`에 보관하며 ROS 2 `console_scripts`에서는 제거했습니다. 경기용 실행에는 사용하지 않습니다.

---

## 6. YOLO26 객체 탐지 실행

RealSense를 먼저 실행한 상태에서 다른 터미널에서 실행한다.

개발 PC에서는 TensorRT를 사용하지 않고 CPU를 강제로 지정한다.

```bash
cd ~/my_cv
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run step yolo26_detector --ros-args -p device:=cpu
```

현재 YOLO26 클래스:

```text
line
ball
goal
backboard
hurdle
```

YOLO26 모델 기본 경로:

```text
src/step/models/best.onnx
→ 빌드 후 share/step/models/best.onnx
```

---

## 7. CPU 강제 실행이 필요한 이유

기본 설정이 다음과 같으면:

```text
device = auto
```

ONNX Runtime이 다음 순서로 실행 장치를 찾는다.

```text
TensorRT
→ CUDA
→ CPU
```

현재 개발 PC에는 TensorRT 관련 라이브러리 중 다음 파일이 없어 오류 로그가 발생했다.

```text
libnvinfer.so.10
```

오류 예:

```text
Failed to load library libonnxruntime_providers_tensorrt.so
libnvinfer.so.10: cannot open shared object file
```

그러나 이후 CPU로 fallback하여 실행은 가능하다.

불필요한 빨간 오류 로그를 피하려면 다음처럼 CPU를 직접 지정한다.

```bash
ros2 run step yolo26_detector --ros-args -p device:=cpu
```

정상 로그 예:

```text
Provider: CPUExecutionProvider
```

---

## 8. 현재 정상 동작 환경

현재 확인된 정상 조합:

```text
Python: 3.10
OpenCV: 4.11.0
NumPy: 1.26.4
OpenCV GUI: QT5
cv_bridge: 정상
```

확인 명령:

```bash
/usr/bin/python3 -c "import cv2, numpy; print('OpenCV:', cv2.__version__); print('NumPy:', numpy.__version__)"
```

GUI 확인:

```bash
/usr/bin/python3 -c "import cv2; print(cv2.getBuildInformation())" | grep -A 5 "GUI"
```

정상 예:

```text
GUI: QT5
```

cv_bridge 확인:

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 -c "from cv_bridge import CvBridge; print('cv_bridge OK')"
```

정상 출력:

```text
cv_bridge OK
```

---

## 9. OpenCV headless 문제

이전에 다음 패키지가 설치되어 있었다.

```text
opencv-contrib-python-headless 4.11.0.86
```

이 버전은 GUI 기능을 포함하지 않기 때문에:

```python
cv2.imshow()
```

실행 시 다음 오류가 발생했다.

```text
The function is not implemented
Rebuild the library with Windows, GTK+ 2.x or Cocoa support
```

확인 명령:

```bash
/usr/bin/python3 -c "import cv2; print(cv2.getBuildInformation())" | grep -A 10 "GUI"
```

문제 상태:

```text
GUI: NONE
```

현재는 GUI 지원 OpenCV를 사용하며 정상 상태는:

```text
GUI: QT5
```

이다.

---

## 10. NumPy 2.x와 cv_bridge 충돌

OpenCV 설치 과정에서 NumPy가 자동으로 2.x로 업그레이드된 적이 있다.

그 결과 ROS 2 Humble의 cv_bridge와 ABI 충돌이 발생했다.

오류 예:

```text
A module that was compiled using NumPy 1.x cannot be run in NumPy 2.x
```

```text
AttributeError: _ARRAY_API not found
```

```text
Segmentation fault
```

해결 방법:

```bash
/usr/bin/python3 -m pip install --force-reinstall "numpy<2"
```

현재 정상 버전:

```text
NumPy 1.26.4
```

확인:

```bash
/usr/bin/python3 -c "import numpy; print(numpy.__version__)"
```

---

## 11. 현재 통합 실행 구조

현재 경기용 구조는 YOLO26 detector, 통합 analyzer 실행, 통합 motion decision으로 구성합니다.

```text
Intel RealSense
        ↓
ROS 2 이미지 토픽
        ↓
/camera/color/image_raw
        ↓
/vision/detections
        ↓
unified_vision_node
        ↓
motion_decision_node
```

각 노드는 별도 터미널에서 실행합니다.

```bash
ros2 run step yolo26_detector --ros-args -p device:=cpu
ros2 run step unified_vision_node
ros2 run mission_control motion_decision_node
```

RealSense 실행 명령은 OpenCV와 YOLO26에서 동일하다.

```bash
ros2 launch realsense2_camera rs_launch.py
```

---

## 12. YOLO26 탐지 결과

현재 YOLO26 노드는 다음 기능을 수행한다.

- RealSense RGB 토픽 구독
- YOLO26 ONNX 모델 로드
- 객체 탐지
- Bounding Box 표시
- 클래스 이름 표시
- Confidence 표시
- FPS 표시
- 탐지 결과 JSON 토픽 발행
- 탐지 결과 이미지 토픽 발행

발행 토픽:

```text
/vision/detections
```

```text
/vision/detections/image
```

기본 구독 토픽:

```text
/camera/color/image_raw
```

---

## 13. 현재 개발 방향

현재 목표는 단순 객체 탐지를 넘어 실제 로봇 주행과 미션 수행에 필요한 정보를 계산하는 것이다.

예정 기능:

```text
line
→ 중심점
→ 선 방향
→ 조향 오차

hurdle
→ 거리
→ 상대 위치

ball
→ 거리
→ 좌우 상대 위치
→ 접근 위치 계산

goal / backboard
→ 골대 위치
→ 정렬
→ 투입 위치 계산
```

향후 최종 비전 출력 예:

```python
{
    "current_zone": "BALL_APPROACH",
    "line": {
        "detected": True,
        "angle_deg": -3.2,
        "offset_m": 0.04
    },
    "basketball": {
        "detected": True,
        "distance_m": 1.42,
        "relative_x_m": -0.17
    },
    "hurdle": {
        "detected": False
    },
    "goal": {
        "detected": False
    }
}
```

---

## 14. 자주 사용하는 실행 순서

### 터미널 1: RealSense

```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py
```

### 터미널 2: YOLO26

```bash
cd ~/my_cv
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run step yolo26_detector --ros-args -p device:=cpu
```

---

## 15. 주의사항

- 새 노드를 추가하면 `setup.py`의 `entry_points`에 등록해야 한다.
- `setup.py` 수정 후 반드시 다시 `colcon build`를 수행한다.
- 빌드 후 `source install/setup.bash`를 다시 실행한다.
- RealSense 토픽 이름이 다르면 코드의 `image_topic` 파라미터를 확인한다.
- 현재 개발 PC에서는 TensorRT를 사용하지 않는다.
- 현재 PC에서는 `device:=cpu` 사용을 권장한다.
- Jetson Orin Nano에서는 TensorRT 또는 CUDA 기반 실행을 별도로 검증할 예정이다.
