# Jetson Orin Nano 환경 설정 및 실행 가이드

## 1. 목적

이 문서는 2026 IRC 휴머노이드 지능형 로봇 대회 비전 프로젝트를 NVIDIA Jetson Orin Nano 환경으로 이전할 때 필요한 설정과 실행 방법을 정리한다.

현재 개발은 개인 노트북 및 동아리 PC에서 진행 중이며, 최종적으로 다음 환경에서 실행할 예정이다.

- NVIDIA Jetson Orin Nano
- Ubuntu/Linux
- ROS 2
- Intel RealSense
- YOLO26
- ONNX Runtime 또는 TensorRT
- GPU 가속 추론

주의:

이 문서는 현재 일부 항목이 미검증 상태이다.
실제 Jetson에서 테스트한 뒤 버전과 명령어를 확정하여 업데이트해야 한다.

---

## 2. 현재 개발 PC와 Jetson의 차이

현재 개발 PC에서는 다음 방식으로 YOLO26을 실행한다.

```bash
ros2 run step yolo26_detector --ros-args -p device:=cpu
```

즉 현재 PC에서는:

```text
YOLO26 ONNX
→ ONNX Runtime
→ CPUExecutionProvider
```

방식으로 실행한다.

Jetson Orin Nano에서는 GPU 가속을 사용하는 것이 목표이다.

예상 구조:

```text
YOLO26 ONNX
→ ONNX Runtime
→ TensorRTExecutionProvider
```

또는:

```text
YOLO26 ONNX
→ ONNX Runtime
→ CUDAExecutionProvider
```

또는 추후 최적화 시:

```text
YOLO26
→ TensorRT Engine
→ Jetson GPU
```

---

## 3. 예상 Jetson 환경

아직 실제 버전은 확정하지 않았다.

Jetson 환경 이전 후 반드시 다음 정보를 기록한다.

```text
Jetson model:
Jetson Orin Nano

JetPack version:
TBD

Ubuntu version:
TBD

Python version:
TBD

ROS 2 distribution:
TBD

CUDA version:
TBD

cuDNN version:
TBD

TensorRT version:
TBD

OpenCV version:
TBD

NumPy version:
TBD

ONNX Runtime version:
TBD

RealSense SDK version:
TBD

realsense2_camera version:
TBD
```

---

## 4. 프로젝트 복사

GitHub 저장소를 Jetson으로 복사한다.

예:

```bash
cd ~
git clone https://github.com/geonwoo0407/IRC_vision.git my_cv
cd ~/my_cv
```

이미 저장소가 존재하는 경우:

```bash
cd ~/my_cv
git pull
```

---

## 5. ROS 2 환경 설정

ROS 2 설치 후 터미널에서 다음을 실행한다.

```bash
source /opt/ros/humble/setup.bash
```

현재 프로젝트는 ROS 2 Humble 기준으로 개발 중이다.

Jetson Ubuntu 및 JetPack 버전에 따라 ROS 2 배포판 호환성을 반드시 확인해야 한다.

---

## 6. 프로젝트 빌드

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

## 7. RealSense 실행

RealSense 카메라 실행:

```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py
```

카메라 토픽 확인:

```bash
ros2 topic list | grep camera
```

기본 컬러 이미지 토픽:

```text
/camera/color/image_raw
```

기본 Depth 관련 토픽 예:

```text
/camera/depth/image_rect_raw
/camera/aligned_depth_to_color/image_raw
```

실제 토픽 이름은 Jetson의 RealSense ROS 설정에 따라 확인해야 한다.

---

## 8. YOLO26 CPU 실행

GPU 설정 전에는 CPU로 먼저 정상 동작 여부를 확인한다.

```bash
cd ~/my_cv
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run step yolo26_detector --ros-args -p device:=cpu
```

CPU에서도 정상 동작해야 한다.

확인 항목:

```text
- 모델 파일 로드
- RealSense RGB 토픽 구독
- line 탐지
- ball 탐지
- goal 탐지
- backboard 탐지
- hurdle 탐지
- Bounding Box 표시
- FPS 표시
- /vision/detections 발행
- /vision/detections/image 발행
```

---

## 9. CUDA 실행 예정

CUDAExecutionProvider가 정상 설치된 경우 다음 방식으로 실행할 예정이다.

```bash
ros2 run step yolo26_detector --ros-args -p device:=cuda
```

예상 정상 로그:

```text
Provider: CUDAExecutionProvider
```

주의:

아직 실제 Jetson에서 검증하지 않았다.

---

## 10. TensorRT 실행 예정

TensorRTExecutionProvider가 정상 설치된 경우 다음 방식으로 실행할 예정이다.

```bash
ros2 run step yolo26_detector --ros-args -p device:=tensorrt
```

예상 정상 로그:

```text
Provider: TensorrtExecutionProvider
```

주의:

아직 실제 Jetson에서 검증하지 않았다.

JetPack, CUDA, TensorRT, ONNX Runtime 버전 호환성을 반드시 확인해야 한다.

---

## 11. device 파라미터

현재 `yolo26_detector.py`는 다음 값을 지원한다.

```text
auto
tensorrt
cuda
cpu
```

예:

```bash
ros2 run step yolo26_detector --ros-args -p device:=auto
```

```bash
ros2 run step yolo26_detector --ros-args -p device:=tensorrt
```

```bash
ros2 run step yolo26_detector --ros-args -p device:=cuda
```

```bash
ros2 run step yolo26_detector --ros-args -p device:=cpu
```

현재 개발 PC에서는:

```text
device:=cpu
```

사용을 권장한다.

Jetson에서는 실제 테스트 후 다음 중 가장 안정적인 방식을 선택한다.

```text
TensorRT
CUDA
CPU fallback
```

---

## 12. 모델 파일 관리

모델은 ROS 패키지에 포함되어 빌드 시 함께 설치된다.

```text
src/step/models/best.onnx
→ install/step/share/step/models/best.onnx
```

따라서 저장소를 clone하고 `colcon build`하면 Jetson에서도 별도 절대경로 수정 없이 기본 모델을 찾습니다.

예:

```text
~/my_cv/src/step/models/best.onnx
```

권장 구조:

```text
my_cv/
├── src/step/models/
│   └── best.onnx
├── docs/
├── src/
└── README.md
```

실행 시 ROS 파라미터로 모델 경로를 지정할 수 있다면 다음과 같이 사용한다.

```bash
ros2 run step yolo26_detector --ros-args \
  -p model_path:=/home/geonwoo/my_cv/models/best.onnx \
  -p device:=cpu
```

Jetson에서는 예:

```bash
ros2 run step yolo26_detector --ros-args \
  -p model_path:=/home/jetson/my_cv/models/best.onnx \
  -p device:=tensorrt
```

실제 사용자 홈 경로에 맞게 수정한다.

---

## 13. Jetson에서 주의할 점

Jetson에서는 다음 문제를 특히 확인해야 한다.

```text
- JetPack 버전
- CUDA 버전
- TensorRT 버전
- ONNX Runtime GPU 패키지
- OpenCV 버전
- NumPy 버전
- cv_bridge 호환성
- RealSense SDK 호환성
- ROS 2 배포판 호환성
- 모델 입력 크기
- GPU 메모리 사용량
- FPS
- 발열
- 전력 모드
```

---

## 14. NumPy와 cv_bridge

개발 PC에서 NumPy 2.x 사용 시 ROS 2 Humble의 cv_bridge와 충돌이 발생한 적이 있다.

오류 예:

```text
AttributeError: _ARRAY_API not found
```

```text
Segmentation fault
```

따라서 Jetson에서도 NumPy 버전을 함부로 올리지 않는다.

개발 PC에서 현재 정상 확인된 버전:

```text
NumPy 1.26.4
```

다만 Jetson에서는 JetPack 및 ROS 패키지와의 호환성을 기준으로 실제 테스트 후 버전을 확정해야 한다.

---

## 15. OpenCV GUI

개발 PC에서 headless OpenCV 사용 시 다음 오류가 발생했다.

```text
The function is not implemented
```

원인:

```text
GUI: NONE
```

현재 개발 PC 정상 상태:

```text
OpenCV 4.11.0
GUI: QT5
```

Jetson을 모니터 없이 headless로 사용할 경우 `cv2.imshow()`를 사용하지 않고 다음 방식으로 결과를 확인하는 것이 더 적합할 수 있다.

```text
/vision/detections/image
```

ROS 이미지 토픽 확인:

```bash
rqt_image_view
```

또는 네트워크를 통해 다른 PC에서 확인하는 방식을 고려한다.

---

## 16. 권장 Jetson 이전 순서

### 1단계

Jetson 기본 환경 확인

```bash
uname -a
```

```bash
python3 --version
```

```bash
nvcc --version
```

```bash
dpkg -l | grep TensorRT
```

### 2단계

ROS 2 정상 동작 확인

```bash
ros2 --version
```

### 3단계

RealSense 연결 확인

```bash
lsusb | grep -i intel
```

### 4단계

RealSense ROS 노드 실행

```bash
ros2 launch realsense2_camera rs_launch.py
```

### 5단계

카메라 토픽 확인

```bash
ros2 topic list | grep camera
```

### 6단계

프로젝트 빌드

```bash
cd ~/my_cv
colcon build --symlink-install
source install/setup.bash
```

### 7단계

YOLO26 CPU 실행

```bash
ros2 run step yolo26_detector --ros-args -p device:=cpu
```

### 8단계

CUDA 테스트

```bash
ros2 run step yolo26_detector --ros-args -p device:=cuda
```

### 9단계

TensorRT 테스트

```bash
ros2 run step yolo26_detector --ros-args -p device:=tensorrt
```

### 10단계

FPS와 안정성 비교

```text
CPU FPS:
CUDA FPS:
TensorRT FPS:
```

---

## 17. 최종 목표

Jetson Orin Nano에서 다음 기능을 실시간으로 수행하는 것이 목표이다.

```text
Intel RealSense
        ↓
RGB + Depth
        ↓
YOLO26 객체 탐지
        ↓
line / ball / goal / backboard / hurdle
        ↓
객체별 거리 및 상대 위치 계산
        ↓
VisionState
        ↓
미니맵 / 위치 추정
        ↓
Mission FSM
        ↓
알고리즘 및 모션 시스템
```

---

## 18. 현재 상태

현재까지 확인된 사항:

```text
[완료]
- 개발 PC에서 RealSense 연결
- ROS 2 컬러 이미지 토픽 구독
- YOLO26 ONNX 모델 로드
- line 탐지
- Bounding Box 표시
- Confidence 표시
- FPS 표시
- CPUExecutionProvider 실행
- OpenCV GUI 정상화
- cv_bridge 정상화

[미완료]
- Jetson Orin Nano 이전
- CUDAExecutionProvider 검증
- TensorRTExecutionProvider 검증
- Depth 연동
- 객체 거리 계산
- 3D 상대 좌표 계산
- line 방향 계산
- VisionState 통합
- 미니맵 및 위치 추정
- 알고리즘 및 모션 시스템 연동
```

---

## 19. 업데이트 원칙

Jetson에서 실제 테스트할 때마다 다음을 반드시 기록한다.

```text
날짜:
JetPack 버전:
ROS 2 버전:
CUDA 버전:
TensorRT 버전:
ONNX Runtime 버전:
OpenCV 버전:
NumPy 버전:
RealSense SDK 버전:
실행 명령:
FPS:
발생한 오류:
해결 방법:
```

추측으로 확정하지 말고 실제 테스트 결과를 기준으로 문서를 갱신한다.
