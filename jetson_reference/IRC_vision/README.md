# IRC-STEP Humanoid Robot System

2026 IRC 휴머노이드 로봇 지능형 경기대회를 위한 STEP 휴머노이드 로봇 통합 시스템이다.

본 프로젝트는 RealSense + YOLO 기반 Vision, mission별 navigation planner, motion decision, 실제 로봇 motion executor를 하나의 ROS 2 pipeline으로 연결한다.

현재 Jetson 기반 실제 로봇 환경에서 다음 경로를 사용한다.

```text
RealSense
   ↓
YOLO / Vision Analyzer
   ↓
Mission Planner
   ↓
Motion Decision
   ↓
/navigation/motion_command
   ↓
Motion Command Bridge
   ↓
/motion/executor/request
   ↓
SDK Motion Executor
   ↓
RobotMotionPlayer
   ↓
Dynamixel
   ↓
STEP Humanoid Robot
1. System Architecture

주요 구성은 다음과 같다.

Vision
Intel RealSense RGB / aligned depth 사용
YOLO TensorRT .engine 기반 object detection
Ball / Goal / Hurdle / Line 분석
Line corner 및 depth 기반 경로 분석
Hurdle + Line fusion
실제 Jetson camera topic 사용
/camera/camera/color/image_raw
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/color/camera_info

주요 Vision output:

/vision/detections
/vision/ball_info
/vision/goal_info
/vision/hurdle_info
/vision/line_info
2. Navigation / Mission Control

지원 mission:

Line following
Ball approach / alignment
Goal approach / alignment
Hurdle approach / alignment

Vision 결과를 mission별 planner에서 처리한 뒤 최종적으로 motion action을 생성한다.

Vision
→ Mission-specific planner
→ Mission priority / lock
→ Motion decision
→ /navigation/motion_command

Mission 간 간섭을 막기 위해 mission lock 및 object priority 정책을 적용한다.

3. Line Navigation

Line navigation에는 다음 기능이 적용되어 있다.

3-frame turn confirmation
Line center correction
Corner detection
Depth 기반 corner distance 계산
Curve-follow guard
Recovery motion
Composite recovery action
Motion 실행 중 Vision buffering
TIMEOUT recovery
Composite Recovery Action

Line correction은 다음 형식의 action을 사용한다.

RECOVER_LEFT_TURN_LEFT_n
RECOVER_LEFT_TURN_RIGHT_n
RECOVER_RIGHT_TURN_LEFT_n
RECOVER_RIGHT_TURN_RIGHT_n

여기서 앞의 RECOVER_LEFT / RECOVER_RIGHT는 line 상태/context를 의미하고,
TURN_LEFT / TURN_RIGHT가 실제 로봇의 회전 방향을 의미한다.

현재 각도와 suffix contract는 다음과 같다.

보정 각도	LEFT suffix	RIGHT suffix
15°	4	4
30°	6	6
45°	8	8
60°	10	10
75°	12	12
90°	15	15
4. Forward Motion

Line 및 mission approach에서 거리별 forward action을 사용한다.

STRAIGHT_0
STRAIGHT_1
STRAIGHT_2
STRAIGHT_3
STRAIGHT_4
STRAIGHT_5

현재 실제 SDK motion mapping:

Internal Motion	RobotMotionPlayer Motion
line_forward_2	전진진짜실전(2회)
line_forward_4	전진진짜실전(4회)
line_forward_6	전진진짜실전(6회)
line_forward_8	전진진짜실전(8회)
line_forward_10	전진진짜실전(8회)

기본 STRAIGHT는 현재:

STRAIGHT
→ line_forward_6
→ 전진진짜실전(6회)

`forward` 별칭은 `APPROACH` 등 다른 동작과의 호환성을 위해
전진진짜실전(8회)에 연결한다.

5. Right Turn Motion

우회전은 기존 production의 legacy internal motion ID를 유지하면서
새 RobotMotionPlayer motion으로 연결한다.

Action suffix	Legacy internal motion	SDK motion
TURN_RIGHT_4	line_turn_right_2	우회전실실전(4회)
TURN_RIGHT_6	line_turn_right_4	우회전실실전(6회)
TURN_RIGHT_8	line_turn_right_6	우회전실실전(8회)
TURN_RIGHT_10	line_turn_right_8	우회전실실전(10회)
TURN_RIGHT_12	line_turn_right_10	우회전실실전(12회)
TURN_RIGHT_15	line_turn_right_large	우회전실실전(15회)

예:

RECOVER_LEFT_TURN_RIGHT_4
        ↓
line_turn_right_2
        ↓
우회전실실전(4회)
RECOVER_RIGHT_TURN_RIGHT_15
        ↓
line_turn_right_large
        ↓
우회전실실전(15회)

RECOVER_LEFT, RECOVER_RIGHT 두 context 모두 동일한 physical turn mapping을 사용한다.

6. Motion Execution Safety

실제 로봇 동작 안정성을 위해 navigation planner와 SDK 사이에 별도의 execution layer를 둔다.

/navigation/motion_command
        ↓
motion_command_bridge_node
        ↓
/motion/executor/request
        ↓
sdk_motion_executor
        ↓
RobotMotionPlayer

적용된 주요 보호 기능:

Executor heartbeat
Startup pose gate
RUNNING motion lock
Request / command / event correlation
Duplicate RUNNING status suppression
Motion completion 확인
Joint settling 확인
TIMEOUT fault handling

Startup 시에는 지정된 초기 자세가 완료되기 전 navigation command 실행을 차단한다.

7. Vision During Motion

Line motion은 로봇이 완전히 멈춘 뒤 Vision을 새로 기다리는 방식이 아니라,
motion 후반부의 Vision을 미리 저장하여 다음 판단에 사용한다.

Line motion RUNNING
        ↓
예상 실행시간 80% 도달
        ↓
지정된 수의 line Vision frame 저장
        ↓
RUNNING 중 planner command 발행 차단
        ↓
Motion SUCCEEDED
        ↓
저장한 Vision frame을 line planner에 replay
        ↓
다음 10 Hz decision cycle
        ↓
다음 motion 즉시 결정

이를 통해 motion 사이의 불필요한 정지 시간을 줄인다.

8. TIMEOUT Recovery

정상 SUCCEEDED와 FAILED + TIMEOUT은 서로 다른 recovery 정책을 사용한다.

TIMEOUT 발생 시 motion 중 저장했던 Vision은 신뢰하지 않는다.

Motion RUNNING
        ↓
FAILED + TIMEOUT
        ↓
기존 buffered Vision 폐기
        ↓
새 motion 발행 차단
        ↓
Robot hold
        ↓
새로운 valid line Vision 10 frames 수집
        ↓
Line planner state reset
        ↓
10 frames 기반 재판단
        ↓
안전 조건 만족 시 다음 motion 실행

정상 완료 시에는 buffered Vision을 사용하고,
TIMEOUT 시에만 fresh Vision을 다시 수집한다.

9. Ball / Goal / Hurdle
Ball
거리 기반 tracking / control
Temporal confirmation
Distance-based forward action
좌우 정렬 판단
Pickup condition 판단
RGB detection과 depth validity 독립 처리
1.5m 이내에서 ball control 전환
공 중심 높이 0.035m와 카메라 높이 0.515m를 사용하고,
표면 Depth에 공 반지름 0.030m를 더한 뒤 피타고라스 정리를 적용한
ground_distance_m 기준 control / STRAIGHT 판단
raw Depth Z 0.50m 이하에서 pickup 트리거
steering_angle_deg 우선 조향 및 raw Depth Z 진단
raw Depth 0.60m 이내 공은 offset norm ±0.05를 중앙 데드밴드로 처리
공 전용 로봇 중심선은 화면 중앙 기준 +96px로 보정
Depth가 없을 때 제자리 정렬만 허용하고 전진 금지
Goal
거리 기반 tracking / control
Temporal confirmation
Goal alignment
Score condition 판단
Hurdle
Depth 기반 거리 판단
RGB candidate와 depth control-ready 독립 처리
ground_distance_m 진단
Production detection 기준 0.40 / 1.5m / 20중 12회 / miss 4 유지
GO geometry 및 5/7 confirmation 안전조건 유지
Line + hurdle fusion
Hurdle path reference 계산
Distance-based approach
좌우 alignment 판단

공 detector만 독립적으로 확인할 때는 full system을 종료한 뒤 다음 ONNX
진단 launch를 사용한다. 이 경로는 analyzer/planner/motion executor를 실행하지
않는다. Production full system은 `grab` class가 포함된 최신 `best.onnx`에서
생성한 `best.engine`을 사용한다. ONNX 파일은 모델 원본 및 CPU 진단용으로
유지하며, Jetson 실로봇 실행에서는 시작 시 엔진을 다시 빌드하지 않는다.

```bash
ros2 launch step ball_only_debug.launch.py
```

10. Jetson / Real Robot Configuration

Production 환경에서는 Jetson에서 실행한다.

주요 설정:

ROS 2 Humble
TensorRT YOLO backend
RealSense aligned depth
RobotMotionPlayer SDK
Dynamixel communication
Production motion alias table
Real robot motion JSON catalog

빌드:

cd ~/IRC/IRC-STEP/IRC_vision_latest

source /opt/ros/humble/setup.bash

colcon build \
  --symlink-install \
  --packages-select step mission_control irc_step_motion_executor

source install/setup.bash

빌드 확인:

ros2 pkg prefix step
ros2 pkg prefix mission_control
ros2 pkg prefix irc_step_motion_executor

각 package가 현재 workspace의 install/ 경로를 가리켜야 한다.

11. Current Validation

현재 integrated source 기준 다음 항목을 검증했다.

Vision / planner integration
Composite RECOVER action contract
Motion bridge
Motion alias
RobotMotionPlayer catalog
Forward motion mapping
Right-turn mapping
Mission control
80% line Vision capture
TIMEOUT recovery
Executor gate / heartbeat
Production TensorRT path

최근 right-turn bridge / gate / catalog 관련 테스트:

153 passed
git diff --check: PASS
12. Remaining Motion Mapping

일부 physical SDK motion은 아직 최종 확정 전이므로 임의 mapping하지 않는다.

현재 미확정 또는 추후 연결 대상:

TURN_LEFT
TURN_RIGHT
ALIGN_LEFT
ALIGN_RIGHT
SHOT
RETREAT_GOAL

또한 다음 motion들도 실제 동작 확인 후 필요 시 연결할 예정이다.

제자리 좌회전
제자리 우회전
미세걸음
후진
꽃게걸음
Left 75° Turn

Action contract상 75° 좌회전은:

*_TURN_LEFT_12

를 사용한다.

다만 현재 사용할 정확한 좌회전 12회 SDK motion은 아직 확정하지 않았으므로
10회 또는 15회 motion으로 임의 대체하지 않는다.

13. Integration Policy

본 프로젝트에서는 Vision/planner의 최신 판단 알고리즘을 적용하면서,
실제 STEP 로봇에서 검증된 production execution infrastructure를 유지한다.

즉 다음 두 영역을 분리한다.

Algorithm
Vision
Mission Planner
Mission Priority
Line / Ball / Goal / Hurdle Logic
Corner / Depth / Fusion
Recovery Policy
Production Robot Infrastructure
TensorRT
RealSense ROS wiring
Motion Command Bridge
SDK Executor
RobotMotionPlayer
Heartbeat
Startup Gate
RUNNING Lock
TIMEOUT Recovery
Dynamixel

알고리즘 변경으로 인해 실제 로봇 execution safety layer가 불필요하게 변경되지 않도록
두 영역의 contract를 명확히 유지하는 것을 원칙으로 한다.
