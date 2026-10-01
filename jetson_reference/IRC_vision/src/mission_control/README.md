# mission_control

`mission_control`은 STEP의 비전 결과를 action으로 결정하고, 그 action을 C++
motion executor 요청으로 연결하는 ROS 2 패키지다. 모든 기능을 하나의
`main.cpp`로 합치지 않으며 `full_system.launch.py`를 안전한 개발 환경의 공식
실행 진입점으로 사용한다.

## 공식 실행 구조

```text
step/yolo26_detector
  -> step/unified_vision_node
  -> mission_control/motion_decision_node
  -> mission_control/motion_command_bridge_node
  -> irc_step_motion_executor/sdk_motion_executor
```

실제 모터 제어를 소유할 수 있는 노드는 `sdk_motion_executor` 하나뿐이다. 같은
ROS graph에서 Python executor나 다른 motor executor를 동시에 실행하면 안 된다.

## 모듈 책임

- `step`
  - 카메라 입력을 처리하고 비전 결과를 생성한다.
  - 모터와 RobotMotionPlayer SDK에 직접 접근하지 않는다.
- `mission_control/motion_decision_node`
  - 비전 결과를 바탕으로 수행할 action을 결정한다.
  - SDK `motion_id`를 직접 실행하지 않고 bridge에 action을 전달한다.
- `mission_control/motion_command_bridge_node`
  - 지원되는 action을 현재 catalog의 motion alias로 변환한다.
  - `request_id`, command/event ID와 executor status의 상관관계를 관리한다.
  - 모터나 외부 SDK에 직접 접근하지 않는다.
- `irc_step_motion_executor/sdk_motion_executor`
  - 실제 SDK 실행의 유일한 소유자다.
  - hardware enable과 torque approval 정책을 검증한다.
  - 실행 상태와 오류를 `/motion/executor/status`로 반환한다.

현재 bridge는 `STRAIGHT`를 `line_forward_6`, `APPROACH`를 `forward`, `PICKUP_NOW`를 `pickup`,
`GO`를 `hurdle`로만 연결한다. 짧은 전진·회전·SHOT처럼 실물 검증된 SDK motion이
없는 action은 executor로 전송하지 않고 `/motion/status`에 `UNSUPPORTED`를
발행한다.

## 공식 실행 명령

안전한 개발 실행은 simulated backend만 사용한다.

```bash
ros2 launch mission_control full_system.launch.py \
  enable_camera:=false display:=false
```

이 launch는 다음 값을 코드에서 고정한다.

- `backend_type=simulated`
- `enable_robot_hardware=false`
- `explicit_torque_approval=false`

향후 실물 실행의 진입점은 아래 파일이다. 아직 완성된 실물 실행 명령이나 실제
값 예시는 제공하지 않는다.

```text
ros2 launch mission_control full_system_robot.launch.py ...
```

`full_system_robot.launch.py`도 기본적으로 camera와 hardware를 비활성화하고,
torque 승인을 부여하지 않는다. 실물 실행 전에는 다음 항목을 모두 별도 안전
절차에서 검토하고 명시해야 한다.

- 승인된 SDK-enabled 빌드
- 기존 파일인 `motion_json_path`
- 승인된 `robot_device_path`
- 승인된 양의 `robot_baud_rate`
- 완전하고 중복 없는 `robot_motor_ids` 정수 배열
- `enable_robot_hardware=true`에 대한 별도 승인
- `explicit_torque_approval=true`에 대한 독립적인 torque 승인
- 로봇 지지, 비상정지 및 시뮬레이션 선행 검증

`enable_robot_hardware=false`인 기본 상태에서는 SDK runtime이나 Dynamixel 객체를
생성해서는 안 된다. 설정 validation 성공 또한 motion-ready를 의미하지 않는다.

## 토픽 인터페이스

| 토픽 | Publisher | Subscriber | 타입 | 역할 |
|---|---|---|---|---|
| `/navigation/motion_command` | `motion_decision_node` | `motion_command_bridge_node` | `std_msgs/msg/String` | action, command/event ID와 source 정보를 전달한다. |
| `/motion/executor/request` | `motion_command_bridge_node` | `sdk_motion_executor` | `std_msgs/msg/String` | 검증된 motion alias와 `request_id`를 JSON 요청으로 전달한다. |
| `/motion/executor/cancel` | 향후 cancel 요청자; 현재 공식 bridge는 미발행 | `sdk_motion_executor` | `std_msgs/msg/String` | 활성 `request_id`의 취소 요청을 전달한다. |
| `/motion/executor/status` | `sdk_motion_executor` | `motion_command_bridge_node` | `std_msgs/msg/String` | RUNNING 및 terminal 실행 상태와 오류를 반환한다. |
| `/motion/status` | `motion_command_bridge_node` | `motion_decision_node` | `std_msgs/msg/String` | navigation 계층에 action 상관관계가 포함된 상태를 반환한다. |

## Deprecated executable

다음 entry point는 이전 테스트와 전환 작업을 위해 아직 설치되지만 공식 실행
경로가 아니며 사용하면 안 된다. 파일과 entry point는 이번 단계에서 삭제하지
않는다.

- `motion_executor_node`: legacy Python executor
- `legacy_motion_executor_adapter`
- `legacy_motion_status_adapter`
- `sdk_motion_stub_node`

특히 Python `motion_executor_node`와 C++ `sdk_motion_executor`를 동시에 실행하면
동일 executor 토픽을 두 구현이 소유할 수 있으므로 금지한다. 공식 launch 두 개는
위 deprecated executable을 하나도 실행하지 않는다.

## 빌드

```bash
cd ~/IRC/IRC-STEP/IRC_vision_latest
source /opt/ros/humble/setup.bash
colcon build --packages-select step irc_step_motion_executor mission_control \
  --symlink-install
source install/setup.bash
```
