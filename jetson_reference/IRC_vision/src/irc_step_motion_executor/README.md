# IRC STEP Motion Executor C++ Wrapper

## Production startup pose contract

`full_system_robot.launch.py`는 fail-closed startup gate를 기본 활성화한다.
`RobotMotionPlayer::initialize()`가 끝난 뒤 executor는 `motion_json_path`에서
`startup_pose_name`과 정확히 일치하는 유일한 frame을 찾아 motor ID 0..22의
유한한 각도 23개를 읽는다. 이어 production SDK 확장 API
`startPoseTransition(const std::vector<double>&, int64_t)`를 호출하고
`updateStartupPose()`가 `Succeeded`를 반환할 때까지 poll한다.

SDK 구현은 `DynamixelMotionHardware::initialize()`에서 저장한 Present Position을
시작 벡터로 사용하고, 기존 time-based/shortest-angle 처리와 hardware write를
재사용해야 한다. JSON/pose 오류, API 미지원, write 실패, settling timeout은
모두 AUTO gate를 영구 잠금 상태로 유지하며 STRAIGHT로 fallback하지 않는다.

이 패키지는 전달받은 C++ `RobotMotionPlayer` SDK를 향후 ROS 2에 연결하기 위한
독립 `ament_cmake` wrapper의 최소 골격이다. 현재 단계는
**catalog-only/mock-safe**이며 production motion executor가 아니다.

## 현재 동작

- `/motion/executor/request`, `/motion/executor/cancel`을
  `std_msgs/msg/String` JSON으로 구독한다.
- `/motion/executor/status`에 기존 JSON String 계약의 필드를 발행한다.
- `config/motion_aliases.yaml`을 읽어 motion alias 존재 여부만 검증한다.
- 알려진 alias의 start 요청도 `REJECTED` / `HARDWARE_NOT_READY`로 반환한다.
- 알려지지 않은 motion은 `REJECTED` / `INVALID_MOTION`으로 반환하며 다른
  motion으로 fallback하지 않는다.
- `RobotMotionPlayer`, Dynamixel backend 및 hardware 객체를 생성하지 않는다.

최신 SDK의 top-level motion 10개는 `sdk_*` alias로 각각 수동 시험할 수 있다.
production canonical alias는 현재 확인된 `forward`, `pickup`, `hurdle`만 둔다.
짧은 전진, shoot, 좌·우 회전 canonical alias는 실물 검증 전이므로 만들지
않았으며, 알 수 없는 motion으로 fallback하지 않는다.

## 단일 motion smoke test

executor만 안전한 simulated backend로 시작한다. 이 명령 자체로 motion request가
발행되지는 않는다.

```bash
ros2 launch irc_step_motion_executor sdk_motion_test.launch.py
```

다른 터미널에서 alias 하나를 반드시 지정해 정확히 한 번 요청한다. 기본 timeout은
15초이며 긴 반복 motion은 사용자가 명시적으로 늘린다.

```bash
ros2 run irc_step_motion_executor manual_motion_request.py --ros-args \
  -p motion_id:=sdk_pickup -p timeout_ms:=15000
```

helper는 자신의 `request_id`에 해당하는 `RUNNING`, `SUCCEEDED`, `FAILED`,
`CANCELLED`, `REJECTED`와 `error_code`/`message`를 출력한다. 전체 catalog를
순차 실행하거나 프로그램 시작만으로 자동 재생하지 않는다.

실물에서는 로봇 지지, 비상정지 수단, 관절 영점·방향·limit를 먼저 확인하고
SDK-enabled 빌드에서만 다음 값을 모두 명시한다. 아래 명령은 torque ON과 실제
동작을 허용하므로 반드시 개별 motion을 simulated로 먼저 검증해야 한다.

```bash
ros2 launch irc_step_motion_executor sdk_motion_test.launch.py \
  backend_type:=robot_motion_player \
  enable_robot_hardware:=true explicit_torque_approval:=true \
  motion_json_path:=/approved/sdk/robot_motions.json \
  robot_device_path:=/dev/ttyUSB0 robot_baud_rate:=4000000 \
  robot_motor_ids:="[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22]"
```

실행 중 취소는 기존 executor API만 사용한다. helper가 출력한 `request_id`와
동일한 값이어야 하며, 이것은 별도의 emergency stop을 의미하지 않는다.

```bash
ros2 topic pub --once /motion/executor/cancel std_msgs/msg/String \
  "{data: '{\"request_id\": 123456789}'}"
```

## SDK JSON/catalog 사전 검증

다음 검사는 serial, SDK, torque 또는 모터에 접근하지 않는다.

```bash
ros2 run irc_step_motion_executor validate_motion_catalog.py \
  /approved/sdk/robot_motions.json \
  --aliases install/irc_step_motion_executor/share/irc_step_motion_executor/config/motion_aliases.yaml
```

JSON의 top-level `motions`, 중복 motion 이름, 각 alias target, 모든 frame의
angles/torques motor ID `0..22`, 그리고 motion별 `completion`, `start_pose`,
`end_pose` 존재 여부를 검사한다.

## JSON 계약

request는 `action`(string), `command_id`(integer 또는 null),
`event_id`(integer 또는 null), `request_id`(integer),
`motion_id`(string)를 필수 key로 사용한다. 특히 `event_id: null`은 유효하지만
`event_id` key 누락은 `REJECTED` / `INVALID_REQUEST`이다.

status는 `status`, `action`, `command_id`, `event_id`, `request_id`,
`motion_id`, `error_code`, `message`를 항상 포함한다. catalog-only core는
정상 계약 요청의 원래 action과 correlation 값을 terminal `REJECTED`
status까지 보존한다.

## Hardware-independent executor core

`MotionBackend`는 실제 SDK 연결 전 단계의 하드웨어 독립 계약이다.
`start_motion()`, `cancel_motion()`, `poll_status()`만 정의하며 실제 SDK
함수명이나 생성자 정보를 포함하지 않는다. `SdkExecutorCore`는 기존 JSON
request를 검증하고 alias를 resolve한 뒤 이 인터페이스를 통해서만 motion
상태를 처리한다.

core 단위 검증은 test 전용 `FakeMotionBackend`만 사용한다. 실제
`RobotMotionPlayer` adapter와 hardware executor node는 아직 구현하지 않았으며,
core library는 `robot_control` target에 링크하지 않는다. 이 core의 빌드와
단위 테스트 통과는 실물 동작 또는 안전성 검증을 의미하지 않는다.

## Simulated executor node

`sdk_motion_executor`는 현재 `SimulatedMotionBackend`만 사용하는 하드웨어 없는
ROS 2 node이다. 실제 SDK backend는 아직 연결되어 있지 않다.

- subscribe: `/motion/executor/request`
- subscribe: `/motion/executor/cancel`
- publish: `/motion/executor/status`
- parameters: `backend_type`(기본 `simulated`),
  `enable_robot_hardware`(기본 `false`), `motion_json_path`(기본 빈 값),
  `motion_aliases_file`,
  `poll_period_ms`(기본 5; production trajectory control 200Hz),
  `running_polls`(기본 2),
  `settling_polls`(기본 1), `force_start_failure`, `force_backend_failure`

```bash
ros2 launch irc_step_motion_executor sdk_executor_simulated.launch.py
```

request는 즉시 status를 만들고 timer poll은 simulated 상태를
`RUNNING → SETTLING → SUCCEEDED`로 진행한다. 외부 status에서 `SETTLING`은
message에 settling을 표시한 `RUNNING`으로 발행된다. cancel은 다음 poll에서
`CANCELLED`가 된다. 전이는 sleep이나 장치 시간이 아닌 poll 횟수만 사용한다.

이 launch에는 실물 로봇이나 serial 장치를 연결하지 않는다. simulated
topic test 통과 역시 실물 안전성 검증을 의미하지 않는다.

`MotionBackend` 생성은 node가 아니라 factory가 담당한다. parameter를
생략하거나 `backend_type=simulated`를 지정하면 simulated backend만 생성되며
실제 motion을 실행하지 않는다. `robot_motion_player`를 요청하면 simulated로
fallback하지 않고 guard 순서에 따라 node 시작을 중단한다.

- `enable_robot_hardware=false`: `ROBOT_HARDWARE_NOT_ENABLED`
- hardware enabled + SDK OFF: `ROBOT_MOTION_PLAYER_BACKEND_NOT_BUILT`
- hardware enabled + SDK ON + runtime factory를 주입하지 않음:
  `ROBOT_MOTION_PLAYER_RUNTIME_NOT_CONFIGURED`

SDK adapter와 production runtime factory target은 존재하지만 node runtime
wiring은 아직 비활성 상태다. 지원하지 않는 backend 이름도 허용값을 포함한
`UNSUPPORTED_BACKEND_TYPE` 오류로 거부한다.

Real runtime은 아직 비활성이다. `backend_type=robot_motion_player`만
지정하면 `ROBOT_HARDWARE_NOT_ENABLED`로 안전하게 시작을 거부한다.
향후 real runtime에는 `enable_robot_hardware=true`와 명시적인
`motion_json_path`가 모두 필요하지만, 기본 node에는 production runtime factory가
연결되지 않았다. real 요청은
simulated로 fallback하지 않는다.

승인된 SDK 작업 복사본에서는 `Dxl` constructor의 port, baud rate, torque 및
operating-mode 접근을 제거하고 명시적인 `Dxl::Initialize()` 단계로 옮겼다.
SDK ON production factory는 `DynamixelMotionHardware`, 주입형 2인자
`RobotMotionPlayer`, borrowed API와 backend 객체를 생성하고 소유권만 구성한다.
검증된 ROS 2 runtime config의 device path, baud rate와 motor IDs는 SDK의
`DynamixelMotionHardwareConfig`로 변환되어 hardware 생성자까지 전달된다.
Production factory는 hardware policy가 모두 통과한 경우에만 owner를 만들고
`RobotMotionPlayer::initialize()`를 정확히 한 번 호출한다. 성공한 경우에만
backend를 반환하며 자동 simulated fallback은 없다. Production backend 선택은
실제 serial port, operating mode 및 torque 접근을 일으킬 수 있다.

Production factory의 `preflight()`는 진단 전용 경로다. fixed SDK hardware
profile을 검증한 뒤 torque-OFF 통신 및 모터 응답 확인만 수행한다.
motion JSON을 읽거나 `RobotMotionPlayer`를 생성하지 않으며, explicit torque
approval를 요구하지 않고 backend도 반환하지 않는다.
반면 `create()`와 그 내부 initialize 경로는 explicit torque approval가 반드시
필요하고, torque ON까지 성공한 경우에만 motion backend를 반환한다. 따라서
preflight 성공은 motion-ready 또는 모션 실행 가능 상태를 뜻하지 않는다.

SDK-enabled 빌드의 `sdk_hardware_preflight`는 일반
`sdk_motion_executor`와 분리된 일회성 diagnostics CLI다. ROS 2 node, topic 또는
지속 실행 loop를 만들지 않으며 `--device`, `--baud`, `--motor-ids`와
`--confirm-hardware-access PREFLIGHT_ONLY_TORQUE_OFF`를 모두 명시해야 한다.
이 확인 문자열은 하드웨어 포트 접근만 승인하며 torque ON 승인이 아니다.
CLI는 `explicit_torque_approval=false`를 유지하고 `initialize()`나 모션 실행을
호출하지 않는다. 성공 메시지가 출력되어도 torque는 OFF로 유지되며
motion-ready 상태가 아니다. 이 도구의 실제 실행은 로봇 지지·비상정지 등
하드웨어 점검 절차를 준비하고 사용자가 연결 상태를 판단한 뒤에만 수행해야 한다.

Production runtime 객체 생성과 hardware initialization 승인은 서로 다른
단계다. 초기화 policy의 기본값은 `enable_robot_hardware=false`이며, JSON 경로,
device path, baud rate, motor ID 목록·중복·범위와 explicit torque approval를
모두 검증한다. validation 성공은 설정 사전조건이 충족됐다는 뜻일 뿐 실제
모터 초기화나 torque enable을 의미하지 않는다. 실제 initialize 호출 구현은
후속 단계에서 별도의 hardware 승인과 함께 추가해야 한다.

`sdk_motion_executor`는 같은 ROS parameter 경로에서 다음 값을 읽어 runtime
config에 복사한다.

- `enable_robot_hardware`: `false`
- `robot_device_path`: 빈 문자열
- `robot_baud_rate`: `0`
- `robot_motor_ids`: 빈 정수 배열
- `explicit_torque_approval`: `false`
- `motion_json_path`: 빈 문자열

각 기본값은 hardware initialization을 허용하지 않는다. 기본 backend도 계속
`simulated`이며, simulated 선택에서는 이 parameter를 지정해도 hardware
policy나 initialize를 실행하지 않는다. ROS parameter는 bool, string, integer,
integer array 타입으로 선언되어 잘못된 타입은 node 구성 단계에서 거부된다.

현재 외부 SDK는 device `/dev/ttyUSB0`, baud rate `4000000`, motor ID `0..22`를
legacy 기본 profile로 사용한다. runtime config의 `device_path`, `baud_rate`,
`motor_ids`는 이 SDK profile과 정확히 일치하는지
확인하는 safety assertion이다. motor ID 순서는 무시하지만 `0..22` 전체 집합이
필요하다. 값은 SDK config 생성자에 전달되고 production factory의 명시적
initialize 단계에서 실제 hardware 접근에 사용된다.

현재 SDK API에서 runtime 생성자에 전달할 수 있는 설정은 motion JSON
경로와 hardware config다. protocol `2.0`은 외부 SDK 저수준 소스에 고정되어
있다. calibration, joint limit 및 실물 안전 정보가 검증되기
전에는 hardware를 활성화하면 안 된다. build/test 성공은 실물 안전성
검증을 의미하지 않는다.

`RobotMotionRuntime`은 SDK runtime 소유자를 backend보다 오래 유지한다.
멤버 소멸 순서상 backend가 먼저 파괴되고 runtime 소유자가 나중에
파괴되므로, 참조 기반 `RobotMotionPlayerBackend`의 dangling reference를
방지할 수 있다. 이 계약은 initialize 횟수를 기록하는 fake SDK ownership
test로 검증한다.

## RobotMotionPlayer backend adapter

`RobotMotionPlayerBackend`는
`IRC_STEP_ENABLE_ROBOT_MOTION_SDK=ON`일 때만 빌드되는 SDK opt-in target이다.
실제 `irc_step::RobotMotionPlayer`를 생성하거나 소유하지 않고, 외부에서
주입된 non-owning API wrapper를 `MotionBackend` 상태로 변환한다.

기본 `sdk_motion_executor` node는 계속 `SimulatedMotionBackend`를 사용한다.
production factory는 SDK ON library로만 제공되며 real backend wiring 및 hardware
launch는 아직 없다. `RobotMotionPlayerBackend`가 node에서 활성화되는 경로도
없다. 관절 방향,
영점, limit, 모션 거리 및 torque 안전 조건이 확인되기 전에는 실제 player를
생성·초기화하거나 이 adapter로 motion을 실행하지 않는다.

## 빌드 모드

기본 빌드는 SDK 경로 없이 catalog-only 모드로 동작한다.

```bash
colcon build --packages-select irc_step_motion_executor
```

SDK-enabled 빌드는 명시적으로 option을 켜는 경우에만 구성된다. SDK 소스의
라이선스, 복사 및 사용에 대한 조직 승인이 끝난 뒤에만 승인된 **외부 경로**를
지정해야 한다. 이 저장소에는 SDK를 복사하지 않으며 `vendor/robot_motion_sdk`
디렉터리나 tar.gz 해제 결과를 만들지 않는다.

```bash
colcon build --packages-select irc_step_motion_executor \
  --cmake-args \
  -DIRC_STEP_ENABLE_ROBOT_MOTION_SDK=ON \
  -DROBOT_MOTION_SDK_DIR=/approved/external/sdk/path
```

`IRC_STEP_ENABLE_ROBOT_MOTION_SDK`의 기본값은 `OFF`,
`ROBOT_MOTION_SDK_DIR`의 기본값은 빈 문자열이다. option이 `OFF`이면 SDK를
탐색하거나 `add_subdirectory()` 하지 않고 어떤 SDK library에도 링크하지
않는다.

option이 `ON`이면 지정한 외부 디렉터리에 다음 항목이 모두 있어야 한다.

- `CMakeLists.txt`
- `robot_motion_player.hpp`
- `robot_motion_player.cpp`
- `add_subdirectory()` 이후 생성되는 `robot_control` CMake target

경로 또는 항목이 없으면 configure 단계에서 명확히 실패하며 다른 경로로
fallback하지 않는다. SDK source는 현재 package build tree 아래의 별도 binary
directory에서 `add_subdirectory()`되고 원본 source는 수정하지 않는다.

SDK-enabled build는 compile probe와 production runtime ownership library를
빌드한다. production factory가 만드는 객체는 메모리상 ownership만 구성하며
hardware initialize나 port/torque 접근을 수행하지 않는다. 따라서 SDK-enabled
build 성공은 실제 로봇에서의 동작 가능성이나 안전성을 의미하지 않는다.

## Launch

```bash
ros2 launch irc_step_motion_executor catalog_only.launch.py
```

launch의 `hardware_enable` 기본값은 `false`, runtime SDK 경로 기본값은 빈
문자열이다. catalog-only node는 두 안전 조건을 강제하며 실제 장치, serial,
torque 또는 motor에 접근하지 않는다. 실물 motion 정보와 안전 조건이 확정되기
전에는 sdk-enabled build 결과를 hardware node로 확장하거나 실물에서 실행하지
않는다. 관절 방향·영점·limit·속도·전류/토크·비상정지 등 calibration 및 안전
정보가 확인되기 전에는 hardware node 사용을 금지한다.

기존 Python `motion_executor_node`, legacy adapter, SDK placeholder,
`full_system.launch.py`는 이 패키지와 별개이며 변경하거나 대체하지 않는다.
