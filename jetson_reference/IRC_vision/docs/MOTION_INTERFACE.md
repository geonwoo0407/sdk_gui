# Motion Interface 초안

## 목적

Vision / Mission에서 판단한 행동을 ROS2 Motion Executor가 안정적으로 실행하고,
SDK의 실행 상태와 오류를 상위 ROS Action 결과로 변환하기 위한 인터페이스를
정의한다.

이 문서는 기존 `/navigation/motion_command` 호환 경로와 향후 ROS Action 경로를
함께 설명한다. 실제 모션 데이터와 실물 로봇 검증 결과가 확정될 때까지는 초안으로
관리한다.

## 전체 구조

```text
Vision / Mission
        ↓
ROS2 Motion Executor
        ↓
RobotMotionPlayer
        ↓
IMotionHardware
        ↓
Dynamixel
```

- Vision / Mission은 상황을 판단하고 실행할 `motion_id`를 선택한다.
- ROS2 Motion Executor는 상위 요청, timeout, 재시도와 Action 결과 변환을 담당한다.
- RobotMotionPlayer는 JSON 모션을 읽고 프레임 단위로 재생한다.
- IMotionHardware는 모션 재생기와 실제 하드웨어 구현 사이의 추상화 계층이다.
- Dynamixel 구현은 실제 모터 통신과 하드웨어 오류를 처리한다.

## 역할 분담

### 알고리즘 담당

- ROS2 Motion Executor
- 미션 판단
- 명령 매핑
- 상위 timeout과 재시도
- SDK 상태를 ROS Action 결과로 변환
- SDK mock 통합 테스트

알고리즘 계층은 SDK 내부의 프레임 전환이나 복합 모션 중간 단계를 상위
Vision / Mission에 노출하지 않는다. 완료 결과에는 최초 요청의 `motion_id`를
유지한다.

### SDK 담당

- RobotMotionPlayer
- JSON 파싱과 모션 재생
- Dynamixel 연결 및 오류 처리
- 프레임 시간과 프로파일
- cancel 안전정지
- 최종 자세 도달 판정
- MockMotionHardware

## 기존 호환 구조

현재 `/navigation/motion_command` JSON과 `motion_command_bridge_node`는 새 ROS
Action 경로가 검증될 때까지 유지한다. 새 경로가 준비되었다는 이유만으로 기존
bridge를 즉시 제거하지 않는다.

현재 호환 요청 형식:

```json
{
  "command_id": 123,
  "action": "STRAIGHT",
  "angle_deg": 0.0
}
```

향후 권장 요청 형식:

```json
{
  "request_id": 1,
  "action": "WALK_FORWARD",
  "parameters": {
    "angle_deg": 0.0
  }
}
```

향후 권장 결과 형식:

```json
{
  "request_id": 1,
  "action": "WALK_FORWARD",
  "status": "SUCCEEDED",
  "error_code": "",
  "message": ""
}
```

## 지원 motion_id

- `home`
- `forward`
- `forward_short`
- `turn_left`
- `turn_right`
- `adjust_left`
- `adjust_right`
- `backward`
- `pick_ball`
- `shoot`
- `hurdle`
- `recover`
- `head_left`
- `head_right`
- `head_center`

지원 목록에 없는 `motion_id`는 RobotMotionPlayer를 호출하지 않고 거부한다.

## MotionExecutorCore

MotionExecutorCore는 ROS에 의존하지 않는 순수 상태 머신이다. mock 가능한
RobotMotionPlayer 객체를 생성자에 주입하며, ROS2 Motion Executor가 이 core를
감싸서 향후 ROS Action 요청과 결과를 연결한다.

### ExecutorState

- `IDLE`
- `STARTING`
- `RUNNING`
- `SETTLING`
- `SUCCEEDED`
- `FAILED`
- `CANCELLED`
- `TIMEOUT`

`STARTING`, `RUNNING`, `SETTLING`은 실행 중인 상태이며 `busy()`가 `true`이다.
`SUCCEEDED`, `FAILED`, `CANCELLED`, `TIMEOUT`은 terminal 상태이다.

### MotionExecutionResult

결과는 다음 필드를 가진다.

- `motion_id`: 최초 요청한 motion ID
- `final_status`: terminal `ExecutorState`
- `success`: 정상 완료 여부
- `error_code`: 문자열 오류 코드
- `message`: 진단용 설명

`MotionExecutionResult.error_code`는 항상 문자열이어야 한다. SDK enum 객체를
그대로 저장하지 않고 반드시 `enum.name`을 사용한다.

예:

```text
MotionError.COMMUNICATION_ERROR
→ "COMMUNICATION_ERROR"
```

### lifecycle

1. `start_motion(motion_id, timeout_ms)`를 호출한다.
2. 요청이 수락되고 `busy()`가 `true`인 동안 주기적으로 `tick(elapsed_ms)`를
   호출한다.
3. `terminal_result()`로 완료 결과를 확인한다.
4. 상위 계층이 결과를 처리한 뒤 `reset()`을 호출한다.
5. `reset()` 이후 새로운 모션을 받을 수 있다.

실행 중에는 `reset()`을 허용하지 않는다. 실행 중 새 `start_motion()` 요청도
거부하며, 현재 active motion과 기존 결과를 덮어쓰지 않는다.

`timeout_ms`가 0 이하인 요청, 지원하지 않는 `motion_id`, 준비되지 않은
하드웨어는 SDK 실행 전에 실패시킨다.

## SDK 계약

RobotMotionPlayer는 다음 메서드를 제공한다.

- `start(motion_id)`
- `update()`
- `running()`
- `status()`
- `succeeded()`
- `result()`
- `lastError()`
- `cancel()`
- `hardwareReady()`
- `currentMotion()`

### SDK 상태 매핑

| MotionStatus | ExecutorState |
| --- | --- |
| `RUNNING` | `RUNNING` |
| `SETTLING` | `SETTLING` |
| `SUCCEEDED` | `SUCCEEDED` |
| `FAILED` | `FAILED` |
| `CANCELLED` | `CANCELLED` |

`FAILED`에서는 `player.result()`의 `MotionError` enum 이름을
`error_code` 문자열로 저장한다. `player.lastError()`는 사람이 읽는 상세
오류 문자열이므로 `message`에 포함한다.

예:

```text
player.result() == MotionError.COMMUNICATION_ERROR
→ error_code = "COMMUNICATION_ERROR"

player.lastError() == "motor communication failed"
→ message에 포함
```

### StartResult 매핑

| StartResult | 처리 |
| --- | --- |
| `ACCEPTED` | active motion과 timeout을 저장하고 `STARTING`으로 전환 |
| `REJECTED_BUSY` | 요청 거부, 현재 실행 상태 유지 |
| `MOTION_NOT_FOUND` | `FAILED`, 모션 데이터 없음 |
| `HARDWARE_NOT_READY` | `FAILED`, 하드웨어 준비 오류 |
| `INVALID_MOTION` | `FAILED`, 유효하지 않은 모션 |

StartResult가 실패하면 해당 StartResult의 `enum.name`을 `error_code`
문자열로 사용한다. `player.lastError()`가 비어 있지 않으면 그 내용을
`message`에 포함한다.

## timeout 및 cancel 규칙

- 상위 timeout에 도달하면 `player.cancel()`을 정확히 한 번 호출한다.
- timeout 처리 이후에는 `player.update()`를 호출하지 않는다.
- 상위 timeout의 terminal 결과는 `TIMEOUT`이다.
- timeout `error_code`는 `"POSITION_TIMEOUT"`을 사용한다.
- cancel 결과가 `CANCELLED`이면 `CANCELLED` terminal 결과를 생성한다.
- `HOLD_FAILED`는 `FAILED` / `"CANCEL_FAILED"`로 변환한다.
- `HARDWARE_NOT_READY`는 `FAILED` / `"HARDWARE_NOT_READY"`로 변환한다.
- `NOT_RUNNING`은 안전하게 아무 동작도 하지 않고 기존 terminal result를
  덮어쓰지 않는다.

## terminal 결과 규칙

terminal 결과는 정확히 한 번만 생성한다. 한 번 생성된
`MotionExecutionResult`는 이후 `tick()`, `cancel()` 또는 중복 완료 신호로
변경하거나 교체하지 않는다.

상위 계층이 결과를 소비하기 전까지 `terminal_result()`는 같은 결과를 반환한다.
결과 소비가 끝난 뒤에만 `reset()`하여 `IDLE`로 돌아간다.

## Executor 공통 규칙

1. `IDLE`에서 유효한 명령을 받으면 실행을 시작한다.
2. 실행이 시작되면 `RUNNING` 상태를 상위 계층에 전달한다.
3. 실행 중 일반 명령은 중복 실행하지 않는다.
4. 정상 완료 시 `SUCCEEDED`를 한 번만 전달한다.
5. SDK 시작 실패나 통신 오류 시 `FAILED`를 전달한다.
6. 제한 시간을 넘으면 안전 정지를 요청하고 `TIMEOUT`을 전달한다.
7. 안전정지 요청은 일반 명령보다 우선한다.
8. 상태와 결과에는 최초 요청의 `motion_id`를 유지한다.
9. SDK 내부 모션 이름이나 중간 단계를 알고리즘 계층에 노출하지 않는다.

## 현재 mock 기반 MotionExecutorNode

현재 `motion_executor_node`는 실제 RobotMotionPlayer나 Dynamixel을 연결하지
않는다. 프로세스 내부에서 `MockRobotMotionPlayer`를 생성하여
MotionExecutorCore에 주입하는 검증용 ROS2 노드 뼈대이다.

기본 timer 주기는 10ms이며 ROS parameter `tick_period_ms`로 변경할 수 있다.
0 이하의 값이 설정되면 안전한 기본값 10ms를 사용한다.

### 요청 topic

- topic: `/motion/executor/request`
- type: `std_msgs/msg/String`

```json
{
  "request_id": 1,
  "command_id": 123,
  "action": "STRAIGHT",
  "motion_id": "forward",
  "timeout_ms": 5000
}
```

잘못된 JSON, 필수 필드 누락과 유효하지 않은 필드 타입은
`REJECTED` / `"INVALID_REQUEST"`로 응답한다. 지원하지 않는 `motion_id`는
`REJECTED` / `"INVALID_MOTION"`으로 응답한다. 실행 중 새 요청은
`REJECTED` / `"REJECTED_BUSY"`로 응답한다. `command_id`는 선택 필드이며
정수 또는 `null`이어야 한다. `action`도 선택 metadata이며, 있으면 비어 있지
않은 문자열이어야 한다.

### 상태 topic

- topic: `/motion/executor/status`
- type: `std_msgs/msg/String`

```json
{
  "request_id": 1,
  "command_id": 123,
  "action": "STRAIGHT",
  "motion_id": "forward",
  "status": "RUNNING",
  "error_code": "",
  "message": ""
}
```

수락 직후 `RUNNING`을 한 번 발행한다. 이후 terminal 결과
`SUCCEEDED`, `FAILED`, `CANCELLED`, `TIMEOUT` 중 하나를 정확히 한 번
발행하고 core를 `reset()`한다. terminal 결과까지 최초 `request_id`와
`command_id`, `action`, `motion_id`를 유지한다.

### cancel topic

- topic: `/motion/executor/cancel`
- type: `std_msgs/msg/String`

```json
{
  "request_id": 10
}
```

cancel의 `request_id`가 현재 실행 중인 최초 요청과 일치할 때만
`MotionExecutorCore.cancel()`을 호출한다. 불일치하면
`REJECTED` / `"REQUEST_ID_MISMATCH"`, 실행 중인 모션이 없으면
`REJECTED` / `"NOT_RUNNING"`을 발행한다. 성공하면 최초 `request_id`와
`motion_id`를 유지한 `CANCELLED`를 정확히 한 번 발행한다.

이 cancel topic은 mock 기반 노드의 개발/검증용 인터페이스이다. 실제 SDK
연동 시에는 ROS Action cancel 처리와 SDK 안전정지 계약에 맞게 교체될 수 있다.

### mock 실패 주입 parameter

- `mock_fail_after_updates` (기본값 `-1`)
  - `-1`: 실패를 주입하지 않는다.
  - `0` 이상: 설정한 update 횟수에 도달하면 mock을 `FAILED`로 전환한다.
- `mock_failure_code` (기본값 `"COMMUNICATION_ERROR"`)
  - `MotionError`의 enum 이름 문자열을 사용한다.

실패가 주입되면 `player.result()`는 지정된 `MotionError` enum을 반환하고,
`player.lastError()`는 사람이 읽을 수 있는 상세 문자열을 반환한다. Executor는
이를 `FAILED`, enum 이름 문자열 `error_code`, 상세 오류가 포함된 `message`로
변환한다. 한 번 `FAILED`가 된 mock은 이후 `SUCCEEDED`로 바뀌지 않는다.

이 parameter들은 실제 고장을 재현하지 않는 개발/검증 전용 기능이다. 실제
RobotMotionPlayer 연동 시 제거되거나 실제 SDK의 오류 주입 방식으로 교체될 수
있으며 Dynamixel 또는 실제 하드웨어에는 접근하지 않는다.

## Legacy Motion Executor Adapter

`legacy_motion_executor_adapter`는 현재 `/navigation/motion_command` JSON을
구독하여 mock 기반 `/motion/executor/request` 형식으로 변환하는 병렬 검증
경로이다.

```text
/navigation/motion_command
        ├─ motion_command_bridge_node → 기존 경로
        └─ legacy_motion_executor_adapter → mock Motion Executor 검증 경로
```

기존 `motion_command_bridge_node`는 삭제하거나 대체하지 않는다. 새 adapter는
mock Executor 통합이 검증되는 동안에만 병렬로 사용하며 실제 SDK 또는
Dynamixel에 접근하지 않는다.

### line action 정책

line planner의 action 의미는 다음과 같다.

- `STRAIGHT`: 직진
- `FINE_LEFT`, `FINE_RIGHT`: 미세 좌·우회전
- `LEFT`, `RIGHT`: 일반 좌·우회전
- `STOP`: 안전 정지

라인을 완전히 중앙에 붙이는 포물선 복귀나 재검출 뒤 반대 방향 추가 정렬
action은 현재 사용하지 않는다.

- 중앙 허용 범위(offset `±0.12`, heading `±4°`)에서는 진행 방향을
  유지하며 일반 주행으로 복귀한다.
- 중간 편차는 `FINE_LEFT`/`FINE_RIGHT`, 큰 offset
  (`|offset| > 0.28`)과 line 상실 복구는 `LEFT`/`RIGHT`를 사용한다.
- `FINE_LEFT`/`FINE_RIGHT`는 executor/SDK mapping이 아직 없다.
  planner는 action 이름과 판단 결과를 보존하되 `valid=False`,
  `reason=fine_turn_motion_unmapped`로 표시한다. gate와 adapter는 이를
  실행하지 않으며 기존 `turn_left`/`turn_right` mapping을 재사용하지 않는다.
- line quality 최솟값은 `0.35`, 완전 상실 기준은 2프레임이다.
- 완전 상실 시 마지막 유효 offset과 heading을 기억해 복구 방향을 정하고,
  최대 3회 후에는 반복 복구 대신 `STOP`한다.
- 허용 범위 안에서 재검출되면 복구 상태를 즉시 해제하고 추가 반대 정렬
  없이 `STRAIGHT`/일반 line planning을 재개한다.

offset은 analyzer의 `predicted_line_x - image_center_x` 정의를 따른다.
따라서 양수는 라인이 오른쪽에 있어 편차 크기에 따라
`FINE_RIGHT` 또는 `RIGHT`, 음수는 왼쪽에 있어 `FINE_LEFT` 또는 `LEFT`를
선택한다. heading도 가까운 점에서 먼 점으로 갈 때의 x 증가가 양수이므로
같은 방향 규칙을 사용한다.

### topic

- 구독: `/navigation/motion_command` (`std_msgs/msg/String`)
- 발행: `/motion/executor/request` (`std_msgs/msg/String`)

입력 예:

```json
{
  "command_id": 123,
  "action": "STRAIGHT",
  "angle_deg": 0.0
}
```

출력 예:

```json
{
  "request_id": 1,
  "command_id": 123,
  "action": "STRAIGHT",
  "motion_id": "forward",
  "timeout_ms": 5000
}
```

`command_id`는 mission 알고리즘이 `/navigation/motion_command`를 발행할 때
생성한 원래 명령 ID이다. `request_id`는 adapter가 Executor 실행 요청을
추적하기 위해 1부터 별도로 증가시키는 ID이다. 따라서 두 값은 서로 달라도
정상이며, adapter는 어느 한쪽으로 다른 쪽을 덮어쓰지 않는다.
원래 `action`도 Executor request metadata로 함께 전달한다.

`command_id`가 없는 구형 legacy 입력은 `null`로 Executor에 전달한다.
`angle_deg`는 호환성을 위해 파싱하지만 현재 Executor 요청에는 포함하지 않는다.
잘못된 JSON, 유효하지 않은 `action`, 지원하지 않는 `action`은 경고만 남기고
발행하지 않는다.

### action 매핑

아래 `motion_id`는 `LegacyMotionExecutorAdapter`가 mock Motion Executor에
보내는 추상 문자열이다. 현재 `sdk` player backend는
`SdkMotionPlayerPlaceholder`이며 `hardwareReady=False`이므로 이 표를 실제
STEP SDK 번호나 함수 매핑으로 해석하면 안 된다.

현재 프로젝트 단계의 대상은 이 SDK Executor 요청 계약이다. Dynamics 직접
제어 bridge는 아래 별도 절의 참고 계약이며 SDK backend 구현 근거로 사용하지
않는다.

| legacy action | motion_id |
| --- | --- |
| `STRAIGHT`, `APPROACH`, `GO` | `forward` |
| `SLOW_APPROACH`, `FINE_FORWARD_STEP` | `forward_short` |
| `APPROACH_GOAL`, `APPROACH_HURDLE` | `forward_short` |
| `ALIGN_LEFT` | `adjust_left` |
| `ALIGN_RIGHT` | `adjust_right` |
| `RETREAT_GOAL` | `backward` |
| `PICKUP_NOW` | `pick_ball` |
| `SHOT` | `shoot` |
| `TURN_LEFT`, `LEFT` | `turn_left` |
| `TURN_RIGHT`, `RIGHT` | `turn_right` |
| `CROSS_FINISH` | `hurdle` |

`FINE_LEFT`와 `FINE_RIGHT`는 위 매핑 표에 포함되지 않는다. 현재 명시적
미지원 action이며 mock/실제 Executor request를 생성하지 않는다.
`WAIT`, `STOP`, `BALL_LOST_STOP`, `GOAL_LOST_STOP`,
`WAIT_SCORE_CONFIRMATION`, `WAIT_GO_CONFIRMATION`은 판단 보류 또는 몸을
움직이지 않는 recovery 단계다. mission planner는 이 action을 `valid=false`로
정규화하므로 legacy adapter가 Executor request를 만들지 않는다.

`HEAD_SCAN_LEFT`, `HEAD_SCAN_RIGHT`, `HEAD_CENTER`는 ball recovery FSM의
시간·재검출 판단을 유지하기 위한 알고리즘 단계 이름이지만, 현재 head SDK
계약이 없어 역시 `valid=false`다. legacy adapter mapping과 새 motion ID를
추가하지 않는다.

goal recovery 내부 단계인 `RECOVER_GOAL_TURN_LEFT`와
`RECOVER_GOAL_TURN_RIGHT`는 mission planner 경계에서 각각 기존 `LEFT`와
`RIGHT`로 정규화한다. `target_heading_change_deg`와
`angular_speed_rad_s`는 `source_command`에 보존하며, adapter에는 새
motion ID를 추가하지 않고 기존 `turn_left`/`turn_right`를 사용한다.
`LEFT`/`RIGHT`는 기존 일반 회전 경로를 유지한다. legacy bridge에서도 새
Dynamics 번호를 추가하지 않고 기존 `TURN_LEFT`/`TURN_RIGHT` 처리의
action 별칭으로만 해석한다.

`enable_ball_lost_recovery`의 production 기본값은 `true`이며, 비활성화할
때만 parameter로 `false`를 지정한다.

`CROSS_FINISH` 매핑과 status correlation은 수동 호환 및 향후 확장을 위해
유지한다. 현재 자동 mission flow는 이 action을 발행하지 않으며,
`finish_enabled=True` 또는 수동 `WALK_TO_FINISH` phase만으로도 자동
발행되지 않는다. 마지막 미션 뒤에는 `AUTO`로 복귀해 라인 주행을 계속하고,
최종 정지는 사람이 직접 수행한다.

### timeout

- 일반 이동: 5000ms
- `pick_ball`: 10000ms
- `shoot`: 10000ms
- `hurdle`: 12000ms
- `recover`: 8000ms

### 실제 STEP Dynamics command 대조

이 절은 SDK Executor 계약과 별개인 Dynamics 직접 제어 경로의 참고 기록이다.
현재 SDK backend 판단과 action 지원 여부에는 사용하지 않는다.

실제 `/motion_command` 경로는 `MotionCommandBridgeNode`가
`robot_msgs/MotionCommand.command` 정수로 변환한다. 아래 번호는
`Dynamics/original_control/main.cpp`와 `Dynamics/safety_control/main.cpp`의
switch 및 callback 구현에서 확인한 값이다.

| planner action | bridge 결과 `(command, angle)` | Dynamics 근거 의미 | 구분 | 상태 |
| --- | --- | --- | --- | --- |
| `STOP` | 미매핑 | command 98이 stop/recovery mode로 정의되어 있으나 bridge busy·status 정책 미정 | 안전 정지 | 미확인 |
| `STRAIGHT` | `(1, 0)` | `forward_six` | 일반 | 확인 |
| `LEFT` | `(2, abs(target_heading_change_deg))` | 좌회전 | 일반 | 확인 |
| `RIGHT` | `(3, abs(target_heading_change_deg))` | 우회전 | 일반 | 확인 |
| `FINE_LEFT`, `FINE_RIGHT` | 미매핑 | 별도 확정 번호 없음 | 일반 | 의도적 미매핑, planner `valid=false` |
| `APPROACH` | `(12, 0)` | `1 step` | ball 일반 접근 | 확인 |
| `SLOW_APPROACH` | `(6, 0)` | `FORWARD_HALF` | ball 일반 접근 | 확인 |
| `FINE_FORWARD_STEP` | `(27, 0)` | `forward_2` | ball 일반 접근 | 확인 |
| `APPROACH_BALL` | 미매핑 | planner가 생성하지 않으며 실제 이름은 `APPROACH` 계열 | 호환 이름 없음 | 미매핑 |
| `APPROACH_HURDLE` | `(13, 0)` | `forward_half_six`, hurdle mode 진입 | 일반 접근 | 확인 |
| `APPROACH_GOAL` | `(6, 0)` | `FORWARD_HALF` | 일반 접근 | 확인 |
| `WAIT` | 미매핑 | 판단 보류이며 Dynamics 명령을 보내지 않음 | 비실행 | 확인 |
| `PICKUP_NOW` | `(9, 0)` | `Picking_Motion`, `Pick::Picking` | 특수 terminal | 확인 |
| `SHOT` | `(17, 0)` | shoot-ready에서 18→19 내부 sequence | 특수 terminal | 확인 |
| `GO` | `(14, 0)` | hurdle mode에서 14 반복 후 20 hurdle motion | 특수 terminal | 확인 |
| `CROSS_FINISH` | 실제 bridge 미매핑 | legacy mock에는 `hurdle` 문자열만 남음 | 수동 호환 | 실제 Dynamics 미확인 |

`STOP`과 `WAIT`는 다르다. `WAIT`는 `valid=false` 판단 보류이므로 bridge와
SDK를 실행하지 않는다. `STOP`은 planner의 안전 결과 또는 FINISHED 호환
결과지만 현재 bridge에는 command 98 매핑이 없다. command 98은 Dynamics에
존재하더라도 bridge가 실행 중 명령을 먼저 차단하므로, 긴급 정지로 연결하려면
busy 해제와 status correlation 정책을 함께 확정해야 한다.

`GO`는 legacy mock adapter에서 `motion_id=forward`로 표현되지만 실제
Dynamics 경로는 `STRAIGHT=1`과 다른 command 14이다. 원 action을
request/status에 보존하는 이유도 두 동작을 문자열 motion ID만으로
역매핑하지 않기 위해서다.

Dynamics `safety_control`은 최초 요청 command를 보존해 내부 17→18→19 또는
14→20 sequence가 끝나도 `/motion_end.command`에 17 또는 14를 발행한다.
bridge는 이 값이 active Dynamics command와 일치할 때만 `SUCCEEDED`를
발행한다. `STEP_REAL_ROBOT_COMMAND_GATE=ON` 빌드는 현재 1과 98 및 선택적
startup command만 허용하므로, 실제 빌드 옵션 확인 전에는 2·3·9·14·17
실행 가능성을 보장할 수 없다.

## mock 통합 검증 launch

`motion_executor_mock.launch.py`는 mock 기반 Motion Executor 통합 흐름만
검증하기 위한 launch 파일이다. 다음 두 노드만 실행 대상으로 포함한다.

- `mission_control / motion_executor_node`
- `mission_control / legacy_motion_executor_adapter`

이 launch는 `MockRobotMotionPlayer`만 사용한다. 실제 RobotMotionPlayer를
실행하지 않고 Dynamixel에 접근하지 않으며 실물 로봇을 움직이지 않는다.
실제 로봇 운용용 launch로 사용하면 안 된다.

기본 실행:

```bash
ros2 launch mission_control motion_executor_mock.launch.py
```

정상 검증용 예:

```bash
ros2 launch mission_control motion_executor_mock.launch.py \
  tick_period_ms:=500
```

실패 주입용 예:

```bash
ros2 launch mission_control motion_executor_mock.launch.py \
  tick_period_ms:=500 \
  mock_fail_after_updates:=2 \
  mock_failure_code:=COMMUNICATION_ERROR
```

launch argument 기본값:

- `player_backend`: `mock`
- `tick_period_ms`: `100`
- `mock_fail_after_updates`: `-1`
- `mock_failure_code`: `COMMUNICATION_ERROR`

### 기존 mission status 호환 흐름

mock 통합 검증에서는 다음 흐름으로 새 Executor를 기존 mission node에
연결한다.

```text
motion_decision_node
→ /navigation/motion_command (mission command_id)
→ legacy_motion_executor_adapter
→ /motion/executor/request (command_id + request_id + 원 action)
→ motion_executor_node
→ /motion/executor/status (command_id + request_id + 원 action)
→ legacy_motion_status_adapter
→ /motion/status (command_id + request_id + 원 action)
→ motion_decision_node
```

두 ID의 토픽별 의미는 다음과 같다.

| 토픽 | `command_id` | `request_id` |
| --- | --- | --- |
| `/navigation/motion_command` | mission이 만든 원 명령 ID | 없음 |
| `/motion/executor/request` | 원 명령 ID 보존, 구형 입력은 `null` | adapter가 만든 실행 요청 ID |
| `/motion/executor/status` | 해당 요청의 원 명령 ID | 해당 실행 요청 ID |
| `/motion/status` | Executor status의 원 명령 ID | Executor status의 실행 요청 ID |

Executor node는 수락한 `MotionRequest` 전체를 publication state에 보존한다.
따라서 `RUNNING`과 `SUCCEEDED`, `FAILED`, `TIMEOUT`, `CANCELLED` terminal
상태에 같은 두 ID와 원 `action`이 포함된다. 실행 중 새 요청의
`REJECTED_BUSY`를 포함한 요청 단위 거부 상태에도 거부된 요청의
`command_id`, `request_id`, `action`을 넣는다.
하드웨어 실행만 담당하는 `MotionExecutorCore`에는 mission metadata를
추가하지 않는다.

`legacy_motion_status_adapter`는 Executor의 `RUNNING`, `SUCCEEDED`,
`FAILED`, `CANCELLED`, `TIMEOUT`, `REJECTED` 값을 그대로 보존하면서 기존
subscriber가 사용하는 `status`, `action`, `command_id`, `event_id`,
`dynamics_command`, `motion_in_progress`, `reason` 필드를 만든다.
Executor의 `request_id`, `motion_id`, `error_code`, `message`도 추가 필드로
보존한다. optional `event_id`도 요청에서 status까지 보존하며, 구형 요청처럼
필드가 없을 때만 `null`이다. 잘못된 JSON, 누락되거나 알 수 없는 status는
경고 후 발행하지 않는다.

### 특수 action 왕복 및 status 상관관계

`PICKUP_NOW`, `SHOT`, `GO`, `CROSS_FINISH`는 `motion_id` 역매핑만으로
복원하지 않고 Executor request/status에 보존된 원 `action`을 우선 사용한다.
이 규칙이 필요한 이유는 `GO`와 일반 `STRAIGHT`가 모두
`motion_id="forward"`를 사용하기 때문이다.

- 원 action이 `GO`이면 status도 `GO`이다.
- 원 action이 `STRAIGHT`이면 status도 `STRAIGHT`이다.
- 원 action metadata가 없는 구형 Executor status만 기존
  `motion_id → canonical action` 매핑으로 fallback한다.
- `request_id`는 Executor 실행 식별자이고 `command_id`는 mission 원 명령
  식별자이며 서로 대체하지 않는다.

`motion_decision_node`는 특수 명령을 최초 발행할 때 active `action`,
`command_id`, `event_id`를 저장한다. 특수 `RUNNING`, `SUCCEEDED`, `FAILED`,
`TIMEOUT`은 다음 조건을 모두 만족할 때만 처리한다.

- status `command_id`가 정수로 존재한다.
- status `command_id`가 active special `command_id`와 같다.
- status `action`이 active special `action`과 같다.
- terminal status 전에 matching `RUNNING`으로 특수 모션 lock이 설정됐다.

다른 ID의 stale status, ID가 없는 status, 같은 ID이지만 action이 다른
status는 phase와 진행도를 변경하지 않는다. 올바른 terminal을 처리하면
active special metadata와 running lock을 해제하므로 같은 terminal이 다시
도착해도 중복 처리되지 않는다. 구형 Executor adapter 요청의
`event_id=null`은 허용하되 `command_id`와 action 일치를 필수로 사용한다.
새 요청은 mission의 `event_id`를 terminal status까지 그대로 보존한다.

Executor는 요청을 시작하기 전에 `REJECTED`를 반환할 수 있으므로 이 status는
matching `RUNNING` 없이도 특수 motion의 실패 terminal로 처리한다. 단,
`REJECTED`는 active `action`, `command_id`, `event_id`가 모두 정확히 일치해야
한다. matching 거부는 성공 진행도를 올리지 않고 해당 action의 기존
`FAILED`/`TIMEOUT` 복귀 phase를 적용한 뒤 lock을 해제한다. `error_code`와
`message`는 status adapter에서 보존되고 node 진단 log에도 포함된다. wrong,
stale 또는 duplicate `REJECTED`는 active state를 변경하지 않으며 즉시 자동
재시도하지 않는다.

이 adapter는 mock 검증을 위한 임시 호환 계층이다. 향후 mission node가
`/motion/executor/status`를 직접 사용하면 제거할 수 있다.

### Vision 입력부터 mission까지의 mock 통합

다음 명령은 실제 카메라나 검출 모델 없이 결정적인 Vision JSON을 기존
mission 판단 및 새 Motion Executor mock 경로에 연결한다.

```bash
ros2 launch mission_control mission_motion_mock.launch.py
```

검증 흐름은 다음과 같다.

```text
mock_mission_input_node
→ /vision/line_info
→ motion_decision_node
→ /navigation/motion_command
→ legacy_motion_executor_adapter
→ /motion/executor/request
→ motion_executor_node (player_backend=mock)
→ /motion/executor/status
→ legacy_motion_status_adapter
→ /motion/status
→ motion_decision_node
```

기본 `straight` scenario는 중앙에 정렬된 고품질 line 정보를 한 번 발행하여
기존 `motion_decision_node`가 기존 JSON 형식의 `STRAIGHT` 명령을 생성하게
한다. 이 launch에는 `motion_command_bridge_node`를 포함하지 않으므로
`/navigation/motion_command`의 실행 adapter는
`legacy_motion_executor_adapter` 하나뿐이다.

이 환경은 mock 통합 검증 전용이다. 실제 카메라, RealSense, YOLO 모델,
실제 RobotMotionPlayer SDK, Dynamixel 및 serial 장치를 실행하거나
접근하지 않는다.

scenario별 실행 예:

직진:

```bash
ros2 launch mission_control mission_motion_mock.launch.py \
  scenario:=straight \
  publish_delay_sec:=10.0
```

왼쪽 회전:

```bash
ros2 launch mission_control mission_motion_mock.launch.py \
  scenario:=turn_left \
  publish_delay_sec:=10.0 \
  publish_once:=false
```

오른쪽 회전:

```bash
ros2 launch mission_control mission_motion_mock.launch.py \
  scenario:=turn_right \
  publish_delay_sec:=10.0 \
  publish_once:=false
```

현재 코드에서 확인한 scenario별 결과는 다음과 같다.

| scenario | line heading | mission action | Executor 변환 |
|---|---:|---|---|
| `straight` | `0.0°` | `STRAIGHT` | `forward` |
| `turn_left` | `-10.0°` | `FINE_LEFT` (`valid=false`) | 미지원 |
| `turn_right` | `+10.0°` | `FINE_RIGHT` (`valid=false`) | 미지원 |

`straight`는 `STRAIGHT`가 `forward` 요청으로 변환되어 Executor의
`RUNNING → SUCCEEDED`가 `/motion/status`로 돌아온다.

회전 scenario는 중간 heading 편차이므로 각각 `FINE_LEFT`,
`FINE_RIGHT`를 선택하지만 현재 mapping이 없어 Executor 요청을 만들지
않는다. `WAIT`, 알 수 없는 action 및 명시적으로 `valid=false`인 명령도
Executor 요청을 만들지 않는다.

`test_mission_phase_flow.py`에는 ROS graph를 시작하지 않는 전체 코스 mock
시나리오도 있다. 실제 planner와 node callback을 직접 연결해 line의
`STRAIGHT`/미지원 `FINE_*`/큰 편차 `LEFT`·`RIGHT`, 공 접근과
`PICKUP_NOW`, 골대 접근·정렬과 `SHOT`, 허들 접근과 `GO`, 마지막
`AUTO` line 주행까지 연속 검증한다. 같은 시나리오에서 command ID가 다른
status, wrong action, duplicate terminal, 특수 motion lock, 일반 motion
gate 및 자동 `CROSS_FINISH`/`FINISHED` 비활성화도 확인한다.

별도 parameterized mock은 `PICKUP_NOW`, `SHOT`, `GO`의 `FAILED`와
`TIMEOUT` 뒤 `AUTO` line 판단 재개를 검증하며, line 상실 mock은 마지막
geometry 기반 회전, 재검출, 복구 횟수 제한을 확인한다. 이 테스트는 정책의
소프트웨어 연속성만 검증하며 카메라·지연·마찰·관절 응답을 포함한 실제 장치
검증은 아직 남아 있다.

### 일반 모션 명령 잠금

`STRAIGHT`, `LEFT`, `RIGHT`는 다음 상태 흐름을 따른다.

```text
IDLE
→ 일반 모션 명령 한 번 발행 및 즉시 잠금
→ RUNNING 동안 잠금 유지
→ SUCCEEDED / FAILED / TIMEOUT / CANCELLED / REJECTED
→ 잠금 해제
→ 새 Vision 입력 이후 다음 판단 허용
```

잠금 중에도 Vision 입력과 planner 계산은 계속되지만 추가
`/navigation/motion_command`는 발행하지 않는다. 따라서 같은 action뿐 아니라
실행 중 판단된 다른 일반 action도 새 Executor 요청으로 전달되지 않는다.
Executor의 `REJECTED_BUSY` 검사는 이 알고리즘 잠금 이후의 마지막 안전망이다.

노드 시작 직후에는 `/navigation/motion_command` publisher의
`get_subscription_count()`가 1 이상일 때만 command를 발행한다. subscriber가
없으면 planner 판단은 다음 timer 주기에 다시 수행하지만 `command_id`,
`event_id`, general motion gate, special motion metadata와 terminal latch는
변경하지 않는다. 연결 대기 warning과 연결 회복 info는 상태가 바뀔 때만
기록한다. 이 검사는 decision node의 최초 outbound command 유실만 방지하며
downstream topic에 별도 handshake나 durable QoS를 추가하지 않는다.

정상 완료, 실패, timeout, cancel 이후에는 terminal 상태보다 나중에 도착한
Vision 메시지가 최소 한 번 있어야 다음 일반 모션을 허용한다. `REJECTED`의
`error_code`가 `REJECTED_BUSY` 또는 `HARDWARE_NOT_READY`이면 일시적 거부로
분류하고, 새 Vision 입력 이후 같은 action을 최대 2회 재시도한다. 한도를
넘으면 action이 바뀔 때까지 억제한다. 이 한도는
`general_motion_transient_retry_limit` parameter로 설정할 수 있다.

`INVALID_REQUEST`, `INVALID_MOTION`, `MOTION_NOT_FOUND`, `INTERNAL_ERROR`와
알 수 없거나 누락된 error code는 fail-closed로 영구 거부 처리한다. 같은
action은 Vision generation이 증가해도 재전송하지 않으며, 다른 일반 action이
선택된 뒤에만 해당 action의 거부 및 retry 상태를 rearm한다.

일반 모션 잠금은 발행한 mission `command_id`를 함께 저장한다. 활성
`command_id`와 status `command_id`가 모두 있으면 두 값이 같은 상태만
처리하므로, 이전 요청의 늦은 `RUNNING`이나 `SUCCEEDED`가 현재 잠금을
변경하거나 해제하지 못한다. ID가 같더라도 `REJECTED` 외 terminal 상태는
해당 요청의 `RUNNING`을 먼저 보아야 한다.

구형 메시지 호환을 위해 status에 `command_id`가 없으면 기존 정책인 action
alias와 선행 `RUNNING`으로 상관관계를 판단한다. 이 fallback은 ID 기반만큼
강하지 않지만, 오래된 같은-action terminal이 단독으로 새 잠금을 해제하는
것은 막는다. `REJECTED`는 정상적으로 `RUNNING` 없이 반환될 수 있어 구형
fallback에서는 matching action으로 해제한다.

현재 Executor adapter 경로는 `command_id`, `event_id`, 원 action을 요청과
status에 함께 전달한다. 구형 입력에서 필드가 없으면 각각 `null`을 유지하며
새 식별자를 임의로 만들지 않는다.

`/vision/line_info`는 `std_msgs/msg/String` JSON이므로 ROS `Header`나
`header.stamp` 필드가 없다. `motion_decision_node`는 메시지를 받은 순간의
`time.monotonic()`을 freshness 시각으로 저장하며 기본 0.5초가 지나면
`no_fresh_detected_target`으로 전환한다. 따라서 지속 관찰용 실행에서는
`publish_once:=false`를 사용한다. mock node는 `publish_delay_sec` 후 첫
메시지를 발행한 다음 0.1초마다 같은 결정적 입력을 갱신하여 freshness
조건을 통과시킨다. `publish_once:=true`는 의도대로 한 번만 발행하므로
0.5초 후 WAIT로 돌아간다.

## Motion Player backend 선택

MotionExecutorNode는 `player_backend` parameter를 factory에 전달하여 player를
생성한다. 현재 지원하는 backend는 다음 두 가지이다.

- `mock`: 기본값. `MockRobotMotionPlayer`를 사용하며 기존 정상 흐름과 실패 주입
  검증을 지원한다.
- `sdk`: 실제 SDK가 아닌 `SdkMotionPlayerPlaceholder`를 사용한다.

`sdk` placeholder는 향후 연결 지점을 준비하기 위한 안전 객체이며 실제 모션을
실행하지 않는다. SDK, serial 또는 Dynamixel 모듈을 import하지 않고
`hardwareReady()`가 항상 `False`이다. 따라서 `player_backend:=sdk`로 실행해도
모션 시작 요청은 `HARDWARE_NOT_READY`로 거부되며 실물 로봇은 움직이지 않는다.

실제 SDK 연동 단계에서는 MotionExecutorCore나 ROS topic을 변경하지 않고
factory의 `sdk` 생성 구현만 실제 RobotMotionPlayer adapter로 교체할 예정이다.
그 전까지 기본 backend는 반드시 `mock`으로 유지한다.

## 아직 미확정인 항목

- 실제 전체 모션 JSON 데이터
- 모션별 실행시간
- 실제 시작/종료 자세
- 실물 로봇 검증 결과
