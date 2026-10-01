# RobotMotionPlayer SDK 인수인계

이 문서는 이 폴더의 현재 소스에서 직접 확인한 내용만 기록한다.
기구/배선 정보가 없어 확인할 수 없는 값은 추정하지 않았다.

## 1. 빌드 구성

이 프로젝트는 현재 ROS2/ament 패키지가 아니라 일반 CMake 프로젝트다.
따라서 `package.xml`은 존재하지 않는다.

생성되는 주요 target:

- `motion_core`: JSON motion 및 legacy callback
- `motion_player_core`: 비동기 `RobotMotionPlayer` 상태 머신
- `robot_control`: 실제 Dynamixel backend를 포함하는 library
- `irc_robot`: 터미널 실행용 executable
- `test_motion_pattern`, `test_motion_callback`, `test_robot_motion_player`

빌드:

```bash
cmake -S . -B build
cmake --build build -j
ctest --test-dir build --output-on-failure
```

필요한 외부 의존성:

- C++20 compiler
- Eigen3
- ROBOTIS Dynamixel SDK (`dynamixel_sdk/dynamixel_sdk.h` 및 library)
- Threads

## 2. ROS2 구성 여부

현재 `RobotMotionPlayer` C++ 클래스만 있고 ROS2 wrapper node는 없다.
`rclcpp`, ROS message/service, publisher/subscriber, launch 파일 및
`package.xml`도 없다.

`RobotMotionPlayer`를 실제 생성하는 위치는 `main.cpp`이며 executable은
`irc_robot`이다. `robot_control`은 executable이 아니라 library target이다.

`irc_robot` 실행:

```bash
./build/irc_robot robot_motions.json
./build/irc_robot robot_motions.json 전진
```

첫 번째 형식은 터미널에서 motion 이름을 입력받는다. ROS topic/service/status
topic은 제공하지 않는다.

## 3. Dynamixel 설정

소스에서 확인되는 값:

- device: `/dev/ttyUSB0`
- baud rate: `4,000,000`
- protocol: `2.0`
- motor ID: `0`부터 `22`까지, 총 23개
- position mode: control table address 11에 값 3
- torque enable: address 64, OFF=0, ON=1
- Drive Mode: address 10의 bit 2 (`0x04`)를 켜 time-based profile 사용
- 최소 firmware: V42
- Profile Acceleration: address 108
- Profile Time: address 112 (`Profile Velocity` register의 time-based 의미)
- Goal Position: address 116
- Present Position: address 132
- profile time 범위: 1~32737 ms
- acceleration time: frame duration의 절반 이하

초기화 순서:

1. 포트 open 및 baud rate 설정
2. Position Control Mode 설정
3. torque OFF
4. 모든 motor의 firmware와 time-based Drive Mode 설정/검증
5. Profile Acceleration/Profile Time을 0으로 초기화
6. 전체 Present Position 읽기
7. torque ON

각 frame 전송 순서는 Profile Acceleration, Profile Time, Goal Position이다.

현재 소스에 없어서 별도 제공이 필요한 정보:

- ID별 정확한 Dynamixel 모델명
- 관절 이름과 motor ID 매핑
- 관절별 방향 부호
- 기구 영점 및 실제 calibration offset
- 관절별 최소/최대 limit
- USB udev rule 또는 사용자/group 권한 설정

`zero_manual_offset`은 현재 23축 모두 0이다. 별도 joint limit 검사는 없고,
최종 raw Goal Position만 0~4095로 clamp한다. 모델 혼용 여부도 코드에
기록되어 있지 않다. 이 세 항목은 실물 연결 전에 반드시 확정해야 한다.

## 4. robot_motions.json

최상위 형식:

```json
{
  "version": 1,
  "motions": [
    {
      "name": "motion name",
      "max_seq_ms": 5000,
      "repeat_count": 1,
      "playback_speed": 1.0,
      "repeatable": true,
      "start_pose": "optional metadata",
      "end_pose": "optional metadata",
      "completion": {
        "position_tolerance_deg": 2.0,
        "settle_duration_ms": 80,
        "settle_timeout_ms": 3000
      },
      "frames": [
        {
          "name": "frame name",
          "start_ms": 0,
          "time_ms": 500,
          "angles": {"0": 0.0},
          "torques": {"0": true}
        }
      ]
    }
  ]
}
```

- `start_ms`, `time_ms`, `max_seq_ms`, completion 시간: millisecond
- `angles`: degree. 손으로 읽은 소수 원본을 GUI 저장·재생·JSON 내보내기에
  그대로 사용한다. 관절 숫자 입력칸과 슬라이더는 반올림한 정수로 표시하지만
  소수 원본은 유지한다. 입력칸에서 수정 없이 Enter를 누르거나 포커스를
  옮겨도 원본값은 보존하며, 직접 입력한 소수값도 사용할 수 있다.
  화살표는 소수 부분을 유지하며 1도씩 조절한다. 모터 미연결 상태에서도
  자세 편집은 가능하고, 숫자칸의 모터 전송은 연결·토크 ON 상태에서 Enter로 한다.
  SDK도 `double`로 읽는다.
  이전에 정수로 내보낸 JSON은 GUI를 재시작한 뒤 원본 시퀀스에서 다시
  내보내야 한다. 이미 정수가 된 JSON만으로는 소수 원본을 복원할 수 없다.
- angle/torque key: motor ID 문자열
- `playback_speed`: timeline 배속, 0.1~5.0으로 clamp
- GUI 시퀀스 조합은 각 원본 모션의 저장 배속을 유지한다. 조합의 `전체 배속`
  1.00x가 원본 속도이며, 편집 타임라인 시간값과 실제 재생 소요시간은 별도로
  표시한다. 조합 저장·JSON 내보내기는 원본 배속을 프레임 시작/끝 시각에
  한 번 반영하고, 조합 전체 배속만 JSON의 `playback_speed`에 남긴다.
- GUI 조합 프레임의 `playback_cycle_end`는 원본 1회 반복의 마지막 프레임을
  표시한다. GUI는 해당 프레임의 최종 목표 전송과 피드백 처리가 끝난 뒤
  다음 반복을 시작해 모션 제작의 단독 반복과 같은 통신 지연을 반영한다.
  이 표시는 GUI 저장·재로드에도 유지되며 SDK 재생은 기존 시간 필드를 사용한다.
- `sdk_gui_timing_repairs.json`은 확인된 옛 조합의 시간 보정 기록이다.
  재시작할 때 시퀀스 ID·배속·반복·프레임 내용 지문이 모두 일치하는 경우만
  적용하므로, 이미 고친 데이터나 사용자가 새로 수정한 프레임은 재보정하지 않는다.
- `repeat_count`: 최소 1로 clamp
- frame은 `start_ms` 기준으로 정렬
- frame overlap, 음수 start, 0 이하 duration, frame보다 짧은
  `max_seq_ms`는 validation 오류
- 같은 motion 이름은 오류
- 관절 궤적이 single-turn -180~180도 경계를 넘으면 오류
- `RobotMotionPlayer`에서는 frame마다 Dynamixel 내부 time-based profile로
  이동한다. `[착지]` frame만 실제 duration의 10%, 최대 30 ms acceleration을
  사용하고 나머지는 0 ms다.
- 해당 frame에 없는 관절은 Goal을 새로 보내지 않으며 기존 목표/자세를
  유지한다.
- legacy `MotionPattern::sample()` 경로는 frame 목표 사이를 제한된
  quintic Hermite 궤적으로 보간하지만, 실제 `RobotMotionPlayer` backend의
  frame 전송 경로와는 별개다.

## 5. 현재 motion 데이터

JSON에서 기계적으로 확인되는 정보:

| 이름 | frame 수 | 1회 timeline | 반복 | 배속 | 예상 명목 시간 | 시작/끝 frame |
|---|---:|---:|---:|---:|---:|---|
| 기본 | 1 | 500 ms | 1 | 1.0 | 500 ms | 기본자세 / 기본자세 |
| 첫발 | 3 | 640 ms | 1 | 1.0 | 640 ms | 기본자세 / 오뒤 |
| 전진 | 5 | 503 ms | 1 | 1.0 | 503 ms | 오뒤 / 오뒤 |
| 전신 최신1 | 4 | 427 ms | 5 | 0.9 | 약 2373 ms | 오들 / 오뒤 |
| 전진 최신2 | 4 | 420 ms | 3 | 0.9 | 1400 ms | 오들 / 오뒤 |
| 전진 가장 좋음 | 4 | 400 ms | 10 | 1.0 | 4000 ms | 오들2 / 오뒤 |

이름과 frame 각도만으로 실제 이동 거리, 첫발/중간/정지 역할, 실물 안정성을
확정할 수 없다. 거리 calibration이나 시험 결과도 저장소에 없다.

이름만 기준으로 한 임시 후보는 `forward=전진`, `forward_short=첫발`이지만
이는 검증된 mapping이 아니다. `전진 가장 좋음`은 이름상 안정 후보일 뿐
10회 반복하는 4초 motion이므로 mission의 단일 `forward` 명령에 바로
연결하면 안 된다. 실제 주행 시험과 SDK/모션 제작자의 확인이 필요하다.

## 6. Lifecycle

- `initialize()` 실패 후 재호출은 가능하다. 자동 retry/reconnect는 없다.
- `shutdown()`은 실행 중이면 현재 위치 hold를 시도한 뒤 torque를 끈다.
- `RobotMotionPlayer` destructor는 `shutdown()`을 호출한다.
- 저수준 `Dxl` destructor도 torque OFF 후 포트를 닫는다.
- 통신 단절 후 자동 재연결은 지원하지 않는다.
- 성공한 `emergencyStop()`은 torque를 끄고 initialized 상태를 해제한다.
  이후 `initialize()`가 성공하면 다시 `start()`할 수 있다.
- `cancel()` 성공 직후 다른 `start()`가 가능하다.
- `FAILED` 전용 reset API는 없다. 하드웨어가 여전히 ready라면 다음 유효한
  `start()`가 상태를 `RUNNING`으로 바꾼다. 하드웨어가 ready가 아니면
  원인을 복구하고 `initialize()`를 다시 호출해야 한다.

### 연속 모션 예약 API

`start()`는 기존처럼 `Running`/`Settling` 상태에서 `RejectedBusy`를
반환한다. busy 검사를 우회하지 말고 다음 모션을 미리 예약할 때는
다음 API를 사용한다.

```cpp
QueueResult queueNext(std::string_view motion_name);
bool hasQueuedMotion() const;
std::string_view queuedMotion() const;
void clearQueuedMotion();
std::uint64_t completionSequence() const;
std::string_view lastCompletedMotion() const;
```

- 대기열은 최대 1개다. 이미 예약이 있으면 `RejectedQueueFull`이다.
- 같은 모션을 연결할 때는 현재 모션이 `repeatable=true`여야 한다.
- 서로 다른 모션은 현재 `end_pose`와 다음 `start_pose`가 같아야 한다.
- 호환되지 않으면 `IncompatibleTransition`을 반환하며 현재 모션은
  계속 실행된다.
- 예약된 모션은 현재 모션의 마지막 반복 타임라인 경계에서
  `Settling`을 거치지 않고 시작한다.
- 경계 전환 시 이전 모션의 frame gate, 최종 Goal, settling timer를
  모두 초기화한다.
- `completionSequence()`는 무정착 경계 전환과 최종 정상 완료마다
  1씩 증가한다. Executor는 이 값의 변화와 `lastCompletedMotion()`을
  이용해 이전 ROS 명령을 완료 처리할 수 있다.

예시:

```cpp
if (player.start("forward") == StartResult::Accepted) {
    // 현재 forward가 끝나기 전에 비전 판단 결과를 1개 예약
    const auto queued = player.queueNext("forward");
}

while (player.running()) {
    player.update();
}
```

## 7. 확인되지 않은 실물 정보

다음 정보는 이 소스만으로 답할 수 없으며 SDK/기구 담당자의 별도 회신이
필요하다.

- ID별 모델과 관절 매핑
- 방향 부호와 calibrated zero offset
- 안전 joint limit
- 각 motion의 실제 이동 거리와 역할
- 가장 안정적인 전진 motion의 실기 결과
- USB 권한/udev 배포 설정
