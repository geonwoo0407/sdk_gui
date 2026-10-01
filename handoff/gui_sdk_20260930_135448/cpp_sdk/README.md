# IRC 2족 휴머노이드 통합 모션 코드

## 최종 데이터 흐름

```text
수정된 sdk_gui.py
  → robot_motions.json (모든 시퀀스 1개 파일)
  → MotionLibrary가 JSON 로드
  → MotionPattern이 이름으로 walk/hurdle/pickup 선택
  → MotionCallback이 약 30ms마다 현재 목표각 계산
  → Dxl_Controller::SetPosition(Eigen radian vector)
  → Dxl::syncWriteTheta()
  → DYNAMIXEL 0~22번 모터
```

GUI가 이미 전체 export 기능으로 수정됐으므로 별도의 Python exporter는 사용하지 않습니다.

## 파일 역할

| 파일 | 역할 |
|---|---|
| `step_dynamixel.hpp/.cpp` | 기존 저수준 DYNAMIXEL SDK 통신(수정본) |
| `dynamixel_controller.hpp/.cpp` | 기존 Eigen 기반 위치·토크 Controller(수정본) |
| `motion_pattern.hpp/.cpp` | 전체 JSON 로드, 시퀀스 선택, 현재 목표각 계산 |
| `motion_callback.hpp/.cpp` | 30ms 전송 주기, 반복, 속도, 종료 관리 |
| `main.cpp` | Pattern → Callback → 기존 Controller 연결 |
| `CMakeLists.txt` | 전체 빌드 설정 |
| `robot_motions.example.json` | GUI 전체 export 형식 예시 |

## JSON 형식

```json
{
  "version": 1,
  "motions": [
    {
      "name": "walk",
      "max_seq_ms": 5000,
      "repeat_count": 2,
      "playback_speed": 1.0,
      "frames": [
        {
          "name": "right_leg_up",
          "start_ms": 0,
          "time_ms": 500,
          "angles": {"13": -20.0, "17": 40.0},
          "torques": {"13": true, "17": true}
        }
      ]
    }
  ]
}
```

## 기존 SDK 코드에서 수정한 핵심

1. GUI와 baudrate를 `4,000,000`으로 통일했습니다.
2. `Mode=3`이 잘못 Current Mode로 바뀌던 `SetPresentMode()` 조건을 수정했습니다.
3. Operating Mode 변경 전 전체 토크를 끕니다.
4. 생성자에서 자동 토크 ON을 제거했습니다. Main이 현재각을 읽은 다음 켭니다.
5. `Dxl_Controller::SetPosition()`이 `syncWriteTheta()`까지 호출합니다.
6. 23축 이동평균필터를 7축 하드코딩 대신 Eigen 행 연산으로 수정했습니다.
7. 관절속도를 getter에서 실제로 SyncRead한 뒤 반환합니다.
8. `dt_us`의 초 변환을 `1e-6`으로 수정했습니다.
9. SyncRead 통신 성공 및 데이터 사용 가능 여부를 확인합니다.
10. 헤더 include를 실제 파일명 `step_dynamixel.hpp`로 맞췄습니다.

## degree와 radian 연결

GUI JSON은 degree입니다.

```json
"13": -20.0
```

기존 `Dxl_Controller::SetPosition()`은 radian `VectorXd`를 받습니다. Main에서 변환합니다.

```cpp
desired_rad[motor_id] = degree * DEG2RAD;
controller.SetPosition(desired_rad);
```

모터 ID가 0~22이고 기존 `dxl_id`도 0~22 순서이므로 ID를 벡터 인덱스로 사용합니다.

## 안전한 실행 순서

```text
Dxl 생성 및 포트 연결
→ Position Mode 설정(토크 OFF)
→ 현재 23축 관절각 SyncRead
→ Pattern 초기각으로 설정
→ 전체 토크 ON
→ Callback 시작
→ 종료 후 토크 OFF
```

## 빌드

필요 패키지는 Eigen3와 ROBOTIS DYNAMIXEL SDK입니다.

```bash
cmake -S . -B build
cmake --build build -j
```

SDK가 자동 검색되지 않으면:

```bash
cmake -S . -B build \
  -DDYNAMIXEL_SDK_INCLUDE_DIR=/path/to/sdk/include \
  -DDYNAMIXEL_SDK_LIBRARY=/path/to/libdxl_x64_cpp.so
cmake --build build -j
```

실행:

```bash
./build/irc_robot robot_motions.json walk
./build/irc_robot robot_motions.json hurdle
./build/irc_robot robot_motions.json pickup_ball
```

## 하드웨어에서 반드시 확인할 것

- 실제 모터 baudrate가 4 Mbps인지 확인하십시오. 기존 파일은 4.5 Mbps였습니다.
- 모든 모델이 주소 64/116/132를 사용하는 Protocol 2.0 계열인지 확인하십시오.
- GUI에서 +각도로 움직이는 방향과 실제 모터의 +방향이 모든 관절에서 같은지 확인하십시오.
- 첫 시험은 로봇을 지지한 상태에서 한 프레임·낮은 동작 범위로 진행하십시오.
- 토크/전류 제어로 전환할 때는 Position Mode용 Pattern 재생과 동시에 실행하지 마십시오.
# ROS2 Motion Executor integration

`main.cpp`는 SDK/실기 확인용 터미널 프로그램입니다. 비전 노드가
`RobotMotionPlayer`를 직접 소유하지 않고, 별도 ROS2 Motion Executor가
`robot_control`을 링크합니다.

```cpp
#include "robot_motion_player.hpp"

int main() {
    irc_step::RobotMotionPlayer player("robot_motions.json");
    if (!player.initialize()) return 1;

    if (player.start("전진") != irc_step::StartResult::Accepted) return 1;
    while (player.running()) {
        player.update();
    }
    return player.succeeded() ? 0 : 1;
}
```

```cmake
add_executable(line_follower algorithm_main.cpp)
target_link_libraries(line_follower PRIVATE robot_control)
```

권장 실행 구조:

```text
Vision/Mission
  -> ROS2 Motion Executor
  -> RobotMotionPlayer::start(motion name)
  -> robot_motions.json frame
  -> Profile Acceleration + Profile Time + Goal Position
  -> DYNAMIXEL internal trajectory and PID
```

Player 상태는 `Idle`, `Running`, `Settling`, `Succeeded`, `Cancelled`,
`Failed`입니다. `update()`는 예외를 ROS timer 밖으로 던지지 않고
`result()`와 `lastError()`에 실패 원인을 저장합니다. 실행 중 새 `start()`는
`RejectedBusy`로 거부됩니다.

각 프레임의 지정 시간은 최소 이동시간입니다. 그 시간이 끝나도 실제
Present Position이 JSON 목표각의 허용오차 안에 들어오지 않으면 모션 시계를
해당 프레임 끝에 고정하고 다음 프레임을 전송하지 않습니다. 메타데이터의
`settle_timeout_ms`까지 도달하지 못하면 현재 위치를 홀드하고
`PositionTimeout`으로 실패합니다.

일반 ROS cancel은 `cancel()`을 호출합니다. 이 함수는 Present Position을
읽어 같은 위치를 새 Goal로 전송하므로 토크를 유지한 채 멈춥니다.
`emergencyStop()`은 전체 토크를 해제하므로 비상시에만 사용합니다.

`IMotionHardware`를 주입하는 생성자로 모터 없는 테스트를 작성할 수 있습니다.

```cpp
irc_step::MockMotionHardware hardware;
irc_step::RobotMotionPlayer player("robot_motions.json", hardware);
```

`test_robot_motion_player`는 정상 완료, busy 거부, cancel, 프레임 전송 실패,
최종 위치 timeout을 검증합니다.

Frames whose names contain `[착지]` use a time-based acceleration/deceleration
window equal to 10% of the actual frame duration, capped at 30 ms. Other frames
explicitly use 0 ms, so the landing setting never leaks into the next frame.

Small algorithm corrections can be applied before starting the next step:

```cpp
player.setJointCorrection(19, -0.5); // degrees
player.setJointCorrection(20,  0.5);
```

These joint corrections apply to that motor in every motion and frame. A
correction for only one named frame can be registered with both its motion and
frame name:

```cpp
player.setFrameCorrection("전진", "L_내딛기[착지]", 19, 1.0);
```

The final target is:

```text
JSON frame angle + global joint correction + motion/frame correction
```

Use `clearJointCorrections()` or `clearFrameCorrections()` to remove one kind,
and `clearCorrections()` to remove both. If the same frame name occurs more than
once in one motion, the named frame correction applies to every occurrence.
