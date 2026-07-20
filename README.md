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
