# Jetson 실행기를 최신 GUI와 같은 모터 제어 방식으로 맞춰 주세요

첨부한 `jetson_gui_sync_20261027.zip`을 기준으로 Jetson의 실제 모션 실행기를
수정해 주세요. 최근 PC GUI에 **연속 보간**과 **5번 모터 제외 옵션**을 추가했습니다.
같은 모션을 선택했을 때 초기 자세, 관절별 목표 위치, 시간·반복 처리,
모터 명령 대상이 PC GUI와 일치하도록 구현하는 작업입니다.

설명이나 계획으로 끝내지 말고 실제 실행 경로의 코드를 수정하고 빌드·테스트까지
완료해 주세요. 변경 전 Jetson 소스·설정·모션 JSON과 빌드 정보를 복원 가능하게
백업하고 위치를 기록하세요. 기존 작업 내용을 보존해 주세요.

## 1. 기준 파일과 실제 실행 경로

- 압축본의 `reference/sdk_gui.py`와 `reference/continuous_trajectory.py`가
  동작의 기준입니다. `MANIFEST.json`의 SHA256으로 받은 파일을 확인하세요.
- `CONTINUOUS_MOTION.md`, `diagnostics/motor5_exclusion.md`에 이번 변경 설명이
  있습니다. 과거 handoff 문서의 ‘모든 프레임 코사인 보간’ 설명은 최신 기본값이
  아닙니다. 현재 호출 경로와 첨부 소스를 우선하세요.
- GUI 실제 경로는 `start_motion_playback → anim_step → scrub_timeline →
  timeline_frame_angles → ContinuousTrajectory.sample_frame → write_goal_positions`
  입니다. `command_time_profile_frame()`은 현재 GUI 재생에서 호출하지 않습니다.
- Jetson의 launch/노드 → 모션 선택·alias → runtime JSON → SDK 플레이어 →
  하드웨어 백엔드 → 실제 버스 전송까지 추적하세요. 실제 import/link된 SDK 위치,
  실행 바이너리, 로드한 JSON의 절대 경로와 해시를 기록하세요.
- `RobotMotionPlayer`, `DynamixelMotionHardware`, `irc_step_motion_executor`,
  `robot_motion_player_backend`, `sdk_executor_core` 등은 탐색할 후보 이름입니다.
  오래된 참조본을 현재 Jetson 코드라고 가정하지 말고 실제 파일을 확인하세요.
- JSON 복사나 launch 주기 변경만으로 완료했다고 하지 마세요. Python 기준을
  C++로 이식하거나 기존 실행기에 통합해 실제 목표각 생성 경로에 연결해야 합니다.

## 2. 최신 기본 보간: 연속 모드

GUI의 `continuous_motion_enabled` 기본값은 `True`입니다. Jetson에도 ON/OFF를
선택할 runtime 설정을 만들고 기본값을 ON으로 맞추세요. OFF는 기존 코사인 방식으로
돌아가야 합니다. 런타임에서 실제 적용 중인 모드를 로그로 확인할 수 있어야 합니다.

`ContinuousTrajectory`의 다음 동작을 그대로 옮기세요.

1. 프레임을 시작 시각으로 정렬하고 첫 프레임은 재생 직전 읽은 실제 관절각에서
   시작합니다. 이후는 이전 프레임들의 목표각을 누적합니다. 누락 관절을 0도로
   초기화하거나 매 tick 실측각으로 시작점을 다시 잡지 않습니다.
2. 관절마다 붙어 있는 두 프레임의 이동 방향이 같으면 경계 속도를 연결합니다.
   양쪽 프레임에 해당 관절이 있어야 하며, 양쪽 평균속도의 절댓값 중 작은 값을
   같은 부호로 사용합니다. 경계 가속도는 양쪽 모두 0입니다.
3. 방향 반전, 이동량 0, 실제 공백, 누락 관절을 사이에 둔 구간, 시퀀스 시작·끝,
   `playback_cycle_end=true`인 프레임 끝, `stop_at_end=true`인 프레임 끝에서는
   공유 속도를 0으로 둡니다. 이름에 `[착지]`가 있는 프레임은 양 끝에서 0입니다.
   인코더 래핑 경계를 넘는 구간은 기준 소스처럼 공유 속도 연결에서 제외합니다.
4. 최단각 변화량은 Python과 같은 `((target-start+180) % 360)-180`입니다.
   C++의 음수 나머지와 Python `%`의 차이를 처리하세요.
5. 일반화한 식은 아래와 같습니다. 시간 단위는 전체 배속을 적용하기 전의
   **실제 playback ms**입니다. 조합 원본 배속이 있으면 시간 매핑을 먼저 반영합니다.

```text
h = mapped_end_ms - mapped_start_ms
delta = shortest_angle_delta(q0, q1)
m0 = start_velocity * h
m1 = end_velocity * h
u = clamp((mapped_now_ms - mapped_start_ms) / h, 0, 1)
c3 = 10*delta - 6*m0 - 4*m1
c4 = -15*delta + 8*m0 + 7*m1
c5 = 6*delta - 3*m0 - 3*m1
q(u) = q0 + m0*u + u^3*(c3 + u*(c4 + u*c5))
```

구간 시작·끝에서는 기준 소스처럼 시작각·저장된 최종각을 정확히 반환하세요.
단순 선형보간, 일반 cubic spline, 매 tick 저역통과 필터, 모든 관절에 최소속도
부여 등으로 대체하지 마세요. 각 관절의 두 목표 사이에서 넘침 없이 움직이도록
기준 소스와 동일한 계수와 경계 속도 제한을 사용하세요.

연속 모드에서는 발 들기도 **프레임 시간 전체**를 사용합니다.
`lift_early_arrival=true`가 있어도 이 모드에는 80% 조기 도착을 중복 적용하지 않습니다.

연속 모드 OFF일 때만 기존 `frame_motion_progress()`를 따릅니다.
이때 이름에 `발들/들기/오들/왼들` 중 하나가 있고 `lift_early_arrival=true`이면
앞 80%에 목표 명령에 도달하고 뒤 20%는 유지합니다. 그 외는 전체 시간을 쓰는
half-cosine 보간입니다. GUI의 라이브러리 단일 프레임 적용은 여전히 이 기존
코사인 경로이므로, 보행 시퀀스와 단일 프레임 적용을 혼동하지 마세요.

궤적 전체 준비는 제어 주기를 시작하기 전에 끝내세요. 위치·속도·가속도의
연결은 명령 궤적의 성질이며 실제 모터 추종이나 물리적 최대성능을 보장하지 않습니다.
정지 구간에서는 기존 코사인보다 순간 최고속도가 커질 수 있으므로 배속·각도·PID를
임의로 바꿔 결과를 맞추지 말고 차이를 보고하세요.

## 3. 모터 설정과 실제 송신

- Protocol **2.0**, baud **4,000,000**, 논리 ID **0~22**입니다. 장치 경로는
  실제 연결과 기존 runtime 설정을 확인하세요. GUI 기본 후보는 `/dev/ttyUSB0`입니다.
- 위치 목표는 `Goal Position(116, 4 bytes)`에 `GroupSyncWrite`로 묶어 보냅니다.
  위치 읽기는 `Present Position(132, 4 bytes)`의 `GroupSyncRead`입니다.
- GUI는 firmware V42 이상과 Drive Mode bit 2(`0x04`)를 준비하지만, 실제 재생은
  Profile Acceleration(108)과 Profile Velocity(112)를 **모두 0**으로 둔 뒤
  중간 목표각을 계속 보냅니다. 최종각·내부 이동시간을 프레임당 한 번 보내는
  방식이나, 매 tick 내부 프로파일 시간을 재설정하는 방식으로 바꾸지 마세요.
- 새 재생 전 현재 위치를 읽고 토크 ON 전에 현재 위치를 Goal로 선등록하는
  절차를 유지하세요. EEPROM 설정이 필요한 경우 현재 토크 조건을 지켜야 합니다.
- GUI는 실행 시 Operating Mode, PID, PWM/전류 제한, Shutdown 설정을 일괄
  재설정하지 않습니다. Jetson 초기화가 무엇을 바꾸는지 확인하고 GUI에서
  측정하지 않은 설정을 정상값으로 추정하지 마세요. 기존 안전·비상정지 처리는 유지하세요.
- degree/radian, ID 매핑, 방향·영점·오프셋과 GUI의 원시 위치 변환을 맞추세요.
  기준은 `angle_to_dxl_position()`이며 4096 step/rev, 중립 2048,
  최종값 0~4095입니다. Python `round()`와 C++ `std::round()`의 반올림 경계 차이도
  시험하세요. 5번 제외 시에도 다른 관절의 ID나 벡터 인덱스를 당겨 바꾸면 안 됩니다.

## 4. 시간, 공백, 반복, 완료

- 목표 제어 주기는 **5ms/200Hz**, 피드백은 원칙적으로 읽기 시작 시각 기준
  **10ms/100Hz** 간격입니다. `perf_counter`에 해당하는 단조 증가 시계를 사용하고
  tick 개수에 5ms를 곱해 재생 시각으로 삼지 마세요.
- 타이머 설정값과 실측 주기를 구분하세요. USB latency, SyncRead 시간,
  ROS callback/상태 발행/예약 전환 때문에 송신 tick이 생략되는 경로도 확인하세요.
  버스 접근은 직렬화해 SDK 포트를 동시에 사용하는 읽기·쓰기 경합을 방지하세요.
- `start_ms`, `time_ms`, `playback_speed`, `repeat_count`를 보존하세요.
  처음 공백은 시작 자세, 중간 공백은 직전 목표 자세를 유지합니다. 미도착 모터는
  대기 중에도 유지되는 목표를 계속 따라갈 수 있습니다.
- 조합 저장/내보내기는 원본 모션 배속을 이미 프레임 시각에 반영합니다.
  저장 JSON의 `playback_speed`를 적용할 때 원본 배속을 다시 적용하지 마세요.
  저장 전 조합을 직접 실행한다면 `composer_timing_segments()`와
  `playback_time_ms()`의 시간 매핑을 구현하고 공유 속도도 이 시간 기준으로 계산하세요.
- 일반 경계에서는 최종 목표각을 확정한 뒤 실제 도착을 기다리지 않습니다.
  지연 tick의 경계 처리도 `gate_timeline_on_frame_arrival()` 본문을 기준으로
  맞추세요. 이름만 보고 위치 도착 gate라고 해석하면 안 됩니다.
- 반복과 `playback_cycle_end` 경계는 최종 목표·피드백 처리를 마친 뒤 GUI처럼
  시계를 재설정합니다. 다음 사이클은 직전 최종 목표 자세를 이어받고 시작 속도는
  0입니다. 새 외부 실행 요청은 실제 위치를 읽어 시작하는 별도 경로입니다.
- 자연 완료는 최종 목표 송신 종료이며 실제 도착 확인과 다릅니다. 호환 모드에서
  기존 매 프레임 위치 gate나 최종 settling이 정상 타임라인에 대기를 추가하지 않게
  하세요. 현재 JSON의 `completion` 메타데이터만으로 GUI가 도착을 기다린다고
  가정하지 마세요. 취소·오류·비상정지 경로와는 구분하세요.
- GUI의 재생 중 위치 읽기 실패는 기록하지만 그 자체로 시간을 늘리지 않습니다.
  기존 Jetson의 장애 감시 정책과 다르면 차이를 명시하세요. 정상 타이밍을 맞추기
  위해 전체 통신 오류·하드웨어 오류 처리를 무조건 제거하지 마세요.

## 5. 5번 제외 옵션

GUI의 `excluded_joint_ids`는 기본적으로 빈 집합이며 **5번 제외는 기본 OFF**입니다.
Jetson에도 별도 runtime/launch 설정을 만들고 사용자가 선택했을 때만 적용하세요.
GUI JSON에는 이 선택이 저장되지 않으므로 JSON만 읽어서는 선택 여부를 알 수 없습니다.

선택 시 다음을 실제 호출 경로 전체에 적용하세요.

- 5번의 탐색, 위치·토크 읽기, SyncRead, 시작 자세 읽기, 자동 토크 ON,
  프로파일 준비, Goal Position 쓰기, 도착/필수 ID 검사를 제외합니다.
- 5번 미응답 때문에 나머지 모터의 초기화·전체 토크 ON·모션 재생이 중단되지
  않아야 합니다. JSON에는 5번 각도를 보존하되 전송 대상에서만 제외하세요.
- 5번만 담긴 중간 프레임은 실제 송신 없이 해당 시간을 진행합니다. 다른 관절의
  목표는 유지합니다. 전체 실행에 유효한 관절이 전혀 없는 경우는 별도로 처리하세요.
- 상태는 `excluded/미확인`으로 표현하고 토크 ON이나 정상 응답으로 꾸미지 마세요.
  나머지 모터의 오프라인·토크 OFF·읽기 실패 검사는 유지하세요.
- **전체 토크 OFF에는 제외한 5번도 포함해 송신**하세요. 명령 송신과 실제 적용
  확인은 구분하세요. 긴급 정지 대상에서 빠뜨리지 마세요.
- 재생·일시정지·동기화·프레임 이동 중 설정을 바꾸지 못하게 하고, 다시 포함할 때는
  실제 탐색·상태 확인을 거치세요. 설정 변경 자체로 5번을 움직이지 마세요.

이 기능은 5번의 토크를 실제로 켜 두거나 보호 정지를 해제하는 기능이 아닙니다.
기존 GUI는 시작 시 전체 토크 OFF를 보내므로 ‘이전 실행에서 ON이었다’고
새 실행에서도 ON이라고 가정하면 안 됩니다. 전원·배선 때문에 버스 전체가
응답하지 않는 문제는 이 옵션만으로 해결됐다고 판단하지 마세요.

## 6. 환경·데이터·검증

PC 환경 정보는 `PC_ENVIRONMENT.json`에 있습니다. Jetson의 OS/아키텍처,
JetPack/ROS 배포판, Python/C++ 도구체인, 실제 DYNAMIXEL SDK 버전과 경로,
포트 권한·baud·USB latency 설정을 확인해 비교표를 작성하세요. 기존 Jetson
환경에서 동작하는 의존성을 우선 활용하고 PC의 바이너리나 가상환경을 복사해
실행 환경이 같아졌다고 판단하지 마세요.

첨부 `sdk_gui_state.json`은 캡처 시점의 편집/저장 데이터이고 `robot_motions.json`은
기존 export 파일입니다. 두 파일이 최신 편집 내용까지 일치한다는 검증은 하지
않았습니다. GUI 현재 선택 모션과 실제 Jetson runtime 모션의 이름·각도·프레임
시각·배속·반복을 먼저 대조하고, 확인 없이 runtime JSON을 덮어쓰지 마세요.

하드웨어 없는 Python 기준 테스트는 `reference/`에서 다음처럼 실행할 수 있습니다.
GUI 생성자는 포트/토크를 조작하므로 기준 추출만을 위해 GUI 본체를 실행하지 마세요.

```bash
QT_QPA_PLATFORM=offscreen python3 -m unittest test_motor5_exclusion test_continuous_trajectory test_sdk_gui_timed_playback test_sdk_gui_precision test_sdk_gui_composer_timeline test_diagnose_mx_bus
```

PC에서는 73개 테스트가 통과했고 실제 모터를 구동하지 않았습니다. 새 보간으로
실제 흔들림이나 넘어짐이 줄었다는 실측 결과도 아직 없습니다.

Jetson 테스트는 위 검증과 기존 프로젝트 테스트를 통합하고, 동일한 가상 시작각·
JSON·배속·tick 시각·통신 지연을 Python 기준과 Jetson 플레이어 양쪽에 주입해
`(시각, motor ID, goal_deg, raw Goal Position)`를 대조하세요. 정상 5ms 격자뿐 아니라
지연 tick과 반복/예약 경계도 시험하고, 첫 불일치를 구체적으로 보고하세요.

필수 사례는 같은 방향 연결의 0이 아닌 경계 속도, 경계 가속도 연속, 방향 반전,
정지 관절, `[착지]`, 공백, 누락 관절, 목표각 초과 방지, 래핑·반올림 경계,
80% 옵션의 모드별 차이, 서로 다른 조합 배속, 반복·예약 전환, 5번만 미응답,
다른 ID 미응답, 5번만 담긴 중간 프레임, 제외 ID를 포함한 전체 토크 OFF입니다.

실제 하드웨어 검증을 하지 않은 경우 그 사실을 결과에 명확히 적으세요.
최종 결과에는 변경 파일, 실제 실행 경로, 적용 설정과 시작 명령,
GUI 대비 일치/불일치 표, 빌드·테스트 결과, 남은 실기 확인 항목,
변경 전 상태로 되돌리는 명령을 포함해 주세요.
