# GitHub 전체 모션 실행 경로 조사

대상: `agent/production-integration-20260912`, 커밋 `45fa1ddc136ff3af0d9ae355d4fbeeb45e5c248f`.

지정 브랜치의 추적 파일 197개 전체를 내려받고 각 파일을 Git blob 해시로 검증했다. 텍스트 195개를 대상으로 모터 통신, 재생 구현, 외부 라이브러리 호출, 빌드 의존성, 동적 로딩 및 실행 경로를 검색했고 모션 관련 경로의 코드를 읽었다. Python 117개는 모두 AST 구문 분석에 성공했다. 2개 바이너리는 비전용 best.engine / best.onnx이며 모터 제어 소스로 취급하지 않았다. ROS/하드웨어 실행은 하지 않았다.

전체 파일은 `IRC_vision/`, 파일별 해시와 분류는 `FULL_AUDIT_MANIFEST.json`에 있다. 이전 `IMPORT_MANIFEST.json`은 최초 선별 다운로드 75개에 관한 기록이다.

## 결론

이 브랜치에는 미션 판단, 모션 요청/시간 초과, SDK 연결 코드 및 실제 모션 각도 JSON이 있다. 그러나 프레임 재생과 Dynamixel 통신을 수행하는 실제 RobotMotionPlayer SDK 구현은 포함되지 않았다. 이는 README만을 근거로 한 결론이 아니라 아래 실행 경로, 빌드 연결, 테스트 대역의 구현 내용으로 확인했다.

```text
full_system_robot.launch.py
  → motion_decision_node / motion_command_bridge_node
  → /motion/executor/request (motion_id, timeout_ms)
  → sdk_motion_executor / SdkExecutorCore
  → RobotMotionPlayerBackend
  → BorrowedRobotMotionPlayerApi
  → 외부 RobotMotionPlayer.start()/update()/cancel()
  → 외부 DynamixelMotionHardware
```

## 확인한 모든 재생 경로

| 경로 | 실제 구현 |
|---|---|
| `src/irc_step_motion_executor/src/robot_motion_player_backend.cpp` | SDK의 start/update/cancel/queueNext 등에 전달하고 결과를 ROS용 상태로 변환 |
| `src/irc_step_motion_executor/src/production_robot_motion_runtime_factory.cpp` | 외부 헤더로 선언된 hardware/player 객체를 생성. 자체 모터 통신 구현 없음 |
| `src/irc_step_motion_executor/src/simulated_motion_backend.cpp` | poll 횟수에 따라 RUNNING → SETTLING → SUCCEEDED. 모터 전송 없음 |
| `src/mission_control/mission_control/motion_player_factory.py` | mock 또는 SdkMotionPlayerPlaceholder 생성 |
| `src/mission_control/mission_control/sdk_motion_player_placeholder.py` | hardwareReady=False, start=HARDWARE_NOT_READY |
| `src/mission_control/mission_control/sdk_motion_stub_node.py` | 동작을 로그로 알리고 완료 상태를 흉내 냄 |
| `src/mission_control/mission_control/mock_motion_player.py` | 테스트용 상태 전이 |
| `src/step/step/step_motion_pose_test.py` | 가상의 걸음 수/로봇 위치를 발행. 모터 제어 없음 |
| `test/fixtures/fake_robot_motion_sdk/robot_motion_player.cpp` | initialize는 카운터/불리언 갱신, update는 항상 Idle, setJointOverride는 빈 함수 |
| `test/fixtures/missing_target_sdk/robot_motion_player.cpp` | 누락된 CMake target 검증용 코드 |
| `archive/`, 비전 planner/controller, JSON 도구 | 비전 판단·상위 명령·데이터 변환. 별도 모터 통신 경로 없음 |

빌드의 `ROBOT_MOTION_SDK_DIR`은 빈 문자열이 기본값이다. SDK ON이면 이 경로의 CMakeLists.txt, robot_motion_player.hpp/.cpp를 확인하고 add_subdirectory로 가져와 robot_control target에 링크한다. SDK OFF 상태에서 실제 backend를 요청하면 ROBOT_MOTION_PLAYER_BACKEND_NOT_BUILT를 반환한다. SDK 자동 다운로드, 모터 라이브러리 바이너리 또는 Git submodule은 발견하지 못했다.

주요 근거:
- [CMake 외부 SDK 연결](https://github.com/dbwldus0220-sys/IRC_vision/blob/45fa1ddc136ff3af0d9ae355d4fbeeb45e5c248f/src/irc_step_motion_executor/CMakeLists.txt#L8-L53)
- [실제 backend의 SDK 호출](https://github.com/dbwldus0220-sys/IRC_vision/blob/45fa1ddc136ff3af0d9ae355d4fbeeb45e5c248f/src/irc_step_motion_executor/src/robot_motion_player_backend.cpp#L173-L197)
- [SDK 미빌드 오류](https://github.com/dbwldus0220-sys/IRC_vision/blob/45fa1ddc136ff3af0d9ae355d4fbeeb45e5c248f/src/irc_step_motion_executor/src/motion_backend_factory.cpp#L66-L74)
- [가짜 SDK update 구현](https://github.com/dbwldus0220-sys/IRC_vision/blob/45fa1ddc136ff3af0d9ae355d4fbeeb45e5c248f/src/irc_step_motion_executor/test/fixtures/fake_robot_motion_sdk/robot_motion_player.cpp#L151-L175)

`.gitignore`는 build/install/log 및 src/king을 제외한다. 따라서 브랜치 파일이 젯슨 디스크의 모든 파일을 포함한다고 단정할 수 없다. src/king이 실제 구동 코드라는 증거는 없다.

## 새로 확인한 모션 데이터 차이

현재 작업 폴더의 `sdk_gui_state.json` saved_sequences와 GitHub `artifacts/robot_motions_runtime.json`을 비교했다. GUI 저장 시퀀스 240개, GitHub 모션 129개이며 공통 이름 121개 중 62개에서 모터에 영향을 주는 데이터가 다르다.

비교 항목: repeat_count, playback_speed, 프레임 순서/개수, start_ms, time_ms, angles, 이름의 [착지] 여부. 숫자는 절대 오차 1e-9까지 같다고 처리하고 일반 이름 변경·frame_id 등 편집 메타데이터는 제외했다. GUI의 아직 저장되지 않은 편집 상태 또는 젯슨 런타임에 실제 로딩된 JSON을 확인한 것은 아니다.

| 모션 | 차이 | 현재 GUI 저장본 | GitHub 실행용 JSON |
|---|---|---|---|
| 공잡기리그랩까지 실전 | 4번째 프레임 모터 9 | 43.41796875° | 57.216796875° |
| 제자리우회전(1회) | 첫 프레임 시간 | 65ms | 60ms |
| 제자리우회전(1회) | 두 번째 프레임 시작/시간 | 135ms / 120ms | 130ms / 140ms |
| 오른쪽꽃게걸음 | 반복 횟수 | 1회 | 4회 |
| 제자리좌회전(2번) | 첫 프레임 모터 20 | -60° | -68° |

전체 차이는 `GUI_RUNTIME_MOTION_DIFF.json`에 저장했다. 경로의 frames 인덱스는 0부터 시작한다.

GUI와 젯슨의 움직임 차이를 재생 알고리즘만의 문제라고 볼 수 없다. 동일한 모션 데이터인지 먼저 대조해야 한다. 이번 조사에서는 GUI 코드나 원본 모션 파일을 변경하지 않았다.

## GitHub에서 확정 가능한 실행 설정

- full_system_robot은 sdk_motion_executor를 실행하고 기본 poll 간격을 5ms로 설정한다. 이는 모터 목표 전송 주기 자체를 의미하지 않는다.
- 기본 runtime JSON은 설치된 irc_step_motion_executor/config/robot_motions_runtime.json이다.
- motion_command_bridge_node의 기본 요청 timeout은 12000ms이며 입력의 timeout_ms로 바뀔 수 있다. SDK executor는 timeout 시 cancel을 호출한다.
- 공 감지에 따른 0번 관절 목표 덮어쓰기와 시작 자세 이동, 모션 예약 기능이 있다.
- GUI에 이번에 적용한 프레임 시간 프로파일 및 2°/3초/80ms 판정은 작업 폴더 C++ SDK를 기준으로 한 것이다. 이 GitHub에는 외부 SDK의 실제 궤적/프레임 도착 판정 구현이 없으므로 완전 동일성을 입증할 수 없다.

지정 브랜치만으로 모든 실제 모터 동작을 구성할 수 있다는 주장은 현재 소스의 외부 의존성과 맞지 않는다. 실제 로봇을 움직이는 기존 실행 프로그램이 있다면, 그 바이너리/연결 라이브러리나 별도 실행 경로까지 포함해야 전체 구성이다.
