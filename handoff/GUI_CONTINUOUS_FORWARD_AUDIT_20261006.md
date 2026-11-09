STEP GUI 연속 전진 재생 경로 조사 — 2026-10-06

**현재 GUI의 A(내부 반복)와 B(조합)는 정상 반복 경계에서 이전 마지막 목표각을 이어받는다. 실제각으로 시작점을 바꾸는 경로는 C(자연 종료 후 새 재생)다.** A/B는 경계 피드백을 읽지만 도착 여부를 기다리지 않는다. 같은 데이터와 가상 통신 조건에서 A/B의 목표각·쓰기/읽기 순서 및 시각은 일치했다. 이 결과만으로 Jetson의 넘어짐 원인이나 실제 관절 오차의 누적을 확정할 수는 없다.

GUI·SDK 소스와 모션 데이터는 수정하지 않았다. 모터 포트를 열거나 모터 명령을 보내지 않았다. 기존 테스트와 현행 GUI 함수를 추출한 가상 통신 검증을 실행했다. 아래의 가상 기록은 실기 측정값이 아니다.

**1. 검토한 버전과 데이터 동일성**

- GUI: 현재 작업 폴더 `sdk_gui.py`. SHA256 `ce6b871a14af1187a5e3f221834265649e1e819cb6b529487d75a090c7e2d527`. 전달용 `gui_reference_for_algorithm_20261006.zip`의 GUI 소스와 바이트 단위로 같다.
- A/B 가상 비교 데이터: 위 ZIP의 동일한 `robot_motions.json`을 양쪽에 사용했다. SHA256 `3afac007c458a2055fe40395510aeba3bc3a82af23bbbd24c77d19224cee4edb`.
- 정확한 전진 모션 이름을 추가 질문했으며, 확인 전이므로 로컬의 `건전진45도(4회)`와 `건전진90도(4회)` 두 후보를 모두 검증했다. 두 후보 모두 첫 프레임 start_ms=90, repeat_count=4, 저장 배속=1.0이다.
- 현재 폴더의 저장 상태·배포 JSON은 서로 일치하지만 전달용 ZIP과는 일부 데이터가 다르다. 사용자가 설명한 배포 JSON 미갱신 문제로 취급한다. 내보내기 오류를 다시 추정하지 않았다.
- 로컬 Jetson 참조본은 `jetson_reference/IRC_vision`, 가져온 커밋 `45fa1ddc136ff3af0d9ae355d4fbeeb45e5c248f`이다. 이 참조본의 runtime JSON에는 위 두 이름이 없다. 현재 Jetson의 실제 바이너리·외부 SDK·설치된 runtime JSON과 동일하다는 증거는 없다.
- 작업 폴더의 `robot_motion_player.cpp`에는 아직 `gateCurrentFrame()`과 마지막 Settling이 있다. 메일에 제공된 최신 Jetson 동작과 다르므로 이 파일을 실제 Jetson 플레이어로 간주하지 않았다.
- 실행 중 GUI가 메모리에 로드한 코드·UI의 미저장 조합·배속과 디스크 스냅샷의 동일성도 별도 확인 대상이다.

따라서 **실제 GUI와 실제 Jetson에 같은 최신 데이터가 로딩되었다는 선행 조건은 아직 미확인**이다. 실기 비교 전에 실행 중 노드의 `motion_json_path`, alias로 해석된 모션 이름, 파일 SHA256, 로딩 시점, 실행 중 repeat/speed 값을 기록해야 한다. 파일을 바꿔도 이미 메모리에 로드한 모션은 자동으로 바뀐다고 가정하지 않는다.

로컬 launch의 기본 경로는 소스 폴더가 아니라 설치 패키지의 `config/robot_motions_runtime.json`이다. 근거: [full_system_robot.launch.py](</home/rnd_3/final step/jetson_reference/IRC_vision/src/mission_control/launch/full_system_robot.launch.py:440>). 실제 Jetson에서는 `ros2 param get /sdk_motion_executor motion_json_path`와 해당 경로의 `sha256sum`을 확인하고, `ROBOT_MOTION_SDK_DIR`·실행 파일 해시·빌드 커밋을 함께 기록한다. 노드명이 다르면 실제 노드명을 사용한다.

**2. 60° 목표 / 57° 측정 사례에 대한 답**

아래는 해당 관절이 마지막 프레임과 다음 첫 프레임에 모두 포함된 정상 연속 재생 기준이다. 각도는 인코더 정수 변환 전의 목표각이다.

| 질문 | A: 저장 시퀀스 내부 반복 | B: 4회 전진 블록을 네 개 조합 | C: 자연 종료 후 새 재생 |
|---|---|---|---|
| 최초 시작각 | 시작 직전 실제각을 읽음 | 조합 전체 시작 직전 실제각을 읽음 | 버튼을 다시 누른 시점에 실제각을 다시 읽음 |
| 다음 반복/블록 시작각 | 이전 마지막 목표 60° | 이전 마지막 목표 60° | 새 읽기에서 얻은 57° |
| 다음 첫 90ms의 목표 | 60° 유지 명령 | 60° 유지 명령 | 읽어 둔 57° 유지 명령 |
| 90ms 경과 후 | 60°를 시작점으로 오들 목표까지 보간 | 동일 | 57°를 시작점으로 오들 목표까지 보간 |
| 실제 60° 도착을 확인하는가 | 확인 조건 없음 | 확인 조건 없음 | 도착 조건 없음 |
| 새 피드백마다 시작각을 바꾸는가 | 바꾸지 않음 | 바꾸지 않음 | 새 재생 시작 때 한 번 읽고, 재생 중 계속 바꾸지 않음 |

60°를 명령해도 실제각이 즉시 60°가 되는 것은 아니다. 90ms 공백 동안 실제 관절이 57°에서 60° 쪽으로 움직일 수 있지만, 도달·정착 여부는 부하·제어기·통신·실제 측정으로 확인해야 한다. 90ms가 지나도 57°라면 A/B는 여전히 목표 시작점 60°에서 다음 궤적을 진행한다. 정확히 90ms에 호출되지 않으면 90ms 이후 처음 실행되는 tick에서 그 경과시간에 해당하는 목표를 보낸다.

근거: [초기 실제각 읽기](</home/rnd_3/final step/sdk_gui.py:4849>), [새 재생의 시작각·시계 설정](</home/rnd_3/final step/sdk_gui.py:4945>), [반복 시작각 갱신](</home/rnd_3/final step/sdk_gui.py:5075>), [이전 목표 누적과 공백 처리](</home/rnd_3/final step/sdk_gui.py:5219>).

**3. A/B/C의 실제 호출 순서와 경계 차이**

| 항목 | A | B | C |
|---|---|---|---|
| 진입 | `play_motion_page_sequence → play_keyframe_sequence → start_motion_playback` | `play_composed_sequence → activate_composer_playback → play_keyframe_sequence → start_motion_playback` | A/B의 새 재생 진입 경로를 다시 실행 |
| 반복 표현 | `playback_repeat_target/current` | `composer_entry_frames()`가 원본 반복 수만큼 펼치고 각 원본 주기 끝에 `playback_cycle_end=True`를 넣음 | 새 실행이므로 반복 카운터 초기화 |
| 마지막 목표·읽기 순서 | 직전 활성 프레임의 종료 목표 전송 → 끝 시점 `scrub_timeline(force_robot=True)`의 목표 전송 → 동기식 피드백 | 같은 순서. 내부 경계에서는 `cycle_end=True`도 전달 | 이전 자연 종료는 마지막 목표·강제 피드백 후 종료. 새 재생은 관절별 현재각을 읽은 뒤 시작 |
| 경계 시계 재시작 | 피드백/FK/출력 처리가 반환된 뒤 `restart_playback_cycle(0, last_frame)` | 반환된 뒤 `restart_playback_cycle(cycle_end_ms, last_frame)` | 새 시작각 읽기·초기화 후 `anim_start_time` 설정 |
| 경계의 실제각 사용 | `feedback_angles` 기록용. 시작각을 덮어쓰지 않음 | 동일 | `joints → start_angles`에 실제 읽기값 적용 |
| 추가 sleep/도착 대기 | 정상 반복에는 없음 | 정상 조합에는 없음 | 관절별 시작각 읽기는 동기식. 실패 재시도 때 5ms sleep 가능 |
| 프로파일 재설정 | 정상 반복마다 하지 않음 | 블록마다 하지 않음 | 동기화 ON을 유지한 자연 종료 후 재시작 자체에는 없음. 동기화 준비/토크 조작 경로를 거치면 별도 설정 있음 |
| 첫 공백 | 원본 90ms를 매 반복 유지 | 추가 gap=0이어도 원본 90ms를 매 반복 유지 | 새 측정 시작각을 유지하며 원본 90ms 진행 |

근거: [A 진입](</home/rnd_3/final step/sdk_gui.py:4899>), [B 진입](</home/rnd_3/final step/sdk_gui.py:4144>), [조합 반복 펼치기](</home/rnd_3/final step/sdk_gui.py:3774>), [조합 반복 수 초기값](</home/rnd_3/final step/sdk_gui.py:3850>), [A/B 경계 실행](</home/rnd_3/final step/sdk_gui.py:5061>), [강제 피드백](</home/rnd_3/final step/sdk_gui.py:5297>).

직전 활성 프레임이 정상적으로 방문된 경우, 경계에서 같은 최종각이 두 번 전송될 수 있다. `gate_timeline_on_frame_arrival()`의 종료 목표 전송과 `scrub_timeline()`의 끝 시점 전송이 둘 다 있기 때문이다. 활성 프레임이 없던 경우에는 이중 전송이 생기지 않을 수 있다. 이 함수 이름의 “arrival”은 실제 도착 대기를 뜻하지 않는다.

일반 프레임 경계에서는 늦은 tick을 우선 직전 프레임 끝으로 제한하여 최종각을 보내지만, `anim_start_time`을 변경하지 않는다. 다음 tick은 실제 경과시간으로 다시 계산한다. 반면 **반복·조합 경계에서는 경계 통신을 모두 마친 현재 시각으로 시계를 재설정**하므로 경계 처리 시간과 이미 늦어진 시간이 다음 주기로 이월되지 않는다. 이것은 오차에 따른 시간 보정이 아니라 고정된 경계 처리 규칙이다. 근거: [일반 경계](</home/rnd_3/final step/sdk_gui.py:5860>), [반복 시계](</home/rnd_3/final step/sdk_gui.py:5075>).

결국 추가 대기 설정이 0ms여도 마지막 목표 전송 이후에는 경계 통신 시간, 다음 tick까지의 시간, 원본 90ms 공백이 존재한다. 90ms는 논리 타임라인 길이이며 실제 벽시계상 정확히 90ms를 보장하지 않는다.

B가 A와 같은지에 대해: 동일 소스 모션·시작 자세·총 반복 수·유효 배속·공백과 동일한 가상 I/O 스케줄에서는 같았다. 실제 UI 비용까지 동일하다는 뜻은 아니다. A는 외부 반복 때 플레이헤드·스크롤을 추가로 갱신하고, B는 펼쳐진 더 긴 프레임 목록을 검색·정렬한다. 렌더링과 운영체제 스케줄링은 가상 검증에서 대역 처리했다.

**4. 배속, 발 들기, 내부 공백**

- A의 타임라인은 `elapsed_real_ms × playback_speed`다. `start_ms=90`인 앞 공백의 명목 실제 시간은 `90 / playback_speed` ms다.
- B의 원본 모션 구간에는 원본 시퀀스 배속과 조합 전체 배속을 함께 적용한다. 명목 앞 공백은 `90 / (source_speed × composer_speed)` ms다.
- 조합에서 추가한 `gap_before_ms`는 원본 배속의 영향을 받지 않고 조합 전체 배속만 적용한다.
- 조합 저장/내보내기는 원본 배속을 프레임 시작·끝 시각에 반영해 정수 ms로 반올림한다. 이후 저장된 조합의 전체 배속만 적용한다. 이 저장 경로는 정수 ms 반올림 때문에 조합 탭의 실시간 재생과 최대 약 1ms 수준의 프레임 길이 차이를 낼 수 있으므로 별도로 구분한다.
- `lift_early_arrival=True`이면서 이름에 `발들/들기/오들/왼들`이 들어간 프레임은 앞 80% 시간에 **명령 목표**가 끝값에 도달하고 나머지 20%는 그 목표를 유지한다. 실제 관절 도착을 확인하거나 오차에 따라 비율을 바꾸지 않는다.
- `[착지]`라는 이름만으로 현재 시퀀스 재생에서 별도 내부 가감속 프로파일을 설정하지 않는다. 이전 방식의 `command_time_profile_frame()`은 정의되어 있지만 현재 재생 경로에서 호출하지 않는다.

근거: [조합 시계·저장 배속](</home/rnd_3/final step/sdk_gui.py:4031>), [반 코사인/80% 목표 도달](</home/rnd_3/final step/sdk_gui.py:5474>).

로컬 후보 `건전진45도(4회)`의 1주기는 다음과 같다. 실제 Jetson에서 선택한 이름은 아직 확인되지 않았다.

| 논리 시각(ms) | 동작 |
|---|---|
| 0~90 | 시작 목표 유지 |
| 90~150 | 김오들, 60ms. 옵션 활성 시 138ms부터 끝 목표 유지 |
| 150~255 | 왼뒤, 105ms |
| 255~339 | 직전 목표 유지, 84ms |
| 339~401 | 김왼들, 62ms. 옵션 활성 시 388.6ms부터 끝 목표 유지 |
| 401~403 | 직전 목표 유지, 2ms |
| 403~513 | 김오뒤, 110ms |

따라서 추가 gap=0이 원본 내부 공백까지 없앤다는 뜻은 아니다. `건전진90도(4회)` 후보는 마지막 프레임이 401~511ms여서 위 2ms 공백이 없다. 45도 후보의 명목 16주기 길이는 8,208ms이며 실제 완료 시각에는 통신·경계·tick 지연이 반영된다.

**5. 통신과 모터 설정**

`QTimer(Qt.PreciseTimer)`는 5ms로 설정되어 있다. 실제 tick 안에서는 목표 계산, 조건부 UI 갱신, SyncWrite, 조건부 SyncRead, 피드백 해석/FK, 로그 출력이 같은 흐름에서 실행된다. `GroupSyncRead.txRxPacket()`은 송신 후 응답 수신을 마칠 때까지 반환하지 않는 동기식 함수다. 그동안 같은 GUI 제어 흐름은 다음 목표를 전송하지 못한다. [타이머 설정](</home/rnd_3/final step/sdk_gui.py:1646>), [전송 후 읽기](</home/rnd_3/final step/sdk_gui.py:5297>), [동기식 읽기](</home/rnd_3/final step/sdk_gui.py:5741>).

일반 피드백은 마지막 읽기 호출 직전 시각을 기준으로 10ms 이상 경과하면 실행한다. 완료·반복 경계는 `force_robot=True`로 강제 읽는다. 경계에서 마지막 읽기 시각을 0으로 초기화하므로 다음 주기 첫 tick에서도 곧바로 읽을 수 있다. 읽기 자체가 10ms를 넘으면 거의 매 tick 읽는 상황이 가능하다. 별도의 100ms 라이브 각도 타이머는 재생 중 읽기를 건너뛴다.

읽기가 늦어지면 일반 구간은 놓친 5ms 샘플을 모두 재전송하지 않고 벽시계 기준 다음 시점으로 진행한다. 큰 지연으로 전혀 방문하지 못한 짧은 프레임이 생길 가능성도 있다. 이미 활성화된 프레임의 끝 목표는 경계 함수가 따로 처리한다. 반복 경계는 앞 절에서 설명한 대로 시계를 재설정한다. 따라서 “항상 전체 타임라인이 늦춰진다”와 “항상 아무 지연 없이 따라잡는다” 모두 부정확하다. Qt 역시 부하로 늦어진 타이머의 만료 신호를 누락된 횟수만큼 모두 발생시키지 않는다고 설명한다. [Qt 공식 QTimer 문서](https://doc.qt.io/qt-6/qtimer.html#accuracy-and-timer-resolution).

SyncWrite의 반환 성공은 전송 성공 상태이며 모터별 목표 수신·실제 도착 확인을 뜻하지 않는다. [ROBOTIS 패킷 처리 설명](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/api_reference/python/python_packethandler/).

| 설정/동작 | 현재 GUI에서 확인한 내용 |
|---|---|
| Profile Acceleration/Velocity | 준비 시 108/112번 주소를 모두 0으로 설정. 정상 시퀀스 매 프레임·반복마다 다시 쓰지 않음 |
| Drive Mode | 준비가 필요한 모터는 토크 OFF 상태에서 기존 값에 bit2(0x04)를 더해 시간 기반 모드 설정. EEPROM 기록 후 20ms 기다리고 검증 |
| PID | 현재 GUI에 PID 레지스터를 변경하는 경로 없음. 기존 모터 설정을 사용 |
| Operating Mode | 현재 GUI에서 설정하거나 읽어서 검증하는 경로 없음 |
| 전류/PWM 한도·방향·Homing Offset | 재생 때 일괄 설정하는 경로 없음 |
| Torque | 사용자 토크 조작·초기 구동 준비에서 설정. 자연 종료 시 토크 OFF 하지 않음 |
| 추종 오차 | 목표/실제각·FK 오차를 계산해 기록. 정상 재생의 목표 궤적이나 시간을 오차 크기에 따라 보정하지 않음 |

근거: [프로파일 초기화](</home/rnd_3/final step/sdk_gui.py:1878>), [Drive Mode 준비](</home/rnd_3/final step/sdk_gui.py:1917>), [실기 준비](</home/rnd_3/final step/sdk_gui.py:5537>), [오차 계산](</home/rnd_3/final step/sdk_gui.py:5642>). 프로파일 0은 내장 궤적 생성의 Step 설정이며 모터의 PID를 끄거나 물리적 속도·토크를 무한대로 만드는 뜻이 아니다. [ROBOTIS MX 2.0 프로파일 설명](https://emanual.robotis.com/docs/en/dxl/mx/mx-106-2/#what-is-the-profile).

현재 설치된 Python SDK의 timeout 상수 `LATENCY_TIMER`는 16이고, 확인 당시 시스템에 보인 ttyUSB1의 USB latency_timer도 16이었다. 이는 실제 읽기 시간이 16ms라는 측정 결과가 아니다. GUI 프로세스가 열고 있던 장치 FD와 동일한 현재 장치인지도 확인되지 않았으므로 PC/Jetson 성능 차이의 증거로 사용하지 않는다.

**6. 정지 시 흔들림을 비교할 때 구분할 경로**

자연 종료는 마지막 목표 전송과 강제 피드백 후 `stop_motion_sequence(keep_timed_goal=True)`를 호출한다. 마지막 목표 60°를 유지하며 실제각 57°로 바꾸거나 토크를 끄지 않는다. 실제 관절에 속도·몸통 운동이 남아 있는지는 이 종료 판정으로 알 수 없다.

사용자가 재생 중 정지/일시정지하면 `hold_timed_motion_at_feedback()`가 피드백을 읽고 프로파일을 0으로 설정한 뒤 측정각을 새 목표로 보내는 경로다. 예시 조건에서는 목표가 60°에서 57°로 바뀔 수 있다. 이 동작도 몸통 운동을 부드럽게 감속시킨다는 보장은 없다.

정지 경로에는 `execute_on_real_robot`, 활성 프레임 token, 준비된 모터 ID 등의 조건이 있다. 특히 반복 직후 첫 프레임 전 공백에는 token이 None일 수 있어 측정각 홀드 함수가 즉시 반환한다. 읽기 실패 시에는 남아 있는 피드백을 사용할 가능성도 있으므로 로그에 측정 성공 여부와 데이터 나이를 남겨야 한다. [자연 종료와 사용자 정지](</home/rnd_3/final step/sdk_gui.py:5138>), [측정각 홀드](</home/rnd_3/final step/sdk_gui.py:5955>).

Jetson에서 관측한 “멈춤”이 정상 완료, 다음 명령 없음, cancel, timeout, 비전/안전 이벤트 중 무엇인지 먼저 분류해야 한다.

**7. 검증 결과와 Jetson에서 비교할 구체적 지점**

기존 GUI 테스트 40개가 통과했다. 추가 가상 대조는 현행 GUI의 실제 함수 본문을 AST로 추출해 실행했다. 생성자·모터 연결은 실행하지 않았다.

- A: 같은 원본 1주기를 16회 반복.
- B: 원본 repeat_count=4인 블록을 네 개, 추가 gap=0으로 조합.
- 데이터 두 종류, 원본 배속 1.0/0.9, 읽기 비용 0/5/15/40ms, 쓰기 비용 0/1ms = 32조건.
- 모든 조건에서 목표각 값, 쓰기·읽기 이벤트 수/순서/가상 시각, 반복 경계 시각이 일치했다.
- 가상 스케줄은 한 tick의 처리가 끝난 뒤 5ms가 지나 다음 tick을 호출한다. 실제 Qt 스케줄러·렌더링 비용·물리 모터 모델을 재현한 것이 아니므로 숫자를 실기 Hz로 해석하지 않는다.
- 측정각을 57°에 고정한 별도 사례에서 A의 다음 시작 목표는 60°, C의 새 시작 목표는 57°임을 확인했다. 이것은 오류 누적을 가정하지 않고 데이터 사용 경로를 확인하는 시험이다.

동일한 가상 SDK 조건에서 예약 연결과 내부 반복이 같았다는 메일의 결과는 코드 경로 차이의 범위를 좁힌다. 그러나 ROS 상위 계층의 요청 도착 시각·poll 횟수·실제 통신 시간을 포함하지 않았다면 실제 연속 재생까지 같다고 입증한 것은 아니다.

**구체적으로 확인한 로컬 참조본의 상위 계층 차이:** `SdkExecutorCore::poll()`은 예약 전환 후 `queued_running_status_pending_` 상태에서 RUNNING 상태를 반환하고 `backend_.poll_status()`를 호출하지 않는다. 그 다음 poll에서 SDK update를 다시 호출한다. [조기 반환 위치](</home/rnd_3/final step/jetson_reference/IRC_vision/src/irc_step_motion_executor/src/sdk_executor_core.cpp:326>), [backend에서 SDK update 호출](</home/rnd_3/final step/jetson_reference/IRC_vision/src/irc_step_motion_executor/src/robot_motion_player_backend.cpp:266>).

실제 참조 C++ core를 대역 backend와 컴파일한 시험 결과:

| core poll 시각(가상 ms) | 누적 backend poll 호출 | 반환 상태 |
|---|---:|---|
| 100 | 1 | SUCCEEDED — 예약 경계 |
| 105 | 1 | RUNNING — 예약 활성화 통지 |
| 110 | 2 | RUNNING |

즉 해당 참조본에는 묶음 예약 경계 직후 한 번의 SDK update를 생략하는 실행 경로가 실제로 있다. 5ms 설정이면 명목 한 poll 기회가 빠지며 실제 간격은 callback 지연의 영향을 받는다. 다만 첫 90ms 홀드 안에서 같은 목표의 재전송 하나가 빠지는 정도라면 영향이 작을 수도 있다. **최신 Jetson에도 이 코드가 있는지, 그 순간 목표/측정각과 통신 시각에 어떤 영향이 있는지 확인하기 전에는 넘어짐 원인으로 확정하지 않는다.**

다른 확인 지점은 다음과 같다.

| 지점 | 확인된 사실 또는 가설 | 필요한 증거 |
|---|---|---|
| 실제 예약 성공 여부 | 메일의 가상 비교에서는 같음. 실기에서 모든 다음 명령이 미리 QUEUED였는지는 미측정 | request/queue result/activation/start/complete의 ID와 시각. 4·8·12회 경계에서 새 start 여부 |
| 경계 시계 기준 | GUI는 최종 피드백 반환 후 시계 재설정 | Jetson의 clock reset이 TX 전/후, RX 전/후 중 어디인지 최신 SDK와 로그로 확인 |
| 실제 TX/RX 간격 | GUI는 동기식 통신. 5ms 설정만으로 동일성 보장 안 됨 | TX 간격 분포, 순수 RX 시간, 처리 총시간, timeout 및 누락 ID |
| 90ms 홀드 | GUI A/B는 각 원본 반복마다 유지. B 추가 gap=0도 동일 | Jetson 내부 반복·queue·새 start 각각에서 첫 공백 보존 여부 |
| 초기 모터 설정 | GUI는 PID/Operating Mode를 재설정하지 않음 | 각 실행 전후 모터 설정 readback. 로컬 구 SDK의 P=850 설정을 최신 Jetson 값으로 추정하지 말 것 |
| 종료 정책 | GUI 자연 종료는 끝 목표 유지, 활성 프레임 중 수동 정지는 측정각 홀드 | Jetson 완료/cancel/timeout 직후 목표·토크·프로파일·속도 |
| 스케줄링/부가 목표 | 참조 launch의 poll 설정은 5ms. 노드는 head override와 상태 처리도 수행 | 최신 노드 callback 시간, head override 활성 여부, CPU/USB 지연 |
| 궤적 세부 사항 | GUI의 코사인·최단각·80% 옵션·프레임 경계 목표 확정 | 동일 데이터에서 raw Goal Position packet 시퀀스 비교 |

**8. 실제 기록 방법 — 이번에는 계측 코드를 적용하지 않음**

현재 GUI는 10Hz 정도로 관절별 `goal/actual/error`, FK 오차와 `SyncRead=...ms`를 stdout에 출력한다. 이 기록은 초기 확인에 쓸 수 있지만 매 목표 전송 시각·쓰기 시간·명확한 반복 번호·시계 재설정을 모두 담지 않는다. 또한 현재 `SyncRead`로 표시하는 시간은 파라미터 준비·통신·디코딩·FK 계산까지 포함하고 콘솔 출력 시간은 제외하므로 순수 버스 읽기 시간으로 해석하면 안 된다. [현재 로그의 측정 범위](</home/rnd_3/final step/sdk_gui.py:5747>).

다음 단계의 최소 계측은 제어식을 바꾸지 않고 다음 이벤트를 단조 시계 `perf_counter_ns()`로 메모리 버퍼에 기록하는 방식이다. 제어 tick 안에서 매번 파일 flush나 콘솔 print를 추가하지 않는다. 기록을 끝낸 뒤 JSONL/CSV로 저장하고 buffer overflow와 계측 비용도 남긴다.

| 계측 위치 | 필수 기록 |
|---|---|
| 새 재생 시작 | 모션 이름/ID, 경로 A/B/C, 실제 로드한 데이터 hash, UI 배속/반복, `start_angles`, 관절별 시작 읽기 시각 |
| `anim_step()` 진입 | 단조 시각, tick 간격, 계산된 requested timeline, 경계 처리 후 effective timeline, 활성 프레임 ID |
| `write_goal_positions()` | 계산 목표각, 반올림/제한 후 raw Goal Position, 실제 전송 ID, 호출 전체 시작/끝 |
| `groupSyncWrite.txPacket()` 바로 전후 | 순수 쓰기 호출 시작/끝, 통신 결과, 전송 일련번호 |
| `groupSyncRead.txRxPacket()` 바로 전후 | 순수 읽기 호출 시작/끝, 응답 성공/누락 ID, raw Present Position |
| `read_feedback_and_report_fk()` 전체 | 실제각, 해당 샘플과 비교한 command 일련번호, FK/기타 처리 시간, 마지막 유효 샘플 나이 |
| `restart_playback_cycle()` 전후 | 주기/블록 번호, 직전 마지막 목표, 직전 실제각, 이전/새 `anim_start_time`, 논리 offset |
| 완료/정지 | natural_end/pause/stop/failure 종류, keep_timed_goal, 직후 명령 목표, 프로파일/토크 쓰기 여부 |

명령 발행 시각과 실제각 수신 시각을 별도로 남긴다. SyncRead의 여러 관절 응답은 완벽한 동시 샘플이라고 가정하지 않는다. PC와 Jetson의 단조 시계는 원점이 다르므로 서로의 절대값을 바로 빼지 않고, 실행/프레임/command ID와 경계 이벤트를 맞춰 비교한다.

소스 파일을 수정하지 않는 보조 기록으로는 현재 터미널 로그 보관과 수동 USB/직렬 트레이스가 가능하다. USB 패킷만으로는 Python 내부 시작각·시계 기준을 알 수 없으므로 소프트웨어 이벤트 기록을 대체할 수 없다. 이미 동작 중인 GUI 옆에서 두 번째 제어 프로그램을 실행하는 방식으로 기록하지 않는다.

오차 누적 여부는 같은 프레임의 같은 위상에서 주기별 `e[k] = shortest_angle_delta(actual[k], commanded[k])`를 비교한다. 경계 직후·90ms 종료·오들 목표 유지 구간·최종 프레임 끝의 오차와 속도를 함께 본다. 일정한 추종 지연, 주기적 진동, 누적되는 편차를 구분하고, 4·8·12회 묶음 경계에만 변화가 있는지도 확인한다. 현재 자료에는 이를 판단할 실기 시계열이 없다.

**9. 원인별 최소 수정 후보와 예상 영향 — 모두 미적용**

| 증거로 확인될 경우 | 최소 수정 후보 | 보행에 미칠 수 있는 영향/검증 |
|---|---|---|
| 실제 runtime 데이터가 다름 | 동일 최신 데이터로 배포·재로딩하고 로드 hash 출력 | 의도한 자세/시간을 맞춤. 이미 확인된 데이터 문제와 제어 문제를 분리 |
| 요청이 늦어 queue 대신 새 start가 됨 | 상위 계층에서 다음 요청을 경계 전에 예약하고 성공/활성화 상태를 기록 | 추가 시작각 읽기·측정각 재설정을 줄임. cancel/중복 요청 의미 보존 필요 |
| 최신 core에도 예약 통지 때문에 update 생략 | 상태 알림과 SDK update 수행을 분리하여 통지 poll도 정상 갱신 | 묶음 경계의 불규칙한 tick을 줄일 가능성. 현재 90ms 홀드 내 영향과 상태 이벤트 순서 확인 |
| Jetson의 clock reset 위치가 GUI와 다름 | 반복·queue 경계의 최종 TX/RX 완료와 시계 재설정 순서를 일치 | 마지막 자세 유지 시간·실제 보행 주기가 달라짐. 고정 sleep을 추가하는 대신 원래 경계 의미를 맞춰 검증 |
| RX/FK/로그 때문에 TX가 크게 늦어짐 | 먼저 불필요한 FK·출력을 제어 흐름에서 줄이거나 지연 처리하고 통신 계획을 명시 | 목표 간격 변동 감소 가능. 무조건 피드백 삭제/병렬 포트 접근은 하지 않음. 우연한 홀드 시간을 줄여 보행이 오히려 빨라질 수 있음 |
| PID/모드/프로파일 값이 다름 | 안정적인 GUI 조건의 실제 readback에 맞춰 필요한 설정만 통일 | 응답 강성·감쇠·전류가 변할 수 있어 실기 검증 필요. 임의 게인값을 제안하지 않음 |
| 자연 종료와 cancel 정책이 다름 | 먼저 같은 종류의 정지끼리 비교. 필요하면 명시적인 정지 모션/전환 규칙 설계 | 몸통 운동과 지지 상태를 고려해야 함. 측정각 즉시 홀드를 안정적인 감속으로 간주하지 않음 |
| 저장된 조합에 원본 배속을 두 번 적용 | 이미 반영된 프레임 시간에는 저장된 전체 배속만 한 번 적용 | 발 들기/접지 타이밍 회복. 80% 옵션 및 정수 ms 경계 비교 |

허용오차를 더 작게 만들기, 모든 프레임에 도착 대기를 추가하기, 매 tick/프레임마다 실제각으로 보간 시작점을 바꾸기는 이번 조사에서 정당화되지 않았다. GUI 정상 재생에도 그런 처리는 없으며, 적용하면 접지·스윙 위상과 주기를 바꿀 수 있다.

남은 핵심 증거는 최신 Jetson 소스/바이너리와 실제 runtime JSON 식별, 실기의 queue 성공 여부, 경계 전후 TX/RX/시계 재설정 기록, 정지 이벤트 종류다. 그 기록 전에는 “오차 누적 때문에 넘어진다” 또는 “60° 시작점을 쓰기 때문에 넘어진다”라고 결론내릴 수 없다.
