요청하신 GUI 및 C++ 모션 SDK 원본을 전달합니다. 이 자료는 2026-09-30 13:54:48 KST에 `/home/rnd_3/final step`의 작업 파일을 복사한 것입니다. **실제 모터 통신 구현은 포함되어 있지만, 현재 C++ SDK의 재생 동작은 최신 GUI와 일치하지 않습니다.** 이 전달본은 동작을 맞춘 수정판이 아니라 현재 구현을 확인할 수 있는 원본과 비교 자료입니다.

Git 기준 커밋은 `72a52243ec7eee737ec52c43bdbdda0f724e94b7`(2026-08-29)이며, GUI와 C++ 등에 커밋되지 않은 변경이 있습니다. 커밋 해시만으로 이 전달본을 재현할 수 없으므로 `SOURCE_MANIFEST.json`의 파일별 SHA-256과 원본 수정 시각을 기준으로 확인해 주세요. 원격 저장소의 7월 16일 코드나 기존 `robot_motion_player_sdk_20260730.tar.gz`를 최신 원본으로 사용하지 않았습니다.

전달 파일은 다음과 같습니다.

| 경로 | 내용 |
|---|---|
| `gui/` | 최신 `sdk_gui.py`, 저장 상태, 시간 보정 데이터, Python 테스트, 읽기 전용 진단 도구 |
| `cpp_sdk/` | `RobotMotionPlayer`, JSON 로더, 실제 `DynamixelMotionHardware`, 컨트롤러, 저수준 통신, CMake, 테스트, 기존 내보내기 JSON |
| `data/robot_motions_from_gui_checked.json` | 동봉된 GUI 저장 상태의 체크된 85개 모션을 GUI의 실제 export 함수로 새로 생성한 JSON |
| `data/export_comparison.json` | 기존 JSON과 새 export의 차이 및 변경 모션의 양쪽 데이터 |
| `ENVIRONMENT.json` | 조사한 컴파일러·Python·패키지·USB latency 정보 |
| `verification/` | 빌드·테스트·API 지원·JSON 로드 검사 소스 및 로그 |
| `SHA256SUMS` | 전달 파일 무결성 확인용 목록 |

GUI 폴더의 `step.urdf`는 원본 작업 폴더에도 없습니다. 따라서 FK 모델 파일은 제공하지 못하며 현재 GUI의 3D 표시도 비활성화되어 있습니다. 외부 ROBOTIS C++ SDK 자체의 소스와 시스템 라이브러리는 이 프로젝트에 포함되어 있지 않아 동봉하지 않았습니다. 프로젝트의 실제 모터 통신 구현과 외부 ROBOTIS SDK는 구분해 주세요.

모션 데이터에서도 차이 하나를 확인했습니다. 기존 `cpp_sdk/robot_motions.json`과 동봉 GUI의 체크 목록은 모두 85개지만, `찐찐라인복귀좌회전45도(4회)`는 최신 GUI와 달랐습니다.

| 항목 | 기존 JSON | 동봉 GUI 기준 새 export |
|---|---|---|
| 배속 | 1.1 | 1.0 |
| 반복 | 4 | 4 |
| 프레임 시작/길이(ms) | 110/60, 170/150, 469/45, 514/80 | 80/65, 145/150, 445/55, 500/100 |
| 마지막 프레임 종료 | 594ms | 600ms |

원본 파일은 그대로 동봉했으며 최신 GUI 상태와 비교할 때는 `data/robot_motions_from_gui_checked.json`을 사용하세요. 새 export는 복사한 데이터에만 적용했고 원본 GUI 상태나 기존 배포 JSON은 덮어쓰지 않았습니다. 두 JSON 모두 실제 C++ JSON 로더가 85개 모션을 읽는 것을 확인했습니다. 로드 성공은 재생 동작 일치를 뜻하지 않습니다.

GUI와 C++의 동작 차이는 다음과 같습니다. C++ 비교 대상은 `main.cpp → RobotMotionPlayer → DynamixelMotionHardware`의 실제 실행 경로입니다. 별도의 구형 `MotionCallback`/`MotionPattern::sample()` 경로와 혼동하면 안 됩니다.

| 확인 항목 | 최신 GUI | 동봉 C++ RobotMotionPlayer |
|---|---|---|
| 궤적 생성 | PC에서 half-cosine + 최단 각도차로 중간 목표 계산 | 프레임의 최종각과 남은 이동시간을 모터에 전달 |
| 시작 자세 | 새 재생 직전에 대상 관절 Present Position 읽기 | 하드웨어 초기화 때 현재각 읽음. 각 `start()`에서 PC 보간용 시작각을 다시 읽는 경로 없음 |
| 발 들기 80% | `lift_early_arrival=true`이고 이름에 `발들/들기/오들/왼들` 포함 시 적용 | 해당 필드를 로드하지 않으며 80% 처리 없음 |
| 모터 프로파일 | Acceleration/Velocity=0 후 중간 목표 반복 송신 | 프레임마다 Acceleration/Duration/Goal을 순서대로 송신. duration은 최소 1ms |
| `[착지]` | 별도 내부 프로파일 함수는 있으나 현재 실행에서 호출하지 않음 | 실제 이동시간의 10%, 최대 30ms 가감속 적용 |
| 목표 전송 주기 | QTimer 5ms 목표. 동기식 통신 지연에 따라 실제 주기는 늘어남 | 프레임 시작 시 최종 목표 전송. `main.cpp`는 `update()` 후 2ms sleep. 200Hz 중간각 송신이 아님 |
| 위치 피드백 | 시퀀스는 10ms 간격 조건, 단일 프레임은 매 제어 호출 | 프레임 끝/최종 Settling에서 20ms 간격 조건 |
| 프레임 끝 | 최종 목표를 보내고 도착 대기 없이 진행 | 목표 허용오차를 만족할 때까지 시계 고정 후 대기 |
| 반복 경계 | 최종 전송·피드백 처리 뒤 반복 시계 재시작 | 마지막 프레임의 도착 gate를 통과한 뒤 반복 시작. 반복마다 최종 Settling 상태에 들어가지는 않음 |
| 조합 내부 경계 | `playback_cycle_end`로 원본 반복 경계를 보존 | 해당 필드를 로드하지 않음 |
| 전체 종료 | 마지막 목표 유지, 위치 도착 대기 없음 | 예약 전환이 없으면 최종 Settling에서 오차·연속 안정시간 확인 |
| 프레임 `torques` | 저장되지만 시퀀스 재생에서 프레임별 토크 전환에 쓰지 않음 | 파싱되지만 RobotMotionPlayer의 프레임 전송에서 토크 전환에 쓰지 않음 |
| 송신 실패 | 재생 중단·일시정지 경로 | `FrameSendFailed`로 Failed. 모든 실패에서 자동 홀드하는 것은 아님 |
| 위치 읽기 실패 | 재생 피드백 실패만으로 자동 중단하지 않음 | 프레임 gate에서는 timeout까지 재시도 후 홀드 시도/실패. 최종 Settling 읽기 실패는 즉시 Failed |
| 일시정지/취소 | 활성 실기 프레임에서 현재 피드백 자세 홀드 후 일시정지/중지 | pause/resume API 없음. `cancel()`은 현재 자세를 읽어 50ms 시간 프로파일로 홀드 |
| 종료 시 토크 | GUI 종료는 포트를 닫으며 자동 torque OFF하지 않음 | `shutdown()`/소멸자 경로가 torque OFF 수행 |
| PID/Operating Mode | 현재 GUI는 값을 매번 설정하지 않음 | Dxl 생성자에서 Position Mode=3 및 P/I/D=850/0/0 설정 시도 |

GUI 조합은 원본 모션 배속과 조합 전체 배속을 곱합니다. 조합 저장 시 원본 배속을 시작/끝 시각에 반영하고 전체 배속만 JSON에 남깁니다. C++는 이 저장된 시간과 JSON의 배속을 한 번 적용해야 합니다. 원본 배속을 추가로 재적용하면 안 됩니다. 일반 공백은 직전 자세를 유지하며 첫 공백도 대기 시간입니다. 현재 C++의 도착 gate 때문에 프레임 시간이 끝난 뒤 대기가 추가될 수 있어, 같은 JSON과 배속만으로 GUI 실행 시간을 재현하지 못합니다.

요청하신 API는 실제 헤더를 대상으로 모터 없이 컴파일한 `verification/api_probe.cpp`로 확인했습니다.

| API | 지원 여부 및 의미 |
|---|---|
| `startPoseTransition(angles, duration_ms)` | 없음 |
| `updateStartupPose()` | 없음 |
| `queueNext()` | 있음. 예약은 1개. 같은 repeatable 모션 또는 end/start pose 이름이 맞는 모션을 허용 |
| `completionSequence()` | 있음. 예약 모션 전환 및 최종 정상 완료마다 증가 |
| `setPositionToleranceEnabled()` | 없음. 현 SDK에서 허용오차 gate를 끄는 공개 API가 없음 |
| `setJointOverride()` | 없음 |
| `clearJointOverride()` | 없음 |

`setJointCorrection`, `setFrameCorrection`은 있지만 목표각을 교체하는 override와 다른 가산 보정 API입니다. 서로 같은 기능으로 대체하면 안 됩니다. 또한 로컬 `jetson_reference/IRC_vision`의 production factory가 요구하는 `DynamixelMotionHardwareConfig`와 설정 객체를 받는 생성자가 동봉 SDK에는 없습니다. 이 ROS 참조 코드를 그대로 실제 SDK에 연결하면 API 호환 작업도 필요합니다. 전체 ROS 2 프로젝트 통합 빌드는 이번에 검증하지 않았습니다.

완료 조건에 대한 답변은 다음과 같습니다. 담당 프로젝트에서 사용 중이라고 알려주신 값은 5도/80ms/3000ms이지만, **동봉 원본 JSON과 새 export는 모두 2도/80ms/3000ms**입니다. 5도로 임의 변경하지 않았습니다.

- GUI는 이 값을 도착 대기에 사용하지 않습니다.
- C++의 `gateCurrentFrame()`은 매 프레임 끝에서 position tolerance와 timeout을 사용합니다. 이 단계에는 80ms 연속 안정시간 조건은 없습니다.
- C++의 `updateSettling()`은 전체 모션 종료 때 position tolerance, 80ms 안정시간, timeout을 모두 사용합니다.
- `queueNext()`는 호환 모션으로 최종 Settling을 생략하고 넘어갈 수 있지만, 그 전에 마지막 프레임의 도착 gate는 남아 있습니다. Settling 중 뒤늦게 예약한 모션도 다음 update에서 전환할 수 있습니다.
- 현재 코드에는 `setPositionToleranceEnabled(false)`가 없으므로 ROS 파라미터나 JSON 값만 바꿔 GUI와 같은 동작을 만들 수 없습니다. 80ms를 0으로 해도 위치 검사 자체는 남습니다.

GUI와 타이밍을 맞추려면 PC 중간각 송신 경로, 발 들기 규칙, 실제 시작각, 조합 내부 반복 경계, 원시 위치값 반올림을 구현하고, 프레임/최종 완료의 도착 대기 정책을 명확히 나눠야 합니다. 내부 duration 쓰기를 5ms마다 계속 사용하는 방식은 GUI의 프로파일 0 경로와 같지 않으며, 현재 저수준 `syncWriteTimeBasedTheta()`는 duration을 최소 1ms로 제한하므로 별도 경로가 필요합니다.

위치 도착 조건을 생략할 경우 완료 이벤트의 의미는 '실제 목표 도착 확인'에서 '계획한 목표 전송 종료'로 달라집니다. 다음 모션이 추종 지연이 남은 상태에서 시작할 수 있고, 위치 미도달 timeout으로 잡던 실패도 사라집니다. 송신 오류·피드백 통신 오류와 도착 검사 정책은 별개로 설계해야 합니다. GUI의 읽기 실패 후 계속 진행 동작까지 그대로 복제할지, ROS 측은 통신 실패를 별도 중단 사유로 유지할지도 구분해서 결정해야 합니다.

실물 설정에 관해서는 아래 범위까지만 확인했습니다.

- GUI 코드상 ID별 모델/관절 매핑은 `gui/sdk_gui.py`의 `joint_data`에 있습니다. 실제 모터를 새로 PING하여 확인한 모델·펌웨어 표는 아닙니다.
- 실제 현재 Operating Mode, Drive Mode, PID, 전류/PWM/위치 제한, Homing Offset을 이번에 모터에서 읽지 않았습니다. C++의 설정 명령과 실제 적용된 값을 동일시하지 마세요.
- C++ `zero_manual_offset`은 코드에서 23축 모두 0이며, 기구 기준의 실측 영점·방향 검증 자료는 없습니다.
- 조사한 PC의 `/sys/bus/usb-serial/devices/ttyUSB0/latency_timer`는 16ms였습니다. 대상 Jetson의 값은 별도 확인해야 합니다. 이 전달 작업에서 USB 설정을 변경하지 않았습니다.
- USB latency는 해당 장치의 sysfs `latency_timer`로 읽을 수 있고 드라이버가 허용하면 값을 쓰거나 장치에 맞는 udev 규칙으로 설정합니다. 이 프로젝트에는 검증된 배포용 udev 규칙이 없으므로, 대상 장치 식별과 지원 여부를 확인한 뒤 적용해야 합니다. 1ms 설정만으로 실제 200Hz가 보장되지는 않습니다.
- `찐찐전진45(8회)`의 이번 실기 타임스탬프/목표/현재각/통신 지연 로그는 없습니다. 모터 연결 없이 소프트웨어만 검증했습니다.

검증 결과는 **Python 43개 테스트 통과, Debug C++ 빌드 성공, CTest 3개 통과, 기존/새 JSON 각각 85개 모션 로드 성공**입니다. 실제 `robot_control` 라이브러리와 `irc_robot` 실행 파일까지 컴파일·링크했지만 실행하지 않았습니다. 이 결과는 GUI–C++ 궤적 일치나 ROS 실행 호환성을 인증하는 결과가 아닙니다. 현재 차이를 제거한 뒤 같은 초기각·가상 시계·통신 지연 조건에서 목표각과 이벤트 시각을 비교해야 합니다.
