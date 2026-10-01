# Motion Decision 역할과 구현 범위

## 현재 구현된 범위

`motion_decision_node`는 네 비전 정보 토픽과 `/mission/phase`를 받아
한 주기에 하나의 명령만 `/navigation/motion_command`로 발행합니다.

- 입력 freshness 검사와 오래된 검출 제외
- line, ball, goal, hurdle planner 선택
- 명시적 phase와 시험용 `AUTO` 우선순위 처리
- 공·골대·허들 거리 및 정렬 조건에 따른 접근/단발 모션 후보 선택
- PICKUP, SHOT, GO 조건의 단발 latch와 event ID 발행
- SDK 모션 실행 중 사용할 `*_LOCK` 단계의 WAIT 처리
- 선택 근거와 원본 planner 결과를 JSON 명령에 포함

## 친구가 제안한 최종 FSM과의 대응

제안된 motion decision의 큰 구조는 현재 패키지 분리와 잘 맞습니다.
비전 검출과 좌표 계산은 `step`, 아래 상태 판단은 `mission_control`,
실제 모터 시퀀스는 SDK/C++ 실행기가 맡습니다.

아래 항목은 최종 모션 계약이 정해진 뒤 추가해야 합니다.

- `total_fail_count >= 3` 무한 루프 탈출 동작
- 시작 시 강제 전진 모션과 IMU 영점 보정
- 실제 SDK motion ID 표와 각 명령의 반복 횟수
- 모션 완료/실패/timeout 피드백 토픽
- 공 줍기와 슛의 재시도 및 성공 판정
- 허들 모션 단계별 감쇠와 통과 후 yaw 복구
- 미션 완료 후 목 스캔과 다음 미션 전환
- 모든 객체를 잃었을 때 IMU와 마지막 라인 방향을 이용한 복귀

이 항목은 비전 노드에 넣으면 안 됩니다. 실제 모션 상태와 미션 진행
상태를 알아야 하므로 `mission_control`의 FSM과 SDK/C++ 실행기 사이의
명확한 명령/응답 계약으로 구현해야 합니다.

## 권장 SDK 계약

`/navigation/motion_command`에는 최소한 다음 값이 필요합니다.

- `command_id`, `event_id`: 중복 실행 방지
- `source`, `action`: 선택된 미션과 추상 행동
- `sdk_motion_id`: 확정된 실제 모션 번호
- `requires_ack`: 완료 응답 대기 여부
- `source_command`: 거리, 각도, 정렬 상태 등 상세 근거

향후 SDK 실행기는 별도 상태 토픽으로 다음 정보를 회신하는 것이
안전합니다.

- 수신한 `command_id` 또는 `event_id`
- `RUNNING`, `SUCCEEDED`, `FAILED`, `TIMEOUT`
- 실패 횟수와 선택적 실패 원인

`mission_control`은 이 응답을 받은 뒤에만 lock을 해제하고 다음 phase로
넘어가야 합니다.
