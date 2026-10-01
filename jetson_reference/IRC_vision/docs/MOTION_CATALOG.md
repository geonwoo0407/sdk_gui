# Motion Catalog

알고리즘과 motion backend 사이에서 사용할 표준 행동 이름이다. 현재 개발
대상은 SDK 실행 계약이며, 아래 `motion_id`는 실제 SDK 번호가 아니라 mock
Motion Executor가 사용하는 추상 문자열이다.

## SDK Executor 계약

| 표준 행동 | 현재 알고리즘 명령 | legacy adapter motion_id | 현재 backend |
|---|---|---|---|
| WALK_FORWARD | STRAIGHT | `forward` | mock |
| WALK_APPROACH | APPROACH | `forward` | mock |
| WALK_SLOW | SLOW_APPROACH | `forward_short` | mock |
| WALK_FINE | FINE_FORWARD_STEP | `forward_short` | mock |
| TURN_LEFT | TURN_LEFT | `turn_left` | mock |
| TURN_RIGHT | TURN_RIGHT | `turn_right` | mock |
| LINE_LEFT | LEFT | `turn_left` | mock |
| LINE_RIGHT | RIGHT | `turn_right` | mock |
| ADJUST_LEFT | ALIGN_LEFT | `adjust_left` | mock |
| ADJUST_RIGHT | ALIGN_RIGHT | `adjust_right` | mock |
| WALK_BACKWARD | RETREAT_GOAL | `backward` | mock |
| PICKUP | PICKUP_NOW | `pick_ball` | mock |
| SHOT | SHOT | `shoot` | mock |
| HURDLE_APPROACH | APPROACH_HURDLE | `forward_short` | mock |
| HURDLE_CROSS | GO | `forward` | mock |
| HEAD_LEFT | HEAD_SCAN_LEFT | 미매핑 | 미지원 |
| HEAD_RIGHT | HEAD_SCAN_RIGHT | 미매핑 | 미지원 |
| HEAD_CENTER | HEAD_CENTER | 미매핑 | 미지원 |
| FINE_LEFT | FINE_LEFT | 미매핑 | 미지원 |
| FINE_RIGHT | FINE_RIGHT | 미매핑 | 미지원 |
| STOP | STOP | 미매핑 | 미지원 |
| CROSS_FINISH | CROSS_FINISH | `hurdle` | mock 호환 |

`BALL_LOST_STOP`, `GOAL_LOST_STOP`, `WAIT_SCORE_CONFIRMATION`,
`WAIT_GO_CONFIRMATION`과 세 `HEAD_*` action은 recovery FSM 상태를 표현하지만
production mission planner에서는 `valid=false`이므로 Executor 요청을 만들지
않는다. goal recovery의 몸 회전은 `LEFT`/`RIGHT`로 정규화되어 기존
`turn_left`/`turn_right` mapping을 사용한다.

`player_backend=sdk`도 실제 SDK adapter가 아니라
`SdkMotionPlayerPlaceholder`를 선택한다. 이 placeholder는
`hardwareReady=False`를 반환하므로 어떤 `motion_id`도 실행하지 않는다.

## Dynamics 직접 제어 계약

Dynamics command 변환은 별도의 `MotionCommandBridgeNode` 경로이다. 현재
프로젝트 단계의 SDK 실행 계약 범위에는 포함하지 않으며, 위 mock
`motion_id`와 Dynamics command를 서로 대응되는 번호로 해석하지 않는다.

## 원칙

- mock motion ID를 실제 SDK 모션 번호로 간주하지 않는다.
- SDK와 Dynamics 직접 제어 계약을 같은 경로로 간주하지 않는다.
- 현재 SDK player backend는 placeholder이며 실제 SDK 호출을 하지 않는다.
- STOP은 일반 모션 이름이 아니라 별도 안전 정지 API가 될 수 있다.
- 하나의 행동 요청에는 완료 또는 실패 상태가 정확히 한 번 반환되어야 한다.
