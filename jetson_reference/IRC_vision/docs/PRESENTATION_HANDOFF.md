# IRC STEP 비전 프로젝트 발표 자료 인계 문서

이 문서는 ChatGPT 등에게 그대로 전달해 PPT 초안을 만들 수 있도록 프로젝트의 배경, 개발 과정, 현재 구조, 핵심 알고리즘, 시연 방법과 남은 과제를 압축한 자료입니다.

## 1. 발표 제목 제안

**RealSense RGB-D와 YOLO26을 이용한 휴머노이드 로봇 미션 비전 통합**

부제: **라인 주행·공 수거·골 넣기·허들 넘기를 위한 ROS 2 인식 및 의사결정 구조**

## 2. 문제 정의

IRC 휴머노이드 로봇이 경기장에서 다음 미션을 수행하려면 카메라 영상으로 주변 상황을 이해하고 모션 알고리즘에 일관된 정보를 전달해야 합니다.

- 바닥 line을 따라 이동
- 공을 탐색하고 중앙 정렬한 뒤 접근
- 골대 또는 backboard를 기준으로 골 넣기 위치 정렬
- 허들에 정렬하고 안전한 거리에서 넘기 모션 준비

초기 OpenCV 색상·윤곽선 방식은 조명, 그림자, 바닥 색상과 배경 변화에 민감했습니다. 이를 개선하기 위해 한 YOLO26 모델에서 `line`, `ball`, `goal`, `backboard`, `hurdle`을 동시에 탐지하고 RealSense depth를 결합했습니다.

## 3. 개발 과정

1. `find_direct`, `find_ddirect`, `look_ground`, `look_gground`로 OpenCV 기반 방향/바닥 탐색을 실험했습니다.
2. YOLO26 ONNX detector와 RealSense RGB 토픽을 연결했습니다.
3. line 객체의 중심점을 가까운 순서로 연결하고 잘못 떨어진 점을 continuity filter로 제거했습니다.
4. line heading, lateral offset, 먼 경로 curve, 품질 지표와 temporal filter를 구현했습니다.
5. ball에 aligned depth와 camera intrinsics를 결합해 실제 거리와 좌우 오차를 계산했습니다.
6. 같은 패턴으로 goal/backboard 및 hurdle analyzer와 planner/controller를 구현했습니다.
7. 각 미션의 계산 로직을 하드웨어 독립 planner로 분리하고 pytest로 경계값을 검증했습니다.
8. 네 analyzer를 한 프로세스로 실행하고 네 planner를 하나의 `motion_decision_node`에서 선택하도록 통합했습니다.
9. YOLO 화면이 실제 선택된 planner에 맞춰 line 전체 경로 또는 객체별 metrics를 자동 표시하도록 개선했습니다.

초기 OpenCV 파일은 삭제하지 않고 `archive/legacy_opencv`로 이동해 개발 이력을 보존했습니다.

## 4. 전체 시스템 구조

```text
RealSense D435i
 ├─ /camera/color/image_raw
 ├─ /camera/aligned_depth_to_color/image_raw
 └─ /camera/color/camera_info
          ↓
 yolo26_detector ──→ /vision/detections
          ↓
 unified_vision_node
 ├─ yolo_line_analyzer  → /vision/line_info
 ├─ ball_analyzer       → /vision/ball_info
 ├─ goal_analyzer       → /vision/goal_info
 └─ hurdle_analyzer     → /vision/hurdle_info
          ↓
 motion_decision_node ← /mission/phase
          ↓
 /navigation/motion_command
          ↓
 C++ STEP SDK motion node (추후 연결)
```

`yolo26_detector`는 영상 한 프레임에서 모든 클래스를 한 번에 추론합니다. `unified_vision_node`는 기존 analyzer 파일을 유지하면서 한 프로세스에서 실행합니다. `motion_decision_node`는 현재 미션에 필요한 planner 하나만 선택합니다.

## 5. 공통 단위와 좌표계

- 영상 좌표: pixel, 원점은 좌측 상단
- 카메라 optical 좌표: x 오른쪽, y 아래, z 전방
- 화면 중심 오차: `px`와 정규화 비율
- 실제 거리와 위치: 내부 ROS JSON에서는 `m`
- SDK가 mm를 요구하면 최종 경계에서 `m × 1000`
- 각도: `deg`, 오른쪽이 양수인 화면/카메라 기준

## 6. Line 로직

### 입력과 분석

YOLO가 여러 개의 `line` 조각을 검출하면 bbox 중심점을 화면 아래쪽부터 위쪽, 즉 near-to-far 순서로 정렬합니다. 멀리 떨어진 오검출점은 path continuity filter로 제거합니다.

계산값:

- 현재 경로 heading
- 화면 중심 대비 lateral offset
- near/far heading 차이와 turn angle
- turn consistency
- heading, geometry, detection quality
- median 및 EMA 기반 filtered heading/offset

### Planner

조향 오차는 heading, offset, 신뢰할 수 있는 먼 경로 preview를 합쳐 계산합니다.

```text
steering_error
= heading_gain × heading
+ offset_gain × lateral_offset
+ preview_gain × far_turn
```

라인 명령은 `STRAIGHT`, `FINE_LEFT`, `FINE_RIGHT`, `LEFT`, `RIGHT`,
`STOP`입니다. 중간 편차는 `FINE_LEFT/RIGHT`, 큰 편차와 라인 상실은
`LEFT/RIGHT`를 사용합니다. `FINE_LEFT/RIGHT`는 Executor 매핑이 확정되지
않아 현재 명시적 미지원이며, quality가 낮거나 복구 한도를 넘으면 STOP합니다.

### 화면

line planner 선택 시 YOLO bbox 라벨 대신 다음을 표시합니다.

- 초록 near-to-far 경로선
- 경로점 번호와 NEAR/FAR
- 카메라 중앙선
- heading 화살표
- lateral offset
- 실제 planner action과 quality

## 7. Ball 로직

ball bbox 중심 주변 depth patch의 median을 사용합니다. 카메라 intrinsics로 bearing, lateral/vertical 위치, 수평거리와 3D 직선거리를 계산합니다.

상태:

```text
SEARCH → NO_DEPTH/FAR/TRACK/APPROACH → PICKUP_READY → PICKUP_NOW
```

현재 기본값과 우선순위:

- 공 모드에서만 카메라 높이 0.515m, 공 지름 0.060m, 공 상단 높이
  0.065m를 사용한다. 공 중심 높이 0.035m를 뺀 세로거리 0.480m와
  표면 Depth Z에 반지름 0.030m를 더해 구한 카메라-공 중심 3D
  직선거리에 피타고라스 정리를 적용해
  `ground_distance_m`을 계산
- 바닥거리 1.50m 안에서 ball planner로 전환하고 추적 위치를 기억
- 좌우 오차가 크면 제자리 `TURN_LEFT/RIGHT`
- raw Depth 0.60m 이내 공의 가로 offset norm이 ±0.05 이내면 근거리
  path angle과 관계없이 중앙 처리
- 공 전용 로봇 중심선은 화면 중앙에서 오른쪽 +96px이며 라인 중심선
  보정값과 분리
- 정렬되면 바닥거리별 `STRAIGHT_0..5` 또는 `STRAIGHT` 선택
- 집기 트리거: raw Depth Z 0.50m 이하(기준 0.48m + 허용오차 0.02m)
- 화면 목표: 가로 중앙 ±0.08, 화면 높이 0.82 ±0.12
- 조건 충족 시 `PICKUP_NOW`

현재 production 기본값은 `enable_ball_lost_recovery=true`입니다. 공 분실 복구 FSM은 유지되지만 `BALL_LOST_STOP`, `HEAD_SCAN_LEFT/RIGHT`, `HEAD_CENTER`는 현재 SDK 실행 계약이 없어 `valid=false`인 비실행 판단 단계입니다. 필요하면 parameter를 `false`로 지정해 분실 복구 판단을 비활성화할 수 있습니다. 아직 실제 공 줍기 SDK를 호출하지 않습니다.

## 8. Goal 로직

큰 `goal` bbox 안에 `backboard`가 함께 검출되면 backboard 중심과 bbox를 조준 기준으로 우선 사용합니다. goal 중앙은 빈 공간이라 depth가 불안정할 수 있기 때문입니다. 기준 bbox 안 5개 위치의 유효 depth median으로 대표 거리를 정합니다.

현재 임시 득점 조건:

- Depth Z 3.0m 안에서 backboard 위치 기억
- Depth Z 0.50~3.0m에서는 line 주행 유지
- Depth Z 0.50m 안에서 goal planner로 전환
- 목표 depth: 0.25m
- 거리 허용오차: ±0.05m
- 중심 허용오차: ±0.10

Planner 명령은 `ALIGN_LEFT/RIGHT`, `APPROACH_GOAL`, `RETREAT_GOAL`, `SHOT`, `WAIT`입니다. 조건 충족 시 화면에 `SHOT`이 표시됩니다.

추적한 골대를 잃으면 `GOAL_LOST_STOP`으로 정지한 뒤 마지막 backboard 방향으로 `RECOVER_GOAL_TURN_LEFT/RIGHT` 제자리 회전을 수행합니다. 재검출 후에도 중앙 `bearing ±5deg` 안에 올 때까지 회전하며, 8초 동안 찾지 못하면 기억을 해제하고 line으로 돌아갑니다.

## 9. Hurdle 로직

허들 bbox 가로 방향 5개 depth를 측정해 대표 depth, 좌우 depth, 추정 폭과 카메라-허들 평행 오차를 계산합니다. depth와 픽셀 좌표로 카메라-허들 3D 직선거리를 구한 뒤, 카메라 높이 0.70m와 허들 측정점 임시 높이 0.10m를 이용해 피타고라스 정리로 카메라 바로 아래 바닥점부터 허들까지의 간격을 계산합니다. 불가능한 기하 상태를 0m로 처리하지 않으며, 화면 하단과 허들 bbox 하단의 픽셀 간격도 depth로 길이 환산해 조기 GO를 차단합니다. 허들은 로봇이 화면 중앙으로 통과할 필요가 없으므로 bbox 중심과 카메라 중심 사이의 수평 오차는 정렬 조건으로 사용하지 않습니다.

현재 임시 GO 조건:

- 목표 바닥 간격: 0.10m
- 바닥 간격 허용오차: ±0.10m
- 화면 하단-허들 depth 환산 간격: 0.05m 이하
- 평행 각도 허용오차: ±8°

Planner는 좌우 끝 depth 차이로 계산한 평행 오차가 ±8°를 벗어나면 `ALIGN_LEFT/RIGHT`, 평행한 상태에서 바닥 간격이 크면 `APPROACH_HURDLE`, 평행하면서 바닥 간격이 0~0.20m에 들어오면 `GO`를 냅니다. 허들의 화면 좌우 위치는 이 판단에 영향을 주지 않습니다.

최종 모션이 허들에 가까이 붙어서 천천히 넘는 방식으로 정해지면 `LOOK_DOWN → CREEP → bottom_gap_px → GO` 상태를 추가할 예정입니다. `bottom_gap_px`는 카메라 자세를 고정했을 때 화면 하단과 hurdle bbox 하단 사이의 픽셀 간격입니다.

## 10. 통합 의사결정

`mission_control` 패키지의 `motion_decision_node`는 네 입력의 최신성을 검사하고 `/mission/phase`에 맞는 planner를 선택합니다. 카메라·YOLO·기하 계산은 `step`, 미션 우선순위와 단일 명령 선택은 `mission_control`이 담당합니다.

- `*_SEARCH`: 목표가 없으면 line을 따라가며 탐색
- `*_APPROACH`: 해당 객체에 집중
- `*_LOCK`: SDK 모션 완료를 기다리며 새 명령 차단
- `AUTO`: 공은 0.90m, 골대는 0.50m 안에서만 line보다 우선하며 각각 3m 안에서는 위치만 기억하는 시험 모드

`PICKUP_NOW`, `SHOT`, `GO`가 여러 프레임 유지돼도 `sdk_motion_requested=true`는 조건 진입 시 한 번만 발행합니다. `command_id`와 `event_id`로 연속 제어 명령과 단발 모션 요청을 구분합니다.

## 11. 시연 실행

필수 터미널은 4개입니다.

```bash
# 1. RealSense
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  align_depth.enable:=true enable_gyro:=true enable_accel:=true
```

```bash
# 2. YOLO 및 통합 화면
cd ~/my_cv
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run step yolo26_detector --ros-args -p metrics_mode:=auto
```

```bash
# 3. 네 analyzer를 한 프로세스로 실행
cd ~/my_cv
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run step unified_vision_node
```

```bash
# 4. 통합 planner 선택
cd ~/my_cv
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run mission_control motion_decision_node
```

```bash
# 미션 변경 및 출력 확인
ros2 topic pub --once /mission/phase std_msgs/msg/String "{data: 'AUTO'}"
ros2 topic echo /navigation/motion_command
```

## 12. 검증

- planner는 ROS나 하드웨어 없이 pytest로 검증 가능
- line 방향, hysteresis, quality fail-safe 검증
- ball 정렬/접근/집기 조건과 각가속도 제한 검증
- hurdle GO 경계 0.70m/0.90m 포함 여부 검증
- 통합 phase 선택, SEARCH의 line fallback, LOCK 대기 검증
- Python compile, flake8, colcon build 수행

## 13. 현재 한계와 다음 작업

1. C++ STEP SDK가 받는 실제 모션 번호표를 확정해야 합니다.
2. Python → C++ 메시지의 `motion_id`, `angle`, `custom_param` 계약이 필요합니다.
3. C++ → Python `busy`, `motion_end`, `success`, `fail` 응답이 필요합니다.
4. 공 줍기, 슛, 허들 넘기 동안 perception 명령을 막는 mission lock을 실제 응답과 연결해야 합니다.
5. 허들 하향 카메라 각도와 최종 `bottom_gap_px`를 실측해야 합니다.
6. 모션 실패 시 후진·목 스캔·재시도와 `total_fail_count` 탈출을 구현해야 합니다.
7. 프레임 개수보다 실제 시간 기반 detection debounce를 적용해야 합니다.

## 14. PPT 구성 제안

1. 대회 미션과 문제 정의
2. 초기 OpenCV 방식과 한계
3. YOLO26 + RealSense RGB-D 전환
4. 전체 ROS 2 아키텍처
5. line 경로 복원과 planner
6. ball 거리·중심 정렬
7. goal/backboard 조준
8. hurdle 폭·기울기·GO 조건
9. 통합 노드와 mission phase 선택
10. 실시간 시각화 및 시연 화면
11. 테스트 결과와 fail-safe
12. 현재 한계 및 SDK 연동 계획

PPT에는 각 미션의 YOLO 화면 캡처, line 전체 경로 시각화, object metrics 패널, `/navigation/motion_command` 예시를 함께 넣으면 좋습니다.
