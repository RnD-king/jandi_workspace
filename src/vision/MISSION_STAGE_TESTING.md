# 미션별 구간 시험

이 문서는 수동 승인/단계별 시험의 단발 실행 계약을 설명한다. 실전 AUTO는 T/F 선택 없이
LINE·원거리 객체에 공통 32/33/34 및 기본 2/2/2를 사용한다. 아래 날짜별 기록은 시험 경로의
변경 이력을 포함하며 최신 실전 정책은 코어 CONTINUOUS_OBJECT_APPROACH.md를 따른다.


PC에서 vision을 실행하고, 실제 몸/카메라 ACK·READY·DONE은 forward_walk에서 받는다.
알고리즘은 모든 노드에서 같은 vision_core를 사용한다. 전용 노드는 다음 세 개다.

LINE → BALL 진입은 `ball.stable_window` 내에서 공 중심의 v가
`ball.upper_acquire_v_px` 이하인 검출 프레임을 세어 판단한다
(운영 설정: 중심 v≤470인 검출이 10프레임 중 6회).
현재 프레임 검출과 접근 추적의 7회 안정 조건은 진입에 요구하지 않는다.
검출 confidence·박스 크기 필터는 그대로 사용한다.
이미 예약된 다음 LINE 보행은 유지하며, 그 보행이 실행되면 BALL 진입을
다시 판단한다. 공을 들고 있거나 PAUSE·최종 정지 상태일 때는 새 진입을 막는다.
FINE 및 DONE 후 공통 관측의 10/7 기준은 유지한다.
실전·튜닝 공통 디버그의 `ENTRY hits/need/frames`로 진입 이력을 확인한다.

LINE 직진은 23번(4걸음)을 사용하며, READY 관측·예약 시점은 유지한다.
공 픽업 후 GOAL 전환의 직진 완료 판정도 23번 DONE을 사용한다.

BALL FINE은 기존 좌우/앞뒤 우선순위로 횡이동이 선택된 경우에만
`ball.fine_side_two_step_margin_px` 이상 픽업 좌우 경계를 벗어나면
같은 방향의 5번/6번을 2회 연속 실행한다(기본 100px).
첫 DONE 뒤 재관측·추가 안정 대기는 생략하고, 두 번째 DONE 뒤 공통 관측을 한다.
튜닝에서도 한 번의 COMMIT으로 두 모션을 실행한다.
ROS/rqt 이름은 `ball_fine_side_two_step_margin_px`이다.
실전 노드는 WAIT_START 또는 PAUSED에서 몸·카메라 DONE 후 편집할 수 있고,
튜닝 노드는 기존 WAIT_PREVIEW/WAIT_COMMIT 편집 규칙을 따른다.

BALL 진입 검출의 실제 좌우 평균을 별도로 기억한다. 진입 후 첫 정지 관측에서
공을 못 찾거나 일반 FORWARD 접근 중 `ball.lost_frames` 연속 미검출이면
같은 전진 탐색 흐름을 사용한다. 좌우 방향이 있으면 WALK 제자리 회전(좌31/우28)
1회 → 전진 1걸음(10번) → 정지 재관측이다. 중앙 근처/방향 미확인은 먼저
정지 관측하고, 실패하면 회전 없이 전진 1걸음 → 정지 재관측을 반복한다.
각 관측은 `ball.recovery_observation_sec`(현재 1초)와 새 10프레임/7회 검출을
모두 충족해야 성공이다. 회전 중 표본은 전진 후 관측에 재사용하지 않는다.

일반 전진 한도는 `ball.recovery_max_steps`(현재 3회), 진입 직후 한도는
`ball.entry_recovery_max_steps`(현재 3회)로 독립 설정한다. 전진의 실제 DONE만
횟수에 포함한다. 진입 직후 한도가 소진되면 공 없이 LINE을 재획득하고 정상
라인 보행으로 복귀한다. 일반 접근 한도 소진 후에는 기존 정지 관측을
`ball.recovery_max_observations`회 수행하고, 계속 실패하면 FAILED다.
옛 `entry_recovery_walk_turn_max`는 `entry_recovery_max_steps`로 대체했다.
ROS/rqt 이름은 `ball_recovery_max_steps`, `ball_entry_recovery_max_steps`이며,
실전에서는 재시작 후 적용하고 튜닝에서는 기존 편집 규칙을 따른다.

중간 좌우 구역(4·6) 접근 뒤 관측 실패는 전진 탐색과 별도다. 기존대로 해당
방향으로 회전 후 제자리 관측하며 최초 회전을 포함해 `ball.recovery_walk_turn_max`
(현재 3회)까지만 회전한다. 근거리 좌우(7·9)의 추가 회전 후 DOWN 전환도 유지한다.

HURDLE 진입 직후 첫 관측 실패는 `hurdle.entry_recovery_max_steps`(현재 3회)
범위에서 직진 1걸음과 정지 관측을 반복한다. 소진하면 정상 LINE을 재획득하고
보행으로 복귀한다. DOWN 접근·FINE에서 15프레임 연속 유실되면 각각 일반
1걸음·미세 전진 1회를 `hurdle.recovery_max_steps`(현재 3회)까지 반복하고,
소진 후에는 기존 정지 관측 최대 5회, 이후 FAILED다. 매 실제 DONE 뒤 새
10프레임/7회 검출과 `recovery_observation_sec`(1초)를 확인한다.
ROS/rqt 이름은 `hurdle_entry_recovery_max_steps`, `hurdle_recovery_max_steps`다.

GOAL의 완전히 빈 첫 관측은 카메라 FORWARD → 새 LINE 1블록 → GOAL 재관측을
`goal.entry_line_retry_max_attempts`까지 수행한다. 이 설정이 0이거나 이미
백보드를 본 뒤 유실되면 저장된 방향을 사용한다. 중앙 유실은 WALK 일반 1걸음(10),
FINE 미세 전진 1회(3) 후 관측한다. 좌우 유실은 최대 `goal.recovery_turn_max`회
회전(WALK 좌31/우28, FINE 좌7/우8 15도)하며, 실제 DONE 뒤 관측에서 다시 찾으면
저장된 복구 방향으로 추가 1회 회전 → 새 관측 → 기존 단계 복귀다. 전진·추가 회전
뒤에도 못 찾으면 기존 정지 복구로 넘어간다. 방향 기록 없음과 영상은 보이지만
Depth·자세가 실패한 경우는 정지 관측한다. 화면 끝 고정 2회전은 별도다.

| 노드 | 미션 |
|---|---|
| ball_tuning_node | 공 |
| hurdle_tuning_node | 허들 |
| goal_tuning_node | 골대 |

`test_stage`는 WAIT_START에서 rqt로 바꾸며 정수 0..3이다.

| 값 | 시험 시작과 종료 |
|---|---|
| 0 | 전체: 큰 접근부터 미션 및 일반 보행 자세 복귀까지 |
| 1 | 큰 접근: BALL/HURDLE은 FORWARD 접근 후 DOWN 전환까지. GOAL은 GOAL 카메라부터 접근해 GOAL_TO_FINE_FEET에서 발 전환을 실행하기 직전까지 |
| 2 | 근접 정렬: DOWN/GOAL 준비, 공·골대는 즉시 실제 2번 발 전환과 미세 정렬, 허들은 기존 21/10 접근 후 발 전환까지. PICK/접촉/SHOOT 직전에 정지 |
| 3 | 실행: 필요한 카메라·미세보행 자세를 준비하고 실행 조건의 10/7 확인 후 PICK/접촉/SHOOT. 공·골대는 후진과 라인 복귀까지 |

허들 실행은 미세 접촉 보행 및 16번 넘기, 이후 FORWARD 카메라와 9번 보행 자세로 정리한다.
공·골대 실행은 각각 후진 3회/2회 및 홀수 오른쪽·짝수 왼쪽 라인 회전, 최대 10회,
유효 라인 정지 확인, 9번 실제 DONE까지다. 실패 HOLD에서는 RESET한다.
3구간은 큰 보행/미세 정렬을 대신 하지 않는다. 물체가 실행 영역 밖이면 HOLD하며
화면에 정렬 필요를 표시한다. 2구간을 사용하거나 물체 위치를 준비한 뒤 새 PREVIEW한다.

START는 별도로 호출한다. 전용 Stage 0~3의 시작 자세(WALK_MODE 또는
DEFAULT_POSE_MODE)와 카메라 준비는 실제 명령으로 자동 실행한다. 모터와 카메라
DONE을 처리한 후 새로운 관측을 진행하며, PICK/SHOOT/HURDLE 등 실제 미션
동작은 기존 PREVIEW/COMMIT 승인이 있어야 실행한다.
같은 노드에서 1→2→3을 선택하면 이미 완료된 카메라·발 자세를 유지하고,
필요한 전환만 수행한다. 2구간을 미세 자세에서 다시 시작하면 9번으로 보행 자세를
준비한다. 0단계/공·허들 1단계는 FORWARD 카메라로 시작한다. 골대 1단계는
FORWARD나 GOAL 카메라의 settled 상태에서 START할 수 있고, GOAL 카메라를
자동 준비한 다음 근거리 접근 진입 시점에 종료한다. 시작 시 보행 자세가 필요하다.
새 노드에서 실제 발 자세를 다시 확인하려면 먼저 RESET을 완료한다.

## 명령 예시

아래 명령은 각각 사용자 PC 터미널에서 실행한다. 명령 예시는 작업 중 실행하지 않았다.

```bash
source /opt/ros/humble/setup.bash
source /home/noh/my_cv/install/setup.bash
export ROS_DOMAIN_ID=22
ros2 run vision ball_tuning_node --ros-args -p test_stage:=2
```

다른 터미널에서:

```bash
ros2 service call /ball_tuning_node/start std_srvs/srv/Trigger '{}'
ros2 service call /ball_tuning_node/preview std_srvs/srv/Trigger '{}'
ros2 service call /ball_tuning_node/commit std_srvs/srv/Trigger '{}'
```

구간 완료로 WAIT_START가 되면:

```bash
ros2 param set /ball_tuning_node test_stage 3
ros2 service call /ball_tuning_node/start std_srvs/srv/Trigger '{}'
```

허들/골대도 실행 파일과 서비스/파라미터 경로의 ball_tuning_node를 각각
hurdle_tuning_node/goal_tuning_node로 바꾼다. 전용 서비스는 private 이름을 사용한다.
perception_tuning_node의 기존 서비스 이름은 유지한다.

시험 회차 test_ball_mission_index/test_goal_mission_index는 WAIT_START에서 변경한다.
전체/실행 구간이 끝나면 해당 회차의 다음 값을 제안한다. 접근/정렬 구간 완료는
회차를 증가시키지 않는다. 골대 시험의 공 보유 상태는 시험 시나리오이며 실제 공은 준비한다.

## 복구·깊이·피드백

공·허들·골대의 물체 재발견은 정지 관측마다 10회 중 7회다. 최소 1초 및 10프레임을
확보하고 판정하며 기본 최대 5회다. PREVIEW 대기로 같은 복구 이력을 초기화하지 않는다.
실패한 관측 구간 다음에는 새 증거를 모으고, 시도 횟수는 유지한다.
GOAL의 완전히 빈 첫 관측은 위 LINE 재탐색 경로를 사용하며, 재탐색 한도를
소진해도 관측에 실패하면 FAILED에서 정지한다.

백보드 중심 깊이는 중심 3×3, 좌우 깊이는 박스 안쪽의 폭 5px × 전체 높이 영역의
유효 깊이 중앙값이다. backboard_depth_strip_width_px는 폭이며 기본값은 공통 YAML이다.
대표 열 좌표도 같은 영역을 사용한다. 중심·좌우 관측 영역을 얇은 테두리로 그린다.
rqt에서 폭을 바꾸면 이전 깊이를 재사용하지 않고 새 PREVIEW가 필요하다.
중심 거리가 근접 기준 안이면 좌우 깊이/pose 부재 때문에 큰 접근 보행을 계속하지 않는다.
유효 깊이 범위(backboard_min/max_depth_m), 림 반지름, 투척 yaw 한계도 rqt에서 변경 가능하다.
유효 깊이 범위나 depth sampling 폭을 바꾸면 이전 PREVIEW는 폐기하고 새 관측을 받는다.
GOAL 1단계는 camera_goal_start_z_m을 사용하지 않고, GOAL 카메라에서
fine_adjust_start_z_m 근접 조건이 성립하여 GOAL_CLOSE_WALK에 진입하면 종료한다.

실제 피드백은 20ms 제어 타이머에서 코어 코디네이터에 반영한다. 이미지가 없어도
정상 DONE 수신과 실행기 대기를 로그로 구분하며, 이미 발행한 미수락 ID만 재전송한다.
지각 FSM은 받은 이벤트를 순서대로 새 영상에 적용한다. 새로운 판단/보행에는 관측을
확보한다. RESET은 실제 완료된 이벤트의 관측 대기를 폐기할 수 있지만, 진행 중인
몸/카메라 동작을 완료로 꾸미지 않는다.

post_motion_observation_sec는 0을 거절한다. 공은 DOWN에서 발견하면 즉시 FINE으로 전환하며
DOWN 일반 보행 전환선은 없다. 화면에는 pickup 사각형을 표시한다.
IMU 좌표 기준 통일과 AUTO의 골대 단독 시작 방식 변경은 이번 범위에서 보류했다.

소스 변경 적용에는 코어 빌드/설치와 vision 재빌드가 필요하다. GitHub·젯슨 반영은 별도다.
발 전환 중 집게손 유지 처리는 친구 10.13.2 실행기를 전제로 한다.

## 2026-10-14 공통 접근 및 RESET 규칙

- BALL: 현재 PC의 원거리/중거리 중앙은 3걸음(22), 근거리 중앙은 2걸음(21) 후 DOWN이다. 근거리 양쪽 및 FORWARD 유실 복구는 WALK 제자리 회전 좌 31/우 28을 사용한다. 구역 경계·회복 횟수는 운영 YAML을 따른다.
- GOAL: 근거리 판단 직후 FINE 발 전환을 요청하고 실제 DONE 뒤 미세 정렬한다. 이전 COARSE 일반 보행 단계와 거리 오차 전환 파라미터는 제거했다. 낙하점·허리 회전·투척 허용값은 기존 운영 YAML을 사용한다.
- HURDLE: FORWARD에서 LINE O/H score의 좌중우와 허들 중심의 세로 위치를 결합한다. DOWN 진입 후는 기존 알고리즘 그대로 유지한다.
- RESET: 기본 흐름과 전용 Stage 시험은 기존 카메라 상태가 FORWARD로 보고되어도 새로운 FORWARD 명령을 전송한다. 일반 GOAL 단독 실행에서 `goal_trial_start_goal=true`이면 GOAL 명령을 전송한다. WALK_MODE와 카메라 명령의 실제 DONE 이후 WAIT_START로 진입한다.
- 백보드 디버그는 좌/중/우 샘플영역의 유효 깊이 픽셀 수, 비율, 중앙값을 표시하지만 새 유효성 거부 임계값을 추가하지 않는다.

## 튜닝 후 검사

코어의 `tests/README.md`와 실행기의 `tests/README.md`에 명령을 정리했다.
회귀 테스트는 전용 시나리오를 사용하고 운영 YAML·실제 JSON 검사는 별도로 실행한다.
운영 YAML/JSON을 바꾼 경우 소스와 설치본 비교를 통해 파일 반영 여부도 확인한다.

## GOAL 경계 보정 및 공통 디버그 화면

- YAML `goal.edge_margin_px`(기본 50px), ROS/rqt `goal_edge_margin_px`로 좌우 화면
  가장자리 보정 범위를 조절한다. 실전 노드는 WAIT_START/PAUSED이고 몸·카메라가
  유휴일 때 변경할 수 있다. 튜닝 노드는 기존 WAIT_PREVIEW/WAIT_COMMIT에서 바꾼다.
- 경계 보정은 기존 좌 31번/우 28번 명령을 같은 방향으로 연속 2회 실행한다.
  각 회전의 실제 DONE을 따로 받고, 첫 DONE 직후 관측 없이 두 번째를 발행한다.
  실전에서는 두 번째 DONE 후 공통 관측 한 번을 마치고 재판단한다. 튜닝은
  단일 COMMIT으로 두 회전을 실행한 뒤 WAIT_PREVIEW로 돌아가며 다음 PREVIEW에서
  공통 1초 관측한다. 중간에 재검출되어도 승인한 두 번째 회전을 생략하지 않는다.
  유실 복구(31/28, FINE 7/8)는 기존 횟수별 관측을 유지한다.
- 복구 횟수 등 공통 ROS 시작 파라미터는 각 미션 구성 대입을 마친 뒤 적용해
  덮어쓰기를 방지한다. 실전·튜닝 노드가 같은 읽기 함수를 사용한다.
- 두 노드는 `mission_debug_view.hpp`의 같은 영상·상태 패널 렌더러를 사용한다.
  YOLO/선택 박스, 판단 중심, 기준선, 픽업 영역, Depth, CONTACT, 상태·피드백·IMU·성능
  표시를 공통으로 그린다. 대기·일시정지에서도 같은 렌더러를 사용한다. 튜닝의
  PREVIEW/COMMIT 결과는 기존대로 보존하며 라이브 검출로 교체하지 않는다.
- 백보드 Depth와 단독 시험 설정은 공통 `mission` YAML에서 각각 `goal`/`ball`로
  이동했다. ROS/rqt의 기존 이름은 유지한다. 코어 설치와 두 노드·실행기 재빌드가
  필요하며 운영 YAML의 기존 수치와 모션 JSON은 이번 작업에서 바꾸지 않았다.

## 일반 GOAL 노드의 시작 카메라 선택

- ROS/rqt: `goal_trial_start_goal`, YAML: `vision_algorithm.goal.trial_start_goal`.
  기본값은 false다. 예전 `goal_trial_start_forward` 이름은 교체한다.
- false: 공을 가진 상태로 LINE에서 시작한다. 23번 DONE/보행 3회 DONE의
  GOAL 진입 조건과 FORWARD RESET을 유지한다.
- true: START 직후 새 GOAL 카메라 명령을 보내고, 카메라 완료 후 공통
  `transition_settle_sec` 및 1초/10·7 관측을 거쳐 접근한다. RESET도 GOAL로 복귀한다.
  START 전의 영상이나 FORWARD에서의 백보드 검출은 시작 조건이 아니다.
- 실전·일반 튜닝 노드에서 GOAL 모드의 WAIT_START일 때 변경하고 다음 START에
  적용한다. 전용 `goal_tuning_node`의 Stage 0~3 시작 구간은 `test_stage`가 결정한다.

```bash
ros2 run vision line_perception_node --ros-args \
  -p algorithm_mode:=goal \
  -p goal_trial_start_goal:=true \
  -p show_debug_view:=true
```

## 2026-10-16: 격자 유실·탐색·bbox·LINE 방어

BALL 5는 접근 DONE 뒤 관측 실패 시 DOWN, 4/6은 좌31/우28 1회 DONE 후 FORWARD
정지 복구다. HURDLE 7/9는 회전 뒤 관측 실패 시 추가 전진 없이 DOWN이다.

DOWN 탐색만 `ball_down_search_min_hits`, `hurdle_down_search_min_hits`로 분리했다.
운영값은 5 (최근 stable_window=10), FINE/복구 stable_min_hits 및 공통 10/7은 유지한다.
LINE `line_p2p_in_place_turn_score` 운영값은 0.5다. 초과 시 좌31/우28 1회 후 새 관측,
기존 line_p2p.recovery_max_turns의 실제 DONE 한도에 닿아도 점수가 크면 FINAL HOLD다.

객체별 `ball|hurdle|goal|backboard_min_box_width` / `_height` (double, 기본 2.0px)를
추가했다. 위 새 파라미터 모두 실전·튜닝 노드에서 시작값과 변경값이 core로 반영된다.
실전은 WAIT_START/PAUSED + 몸/카메라 DONE, 튜닝은 idle WAIT_PREVIEW/WAIT_COMMIT에서
변경한다. bbox 필터 변경은 frozen preview/이전 관측을 무효화하되 실제 DONE을 보존한다.
공통 상태 패널에도 새 탐색/점수/bbox 기준이 표시된다. 빌드·실기 검증은 별도다.


### 추가 정리: 탐색 횟수 통일·LINE 복구·라인 bbox

공과 허들은 같은 DownSearchWalk를 사용하며 각각 down_search_max_steps=10회,
10번 모션의 실제 DONE 기준으로 종료한다. 공의 down_search_timeout_sec는 제거했다.
ROS/rqt: ball_down_search_max_steps / hurdle_down_search_max_steps (positive integer).
마지막 걸음에서 검출 조건을 채우면 실패보다 접근 복귀가 우선한다.
LINE 큰 점수 회전은 SCORE RECOVERY 상태로 표시하며 실제 DONE 카운터와
line_p2p.recovery_max_turns(기본 5)를 표시한다. 5회 뒤에도 다음 회전이 필요하면 HOLD다.
object_targets의 공통 min_box_width/height 및 object_min_box_width/height ROS
파라미터는 제거했다. 라인은 기존 line_detection.min_box_width/height=1.0px를 유지하며,
line_min_box_width / line_min_box_height (double)로 다른 객체와 같이 rqt 반영한다.
