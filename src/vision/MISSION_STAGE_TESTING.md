# 미션별 구간 시험

PC에서 vision을 실행하고, 실제 몸/카메라 ACK·READY·DONE은 forward_walk에서 받는다.
알고리즘은 모든 노드에서 같은 vision_core를 사용한다. 전용 노드는 다음 세 개다.

| 노드 | 미션 |
|---|---|
| ball_tuning_node | 공 |
| hurdle_tuning_node | 허들 |
| goal_tuning_node | 골대 |

`test_stage`는 WAIT_START에서 rqt로 바꾸며 정수 0..3이다.

| 값 | 시험 시작과 종료 |
|---|---|
| 0 | 전체: 큰 접근부터 미션 및 일반 보행 자세 복귀까지 |
| 1 | 큰 접근: BALL/HURDLE은 FORWARD 접근 후 DOWN 전환까지. GOAL은 GOAL 카메라부터 접근해 GOAL_CLOSE_WALK 진입 직전까지 (근거리 보행은 시작하지 않음) |
| 2 | 근접 정렬: DOWN/GOAL 준비, 21/10 거리 조정, 실제 2번 발 전환, 미세 정렬까지. PICK/접촉/SHOOT 직전에 정지 |
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
GOAL 시야에서 처음부터 백보드를 못 본 상태는 의도대로 HOLD한다.

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

post_motion_observation_sec는 0을 거절한다. 공의 미세 진입선은
min(ball_fine_mode_v_px, ball_pickup_v1_px)를 실제 로직과 동일하게 표시한다.
IMU 좌표 기준 통일과 AUTO의 골대 단독 시작 방식 변경은 이번 범위에서 보류했다.

소스 변경 적용에는 코어 빌드/설치와 vision 재빌드가 필요하다. GitHub·젯슨 반영은 별도다.
발 전환 중 집게손 유지 처리는 친구 10.13.2 실행기를 전제로 한다.

## 2026-10-14 공통 접근 및 RESET 규칙

- BALL: 현재 PC의 원거리/중거리 중앙은 3걸음(22), 근거리 중앙은 2걸음(21) 후 DOWN이다. 근거리 양쪽 및 FORWARD 유실 복구는 WALK 제자리 회전 좌 31/우 28을 사용한다. 구역 경계·회복 횟수는 운영 YAML을 따른다.
- GOAL: 일반 접근은 기존 거리/화면 구역 로직을 사용한다. GOAL 근거리(COARSE)에서는 거리 오차가 `fine_one_step_range_error_m` 이상일 때 `shoot_yaw_limit_deg` 밖이면 WALK 제자리 회전 좌 31/우 28을 우선한다. 경계 미만이면 FINE으로 전환한다. FINE의 림 위치 보정·낙하점·투척 허용값은 운영 YAML의 `hoop_radius_m`, `throw_landing_x_m`, `throw_landing_z_m`, `position_tolerance_m`, `shoot_yaw_limit_deg`를 사용한다.
- HURDLE: FORWARD에서 LINE O/H score의 좌중우와 허들 중심의 세로 위치를 결합한다. DOWN 진입 후는 기존 알고리즘 그대로 유지한다.
- RESET: 기존 카메라 상태가 FORWARD로 보고되어도 새로운 카메라 FORWARD 명령을 전송한다. WALK_MODE와 카메라 명령의 실제 DONE 이후 WAIT_START로 진입한다.
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
