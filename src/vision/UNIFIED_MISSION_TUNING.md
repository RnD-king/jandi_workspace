# 통합 미션 튜닝 — 2026-10-12

`forward_walk` 원본은 `RnD-king/2026_motion`, 커밋
`f477db208c4877f36a4497583f20d976f55fa2d6`의 `10.13.1/forward_walk`이다.
이 패키지는 수정하지 않는다. 메시지는 친구의 ActionCommand 정의와 일치한다.
`vision` 및 독립 CMake 라이브러리 `vision_core`의 원본은 이 jandi_workspace
체크아웃이다. PC는 이 소스를 my_cv와 ~/vision_core에 동기화하고,
Jetson은 같은 브랜치/커밋의 소스를 빌드한다.

## 제어와 COMMIT

`start → preview → commit` 서비스를 사용한다. 일반 접근/미세조정은 COMMIT당
한 동작을 실행한다. 다음 단계는 COMMIT 한 번으로 해당 단계만 승인한다.

| 승인 단계 | 실행 | 종료 |
|---|---|---|
| BALL/HURDLE DOWN 검색 | action 10 연속 | 최근 10 관측 중 7 감지 또는 10초 |
| BALL 후진 | action 4 세 번 | 세 번째 DONE |
| GOAL 후진 | action 4 두 번 | 두 번째 DONE |
| BALL/GOAL 라인 탐색 | action 7/8 반복 | 유효 라인 7-hit 또는 최대 5회 완료 |
| HURDLE 접촉 | action 3 반복 | hurdle_contact_steps 완료 |

모션마다 고유 action_id와 실제 ACK/DONE을 처리한다. 재전송에는 같은 ID를
사용한다. 반복 중 DONE은 감지 이력을 초기화하지 않으며, 별도의 정지 관측이나
COMMIT을 기다리지 않는다. 조건을 충족해도 실행 중인 모션은 끝까지 완료한다.
그 뒤 추가 명령을 중단한다. 10초 이후에는 새 걸음을 발행하지 않으므로,
실제 종료 시점에는 마지막 걸음의 남은 시간이 포함될 수 있다.

단계 승인은 다음 단계로 넘어갈 때 해제된다. PICK/RECATCH/자세 복귀,
HUDDLE/SHOOT 및 새로운 반복 단계는 별도 COMMIT이 필요하다.
카메라 전환은 기존처럼 body DONE 이후 자동으로 진행한다.
`stop_repeat`는 반복 승인을 취소한다. 현재 수락된 모션의 DONE은 기다린다.
이는 실행 중인 모션을 중간 취소하는 서비스가 아니다.

```bash
ros2 service call /perception_tuning_node/start std_srvs/srv/Trigger '{}'
ros2 service call /perception_tuning_node/preview std_srvs/srv/Trigger '{}'
ros2 service call /perception_tuning_node/commit std_srvs/srv/Trigger '{}'
ros2 service call /perception_tuning_node/stop_repeat std_srvs/srv/Trigger '{}'
ros2 service call /perception_tuning_node/reset std_srvs/srv/Trigger '{}'
```

RESET은 idle 상태에서 시험을 초기화하고 카메라를 FORWARD로 복귀시킨다.
동작이나 미처리 feedback이 남으면 거부한다. 기존 Action/Camera ID는 보존한다.

## 보행과 관측

LINE은 11/12/13을 유지한다. 기본 기준 u=420px, O/H gain은 각각 0.3이다.
공 접근은 v에 따라 직진 23→22로 줄이고, 좌우 오차가 크면 24/25를
사용한다. GOAL 접근은 중심 depth로 23/22를 고른다. 가까운 미세조정에서는
기존 미세 보행을 유지하고, 전진 오차가 큰 경우에는 21/10을 사용한다.

BALL/HURDLE 위치는 최근 카메라 관측 창에서 뒤쪽에 1→3 가중치를 준다.
미검출 프레임도 창의 한 칸을 차지하지만 좌표 0으로 평균하지 않는다.
10/7 감지 증거와 위치 평균은 구분한다. 반복 중에는 창을 유지하고,
카메라 전환이나 새로운 정지 위치 관측에서는 창을 정리한다.
GOAL의 RGB-D pose 및 raw fine 슛 기하 계산은 유지한다.

READY는 11/12/13 및 21~26에서 관측 종료 이정표다. 완료 신호가 아니다.
기존 40%-to-READY 및 최소 표본 조건을 유지한다. 표본이 부족하면 DONE 뒤
새 PREVIEW를 사용한다. 실행기 한 칸 예약은 LINE→LINE에서만 허용되며,
수동 튜닝은 COMMIT 전 body 선예약을 하지 않는다.

## 미션 시퀀스

BALL: FORWARD 접근 → v 조건으로 DOWN → 검색 → 새 위치 관측/미세조정
→ PICK → 잔존 공 확인 → 성공이면 RECATCH → 기본 자세 → 후진 3회
→ FORWARD → 라인 탐색. 실패하면 재정렬 후 PICK을 재시도하며 최대 10회다.
성공 판정은 기존처럼 10 관측 중 7회 미만 감지이다. 손 안의 공을 직접 검증하는
센서는 아니다. DOWN 검색 시간 소진과 라인 탐색 횟수 소진은 RESET 전 HOLD다.

HURDLE: FORWARD 접근 → v 조건으로 DOWN → action 10 검색 → 새 위치
관측/미세조정 → contact_v 조건 → action 3 접촉 반복 → 별도 COMMIT으로
HUDDLE → FORWARD/기존 라인 재관측. action 20은 실행기 미구현이므로 발행하지 않는다.

GOAL 통합 모드: 공 보유/라인 복귀 → LINE 보행 실행시간 누적 5초 →
새 LINE 예약 차단 → 이미 수락된 보행 DONE → GOAL 카메라 → 접근/미세조정
→ SHOOT → 후진 2회 → FORWARD → 라인 탐색. 팔 전환은 보행 실행시간에
포함하고 사람의 COMMIT 대기, 정지 관측, 후진, 회전은 제외한다. 예약 ACK는
저장 완료일 뿐이므로 현재 보행 DONE 뒤 실제 실행부터 누적한다.
고개를 올린 뒤 골대가 안 보이면 HOLD한다. 단독 시험은 FORWARD 접근부터
시작하고 중심 depth 조건으로 고개를 올리는 기존 시험 시퀀스를 유지한다.

공·허들의 초기 DOWN 검색은 최대 10초 전진이다. 일단 발견하여 미세조정에
들어간 뒤 객체를 놓치면 초기 검색으로 돌아가지 않고 별도 정지 재관측을
실행한다. 매회 새로운 1초 관측/10프레임 중 7회 검출을 사용하며 총 5회까지만
시도한다. 재발견하면 미세조정으로 돌아가고 5회 실패하면 HOLD/RESET이다.
몸 동작을 발행하거나 DONE을 모사하지 않으며, 사람의 새 PREVIEW 없이 관측을
계속한다. 재발견 이후의 몸 동작에는 다시 COMMIT이 필요하다.

허들은 현재 v 기준 앞뒤 접근만 사용한다. 허들 bbox 중심을 이용한 회전 전진,
옆걸음, 놓친 객체의 방향 회전은 비활성화했다. 좌우/방향 정렬은 나중에 라인을
기준으로 구현한다. hurdle_target_u_norm/hurdle_u_deadband는 현재 사용하지 않는다.

공·골대 라인 복귀는 회전 도중 10/7 발견 후 현재 회전을 완료하고, 별도 1초
정지 관측/10프레임 중 7회로 재확인한다. 확인에 실패하면 사용한 회전 횟수를
유지하고 남은 회전만 시도한다. 다섯 번째 회전 후 재확인 실패는 HOLD다.

## 골대 전체 시퀀스 단독 시험

`goal_tuning_node`는 perception_tuning_node와 같은 구현 및 공통 코어를 사용한다.
다른 미션은 비활성화하고 FORWARD 접근부터 시작한다. 시험 완료 후 WAIT_START로
돌아가며, 성공적으로 완료한 골대 시험은 다음 회차를 rqt에 설정한다.
공을 다시 준비한 뒤 START로 반복하거나, WAIT_START에서 회차를 다시 지정한다. 골대 mode는 goal로 고정한다.
perception_tuning_node의 `algorithm_mode:=goal`도 같은 단독 시퀀스를 제공한다.

```bash
ros2 run vision goal_tuning_node
ros2 service call /goal_tuning_node/start std_srvs/srv/Trigger '{}'
ros2 service call /goal_tuning_node/preview std_srvs/srv/Trigger '{}'
ros2 service call /goal_tuning_node/commit std_srvs/srv/Trigger '{}'
```

실행 알고리즘 노드는 perception_tuning_node/goal_tuning_node 중 하나를 사용한다.
TensorRT engine 경로는 yolo26_runtime.yaml의 PC/Jetson 설정 또는 engine_path로 선택한다.

## rqt와 화면

`test_ball_mission_index`, `test_goal_mission_index`는 WAIT_START에서 바꾼다.
각 미션의 회차는 독립적이며 홀수는 오른쪽, 짝수는 왼쪽으로 라인을 탐색한다.
실제 미션 진입에서만 회차가 진행되며 PREVIEW와 재전송은 증가시키지 않는다.

일반 튜닝 값은 WAIT_PREVIEW/WAIT_COMMIT의 idle 상태에서 변경한다.
변경 시 후보를 다시 계산한다. 반복 실행 중에는 변경하지 않는다.

| 용도 | 추가 rqt 파라미터 |
|---|---|
| BALL 접근 | ball_approach_short_v_norm |
| BALL 큰 미세 전진 | ball_fine_one_step_v_error_norm, ball_fine_two_step_v_error_norm |
| HURDLE 접근 | hurdle_approach_short_v_norm (u 정렬 파라미터는 현재 비활성) |
| HURDLE DOWN 접근 | hurdle_fine_target_v_norm, hurdle_fine_one_step_v_error_norm, hurdle_fine_two_step_v_error_norm |
| HURDLE 접촉 | hurdle_contact_v_norm, hurdle_contact_steps |
| GOAL 접근 | goal_approach_short_z_m, goal_camera_goal_start_z_m |
| GOAL 큰 미세 전진 | goal_fine_one_step_range_error_m, goal_fine_two_step_range_error_m |

기존 LINE, BALL pickup u/v target/deadband, 카메라 DOWN v, GOAL 슛 파라미터도 유지한다.
접근 보행 전환 v는 DOWN v 이하, one-step 오차는 two-step 오차 이하로 설정한다.
v는 [0,1] 값이며 패널에는 현재 이미지 높이에 곱한 픽셀 기준도 표시한다.

영상 배경과 raw YOLO bbox/labels는 LIVE다. 최근 판단 중심·박스는 retained
preview에 따라 표시한다. BALL/HURDLE의 보행 전환선과 카메라 전환선은 1px이다.
픽업 허용 영역은 기존 노란 사각형이다. 상태 패널에 승인 범위, 회차와 회전 방향,
회전 완료 횟수, 전환선 픽셀값을 표시한다. 골대 전환까지의 누적 보행시간과
잔여시간, 초기 DOWN 검색 잔여시간, 정지 복구 관측 회차와 라인 정지 재확인
상태도 표시한다. 허들 마커/패널은 보행 길이와 앞뒤 미세조정에 사용하는 후반
가중 v를 표시한다. LINE 정지/READY 집계는 모든 실행 노드에서 공통 최소 유효
표본 수(line_p2p.failure_min_valid_samples, 기본 5)를 요구한다.

## 검증 범위

코어 테스트는 미검출 검색, 10초, 단계별 승인 경계, 중복 DONE, 후진 3/2회,
교대 회전, 5회 HOLD, 다섯 번째 회전에서 발견, 슛 후 단독 시험 종료 및
허들 접촉/넘기 분리를 확인한다. 실행기 테스트는 in-memory Dynamixel을 사용한다.
PC 빌드/소프트웨어 테스트와 실제 로봇 검증은 구분한다.
영상 독립 watchdog, DONE 대기 제한과 공 보유에 따른 팔 정책 변경은 이번 범위에서 제외했다.
