# PC 통합 미션 튜닝 — 2026-10-13

현재 PC의 forward_walk는 친구 2026_motion의 10.13.2/forward_walk 원본이다.
12번의 기존 이름과 번호는 유지하며 최신 JSON 매핑을 따른다.
이 수정은 PC의 my_cv/src/vision 및 ~/vision_core에서 개발한다.
GitHub·젯슨·로컬 jandi_workspace는 이 작업에서 갱신하지 않는다.

## 파라미터와 관측

알고리즘 기본값은 ~/vision_core/config/vision_algorithm.yaml 한곳에서 관리한다.
노드의 기본 실행 설정은 ~/vision_core/install/share/shared_vision_core/config/vision_algorithm.yaml이다.
YAML만 변경할 때는 이 설치본으로 복사하고 노드를 재시작한다.
소스·헤더 변경은 코어 설치 및 vision 재빌드가 필요하다.

화면 좌표 기준은 입력 이미지 픽셀이다. 초기값은 640×480 기준이다.
해상도를 바꾸면 기준을 다시 맞춘다. 골대의 실제 거리(m)와 LINE O/H 점수는
기존 단위를 유지한다. 공·허들 위치는 최근 관측 창에서 후반 가중치 1→3을 사용한다.
미검출도 창의 한 칸이지만 좌표 0으로 평균하지 않는다.

공·허들 진입은 원본 중심 v≤470px인 관측이 최근 10회 중 7회일 때다.
공/허들 upper_acquire_v_px는 같은 방향의 상한 기준이다.
보행 구간은 판단용 후반 가중 v로 선택한다.

## 접근 구간과 화면

공 FORWARD: 좌우 허용 폭과 v 경계로 3×3을 만든다.

| 구간 | 동작 |
|---|---|
| 1·4 | 24: 좌회전 전진 |
| 3·6 | 25: 우회전 전진 |
| 2 | 23: 직진 4걸음 |
| 5 | 22: 직진 3걸음 |
| 7·8·9 | 21: 직진 2걸음 완료 후 DOWN |

아래 구간에 대해 2걸음→DOWN을 잠근다. 모션 중 검출이 사라져도 실제 DONE 뒤
DOWN으로 간다. 카메라가 내려가기 전 추가 접근 보행은 발행하지 않는다.
공 접근의 세로 경계 2개와 가로 경계 2개를 두께 1px로 표시한다.
객체 접근의 이동 중 평균 대상은 `vision_algorithm.mission.object_ready_mean_actions`
목록 하나로 관리한다. 기본 `[22, 23]`에서는 직진 3·4걸음만
40%~READY 이동 중 평균을 사용하고, 회전전진 24/25는 READY가 와도
DONE 이후 1초 정지 관측을 사용한다. 나중에 JSON 모션을 검증한 뒤
`[22, 23, 24, 25]`로 바꾸면 AUTO·튜닝 모두 동일하게 적용된다.
`[]`로 설정하면 객체 READY 평균을 모두 끈다. LINE 11/12/23 READY
예약과 forward_walk의 READY 전송 목록은 이 설정과 독립적이다.
지원하지 않는 번호·중복 번호는 YAML 로드 시 오류로 거부한다.
목록은 rqt 동적 파라미터로 노출하지 않으며 YAML 변경 후 노드를 재시작한다.

허들 FORWARD는 v로 4/3/2걸음만 사용한다. 좌우 정렬은 추후 LINE 기반으로 구현한다.
4걸음 뒤 READY 평균 또는 (증거 부족 시) DONE 후 정지 관측에서
유효 검출을 못 얻으면 복구로 간다.
3걸음 뒤에는 같은 관측 기준으로 판단하며, 재관측 실패 시 DOWN으로 간다.
2걸음 구간은 실제 DONE 직후 검출 여부와 관계없이 DOWN으로 간다.
단, AUTO에서 3/4걸음 접근의 유효한 READY 관측이 확보되면 정지 후 새
1초 관측 대신 40%~READY 평균으로 다음 구간을 결정한다. READY 관측이
부족하거나 유실되면 기존 DONE 후 관측을 유지한다.
가로 경계선은 2개다.

골대 FORWARD/GOAL 접근은 target_u_px와 approach_u_deadband_px로 방향을 정한다.
중심선과 좌우 경계 3개를 표시한다. 거리로 직진 23/22를 고르는 기존 접근을 유지한다.

공·허들 DOWN 초기 검색은 10번 모션을 반복한다. 각 down_search_max_steps(기본 10회)의
실제 DONE 횟수로 제한하며 시간 제한은 없다. 최근 최대 10프레임에서
down_search_min_hits(기본 5회) 검출과 현재 프레임 검출로 탐색 종료를 결정한다.
발견 조건을 채워도 현재 수락된 걸음의 실제 DONE까지 기다린다.
이후 큰 보행으로 거리만 조정한다:

- v < down_one_step_v_px: 21번, 2걸음.
- down_one_step_v_px ≤ v < fine_mode_v_px: 10번, 1걸음.
- v ≥ fine_mode_v_px: 2번 자세 전환 → 새 관측 → 미세보행.

공이 줍기 영역의 v 구간까지 도달한 경우에도 더 전진하지 않고 미세보행으로 전환한다.
전환선은 fine_mode_v_px와 pickup_v1_px 중 먼저 도달하는 실제 기준을 표시한다.
공 DOWN 화면은 줍기 사각형 하나와 가로선 2개다. 허들 DOWN도 가로선 2개만 표시하며
접촉 기준은 글자 패널과 contact_v_px에서 확인한다.
미세보행 진입 후 공은 3/4/5/6, 허들은 3번만 사용한다. 10/21로 자동 복귀하지 않는다.

| rqt 파라미터 | 기본값(px) | 의미 |
|---|---:|---|
| ball_upper_acquire_v_px | 470 | 공 진입 상한 |
| hurdle_upper_acquire_v_px | 470 | 허들 진입 상한 |
| ball_far_u_des_px | 320 | 공 접근 좌우 중심 |
| ball_approach_u_deadband_px | 16 | 공 좌우 허용 반폭 |
| ball_approach_short_v_px | 288 | 중앙 4→3걸음 경계 |
| ball_approach_two_step_v_px | 360 | 공 아래 행 2걸음→DOWN 경계 |
| ball_down_one_step_v_px | 216 | DOWN 2→1걸음 경계 |
| ball_fine_mode_v_px | 278.4 | 미세보행 진입선 |
| ball_pickup_u1_px / ball_pickup_u2_px | 281.6 / 358.4 | 줍기 영역 왼쪽 / 오른쪽 |
| ball_pickup_v1_px / ball_pickup_v2_px | 307.2 / 364.8 | 줍기 영역 위쪽 / 아래쪽 |
| hurdle_approach_short_v_px | 312 | 허들 4→3걸음 경계 |
| hurdle_approach_two_step_v_px | 360 | 허들 3→2걸음 경계 |
| hurdle_down_one_step_v_px | 288 | DOWN 2→1걸음 경계 |
| hurdle_fine_mode_v_px | 350.4 | 미세보행 진입선 |
| hurdle_contact_v_px | 408 | 미세 접촉 반복 진입 |
| goal_target_u_px / goal_approach_u_deadband_px | 320 / 16 | 골대 접근 중심 / 허용 반폭 |

기존 ball_tilt_down_v_px/hurdle_tilt_trigger_v_px는 approach_two_step_v_px로,
hurdle_acquire_min_v_px는 hurdle_upper_acquire_v_px로 대체한다.
공·허들의 fine_one_step_v_error_px/fine_two_step_v_error_px는 두 절대 v 경계로 대체한다.
hurdle_fine_target_v_px 대신 접촉 목표는 hurdle_contact_v_px를 사용한다.

## 발 자세와 미션

일반 보행 자세와 미세보행 자세를 shared core에서 관리한다.
2번 DEFAULT_POSE_MODE가 미세보행 전환, 9번 WALK_MODE가 일반 보행 전환이다.
전환은 실제 ACK/DONE으로 완료하고 READY는 완료로 취급하지 않는다.
전환 DONE은 후진·접촉·라인 회전 카운터에 전달하지 않는다.
미션 완료는 보행 자세 복귀 DONE 뒤 확정하며, 골대 단독 시험 완료도 동일하다.
일반 LINE 실패 복구의 7/8 회전에도 필요한 발 전환을 적용한다.

BALL: 접근 → 2걸음/DOWN → 검색 → 21/10 거리 조정 → 2 전환 → 미세 정렬
→ 14 PICK(모션 내부 재그립) → 잔존 공 관측 → 성공이면 1 기본 자세
→ 4 후진 3회 → FORWARD → 같은 방향 회전·라인 확인 → 9 → LINE.
PICK 실패 시 다시 미세 정렬하며 최대 10회다. 성공 판정은 기존 잔존 공 10/7 기준이다.

GOAL: 공 보유/라인 복귀 뒤 LINE 실행시간 5초 → 수락된 보행 DONE → GOAL 시야
→ 접근 → 림 거리 오차로 21/10 조정 → 2 → 3/4/5/6 미세 정렬 → 17 슛
→ 4 후진 2회 → FORWARD → 회전·라인 확인 → 9 → LINE.
골대 단독 전체(0) 시험은 FORWARD 접근부터 시작한다. 골대 접근(1) 시험은
GOAL 카메라를 먼저 준비하고 GOAL_CLOSE_WALK 진입 직전에 종료한다.
GOAL로 올렸는데 아직 골대를 못 본
상태에서는 의도대로 정지한다. 팔 전환을 포함한 실제 LINE 보행 시간만 5초에 포함한다.

HURDLE: 접근 → DOWN → 검색 → 21/10 거리 조정 → 2 → 3 미세 접근
→ contact_v_px 도달 → 3 접촉 반복(hurdle_contact_steps 기본 3) → 16 넘기
→ FORWARD → 9 → LINE 재관측. 20번 박치기는 발행하지 않는다.

정면 접근 및 초기 DOWN 검색 이후 공·허들·골대에서 놓친 물체는 정지 재관측 최대 5회다.
각 관측은 1초 이상이며 10프레임 증거를 확보한 뒤 10/7로 판정한다. 연속 3회 재발견 기준은 사용하지 않는다. 미세보행 진입 여부를
유지하여 재발견 후 큰 보행과 미세보행을 섞지 않는다.

## 라인 복귀와 실패 표시

공·골대의 회전은 mission_recovery_max_turns 기본 10회, 요청 각도 15도다.
각 미션 회차가 홀수면 오른쪽, 짝수면 왼쪽이다. 일반 LINE 실패 회전의 5회 설정은 별도다.
마지막 회전에도 정지 확인 기회를 준다. 1초가 지나도 10프레임이 없으면 관측을 계속한다.
최근 10회 중 유효 guide 7회와 현재 guide 유효를 확인해야 복귀한다.
확인 실패는 사용한 회전 횟수를 유지하고, 10회 소진 후 실패하면 HOLD/RESET이다.
UI 상단에 HOLD/RESET 이유를 표시하고 PREVIEW로 실패 상태를 해제하지 않는다.

LINE 기준은 410px, O/H gain은 각각 0.3이다. 청록색 기준선과 초록색 offset 판단선을 유지한다.
실시간 RGB·YOLO 박스와 판단 좌표는 유지한다. WAIT_COMMIT에서는 판단 결과만 유지한다.

## PREVIEW/COMMIT과 RESET

start → preview → commit 서비스를 사용한다. 일반 접근/작은 보행/자세 전환은 각각
한 COMMIT이다. 반복 검색·후진·라인 회전·허들 접촉은 COMMIT 한 번으로 해당 단계만 승인한다.
다음 단계로 넘어가면 승인 해제, 수락된 동작은 실제 DONE까지 완료한다.
전환 후 AUTO는 새 정지 관측, 수동 튜닝은 새 PREVIEW로 위치를 확인한다.

RESET은 실제 동작이 완료되고 feedback을 접수한 상태에서 보행 자세9와 카메라 FORWARD를 복귀시킨다.
이미 완료된 이벤트의 영상 적용 대기는 RESET으로 폐기할 수 있다.
실제 동작 DONE과 카메라 settled를 모두 확인한 뒤 WAIT_START가 된다.
영상이 없어도 제어 타이머에서 일반 실행 및 RESET feedback을 코디네이터에 반영한다.
새 지각 판단은 실제 영상 관측을 기다린다. 이미 발행한 미수락 ID의 재전송만 타이머에서 수행한다. 명령 ID는 보존한다.
RESET/START는 새 미션 시험이며 START 이전 실제 발 자세를 임의로 WALK로 간주하지 않는다.
일반 rqt 값은 idle WAIT_PREVIEW/WAIT_COMMIT에서, 시험 회차는 WAIT_START에서 변경한다.

미션별 독립 노드와 test_stage 0/1/2/3 사용법은 [MISSION_STAGE_TESTING.md](MISSION_STAGE_TESTING.md)를 따른다.

골대 단독 전체 시험:

```bash
ros2 run vision goal_tuning_node
ros2 service call /goal_tuning_node/start std_srvs/srv/Trigger '{}'
# UI가 WAIT_PREVIEW이면 preview, WAIT_COMMIT이면 commit을 하나씩 호출한다.
ros2 service call /goal_tuning_node/preview std_srvs/srv/Trigger '{}'
ros2 service call /goal_tuning_node/commit std_srvs/srv/Trigger '{}'
```

## GOAL 카메라 접근 및 실시간 조정 (2026-10-13)

GOAL 단독 시험의 test_stage=1은 GOAL 카메라가 이미 settled여도 START할 수 있다.
GOAL 시야에서 백보드 깊이와 가로 위치를 사용해 접근하며,
`fine_adjust_start_z_m` 조건으로 GOAL_CLOSE_WALK에 진입할 때 종료한다.
test_stage=0은 이전과 같이 FORWARD 접근부터 시작한다.

기존 rqt 튜닝 항목을 유지하며 `goal_hoop_radius_m`,
`goal_shoot_yaw_limit_deg`, `backboard_min_depth_m`,
`backboard_max_depth_m`을 추가로 실시간 적용한다.
백보드 유효 깊이 범위 또는 depth strip 폭을 바꾸면 새로운 PREVIEW를 요청한다.
튜닝은 기존 WAIT_PREVIEW/WAIT_COMMIT 게이트에서만 가능하며, YAML 자동 저장은 하지 않는다.

## 2026-10-14 투척·스테이지·Reset 동작

- GOAL 좌표 튜닝: 카메라 렌즈 기준 기본 낙하점은 X=-0.08m, Z=0.48m이다. 새로운 파라미터는 `goal_throw_landing_x_m`, `goal_throw_landing_z_m`이며 2차원 림/예상 낙하점 오차와 허리 회전 한도로 SHOOT을 판단한다.
- 공의 FORWARD 3×3: 중앙 먼/중간/가까운 영역은 각각 직진 4/3/2걸음이며, 가까운 좌우 영역은 짧은 보행 자세 회전 후 재관측한다.
- 허들은 FORWARD 카메라에서 LINE O/H 값으로 가로 3구역을 정하고 허들 중심 높이로 세로 3구역을 정한다. DOWN 이후는 유지한다.
- 전용 스테이지 START 시 초기 발 자세(WALK/FINE)와 카메라를 자동 준비하며, PICK/SHOOT/HURDLE 및 접근 모션은 계속 COMMIT을 요구한다.
- RESET은 저장된 카메라 상태가 FORWARD이더라도 새 FORWARD 명령 ID와 WALK_MODE 명령을 발행하고 양쪽 완료를 기다린다.
- 백보드 RGB-D 디버그에는 좌/중/우 영역 유효 깊이 픽셀 수, 비율, 중앙값을 표시한다. 기존 깊이 유효성 판정은 변경하지 않는다.
