# LINE에서 객체까지 연속 접근

운영 YAML `config/vision_algorithm.yaml`의 `vision_algorithm.common.mission`에서
`object_approach_continous`를 선택한다. 철자는 최초 합의한 `continous`를 사용한다.
기본 `false`는 기존 후보 발견 → LINE 배출 → 정지 관측/확정 → 미션 접근이다.
`true`는 실전 AUTO의 LINE 및 원거리 객체 접근을 같은 보행 계획기로 연결한다.
설정은 노드 시작 시 읽으며 실행 중 변경하지 않는다. 기존 MANUAL PREVIEW/COMMIT과
전용 Stage 0~3 튜닝은 단발 실행 계약을 유지한다.

## 원거리 경로와 보행

- 실제 LINE은 기존 점선의 가까운 직선 fit에서 O/H를 구한다.
- BALL은 bbox 아래 중앙의 지면 접점을 카메라 장착 위치/각도로 몸통 기준 지면에
  투영하고, 몸통에서 목표까지의 직선을 카메라 영상에 다시 투영한다.
- GOAL은 백보드 중앙의 optical Z와 카메라 내/외부 파라미터로 몸통 기준 위치를
  구해 그 지면 투영 위치까지의 직선을 사용한다. GOAL 뷰에서도 바닥 LINE은 필요 없다.
- HURDLE의 방향은 기존과 같이 실제 LINE을 사용한다. bbox의 횡 위치로 조향하지 않는다.

실제/가상 직선 모두 `ImageLineGuide`의 동일한 영상 반폭 정규화와 `-atan(du/dv)`를
거쳐 기존 `LineP2pController`의 offset/heading gain 및 deadband를 사용한다.
실제 LINE의 가까운 fit 기준 높이가 있으면 공유하고, 없으면 `reference_v_ratio`를 쓴다.
IMU 보정 사용 시 가상 경로에도 기존 픽셀 rectifier를 적용한다. 가상 선분은 기하 계산용이며
추가 검출이나 관측 성공 프레임으로 세지 않는다. LINE→객체 최초 전환에는 `blend_frames`를 쓴다.

32/33/34번과 `line.walk`의 좌/직진/우 반복 횟수, 보행 중 관측 창 및 READY 선예약을
공유한다. `object_ready_mean_actions`에 번호를 추가할 필요는 없다. 실제 실행기 큐는
기존대로 현재 1개 + 선예약 1개이며, 미션 FSM이나 큐를 별도로 복제하지 않는다.
원거리 동안 `active_mission`은 LINE이고 별도의 `approach_mission`이 접근 후보를 표시한다.

## 후보, 3회 락, 유실

기존 confidence/크기/depth/동일 ID 진입 증거를 사용한다. BALL의 진입 hit 수,
HURDLE/GOAL의 안정 hit 수를 더 높이지 않는다. 원거리 진입 전에 정지 10/7 확인은 하지 않는다.

보행 묶음을 시작할 때 후보 세대를 기록하고, 그 묶음의 모든 단위가 **실제 DONE**된 경우에만
해당 후보의 완료 횟수를 1 올린다. 기존 LINE 묶음, ACK/READY, 같은 DONE 재전송,
중단된 묶음, 다른 객체 ID의 묶음은 포함하지 않는다. 같은 후보의 `lock_blocks`(기본 3)
완료 전에는 임시 추적이고, 3회 완료 후 해당 객체 추적 ID를 고정한다.

`missing_frames`는 **연속 유실 영상 프레임 수**이며 기본 3이다. 보행 3회와는 다른 수다.
**원거리** 확정 전 유실이면 후보/횟수를 해제하고 유효한 실제 LINE으로 돌아간다. LINE도 유효하지 않으면
기존 LINE 관측/복구 정책을 사용한다. GOAL 뷰에서 LINE 복귀할 때는 접수된 몸 동작을 모두
끝낸 뒤 FORWARD를 요청하고, 실제 카메라 settled 뒤 LINE을 사용한다.
GOAL을 다시 관측하는 시도는 추가 LINE 묶음 완료 뒤에만 허용한다.
최초 GOAL 화면에 백보드가 계속 없으면 `goal.lost_frames` 이후 FORWARD→실제 LINE
한 묶음 완료→GOAL 확인을 재시도한다. `object_approach.goal_empty_max_retries`는
최초 확인 뒤의 **추가 재시도** 한도이며 기본 3이다(최초 1회 + 추가 3회).
이를 소진하면 `GOAL_SEARCH_FAILED` / FINAL HOLD를 유지하며 Reset 전에는 재개하지 않는다.
실제로 백보드 후보를 획득하면 이 최초 탐색 재시도 횟수를 지운다.

확정 후 유실이면 아직 접수된 동작을 끝낸 뒤 기존 객체 미션의 유실 복구로 넘긴다.
현재의 방향 기억, 복구 횟수/거리 한도, 실패 처리는 해당 컨트롤러가 소유한다.
정지 관측으로 재발견이 확정되면, 아직 원거리일 때 기존 격자 접근을 발행하지 않고
공통 가상 라인 보행으로 복귀한다. 기존 완료 묶음 수/락/객체 ID와 미션 순번을 보존한다.
복구 중 근거리 경계를 확인했다면 그 경계를 유지하고 근거리로 넘긴다.
실제 LINE의 마지막 조향 점수는 시간 만료 없이 다음 실제 LINE 판단까지 유지하지만,
LINE 재탐색 시작/Reset뿐 아니라 **근거리 접근 시작**, **미션 완료 후 정상 LINE 재시작**에 지운다.
몸 방향과 카메라 자세가 달라진 뒤 이전 LINE 방향으로 탐색하는 것을 막기 위한 것이다.
확정된 BALL/HURDLE은 `entry_recovery_max_steps`나 탐색 소진 후 LINE 복귀를 쓰지 않는다.
BALL은 일반 FORWARD 복구 한도(기본 회전→전진→관측 3회), HURDLE은 일반 전진 복구
한도(기본 3걸음)를 쓰고, 이후 제한된 정지 관측까지 실패하면 실패 상태를 유지한다.
근거리 인계 이후에는 락 횟수에 관계없이 진입 직후 FORWARD 복구/LINE 복귀를 쓰지 않는다.
재발견이 확정되면 해당 복구 횟수와 한도 소진 표시를 초기화한다. 일반 GOAL RecoverySearch의
정지 재발견에도 동일하게 적용하며, 한두 프레임 일시 검출로는 초기화하지 않는다.
RESET은 후보/완료 횟수/락을 지운다. PAUSE/RESUME의 관측 초기화는 완료 횟수를 늘리지 않는다.

## 근거리 경계

| 객체 | 경계 | 이후 |
| --- | --- | --- |
| BALL | `approach_two_step_v_px` 아래쪽, 즉 7/8/9 | 접수된 보행 DONE 후 DOWN·근거리 탐색/미세 접근 |
| GOAL | depth/유효 pose의 Z가 `fine_adjust_start_z_m` 이하 | 기존 근거리 자세·정렬·슛 |
| HURDLE | `approach_two_step_v_px` 아래쪽 | 접수된 보행 DONE 후 DOWN·근거리 탐색/정렬 |

경계를 확인하면 새 보행 예약을 막고 아직 보내지 않은 반복을 버린다. 이미 접수된 단위는
취소하거나 완료로 꾸미지 않고 모두 실제 DONE을 기다린다. **근거리 경계는 한 번 진입하면
해제하지 않는다.** 잠시 영상/깊이가 경계 밖으로 돌아오거나 3회 락 전에 유실돼도 LINE으로
돌아가지 않는다. 실제 최근 관측이 안정적이거나 유실 기준을 충족하면 근거리 처리로 넘기며,
공/허들은 DOWN, 골대는 GOAL의 FINE 복구만 사용한다. 3회를 채우려고 추가로 걷지 않는다.
READY 직후 경계를 지나면 이미 예약한 단위 한 개가 남을 수 있으므로 실제 보행 거리와 함께
경계값을 조절해야 한다. 카메라 전환에는 기존 DONE/settled 계약을 그대로 사용한다.

## 공 DOWN 및 골대 근거리의 즉시 FINE 전환

이 규칙은 `object_approach_continous`의 T/F 모두에 적용한다.
공은 DOWN 최초 탐색의 검출 조건을 만족하면 즉시 FINE 자세 변경을 요청하고, 실제 DONE 뒤
새 관측으로 pickup 사각형 기준 상하좌우 미세 정렬을 한다. 발견 이후 일반 DOWN 보행이나
최초 WALK 탐색으로 돌아가지 않는다. 공을 찾기 전의 최초 탐색 한도/검출 조건은 유지한다.
공의 `down_one_step_v_px`, `fine_mode_v_px`는 설정·코어·rqt·디버그 전환선에서 제거했다.

골대는 기존 근거리 Z 기준을 만족하면 즉시 FINE 자세 변경을 요청한다. FINE 전환을 위한
COARSE 일반 보행은 없으며, 사용하지 않는 `fine_one_step_range_error_m`,
`fine_two_step_range_error_m`도 제거했다. 실제 발 전환 DONE을 기다린 뒤 미세 정렬/복구한다.
허들의 DOWN 일반 보행 및 FINE 진입 임계값은 기존대로 사용한다.

## 공 DOWN 유실 복구

이미 DOWN에서 본 공을 잃으면 마지막 실제 DOWN 관측 위치를 pickup 사각형과 비교한다.
정상 미세 정렬과 동일한 정규화 경계 초과량으로 축 우선순위를 선택하며, 왼쪽/오른쪽은
미세 옆걸음 5/6, 위쪽/아래쪽은 미세 전진/후진 3/4다. FORWARD 기준선이나 마지막
FORWARD 위치는 쓰지 않는다. FINE 자세가 아니면 실제 자세 변경 DONE부터 기다린다.

고정한 방향으로 1회 실행→실제 DONE→새 정지 관측을 반복한다. 이동 중 검출만으로
복구를 완료하지 않으며, 정지 관측에서 안정 재발견하면 현재 위치로 FINE 정렬을 재개한다.
`ball.down_recovery_max_actions`는 기본 3이고, 마지막 관측까지 실패하면 즉시 실패한다.
마지막 관측이 pickup 사각형 내부였거나 방향 기억이 없으면 맹목적으로 픽업하지 않고
제자리 관측을 같은 한도만큼 시도한다. 최초 DOWN 미검출 탐색 10걸음과는 별도 경로다.
이 DOWN 복구 변경은 연속 접근 T/F와 관계없이 공 컨트롤러에 적용된다.

## 백보드 영상은 보이지만 Depth·좌표 계산 실패

백보드 영상이 실제로 보이는 동안 좌표를 얻지 못하는 상태가 `goal.lost_frames` 이상
지속되면 일반 WALK는 1걸음(21), FINE은 미세 전진(3)을 한 번씩 수행한다.
실제 DONE마다 새 정지 관측을 시작하고 안정된 좌표를 재획득하면 이전 접근을 재개한다.
`goal.depth_recovery_max_steps`는 기본 3이며, 세 번째 전진 후 관측까지 실패하면
실패 정지를 유지한다. 전송/ACK/READY/중복 DONE이나 이동 중 일시적 좌표는 횟수나
복구 성공으로 취급하지 않는다. 백보드 영상 자체가 사라지면 객체 유실 복구로 전환한다.

동일한 GoalController 복구 처리를 연속 모드 원거리와 기존 근거리 FSM에서 공유한다.
원거리는 이미 접수된 보행을 모두 끝낸 뒤 전진하며, 이 1걸음 탐색은 가상 라인 보행
묶음 완료/3회 락으로 세지 않는다. 좌표가 복구되면 유지한 후보로 가상 라인 접근을 재개한다.
원거리 한도 소진은 `GOAL_DEPTH_FAILED`, 기존 GOAL FSM은 `GOAL_FAILED`로 표시한다.
이는 백보드 자체를 잃어서 회전 탐색하는 경로와 별도다.

## 카메라 보정값

`object_approach.forward_camera` / `goal_camera`는 **사용자 요청에 따른 임시값**이다.
현재 right=0, forward=0, height=0.45 m이며 pitch는 FORWARD=20°, GOAL=0°, yaw=0°다.
모터 tick을 도 단위로 추측한 값이 아니다. 실측 전의 초기 설정으로만 두었으므로 교체해야 한다.

몸통 원점 기준 광학 중심의 right/height/forward는 오른쪽/위/전방이 +m,
pitch는 아래를 볼 때 +°, yaw는 오른쪽을 볼 때 +°다. Intrinsics는 실제 CameraInfo를 쓴다.
지면이 평평하다는 가정이며, 지면과 교차하지 않는 BALL ray, 유효하지 않은 깊이/투영은 HOLD다.
추론 엔진이나 YOLO 횟수를 추가하지 않는다.

## 표시와 검증

GUI 상태 패널과 상태 로그에 후보 미션/ID, 완료 묶음 수, OPEN/LOCK, 유실 프레임,
NEAR/DRAIN을 표시한다. SSH 터미널의 phase에도 `APP_BALL 2/3 OPEN`처럼 표시한다.

구현 중에는 테스트를 컴파일하고 빌드·정적 확인만 한다. 실행은 사용자가 아래 명령으로 한다.

```bash
cmake -S /home/noh/vision_core -B /home/noh/vision_core/build_shoot \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DCMAKE_INSTALL_PREFIX=/home/noh/vision_core/install
cmake --build /home/noh/vision_core/build_shoot -j4
ctest --test-dir /home/noh/vision_core/build_shoot \
  -R 'immediate_fine_entry|continuous_object_approach|goal_depth_recovery|goal_recovery_policy|ball_down_recovery|ball_forward_recovery|hurdle_recovery_steps|config_loader|runtime_config_check' --output-on-failure
ctest --test-dir /home/noh/vision_core/build_shoot --output-on-failure
cmake --install /home/noh/vision_core/build_shoot

source /opt/ros/humble/setup.bash
cd /home/noh/my_cv
CMAKE_PREFIX_PATH=/home/noh/vision_core/install${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH} \
  colcon build --packages-select vision forward_walk --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

코어 구성체에 필드를 추가했으므로 코어를 먼저 설치한 뒤 `vision`, `forward_walk`를
모두 재빌드하고 노드를 재시작한다.
