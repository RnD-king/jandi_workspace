# Perception 통합 수동 튜닝 (시험용)

> 코드 상태: 소스 수정 및 정적 검토 대상. ROS 2/CUDA/TensorRT 빌드와 실기 검증은 **별도** 필요.
> 기존 `line_p2p_tuning_node`, `forward_walk`, 친구 `2026_motion`은 수정하지 않는다.

## 실행 파일

- `line_perception_node`: 기존 AUTO, READY one-slot queue 유지.
- `line_p2p_tuning_node`: 기존 LINE 전용 수동 시험 유지.
- `perception_tuning_node`: 신규. 동일 YOLO TensorRT, RGB-D, MissionController를 사용하지만 **몸체 Action은 COMMIT 승인 후 하나씩** 실행. 카메라 DOWN/FORWARD/GOAL은 자동 실행.

반드시 **한 번에 하나의 Vision 노드만** `/jandi_vision/action_cmd`에 발행해야 한다. 튜닝 중 실전 노드를 동시에 실행하지 말 것.

**중요 — 실행 파일을 전환할 때:** `forward_walk`는 최근 완료 Action/Camera ID를 기억한다. 실전 노드와 튜닝 노드의 초기 ID가 겹치지 않도록, 다른 Vision 노드로 전환하는 시험에서는 `forward_walk`도 안전하게 정지·재시작하고 로봇 동작이 끝났음을 확인할 것. 단순히 두 Vision 노드를 동시에 실행하지 않는 것으로는 ID 중복이 예방되지 않는다.

## 동작 — 명시적 PREVIEW / COMMIT

- **최초 실행:** WAIT_START. `ros2 service call /perception_tuning_node/start std_srvs/srv/Trigger "{}"`를 누르면 처음 1초 관측 후 WAIT_COMMIT으로 진입한다.
- **COMMIT:** `ros2 service call /perception_tuning_node/commit std_srvs/srv/Trigger "{}"`. 현재 저장·확인한 **몸체 Action 단 하나**를 실제로 전송한다. Action ACK/READY/DONE을 처리하는 동안에는 COMMIT과 PREVIEW 모두 거부한다.
- **일반 BALL/GOAL 접근·미세조정 및 HURDLE 판단:** 모션 DONE 후 WAIT_PREVIEW로 대기한다. `ros2 service call /perception_tuning_node/preview std_srvs/srv/Trigger "{}"`를 **누른 시점 이후** 새 영상으로 1초 관측·10/7 유효성 검사·평균·Action 계산을 한다. 유효한 Action이 나오면 WAIT_COMMIT. 관측 부족 시 HOLD/복구 상태로 남고 재PREVIEW할 수 있다.
- **모든 미션의 Action 11·12·13:** 미션이 LINE/BALL/GOAL/HURDLE 무엇이든 보행 중 40%-to-READY 구간의 관측을 집계한다. READY에서 유효한 목표 평균(객체는 최근 10프레임 7회 이상, 최근 0.3초 검출)을 저장하면, DONE 후 **PREVIEW 버튼 없이** 다음 Action을 자동 계산해 WAIT_COMMIT에 표시한다. READY가 없거나 대상 데이터가 부족하거나 카메라 각도가 바뀌어 통일된 평균을 만들 수 없으면 WAIT_PREVIEW로 대기하며 임의로 움직이지 않는다.
- **LINE 복구:** 유실 후 2초, 회전 복구 후 1초라는 기존 관측 시간을 PREVIEW에서 사용한다. READY 보행 실패 후에도 복구 FSM을 통해서만 다음 Action을 선택한다.
- **고정 몸체 시퀀스:** PICK_BALL→RECATCH, DEFAULT_POSITION→STEP_BACK, HURDLE 접촉보행→허들넘기 등은 새로운 대상 평균이 필요 없으면 자동으로 WAIT_COMMIT에 표시한다. 모든 몸체 모션은 여전히 COMMIT으로만 실행한다. RECATCH 후 픽업 검증은 PREVIEW로 새로운 관측을 한다.
- **카메라:** DOWN/GOAL/FORWARD는 기존 CameraCommand로 자동 이동한다. 카메라 DONE 이후 새 객체 관측이 필요하면 WAIT_PREVIEW로 대기한다. HURDLE DOWN 이후 고정 접촉보행은 바로 WAIT_COMMIT에 표시할 수 있다.
- **WAIT_COMMIT에서도 PREVIEW 재요청 가능:** 해당 Action을 버리고 새로운 1초 관측을 수행한다. WAIT_COMMIT에서 RQT 18개 파라미터를 조정하면 저장된 동일 관측으로 Action만 다시 계산한다.
- **RESET:** 진행 중 몸체 Action/카메라 명령/피드백이 없을 때 `ros2 service call /perception_tuning_node/reset std_srvs/srv/Trigger "{}"`. 카메라 FORWARD로 자동 복귀한 뒤 **WAIT_START**. RESET 자체는 다음 관측/Action을 시작하지 않는다. 반드시 START를 다시 누른다. Action ID는 보존한다.

RQT 18개 튜닝 파라미터는 **WAIT_PREVIEW / WAIT_COMMIT**에서만 변경할 수 있다. OBSERVING, 몸체 실행, 카메라 이동, RESET 중에는 거부한다. 변경값은 YAML에 자동 저장되지 않는다. READY 평균은 단일 카메라 시점의 데이터만 사용하며, 정상적인 보행 중 영상 흔들림을 수치적으로 제거한다는 뜻은 아니다.

**forward_walk 호환성:** 기존 실행기는 LINE(미션 1)의 11·12·13에서만 READY를 발행했다. 이 브랜치에서는 **모든 미션의 Action 11·12·13에서 READY를 한 번 발행**하도록 `motion_executor_state.hpp`의 READY 조건을 변경했다. **one-slot queue 입장은 기존 LINE→LINE만** 허용하며, 수동 튜닝 노드는 READY가 와도 다음 Action을 선큐잉하지 않는다. ACK/DONE, Action ID, 토픽, JSON 맵핑은 변경하지 않았다.

## RQT 튜닝 파라미터

| Mission | 이름 |
|---|---|
| LINE | `line_reference_u_px`, `line_p2p_offset_gain`, `line_p2p_heading_gain`, `line_p2p_steering_deadband` |
| BALL 접근 | `ball_far_u_des_norm`, `ball_approach_u_deadband`, `ball_tilt_down_v_norm` |
| BALL 픽업 | `ball_fine_target_u_norm`, `ball_fine_target_v_norm`, `ball_fine_u_deadband`, `ball_fine_v_deadband` |
| GOAL 접근 | `goal_target_u_norm`, `goal_approach_u_deadband`, `goal_fine_adjust_start_z_m` |
| GOAL 슛 | `goal_throwing_range_m`, `goal_position_tolerance_m` |
| HURDLE | `hurdle_acquire_min_v_norm`, `hurdle_tilt_trigger_v_norm` |

총 18개. 객체 인식 안정화 10/7, EMA alpha, 슛 림 기하학 상수는 이번 튜닝 대상에서 제외한다.

LINE 기준 `line_reference_u_px`는 영상상의 원하는 라인 위치이고 **RGB-D 내부 파라미터 cx가 아니다**. WAIT_COMMIT에서 기준 중심을 바꾸면 저장된 1초/READY 평균 Offset을 이미지 반폭으로 정규화하여 새 기준 중심으로 환산하고 Preview를 재계산한다. Heading은 변하지 않는다. BALL 픽업 허용 사각형은 (fine target u/v ± fine deadband u/v)로 정의하며, 영상 안의 판단용 중심점(1초 평균)을 표시한다. 두 축 모두 허용 영역 안이면 PICK_BALL; 그 밖에서는 각 Deadband로 정규화한 **사각형 초과량이 큰 방향**부터 이동한다.

GOAL 디버그의 `GOAL FINE INPUT`은 `FineAction`에 실제 전달한 좌표·거리·Yaw를 표시하며, 평활화된 추적기 위치는 별도로 `GOAL TRACKED POSE`라 명시한다. WAIT_COMMIT에서 파라미터 변경으로 HOLD/NONE이 나오더라도 추가 변경은 계속 가능하지만 COMMIT은 실행하지 않는다.

## Depth

Depth 토픽은 지속 구독한다. YOLO 검출 결과에서 BACKBOARD가 존재할 때에만 대응하는 aligned Depth 프레임을 변환해서 3×3 중앙/양쪽 샘플을 추출한다. 실제 Depth 스트림 생산·정렬 비용은 별개이므로 이것만으로 Depth 센서 전체 연산이 사라진다고 주장하면 안 된다. 3×3 확대는 별도 실측 후 결정한다.

## 빌드와 시험

`src/vision_core`는 ROS와 독립된 별도 shared library다. 새로운 API가 추가되어 **이 코어를 먼저 빌드하고 그 설치 디렉터리로 vision을 빌드**해야 한다. 이전 코어 라이브러리와 섞지 말 것.

```bash
cd /path/to/jandi_workspace
source /opt/ros/humble/setup.bash
cmake -S src/vision_core -B build/manual_shared_core \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$PWD/build/manual_shared_core/install"
cmake --build build/manual_shared_core -j2
ctest --test-dir build/manual_shared_core --output-on-failure
cmake --install build/manual_shared_core
export VISION_CORE_ROOT="$PWD/build/manual_shared_core/install"
colcon build --packages-select vision --symlink-install --cmake-clean-cache \
  --cmake-args -DSHARED_VISION_CORE_ROOT="$VISION_CORE_ROOT"
source install/setup.bash
```

모터 없는 테스트 시 `simulate_decision:=true`, `simulate_camera_feedback:=true`를 명시적으로 설정할 것. 실기 테스트 전에 CAMERA와 ACTION 채널, READY/DONE, 수정한 FSM, Depth 동기화, 재시작 직후 ID 재사용 방지 등을 각각 검증한다.

**안전:** COMMIT은 테스트용 단계 진행이며 긴급 정지 기능이 아니다. `reset`은 몸체 자세·토크를 바꾸지 않지만 **카메라를 FORWARD 자세로 움직인다**. RESET 카메라 DONE을 확인한 뒤 START를 눌러야 관측이 시작된다. 첫 로봇 시험에는 기체를 지지하고 비상 전원 차단을 준비한다.
