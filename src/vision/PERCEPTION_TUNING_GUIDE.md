# Perception 통합 수동 튜닝 (시험용)

> 코드 상태: 소스 수정 및 정적 검토 대상. ROS 2/CUDA/TensorRT 빌드와 실기 검증은 **별도** 필요.
> 기존 `line_p2p_tuning_node`, `forward_walk`, 친구 `2026_motion`은 수정하지 않는다.

## 실행 파일

- `line_perception_node`: 기존 AUTO, READY one-slot queue 유지.
- `line_p2p_tuning_node`: 기존 LINE 전용 수동 시험 유지.
- `perception_tuning_node`: 신규. 동일 YOLO TensorRT, RGB-D, MissionController를 사용하지만 **몸체 Action은 NEXT 승인 후 하나씩** 실행. 카메라 DOWN/FORWARD/GOAL은 자동 실행.

반드시 **한 번에 하나의 Vision 노드만** `/jandi_vision/action_cmd`에 발행해야 한다. 튜닝 중 실전 노드를 동시에 실행하지 말 것.

**중요 — 실행 파일을 전환할 때:** `forward_walk`는 최근 완료 Action/Camera ID를 기억한다. 실전 노드와 튜닝 노드의 초기 ID가 겹치지 않도록, 다른 Vision 노드로 전환하는 시험에서는 `forward_walk`도 안전하게 정지·재시작하고 로봇 동작이 끝났음을 확인할 것. 단순히 두 Vision 노드를 동시에 실행하지 않는 것으로는 ID 중복이 예방되지 않는다.

## 동작

1. 실행 직후에는 **WAIT_START**로 대기하며 관측·Action Preview를 시작하지 않는다. 먼저 `ros2 service call /perception_tuning_node/start std_srvs/srv/Trigger "{}"`를 호출한다. **START 이후 1초간 관측**하고, 카메라 변경이 필요하면 기존 CameraCommand를 자동으로 발행한다.
2. 화면에 다음 몸체 Action이 표시되면 `WAIT_NEXT`이다. 이 시점에만 RQT로 18개 튜닝 파라미터를 변경할 수 있다. **저장된 동일 관측에 새 파라미터**를 적용해 Action 및 디버그 수치를 재계산한다. 디버그 RGB 역시 Preview 당시 저장한 프레임을 표시하며 LINE 기준 중심은 실선으로 표시한다. FSM/Action ID/모터는 변하지 않는다.
3. `ros2 service call /perception_tuning_node/next std_srvs/srv/Trigger "{}"`로 현재 표시된 Action을 확정·전송한다. ACK/DONE까지 NEXT는 거부한다.
4. DONE 후 일반 물체 접근·미세 이동은 1초간 유효 관측을 평균해 다음 행동을 판단한다. 최초 객체 검출 진입 조건인 **최근 10프레임 중 7프레임**은 계속 유지한다. DONE 후 별도의 10/7 게이트를 반복하지 않으며, 관측 실패 시 기존 객체 복구/탐색 FSM으로 보낸다.
5. LINE 11·12·13 보행은 40%-to-READY 보행 관측을 내부에 보관하지만, **다음 Action을 forward_walk에 큐잉하지 않는다**.
6. 진행 중인 몸체 Action·카메라 명령·피드백이 없을 때 `ros2 service call /perception_tuning_node/reset std_srvs/srv/Trigger "{}"`를 호출하면 기존 CameraCommand(FORWARD)를 자동 송신한다. 카메라 **DONE 이후 WAIT_START**로 돌아가며, **RESET 자체로는 1초 관측·Preview·몸체 Action을 시작하지 않는다. 반드시 `/perception_tuning_node/start`를 다시 호출해야** 새로운 1초 관측을 시작한다. 카메라 시뮬레이션 모드에서도 RESET 후 WAIT_START이다. Action ID는 보존한다. 카메라 전송을 끈 경우(카메라 시뮬레이션 제외) RESET은 거부한다.

**WAIT_NEXT에서만** 18개 튜닝 파라미터 변경을 허용한다. WAIT_START, 초기 관측, DONE 후 1초 관측, 몸체 Action/카메라 실행 및 RESET 카메라 복귀 중에는 거부한다. 현재 튜닝 값은 YAML 파일에 자동 저장되지 않는다. LINE READY 구간에서 이미 평균한 guide는 과거 기준 중심으로 계산된 값이므로, 기준 중심 보정은 가능한 한 초기 정지 관측에서 완료한다.

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

LINE 기준 `line_reference_u_px`는 영상상의 원하는 라인 위치이고 **RGB-D 내부 파라미터 cx가 아니다**. BALL 픽업 허용 사각형은 (fine target u/v ± fine deadband u/v)로 정의하며, 영상 안의 판단용 중심점(1초 평균)을 표시한다. 두 축 모두 허용 영역 안이면 PICK_BALL; 그 밖에서는 각 Deadband로 정규화한 **사각형 초과량이 큰 방향**부터 이동한다.

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

**안전:** NEXT는 테스트용 단계 진행이며 긴급 정지 기능이 아니다. `reset`은 몸체 자세·토크를 바꾸지 않지만 **카메라를 FORWARD 자세로 움직인다**. RESET 카메라 DONE을 확인한 뒤 START를 눌러야 관측이 시작된다. 첫 로봇 시험에는 기체를 지지하고 비상 전원 차단을 준비한다.
