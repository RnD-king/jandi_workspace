# shoot_tuning_node

골대 앞에 놓은 로봇으로 인식 좌표, 미세 보행 판단, 슛을 단발 시험한다.
실행 시 Qt 창 하나에 기존 `line_perception_node`와 같은
`RenderMissionDebug` 영상, 버튼, 파라미터 조절 영역이 열린다.
영상 탭에는 실시간 디버그, 마지막 계산 프레임, 기존 상태 디버그가 있다.
자동 미션은 시작하지 않는다. 기본자세 버튼은 몸 복귀가 끝난 뒤 카메라 정면 복귀를 요청한다.

## 빌드

`vision_core`에도 상태를 바꾸지 않는 `GoalController::PlanFineAction` API가
추가되었으므로 코어를 먼저 설치한다. Qt5 Widgets 개발 패키지가 필요하다.

```bash
cmake -S /home/noh/vision_core -B /home/noh/vision_core/build_shoot \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/home/noh/vision_core/install
cmake --build /home/noh/vision_core/build_shoot -j4
cmake --install /home/noh/vision_core/build_shoot

source /opt/ros/humble/setup.bash
cd /home/noh/my_cv
CMAKE_PREFIX_PATH=/home/noh/vision_core/install${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH} \
  colcon build --packages-select vision forward_walk --symlink-install --cmake-clean-cache
```

## 실행

계산·좌표 표시에는 모터나 `forward_walk`가 필요하지 않다. 자동 계산과 픽셀
클릭에는 카메라의 RGB, aligned depth, color CameraInfo를 발행한다. 직접 입력한
탑뷰 좌표를 저장하고 다음 동작을 표시하는 기능은 카메라 프레임 없이도 사용한다.
실제 동작을 시험할 때만 `forward_walk`를 별도 터미널에서 실행한다.
같은 명령 토픽을 사용하는 다른 vision 제어 노드는
종료한 뒤 사용한다. 카메라는 백보드를 볼 수 있는 자세로 준비한다.
`forward_walk`의 시작 자세 도달 및 명령 구독 준비가 끝난 다음 버튼을 사용한다.

```bash
source /opt/ros/humble/setup.bash
source /home/noh/my_cv/install/setup.bash
ros2 run vision shoot_tuning_node
```

ROS 실행 설정 파일을 사용할 수도 있다.

```bash
ros2 run vision shoot_tuning_node --ros-args \
  --params-file /home/noh/my_cv/src/vision/config/vision_params.yaml
```

모델 경로는 기존 `yolo26_runtime.yaml`의 PC/Jetson 설정을 사용한다.
`YOLO26_ENGINE_PATH`, `YOLO26_RUNTIME_CONFIG`, `-p engine_path:=...`로 바꿀 수 있다.
이 노드는 화면이 있는 데스크톱 세션에서 실행해야 한다.

## 버튼

| 버튼 | 동작 |
| --- | --- |
| 계산하기 | 누른 순간 화면에 표시 중인 최신 처리 프레임의 골대 중심을 계산 결과로 저장한다. 명령은 전송하지 않는다. |
| 슛 | 인식 결과와 무관하게 `SHOOT(17)`, 허리 각도 `0°`를 전송한다. |
| 기본자세 | 전용 요청으로 `DEFAULT_POSE_MODE(2)`를 지정 시간에 실행하고, 실제 몸 DONE 뒤 카메라 `FORWARD`를 요청한다. |
| 계산된 값에서 슛 | 저장된 골대 중심 좌표와 현재 슛 파라미터로 다음 한 동작을 판단하여 전송한다. 새 영상을 계산하지 않는다. |
| 클릭 결과를 계산 결과로 사용 | 마지막으로 클릭해서 표시한 골대 중심을 계산 결과로 저장한다. 영상이 바뀌어도 클릭한 프레임의 좌표를 사용한다. |
| 입력 좌표를 계산 결과로 사용 | 직접 입력한 골대 중심의 탑뷰 `(X,Y)`를 계산 결과로 저장한다. 카메라 인식 결과는 필요하지 않다. |

일반적인 순서는 **계산하기 → 계산된 값에서 슛**이다.
수동 지정은 **영상 클릭 → 클릭 결과를 계산 결과로 사용 → 계산된 값에서 슛**,
또는 **탑뷰 좌표 입력 → 입력 좌표를 계산 결과로 사용 → 계산된 값에서 슛**이다.
클릭하거나 숫자를 편집하는 것만으로는 저장값을 바꾸지 않으며 적용 버튼을 눌러야 한다.
결과 영역에는 현재 저장값의 출처(자동 인식 / 픽셀 클릭 / 직접 입력)를 함께 표시한다.
직접 입력은 백보드 좌표가 아닌 **골대 림 중심** 좌표이며, 단위는 m,
X는 유한한 수, Y는 양수여야 한다. 림 반지름을 중복해서 빼지 않는다.

처음에는 **기본자세**로 미세보행 준비 자세를 잡는다. 슛 뒤 다시 준비할 때도
이 버튼을 사용한다. 임의 관절 위치를 저장했다 복원하는 기능은 아니다.

기본자세의 **이동 구간 합계**는 이 노드의 `shoot_prepare_duration_sec`로 조절하며
기본값은 2초, 범위는 0.2~60초이다. 오른쪽 UI와 rqt에서 수정할 수 있다.
기존 미세보행 초기자세 JSON의 중간 자세와 구간별 시간 비율을 유지하면서
이 요청에만 시간을 적용한다. 공용 JSON과 다른 노드의 일반 기본자세 명령은 바꾸지 않는다.
관절 속도 제한, 기존 팔 전환, 홀드 및 이후 카메라 이동 시간은 추가될 수 있다.
카메라는 기존 `camera_forward_yaw_tick`/`camera_forward_pitch_tick` 목표를 사용하고,
정면 복귀 실제 DONE을 받을 때까지 추가 동작을 막는다. 계산용 영상도 카메라 DONE
이후 촬영된 새 프레임을 사용한다.

기본자세 전용 `/jandi_vision/shoot_tuning_pose` 토픽은 `ShootTuningPose`에
action ID와 시간을 함께 담는다. `forward_walk`도 함께 재빌드해야 하며, 이전
실행기가 이 토픽을 구독하지 않으면 알림만 표시하고 기본자세 요청을 건너뛴다.
기존 `ActionCommand`/`CameraCommand`/`CommandStatus` 정의와 동작 번호는 유지한다.

`계산된 값에서 슛`은 현재 생산 코드의 `GoalController::FineAction`을 재사용한다.
가능한 경우에는 허리 각도를 포함한 `SHOOT` 한 명령을 전송하고, 그렇지 않으면
미세 전진/후진 또는 좌/우 이동 한 명령을 전송한다. 허리 회전은 executor가
SHOOT JSON에 적용한다. 이동 완료 후 재계산, 연속 보행, 슛 후 후진은 하지 않는다.
현재 미세조정 판단은 별도의 몸 회전 action을 선택하지 않는다.

명령을 보낸 뒤에는 추가 동작 요청과 파라미터 편집을 잠그고 실제 executor의 같은 ID
`ACK`/`DONE`을 기다린다. 계산하기, 픽셀 클릭, 클릭 결과 적용, 직접 좌표 입력·적용은
대기 중에도 계속 사용할 수 있다. `ACK` 전 재전송은 같은 ID를 사용하며, `READY`를 받아도
추가 명령을 예약하지 않는다. `DONE`을 받으면 정지 상태로 다음 버튼을 기다린다.
완료 뒤 계산에는 DONE 이후 촬영된 프레임만 사용한다.
저장 좌표는 새 계산 또는 수동 적용으로 교체할 때까지 유지하며, 시간 경과나
실시간 영상 갱신으로 덮어쓰지 않는다. 이동/회전 후에는 새 위치에 맞게 다시
계산하거나 수동 지정한다. 자동 계산이 실패하면 기존 계산 결과를 해제한다.
모션 수신 노드가 없으면 동작 요청을 건너뛰고 알림만 표시한다. 요청 뒤 연결이
끊기거나 ACK 제한 시간이 지나도록 응답이 없으면 로컬 요청 대기를 해제하고
알림을 표시한다. 재시작 없이 다음 버튼을 사용할 수 있고 저장 좌표도 유지한다.
ACK 제한 시간은 공통 알고리즘 설정의 `command.action_ack_timeout_sec`를 사용한다
(현재 10초). 모터 전원을 직접 확인하는 기능은 아니며 수신 노드 연결과 ACK로 판단한다.
이 처리는 STOP 전송이나 가짜 DONE 처리가 아니다. 완료했다고 표시하거나
자동으로 다음 동작을 보내지 않는다. 실제 ACK를 받고 연결이 유지된 모션은
기존대로 실제 DONE을 기다린다. 창을 닫아도 이미 접수된 모션은 executor에서 계속 실행된다.

## 좌표와 픽셀 클릭

표시하는 탑뷰 좌표 `(X,Y)`는 기존 코어의 카메라 기준 `(x,z)`이다.
단위는 m, `+X`는 오른쪽, `+Y`는 전방이다. 카메라와 로봇 사이의 외부 변환을
추가하는 기능은 포함하지 않는다.

자동 계산은 필터를 통과한 백보드의 중앙·좌·우 깊이로 백보드 자세를 구하고,
백보드 중심에서 림 반지름만큼 로봇 쪽으로 옮긴 골대 중심을 표시한다.
영상·깊이의 시간 차이/크기, CameraInfo 크기, 프레임 신선도, 백보드 및 깊이의
유효성을 확인한다. 인식/깊이가 유효하지 않으면 이유를 표시하고 동작하지 않는다.
마지막 성공한 계산 영상과 좌표는 새 실시간 프레임과 별도로 남는다.

실시간 영상 또는 계산 프레임에서 픽셀을 누르면 **해당 표시 프레임**의
`5×5` 영역을 사용한다. 테두리에서는 영상 안으로 영역을 자른다.
깊이 0/음수/NaN/Inf를 제외하고 중앙값을 계산하며, 유효 개수가 짝수이면
가운데 두 값의 평균을 사용한다. 유효 픽셀 수와 프레임 시각도 표시한다.

클릭한 점을 백보드 중앙으로 가정한 골대 좌표는
`click_backboard_yaw_deg`에 지정한 백보드 yaw를 사용한다. 기본 `0°`는 정면
백보드 가정이다. 한 픽셀만으로 백보드 방향을 알아내지는 않는다.
클릭 자체는 참고값을 표시하며, **클릭 결과를 계산 결과로 사용**을 누르면
그때 표시된 골대 중심을 슛 판단 입력으로 선택한다. 클릭 당시의 깊이, 프레임,
yaw 가정으로 구한 좌표를 보관하므로 적용 버튼을 누르는 사이 영상이나 파라미터가
바뀌어도 다른 위치로 바꾸지 않는다. 유효한 CameraInfo/깊이가 없거나 환산한
골대 중심 Y가 양수가 아니면 적용 버튼을 활성화하지 않는다.
직접 입력 결과에는 대응하는 카메라 프레임이 없으므로 계산 프레임 탭을 비운다.

## 파라미터

오른쪽 조절 영역과 rqt는 동일한 ROS 파라미터를 편집한다.

```bash
ros2 run rqt_reconfigure rqt_reconfigure
```

`/shoot_tuning_node`를 선택한다. 주요 파라미터는 다음과 같다.

| 파라미터 | 용도 |
| --- | --- |
| `conf_thres` | YOLO 1차 confidence 하한. 기존 인식 노드와 같은 공통 기본값(현재 0.60)을 사용하며 UI/rqt 편집을 즉시 반영한다. |
| `shoot_prepare_duration_sec` | 이 슛 튜닝 노드의 기본자세 이동 시간 합계. 기본 2초. |
| `goal_hoop_radius_m` | 백보드에서 림 중심으로 옮길 거리 |
| `goal_throw_landing_x_m`, `goal_throw_landing_z_m` | 허리 0°에서 측정한 슛 도달점 `(오른쪽, 전방)` |
| `goal_position_tolerance_m` | 회전된 슛 도달점과 림 중심 사이 허용오차 |
| `goal_shoot_yaw_limit_deg` | 슛 판단에서 허용할 허리 회전 한계 |
| `backboard_conf_thres`, `goal_conf_thres` | 검출 confidence 하한 |
| `backboard_min_box_width`, `backboard_min_box_height` | 백보드 최소 픽셀 크기 |
| `goal_min_box_width`, `goal_min_box_height` | 골대 최소 픽셀 크기 |
| `backboard_min_depth_m`, `backboard_max_depth_m` | 백보드 중앙 깊이 허용 범위 |
| `click_backboard_yaw_deg` | 픽셀 클릭 환산에만 사용하는 방향 가정 |

슛 허리 한계는 `forward_walk`의 `waist_max_yaw_deg` 이하여야 계산 각도가
executor에서 잘리지 않는다. 현재 executor 기본값은 30°이다.
도달점 파라미터는 관측한 도달 위치를 입력하는 보정값이며 슛 JSON의
팔 동작, 속도, 발사 세기를 직접 변경하지 않는다.
`conf_thres`보다 낮은 confidence 후보는 YOLO 출력에서 먼저 제외되므로,
개별 골대/백보드 threshold를 더 낮춰 시험하려면 1차 기준도 함께 낮춘다.

기본 알고리즘 값은 `vision_core/config/vision_algorithm.yaml`에서 읽는다.
편집값은 실행 중에만 적용되며 YAML을 자동 변경하지 않는다.
동작 중 수정은 거절한다. 변경 직후 자동 계산은 새 설정으로 처리한 프레임을 기다린다.
저장된 골대 중심 자체는 파라미터 변경으로 이동시키지 않으며, 다음 동작과 허리
각도는 현재 슛 도달점·허용오차·회전 한계에 맞춰 갱신한다.
조절 항목 밖의 토픽, 엔진, 추론 주기, 깊이 strip 너비 등은 실행 시 지정하며
실행 중에는 읽기 전용이다.

## 검증 명령

구현 시에는 빌드 및 정적 확인만 수행했다. 테스트 실행과 카메라/로봇 구동은
사용자가 아래 명령으로 수행한다.

```bash
ctest --test-dir /home/noh/vision_core/build_shoot \
  -R shared_vision_core_goal_controller_test --output-on-failure
cd /home/noh/my_cv
colcon test --packages-select vision --ctest-args -R vision_shoot_tuning_test --output-on-failure
colcon test --packages-select forward_walk --ctest-args -R 'executor_(logic|integration)_test' --output-on-failure
colcon test-result --verbose
```

추가한 테스트는 클릭 ROI/중앙값/좌표 변환, 상태 없는 단일 프레임 판단,
부호 있는 슛 각도, 수동/클릭/자동 결과의 동일한 골대 중심 해석,
저장 좌표 유지와 파라미터 재적용, 실제 DONE을 기다리는 단발 명령 계약을 다룬다.
수신 노드 없음/ACK 무응답 시 대기 해제, 새 요청 ID 보존도 검증한다.
