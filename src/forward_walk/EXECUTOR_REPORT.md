# forward_walk executor 구현·검증 보고서

## A. 변경 파일 목록

기존 파일: `src/main.cpp`, `include/p2p_motion_player.hpp`,
`src/p2p_motion_player.cpp`, `include/dynamixel.hpp`, `src/dynamixel.cpp`,
`package.xml`, `CMakeLists.txt`.

추가 파일: `include/motion_executor_state.hpp`,
`tests/executor_logic_test.cpp`, `tests/executor_integration_test.cpp`,
`tests/fake_dynamixel.hpp`, `tests/source_contract_test.py`, 이 보고서.

`src/main (copy).cpp`, 모션/mission JSON, 기존 캡처 스크립트,
vision 및 vision_core 코드는 변경하지 않았다.

## B. 핵심 구현 요약

LINE 11/12/13의 READY와 검증된 프로그램을 저장하는 한 칸 큐를 추가했다.
Action과 Camera 각각 최근 32개 완료 ID를 보관한다.
카메라 21/22번 목표는 body 목표보다 우선하며, Camera DONE 뒤에도 유지한다.
Goal Position 전송은 MainNode 한 곳에서 23개 목표를 합쳐 수행한다.

Protocol 2.0, 4 Mbps, 23 motors, Position Mode, P850/I0/D0,
50 Hz 고정 단계 보간과 지연 시 단계 보존, startup WALK_MODE 마지막 자세
3초 이동을 유지했다. 로컬에 이미 있던 1 ms polling은 유지하며,
실제 목표값 전송 주기는 20 ms다. 모션 보간 수학식·반올림·step count는 바꾸지 않았다.

## C. Action state transition

| 현재 상태/입력 | 처리 | 결과 |
|---|---|---|
| idle + 새 유효 A | Prepare → StartPrepared → active 저장 → ACK(A) | A 실행 |
| A 실행, READY 전 + 다른 ID | 거부, ACK 없음 | A 계속 실행 |
| A 마지막 motion/frame, 잔여 시간 ≤ lead | READY(A) 한 번 | 큐 수용 가능 |
| READY 뒤 + LINE B, 큐 비어 있음 | Prepare → 프로그램/ID 저장 → ACK(B) | A 실행, B 대기 |
| A 실제 완료, B 없음 | DONE(A) → 완료 이력 → active 해제 | body idle |
| A 실제 완료, B 있음 | DONE(A) → 완료 이력 → B 승격 → StartPrepared(B) | B 자동 실행, ACK 추가 없음 |
| B 시작/최종 writer 실패 | ERROR 로그 → playback/active/queue 정리 | 실패 명령의 가짜 DONE 없음 |

## D. duplicate ID 처리

| ID 상태 | Action | Camera |
|---|---|---|
| 0 | 거부, ACK 없음 | 거부, ACK 없음 |
| active와 같은 ID | ACK 재전송, 재실행 없음 | ACK 재전송, 재실행 없음 |
| queued와 같은 ID | ACK 재전송, 재저장 없음 | 카메라 큐 없음 |
| 최근 완료 32개에 포함 | DONE 재전송, 재실행 없음 | DONE 재전송, 재실행 없음 |
| busy 중 다른 ID | READY 뒤 적격 B 한 개 외 거부 | 전환 중 거부, preemption 없음 |

이력은 두 ID 공간에 독립적이다. 메모리 내 최근 32개를 보호하며,
노드 재시작 또는 이력에서 제거된 더 오래된 ID까지 영구 보호하는 구조는 아니다.

## E. Camera state transition

| 상태 | 입력/조건 | 처리 |
|---|---|---|
| startup/첫 명령 전 | camera override 없음 | 기존 JSON의 머리 자세 유지 |
| idle | 유효 새 요청 | 마지막 전송 목표에서 전환 시작 → active 저장 → ACK |
| 이동 중 | 20 ms 전송 기회 | smoothstep의 다음 한 단계로 ID21/22 override |
| 최종 목표 전송 후 | 마지막 주기 대기 완료 | settle 시작 |
| settle 완료 | camera_settle_sec 경과 | DONE → 별도 완료 이력 → active 해제 |
| 완료 후 | body 다음 키프레임 | 최종 camera override 계속 유지 |
| body idle + 새 CameraCommand | full target cache 사용 | 카메라 실행 → settle/DONE 후 timer 정지 가능 |

Camera ID는 body action/queued action/완료 이력을 변경하지 않는다.
카메라 시작 위치는 추가 SyncRead 없이 마지막으로 전송한 목표를 사용한다.
실제 encoder 추종 오차에 대한 보정 기능을 추가한 것은 아니다.

## F. READY 조건과 exactly-once

모두 만족해야 READY를 보낸다:

- active mission이 MISSION_LINE이고 action이 11, 12 또는 13
- startup이 아니며 player가 실제 재생 중
- 전체 프로그램의 마지막 motion, 마지막 keyframe
- `RemainingNominalSec() <= ready_lead_sec` (기본 0.25초)
- 해당 active의 `ready_sent`가 false

move 중에는 `(total_steps - completed_steps) * 0.02 + hold_sec`로 계산한다.
벽시계가 늦어져도 아직 보내지 않은 단계 수가 줄지 않는다.
hold 중에는 `max(0, hold_sec - 실제 hold elapsed)`로 계산한다.
`MarkReady()`가 false→true를 한 번만 허용하며 새 active 승격 시 초기화한다.
18/19, 다른 mission/action에는 READY/큐 규칙을 확장하지 않았다.

## G. queue full

B가 있으면 다른 새 C는 거부하고 ACK하지 않는다.
B 동일 ID 재전송은 ACK만 다시 보내며 큐 내용을 바꾸지 않는다.
빈 슬롯도 READY 전에는 사용하지 않는다. READY 뒤에는 LINE 11/12/13만 가능하다.

## H. non-destructive PrepareProgram

`PrepareProgram()`은 const 메서드이며, `dxl=nullptr`인 별도 로컬 parser에서
파일 파싱과 옵션 적용을 수행한다. 활성 player의 motion/frame/step/hold 상태를
수정하지 않으며 Stop, SelectMotion, SyncRead, PD write, Goal write를 호출하지 않는다.
필수 ID 목록은 `Dxl::MotorIds()`의 상수 배열을 읽는다.

실제 모션·mission 참조 파일, 키프레임, 유한한 timing/speed,
interpolation, 23개 ID와 정수 tick 0..4095, 선택적 PD metadata 0..16383,
trailing program과 offset/repeat/final-pose 옵션을 검증한다.
숫자로 변환할 수 없는 timing 필드를 기본값으로 숨기지 않고 거부한다.

큐에는 PreparedProgram 자체를 저장하므로 ACK 뒤 파일이 수정되어도
검증 당시의 프로그램을 실행한다. 실제 시작 시에만 encoder를 읽고 재생 상태를 설정한다.
선택적 PD metadata는 검증만 하며, 이번 로컬 구현의 검증된 고정 PID 초기화를 유지한다.

## I. single writer arbitration

```text
P2P Update → body 23-motor target ─┐
                                ├→ camera override merge → MainNode SyncWriteRawPositions
CameraMotion → yaw/pitch target ─┘
```

P2P Update는 더 이상 모터에 쓰지 않는다. body와 camera에 별도 전송 타이머를
두지 않는다. 전역 write deadline이 패킷 간격을 조절하고, 실제 write 시간은
각 계산기에 전달한다. 늦어진 step을 한꺼번에 따라잡지 않는다.

body의 새 목표가 없으면 마지막으로 성공한 full packet을 base로 사용한다.
카메라 완료 후에도 다음 body packet의 카메라 ID는 최종 camera target으로 덮어쓴다.
오류 시에는 완료 상태를 조작하지 않고 현재 재생을 중단한다.
물리적인 관절 도착 여부를 측정해 DONE을 내는 폐루프 제어는 추가하지 않았다.

## J. camera calibration parameters

| parameter | 기본값 |
|---|---:|
| camera_yaw_motor_id | 21 |
| camera_pitch_motor_id | 22 |
| camera_forward_yaw_tick | 2077 |
| camera_forward_pitch_tick | 1537 |
| camera_down_yaw_tick | 2036 |
| camera_down_pitch_tick | 2013 |
| camera_goal_yaw_tick | 2054 |
| camera_goal_pitch_tick | 966 |
| camera_move_duration_sec | 0.5 |
| camera_settle_sec | 0.2 |
| ready_lead_sec | 0.25 |

FORWARD는 평상시 주행용으로 약 45도 아래를 보는 자세다.
값은 ROS parameter override로 조정할 수 있다.

## K. diff 및 upstream 확인

작업 환경의 `.git`이 비어 있어 일반 git status/branch/diff를 조회할 수 없었다.
GitHub main을 connector로 읽어 로컬과 비교했으며, 확인한 jandi main commit은
`bc9d6bdc570d1a17d2639a4c99c2bb8dcb9e02d2`다.
vision의 세 msg 파일과 vision_core의 최신 mission_controller도 읽어 프로토콜을 확인했다.
로컬 vision simulation에 더 넓은 long-action 분류가 있어도 이를 따라 확장하지 않고
요청된 11/12/13만 대상으로 구현했다.

변경 전 파일을 `/tmp/jandi_executor_before/forward_walk`에 보관했다.
`git diff --no-index`로 전체 변경을 검토하고 `--check`를 통과했다.
`main (copy).cpp`의 변경 전후 SHA256은 동일하다:
`c495c0666b4b2a13b9dc45e185df72b18f8307a3c95cd8827f0b85bbfa067aee`.

추가 수정: `/dev/jandi_dxl`, package.xml `eigen`, DEFAULT_POSITION 파일명
`초기자세_다소곳_.json`, 포트/baud 실패 즉시 예외 처리,
encoder 읽기 실패를 정상 시작으로 오인하지 않도록 통신 결과 확인.
Eigen3 CMake, 기존 11/12/13 매핑, CONTACT_WALK 미구현은 유지했다.

## L. colcon build

ROS 2 Humble에서 다음 명령으로 성공했고 실행 파일도 설치했다.
다른 패키지 또는 캐시를 제거하지 않았다.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select forward_walk
```

기존 Boost bind deprecation 안내 외 빌드 오류 없음.

## M. hardware 없이 수행한 tests

`colcon test --packages-select forward_walk` 및 test-result: **3 tests, 0 failures**.

| test | 확인 내용 |
|---|---|
| executor_logic_test | 실제 11/12/13 파일 검증, 잘못된 JSON/ID/tick/timing/PD/참조/offset 거부, 준비 중 active 보존, 90 ms 지연 step 보존, 마지막 구간 READY/hold, duplicate/queue/history, camera 보간·목표·병합 |
| executor_integration_test | 실제 MainNode를 가짜 Dxl에 연결해 startup, ACK/READY/DONE, READY 이전 거부, 큐 B 자동 시작/추가 ACK 없음, 큐 검증 중 하드웨어 읽기 없음, 파일 변경 후 prepared 실행, 두 ID 공간, 동시/idle camera, override 유지, packet 간격, 실패 시 가짜 DONE 없음 |
| executor_source_contract | Protocol 2.0/4 Mbps/23 motors/Position/P850/고정 step/startup3초/장치/eigen/매핑/단일 writer/CONTACT_WALK/포트·baud 실패 분기 등 17개 계약 확인 |

통합 테스트는 테스트 target에만 가짜 Dxl을 강제 포함한다. 실제 driver constructor나
모터 장치를 사용하지 않는다. ROS localhost DDS가 필요해 제한된 sandbox 밖에서 실행했다.
실제 serial open/baud 실패는 코드 분기 확인이며, 실제 FTDI 장치로 fault injection하지 않았다.

## N. 실제 로봇에서 확인할 순서

1. `/dev/jandi_dxl` 연결과 startup 마지막 WALK_MODE 자세의 3초 이동을 확인한다.
   첫 CameraCommand 전 머리가 기존 JSON 자세를 유지하는지도 확인한다.
2. 기존 11/12/13을 각각 단독 실행해 움직임과 ACK/DONE을 확인한다.
3. LINE A 실행 중 READY 이전 B 거부, READY 뒤 B ACK, A DONE 뒤 B 자동 실행을 확인한다.
   A/B 재전송과 큐가 찬 상태의 C에 대해 재실행·불필요한 ACK가 없는지 확인한다.
4. body idle에서 FORWARD → DOWN → GOAL을 서로 다른 Camera ID로 실행하고
   0.5초 이동·0.2초 settle 및 실제 시야를 조정한다. 다시 body를 실행해 카메라 목표 유지 여부를 확인한다.
5. body와 camera를 동시에 실행하고 vision·카메라 부하를 추가해 패킷 지연과 균형을 확인한다.
   단계 보존은 지연 자체를 제거하지 않으므로 실제 동적 안정성은 별도 실기 검증이 필요하다.

카메라 명령 예시(로봇에서 startup 완료 후 실행):

```bash
ros2 topic pub --once /jandi_vision/camera_cmd vision/msg/CameraCommand '{command_id: 1001, request: 2}'
ros2 topic echo /jandi_vision/camera_status
```

request: DOWN=1, FORWARD=2, GOAL=3. 실제 모터 실행은 이 작업 환경에서 수행하지 않았다.
