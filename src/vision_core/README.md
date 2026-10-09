# vision_core

ROS와 하드웨어에 의존하지 않는 C++ perception·mission-control 라이브러리다.
실제 ROS2 adapter는 `/home/noh/my_cv/src/vision`에 있고, 알고리즘 기본값은
`config/vision_algorithm.yaml`이 단일 원본이다. 필수 키가 없거나 범위를
벗어나면 설정 로딩 단계에서 예외를 발생시킨다.

## P2P-only command contract

`MissionController::StepPerception()`에 detection, depth, camera feedback,
ACK/READY/DONE을 입력하면 `ControlCommand` 하나가 반환된다. 출력은 기존
1–19 action ID와 예약 ID `CONTACT_WALK=20`, `action_id`, 카메라 요청으로만
구성된다. 현재 HURDLE contact는 실행기 mapping이 준비될 때까지 ID 10을 쓴다.

정상 LINE/BALL APPROACH/GOAL APPROACH는 관측값에서 direct 3-way cruise를
선택한다. direct 대상인데 관측이 invalid라 `NONE`이면 HOLD하며 다른 경로로
fallback하지 않는다. fine/search/recovery/특수 동작은 각 mission FSM이 discrete
action으로 직접 결정한다.

LINE READY는 current action 취소가 아니다. 실행 중 긴 LINE action의 후반부
LineGuide를 집계해 같은 direct selector로 action 하나만 예약하고, current DONE
뒤 queued action을 시작한다.

현재 action의 ACK는 실행기가 action을 수락하고 시작했다는 뜻이다. READY 뒤
발행된 queued action의 ACK는 실행기가 그 ID와 payload를 내부 queue에 실제로
저장했다는 뜻이며, current DONE 직후 정확히 한 번 실행해야 한다. 저장하지
못한 command에는 ACK하지 않고, 같은 ID의 재수신은 중복 실행하지 않는다.

## LINE recovery

하단 O/H guide는 최소 3개 line center가 있어야 valid다
(`line_features.guide_min_points: 3`). 2점 이하는 기존 failure/recovery로 처리한다.

- 첫 실패: 2초 stationary observation
- 유효 O/H 표본 5개 이상: 평균 guide로 정상 복귀
- 5개 미만 + 방향 기억: 같은 방향 TURN 15도 후 2초 재관측, 최대 5회
- 5개 미만 + 방향 기억 없음: 회전 없이 2초 재관측을 최대 5회 추가
- 한도 초과: Reset 전까지 FINAL HOLD

불안정 표본은 방향 기억을 갱신하지 않는다. curvature는 별도 validity와 분모로
누적하는 진단값이며 steering, mission 전환, recovery 결정에 사용하지 않는다.

## Mission 요약

- BALL: 원거리 direct cruise → 카메라 DOWN → lateral 우선 fine → 전후 보정
  → PICK_BALL → 한 번 STEP_BACK → 카메라 FORWARD → LINE recovery
  (RECATCH 후 10-frame verification에서 7회 이상 검출되면 남은 attempt에
  한해 FineAdjust로 복귀한다. 이 관찰 상태에서만 association 없이 raw 최상위
  ball 후보를 사용하며, 다음 일반 tracking은 새 identity를 acquire한다.)
- HURDLE: F5 반복 → raw v 0.75, 10-window/7-hit latch → 현재 action DONE
  → camera DOWN → STEP_FORWARD_ONE(10) → HUDDLE(16)
- GOAL: camera GOAL → backboard RGB-D geometry → 거리 우선 fine → shoot yaw
  ±30도 이내 signed SHOOT, 아니면 side step 후 settle → 현재 frame의 fresh
  raw RGB-D pose로 geometry 재계산

카메라 trigger는 진행 중 locomotion을 취소하지 않는다. DONE 뒤 camera request를
내고, camera settled 동안에는 locomotion을 HOLD한다.

## Build and test

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$PWD/install"
cmake --build build -j
ctest --test-dir build --output-on-failure
cmake --install build
```
