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

LINE READY는 current action 취소가 아니다. 긴 LINE action에서 최초 ACK
이후 관측을 모으고, READY가 남은 시간 약 0.25초 전에 도착한다고 가정해
전체 추정 실행시간의 40% 지점부터 READY까지의 유효 LineGuide를
1→3 시간 가중평균해 direct selector로 다음 action 하나를 예약한다.
실제 READY 없이 DONE에 도착한 경우에는 남은 시간을 0으로 취급한다.
current DONE 뒤 queued action을 시작한다. [LINE OBS] 로그에 사용 프레임 수와
집계 구간이 출력되며 유효 표본 최소 개수는 아직 강제하지 않는다.

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
  raw RGB-D pose로 geometry 재계산. GOAL 카메라에서 이전 백보드 관측이 없거나
  마지막 관측이 중앙(설정 기본 ±0.12)이면 Search에서 정지 관측한다. 기존 안정 인식 조건을 충족했던 왼쪽/오른쪽
  마지막 관측을 잃었으면 같은 방향으로 15도 TURN을 실행하고, 완료 후 0.6초
  정착 관측 및 연속 3회 재인식으로 Search에 복귀한다. 방향 복구가 5초 내
  성공하지 않으면 FAILED에서 정지한다.

카메라 trigger는 진행 중 locomotion을 취소하지 않는다. DONE 뒤 camera request를
내고, 카메라 모션이 끝날 때까지 새 locomotion을 HOLD한다.
카메라가 FORWARD/DOWN/GOAL 사이를 이동하는 동안 object association 기록은
갱신하지 않으며, 카메라 시야가 전환되면 identity를 초기화한다. BALL/HURDLE/
GOAL controller도 WaitCamera 단계에서 관측·안정성 이력을 갱신하지 않고,
카메라 완료 시 이전 시야의 tracking 이력을 비운 뒤 새 영상으로 판단한다.
카메라 이동시간 동안의 검출은 미세조정과 미션 진행 판정에 사용하지 않는다.
일반 Action/Camera 오류 이후 자동 복구나 timeout 자동 해제는 적용하지 않는
의도적인 fail-stop 정책이다. A→B 전환 때 실제 엔코더 재조회는 유지한다.

## Build and test

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$PWD/install"
cmake --build build -j
ctest --test-dir build --output-on-failure
cmake --install build
```

## DONE 이후 관측 창 (실기 기본값)

- `mission.post_motion_observation_sec=1.0`: 짧은 움직임 및 카메라 DOWN/GOAL 완료 이후
  이전 추적 이력을 초기화하고, 새로운 영상으로 최소 1초 관측한다.
- BALL/HURDLE/GOAL: 이 관측 구간에서 촬영한 최근 **10프레임 중 최소 7회**
  유효 검출이 있어야 객체 기반 다음 동작을 선택한다. 프레임 7개만 도착했다고
  즉시 결정하지 않으며, 새 관측이 부족하면 추가 명령 없이 HOLD한다.
- 일반 DONE은 최소 1초 관측을 마칠 때까지 coordinator에 전달하지 않는다.
  모션 도중의 추적값을 다음 동작의 판단 근거로 재사용하지 않는다.
- GOAL/BALL 카메라 전환은 완료 직후 추적을 초기화하며, 새 시야에서의
  1초 관측까지 고정한다. HURDLE DOWN 이후 `STEP_FORWARD_ONE`은 고정 시퀀스이므로
  카메라 관측시간만 적용하고 10/7 타깃 검출은 요구하지 않는다.
- 긴 LINE 11/12/13은 기존 READY 40% 관측 및 one-slot queue 유지.
- LINE 인식 실패의 정지 관측은 2초, LINE 복구 회전 완료 뒤는 1초로 구분한다.
- 픽업 → 재집기 검증, 허들 접근 마지막 1걸음 → 넘기기, 슛 → 복귀 같은
  이미 결정된 연속 시퀀스에는 추가 관측 대기를 삽입하지 않는다.
- 실기 ROS 이미지 어댑터는 DONE 수신 이전 timestamp의 이미지를 폐기한다.
  카메라/시스템 ROS clock과 이미지 header stamp가 같은 시계 기준이어야 한다.
