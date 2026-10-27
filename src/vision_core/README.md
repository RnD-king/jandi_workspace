# vision_core

ROS와 하드웨어에 의존하지 않는 C++ perception·mission-control 라이브러리다.
실제 ROS2 adapter는 `/home/noh/my_cv/src/vision`에 있고, 알고리즘 기본값은
`config/vision_algorithm.yaml`이 단일 원본이다. 필수 키가 없거나 범위를
벗어나면 설정 로딩 단계에서 예외를 발생시킨다.

## 객체별 YAML 구성

운영 `config/vision_algorithm.yaml`은 다음 다섯 묶음을 사용한다.

```text
vision_algorithm
├─ line
│  ├─ detection       클래스·신뢰도·BBox 최소 크기
│  ├─ features        중심점·최소제곱·RANSAC
│  ├─ stability       LINE 검출 누적
│  └─ walk            보행 점수·READY·라인 복구
├─ ball
│  ├─ enabled         공 미션 사용 여부
│  ├─ detection       신뢰도·BBox·비율·FORWARD/DOWN Depth
│  ├─ candidate_tracking
│  └─ control         검출 누적·DOWN 탐색·접근·픽업·복구
├─ hurdle
│  ├─ enabled
│  ├─ detection       신뢰도·BBox·비율·Depth
│  ├─ candidate_tracking
│  └─ control         검출 누적·DOWN 탐색·접근·넘기·복구
├─ goal
│  ├─ enabled
│  ├─ detection       GOAL 클래스 검출 필터
│  ├─ backboard
│  │  ├─ detection    백보드 클래스 검출 필터
│  │  ├─ depth        백보드 중심 거리 범위·좌우 표본 폭
│  │  └─ candidate_tracking
│  └─ control         접근·투척 위치/방향·복구
└─ common
   ├─ depth_roi_size_px  공·허들 BBox 중심 Depth 표본 크기
   ├─ object_association 공통 단일 객체 연결 설정
   ├─ command            명령 ID·ACK 제한 시간
   └─ mission            시험 미션·공통 복구·카메라·관측 대기·시작 상태
```

예를 들어 공의 FORWARD Depth 상한은
`vision_algorithm.ball.detection.forward_max_depth_m`, 공 진입 검출 횟수는
`vision_algorithm.ball.control.entry_min_hits`, 공통 정지 관측 시간은
`vision_algorithm.common.mission.post_motion_observation_sec`에서 변경한다.
`line.detection.confidence`는 실전 노드의 YOLO 전체 1차 신뢰도 기본값으로도
사용한다. 객체별 신뢰도를 더 낮게 주어도 1차 필터에서 제외된 검출은 복원되지 않는다.

검출 필터, 후보 ID 추적, `control.lost_frames`는 서로 다른 단계의 설정이다.
필터 통과가 미션 진입을 뜻하지는 않는다. 기존 C++ 설정 필드와 ROS 파라미터
이름, 미션 FSM 및 ACK/READY/DONE 계약은 유지한다.

이 객체별 형식을 처음 사용하기 전에는 새 로더를 포함한 코어를 한 번
빌드·설치해야 한다. 이후 값만 변경할 때는 노드가 실제로 읽는 설치본에
YAML을 반영하고 재시작하면 된다. 기존 `object_targets`, `line_features`,
`*_candidate_tracking`, `mission` 등이 루트에 있는 이전 YAML도 계속 읽는다.
고정 회귀 시나리오 `tests/fixtures/mission_scenarios.yaml`은 그 이전 형식을
유지하며 운영 튜닝값으로 다시 생성하지 않는다.

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
(`line.features.guide_min_points: 3`). 2점 이하는 기존 failure/recovery로 처리한다.

- 첫 실패: 2초 stationary observation
- 유효 O/H 표본 5개 이상: 평균 guide로 정상 복귀
- 5개 미만 + 방향 기억: 같은 방향 TURN 15도 후 2초 재관측, 최대 5회
- 5개 미만 + 방향 기억 없음: 회전 없이 2초 재관측을 최대 5회 추가
- 한도 초과: Reset 전까지 FINAL HOLD

불안정 표본은 방향 기억을 갱신하지 않는다. curvature는 별도 validity와 분모로
누적하는 진단값이며 steering, mission 전환, recovery 결정에 사용하지 않는다.

## Mission 요약

BALL/HURDLE 후보의 탐색 방향에는 시간 만료를 적용하지 않는다. 최초 후보의
좌우 위치를 기준으로 탐색 방향을 고정하고, 실제 DONE으로 완료한 탐색 회전과
반대 방향 복원 횟수를 관리한다. 탐색 한도는
`common.mission.candidate_search_max_turns`를 사용한다.

- BALL: 원거리 direct cruise → 카메라 DOWN → lateral 우선 fine → 전후 보정
  → PICK_BALL(내부 재그립·일어나기 포함) → 잔존 공 확인 → 기본 자세
  → 설정된 후진(`post_pickup_back_steps`, 기본 0회)
  → 카메라 FORWARD → LINE recovery
  (PICK_BALL(일어나기 포함) DONE 후 10-frame verification에서 3회 이상 검출되면 남은 attempt에
  한해 FineAdjust로 복귀한다. 잔존 공 기준은 `ball.control.pickup_verify_min_hits`이며,
  일반 안정 검출의 `stable_min_hits`와 별개다. 이 관찰 상태에서만 association 없이 raw 최상위
  ball 후보를 사용하며, 다음 일반 tracking은 새 identity를 acquire한다.)
- HURDLE: F5 반복 → raw v 0.75, 10-window/7-hit latch → 현재 action DONE
  → camera DOWN → STEP_FORWARD_ONE(10) → HUDDLE(16)
- GOAL: camera GOAL → backboard RGB-D geometry → 거리 우선 fine → shoot yaw
  ±30도 이내 signed SHOOT, 아니면 side step 후 settle → 현재 frame의 fresh
  raw RGB-D pose로 geometry 재계산. 첫 GOAL 정지 관측에서 백보드가 한 번도
  보이지 않으면 FORWARD 도달 확인 → 기존 LINE 복구/판단 → 새 LINE 보행
  1블록의 실제 DONE → GOAL 재관측을 수행한다. 추가 재탐색 한도는
  `goal.control.entry_line_retry_max_attempts`(기본 3회)이며, 한도 뒤의 관측도
  비어 있으면 FAILED에서 정지한다. 0이면 기존 방향/정지 복구를 사용한다.
  LINE 블록 길이는 기존 `line.walk.*_repetitions`를 따른다. READY나 과거 DONE은
  재관측을 허용하지 않으며, 재탐색은 공 보유 상태와 GOAL 미션 번호를 유지한다.
  백보드가 잠깐이라도 보였거나 영상은 있으나 Depth가 실패한 경우, 접근 중
  기존 백보드를 잃은 경우는 기존 방향 회전·정지 관측 복구를 유지한다.

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

튜닝과 테스트 실행 방법은 [tests/README.md](tests/README.md)를 참고한다.
회귀 테스트는 `tests/fixtures/mission_scenarios.yaml`의 고정된 시나리오를 사용한다.
운영 `config/vision_algorithm.yaml`은 별도 검사에서 실제 값의 로딩과 유효성을
확인하므로, 유효한 튜닝값을 변경할 때마다 테스트의 기대 숫자를 수정하지 않는다.

## DONE 이후 관측 창 (실기 기본값)

- `common.mission.post_motion_observation_sec=1.0`: 짧은 움직임 및 카메라 DOWN/GOAL 완료 이후
  이전 추적 이력을 초기화하고, 새로운 영상으로 최소 1초 관측한다.
- BALL/HURDLE/GOAL: 이 관측 구간에서 촬영한 최근 **10프레임 중 최소 7회**
  유효 검출이 있어야 객체 기반 다음 동작을 선택한다. 프레임 7개만 도착했다고
  즉시 결정하지 않으며, 새 관측이 부족하면 추가 명령 없이 HOLD한다.
- 일반 DONE은 최소 1초 관측을 마칠 때까지 coordinator에 전달하지 않는다.
  모션 도중의 추적값을 다음 동작의 판단 근거로 재사용하지 않는다.
- GOAL/BALL 카메라 전환은 완료 직후 추적을 초기화하며, 새 시야에서의
  1초 관측까지 고정한다. HURDLE DOWN 이후 `STEP_FORWARD_ONE`은 고정 시퀀스이므로
  카메라 관측시간만 적용하고 10/7 타깃 검출은 요구하지 않는다.
- 긴 LINE 11/12/23은 기존 READY 40% 관측 및 one-slot queue 유지.
- LINE 인식 실패의 정지 관측은 2초, LINE 복구 회전 완료 뒤는 1초로 구분한다.
- 픽업 → 재집기 검증, 허들 접근 마지막 1걸음 → 넘기기, 슛 → 복귀 같은
  이미 결정된 연속 시퀀스에는 추가 관측 대기를 삽입하지 않는다.
- 실기 ROS 이미지 어댑터는 DONE 수신 이전 timestamp의 이미지를 폐기한다.
  카메라/시스템 ROS clock과 이미지 header stamp가 같은 시계 기준이어야 한다.
