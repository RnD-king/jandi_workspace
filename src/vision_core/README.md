# vision_core

ROS와 하드웨어에 의존하지 않는 C++ perception·mission-control 라이브러리다.
실제 ROS2 adapter는 `/home/noh/my_cv/src/vision`에 있고, 알고리즘 기본값은
`config/vision_algorithm.yaml`이 단일 원본이다. 필수 키가 없거나 범위를
벗어나면 설정 로딩 단계에서 예외를 발생시킨다.

LINE→객체 연속 접근의 T/F 선택, 3회 완료 후 락, 카메라 임시 보정값과 검증 명령은
[CONTINUOUS_OBJECT_APPROACH.md](CONTINUOUS_OBJECT_APPROACH.md)를 참고한다.

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
전체 추정 실행시간의 `line.walk.observation_start_ratio` 지점
(현재 운영값 `0.20`, 20%)부터 READY까지의 유효 LineGuide를
1→3 시간 가중평균해 direct selector로 다음 action 하나를 예약한다.
시작 비율은 `0 이상 1 미만`이며 `0.20`이면 20%부터 수집한다.
YAML 변경 뒤 설치본에 반영하고 노드를 재시작한다. 객체 접근의 READY
관측 정책은 별도이며 이 LINE 설정으로 바뀌지 않는다.
실제 READY 없이 DONE에 도착한 경우에는 남은 시간을 0으로 취급한다.
current DONE 뒤 queued action을 시작한다. [LINE OBS] 로그에 사용 프레임 수와
집계 구간이 출력되며 유효 표본은 `line.walk.failure_min_valid_samples`
개 이상이어야 한다.

`forward_walk` 실행기도 같은 설치본 YAML의 `line.walk.ready_lead_sec`를
읽는다(현재 `0.25`초). 별도의 다른 ROS override는 거부한다. 같은 파일을
사용하려면 양쪽 프로세스에 동일한 `VISION_CORE_ALGORITHM_CONFIG` 경로를
지정할 수 있다. YAML 수정 후 두 노드를 재시작한다.

현재 action의 ACK는 실행기가 action을 수락하고 시작했다는 뜻이다. READY 뒤
발행된 queued action의 ACK는 실행기가 그 ID와 payload를 내부 queue에 실제로
저장했다는 뜻이며, current DONE 직후 정확히 한 번 실행해야 한다. 저장하지
못한 command에는 ACK하지 않고, 같은 ID의 재수신은 중복 실행하지 않는다.

## LINE recovery

입력 중심점이 3개 이하이면 그 프레임의 guide는 invalid다. 처음 4개 이상에서
RANSAC으로 걸러져 3개가 남은 경우에는 fitting할 수 있다. 보행 중 10프레임
연속 invalid면 미발행 반복을 취소하고 이미 발행된 동작의 DONE을 기다린다.
마지막 READY 평균의 유효 표본이 `failure_min_valid_samples`(현재 5)개 미만인
경우에도 DONE 뒤 복구를 시작한다.

- 첫 실패: `failure_observation_sec`(현재 2초) 정지 관측.
- 유효 guide 표본 5개 이상: 평균 guide로 정상 복귀하고 두 복구 횟수를 초기화.
- 관측 실패: 현재 프레임의 유한한 점선 중심점 2개 이상이면 평균 X를 실제
  LINE 기준 중심과 비교해 좌/우 회전. 각도·한도는
  `visible_recovery_turn_yaw_deg` / `visible_recovery_max_turns`(현재 15도/5회).
- 1개 이하 또는 평균 X가 정확히 기준 중심이면 마지막 유효 판단 점수의 부호로
  회전. 음수는 좌, 0 이상과 기록 없음은 우. 직진 deadband 안의 점수도 보관한다.
  각도·한도는 `last_score_recovery_turn_yaw_deg` /
  `last_score_recovery_max_turns`(현재 15도/5회).
- 회전의 실제 DONE과 settle 뒤 `recovery_observation_sec`(현재 1초) 정지 관측.
  실패하면 위 우선순위를 다시 평가한다. 경로를 바꿔도 각 경로의 횟수는 누적된다.
- 선택한 경로의 한도가 소진되면 Reset 전까지 FINAL HOLD.

옛 `no_evidence_max_retries`와 단일 `recovery_turn_yaw_deg`는 위 두 경로로
대체했다. `recovery_max_turns`는 유효한 `|score| > in_place_turn_score`일 때의
기존 WALK 회전 보정 한도로만 사용한다. 유실 복구의 TURN_LEFT/RIGHT는
기존 FINE 자세 전환과 실제 ACK/DONE을 유지한다.

마지막 점수는 유효한 보행 판단에서만 갱신하며 유실 프레임이나 희소 점선의
위치로 덮어쓰지 않는다. Reset/미션 후 라인 재획득 시작 시 초기화한다.
curvature는 진단값이며 steering, mission 전환, recovery 결정에 사용하지 않는다.

## Mission 요약

BALL의 진입 직후 첫 관측 실패와 일반 FORWARD 연속 유실은 같은 탐색 순서를
사용한다. 저장된 공 위치가 중심 허용 범위 밖이면 해당 방향 WALK 제자리 회전
1회 → 전진 1걸음 → 정지 관측이다. 중앙/방향 미확인은 정지 관측부터 시작하고,
실패하면 회전 없이 전진한다. 각 전진 뒤 새 10프레임/7회 검출과
`ball.control.recovery_observation_sec`(1초)를 확인한다.
전진 실제 DONE 한도는 `entry_recovery_max_steps`(진입 직후)와
`recovery_max_steps`(일반 접근), 각각 현재 3회다. 진입 직후 한도 소진 시에는
픽업 성공 처리 없이 공 모드를 해제하고 정상 LINE 재획득·보행으로 복귀한다.
일반 접근은 한도 소진 후 정지 관측 최대 5회, 이후 FAILED다.
중간 좌우 구역(4·6)은 기존 `recovery_walk_turn_max`(3회)의 회전 후 제자리
관측만 사용한다. 기존 `entry_recovery_walk_turn_max` 키는 제거했다.

HURDLE 진입 직후 첫 정지 관측이 실패하면 직진 1걸음 → 정지 관측을
`hurdle.control.entry_recovery_max_steps`(현재 3회)까지 시도한다. 끝까지
못 찾으면 허들 넘기를 실행하지 않고 정상 LINE 재획득·보행으로 복귀한다.
DOWN 접근/미세 조정 중 `lost_frames`(15) 연속 유실이면 접근은 일반 1걸음,
FINE은 미세 전진 1회 → 정지 관측을 `hurdle.control.recovery_max_steps`
(현재 3회)까지 반복한다. 그 뒤에도 못 찾으면 기존 1초 정지 관측 최대 5회,
이후 FAILED다. 복구 전진의 실제 DONE만 횟수에 포함하고 매번 새 10/7 표본과
`recovery_observation_sec`를 확인한다. 초기 DOWN 탐색과 접촉/넘기 시퀀스의
별도 한도는 이 복구 전진 횟수와 독립적이다.

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
  접근·근거리에서 백보드를 연속 `goal.control.lost_frames`만큼 잃으면 마지막
  안정 위치로 복구한다. 중앙(`recovery_center_tolerance_px` 이내)이면 WALK는
  일반 전진 1걸음(10), FINE은 미세 전진 1회(3) 후 새 정지 관측을 한다.
  좌우면 기존 방향으로 회전하고, 실제 DONE 뒤 관측에서 다시 찾으면 같은
  방향으로 추가 1회 회전한 뒤 다시 관측하고 이전 단계로 복귀한다. 추가 회전은
  탐색 회전 한도와 별도이며 반복해서 추가하지 않는다. 전진·추가 회전 뒤에도
  못 찾으면 기존 정지 복구로 넘어간다. 방향 기록이 없거나 백보드 영상은 있지만
  Depth·자세만 실패하면 정지 관측한다. 화면 끝 보정의 고정 2회전은 별도다.

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
