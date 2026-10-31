# 실행기 로직과 현재 모션 파일 검사

운영 JSON은 `../motions/`, `../missions/`에서 수정한다. 로직 테스트의 목표 tick,
속도, 시간, 미션 순서는 테스트가 직접 생성하는 전용 JSON의 기대값이다.
운영 JSON의 튜닝값을 그 기대값에 맞춰 바꾸지 않는다.

- `logic`: 테스트용 JSON과 가짜 Dxl로 ACK/READY/DONE, 한 슬롯 큐, 보행 간격,
  보간·속도·시간·모터 범위 검증, 미션 참조 순서와 반복을 검사한다. 기본 보행
  간격은 테스트에서 명시하므로 운영 YAML의 간격을 바꿔도 이 시나리오는 유지된다.
- `assets`: 현재 실행기 소스의 `ProgramFileForAction`에서 JSON 경로를 읽고,
  실제 실행기의 `PrepareProgram`으로 모든 매핑과 참조 모션을 파싱한다.
  모터 구성·tick 범위·시간·속도 등 실행기가 거부하는 파일을 확인하며 특정한
  과거 속도, 키프레임 번호, 미션 파일의 이전 배열을 정답으로 고정하지 않는다.
- `motion_assets_check --installed-share DIR`: 소스와 설치 경로를 각각 검사하고,
  두 경로의 `motions/`, `missions/` 아래 모든 JSON을 파일명·내용으로 비교한다.
  누락, 변경, 설치본에만 남은 파일을 구분해 보고한다. colcon의 파일 심볼릭
  링크도 지원한다. 실제 노드에서 별도 `p2p_asset_directory`를 지정했다면 그
  디렉터리를 `--installed-share`에 직접 넘긴다.

매핑의 action 번호와 고정 ROS/실행기 계약 검사는 계속 유지한다. JSON을 실제로
읽는 검사는 모션을 시작하거나 모터·엔코더·시리얼 장치를 사용하지 않는다.
`logic`의 통합 테스트도 실제 장치 대신 가짜 Dxl를 사용한다.

READY 종료 전 여유 시간은 비전과 같은 `vision_algorithm.yaml`의
`line.walk.ready_lead_sec`에서 읽는다(현재 0.25초). 다른 값의 ROS override는
시작 시 거부하며 실행 중에는 읽기 전용이다. YAML 변경 후 두 노드를 재시작한다.
통합 테스트는 별도 YAML 사본을 0.20초로 바꿔 고정된 기본값이 사용되지 않는지
검사한다. 운영 YAML은 수정하지 않는다.

GOAL 화면 경계 보정은 기존 좌 31번/우 28번을 같은 방향으로 두 번 실행한다.
코어가 첫 실제 DONE 직후 관측 없이 다음 명령을 내보내며, 각 명령은 서로 다른
ID와 각각의 ACK/DONE을 갖는다. READY, 모션 JSON, 실행기의 기존
`walk_inter_action_hold_sec`(기본 0초) 규칙은 유지한다. 통합 테스트는 두 명령이
각각 완료되고, 늦게 도착한 첫 명령의 중복 전달이 두 번째 실행을 바꾸지 않는지
확인한다. 관측 생략·최종 관측·튜닝 단일 COMMIT은 코어 회귀 테스트가 담당한다.

## 10.16.1 보행 파일 연결

10번과 호환용 35번은 모두 `보행_1걸음_초기X_수정3.json`을 사용한다.
21번은 같은 모션을 두 번 반복하는 `2걸음_1차찐.json`을 사용하고,
22/23번과 13번의 연속걷기도 같은 1걸음 모션을 반복한다.
연속된 1걸음 명령도 10/35번을 교대하지 않고 요청된 10번을 그대로 실행한다.
11/12번은 좌/우 `3회`, 24/25번은 좌/우 `2회`, 26/27번은 우/좌 `1회` 미션으로 연결한다.
25번의 새 경로는 `missions/우회전전진_2회.json`이며 `_ball` 접미사는 없다.
좌/우 미션은 각각 새로 받은 `_수정` / `_찐` 원본 모션을 참조한다.
기존 JSON은 그대로 남기며, 소스 계약 검사와 통합 테스트의 전용 미션 파일명도
새 매핑에 맞춘다. 운영 모션의 목표 tick·속도·시간은 테스트 상수로 고정하지 않는다.

## 10.30 LINE 직진 단위 연결

33번 `LINE_STRAIGHT_UNIT`은 `missions/2걸음_1차찐.json`을 사용한다.
이 미션은 `보행_1걸음_초기X_수정3.json`을 두 번 재생한다.
각 모션은 4프레임·0.59초이며 전체는 8프레임·1.18초다.
두 모션 전체가 하나의 action ID와 ACK/READY/DONE 수명주기를 갖는다.
`line.walk.straight_repetitions`는 이 2걸음 묶음의 반복 횟수다.
연속 객체 접근이 활성화되어 33번을 선택할 때도 같은 미션을 사용한다.
비전의 모의 피드백은 33번 시간을 `simulate_step_forward_one_duration_sec`의
2배로 계산한다. 실기에서는 JSON 재생에 따른 실제 실행기 피드백을 기다린다.

2번 FINE 기본자세는 `미세보행_초기자세가기_수정2.json`, 16번 허들 넘기는
`허들넘기_real.json`을 사용한다. 허들은 새 왼발 모션을 적용하고 오른발은
기존 `허들_오른발만_수정_2.json`을 유지한다.

## 검사 도구 빌드

현재 코어와 vision의 설치가 준비된 뒤 사용자가 실행한다. 아래 명령의 colcon
빌드는 실행기 설치를 포함하며, 검사 자체는 파일을 수정하거나 설치하지 않는다.

```bash
source /opt/ros/humble/setup.bash
source /home/noh/my_cv/install/setup.bash
export VISION_CORE_ROOT=/home/noh/vision_core/install
cd /home/noh/my_cv
colcon build --packages-select forward_walk --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -Dshared_vision_core_DIR=/home/noh/vision_core/install/lib/cmake/shared_vision_core
```

## JSON 튜닝 후

```bash
ctest --test-dir /home/noh/my_cv/build/forward_walk -L assets --output-on-failure
```

설치까지 반영한 뒤에는 현재 ROS 환경에서 선택하는 패키지의 파일을 비교한다.
불일치는 자동 수정하지 않는다.

```bash
source /home/noh/my_cv/install/setup.bash
/home/noh/my_cv/build/forward_walk/motion_assets_check \
  --installed-share "$(ros2 pkg prefix forward_walk)/share/forward_walk"
```

`--asset-root DIR`로 비교 기준 소스를 바꿀 수 있다. 실행기 매핑은 항상 이 검사
바이너리를 빌드한 소스의 `main.cpp`에서 읽는다. 설치된 실행 바이너리 자체가
최신인지, 실행 중인 노드가 이미 준비한 모션을 다시 로딩했는지는 이 검사로
판별하지 않는다. 파일을 반영하고 노드를 다시 시작하는 과정은 별도로 필요하다.

## 실행기 코드 변경 후

```bash
ctest --test-dir /home/noh/my_cv/build/forward_walk -L logic --output-on-failure
# 현재 모션 파일 검사까지 모두 실행:
ctest --test-dir /home/noh/my_cv/build/forward_walk --output-on-failure
```

소스의 JSON만 파싱할 때는 설치를 요구하지 않는다. 소스/설치본 비교는 배포 후
명시적으로 실행하므로, 아직 설치하지 않은 튜닝 때문에 로직 테스트가 실패하지
않는다. 실제 보행·관절 도착·투척 결과는 실기 검증이 필요하다.
