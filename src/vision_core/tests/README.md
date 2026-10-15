# 튜닝과 회귀 테스트

운영 설정은 `../config/vision_algorithm.yaml`이다. 로봇 노드는 계속 이 설정의
설치본 또는 `VISION_CORE_ALGORITHM_CONFIG`로 지정한 파일을 사용한다.

CTest의 `logic` 검사는 `fixtures/mission_scenarios.yaml`을 사용한다. 이 파일은
거리·각도·프레임·타이머를 고정한 테스트 시나리오이며 운영 기본값이 아니다.
운영 YAML을 튜닝해도 복사하거나 다시 생성하지 않는다. 필수 설정 키를 추가하는
코드 변경처럼 설정 형식 자체가 바뀌는 경우에는 테스트 시나리오도 갱신한다.
컨트롤러의 기본 생성자도 같은 시나리오를 읽도록 CTest가 환경변수를 지정한다.
테스트 바이너리를 직접 실행할 때는 해당 환경변수를 직접 지정해야 한다.

`config` 검사는 현재 운영 YAML을 실제 코어 로더로 읽는다. 모든 필수 키와
타입·값의 유효성을 검사하고, 파일의 각 값이 대응하는 구성 필드에 반영됐는지
확인한다. 운영 각도가 반드시 30도인지 같은 과거 튜닝값 검사는 하지 않는다.
GOAL 회전/FINE 동작은 별도 고정 시나리오에서 15도와 30도를 모두 확인한다.

## YAML 미션별 배치

백보드 Depth 설정은 `goal.backboard_depth_strip_width_px`,
`goal.backboard_min_depth_m`, `goal.backboard_max_depth_m`에서 읽는다.
단독 시험 설정은 `goal.trial_start_goal`, `goal.test_mission_index`,
`ball.test_mission_index`에서 읽는다. 이 항목들의 예전 `mission` 경로는 사용하지
않는다. ROS/rqt 파라미터 이름과 기존 C++ 구성 필드는 유지했다.

`goal.edge_margin_px`는 화면 좌우 끝의 회전 보정 여유 폭(px)이다.
실제 노드에서는 `goal_edge_margin_px`로 설정한다. 필수 키 누락·음수·NaN·무한대는
거부한다. 운영 YAML과 테스트 시나리오 모두 새 배치를 사용한다.

ROS `goal_trial_start_goal` / YAML `goal.trial_start_goal`의 기본값은 false다.
false는 기존 LINE 출발·FORWARD RESET을 유지한다. true는 직접 GOAL 카메라를
요청하고 카메라 완료 후 전환 안정 대기와 공통 관측을 마쳐 접근한다. RESET도
GOAL 카메라로 돌아간다. 예전 `goal_trial_start_forward` / `trial_start_forward`는
새 이름으로 교체해야 한다. 전용 튜닝의 Stage 0~3은 기존 시작 구간을 유지한다.

`mission_revision_test`에 직접 GOAL 시작, 명령 비활성 중 요청 보존, 이미 GOAL인
상태의 새 요청, 0.6초+1초 관측, GOAL/FORWARD RESET 회귀 시나리오를 추가했다.

## 검사 도구 빌드

아래는 사용자가 실행하는 명령이다. 설치하거나 로봇 노드를 시작하지 않는다.

```bash
cmake -S /home/noh/vision_core -B /home/noh/vision_core/build_pc_tests \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DCMAKE_INSTALL_PREFIX=/home/noh/vision_core/install
cmake --build /home/noh/vision_core/build_pc_tests -j4
```

## YAML 튜닝 후

```bash
ctest --test-dir /home/noh/vision_core/build_pc_tests -L config --output-on-failure
```

운영 YAML을 설치한 뒤에는 다음 명령으로 실제 사용 예정 파일과 소스의 내용도
비교한다. 환경변수로 별도 YAML을 지정했다면 그 경로를 검사한다. 주석을 포함한
파일 내용이 다르면 불일치로 보고하며, 파일을 자동으로 복사하거나 고치지 않는다.

```bash
/home/noh/vision_core/build_pc_tests/shared_vision_core_runtime_config_check \
  /home/noh/vision_core/config/vision_algorithm.yaml \
  --installed "${VISION_CORE_ALGORITHM_CONFIG:-/home/noh/vision_core/install/share/shared_vision_core/config/vision_algorithm.yaml}"
```

## 제어 코드 변경 후

```bash
ctest --test-dir /home/noh/vision_core/build_pc_tests -L logic --output-on-failure
# 운영 설정 검사까지 모두 실행:
ctest --test-dir /home/noh/vision_core/build_pc_tests --output-on-failure
```

실제 JSON 및 소스/설치본 비교는 `forward_walk/tests/README.md`를 참고한다.
이 검사들은 실제 카메라 품질, 관절 도착, 보행 안정성이나 실제 투척 성공을
검증하지 않는다. 테스트 구현 변경 후 빌드·실행 결과는 사용자가 확인해야 한다.
