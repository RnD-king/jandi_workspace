# 초FAST: 허들 통과 후 높은 공 탐색 시야

브랜치 `race-round2-ultrafast-20261113`은 FAST에서 분리했다. 설치·실행 버전 이름은
`ultrafast`이며 MAIN/FAST와 다른 `~/jandi_ultrafast` Release를 사용한다.
공 줍기는 `공_줍고일어나기찐찐.json`이며, 픽업 영역은 합의한 7개 꼭짓점이다.

공 전 허들(첫째/둘째 공 구간)의 실제 HURDLE DONE에서만 높은 FORWARD를 요청한다.
기존 FAST의 WALK_MODE/카메라 병행과 ACK/DONE 대기를 유지한다. FRAME/ACK/READY만으로
전환하지 않는다. LINE·곡선 목 탐색에서도 높은 pitch를 유지하고 복귀한다.
BALL_VIEW가 실제로 완료되면 기존 시야와 기존 공 접근 기준으로 돌아간다. Reset도 해제한다.
새 카메라 요청 값은 15(POST_HURDLE_FORWARD)이며 기존 요청 번호와 액션 번호는 그대로다.

설정 파일은 Jetson의 **`~/jandi_ultrafast/src/vision_core/config/vision_algorithm.yaml`**이다.

```yaml
vision_algorithm:
  common:
    camera:
      post_hurdle_ball_view:
        enabled: true
        pitch_tick: 1452
        near_v_px: 240.0
        one_step_v_px: 410.0
```

위는 전체 YAML 중 변경할 부분이다. 기본 FORWARD pitch 1537보다 85tick(약 7.5도)
위쪽을 본다. `near_v_px`는 높은 시야의 근거리 진입, `one_step_v_px`는 이후 전진
2걸음/1걸음 경계다. 양쪽 측면 구역도 회전 후 같은 경계를 사용한다.
240/410은 기존 값으로 시작하므로 높아진 시야에서 사용할 값은 이 두 항목에서 조정한다.
`approach_short_v_px`(100)는 AUTO 원거리 접근에서 사용하지 않는다.
화면의 근거리 가로선도 실제 FSM과 같은 두 값을 표시한다.
YAML 값 변경은 노드를 완전히 종료·재실행하면 적용되며 재빌드는 필요 없다.

## 공 줍기·던지기 후 LINE 복귀 안정 판정

ULTRAFAST에서만 미션 후 LINE 복귀 확인을 기존 8프레임 중 4회에서
**8프레임 중 5회**로 강화한다. 일반 LINE 보행·결승선·허들 판단의
8중4 조건은 바꾸지 않는다. H 허용각 ±30°와 회전 DONE 이후 신규
정지 관측 최소 1초, 30회 회전 한도는 그대로 유지한다.

`vision_algorithm.mission.mission_recovery_stable_min_hits: 5`를 사용한다.
값 0은 기존 일반 LINE 판정 횟수를 상속한다. 1~LINE 창 길이의 정수로
조정할 수 있고, 잘못된 값은 시작 시 검증에서 거부된다.
BALL 픽업과 GOAL 투척 후의 공통 `ComputeMissionLineRecovery` 경로에만
적용한다.

## 커브 확인 후 반대쪽 전진

일반 LINE의 O/H 게인은 `line.walk.offset_gain: 0.2`, `heading_gain: 0.4`다.
결승과 유실 복구의 전용 게인은 기존 값을 유지한다.
`line.curve_search.opposite_walk_before_scan: true`이면 기존 커브 확정 뒤
우커브에서는 좌전진 32번, 좌커브에서는 `right_unit_action`(기본 36번)으로
물리 보행을 한 번만 한다. 반복 횟수 설정과 무관하며 READY로 다음 보행을 예약하지 않는다.
실제 해당 ID의 DONE 뒤 공 진입 조건을 재확인하고, 공이 확인되지 않으면 기존 방향으로
목 탐색을 시작한다. 이미 확인된 공은 추가 보행 시작 전에도 우선한다.
탐색 실패 뒤 재시도는 기존 정상 LINE 1유닛을 유지하며 반대쪽 보행을 반복하지 않는다.
옵션을 끄거나 이전 YAML에서 항목을 생략하면 기존 커브 탐색 순서를 사용한다.

## 기존 Jetson에서 처음 추가하기 (사용자가 실행)

새 버전을 인식하도록 배포 도구를 한 번 갱신한다. 다음 명령은 기존 `~/jandi`가
GitHub 저장소인 설치를 기준으로 한다. 기존 MAIN/FAST 저장소·설치 경로는 보존한다.

```bash
mkdir -p ~/jandi-tools-ultrafast
git -C ~/jandi fetch origin race-round2-ultrafast-20261113
git -C ~/jandi show FETCH_HEAD:src/vision/scripts/jandi_deploy.py > ~/jandi-tools-ultrafast/jandi_deploy.py
/usr/bin/python3 ~/jandi-tools-ultrafast/jandi_deploy.py install --machine jetson
~/jandi_update.sh ultrafast
~/run_jandi.sh ultrafast
```

배포 도구 설치 자체는 로봇/카메라 실행이나 빌드를 하지 않는다. `jandi_update.sh`가
초FAST 소스를 별도 저장소로 가져와 빌드한다. `run_jandi.sh`의 버전 선택은
MAIN → FAST → ULTRAFAST 순서로 순환하며, 각 버전의 소스 YAML을 직접 읽는다.

`ros2 run`으로 직접 시험할 때는 같은 초FAST 설치 환경을 활성화하고
`VISION_CORE_ALGORITHM_CONFIG`를 위 초FAST 소스 YAML로 지정해야 한다.
기존 MAIN/FAST 바이너리는 새 카메라 요청 15를 지원하지 않으므로 함께 섞어 실행하지 않는다.

## 검증 범위

젯슨 `jandi_update.sh`의 코어·ROS 빌드는 이미 `BUILD_TESTING=OFF`이며 회귀 테스트를
빌드하거나 실행하지 않는다. 시작 전 `config_check`는 잘못된 YAML을 거부하는 실행용
검증이라 유지한다. GitHub CI가 테스트를 담당한다.

GitHub CI는 코어·회귀 테스트, 표시 코드, 카메라 모션과 탐색 복귀, 배포 도구의
모의 테스트를 실행한다. 실제 ROS2 메시지 생성과 forward_walk 전체 빌드는 Jetson에서 한다.
`executor_integration_test`에도 YAML pitch/팔 자세/중복 DONE 검증을 추가했다.
PC 원본 소스와 Jetson 파일·설치본은 이 변경 작업에서 수정하지 않았다.
