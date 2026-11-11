# MAIN / FAST 수동 시험과 HTTP 화면

MAIN 소스는 `~/my_cv/src/vision`, FAST 소스는
`~/my_cv/variants/fast/src/vision`이다. 코어 원본은 각각 `~/vision_core`,
`~/vision_core_fast`다. `jandi_main` / `jandi_fast`는 설치된 환경을 선택하는
함수이며 소스를 빌드하지 않는다. 코드 변경은 아래처럼 먼저 빌드해야 한다.

## PC에서 미전송 소스 빌드·설치

GitHub에 올리기 전 PC 원본을 빌드할 때 사용한다. 원격 동기화나 push는 하지
않으며 기존 배포 도구의 버전별 빌드 캐시·설치 완료 검사·환경 설정을 사용한다.
`~/jandi_update.sh`는 원격 동기화도 하므로 이 로컬 빌드와 역할이 다르다.
같은 버전의 노드와 다른 업데이트/빌드를 종료한 뒤 실행한다.

```bash
jandi_test_version=main
# FAST를 빌드할 때는 jandi_test_version=fast

/usr/bin/python3 - "$jandi_test_version" <<'PY'
import importlib.util
from pathlib import Path
import sys

path = Path.home() / "my_cv/src/vision/scripts/jandi_deploy.py"
spec = importlib.util.spec_from_file_location("jandi_local_build", path)
deploy = importlib.util.module_from_spec(spec)
spec.loader.exec_module(deploy)
cfg = deploy.load_config()
if cfg.get("machine") != "pc":
    raise SystemExit("PC에서만 실행하는 로컬 소스 빌드입니다.")
version = sys.argv[1]
if version not in deploy.BRANCHES:
    raise SystemExit("main 또는 fast를 지정하세요.")
repo = Path(cfg["repositories"][version]).expanduser()
roots = deploy.source_roots(cfg, version, repo)
# ROS 단위 테스트 실행 파일도 빌드한다. deployment.json은 변경하지 않는다.
cfg["cmake_args"] = list(cfg.get("cmake_args", [])) + ["-DBUILD_TESTING=ON"]
with deploy.operation_lock():
    deploy.require_stopped(version)
    deploy.build_release(cfg, version, repo, roots)
PY
```

이후 해당 환경을 새로 선택한다. PC·Jetson 모두 설치본과 같은 버전을 사용한다.

```bash
jandi_main
# 또는 jandi_fast
```

## 별도 터미널에서 카메라·모션 실행

각 터미널에서 먼저 `jandi_main` 또는 `jandi_fast`를 실행한다.

```bash
# 카메라 터미널: 이미 실행 중이면 중복 실행하지 않는다.
ros2 launch realsense2_camera rs_launch.py

# 모션 터미널
ros2 run forward_walk main_node
```

## 카메라 HTTP 송신

아래 중 하나만 실행한다. 미션 명령 발행 노드를 대회 코드와 동시에 실행하지
않는다. 0은 전체, 1은 원거리 접근, 2는 근거리 정렬, 3은 실행·복귀 시험이다.
구간 3은 물체가 이미 실행 가능한 위치에 있어야 한다.

```bash
ros2 run vision ball_tuning_node --ros-args -p test_stage:=0

ros2 run vision hurdle_tuning_node --ros-args -p test_stage:=0

ros2 run vision goal_tuning_node --ros-args -p test_stage:=1

ros2 run vision goal_shot_preview_node
```

Jetson 브라우저 주소는 `http://192.168.0.133:8080`, PC에서 자기 영상을 볼 때는
`http://localhost:8080`이다. 모든 도구의 기본 포트는 8080이며 같은 장비에서는
하나씩 실행한다. 공·허들·골대 미션 튜닝은 카메라
디버그 영상을 송신한다. `shoot_tuning_node`와 `goal_shot_preview_node`는
기존 Qt GUI 전체를 HTTP로 표시하여 영상 클릭, 직접 좌표 입력, 계산·기본자세·
투척 버튼, 파라미터 편집 및 스크롤을 사용할 수 있다. 자세한 조작은
[SHOOT_TUNING.md](SHOOT_TUNING.md)를 따른다.
PC·Jetson 모두 HTTP 영상 송신은 기본 ON, 로컬 창은 기본 OFF다.
HTTP를 끄려면 `-p stream_debug_view:=false`, 로컬 창을 켜려면
`-p show_debug_view:=true`를 지정한다. JPEG 인코딩·네트워크 처리는 기존 별도 작업 스레드를
사용한다. 브라우저가 없으면 HTTP용 렌더링과 인코딩을 요청하지 않는다.

`goal_shot_preview_node`는 DISPLAY/Wayland가 없는 SSH에서 Qt offscreen을 자동
선택한다. Qt offscreen 플러그인이 필요하며, `show_debug_view=false`에서는
창을 표시하지 않는다. 기존 슈팅 기본자세 자동 준비와 실제 DONE 대기는 유지한다.
공유 소스를 사용하는 `shoot_tuning_node`도 같은 영상 송신 옵션을 지원한다.

## 공·허들·골대 튜닝 서비스

서비스 터미널도 같은 버전 환경과 ROS_DOMAIN_ID=22를 사용한다. 아래 명령은
필요한 시점에 하나씩 실행한다. PREVIEW가 관측을 완료해 WAIT_COMMIT이 된 뒤
COMMIT한다. 서비스의 success는 요청 수락이며 동작 완료는 실제 DONE으로 판단한다.

```bash
jandi_node=ball_tuning_node
# jandi_node=hurdle_tuning_node
# jandi_node=goal_tuning_node
jandi_stage=2

# 단계 설정은 WAIT_START에서만 가능하다.
ros2 param set "/$jandi_node" test_stage "$jandi_stage"
ros2 service call "/$jandi_node/start" std_srvs/srv/Trigger "{}"
ros2 service call "/$jandi_node/preview" std_srvs/srv/Trigger "{}"
ros2 service call "/$jandi_node/commit" std_srvs/srv/Trigger "{}"

# 다음 반복을 중지한다. 이미 접수된 동작은 실제 DONE까지 완료한다.
ros2 service call "/$jandi_node/stop_repeat" std_srvs/srv/Trigger "{}"

# 몸·카메라 동작이 끝난 뒤 초기화한다. 복귀 완료 후 WAIT_START를 기다린다.
ros2 service call "/$jandi_node/reset" std_srvs/srv/Trigger "{}"
```

RESET은 선택한 test_stage를 유지한다. 공·허들·골대 튜닝에는 pause/resume
서비스가 없다. 다른 구간을 시험하려면 WAIT_START에서 test_stage를 변경한다.
골대 1구간은 GOAL 시야 원거리 접근부터 FINE 발 전환 직전까지다.
허들 2구간은 정렬 후 접촉 보행까지 진행하고 실제 넘기 직전에 종료한다.

## 투척 전용 서비스

test_stage는 사용하지 않는다. 각 단계의 실제 완료/응답을 확인하면서 호출한다.

```bash
ros2 service call /goal_shot_preview_node/start std_srvs/srv/Trigger "{}"
ros2 service call /goal_shot_preview_node/preview std_srvs/srv/Trigger "{}"
ros2 service call /goal_shot_preview_node/commit std_srvs/srv/Trigger "{}"
```

START는 슈팅 기본자세 준비, PREVIEW는 새 RGB-D 관측으로 허리각 계산,
COMMIT은 저장한 각도로 한 번 투척이다. pause/reset 서비스는 없다.
투척 후 실제 DONE을 받은 다음 START로 다시 준비한다.

## line_perception_node 서비스

수동으로 직접 실행하고 별도 namespace/remap을 지정하지 않은 경우다.

```bash
ros2 run vision line_perception_node

# 별도 서비스 터미널에서 필요한 명령을 하나씩 호출한다.
ros2 service call /start std_srvs/srv/Trigger "{}"
ros2 service call /pause std_srvs/srv/Trigger "{}"
ros2 service call /resume std_srvs/srv/Trigger "{}"
ros2 service call /reset std_srvs/srv/Trigger "{}"
```

PAUSE는 접수된 몸·카메라 동작의 실제 DONE을 기다려 PAUSED가 된다.
RESUME은 PAUSED에서 재관측 후 이어간다. RESET은 PAUSED에서만 가능하며
물리적 자세·카메라 복귀가 완료되면 WAIT_START가 된다. 그 뒤 START한다.
Ctrl+C로 제어 노드를 종료해도 executor가 이미 접수한 모션은 취소되지 않는다.

일반 `perception_tuning_node`도 HTTP 옵션을 지원한다. namespace/remap 없는
직접 실행에서는 `/start`, `/preview`, `/commit`, `/stop_repeat`, `/reset`을 사용한다.

## 이번 튜닝값

MAIN `~/vision_core/config/vision_algorithm.yaml`, FAST
`~/vision_core_fast/config/vision_algorithm.yaml`에 각각 적용한다.

- LINE 최소 박스 가로·세로: 각각 15px.
- 허들 `fine_line_offset_tolerance_px`: 200px (±200px).
- 공 `fine_pose_pre_hold_sec`: 0.5초. DOWN/BALL_VIEW에서 발견 후 2번 발 전환
  요청 직전에 대기한다. 콜백은 계속 처리하며, 이전 보행의 DONE을 발 전환 DONE으로
  사용하지 않는다. 0으로 설정하면 전환 전 대기를 생략한다.
- 골대 `edge_margin_px`: 70px. 원본 박스 중심 u로 판단한다.

## 사용자가 실행할 검사

위 PC 로컬 빌드를 마친 뒤, 같은 터미널의 jandi_test_version 값으로 실행한다.
카메라나 모터를 시작하지 않는 단위 검사다. 이번 작업에서는 빌드·설치·검사를
실행하지 않았으며 Jetson HTTP 화면과 실제 보행은 장비에서 별도로 확인한다.

```bash
jandi_core_source="$HOME/vision_core"
if [ "$jandi_test_version" = fast ]; then
  jandi_core_source="$HOME/vision_core_fast"
fi
jandi_test_release="$(/usr/bin/python3 - "$jandi_test_version" <<'PY'
import json
from pathlib import Path
import sys
print(json.loads((Path.home()/f"jandi_{sys.argv[1]}"/"current.json").read_text())["release"])
PY
)"
cmake -S "$jandi_core_source" -B "$jandi_test_release/build/core-tests" \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build "$jandi_test_release/build/core-tests" -j2
ctest --test-dir "$jandi_test_release/build/core-tests" \
  -R 'stationary_scan|scan_body_correction|scan_recovery_sequence|ball_pickup_verification|ball_fine_pose_hold|ball_controller|pickup_polygon|curve_search|curve_rearm|mission_controller|mission_revision|staged_mission|unified_sequence|continuous_object_approach|config_loader' --output-on-failure
ctest --test-dir "$jandi_test_release/build/ros/vision" \
  -R 'vision_(debug_mjpeg|qt_web_view|mission_parameters)_test' --output-on-failure
ctest --test-dir "$jandi_test_release/build/ros/forward_walk" \
  -R 'scan_motion_test|executor_source_contract' --output-on-failure
```

로컬 확인을 마치면 기존 push 명령으로 해당 버전을 GitHub에 올린다.
Jetson에서는 `~/jandi_update.sh main` / `~/jandi_update.sh fast`로 내려받아
빌드·설치한다. 이 문서의 로컬 빌드 명령은 PC용이다.

LINE 복구·목 탐색·픽업 전 재검증의 확정 동작과 새 파라미터는
[SCAN_RECOVERY_UPDATE.md](SCAN_RECOVERY_UPDATE.md)에 정리했다.
