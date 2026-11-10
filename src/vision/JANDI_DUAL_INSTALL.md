# MAIN / FAST 독립 설치와 최종 제어 변경

두 브랜치는 각각 `main`, `race-round2-fast-20261109`를 사용한다.
`backup/pre-final-main-20261108`은 fetch/push/수정 대상이 아니다.
실제 로봇·모터·카메라는 이번 구현·자동 검증에서 실행하지 않는다.

## 알고리즘 설정

실제 실행은 선택 버전의 **소스 `vision_core/config/vision_algorithm.yaml`**을 직접 읽는다.
이 파일은 GitHub push/update 대상이다. PC에서 바꾼 값을 push하고 Jetson에서 해당 버전을
update하면 같은 값이 적용된다. 기존 홈 튜닝값을 병합하거나 유지하는 별도 복사본은 없다.

| 장비 | MAIN | FAST |
|---|---|---|
| PC | `~/vision_core/config/vision_algorithm.yaml` | `~/vision_core_fast/config/vision_algorithm.yaml` |
| Jetson | `~/jandi_main/src/vision_core/config/vision_algorithm.yaml` | `~/jandi_fast/src/vision_core/config/vision_algorithm.yaml` |

로컬에서 이 파일을 저장한 뒤 JANDI를 전체 종료하고 `~/run_jandi.sh`로 다시 실행한다.
파라미터 값만 바꿀 때 `cp`나 재빌드는 필요 없다. 다른 장비에 전달할 때는 push/update한다.
update는 최신 소스 YAML을 가져온다. 로컬 미커밋 수정이 있으면 기존 Git 보호 규칙에 따라
중단하므로 Jetson에서 튜닝했다면 공유할 변경을 먼저 push한다. 설치된 Release 내부 YAML은
수정하지 않는다. 기존 `~/.config/jandi/main/vision_algorithm.yaml`과 fast 파일은
그대로 보존하지만 읽거나 새로 생성하지 않는다. 기존 홈에서만 바꾼 튜닝값은 자동 이관하지
않으므로 필요한 값이 있다면 확인 후 위 소스 YAML에 직접 반영한다.
시작 전에는 선택 버전의 C++ config_check로 소스 YAML의 범위·형식을 검사한다.
실패 시 카메라·모터 관리자를 시작하지 않으며 잘못된 YAML을 자동 덮어쓰지 않는다.
직접 ROS 노드를 실행하는 경우에는 해당 소스 파일을 VISION_CORE_ALGORITHM_CONFIG로 지정한다.
엔진 선택용 yolo26_runtime.yaml도 선택 버전의 소스 파일을 읽는다.

### 이전 홈 튜닝 방식에서 한 번 전환하기

홈 명령은 ~/.local/share/jandi-tools/jandi_deploy.py 복사본을 실행하므로
소스만 업데이트하는 것으로는 기존 배포 도구가 바뀌지 않는다. 이번 전환 때 한 번 갱신한다.
이 명령은 도구만 설치하고 빌드·노드를 실행하지 않는다.

PC에서 수정한 소스의 도구를 설치한 뒤 두 버전을 push한다:

```bash
/usr/bin/python3 ~/my_cv/src/vision/scripts/jandi_deploy.py install --machine pc
~/push_jandi_workspace.sh main
~/push_jandi_workspace.sh fast
```

PC push 완료 후 Jetson에서, JANDI와 진행 중인 업데이트를 종료한 상태로:

```bash
mkdir -p ~/jandi-tools
git -C ~/jandi fetch origin main
git -C ~/jandi archive origin/main src/vision/scripts/jandi_deploy.py | tar -x -C ~/jandi-tools
/usr/bin/python3 ~/jandi-tools/src/vision/scripts/jandi_deploy.py install --machine jetson
~/jandi_update.sh main
~/jandi_update.sh fast
```

이후 일반 알고리즘 값 변경은 기존 push/update 흐름만 사용한다. 배포 도구 코드 자체가
변경되는 경우에는 위와 같이 도구 복사본도 갱신해야 한다. 배포 도구를 PC에 설치하면
현재 정상 Release를 재빌드하지 않아도 시작 시 새 소스 YAML 선택 방식이 적용된다.

기본 가중치:

- `line.walk.ready_observation_end_weight: 1.0`
- `ball.control.ready_observation_end_weight: 1.0`
- `goal.control.ready_observation_end_weight: 1.0`
- `hurdle.control.ready_observation_end_weight: 1.0`

유효한 양수만 허용한다. `1`은 균등 평균, `3`은 기존 시작 1→끝 3 가중치다.
정규화된 가중치 합으로 나누며, O/H 게인·점수식·READY 타이밍은 변경하지 않는다.
BALL/GOAL 중심점은 u와 v 모두 같은 해당 미션 가중치를 쓴다.
BALL 최초 이동 진입 후보에도 BALL 값을 사용한다. 기존 후보 이력의 정규화 방식은 유지한다.
정지 관측은 기존 상태별 가중/균등 평균을 유지한다.

공 접근 중심은 `ball.control.approach_top_center_u_px`,
`approach_bottom_center_u_px` 두 값으로 지정한다. 초기에는 모두 기존 중심값 400이다.
폭 설정은 유지한다. 원거리·근거리·READY 분류·복구·디버그가 같은 보간 경계를 쓴다.
이전 YAML의 `far_u_des_px`는 단독으로 사용할 때 두 중심으로 변환한다.
이전 키와 새 키를 함께 지정하면 오류로 중단한다. 새 ROS 파라미터는
`ball_approach_top_center_u_px`, `ball_approach_bottom_center_u_px`다.
기존 단일 `ball_far_u_des_px` ROS 파라미터는 이 두 파라미터로 대체됐다.

BALL FINE 전진·후진·좌우 이동은 계획된 n회(최대 3회)를 모두 실행한다.
GOAL FINE의 n-1 정책과 이동량·픽업 다각형·목표 통과 방지는 유지한다.

결승선은 기존 진입 조건을 유지한다. 유효한 바닥 가이드가 있으면 일반 LINE
O/H 조향·큰 점수의 WALK 제자리 회전·DONE 후 관측을 사용한다.
점이 2개 이하이거나 가이드를 만들 수 없으면 직진한다. 이미 발행한 동작은 완료한다.
LINE 점수 회전 한도를 소진하면 결승선에서는 Reset 전까지 직진으로 전환한다.
카메라/전송/모터 오류 정지 조건은 해제하지 않는다. 일반 구간 FINAL HOLD는 유지한다.

GOAL 가장자리는 원거리 보행 시작~READY까지만 관측한다. READY에서 먼저 8중5를
판단한 뒤 다음 일반 보행의 예약 여부를 정한다. READY 이후~다음 실제 보행 시작 전
프레임은 제외한다. 예약 ACK는 실제 보행 시작으로 취급하지 않는다.
확정되면 발행된 동작의 실제 DONE을 기다리고 기존 방향 회전 2회를 연속 수행한다.
두 회전 사이에 별도 관측을 끼우지 않는다. 이후 새 정지 관측은 반드시 수행한다.
MAIN은 1초와 전체 창, FAST는 최소 8프레임 및 조건 충족 시 조기 완료하며
실패 판정은 기존 1초까지 기다린다. RGB 유실과 Depth 실패는 각각 기존 복구로 넘긴다.
보정 묶음의 추가 상한은 두지 않는다. 근거리 가장자리 판정도 8중5를 적용한다.
기존 Depth median/ROI 확대, ID 교체, 골대 근거리 진입, 투척 계산은 유지한다. 허들 v 투표 변경은 아래 항목을 따른다.

K는 확인 화면만 연다. 확인 화면의 1만 기존 긴급종료를 호출한다.
0/Esc는 원래 메뉴로 복귀하며 노드·미션 상태를 바꾸지 않는다. 반복 K는 종료하지 않는다.

## 처음 설치: Jetson (사용자가 직접 실행)

기존 `/home/rnd/jandi`와 기존 스크립트는 보존한다. 실제 Jetson 파일은 이 작업에서
읽거나 변경하지 않았다. 배포 묶음의 파일은 PC에서 제공하며 Jetson 설치는 별도다.

제공한 묶음을 Jetson에 복사한 경우:

```bash
python3 ~/jandi_tools_bundle/scripts/jandi_deploy.py install --machine jetson
```

GitHub에서 도구 파일만 가져오는 경우(기존 체크아웃을 전환하지 않음):

```bash
mkdir -p ~/jandi-tools
 git -C ~/jandi fetch origin main
 git -C ~/jandi archive origin/main src/vision/scripts/jandi_deploy.py | tar -x -C ~/jandi-tools
python3 ~/jandi-tools/src/vision/scripts/jandi_deploy.py install --machine jetson
```

설치기는 홈의 기존 명령 파일을 `~/.local/state/jandi/script-originals/<시각>/`에 보존한 뒤
호환 명령을 설치한다. 노드를 실행하거나 기존 설치본을 삭제하지 않는다.

`~/.config/jandi/deployment.json`에서 다음을 확인한다.

- `repository`: 기존 Git 저장소(기본 `~/jandi`). 다른 경로면 실제 경로로 수정.
- `dependency_prefixes`: 기존 설치의 **패키지별** RealSense/SDK prefix.
  예: `~/my_cv/install/dynamixel_sdk`, `~/realsense_ws/install/realsense2_camera`.
  `vision`·`forward_walk`가 함께 들어 있는 전체 기존 install을 지정하지 않는다.
  `/opt/ros/humble`에 설치된 의존성은 별도 항목이 필요 없다.
- `jobs`: 기본 2. 8GB Jetson에서 병렬도를 임의로 크게 늘리지 않는다.
- `library_paths`: 공통 CUDA 등의 추가 경로. 다른 MAIN/FAST 경로를 넣지 않는다.

```bash
~/jandi_update.sh main
~/jandi_update.sh fast
~/run_jandi.sh
```

업데이트는 소스 수신과 Release 빌드·설치를 수행하며 **노드를 실행하지 않는다**.
사용자가 `run_jandi.sh`를 실행해 메뉴에서 선택한 뒤에만 기존 런처가 노드를 시작한다.
MAIN/FAST 각각 Git worktree와 build/install을 사용하고, 기존 `~/jandi` 브랜치는 바꾸지 않는다.

```text
~/jandi_main/src/                    MAIN 소스 worktree
~/jandi_fast/src/                    FAST 소스 worktree
~/jandi_main/releases/<빌드ID>/      MAIN core/ros/build/log
~/jandi_fast/releases/<빌드ID>/      FAST core/ros/build/log
~/jandi_main/current.json            MAIN의 core+ROS 경로 한 묶음
~/jandi_fast/current.json            FAST의 core+ROS 경로 한 묶음
~/jandi_main/install                 성공한 MAIN ROS 설치본의 편의 링크
~/jandi_fast/install                 성공한 FAST ROS 설치본의 편의 링크
~/vision_core_main/install           성공한 MAIN core 편의 링크
~/vision_core_fast/install           성공한 FAST core 편의 링크
```

실행은 편의 링크 두 개를 조합하지 않고 **current.json 하나**를 읽는다.
새 최종 경로에 처음부터 설치하고, 전체 성공 후 current.json을 원자적으로 전환한다.
빌드 중간 실패는 이전 성공 설치본에 영향을 주지 않는다. 실패 로그는 새 release에 남는다.
같은 소스와 빌드 설정이며 필수 파일·설치 파일 해시가 정상인 경우 빌드를 생략한다.
설치 파일 누락·변조, 알고리즘 소스 YAML 변경, 빌드 설정 변경은 생략하지 않는다.

```bash
~/jandi_update.sh main --force
~/jandi_update.sh fast --force
python3 ~/.local/share/jandi-tools/jandi_deploy.py status
```

이전 설치본으로 복귀할 때는 전체 종료 후 이전 release의 `complete.json`을 확인하여
같은 버전의 `current.json`으로 복사한다. core와 ROS 경로를 따로 편집하지 않는다.

실행 중인 버전의 업데이트는 거부한다. 버전 변경은 기존 콘솔에서 전체 종료한 뒤 한다.
SSH 재연결 후 `run_jandi.sh`는 실행 중인 기존 콘솔에 연결하고 새 노드를 만들지 않는다.
명시한 다른 버전으로 시작하려 하면 거부한다. 관리자가 비정상 종료했어도 기록된
프로세스가 살아 있으면 새 실행·업데이트를 거부한다.

## 기존 명령 호환

| 기존 명령 | 새 역할 |
|---|---|
| `~/push_jandi_workspace.sh [main\|fast]` | PC 원본 → 해당 브랜치 |
| `~/sync_jandi_to_my_cv.sh [main\|fast]` | GitHub → 해당 PC 원본 → 독립 Release 설치 |
| `~/jetson_push_to_github.sh [main\|fast]` | Jetson 해당 worktree 변경 → 해당 브랜치 |
| `~/jetson_pull_from_github.sh [main\|fast]` | 해당 Jetson 버전 업데이트·독립 설치 |
| `bash ~/jandi_branch_switch.sh main` | 기본 대상 MAIN 선택; 소스/설치본을 덮어쓰지 않음 |
| `bash ~/jandi_branch_switch.sh fast` | 기본 대상 FAST 선택; 소스/설치본을 덮어쓰지 않음 |
| `~/run_jandi.sh` 또는 `run_jandi` | 버전·OpenCV/터미널/HTTP 옵션 선택 |

버전을 생략한 업로드/업데이트는 선택된 기본 버전을 사용한다. 명시하는 편이 분명하다.
예전 `JANDI_BRANCH=main` 또는 `JANDI_BRANCH=race-round2-fast-20261109`도 지원한다.
그 외 브랜치는 거부하며 force push를 하지 않는다.
PC/GitHub가 같은 파일을 다르게 고친 경우 자동 덮어쓰기 없이 중단한다.
Jetson 미커밋 변경과 원격 변경이 함께 있으면 원본을 보존하고 중단한다.

## PC 원본

PC MAIN 원본은 `~/my_cv/src/{vision,forward_walk,camera_overlay}`와 `~/vision_core`다.
PC FAST 원본은 `~/my_cv/variants/fast/src/{vision,forward_walk,camera_overlay}`와
`~/vision_core_fast`다. `variants/COLCON_IGNORE`로 기본 워크스페이스와 분리한다.
Git 업로드 staging은 각각 `~/jandi_workspace`, `~/jandi_workspace_fast`다.
staging에서 기능을 먼저 구현하지 않는다.

PC 초기 명령 설치:

```bash
python3 ~/my_cv/src/vision/scripts/jandi_deploy.py install --machine pc
```

PC도 Release 실행본은 `~/jandi_main`, `~/jandi_fast`의 독립 release를 사용한다.
기존 `~/my_cv/install`, `~/vision_core/install`은 삭제·교체하지 않는다.

## 엔진 경로: yolo26_runtime.yaml 한 곳에서 관리

`deployment.json`에는 engine을 지정하지 않는다. 설치 시 예전 engine 키를 제거하고
실행 시에도 그 값이나 물려받은 YOLO26_ENGINE_PATH를 사용하지 않는다.
JANDI는 선택 버전의 아래 **소스 runtime 파일**을 직접 읽는다.

| 장치 | MAIN | FAST |
|---|---|---|
| PC | `~/my_cv/src/vision/config/yolo26_runtime.yaml` | `~/my_cv/variants/fast/src/vision/config/yolo26_runtime.yaml` |
| Jetson | `~/jandi_main/src/vision/config/yolo26_runtime.yaml` | `~/jandi_fast/src/vision/config/yolo26_runtime.yaml` |

PC는 `pc_engine_path`, Jetson(aarch64/arm64)은 `jetson_engine_path`를 읽는다.
엔진 파일을 준비하고 해당 YAML 경로를 수정한 뒤 기존 JANDI를 **전체 종료 후 다시 실행**한다.
엔진 경로만 바꾸면 재빌드는 필요 없다. 실행 전 같은 runtime loader로 파일 존재를 확인하며
없거나 설정이 잘못되면 카메라/모터 관리자를 시작하지 않는다.
소스 파일을 수정하는 것이므로 GitHub 업데이트의 로컬 변경 보호 정책은 그대로 적용된다.
설치된 release 내부 YAML을 직접 수정하지 않는다.

## RealSense 설정: 기존 rs_launch.py를 직접 수정

JANDI의 카메라 명령은 아래와 동일하다. 추가 launch/YAML override는 전달하지 않는다.

```bash
ros2 launch realsense2_camera rs_launch.py
```

Jetson의 실제 소스 경로는 사용자가 확인한 다음 파일이다.
`~/realsense_ws/src/realsense-ros/realsense2_camera/launch/rs_launch.py`
PC 소스는 `~/my_cv/src/realsense-ros/realsense2_camera/launch/rs_launch.py`다.
VS Code에서 기존 RGB 설정이 있는 `parameters=[params, params_from_file, {...}]`의
마지막 딕셔너리에 다음을 넣고 직접 값을 수정한다.

```python
# Depth exposure overrides
'depth_module.enable_auto_exposure': False,
'depth_module.exposure': 5000,
'depth_module.gain': 16,
```

마지막 딕셔너리가 앞의 런치 인자와 config_file 값보다 우선한다.
기존 RGB 설정과 다른 카메라 설정은 유지한다. PC와 확인된 Jetson 모두 소스 링크 설치이므로
이 파일 저장 후 카메라를 재시작하면 반영되며 재빌드는 필요 없다.
설치기는 RealSense 런치파일을 자동 교체하지 않는다. Jetson ZIP의 reference/rs_launch.py는
사용자가 보내준 파일에 위 세 항목만 추가한 참고본이다.
이전 별도 `~/.config/jandi/realsense.launch.py`, 카메라 YAML과 환경변수 override는 적용하지 않는다.
이전 사용자 파일은 보존하고, 도구가 만들었던 수정되지 않은 camera/depth 홈 wrapper는 원본 보존 폴더로 옮긴다.

## 일반 ROS 노드로 Depth 뷰어 실행

카메라는 위 기본 RealSense 명령으로 실행한다. 별도 터미널에서 현재 ROS 환경과
사용할 camera_overlay 설치본을 불러온 뒤 아래 노드만 실행한다.

```bash
# Jetson 뷰어 터미널: MAIN 설치를 쓰는 예시
source /opt/ros/humble/setup.bash
source ~/realsense_ws/install/setup.bash
source ~/jandi_main/install/local_setup.bash
export ROS_DOMAIN_ID=22
ros2 run camera_overlay depth_color_viewer
```

FAST 설치는 마지막 source를 `~/jandi_fast/install/local_setup.bash`로 바꾸면 된다.
PC에서도 기존 ROS 환경에 선택한 `~/jandi_main/install/local_setup.bash`를 불러오면
같은 ros2 run 명령을 사용한다. 이 노드는 Depth 토픽을 구독하며 카메라·모터를 시작하지 않는다.
기본은 HTTP ON / 로컬 OpenCV OFF / 포트8081이다.
Jetson HTTP는 `http://192.168.0.133:8081`, PC 로컬 실행은 `http://localhost:8081`로 접속한다.
자동노출 ON/OFF, 수동 exposure/gain, 클릭 Z와 21×21 ROI 유효 비율을 제공한다.
수동 변경은 AE를 끈 뒤 Apply manual values로 보낸다. 카메라 descriptor 범위와 step을 확인한다.
실시간 변경은 실행 중 카메라에만 적용하며 시작값은 기존 rs_launch.py에서 직접 수정한다.
동작 중인 JANDI 카메라가 있으면 뷰어만 실행한다. 뷰어 종료는 카메라를 종료하지 않는다.
추가 토픽·거리·표시 옵션은 `camera_overlay/DEPTH_COLOR_VIEWER.md`를 참조한다.

첫 JANDI 메뉴는 1=MAIN/FAST, 3/4/5=OpenCV/터미널/HTTP다. PRACTICE/IRC 구분은 없다.
업데이트는 Release 빌드·설치를 수행하며 자동 테스트나 로봇 실행은 하지 않는다.

## 검증과 미검증

빌드와 테스트는 PC에서 하드웨어 없이 수행한다. 코어 결정론/READY/경계/복구 테스트,
ROS 합성 peer와 fake Dynamixel 실행기 검사, 런처 입력·배포 실패 검사를 구분한다.
Release 설치 성공 기록은 실기 성공이나 전체 테스트 통과 기록을 의미하지 않는다.
Jetson ARM64 빌드와 실제 카메라·모터·주행 성능은 별도 확인 대상이다.

## 허들 DOWN 이후 v 판정

평균 v 대신 원본 프레임의 임계값 충족 횟수로 판단한다. MAIN 관측창은 10프레임,
FAST는 8프레임이다. `hurdle.control`의 `down_v_min_hits: 3`은
`v >= fine_mode_v_px`(현재 220) 전환 기준, `contact_v_min_hits: 3`은
`v >= contact_v_px`(현재 220) 접촉 확정 기준이다.
`down_two_step_min_hits`는 MAIN 7 / FAST 6이며, `v < down_one_step_v_px`(현재 40)가
그 횟수를 충족할 때만 2걸음, 나머지는 1걸음 후 기존 새 정지 관측을 수행한다.
가까움 투표가 3회 충족되면 FINE 전환을 우선한다. 미검출·비유한 v는 표로 세지 않지만
전체 프레임 수에는 포함한다. 완전한 관측창이 필요하며 평균 좌표로 가상 투표를 만들지 않는다.

접촉은 FINE 자세 진입 후 투표가 확정되면 이후 낮은 v/유실로 취소하지 않는다.
실제 진행 중 동작의 DONE과 라인 정렬/명령 허용 조건을 기다린 뒤 접촉 보행을 시작한다.
전체 Reset, 대상 무효화, v 임계값/투표 설정 변경은 이전 증거를 초기화한다.
FAST 최초 DOWN 탐색의 6중3 검출과 이후 8중3 검출은 유지하며,
거리 결정은 기존 정책대로 새 정지 관측의 8프레임 투표로 수행한다.
허들 READY 가중치는 1이며, 원거리 바닥 라인 O/H와 DOWN 직전 추가 보행에도 적용한다.
