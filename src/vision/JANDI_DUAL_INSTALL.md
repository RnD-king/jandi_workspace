# MAIN / FAST 독립 설치와 최종 제어 변경

두 브랜치는 각각 `main`, `race-round2-fast-20261109`를 사용한다.
`backup/pre-final-main-20261108`은 fetch/push/수정 대상이 아니다.
실제 로봇·모터·카메라는 이번 구현·자동 검증에서 실행하지 않는다.

## 알고리즘 설정

`vision_core/config/vision_algorithm.yaml`에서 설정한다.

- `line.walk.ready_observation_end_weight: 1.0`
- `ball.control.ready_observation_end_weight: 1.0`
- `goal.control.ready_observation_end_weight: 1.0`
- `hurdle.control.ready_observation_end_weight: 3.0`

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
기존 Depth median/ROI 확대, ID 교체, 근거리 진입, 투척 계산, HURDLE 로직은 유지한다.

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
 git -C ~/jandi archive origin/main src/vision/scripts/jandi_deploy.py src/vision/launch/realsense.launch.py | tar -x -C ~/jandi-tools
python3 ~/jandi-tools/src/vision/scripts/jandi_deploy.py install --machine jetson
```

설치기는 홈의 기존 명령 파일을 `~/.local/state/jandi/script-originals/<시각>/`에 보존한 뒤
호환 명령을 설치한다. 노드를 실행하거나 기존 설치본을 삭제하지 않는다.

`~/.config/jandi/deployment.json`에서 다음을 확인한다.

- `repository`: 기존 Git 저장소(기본 `~/jandi`). 다른 경로면 실제 경로로 수정.
- `dependency_prefixes`: 기존 설치의 **패키지별** RealSense/SDK prefix.
  예: `~/my_cv/install/dynamixel_sdk`, `~/jandi/install/realsense2_camera`.
  `vision`·`forward_walk`가 함께 들어 있는 전체 기존 install을 지정하지 않는다.
  `/opt/ros/humble`에 설치된 의존성은 별도 항목이 필요 없다.
- `engine`: 기존 엔진의 절대 경로. 기본은 `~/jandi/src/dataset/best_1029.engine`.
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

## 공통 RealSense 런치와 실시간 노출 실험

MAIN/FAST 모두 `~/.config/jandi/realsense.launch.py` 하나를 사용한다.
이 파일 상단의 세 항목을 직접 수정한다. YAML은 사용하지 않는다.

```python
DEPTH_AUTO_EXPOSURE = False
DEPTH_EXPOSURE = 5000
DEPTH_GAIN = 16
```

이 파일은 드라이버의 기존 `rs_launch.py`를 포함하면서 위 세 값을 런치 인자로 전달한다.
기존 RGB·해상도/FPS·alignment·IMU·토픽 설정은 원래 드라이버 런치를 따른다.
공유 파일은 빌드 설치물 바깥에 있어 재빌드 없이 카메라 재시작에 반영된다.
설치기를 다시 실행해도 사용자가 수정한 이 파일은 덮어쓰지 않는다.
이전 `realsense_depth.yaml`은 파일만 보존하고 읽거나 적용하지 않는다.

일반 실행:

```bash
~/run_jandi.sh
```

첫 메뉴의 1은 버전, 3/4/5는 OpenCV/터미널/HTTP다. 번호 입력 후 Enter로 변경하며
빈 입력 Enter로 시작한다. PRACTICE/IRC 구분과 mode 파라미터는 제거했다.
CLI에서 메뉴를 생략할 때는 `~/run_jandi.sh main --no-menu --terminal --opencv`처럼
버전과 옵션을 직접 지정한다. 기본은 OpenCV OFF / 터미널 OFF / HTTP ON이다.

카메라 노출만 실험하려면 전체 JANDI 대신 다음을 사용한다.

```bash
# 터미널 1: 카메라만 실행. 모터와 비전은 실행하지 않음.
~/jandi_camera.sh

# 터미널 2: 이미 발행 중인 Depth를 구독하고 HTTP 뷰어 실행.
~/jandi_depth.sh main
# FAST 설치의 뷰어를 쓸 때는 ~/jandi_depth.sh fast
```

PC 브라우저에서 `http://192.168.0.133:8081`에 접속한다(Jetson에서 실행한 경우).
고정 거리 색상, 클릭 Z, 21×21 ROI median/유효 비율, AE ON/OFF와 수동 exposure/gain을 제공한다.
AE를 끈 뒤 수동값을 적용한다. 카메라가 보고한 범위와 step을 검증하며 성공/거부 및
현재 ROS 파라미터를 다시 표시한다. 뷰어 시작만으로 카메라 값을 변경하지 않는다.
브라우저 변경은 실행 중 카메라에만 적용된다. 다음 시작에도 유지하려면 위 런치파일을 수정한다.

일반 JANDI가 이미 카메라를 실행 중이면 카메라를 하나 더 실행하지 않고 뷰어만 실행한다.
`jandi_camera.sh`는 JANDI 관리자가 실행 중일 때 거부한다.
뷰어는 Ctrl+C로 종료한다. 뷰어 종료는 카메라를 종료하거나 기존 노출을 되돌리지 않는다.
HTTP 뷰어의 일반 포트는 8081로 기존 비전 8080과 분리한다.
직접 ROS 실행/거리 범위/토픽 선택은 `camera_overlay/DEPTH_COLOR_VIEWER.md`를 참조한다.

카메라를 사용자가 실행한 뒤 적용값을 확인할 수 있다.

```bash
export ROS_DOMAIN_ID=22
ros2 param get /camera/camera depth_module.enable_auto_exposure
ros2 param get /camera/camera depth_module.exposure
ros2 param get /camera/camera depth_module.gain
```

업데이트 명령은 Release 빌드와 설치를 수행하며 자동 테스트는 실행하지 않는다.
테스트는 수정 과정에서 별도로 수행한다. fake 카메라/모터 검사는 실제 장치 동작 검증을 대신하지 않는다.

## 검증과 미검증

빌드와 테스트는 PC에서 하드웨어 없이 수행한다. 코어 결정론/READY/경계/복구 테스트,
ROS 합성 peer와 fake Dynamixel 실행기 검사, 런처 입력·배포 실패 검사를 구분한다.
Release 설치 성공 기록은 실기 성공이나 전체 테스트 통과 기록을 의미하지 않는다.
Jetson ARM64 빌드와 실제 카메라·모터·주행 성능은 별도 확인 대상이다.
