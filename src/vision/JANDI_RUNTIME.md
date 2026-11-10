> MAIN/FAST 독립 설치·버전 선택·공통 카메라 설정은 [JANDI_DUAL_INSTALL.md](JANDI_DUAL_INSTALL.md)를 우선 적용합니다. 아래 단일 설치 등록 절차는 이전 운영 방식입니다. 긴급종료는 K → 확인 화면 → 1로 실행합니다.

# Jetson 통합 실행과 숫자 메뉴

Jetson Orin Nano에서만 카메라·추론·미션 판단·모터 실행을 함께 구동한다.
미션 알고리즘, JSON 모션, 모터 실행 로직은 변경하지 않는다.

## 환경과 설치

기존 코드의 Jetson 엔진 경로는 `config/yolo26_runtime.yaml`의
`/home/rnd/jandi/src/dataset/best_1029.engine`이다. 이 경로에서 기본 워크스페이스
`/home/rnd/jandi`를 구한다. 코어 설치는 기존 `VISION_CORE_ROOT`, 없으면
`~/vision_core/install`이다. 운영 YAML은 기존 `VISION_CORE_ALGORITHM_CONFIG`,
없으면 코어 설치의 `share/shared_vision_core/config/vision_algorithm.yaml`을 사용한다.
설정값이나 YAML·모션을 자동 복사/튜닝하지 않는다.
기존 `YOLO26_ENGINE_PATH` 환경변수가 있으면 노드의 기존 규칙대로 엔진 선택에 우선 적용한다.

먼저 기존 프로젝트 절차로 코어 Debug 검사와 Release 설치를 완료한다.
이 변경은 코어 재컴파일을 요구하지 않지만 설치된 코어와 운영 YAML은 필요하다.
Jetson에 RealSense ROS 드라이버와 기존 모델·Dynamixel SDK·udev 설정이 있어야 한다.

```bash
source /opt/ros/humble/setup.bash
source /home/rnd/jandi/install/setup.bash
cd /home/rnd/jandi
# Debug 검증은 기존 운영 Release와 분리한다.
(
  colcon build --build-base build_jandi_debug --install-base install_jandi_debug \
    --packages-select vision forward_walk --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON \
    -DSHARED_VISION_CORE_ROOT="$HOME/vision_core/install" &&
  source install_jandi_debug/setup.bash &&
  colcon test --build-base build_jandi_debug --install-base install_jandi_debug \
    --packages-select vision forward_walk --return-code-on-test-failure \
    --event-handlers console_direct+ &&
  colcon test-result --test-result-base build_jandi_debug --verbose
) && colcon build --packages-select vision forward_walk --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DSHARED_VISION_CORE_ROOT="$HOME/vision_core/install" &&
source install/setup.bash &&

# 숫자 메뉴 명령 등록: 파일 생성만 하며 노드를 시작하지 않는다.
ros2 run vision install_jandi_command.sh /home/rnd/jandi "$HOME/vision_core/install" &&
export PATH="$HOME/.local/bin:$PATH"
```

`~/.local/bin`이 새 SSH 창의 PATH에도 들어 있도록 설정한다. 엔트리포인트는
ROS와 위 워크스페이스 환경을 로드하고 Domain 22를 설정한다.
등록되는 명령은 `run_jandi`이며 기존 Jetson 환경 함수 `jandi()`와 함께 사용할 수 있다.
기존 함수나 이전에 등록된 `~/.local/bin/jandi`는 덮어쓰거나 삭제하지 않는다.
다른 경로를 이미 사용한다면 설치 명령의 경로를 그 실제 경로로 지정한다.
`vision`에 런치·스크립트·문서를 모두 설치하므로 기존 세 패키지 동기화 대상에 포함된다.
`forward_walk`는 `vision`에 의존하므로 `vision`에 역방향 패키지 의존성은 추가하지 않는다.

## 첫 번째 PowerShell SSH 창

```bash
run_jandi
```

처음 `run_jandi`를 실행하면 노드 시작 전에 옵션을 선택한다.
1번 OpenCV 디버그 화면 기본 OFF, 2번 터미널 디버그 기본 OFF,
3번 HTTP 영상 스트리밍 기본 ON이다. 번호로 ON/OFF를 바꾸고 Enter로 확정한다.
Esc/Ctrl+C/SSH 단절로 선택을 취소하면 노드를 시작하지 않는다.
확정 후 `vision_params.yaml`을 읽고 선택한 세 값을 명시적으로 적용한다.
rosbag과 내부 실행 모사는 끈다.
추론 FPS·미션 주기·알고리즘 값은 변경하지 않는다. RQT·튜닝·ASCII 뷰어는 자동 실행하지 않는다.

첫 실행의 시간표는 **0초 RealSense → 5초 main_node → 10초 line_perception_node →
15초부터 수동 서비스 요청 가능**이다. 특정 노드가 실패해도 나머지 시간표를 계속한다.
비전 노드를 수동 재시작하면 그 노드 실행부터 다시 5초 후 서비스 요청이 가능하다.
미션 START는 자동 발송하지 않는다. 단, 기존 모터 실행기는 켜질 때 초기 WALK 자세로 이동한다.

```text
1. RealSense   2. 모터   3. 비전
4. START      5. PAUSE  6. RESUME  7. RESET
8. 오류·로그  9. 콘솔 나가기(실행 유지)  0. 전체 종료
```

1~3의 하위 메뉴는 `1 켜기 / 2 끄기 / 3 재시작 / 0 뒤로`다.
메뉴 이동은 숫자 한 번으로 한다. **서비스 4~7은 선택 후 Enter에서 딱 한 번만 전송**한다.
Enter 전에는 Esc/0으로 취소할 수 있고, 그 사이 콘솔/SSH가 끊겨도 서비스는 전송하지 않는다.
전송 후 응답이 불명확해도 자동 재전송하지 않는다. 상태는 0.5초마다 갱신한다.
`RUNNING`은 프로세스 존재, `ROS READY`는 ROS endpoint/서비스 및 초기화 확인이다.
RGB·aligned depth publisher와 CameraInfo 수신 경과도 관찰하지만 RGB/depth 이미지를
추가 구독하거나 추가 YOLO를 돌리지 않는다. READY는 보행·픽업 성공의 검증이 아니다.

로그는 `/tmp/jandi-UID/{camera,motor,vision,manager}.log`에 보관한다.
ERROR/FATAL/WARN과 비정상 종료는 화면에 알리며, **그 어떤 로그도 PAUSE·RESET·중지·재시작을
발생시키지 않는다.** 프로세스 실패 시 다른 프로세스를 그대로 유지한다.

관리자는 SSH와 분리된 세션에서 실행된다. SSH 단절, 콘솔 9번, 콘솔 Ctrl+C는
미션·노드 상태를 바꾸지 않는다. 다시 `run_jandi`로 동일 세션에 연결한다.
실행 중 재접속에서는 옵션 선택을 건너뛰며 기존 설정/노드/FSM을 변경하지 않는다.
명시적으로 다른 설정의 백그라운드 관리자를 시작하려 하면 중복 실행하지 않고 안내한다.
`run_jandi attach`는 현재 세션에만 연결한다.

## 두 번째 PowerShell SSH 창: Practice ASCII 화면

```bash
source /opt/ros/humble/setup.bash
source /home/rnd/jandi/install/setup.bash
export ROS_DOMAIN_ID=22
ros2 run vision vision_terminal_node
```

설치 실행파일 이름은 `.py`가 없는 `vision_terminal_node`다.
Windows에 ROS는 필요 없다. 시작 옵션에서 터미널 디버그를 켠 경우에만 메타데이터를 발행한다.

## 서비스와 종료

기존 C++는 `start`, `pause`, `resume`, `reset`을 상대 서비스명으로 생성한다.
통합 실행에서만 명시적 리매핑해 문서와 일치시킨다:

```text
/line_perception_node/start   std_srvs/srv/Trigger
/line_perception_node/pause   std_srvs/srv/Trigger
/line_perception_node/resume  std_srvs/srv/Trigger
/line_perception_node/reset   std_srvs/srv/Trigger
```

콘솔 4~7을 선택하고 Enter로 확정했을 때에만 한 번 호출하며, 응답 실패·시간 초과 시 재전송하지 않는다.
RESET은 기존대로 발 자세와 카메라를 실제 이동시킨다.

카메라·비전은 사용자가 개별 종료/재시작할 수 있으며 모터에 중지 명령을 보내지 않는다.
이미 모터에 접수된 동작은 기존 실행기에서 계속 완료된다.
몸·카메라 DONE 대기 중이거나 비전이 새 명령을 내릴 수 있는 상태에서는
사용자의 모터 종료·재시작과 전체 종료를 보류한다. 필요하면 사용자가
**5 PAUSE → PAUSED 및 DONE 완료 확인 → 노드 종료/재시작** 순서로 조작한다.
자동 PAUSE는 없다. 정지 요청은 SIGINT만 사용하며 TERM/KILL로 강제 승격하지 않는다.
응답하지 않는 프로세스는 종료 대기 상태로 표시한다.

전체 종료는 `0 → 로봇 지지 확인 → 1`이다. 비전 → 모터 → 카메라 순으로 종료한다.
**모터 실행기의 정상 종료는 Torque OFF**, 재실행은 초기 자세 이동을 포함한다.
비전만 재시작하면 기존 FSM·명령 ID가 초기화되며 미션을 이어 복구하는 기능은 없다.
동작 이력이 있는 세션을 새로 시작하려면 사용자가 PAUSE 완료 후 비전을 끄고,
모터를 재시작하고, 비전을 다시 켜서 실행기 ID 이력도 함께 새로 시작한다.
다른 노드를 자동으로 묶어 재시작하지 않는다.

## ros2 launch 직접 사용

```bash
export ROS_DOMAIN_ID=22
ros2 launch vision jandi.launch.py \
  show_debug_view:=false terminal_node:=false stream_debug_view:=true
```

같은 분리형 관리자를 시작하는 런치다. 관리자 준비 후 런치 명령은 반환되며 노드는
계속 실행된다. 숫자 메뉴는 `run_jandi` 또는 `ros2 run vision run_jandi attach`로 연다.
launch 프로세스나 SSH의 수명으로 모터가 종료되지 않게 하기 위한 구성이다.
필요하면 `workspace:=... core_prefix:=...`로 기존 설치 위치를 지정한다.
RealSense는 **현재 설치된 `realsense2_camera/rs_launch.py`를 그대로 사용**한다.
640×480/15Hz, RGB8, depth 정렬·동기화 및 노출값은 기존 커스텀 카메라 런치 소유다.
오래된 `robot_bringup.launch.py`나 드라이버 기본값의 복제본을 사용하지 않는다.

## 하드웨어 없는 검사

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 src/vision/tests/jandi_runtime_test.py
/usr/bin/python3 src/vision/tests/jandi_launch_test.py
ros2 launch vision jandi.launch.py --show-args
```

검사는 Python 가짜 프로세스와 가짜 ROS 관찰자로 실행 순서, 서비스 시간표,
명시적 조작, 오류·단절 시 무제어, 개별 재시작, 중복 방지, 설치/런치 연결을 검증한다.
PTY에서 숫자 메뉴와 콘솔 분리도 검증한다. 서비스 Enter 확인, 옵션 전달,
로그 필터/긴 메시지/예외 스택/페이지 이동은 `jandi_options_logs_test.py`로 검증한다. `vision_jandi_ros_test`는
격리된 Domain 86의 합성 ROS 노드로 구독·서비스·graph 연결을 검증한다.
실제 Jetson에서 카메라 드라이버/USB, TensorRT 엔진, 모터 도착과 Torque OFF,
SSH 네트워크 단절은 실기 검증이 필요하다.

## 로그 메뉴

8번에서 다음 화면으로 이동한다. 0번은 로그 보기 → 로그 선택 → 메인 순서로 돌아간다.

| 번호 | 내용 |
|---|---|
| 1 | 카메라·모터·비전·관리자의 모든 등급 로그 |
| 2 | 모든 노드·관리자의 ERROR/FATAL 및 예외 스택 |
| 3 | 카메라 오류·경고 |
| 4 | 모터 오류·경고 |
| 5 | 비전 오류·경고 |
| 6 | 카메라 모든 로그 |
| 7 | 모터 모든 로그 |
| 8 | 비전 모든 로그 |

보기 화면에서 p는 이전 페이지, n은 다음 페이지, f는 실시간 끝으로 이동한다.
자식 프로세스의 원본 stdout/stderr 파일은 그대로 유지한다. `all.jsonl`에는 노드와
관리자의 로그를 등급/시간과 함께 보존한다. 오류 뒤의 태그 없는 예외 문맥은 다음
명시적 로그 등급까지 오류로 분류한다. 화면 페이지 수로 저장 로그를 제한하지 않는다.
콘솔은 오프셋만 인덱싱하고 매 갱신마다 파일 전체를 다시 읽지 않는다.
로그 조회는 서비스·정지·재시작 요청을 발생시키지 않는다.

## HTTP 영상과 기존 명령 호환성

시작 옵션에서 HTTP를 켜면 `http://JETSON_IP:8080`으로 최종 디버깅 영상을 본다.
JPEG 품질 40, 최대 15FPS, 브라우저 미접속 시 인코딩하지 않는다.
브라우저 종료/영상 연결 단절은 미션 상태를 변경하지 않는다.
`line_perception_node`의 HTTP 화면은 읽기 전용이며 보정 도구의 클릭 입력을 받지 않는다.

예전 `practice`/`IRC` 인자는 호환용으로 남긴다. 새 일반 사용은 인자 없는
`run_jandi`와 시작 옵션 선택이다. 비대화형 실행은 `--background custom`과
`--show-debug-view true/false`, `--terminal-node true/false`,
`--stream-debug-view true/false`를 지정한다. `ros2 launch`는 대화형 선택 대신
동일 이름의 세 launch 인자를 사용하며 기본값은 OFF/OFF/ON이다.
