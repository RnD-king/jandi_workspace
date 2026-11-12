# Jetson 보정 도구의 HTTP 화면과 조작

OpenCV 창 없이 브라우저에서 기존 카메라 디버그 화면을 보고 클릭/키보드로 조작한다.
최대 15FPS, JPEG 품질 40, 기본 포트 8080이다. RealSense는 별도로 실행한다.

```bash
ros2 run camera_overlay motion_calibration_node
```

노트북 Windows/Ubuntu 브라우저에서 `http://JETSON_IP:8080`에 접속한다.
모션 보정은 RealSense + `forward_walk/main_node` + 이 보정 노드로 사용하며,
동시에 `line_perception_node`를 실행하지 않는다. 모션 보정 엔진의 기존 Jetson 경로
`/home/rnd/jandi/src/dataset/best_1029.engine`에 해당 Jetson용 엔진이 있어야 한다.

아래 실행파일도 같은 두 파라미터를 지원한다.

```bash
ros2 run camera_overlay camera_overlay_node
ros2 run camera_overlay click_depth_node
ros2 run camera_overlay ball_pixel_pin_node
ros2 run camera_overlay forward_yolo_node
ros2 run camera_overlay goal_yolo_node
```

모든 도구의 기본 포트는 8080이다. 같은 장비에서는 8080을 사용하는 도구를 하나씩 실행한다.
PC·Jetson 모두 기본 `stream_debug_view=true`, `show_debug_view=false`다.
로컬 창이 필요하면 `-p show_debug_view:=true`, HTTP를 끄려면
`-p stream_debug_view:=false`를 지정한다.
DISPLAY/Wayland가 없는 SSH 환경에서는 로컬 창을 만들지 않는다.
실시간 YOLO 전용 두 노드의 카메라 동작·깊이 밴드는 [YOLO_INSPECTION.md](YOLO_INSPECTION.md)를 참고한다.
기존 launch XML도 노드의 HTTP 기본 ON 설정을 따른다. 개별 옵션 지정은 위 `ros2 run`에 파라미터를 추가한다.

## 송신 화면

- 사각형/공 픽셀 핀: 기존 카메라 디버그 화면.
- 클릭 깊이: 클릭 측정값을 표시하는 카메라 화면만. 별도 GOAL X/Z 창은 송신하지 않는다.
- 모션 보정: 모드 선택, 640×480 카메라 영상, BEFORE/AFTER 및 조작 버튼을 포함한
  기존 1040×670 주화면. 좌표/버튼 배치를 바꾸지 않는다.
- JPEG는 보기 용도로만 사용한다. 인식·측정은 원본 RGB/Depth로 처리한다.

## 브라우저 조작

영상 클릭 위치는 브라우저 표시 크기에서 원본 화면 좌표로 변환한다.
왼쪽/오른쪽 클릭 및 기존 노드가 처리하는 ASCII 키 입력을 전달한다.
공 픽셀 핀은 I 좌표 입력, 숫자/쉼표/Backspace/Enter, P 다각형, Ctrl+C 실시간 복귀를 지원한다.
클릭 깊이는 Space/C로 실시간 복귀한다. 모션 보정은 기존 화면의 버튼을 사용한다.
브라우저 아래 Space, Ctrl+C, Enter, Esc 버튼도 제공한다.

각 JPEG는 원본 화면 ID를 가진다. 클릭/키 입력도 보고 있던 화면 ID를 보낸다.
최근 최대 32개 화면의 측정 문맥을 보관하며, 만료된 화면의 입력은 버리고 경고한다.
클릭 깊이는 그 화면에 대응하는 RGB·Depth·CameraInfo를 사용한다. 최신 Depth를 대신 쓰지 않는다.
모드/측정 상태가 바뀐 뒤 들어온 이전 화면의 입력은 실행하지 않는다.
모션 보정 BEFORE/AFTER는 표시 화면 시점의 기존 10프레임 관측 평균을 사용하며,
실제 DONE 대기와 다음 측정을 위한 새 프레임 수집 조건은 유지한다.

입력은 네트워크 스레드에서 제한된 큐에만 넣고 노드 스레드에서 기존 처리 함수를 호출한다.
JPEG 인코딩/네트워크 송신은 별도 작업 스레드다. 느린 연결은 제한 시간 뒤 끊고,
최신 영상만 유지한다. 동일한 고정 화면은 반복 인코딩하지 않는다. 고정 화면 재접속은
최근 JPEG를 공유한다. 연결이 끊긴 입력은 자동 재전송하지 않는다.

## 빌드 및 검사

먼저 기존 절차로 `vision_core`를 설치한 뒤:

```bash
source /opt/ros/humble/setup.bash
source ~/my_cv/install/setup.bash
cd ~/my_cv
colcon build --packages-select vision camera_overlay --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DSHARED_VISION_CORE_ROOT="$HOME/vision_core/install"
source install/setup.bash
ctest --test-dir build/vision -R 'debug_mjpeg|jandi' --output-on-failure
ctest --test-dir build/camera_overlay --output-on-failure
```

HTTP 결합 검사는 격리된 ROS Domain 89의 합성 RGB-D만 사용한다. 카메라/모터/YOLO를
실행하지 않고 headless 화면 송신, 이전 화면의 Depth 측정, 고정 화면 인코딩 억제,
브라우저 연결 해제/재접속 및 노드 종료를 확인한다. Jetson 실기 성능·보행은 별도 확인 대상이다.
