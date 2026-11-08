# Camera overlay and click depth measurement

기존 `sensor_msgs/msg/Image` 토픽을 구독하는 독립 C++ ROS 2 노드.
카메라 드라이버를 실행하거나 장치를 직접 열지 않는다.
사각형·클릭 깊이 측정은 YOLO를 실행하지 않는다. 공 픽셀 핀 및 모션 보정 노드는
`vision`의 TensorRT 엔진을 사용하고, 패키지 빌드에는 `vision`, CUDA/TensorRT 및
별도 설치된 `shared_vision_core`가 필요하다. C++ 구현이므로 Python NumPy에 의존하지 않는다.
HTTP 조작은 [HTTP_DEBUG_STREAMING.md](HTTP_DEBUG_STREAMING.md)를 참고한다.

## 빌드

업데이트된 `vision`을 먼저 설치한 뒤 `camera_overlay`를 빌드한다. 기존 캐시의 외부 코어 경로를
제거하도록 최초 업데이트 시 `--cmake-clean-cache`를 사용한다.

```bash
source /opt/ros/humble/setup.bash
cd /home/noh/my_cv
colcon build --packages-select camera_overlay --symlink-install --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source /home/noh/my_cv/install/setup.bash
```

## 실행

RealSense는 따로 실행해서 **640×480 컬러 이미지**를 발행한다.
화면을 볼 PC에서 다음 명령을 실행한다.

```bash
ros2 launch camera_overlay overlay.launch.xml
```

토픽이 다르면 다음처럼 지정한다.

```bash
ros2 launch camera_overlay overlay.launch.xml image_topic:=/cam1/color/image_raw
```

launch 없이 노드만 실행할 수도 있다.

```bash
ros2 run camera_overlay camera_overlay_node --ros-args \
  -p image_topic:=/camera/camera/color/image_raw \
  -p box_width_px:=160 -p box_height_px:=120 -p box_alpha:=0.35
```

## 조작

- 영상에서 **왼쪽 클릭**: 사각형 중심 변경. 초기 중심은 `(320,240)`.
- 반투명 빨간 채움과 빨간 테두리, 중심 십자, 가로·세로 픽셀 수 표시.
- **Esc / q / 터미널 Ctrl+C**: 노드 종료. 창 닫기 버튼은 종료 판단에 사용하지 않는다.
- 해상도가 640×480이 아닌 프레임은 경고 후 무시하며 크기를 변환하지 않는다.
- 클릭한 중심은 유지한다. 사각형이 영상 밖으로 나가면 보이는 부분만 그리고
  `CLIPPED`를 표시한다. 표시 크기는 픽셀 단위이며 실제 물체의 cm 길이가 아니다.

## rqt 파라미터

```bash
ros2 run rqt_reconfigure rqt_reconfigure
```

`/camera_overlay` 노드를 선택해서 다음 값을 바꾸면 즉시 화면에 반영된다.
카메라가 멈춰 있어도 마지막 프레임에 새 크기를 표시한다.

| 파라미터 | 기본값 | 범위 |
|---|---:|---|
| `box_width_px` | 160 | 정수 1~640 |
| `box_height_px` | 120 | 정수 1~480 |
| `box_alpha` | 0.35 | 실수 0.0~1.0, 클수록 빨강이 진해짐 |

`image_topic`은 시작할 때만 지정한다. 기본값은
`/camera/camera/color/image_raw`. launch의 기본 크기·불투명도는
`config/overlay.yaml`에서 바꿀 수 있다. rqt 변경은 실행 중에만 적용되며
YAML에 자동 저장하지 않는다.

터미널에서도 변경 가능하다.

```bash
ros2 param set /camera_overlay box_width_px 200
ros2 param set /camera_overlay box_height_px 100
ros2 param set /camera_overlay box_alpha 0.4
```

이 패키지 추가 시 빌드, ROS 노드 실행, 실제 카메라 확인은 수행하지 않았다.

## 클릭 깊이 측정 테스트 노드 — 2026-10-22

기존 사각형 노드와 별도로 `click_depth_node`를 추가했다.
컬러와 **컬러에 정렬된 depth**를 구독하며, 화면 좌표와 depth 픽셀을 그대로
대응시킨다. 해상도를 변경하지 않는다. 같은 크기여도 정렬되지 않은 raw depth는
올바른 측정에 사용할 수 없다.

패키지를 위 명령으로 다시 빌드한 뒤:

```bash
source /home/noh/my_cv/install/setup.bash
ros2 launch camera_overlay click_depth.launch.xml
```

카메라는 따로 실행해야 한다. 기존 RealSense 실행 시 `align_depth.enable:=true`로
설정해서 아래 토픽이 발행되도록 한다. 다른 이름이면 다음처럼 지정한다.

```bash
ros2 launch camera_overlay click_depth.launch.xml \
  image_topic:=/camera/camera/color/image_raw \
  depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/camera/color/camera_info \
  max_sync_skew_ms:=30.0
```

노드만 실행할 수도 있다.

```bash
ros2 run camera_overlay click_depth_node --ros-args \
  -p image_topic:=/camera/camera/color/image_raw \
  -p depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
  -p camera_info_topic:=/camera/camera/color/camera_info \
  -p max_sync_skew_ms:=30.0
```

- **왼쪽 클릭**: 현재 표시 중인 컬러·depth 쌍을 고정하고, 클릭 `(u,v)`의
  `[u-2,u+2] × [v-2,v+2]` 깊이 중앙값을 화면과 터미널에 표시한다.
  수신 콜백에 더 최신 프레임이 있어도 클릭한 화면의 쌍을 사용한다.
- **고정 화면에서 다시 클릭**: 같은 프레임의 다른 위치를 측정한다.
  백보드 중앙·좌우 끝점처럼 같은 순간의 여러 위치를 비교할 때 쓸 수 있다.
- **Space / c**: 고정과 측정값을 해제하고 최신 영상으로 돌아간다.
- **Esc / q / Ctrl+C**: 종료한다.

표시값은 **카메라 광학축 방향의 Z 깊이와 카메라 기준 X/Z 좌표**다.
카메라에서 점까지의 직선거리나 로봇 몸통 중심 좌표는 아니다.
픽셀 좌표, 중앙값(m/mm), 유효 픽셀 수,
컬러 타임스탬프와 컬러/depth 시간차를 함께 표시한다. 단위 표시의 소수 자릿수는
센서 정확도를 의미하지 않는다.

`16UC1`은 mm, `32FC1`은 m로 해석한다. 0·음수·NaN·무한대는 제외하고, 유효
샘플이 짝수이면 가운데 두 값의 평균을 쓴다. 모두 무효이면 `NO VALID DEPTH`를
표시하며 이전 값을 재사용하지 않는다. 영상 가장자리에서는 존재하는 픽셀만
사용하고 `CLIPPED`와 실제 샘플 수를 표시한다. 물체 경계의 5×5 영역에는 배경이
섞일 수 있으므로 측정할 면 안쪽을 클릭한다.

`message_filters`의 ApproximateTime으로 컬러/depth 타임스탬프 차이를 기본
30ms 이내로 제한한다. 측정 장면에서 움직임이 있으면 차이를 더 작게 설정할 수
있다. 시간차가 큰 프레임이나 크기·encoding이 맞지 않는 쌍은 사용하지 않는다.
두 토픽이 모두 있어야 영상이 표시되며, 카메라 장치를 직접 열지 않는다.

중앙값 계산의 무효값·단위·가장자리·ROI stride 테스트 소스를 추가했다.
사용자가 실행할 명령은 다음과 같다. 실제 카메라나 모터를 구동하지 않는 단위 테스트다.

```bash
cd /home/noh/my_cv
colcon build --packages-select camera_overlay --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
ctest --test-dir /home/noh/my_cv/build/camera_overlay \
  -R 'camera_overlay_(depth|goal)_measurement_test' --output-on-failure
```

이번 변경의 빌드·테스트 실행·카메라 실행은 수행하지 않았다.

## GOAL과 같은 X/Z 및 림 중심 계산

컬러 `CameraInfo.K`에서 `fx=K[0]`, `cx=K[2]`를 읽는다. GOAL과 동일하게
`X=(u-cx)Z/fx`를 사용하며, 렌즈 왜곡 보정·IMU·몸통 좌표 변환을 추가하지 않는다.
카메라 이미지와 CameraInfo의 해상도·frame_id, 유효한 K를 확인하고, 표시한 RGB-D
쌍과 함께 보정값도 고정한다. CameraInfo가 없으면 깊이는 표시하지만 X/Z와 림은
계산하지 않는다. 나중에 보정값이 수신되면 Space로 실시간 복귀 후 다시 클릭한다.

클릭을 백보드 중심으로 가정하고 다음 표본을 함께 표시한다.

| 표본 | 위치·크기 | 용도 |
|---|---|---|
| CLICK | 클릭 중심 5×5 | 기존 깊이 및 점 X/Z |
| B | 클릭 중심 3×3 | 실제 GOAL 방식의 백보드 중심 깊이 |
| L / R | 클릭점에서 좌우 30px, 폭은 로컬 설정(기본 5px), 높이 5px | 백보드 기울기 추정 |

현재 로컬 설정의 밴드 폭은 5px이므로 L/R은 각각 `(u-30,v)`, `(u+30,v)`를
중심으로 한 5×5다. 짝수 폭으로 바꾸면 실제 영역의 픽셀 중심(반 픽셀)을 계산에
사용한다. 밴드 높이 5px는 검출 박스가 없는 수동 시험의 정의이며, **실제 GOAL은
검출된 백보드 박스 전체 높이**를 사용한다. 밴드가 화면 밖이면 림 추정을 중단한다.

GOAL의 `MedianRegionMeters`와 `CountValidDepthPixels` 규칙을 로컬에 구현해서
림 계산용 표본은 `0.05 < Z < 10m`의 값만 사용하고, 짝수 개이면 위쪽 중앙값을
선택한다. 기존 CLICK 5×5의 양수 깊이·짝수 평균 규칙은 유지한다. 따라서 CLICK과
B의 Z 값은 다를 수 있으며 화면에서 구분한다.

이 패키지 내부의 `goal_geometry.hpp`에서 GOAL과 같은 식으로 각도와 림 중심을
계산한다. 외부 함수나 타입을 호출하지 않는다.

```text
X_L = (u_L - cx) Z_L / fx
X_R = (u_R - cx) Z_R / fx
φ = atan2(Z_L - Z_R, X_R - X_L)
B = ((u - cx) Z_B / fx, Z_B)
G = (B_x - R sinφ, B_z - R cosφ)
```

림 반지름·밴드 폭·백보드 거리 범위는 2026-10-22의 GOAL 설정을 복사한
**로컬 값**이다. 기본값은 R=0.10m, 폭=5px, 중심 거리 범위=0.10~2.50m다.
다음 시작 파라미터로만 변경하며, GOAL의 YAML이나 환경변수를 읽지 않는다.
향후 GOAL 설정을 바꾸더라도 이 시험 노드의 값은 자동 변경되지 않는다.

```bash
ros2 run camera_overlay click_depth_node --ros-args \
  -p hoop_radius_m:=0.10 \
  -p backboard_depth_strip_width_px:=5 \
  -p backboard_min_depth_m:=0.10 -p backboard_max_depth_m:=2.50
```

컬러 화면에는 클릭점, 중앙 및 좌우 표본 영역, X/Z·기울기·림 좌표를 표시한다.
별도 **GOAL XZ** 탑뷰에는 카메라 원점, 클릭점, 백보드 방향, 림 중심과 반지름을
표시한다. 수동으로 세 점이 같은 백보드 면에 있다고 가정하는 진단이며, 자동
백보드 검출이나 실제 미션의 시간 관측·진입 판단을 수행하지 않는다.

계산·깊이 표본·상수·시각화·단위 테스트는 모두 `camera_overlay` 내부에 있다.
`vision_core` 링크, `vision` 헤더 포함, 외부 설정 파일 읽기가 없고,
CUDA/TensorRT나 vision 실행 노드를 링크하거나 시작하지 않는다.
