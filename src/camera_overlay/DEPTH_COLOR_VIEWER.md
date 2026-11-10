# Depth 거리 컬러·노출 뷰어

기존 ROS Depth를 구독하는 독립 도구다. 카메라/모터를 직접 열거나 시작하지 않는다.
16UC1은 mm → m, 32FC1은 m로 해석한다. 색 범위는 고정이며 프레임별 정규화를 하지 않는다.
기본 0.1m 빨강 → 초록 → 2.5m 파랑. 양수 범위 밖 거리는 끝 색에 포화하며
0/음수/NaN/Inf는 검정이다. 인식용 Depth에 필터나 값을 덧씌우지 않는다.

## 실행

새 배포 도구로 설치했다면:

```bash
~/jandi_camera.sh      # 카메라가 아직 꺼져 있을 때, 터미널 1
~/jandi_depth.sh main  # 터미널 2
```

Jetson HTTP: `http://192.168.0.133:8081`. 카메라가 기존 JANDI에서 실행 중이면 뷰어만 실행한다.
`~/jandi_depth.sh fast`로 FAST에 설치된 같은 도구를 선택할 수 있다.
기본은 headless HTTP다. PC 로컬 화면을 추가하려면 `~/jandi_depth.sh main --opencv`.

직접 ROS 환경을 선택한 터미널에서는:

```bash
ros2 launch camera_overlay depth_color.launch.py
# RGB 좌표계로 정렬된 Depth로 변경
ros2 launch camera_overlay depth_color.launch.py depth_topic:=/camera/camera/aligned_depth_to_color/image_raw
# 표시 거리 변경
ros2 launch camera_overlay depth_color.launch.py min_depth_m:=0.2 max_depth_m:=3.0
```

launch 인자: depth_topic, camera_node, min_depth_m, max_depth_m,
show_debug_view, stream_debug_view, stream_debug_port.
기본 Depth는 `/camera/camera/depth/image_rect_raw`, 파라미터 대상은 `/camera/camera`다.
카메라 동작 도메인과 같은 ROS_DOMAIN_ID(기본22)를 사용한다.

## 조작

- 화면 클릭: 해당 표시 프레임의 Z와 21×21 ROI median·유효 픽셀 비율.
- 선택 지점의 현재 프레임 측정값도 계속 표시한다. 오른쪽 클릭으로 선택 해제.
- AE 체크박스: 자동노출 ON/OFF. 자동노출을 끄면 수동 exposure/gain 입력을 사용할 수 있다.
- 슬라이더 또는 숫자로 값을 정한 뒤 Apply manual values를 누른다.
- 장치 descriptor의 범위·step을 검사하고 응답 후 ROS 파라미터를 다시 읽는다.
- 센서 거부, 서비스 부재·타임아웃, 오래된 화면 클릭은 오류로 표시한다.
- 뷰어 시작과 단순 조회는 카메라 파라미터를 변경하지 않는다.
- 실제 허용 범위/단위는 설치된 드라이버와 카메라 descriptor를 따른다. HDR 등 장치 상태에 따라 변경이 거부될 수 있다.

HTTP 페이지는 JPEG를 약10fps로 조회하며 클릭한 프레임 ID를 보낸다.
영상만 받을 때는 `/stream.mjpg` MJPEG 또는 `/frame.jpg`를 사용할 수 있다.
OpenCV 화면에서는 클릭 측정·우클릭 해제·q/Esc 종료를 지원한다.
HTTP는 LAN에서 접근 가능한 인터페이스로 제공하며 별도 로그인은 없다.

실시간 변경은 파일에 저장하지 않는다. 다음 실행 기본값은
`~/.config/jandi/realsense.launch.py` 상단 DEPTH_AUTO_EXPOSURE / DEPTH_EXPOSURE / DEPTH_GAIN을
직접 수정한 뒤 카메라를 재시작한다. MAIN/FAST가 같은 파일을 사용하며 재빌드는 필요 없다.

## 검사 범위

고정 색 범위·무효 깊이, 단위 변환, 합성 ROS 영상의 HTTP 클릭과
fake 카메라 parameter service의 실제 변경/거부/AE 수동 제약을 검사한다.
실제 D435i 노출 적용과 Jetson 브라우저 성능은 실기 검증 대상이다.
