# 선택 녹화를 위한 디버깅 이미지 발행

```bash
ros2 run vision line_perception_node --ros-args -p rosbag:=true
```

`rosbag`은 기본 `false`다. `true`는 디버깅 이미지 토픽 발행을 허용하며,
bag 파일 저장은 별도 녹화 스크립트가 담당한다.
`rosbag_fps` 기본값은 `10.0`이며 최대 발행 빈도를 제한한다.
둘 다 실행 시 지정하는 파라미터다.

적용 실행 파일:

- `line_perception_node`
- `line_p2p_tuning_node`
- `perception_tuning_node`
- `ball_tuning_node`
- `hurdle_tuning_node`
- `goal_tuning_node`

`show_debug_view` / `show_yolo_debug_view`와 독립적이므로 창 없이도 녹화 가능하다.
화면 없는 환경에서 튜닝 노드를 실행할 때는 `-p show_debug_view:=false`를 지정한다.

발행 토픽 (`sensor_msgs/msg/Image`, `bgr8`):

| 토픽 | 내용 |
| --- | --- |
| `/jandi_vision/debug/image` | 판정 결과를 그린 영상 |
| `/jandi_vision/debug/panel` | 상태 패널 |
| `/jandi_vision/debug/yolo` | YOLO 원시 검출 영상; LINE 전용 튜닝 노드에서는 생략 |

원본 프레임의 header를 보존한다. best-effort / volatile / KeepLast(1)을 사용하며,
해당 구독자가 있을 때만 최대 fps 범위에서 녹화용 화면을 생성한다.
메시지 변환·전송은 별도 스레드에서 처리하고 대기 프레임은 하나만 유지한다.
렌더링·저장 부하는 남으며 모든 디버깅 프레임 전달을 보장하지 않는다.
노드 여러 개를 동시에 실행하면 ROS remap으로 디버깅 토픽을 구분해야 한다.

다른 터미널에서:

```bash
bash ~/ros2_bag/record.sh 1 2 3  # 원본 / 디버깅 / 동작 데이터 모두
bash ~/ros2_bag/record.sh 1 3    # 원본 + 동작 데이터, rosbag:=true 불필요
bash ~/ros2_bag/record.sh        # 선택 메뉴
bash ~/ros2_bag/play.sh latest 1 3
```

저장 위치는 `~/ros2_bag/recordings/실행시각/{original,debug,action}/`다.
녹화를 시작한 뒤 vision의 START를 실행한다. Ctrl+C는 모든 선택 녹화를 함께 종료한다.
재생은 선택한 bag을 기록 시각 기준으로 통합하고 `/replay/...` 토픽으로 발행한다.
전체 사용법: [ros2_bag/README.md](/home/noh/ros2_bag/README.md).
