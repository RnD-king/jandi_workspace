# 실시간 YOLO 확인 노드

RealSense와 `forward_walk/main_node`는 별도 터미널에서 실행한다.
아래 노드는 `/start` 없이 계속 탐지 화면을 보여준다. 몸 동작 명령은 발행하지 않는다.
실제 미션의 클래스별 박스 색상을 사용하며, 미션 FSM·접근·정렬·던지기는 실행하지 않는다.

```bash
# 카메라 자세를 건드리지 않고 현재 FORWARD 영상을 표시
ros2 run camera_overlay forward_yolo_node

# GOAL 카메라 자세 요청 → 실제 DONE → 박스 + 중앙·좌우 깊이 밴드 표시
ros2 run camera_overlay goal_yolo_node
```

기본값은 `stream_debug_view:=true`, `show_debug_view:=false`, 포트 8080이다.
브라우저에서 `http://JETSON_IP:8080`으로 접속한다.
두 노드는 보통 하나씩 실행한다. 동시에 화면을 열려면 한쪽 포트를 바꾼다.
두 화면 모두 같은 실제 카메라 영상을 받으므로 GOAL 노드를 켜면 FORWARD 화면에도
고개를 올린 영상이 보인다.

```bash
ros2 run camera_overlay goal_yolo_node --ros-args -p stream_debug_port:=8081
# 로컬 OpenCV 창도 표시하려면 -p show_debug_view:=true
```

GOAL 요청은 `/jandi_vision/camera_cmd`로 보내고, 같은 command_id의
`/jandi_vision/camera_status` DONE을 기다린다. ACK 또는 시간 경과를 DONE으로 간주하지 않는다.
누락된 피드백은 같은 ID 재전송으로 복구하고, DONE 이후에는 카메라 명령을 더 보내지 않는다.
종료 시 FORWARD 자세로 돌리는 명령도 보내지 않는다.

GOAL의 BACKBOARD 박스에는 실제 디버그 화면과 동일한 `DrawDepthRegions`를 사용한다.
중앙 ROI/fallback, 좌우 밴드 자동 확장, 깊이 중앙값, 유효 픽셀 수/비율(%)과
유효 깊이 범위는 선택된 shared_vision_core의 `vision_algorithm.yaml`에서 가져온다.
GOAL 카메라 DONE 이후 촬영된 RGB와 해상도가 같고 타임스탬프 차이가 30ms 이내인
aligned depth만 사용한다. Depth가 없거나 맞지 않으면 박스·영역 표시는 계속하되
깊이 숫자를 표시하지 않고 Depth unavailable 문구를 표시한다.

공통 파라미터: `image_topic`, `engine_path`, `conf_thres`, `inference_hz`(기본 15Hz),
`show_debug_view`, `stream_debug_view`, `stream_debug_port`.
GOAL 추가 파라미터: `depth_topic` (기본 `/camera/camera/aligned_depth_to_color/image_raw`).
엔진 선택은 기존 노드처럼 `YOLO26_ENGINE_PATH`, `YOLO26_RUNTIME_CONFIG`,
설치된 `vision/config/yolo26_runtime.yaml` 순서다.
