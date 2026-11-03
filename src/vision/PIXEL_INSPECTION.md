# RGB pixel inspection nodes (read-only)

Two standalone ROS 2 / OpenCV HighGUI windows use **original RGB pixels**.
The image origin is (0,0) at top-left; u increases right, v increases down.
Default input: /camera/camera/color/image_raw (typically 640x480).

Neither node publishes robot motor/action/camera commands. Start the RealSense
driver separately; neither tool moves the robot or camera.

## 1. Mouse click: click_pixel_node

~~~bash
ros2 run vision click_pixel_node
~~~

Click the RGB window to freeze the **displayed** frame. A yellow cross and
CLICK (u,v) = (xxx, yyy) appear on the image and terminal.
Click again to pick another pixel on the **same frozen image**.
Space or c: resume live. q or Esc: exit.
No depth subscription or 3-D calculation is involved.

## 2. YOLO ball: ball_pixel_node

~~~bash
ros2 run vision ball_pixel_node
~~~

The node runs the same vision::YoloTrtEngine TensorRT wrapper as the mission
node. In LIVE mode it draws **all raw BALL bounding boxes** (green), a center
dot (red) on each box and each confidence value.

Press Enter to freeze exactly the **displayed image and its YOLO results**.
The highest-confidence qualifying ball receives a yellow center cross.
Its original-image pixel center and confidence appear on screen:

~~~text
BALL CENTER (u,v) = (xxx, yyy) | conf=...
~~~

The terminal also prints bbox and image timestamp. If there are no matching
BALL detections, Enter still freezes and reports NO BALL at ENTER.
Space or c: resume live. q or Esc: exit.

Bounding-box center is (x + width / 2, y + height / 2), using integer pixel
arithmetic, in the original unrectified color image. These are **raw YOLO
detections**, not mission-accepted targets: no depth, temporal stability,
object association or minimum size filtering is performed. The wrapper
un-letterboxes engine predictions into the source image coordinates.

## Settings and build

Default ball class (1), primary YOLO confidence threshold and second BALL
confidence threshold come from the installed shared_vision_core
vision_algorithm.yaml. Engine selection follows line_perception_node:
YOLO26_ENGINE_PATH if set; otherwise YOLO26_RUNTIME_CONFIG if set; otherwise
the installed vision/config/yolo26_runtime.yaml with pc_engine_path or
jetson_engine_path depending on system architecture.

Override parameters when launching:

~~~bash
ros2 run vision click_pixel_node --ros-args \
  -p image_topic:=/camera/camera/color/image_raw

ros2 run vision ball_pixel_node --ros-args \
  -p image_topic:=/camera/camera/color/image_raw \
  -p inference_hz:=15.0 \
  -p ball_class_id:=1 \
  -p conf_thres:=0.60 \
  -p ball_conf_thres:=0.60
~~~

conf_thres applies inside the TensorRT wrapper; ball_conf_thres applies to
ball boxes afterward. Lowering the latter cannot recover predictions already
removed by the former.

Build in ROS 2 Humble with CUDA and TensorRT after shared_vision_core
has been installed:

~~~bash
source /opt/ros/humble/setup.bash
cd ~/my_cv   # or the appropriate ROS workspace on Jetson
colcon build --packages-select vision --symlink-install
source install/setup.bash
ros2 pkg executables vision | grep pixel_node
~~~

The usual PC/Jetson sync scripts may already perform the build.

Both windows require a graphical display (DISPLAY and OpenCV HighGUI).
A headless SSH session without X11 forwarding cannot show them.
This second inference node imposes **additional GPU work** if the mission
node is running concurrently; avoid it during production latency tests.

These standalone nodes intentionally have no start/reset, camera-control,
or locomotion service. Change head pose only through a separate authorized
camera-control workflow.
