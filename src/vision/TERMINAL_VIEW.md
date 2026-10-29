# SSH terminal viewer

`vision_terminal_node` runs **on Jetson inside the SSH session**. Windows does
not need ROS or image transport. Jetson does the inference and ASCII rendering;
only terminal text crosses SSH. The viewer never publishes motion commands.

## Build on the target machine

The package adds two ROS messages, so rebuild `vision` on Jetson as well as PC:

```bash
source /opt/ros/humble/setup.bash
cd ~/my_cv  # use your Jetson workspace path
colcon build --packages-select vision --symlink-install
source install/setup.bash
```

The existing shared `vision_core`, CUDA and TensorRT build prerequisites apply.
No core algorithm changes are required for this viewer.

## Run in two SSH panes

On Windows Terminal/PowerShell, connect to Jetson using SSH in each pane.
Use `ssh -t user@jetson` if an interactive PTY was not allocated automatically.
Source the target ROS/workspace environment in both sessions.

In the perception pane, with camera topics already running:

```bash
ros2 run vision line_perception_node --ros-args \
  -p terminal_node:=true \
  -p show_debug_view:=false \
  -p show_yolo_debug_view:=false \
  -p rosbag:=false
```

In the viewer pane:

```bash
ros2 run vision vision_terminal_node
```

These are the normal ROS 2 parameters (`--ros-args -p`), not a literal
`--terminal_node:=true` command-line switch. `terminal_node` defaults to false:
no terminal publisher is created unless explicitly enabled. Starting the viewer
does not launch perception, infer a second time, or start/reset a robot mission.
Existing START/PAUSE/RESUME/RESET controls are unchanged.

Both ROS nodes must use the same domain. Keep their ROS traffic local to Jetson
when using the hotspot; subscribing to RGB/depth from Windows would add image
traffic independently of this viewer. Existing perception logs belong in the
other pane so they do not interfere with the viewer screen.

## Screen

Default 640x480 input is shown in a **48-column x 18-row** grid. With two border
rows and four status rows the entire screen is **50 columns x 24 rows**.
This assumes a terminal character is roughly twice as tall as it is wide.
The renderer reserves the terminal's final column, has no trailing newline,
and scales down if necessary to avoid wrapping/scrolling. Extra horizontal space
does not stretch the camera image. Set `grid_columns` to change the desired width;
the actual size still fits the terminal and image aspect ratio.

* `.`: accepted line bounding-box center in original image coordinates.
* `B`: ball, `G`: goal, `D`: backboard, `H`: hurdle.
* Objects use a distinct color per class and fill cells whose area is **more
  than 50%** covered by their bounding box (exactly 50% is excluded). The cell
  containing the object's center is always filled, even for a tiny box. If the
  center is outside the image but part of the box is visible, the visible
  clipped rectangle's center is used instead. Existing overlap priority remains:
  smaller objects cover larger ones, and line points are drawn last.
* Running perception shows `FILTERED`, using the core's current accepted object
  indices, including confidence/size/depth/aspect filtering. No second filter or
  target association is implemented in the terminal node.
* WAIT_START/PAUSED/RESETTING and camera transitions show `RAW` current YOLO
  objects; frozen mission boxes are never reused as new detections. Raw objects
  have not passed the core's depth/temporal acceptance gates.
* The four status rows show run state and receive frequency, mission/phase,
  action/camera, and detection counts/data age/text output rate. Long fields are
  truncated to the available width. `CAM:...*` means the camera is not settled.
* No message for two seconds shows `STALE` and clears the object grid. Before the
  first message the screen shows `NO DATA` with a parameter reminder.

Ctrl+C restores the cursor and the original terminal screen. No OpenCV window,
X forwarding, NumPy or additional terminal package is needed by the Python viewer.
The renderer emits single-byte ASCII, plus ANSI cursor/color control sequences.

## Rate and parameters

Publication and display both default to **5 Hz**. Latest-only best-effort QoS
(depth one) and a steady-clock rate cap prevent a backlog of old frames. The
publisher builds metadata only while a viewer is subscribed. Inference retains
its existing rate and behavior. Parameters are startup-only.

| Node | Parameter | Default |
| --- | --- | --- |
| line_perception_node | terminal_node | false |
| line_perception_node | terminal_fps | 5.0 |
| both | terminal_topic | /jandi_vision/terminal |
| vision_terminal_node | refresh_hz | 5.0 |
| vision_terminal_node | grid_columns | 48 |
| vision_terminal_node | stale_timeout_sec | 2.0 |
| vision_terminal_node | color | true |

For a slower link, set the publisher to `-p terminal_fps:=2.0` and the viewer to
`--ros-args -p refresh_hz:=2.0`. `-p color:=false` reduces ANSI bytes further.

The `OUT:...KB/s` field counts ASCII/ANSI bytes emitted by the viewer over a
recent window. It is not a network measurement and excludes SSH/TCP overhead,
other panes, and any camera or ROS traffic crossing the network.
The earlier 0.1-0.3 Mbps at 5 Hz was a planning estimate, not a measured result.
No image data is present in `/jandi_vision/terminal` (`vision/msg/TerminalFrame`).

## Verification commands

Implementation verification uses compilation/static checks. To run the focused
logic tests yourself after building:

```bash
colcon test --packages-select vision --ctest-args \
  -R 'vision_terminal_(view|publisher)_test' --output-on-failure
colcon test-result --verbose
```

These cover the 24-row layout/aspect ratio, clipping/overlap, filtered/raw modes,
stale-frame clearing, terminal escape sanitization and ROS metadata generation.
Inspect publication without subscribing to images:

```bash
ros2 topic info /jandi_vision/terminal
ros2 topic hz /jandi_vision/terminal
```
