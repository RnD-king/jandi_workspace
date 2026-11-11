> MAIN/FAST 튜닝 HTTP·SSH 실행, 단계 선택 및 서비스 명령은 [TUNING_HTTP_GUIDE.md](TUNING_HTTP_GUIDE.md)를 참고한다.

# HTTP MJPEG debug view

SSH can start the node without HDMI or a graphical session. After building and
sourcing the workspace, run on the Jetson:

```bash
source /opt/ros/humble/setup.bash
source ~/my_cv/install/setup.bash
ros2 run vision line_perception_node --ros-args \
  -p terminal_node:=true \
  -p stream_debug_view:=true
```

The existing RealSense, executor, and mission start procedures are unchanged.
`stream_debug_view` is a startup-only boolean, default `false`. Add
`-p show_debug_view:=true` only when local OpenCV windows are also wanted.

On Windows or Ubuntu, open `http://<JETSON_IP>:8080` in a browser. Use the Jetson's
hotspot-facing IPv4 address (`hostname -I` on the Jetson), not `localhost` on the
laptop. Both devices must be on the same reachable network; hotspot client
isolation or a firewall can prevent access. An Internet connection is not
needed. Allow TCP 8080 on that interface if a firewall is enabled. The HTTP
server binds all IPv4 interfaces and has no authentication; use the intended
local network. No ROS or SSH client is required on a viewing device.

## Data and execution path

- `/` is a small HTML page containing `/stream.mjpg`.
- Only the final `RenderMissionDebug().image` is encoded. Raw RGB, raw depth and
  the state panel are never transmitted. Input dimensions are retained.
- JPEG quality is 40. The producer and encoder each limit work to at most 15
  frames/s using a steady clock; an old input is not periodically replayed.
- With no streaming viewer, streaming requests no rendering or JPEG encoding.
  Existing GUI/rosbag consumers still get their requested images.
- The node hands over an immutable, reference-counted `cv::Mat` using a
  try-lock; contention drops the submission. There is one replaceable pending
  image, not a FIFO. JPEG encoding runs on its own thread.
- A separate nonblocking `poll` thread handles up to eight HTTP connections.
  All viewers share each encoded JPEG. A client finishes its current JPEG,
  then selects the latest one; no per-client frame queue is built.
- Kernel send buffers are bounded. The next JPEG is selected only after the
  previous response drains from TCP's send queue, including unacknowledged
  bytes, so already-buffered old JPEGs cannot form a hidden FIFO. A frame/response
  that cannot be sent and drained within one second closes that connection,
  so very slow viewers may need a browser
  refresh. Pending HTTP requests have a three-second deadline and 4 KiB limit.
  Closing a viewer does not stop perception or the encoder's other viewers.
- Shutdown wakes both workers, closes client sockets and joins both threads.
  Port conflicts disable streaming with a warning and leave robot control
  running. Encoding errors drop that image; they do not propagate to control.
- In image-only mode the renderer skips state-panel allocation, text wrapping
  and drawing. Image overlays remain identical; small shared diagnostic
  calculations are still performed.

Network waits and JPEG compression are outside ROS callbacks. Rendering and the
small frame handoff still consume time on the existing processing path. Worker
CPU and memory bandwidth are shared with inference: there is no guarantee of
zero scheduling impact. Check actual Jetson performance before competition.

## Measurement

`http://<JETSON_IP>:8080/stats` returns JSON counters: connected streaming
clients, encoded frames/bytes, completed per-client frame sends, last/mean JPEG
encoding time in milliseconds, encode errors, contended submissions, and slow
disconnects. A stats request itself does not enable rendering or encoding.

For two snapshots separated by T seconds:

- encoding FPS = delta `encoded_frames` / T;
- total completed-send FPS = delta `sent_frames` / T (sum across viewers);
- JPEG payload Mbit/s per fully served viewer is approximately
  delta `encoded_bytes` * 8 / T / 1,000,000, excluding HTTP/TCP overhead.

Image content affects JPEG size, so quality 40 does not impose a byte-rate cap.
Compare the existing node's inference/loop/FPS display with streaming off, one
viewer, and multiple viewers. Use a frame timestamp or a visible changing clock
to measure end-to-end latency; successful `send` means bytes entered the socket,
not that the browser displayed them.

## Verification without camera or robot

`vision_debug_mjpeg_test` uses synthetic 640x480 images and loopback sockets on
ephemeral ports. It checks idle behavior, the HTML and statistics routes,
quality/dimensions, shared encoding, a latest-image mailbox, frame-rate bounds,
disconnect/reconnect, a stalled viewer alongside a healthy one, active shutdown,
port conflicts and repeated construction/destruction. It does not launch the
perception node, camera, or executor.

```bash
ctest --test-dir ~/my_cv/build_jandi_debug/vision \
  -R 'vision_(debug_mjpeg|mission_debug_view)_test' --output-on-failure
```

Browser behavior on the actual Windows/Ubuntu laptop and Jetson CPU/load/thermal
behavior require hardware testing; loopback tests do not establish those.
