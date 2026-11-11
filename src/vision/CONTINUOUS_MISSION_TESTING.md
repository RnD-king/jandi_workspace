# Continuous object mission tests

PC source targets (available after rebuilding/installing the matching core and
`vision` package):

- `ball_continuous_test_node`
- `hurdle_continuous_test_node`
- `goal_continuous_test_node`

These three executables compile `src/line_perception_node.cpp`, the same runtime
loop as the real AUTO node. They use its CameraInfo gate, RGB-D/scan freshness,
READY queue, camera scan transport, and PAUSE/RESUME handling. The selected stage
only prepares an entry state. Far acquisition and walking use the production
LINE planner and continuous-object router; near/FINE entry uses the production
controller hooks. The existing `*_tuning_node` executables retain their
PREVIEW/COMMIT behavior.
The continuous nodes default to `show_debug_view:=false` and
`stream_debug_view:=true` (HTTP port 8080). Run one robot command producer at a time.

```bash
ros2 run vision goal_continuous_test_node --ros-args -p test_stage:=1
```

`test_stage` selects the starting point: 0 full entry, 1 approach, 2 near
alignment, 3 execution/alignment recovery. The YAML default remains the default
when this parameter is omitted. The selected object's FSM then continues through
execution and its normal completion/recovery; stages 1 and 2 do not stop at the
old tuning boundaries. GOAL starts in GOAL view for stages 1–3. BALL and HURDLE
start in FORWARD for stages 0–1, BALL/DOWN respectively for stages 2–3.
Stage 3 prepares FINE stance; the other stages prepare WALK stance. A valid RGB
CameraInfo is required before START, exactly as in `line_perception_node`.

On launch, only the selected starting stance and camera are prepared. Wait for
`WAIT_START` after their actual DONE feedback, then START:

```bash
ros2 service call /goal_continuous_test_node/start std_srvs/srv/Trigger '{}'
ros2 service call /goal_continuous_test_node/pause std_srvs/srv/Trigger '{}'
ros2 service call /goal_continuous_test_node/resume std_srvs/srv/Trigger '{}'
ros2 service call /goal_continuous_test_node/reset std_srvs/srv/Trigger '{}'
```

PAUSE admits no new commands and waits for all already dispatched body/camera
commands to finish. RESUME retains mission progress, consumes the saved ordered
executor feedback, and uses fresh observations. RESET is accepted in PAUSED or WAIT_START once no command
is outstanding; it resets mission evidence and prepares the selected stage again.
It does not move the robot back to its earlier physical location. Completion or
failure requires RESET before a new START; an explicit stage change also prepares
a new start. PREVIEW/COMMIT are not used in these nodes.

After RESET completes in WAIT_START, change the stage independently:

```bash
ros2 param set /goal_continuous_test_node test_stage 2
```

Wait for the new stance/camera preparation to complete, then START again.
Substitute `ball` or `hurdle` in the node/service names for the other objects.
Shared algorithm values still come from the selected core YAML. Continuous-test
policy is assigned by these dedicated executables; it is not a YAML algorithm
default. Build/install the matching `vision_core` before rebuilding `vision`,
because the shared config and core API changed.
