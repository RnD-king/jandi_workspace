# Perception operator controls (START / PAUSE / RESUME / RESET)

Scope: **only** `jandi_workspace/src/vision` and its local
`src/vision_core`. `forward_walk` and friend's `2026_motion` are untouched.

## Services (std_srvs/srv/Trigger)

```bash
ros2 service call /line_perception_node/start std_srvs/srv/Trigger "{}"
ros2 service call /line_perception_node/pause std_srvs/srv/Trigger "{}"
ros2 service call /line_perception_node/resume std_srvs/srv/Trigger "{}"
ros2 service call /line_perception_node/reset std_srvs/srv/Trigger "{}"
```

States: `WAIT_START -> OBSERVING (1 s) -> RUNNING -> PAUSING ->
PAUSED -> OBSERVING (1 s) -> RUNNING`. RESET from PAUSED returns to
WAIT_START. START is only accepted from WAIT_START; RESUME only from PAUSED.
PAUSE is accepted from RUNNING or OBSERVING. Rejected requests reply
`success=false` and a reason.

### PAUSE semantics

- Do not stop any already-running JSON/P2P motion in the middle.
- If the previous LINE motion has already queued the next action B in
  forward_walk, B finishes too. READY received but **no B published yet**
  is *not* permission to send B after PAUSE.
- The Vision core is still stepped during PAUSING with a **global gate
  for LINE/BALL/GOAL/HURDLE and all camera requests**. Already-published
  action IDs may be retransmitted before ACK; no new ID may be sent.
- An already-sent, unacknowledged camera request is retransmitted with its
  *original command_id*. All ACK/READY/DONE feedback continues to update
  the mission coordinator.
- PAUSED is acknowledged internally only when **every ActionCommand
  published by this node** has matching DONE, there is no pending camera
  command, and the feedback queue has been drained through the core.
- If any dispatched command is lost or rejected without DONE, PAUSING
  intentionally remains pending: **no fake DONE or automatic RESET**.
- Other publishers on `/jandi_vision/action_cmd` are not tracked.
  Only one Vision command source should be active for a real robot.

### RESUME

- Discards pre-pause tracking and LINE history while retaining mission FSM.
- Collects 1.0 second of new, stationary camera frames.
- For LINE, applies uniform-mean O/H across valid fresh observations
  (same `FinishAll` mechanism as the first stationary tuning trial).
  Insufficient valid samples => invalid guide, not arbitrary movement.
- For object missions, the same 1-second period fills fresh tracking history;
  existing object-mission stability/settling requirements still apply.
- Does **not** execute a cached previous NEXT action.
- Mission logical time excludes both PAUSING and PAUSED intervals to avoid spuriously
  expiring object/camera state timers while the user inspects the robot.
- Once the one-second period completes, normal READY/one-slot queue
  continuous execution is restored.

### RESET

- Accepted **only in PAUSED** when all command IDs and camera activity have
  completed and no unprocessed action feedback remains.
- Clears all mission controller state, line evidence, object tracking, next
  command state; leaves motor torque and current commanded robot pose alone.
- Action IDs remain strictly increasing within the running Vision process,
  preventing collisions with the executor's last-32 completed-ID cache.
  Camera command IDs are never reset either.
- Retains the actual last known camera mode, rather than falsely treating
  the camera as FORWARD. RESET does **not** move the head to FORWARD;
  if the camera is physically DOWN/GOAL, position it appropriately before
  beginning LINE operation again.
- This is an **operator workflow pause**, not an emergency stop. Motions
  already accepted by forward_walk may continue after pressing PAUSE.

## Building with the updated local shared library (important)

Do not reuse the old, independent `~/vision_core/install` for this
version: the local shared core gained a global command gate, fresh-evidence
reset, and reset-with-ID-preservation APIs.

From the target workspace (PC or Jetson, after git pull):

```bash
cd ~/jandi  # Or the real path of the jandi_workspace clone
source /opt/ros/humble/setup.bash
cmake -S src/vision_core -B build/local_shared_core \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$PWD/local_shared_core/install"
cmake --build build/local_shared_core -j2
ctest --test-dir build/local_shared_core --output-on-failure
cmake --install build/local_shared_core

export VISION_CORE_ROOT="$PWD/local_shared_core/install"
colcon build --packages-select vision --symlink-install \
  --cmake-clean-cache \
  --cmake-args -DSHARED_VISION_CORE_ROOT="$VISION_CORE_ROOT"
source install/setup.bash
```

Ensure `LD_LIBRARY_PATH` / RPATH points to the freshly built shared
core; `ldd install/vision/lib/vision/line_perception_node` can verify.

## Test cases

1. Before START: YOLO/debug continues but no ActionCommand or CameraCommand.
2. START: no action for the first 1 second; then valid LINE/action decision.
3. PAUSE before READY(A): A DONE, no B sent, PAUSED.
4. PAUSE after B already sent: A DONE -> B DONE -> PAUSED; no C sent.
5. PAUSE during BALL PICK/GOAL SHOOT/HURDLE: current action completes;
   any not-yet-published successor is blocked; no new CameraCommand.
6. PAUSE while camera request pending: its original ID may be retried;
   PAUSED only after camera DONE.
7. RESUME: older frames and trackers do not decide the new action; 1 s of
   fresh observations, then normal motion.
8. RESET while RUNNING/PAUSING: rejected. RESET while PAUSED: accepted.
   After RESET+START the action ID must never reuse an old ID.
9. Lost ACK/DONE: no false PAUSED or RESET; inspect executor error logs.
10. Do NOT operate both `line_perception_node` and
    `line_p2p_tuning_node` against the same robot simultaneously.

Hardware acceptance tests are required separately from unit tests.
