# Manual Mock Executor for Vision Tuning (NO MOTOR)

This is a **manual ROS 2 executor substitute**, not a motor simulator.
It subscribes to the same commands as `forward_walk` and publishes the same
`vision/msg/CommandStatus` acknowledgments. It never opens Dynamixel devices.

## Safety / ROS graph isolation

**Stop the real `forward_walk` executor first.** The mock does not prevent
the real executor from receiving and executing a `/jandi_vision/action_cmd`
command. If a physical executor is still running in the same ROS domain,
robot motors may move. For a fully PC-local test, isolate *all* test nodes and
camera sources in an unused `ROS_DOMAIN_ID` (e.g. 87), separate from the
robot's domain 22. A camera on domain 22 will not be visible on domain 87
without an explicit bridge or bag replay.

Only one mock/real executor should subscribe to the command topics at a time.

## Install/build on the PC

After pulling GitHub and syncing the `vision` source to `~/my_cv/src/vision`:

```bash
sudo apt install python3-tk
source /opt/ros/humble/setup.bash
source ~/my_cv/install/setup.bash
cd ~/my_cv
colcon build --packages-select vision --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DSHARED_VISION_CORE_ROOT=$HOME/vision_core/install
source ~/my_cv/install/setup.bash
```

`rclpy` and `vision` message support are required at runtime.

## Run

Terminal A, launch the GUI (no Dynamixel required):

```bash
source /opt/ros/humble/setup.bash
source ~/my_cv/install/setup.bash
ros2 run vision jandi_mock_executor_ui --tuning-node perception_tuning_node
```

Terminal B, launch the tuning node with *external* feedback enabled:

```bash
source /opt/ros/humble/setup.bash
source ~/my_cv/install/setup.bash
ros2 run vision perception_tuning_node --ros-args \
  -p simulate_decision:=false \
  -p simulate_camera_feedback:=false \
  -p enable_command_transport:=true
```

Start your RGB/depth/camera_info publishers as appropriate for the tuning
mode and the selected camera. An engine path and live/replayed images are
still required by the perception node. The mock generates **statuses only**.

For dedicated `ball_tuning_node`, `goal_tuning_node`, or
`hurdle_tuning_node`, pass that node name to the GUI via `--tuning-node`;
service buttons will use the corresponding node-private service names.
For `line_p2p_tuning_node`, the GUI provides its START trial button.

If the tuning node uses custom command/status topics, set the matching
`--action-cmd`, `--action-status`, `--camera-cmd`, and `--camera-status`
arguments on the GUI.

## Controls

| Group | Button / option | Effect |
| --- | --- | --- |
| BODY | ACK | ACK active action |
| BODY | QUEUED ACK | ACK one reserved LINE action |
| BODY | READY | READY active walk (only 11/12/13 and 21-26) |
| BODY | DONE | Complete active action; promote ACKed queued action |
| CAMERA | ACK / DONE | Acknowledge/complete camera command separately |
| MOCK | RESET MOCK STATE | Clear mock state; **does not emit DONE** |
| TUNING | START / PREVIEW / COMMIT | Call existing tuning services |
| TUNING | STOP_REPEAT / RESET | Call existing tuning services |
| Options | Auto body ACK | Automatically ACK accepted body actions |
| Options | Auto camera ACK | Automatically ACK camera requests |
| Options | Auto camera DONE | Automatically ACK and complete camera requests |

Only the services supported by the chosen tuning node are shown.
`line_p2p_tuning_node` has a different START service and no preview/commit.

The mock maintains one active body action, at most one queued LINE->LINE
action admitted after READY, and one independent camera action. It replays
ACK for duplicate active/queued commands, DONE for recently completed IDs,
and rejects invalid/busy requests without fabricated DONE.

**Mock RESET is not the Vision FSM RESET.** If mock state is cleared during
an unfinished command, the tuning node may still be waiting for completion.
Complete the required action or reset/restart the tuning node separately.

## Tests (no motor/ROS required)

```bash
python3 ~/my_cv/src/vision/tests/mock_executor_protocol_test.py
```

The `vision_mock_executor_protocol_test` is also registered for CTest when
`BUILD_TESTING=ON`. Tests cover duplicate replies, READY gates, one-slot
LINE reservation, manual queued ACK, camera independence, and reset behavior.
These are protocol model tests, not end-to-end ROS, vision inference, or
physical motion verification.
