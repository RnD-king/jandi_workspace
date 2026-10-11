# forward_walk 10.12.1 integration

Base: `jandi_workspace/feature/perception-unified-tuning-20261011` (2026-10-11).
Imported from: `2026_motion/main/10.12.1/forward_walk`.
The other workspace packages (`vision`, `vision_core`, etc.) have not been changed in this integration.

## Intended behavior

- LINE locomotion (actions 2–13, 18/19) uses the symmetric arm-target override: ID 15 = 1379, ID 16 = 2765 by default. BALL/GOAL/HURDLE use the original per-motion arm positions.
- Action 11 (STEP_FORWARD_LEFT): `missions/직진_좌회전_3번.json` (three repeats).
- Action 12 (STEP_FORWARD_RIGHT): `missions/직진_우회전_3번.json` (three repeats). Two-repeat alternatives are also copied but not mapped.
- Preserve camera ID 21/22 calibration: FORWARD 2077/1537, DOWN 2036/2013, GOAL 2054/966. In particular, `motions/보행_왼발만앞에.json` remains the original from `jandi_workspace`, not the upstream 10.12.1 variant.
- Action 11/12/13 emits READY once before DONE **for every mission** (LINE/BALL/GOAL/HURDLE). The speculative one-slot queue is allowed **only** for LINE-to-LINE action 11/12/13; a non-LINE READY does not allow early queuing.
- Preserve existing `ready_lead_sec` (default 0.25 seconds), ACK/DONE, camera/Action ID separation, 50 Hz writer, and pre-validation of queued programs.
- `dynamixel_device` ROS parameter allows choosing the adapter (default `/dev/jandi_dxl`). The optional udev rule targets FTDI serial FT9BTGLH and must be adjusted for any other adapter.

## Tests and verification

- `tests/executor_logic_test.cpp`: arm target/range/P2P and repeat tests; cross-mission READY and LINE-only queue; actual three-repeat mission manifests.
- `tests/executor_integration_test.cpp`: production `missions/` paths with three repeats; startup/LINE arm override; BODY/CAMERA statuses; one-slot queue; READY for non-LINE 11.
- `tests/source_contract_test.py`: production Action→JSON paths, 2x/3x mission content and camera defaults.

Run a ROS 2 Humble build and all forward_walk CTest checks before motor testing:

```bash
# From the jandi_workspace root, after building/installing vision_core and vision:
colcon build --symlink-install --packages-up-to forward_walk
source install/setup.bash
colcon test --packages-select forward_walk --event-handlers console_direct+
colcon test-result --verbose
```

This GitHub integration was source-reviewed; it does **not** claim to have run the ROS 2 build, CTest, or real motor verification. The three-repeat trajectories increase open-loop travel/time, so verify steering gains and READY observation timing on the robot before regular autonomous operation.

## Sharing with the motion developer

Copy the **integrated** `src/forward_walk` directory from this branch. Do not overwrite it again with `2026_motion/10.12.1/forward_walk`: doing so would revert the cross-mission READY policy and the preserved camera startup JSON values. The recipient's `vision/msg/ActionCommand.msg` and camera/status messages must be compatible.
