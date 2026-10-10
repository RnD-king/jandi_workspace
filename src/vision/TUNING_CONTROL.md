# LINE P2P tuning / Perception start (workspace-only)

This document describes the local implementation in `src/vision`. The independent
`RnD-king/vision` and `RnD-king/vision_core` repositories and
`src/forward_walk` are intentionally unchanged.

## Single-action tuning node

- Start: `ros2 run vision line_p2p_tuning_node` (camera/debug windows require a display).
- Change gains in RQT only while `IDLE`:
  `line_p2p_offset_gain`, `line_p2p_heading_gain`,
  `line_p2p_steering_deadband`.
- Each press:
  `ros2 service call /line_p2p_tuning_node/start_line_tuning_trial std_srvs/srv/Trigger "{}"`
- First press (or no valid saved guide): perform **1.0 second stationary observation**;
  issue a single action using the averaged valid LineGuide.
- Once ACK arrives, accumulate frames during motion. At READY (fallback DONE),
  finalize using **the production `LineGuideAccumulator::Finish()` 40%-to-READY
  time window and 1-to-3 weighting**, using configured `ready_lead_sec`.
- Save the resulting mean O/H *only inside the tuning node*. Never send a
  READY-queued second ActionCommand to `forward_walk`; it auto-starts queued
  commands at DONE and would violate manual single-step operation.
- After DONE: IDLE, with PREVIOUS and NEXT shown in the independent
  `Line P2P Tuning Status` window. The camera window shows bounding boxes only.
- On an IDLE gain update, re-evaluate NEXT from the stored O/H using the
  shared `LineP2pController`. The service uses the same stored O/H and current
  gains when it issues the action.
- If the moving observation has no valid LineGuide, NEXT is NONE and the next
  service call performs a fresh stationary observation. Never guess an action.

Note: the production node's READY accumulator begins on its first observable
ACK frame; the tuning node follows that convention. When the READY feedback is
missed, both use DONE with zero remaining time. The actual learned controller
output still depends on the camera image, timestamp, model and message timing.

## Perception explicit start

- Start: `ros2 run vision line_perception_node`.
- Its initial state is DISARMED. YOLO and preview run, but
  `MissionController::StepPerception` does not run and no motion/camera commands
  are published.
- To start **one time**:
  `ros2 service call /line_perception_node/start std_srvs/srv/Trigger "{}"`
- A second start request is rejected. Pause, resume, and reset are **not**
  implemented in this phase.
- This is a motion **start gate**, **not an emergency stop**. Use physical power
  controls / a robot-safe procedure for emergencies.

## Integration test checklist (real hardware)

1. Same ROS domain/network, no duplicate vision node, same ActionCommand and
   CommandStatus message definitions.
2. The Jetson RealSense + forward_walk start safely; secure the robot because
   forward_walk moves to WALK_MODE startup pose upon launch.
3. Start tuning node (PC), change gains while IDLE, start one trial, check
   a single ActionCommand and matching ACK/READY/DONE.
4. At READY, status window shows an internally saved NEXT. No second command
   is sent until the next service call. At DONE the state becomes IDLE.
5. Reconfigure gains while IDLE. NEXT should change immediately if the
   score crosses the deadband without a new observation.
6. Hide/obscure the line during a motion: NEXT NONE; next request holds
   and re-observes, not an arbitrary action.
7. Independently start perception node without the start service: it should
   publish no ActionCommand/CameraCommand. Call start; motion is now enabled.
8. Never run both tuning and perception nodes on the same actuator topics
   concurrently.
