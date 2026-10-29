# Used only by shoot_tuning_node. One fixed 23-motor pose (including GOAL head
# ticks), without walking keyframes, arm/gripper overrides or a separate camera
# request. Uses normal action_status ACK/DONE and the shared action ID history.
# Other ActionCommand callers retain their original DEFAULT_POSE_MODE behavior.
uint64 action_id
# Full pose interpolation time; the existing 60 deg/s speed limit may add time.
float64 duration_sec
