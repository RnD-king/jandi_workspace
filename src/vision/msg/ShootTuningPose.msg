# Used only by shoot_tuning_node. The executor runs DEFAULT_POSE_MODE(2)
# with this action ID, preserving the normal action_status ACK/DONE contract.
# Other ActionCommand callers retain the original motion timing.
uint64 action_id
# Sum of keyframe movement durations; speed limits/arm transitions may add time.
float64 duration_sec
