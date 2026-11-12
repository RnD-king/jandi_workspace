uint8 DOWN=1
uint8 FORWARD=2
uint8 GOAL=3
uint8 SCAN_LEFT=4
uint8 SCAN_RIGHT=5
uint8 SCAN_HOLD=6
uint8 SCAN_RESUME=7
uint8 SCAN_RETURN=8
uint8 BALL_VIEW=9
uint8 BALL_VIEW_LINE=10
uint8 LINE_SCAN_LEFT=11
uint8 LINE_SCAN_RIGHT=12
uint8 GOAL_SCAN_LEFT=13
uint8 GOAL_SCAN_RIGHT=14
# Same FORWARD observation mode; a distinct pose request and physical DONE.
uint8 POST_HURDLE_FORWARD=15

uint64 command_id
uint8 request
uint16 scan_max_yaw_deg
float32 scan_move_sec
float32 scan_hold_sec
float32 scan_return_sec
# Bound one stationary stop; zero selects legacy continuous scanning.
float32 scan_observation_timeout_sec
# Idempotent controls target the exact stop being observed (zero for legacy).
uint32 scan_stop_id
