uint8 IDLE=0
uint8 OUTBOUND=1
uint8 DWELL=2
uint8 PAUSED=3
uint8 RETURNING=4
uint8 VERIFY=5
uint8 SETTLING=6

uint64 command_id
uint8 phase
# Last commanded (not measured) head yaw relative to scan origin (FORWARD or GOAL), degrees.
float64 commanded_yaw_deg
uint32 stop_id
builtin_interfaces/Time stationary_since
