# Metadata only. Coordinates refer to the original image, never an RGB payload.
std_msgs/Header header
uint32 image_width
uint32 image_height
string control_state
string mission
string phase
string action_name
uint16 action
uint64 waiting_action_id
string camera
bool camera_settled
# False while disarmed/paused/resetting: current raw YOLO boxes are displayed
# instead of reusing the frozen mission's accepted detections.
bool filters_current
# Full-width status rows, populated by terminal_publisher.
string curve_info
string object_info
string control_info
string walk_info
string fine_info
TerminalDetection[] detections
