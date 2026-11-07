# LINE / GOAL stationary search

AUTO LINE-loss and backboard searches visit **left → right → center** in one
camera transaction. The left-to-right movement passes through center without
a center stop; its duration scales with the angular distance. Existing curve
BALL searches retain their continuous sweep behavior.

At each stop the executor verifies the two head encoders before publishing a
new `stop_id` and `stationary_since` timestamp. The vision adapter excludes
captures from before that stop and duplicate/out-of-order RGB timestamps.
Motion phases never vote. RGB and aligned Depth are paired from a bounded
eight-message cache, within `scan_depth_sync_tolerance_sec` (default 0.03 s)
for GOAL search and central rechecks; other mission depth-age rules are unchanged.

Each stop observes for at least 0.5 s. If a candidate appears, a NEW ten-frame
window starts at that first candidate, including misses as frames. Five valid
frames confirm the target. Earlier misses do not consume this window, and a
missing final frame does not erase a confirmed result. Each physical stop has
a 2 s watchdog; missing vision/control messages cannot hold the camera forever.

- LINE: a finite two-or-more-point line fit within ±100 px of its configured
  LINE reference is a search candidate. This does not relax normal walking's
  four-point gate or the separately scoped three-point GOAL floor-return gate.
- GOAL: a filtered backboard with valid central Depth and center within ±100 px
  of `goal.auto_approach_center_u_px` is a candidate.

After a confirmed side sighting, the head returns to center and the existing
bounded body-alignment actions run. A new stationary central window checks the
target again. If a LEFT sighting cannot be reacquired centrally, RIGHT and the
final CENTER remain available instead of an immediate floor-view fallback.
Successful GOAL reacquisition replaces stale target identity and loss counters.
Only exhausted search falls back to the existing bounded LINE/retry policy.

Canonical settings are in `/home/noh/vision_core/config/vision_algorithm.yaml`:
`line.recovery.scan_center_half_width_px`, `scan_hold_sec`,
`scan_observation_timeout_sec`, `scan_verify_window`, `scan_verify_hits`,
`scan_max_pairs`, and the corresponding `goal.control.head_scan_*` values.
The installed YAML must be refreshed and nodes restarted after tuning.

`CameraCommand` and `CameraScanStatus` have new stop/observation fields.
**Rebuild vision and forward_walk together** after installing the updated core;
the previous ROS interface binaries are not compatible with the new messages.
No action numbers or action ACK/READY/DONE contracts are changed. Tests use
synthetic frames and an in-memory motor driver; hardware motion is not launched
by this workflow.
