#pragma once

#include "vision_core/types.hpp"

namespace vision_core {
// Body coordinates: right/up/forward in metres. Camera pixels use right/down.
// Positive pitch looks down; positive yaw looks right. These are measured
// optical poses, NOT Dynamixel encoder ticks.
struct ApproachCameraPose {
  double right_m{}, height_m{}, forward_m{}, pitch_deg{}, yaw_deg{};
};
struct ObjectApproachConfig {
  int lock_blocks{}, blend_frames{};
  int prelock_loss_extra_blocks{};
  int goal_empty_max_retries{};
  double reference_v_ratio{};
  double heading_gain{}, steering_deadband{}, in_place_turn_score{};
  ApproachCameraPose goal_camera;
};

// Image geometry for the debug path. BALL steering uses its image H; GOAL
// steering uses the optional body bearing, not the projected image-line H.
LineGuide ImageLineGuide(double du_dv, double u_at_reference, double reference_u,
                         int image_width, double confidence);
// The backboard uses measured optical Z. A ball connects its RGB center to the
// robot image reference; no ball depth or floor-plane calibration is needed.
// body_bearing_rad is written only for a valid backboard body-frame (x,z).
LineGuide ObjectApproachGuide(const ObjectTarget& target, bool backboard,
    double optical_depth_m, const Intrinsics& intrinsics,
    const ApproachCameraPose& camera, int image_width, int image_height,
    double reference_u, double reference_v, bool rectify,
    double roll_rad, double pitch_rad, double* body_bearing_rad = nullptr);
}
