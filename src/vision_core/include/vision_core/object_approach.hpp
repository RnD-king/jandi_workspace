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
  int missing_frames{}, lock_blocks{}, blend_frames{};
  int goal_empty_max_retries{};
  double reference_v_ratio{};
  ApproachCameraPose forward_camera, goal_camera;
};

// The same image-line-to-O/H conversion is used by actual and virtual lines.
LineGuide ImageLineGuide(double du_dv, double u_at_reference, double reference_u,
                         int image_width, double confidence);
// The backboard uses measured optical Z. A ball uses its box's ground-contact
// pixel and the calibrated floor plane; no synthetic detections are generated.
LineGuide ObjectApproachGuide(const ObjectTarget& target, bool backboard,
    double optical_depth_m, const Intrinsics& intrinsics,
    const ApproachCameraPose& camera, int image_width, int image_height,
    double reference_u, double reference_v, bool rectify,
    double roll_rad, double pitch_rad);
}
