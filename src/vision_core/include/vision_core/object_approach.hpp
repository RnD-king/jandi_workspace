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
  int lock_blocks{};
  int prelock_loss_extra_blocks{};
  int goal_empty_max_retries{};
  double reference_v_ratio{};
  double steering_deadband_deg{}, in_place_turn_deg{};
};

// BALL and GOAL far-approach steering both use the image-path heading.
// The optional 3D projection remains available for geometry diagnostics.
LineGuide ImageLineGuide(double du_dv, double u_at_reference, double reference_u,
                         int image_width, double confidence);
// With backboard=false, BALL and GOAL far approach connect the image center
// to the robot image reference; no depth or floor-plane calibration is used.
// With backboard=true, the legacy 3D projection remains for diagnostics;
// body_bearing_rad is written only for a valid body-frame (x,z).
LineGuide ObjectApproachGuide(const ObjectTarget& target, bool backboard,
    double optical_depth_m, const Intrinsics& intrinsics,
    const ApproachCameraPose& camera, int image_width, int image_height,
    double reference_u, double reference_v, bool rectify,
    double roll_rad, double pitch_rad, double* body_bearing_rad = nullptr);
}
