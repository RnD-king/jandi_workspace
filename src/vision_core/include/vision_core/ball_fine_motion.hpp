#pragma once
#include "vision_core/types.hpp"

namespace vision_core {
// Measured displacement of the BALL CENTER in the DOWN image after one
// completed body action. A zero vector means this direction is uncalibrated.
struct BallFineMotionConfig {
  int max_repetitions{3};
  Point2 forward_delta_px,backward_delta_px,left_delta_px,right_delta_px;
  bool HasCalibration() const;
};
struct BallFineMotionPlan {
  MissionAction action{MissionAction::kNone};
  int repetitions{0};
  Point2 predicted_center_px;
};
// A single direction only. Every predicted prefix must improve rectangle
// error, keep the center visible and avoid crossing the opposite target edge.
BallFineMotionPlan PlanBallFineMotion(const BallFineMotionConfig& config,
    Point2 center,double u1,double u2,double v1,double v2,
    int image_width,int image_height,int repetition_limit=3);
} // namespace vision_core
