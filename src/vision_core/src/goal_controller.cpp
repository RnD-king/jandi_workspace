#include "vision_core/goal_controller.hpp"

#include "vision_core/config_loader.hpp"
#include "vision_core/cruise_selector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace vision_core {
namespace {
constexpr double kEpsilon = 1e-9;
constexpr double kPi = 3.14159265358979323846;
double Denom(double value) { return std::max(value, 1e-6); }
ActionRequest Locomotion(MissionAction action) {
  return {action, ActionCategory::kLocomotion, 0, false};
}
ActionRequest Mission(MissionAction action, std::int16_t yaw = 0) {
  return {action, ActionCategory::kMission, yaw, false};
}
double Wrap(double angle) {
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
}
} // namespace

GoalController::GoalController() {
  const auto config = LoadDefaultAlgorithmConfig();
  config_ = config.goal;
  camera_motion_timeout_sec_ = config.camera_motion_timeout_sec;
}
GoalController::GoalController(const GoalConfig &config,
                               double camera_motion_timeout_sec)
    : config_(config), camera_motion_timeout_sec_(camera_motion_timeout_sec) {}

GoalPoseObservation EstimateGoalPoseFromEdgeDepths(
    double left_u_px, double left_depth_m, double right_u_px,
    double right_depth_m, const Intrinsics &intrinsics, double confidence) {
  if (!std::isfinite(left_u_px) || !std::isfinite(right_u_px) ||
      !std::isfinite(left_depth_m) || !std::isfinite(right_depth_m) ||
      !std::isfinite(intrinsics.fx) || std::abs(intrinsics.fx) <= kEpsilon ||
      left_depth_m <= 0.0 || right_depth_m <= 0.0) return {};
  const double left_x =
      (left_u_px - intrinsics.cx) * left_depth_m / intrinsics.fx;
  const double right_x =
      (right_u_px - intrinsics.cx) * right_depth_m / intrinsics.fx;
  const double width = right_x - left_x;
  if (!std::isfinite(width) || width <= kEpsilon) return {};
  GoalPoseObservation result;
  result.valid = true;
  result.x_m = 0.5 * (left_x + right_x);
  result.z_m = 0.5 * (left_depth_m + right_depth_m);
  result.yaw_rad = std::atan2(left_depth_m - right_depth_m, width);
  result.confidence = confidence;
  return result;
}

GoalPoseObservation EstimateGoalPoseFromBackboardDepths(
    double center_u_px, double center_depth_m, double left_u_px,
    double left_depth_m, double right_u_px, double right_depth_m,
    const Intrinsics &intrinsics, double confidence) {
  GoalPoseObservation result = EstimateGoalPoseFromEdgeDepths(
      left_u_px, left_depth_m, right_u_px, right_depth_m, intrinsics,
      confidence);
  if (!result.valid || !std::isfinite(center_u_px) ||
      !std::isfinite(center_depth_m) || center_depth_m <= 0.0 ||
      !std::isfinite(intrinsics.fx) || std::abs(intrinsics.fx) <= kEpsilon)
    return {};
  result.x_m =
      (center_u_px - intrinsics.cx) * center_depth_m / intrinsics.fx;
  result.z_m = center_depth_m;
  return result;
}

const char *GoalController::ModeName(GoalMode mode) {
  switch (mode) {
  case GoalMode::kLineFollow: return "LINE_FOLLOW";
  case GoalMode::kWaitCameraGoal: return "GOAL_CAMERA";
  case GoalMode::kSearch: return "GOAL_SEARCH";
  case GoalMode::kApproach: return "GOAL_APPROACH";
  case GoalMode::kFineAdjust: return "GOAL_FINE";
  case GoalMode::kShoot: return "GOAL_SHOOT";
  case GoalMode::kReturnCameraToLine: return "GOAL_CAMERA_FORWARD";
  case GoalMode::kHeadingRecovery: return "GOAL_LINE_RECOVERY";
  case GoalMode::kFailed: return "GOAL_FAILED";
  }
  return "UNKNOWN";
}

double GoalController::Clamp(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

void GoalController::StartAfterPickup(double) {
  SetHasBall(true);
  goal_entry_armed_ = true;
}

void GoalController::SetHasBall(bool has_ball) {
  if (!has_ball) {
    has_ball_ = false;
    goal_entry_armed_ = false;
    post_pickup_line_wait_active_ = false;
    if (mode_ == GoalMode::kLineFollow) ball_consumed_ = false;
    return;
  }
  if (!ball_consumed_) has_ball_ = true;
}

void GoalController::UpdateBallState(const BallResult &ball) {
  SetHasBall(ball.has_ball);
  goal_entry_armed_ = has_ball_ && !ball_consumed_ &&
                      ball.mode == BallMode::kLineFollow;
  if (!goal_entry_armed_) post_pickup_line_wait_active_ = false;
}

void GoalController::UpdateGoalTracker(
    const std::optional<ObjectTarget> &target, int image_width,
    int image_height) {
  const bool detected = target && image_width > 1 && image_height > 1;
  hit_history_.push_back(detected);
  while (static_cast<int>(hit_history_.size()) >
         std::max(1, config_.stable_window)) hit_history_.pop_front();
  if (detected) {
    lost_count_ = 0;
    const Point2 center = target->center_rectified
                              ? target->rectified_center_px
                              : target->center_px;
    TrackedGoal observed;
    observed.visible = true;
    observed.u_norm = Clamp(center.u / Denom(image_width), 0.0, 1.0);
    observed.v_norm = Clamp(center.v / Denom(image_height), 0.0, 1.0);
    observed.h_norm = Clamp(target->height_px / Denom(image_height), 0.0, 1.0);
    observed.confidence = target->confidence;
    const double alpha = Clamp(config_.smooth_alpha, 0.0, 1.0);
    if (!has_smoothed_) {
      tracked_ = observed;
      has_smoothed_ = true;
    } else {
      tracked_.visible = true;
      tracked_.u_norm = (1.0 - alpha) * tracked_.u_norm + alpha * observed.u_norm;
      tracked_.v_norm = (1.0 - alpha) * tracked_.v_norm + alpha * observed.v_norm;
      tracked_.h_norm = observed.h_norm;
      tracked_.confidence = observed.confidence;
    }
  } else {
    ++lost_count_;
    tracked_.visible = false;
  }
  const int hits = static_cast<int>(
      std::count(hit_history_.begin(), hit_history_.end(), true));
  tracked_.stable = hits >= std::max(1, config_.stable_min_hits);
}

void GoalController::UpdatePoseTracker(
    const std::optional<ObjectTarget> &backboard,
    const GoalPoseObservation &pose) {
  const bool detected = backboard && pose.valid && std::isfinite(pose.x_m) &&
                        std::isfinite(pose.z_m) && pose.z_m > 0.0 &&
                        std::isfinite(pose.yaw_rad);
  pose_hit_history_.push_back(detected);
  while (static_cast<int>(pose_hit_history_.size()) >
         std::max(1, config_.stable_window)) pose_hit_history_.pop_front();
  fine_enter_history_.push_back(detected &&
                                pose.z_m <= config_.fine_adjust_start_z_m);
  while (static_cast<int>(fine_enter_history_.size()) >
         std::max(1, config_.stable_window)) fine_enter_history_.pop_front();
  if (detected) {
    pose_lost_count_ = 0;
    const double alpha = Clamp(config_.smooth_alpha, 0.0, 1.0);
    if (!has_pose_smoothed_) {
      tracked_pose_.x_m = pose.x_m;
      tracked_pose_.z_m = pose.z_m;
      tracked_pose_.yaw_rad = pose.yaw_rad;
      has_pose_smoothed_ = true;
    } else {
      tracked_pose_.x_m = (1.0 - alpha) * tracked_pose_.x_m + alpha * pose.x_m;
      tracked_pose_.z_m = (1.0 - alpha) * tracked_pose_.z_m + alpha * pose.z_m;
      tracked_pose_.yaw_rad = Wrap(tracked_pose_.yaw_rad +
          alpha * Wrap(pose.yaw_rad - tracked_pose_.yaw_rad));
    }
    tracked_pose_.visible = true;
    tracked_pose_.confidence = pose.confidence;
  } else {
    ++pose_lost_count_;
    tracked_pose_.visible = false;
  }
  const int hits = static_cast<int>(std::count(
      pose_hit_history_.begin(), pose_hit_history_.end(), true));
  tracked_pose_.stable = hits >= std::max(1, config_.stable_min_hits);
}

bool GoalController::PoseReadyForFineAdjust() const {
  const int hits = static_cast<int>(std::count(
      fine_enter_history_.begin(), fine_enter_history_.end(), true));
  return tracked_pose_.stable && tracked_pose_.visible &&
         hits >= std::max(1, config_.stable_min_hits);
}

double GoalController::ComputeShootYawRad(double x_m, double z_m,
                                         double yaw_rad) const {
  const double rim_x = x_m + config_.hoop_radius_m * std::sin(yaw_rad);
  const double rim_z = z_m - config_.hoop_radius_m * std::cos(yaw_rad);
  if (!std::isfinite(rim_x) || !std::isfinite(rim_z) ||
      std::hypot(rim_x, rim_z) <= kEpsilon) return 0.0;
  return Wrap(-std::atan2(rim_x, rim_z));
}

ActionRequest GoalController::FineAction(
    const GoalPoseObservation &pose) const {
  if (!pose.valid || !std::isfinite(pose.x_m) || !std::isfinite(pose.z_m) ||
      pose.z_m <= 0.0 || !std::isfinite(pose.yaw_rad)) return {};
  const double rim_x = pose.x_m +
      config_.hoop_radius_m * std::sin(pose.yaw_rad);
  const double rim_z = pose.z_m -
      config_.hoop_radius_m * std::cos(pose.yaw_rad);
  const double error = std::hypot(rim_x, rim_z) - config_.throwing_range_m;
  if (error > config_.position_tolerance_m)
    return Locomotion(MissionAction::kStepForwardHalf);
  if (error < -config_.position_tolerance_m)
    return Locomotion(MissionAction::kStepBack);
  const double yaw_deg =
      ComputeShootYawRad(pose.x_m, pose.z_m, pose.yaw_rad) * 180.0 / kPi;
  if (std::abs(yaw_deg) <= config_.shoot_yaw_limit_deg) {
    const double clamped = Clamp(yaw_deg, -180.0, 180.0);
    return Mission(MissionAction::kShoot,
                   static_cast<std::int16_t>(std::lround(clamped)));
  }
  return Locomotion(rim_x > 0.0 ? MissionAction::kRightSideStep
                                 : MissionAction::kLeftSideStep);
}

GoalResult GoalController::Compute(
    const std::optional<ObjectTarget> &, const std::optional<ObjectTarget> &backboard,
    const GoalPoseObservation &pose, int image_width, int image_height,
    double now_sec, bool line_reference_valid, const CameraFeedback &camera,
    const ActionExecutionFeedback &feedback) {
  UpdateGoalTracker(backboard, image_width, image_height);
  UpdatePoseTracker(backboard, pose);

  if (mode_ == GoalMode::kLineFollow && has_ball_ && goal_entry_armed_) {
    if (!line_reference_valid) {
      post_pickup_line_wait_active_ = false;
    } else {
      if (!post_pickup_line_wait_active_) {
        post_pickup_line_wait_active_ = true;
        post_pickup_line_wait_start_sec_ = now_sec;
      }
      camera_trigger_latched_ = camera_trigger_latched_ ||
          now_sec - post_pickup_line_wait_start_sec_ >=
              config_.post_pickup_wait_sec;
      if (camera_trigger_latched_ &&
          (!feedback.action_active || feedback.action_done)) {
        mode_ = GoalMode::kWaitCameraGoal;
        state_enter_sec_ = now_sec;
      }
    }
  }

  GoalResult result;
  result.active = mode_ != GoalMode::kLineFollow &&
                  mode_ != GoalMode::kHeadingRecovery;
  result.mode = mode_;
  result.tracked = tracked_;
  result.pose = tracked_pose_;
  result.shoot_yaw_rad = tracked_pose_.visible
                             ? ComputeShootYawRad(tracked_pose_.x_m,
                                                  tracked_pose_.z_m,
                                                  tracked_pose_.yaw_rad)
                             : shoot_yaw_rad_;
  switch (mode_) {
  case GoalMode::kLineFollow:
    return result;
  case GoalMode::kWaitCameraGoal:
    result.camera_request = CameraRequest::kGoal;
    if (camera.actual_mode == CameraMode::kGoal && camera.settled) {
      mode_ = GoalMode::kSearch;
      ClearTracking();
      result.mode = mode_;
      result.camera_request = CameraRequest::kNone;
      result.tracked = {};
      result.pose = {};
    } else if (now_sec - state_enter_sec_ >= camera_motion_timeout_sec_) {
      mode_ = GoalMode::kFailed;
      result.mode = mode_;
      result.camera_request = CameraRequest::kNone;
    }
    return result;
  case GoalMode::kSearch:
    if (PoseReadyForFineAdjust()) {
      mode_ = GoalMode::kFineAdjust;
      settle_until_sec_ = now_sec + config_.fine_settle_duration_sec;
      result.mode = mode_;
    } else if (tracked_.stable && tracked_.visible) {
      mode_ = GoalMode::kApproach;
      result.mode = mode_;
      const auto decision = SelectCruiseDecision(
          true, true, tracked_.u_norm - config_.target_u_norm,
          config_.approach_u_deadband);
      result.action = Locomotion(CruiseAction(decision.direction));
    }
    return result;
  case GoalMode::kApproach: {
    fine_trigger_latched_ = fine_trigger_latched_ || PoseReadyForFineAdjust();
    if (fine_trigger_latched_) {
      if (feedback.action_active && !feedback.action_done) return result;
      mode_ = GoalMode::kFineAdjust;
      settle_until_sec_ = now_sec + config_.fine_settle_duration_sec;
      result.mode = mode_;
      return result;
    }
    if (lost_count_ >= std::max(1, config_.lost_frames)) {
      mode_ = GoalMode::kSearch;
      ClearTracking();
      result.mode = mode_;
      return result;
    }
    const auto decision = SelectCruiseDecision(
        true, tracked_.visible, tracked_.u_norm - config_.target_u_norm,
        config_.approach_u_deadband);
    result.action = Locomotion(CruiseAction(decision.direction));
    return result;
  }
  case GoalMode::kFineAdjust:
    if (feedback.action_active && !feedback.action_done) return result;
    if (feedback.action_done) {
      settle_until_sec_ = now_sec + config_.fine_settle_duration_sec;
      return result;
    }
    if (now_sec + kEpsilon < settle_until_sec_) return result;
    if (pose_lost_count_ >= std::max(1, config_.lost_frames)) {
      mode_ = tracked_.visible ? GoalMode::kApproach : GoalMode::kSearch;
      result.mode = mode_;
      fine_trigger_latched_ = false;
      return result;
    }
    // Fine locomotion이 끝나고 settle된 뒤 현재 frame에서 새로 계산된 raw
    // RGB-D pose만 사용한다. motion 중 누적된 smoothing 이력은 다음 fine
    // action의 geometry에 섞지 않는다.
    result.action = FineAction(pose);
    if (result.action.action == MissionAction::kShoot) {
      shoot_yaw_rad_ = ComputeShootYawRad(pose.x_m, pose.z_m, pose.yaw_rad);
      mode_ = GoalMode::kShoot;
      result.mode = mode_;
      result.shoot_yaw_rad = shoot_yaw_rad_;
    }
    return result;
  case GoalMode::kShoot:
    result.action = Mission(MissionAction::kShoot,
                            static_cast<std::int16_t>(std::lround(
                                Clamp(shoot_yaw_rad_ * 180.0 / kPi,
                                      -180.0, 180.0))));
    if (feedback.action_done) {
      has_ball_ = false;
      ball_consumed_ = true;
      mode_ = GoalMode::kReturnCameraToLine;
      result.mode = mode_;
      result.action = {};
      result.camera_request = CameraRequest::kForward;
    }
    return result;
  case GoalMode::kReturnCameraToLine:
    result.camera_request = CameraRequest::kForward;
    if (camera.actual_mode == CameraMode::kForward && camera.settled) {
      mode_ = GoalMode::kHeadingRecovery;
      result.mode = mode_;
      result.active = false;
      result.camera_request = CameraRequest::kNone;
    }
    return result;
  case GoalMode::kHeadingRecovery:
    result.active = false;
    if (line_reference_valid) {
      mode_ = GoalMode::kLineFollow;
      result = {};
      result.mode = GoalMode::kLineFollow;
    }
    return result;
  case GoalMode::kFailed:
    return result;
  }
  return result;
}

void GoalController::ClearTracking() {
  hit_history_.clear();
  pose_hit_history_.clear();
  fine_enter_history_.clear();
  lost_count_ = 0;
  pose_lost_count_ = 0;
  has_smoothed_ = false;
  has_pose_smoothed_ = false;
  tracked_ = {};
  tracked_pose_ = {};
}

void GoalController::Reset() {
  mode_ = GoalMode::kLineFollow;
  state_enter_sec_ = 0.0;
  settle_until_sec_ = 0.0;
  shoot_yaw_rad_ = 0.0;
  has_ball_ = false;
  ball_consumed_ = false;
  goal_entry_armed_ = false;
  post_pickup_line_wait_active_ = false;
  camera_trigger_latched_ = false;
  fine_trigger_latched_ = false;
  ClearTracking();
}

} // namespace vision_core
