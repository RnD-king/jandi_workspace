#include "vision_core/goal_controller.hpp"

#include "vision_core/config_loader.hpp"
#include "vision_core/cruise_selector.hpp"
#include "vision_core/object_observation_window.hpp"

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
  while (angle > kPi)
    angle -= 2.0 * kPi;
  while (angle < -kPi)
    angle += 2.0 * kPi;
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
      left_depth_m <= 0.0 || right_depth_m <= 0.0)
    return {};
  const double left_x =
      (left_u_px - intrinsics.cx) * left_depth_m / intrinsics.fx;
  const double right_x =
      (right_u_px - intrinsics.cx) * right_depth_m / intrinsics.fx;
  const double width = right_x - left_x;
  if (!std::isfinite(width) || width <= kEpsilon)
    return {};
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
  GoalPoseObservation result =
      EstimateGoalPoseFromEdgeDepths(left_u_px, left_depth_m, right_u_px,
                                     right_depth_m, intrinsics, confidence);
  if (!result.valid || !std::isfinite(center_u_px) ||
      !std::isfinite(center_depth_m) || center_depth_m <= 0.0 ||
      !std::isfinite(intrinsics.fx) || std::abs(intrinsics.fx) <= kEpsilon)
    return {};
  result.x_m = (center_u_px - intrinsics.cx) * center_depth_m / intrinsics.fx;
  result.z_m = center_depth_m;
  return result;
}

const char *GoalController::ModeName(GoalMode mode) {
  switch (mode) {
  case GoalMode::kLineFollow:
    return "LINE_FOLLOW";
  case GoalMode::kWaitCameraGoal:
    return "GOAL_CAMERA";
  case GoalMode::kSearch:
    return "GOAL_SEARCH";
  case GoalMode::kApproach:
    return "GOAL_APPROACH";
  case GoalMode::kFineAdjust:
    return "GOAL_FINE";
  case GoalMode::kShoot:
    return "GOAL_SHOOT";
  case GoalMode::kReturnCameraToLine:
    return "GOAL_CAMERA_FORWARD";
  case GoalMode::kHeadingRecovery:
    return "GOAL_LINE_RECOVERY";
  case GoalMode::kFailed:
    return "GOAL_FAILED";
  case GoalMode::kRecoverySearch:
    return "GOAL_RECOVERY_SEARCH";
  case GoalMode::kForwardApproach:
    return "GOAL_FORWARD_APPROACH";
  case GoalMode::kCoarseAdjust:
    return "GOAL_CLOSE_WALK";
  case GoalMode::kWaitFinePose:
    return "GOAL_TO_FINE_FEET";
  case GoalMode::kFineRecovery:
    return "GOAL_FINE_RECOVERY";
  case GoalMode::kCoarseRecovery:
    return "GOAL_CLOSE_RECOVERY";
  case GoalMode::kPostShootBackAway:
    return "GOAL_BACK_AWAY";
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

void GoalController::StartForwardTrial() {
  SetHasBall(true);
  goal_entry_armed_ = true;
  mode_ = GoalMode::kForwardApproach;
  ClearTracking();
}

void GoalController::SetHasBall(bool has_ball) {
  if (!has_ball) {
    has_ball_ = false;
    goal_entry_armed_ = false;
    post_pickup_line_wait_active_ = false;
    if (mode_ == GoalMode::kLineFollow)
      ball_consumed_ = false;
    return;
  }
  if (!ball_consumed_)
    has_ball_ = true;
}

void GoalController::UpdateBallState(const BallResult &ball) {
  SetHasBall(ball.has_ball);
  goal_entry_armed_ =
      has_ball_ && !ball_consumed_ && ball.mode == BallMode::kLineFollow;
  if (!goal_entry_armed_)
    post_pickup_line_wait_active_ = false;
}

void GoalController::UpdateGoalTracker(
    const std::optional<ObjectTarget> &target, int image_width,
    int image_height) {
  const bool detected = target && image_width > 1 && image_height > 1;
  hit_history_.push_back(detected);
  while (static_cast<int>(hit_history_.size()) >
         std::max(1, config_.stable_window))
    hit_history_.pop_front();
  if (detected) {
    lost_count_ = 0;
    const Point2 center = target->center_rectified ? target->rectified_center_px
                                                   : target->center_px;
    TrackedGoal observed;
    observed.visible = true;
    observed.center_px = center;
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
      tracked_.center_px.u =
          (1.0 - alpha) * tracked_.center_px.u + alpha * observed.center_px.u;
      tracked_.center_px.v =
          (1.0 - alpha) * tracked_.center_px.v + alpha * observed.center_px.v;
      tracked_.u_norm =
          (1.0 - alpha) * tracked_.u_norm + alpha * observed.u_norm;
      tracked_.v_norm =
          (1.0 - alpha) * tracked_.v_norm + alpha * observed.v_norm;
      tracked_.h_norm = observed.h_norm;
      tracked_.confidence = observed.confidence;
    }
    last_seen_u_px_ = tracked_.center_px.u;
  } else {
    ++lost_count_;
    tracked_.visible = false;
  }
  const int hits = static_cast<int>(
      std::count(hit_history_.begin(), hit_history_.end(), true));
  tracked_.stable = hits >= std::max(1, config_.stable_min_hits);
  if (detected && tracked_.stable && mode_ != GoalMode::kLineFollow)
    seen_after_camera_goal_ = true;
}

void GoalController::UpdatePoseTracker(
    const std::optional<ObjectTarget> &backboard,
    const GoalPoseObservation &pose) {
  const bool detected = backboard && pose.valid && std::isfinite(pose.x_m) &&
                        std::isfinite(pose.z_m) && pose.z_m > 0.0 &&
                        std::isfinite(pose.yaw_rad);
  if (mode_ == GoalMode::kFineAdjust) {
    if (fine_pose_mean_pending_) {
      fine_pose_window_.clear();
      fine_pose_is_preaveraged_ = detected;
      fine_pose_mean_pending_ = false;
    }
    if (!fine_pose_is_preaveraged_ || fine_pose_window_.empty()) {
      fine_pose_window_.push_back(
          detected ? std::optional<GoalPoseObservation>{pose} : std::nullopt);
      while (fine_pose_window_.size() >
             static_cast<std::size_t>(std::max(1, config_.stable_window)))
        fine_pose_window_.pop_front();
    }
  }
  pose_hit_history_.push_back(detected);
  while (static_cast<int>(pose_hit_history_.size()) >
         std::max(1, config_.stable_window))
    pose_hit_history_.pop_front();
  fine_enter_history_.push_back(detected &&
                                pose.z_m <= config_.fine_adjust_start_z_m);
  while (static_cast<int>(fine_enter_history_.size()) >
         std::max(1, config_.stable_window))
    fine_enter_history_.pop_front();
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
      tracked_pose_.yaw_rad =
          Wrap(tracked_pose_.yaw_rad +
               alpha * Wrap(pose.yaw_rad - tracked_pose_.yaw_rad));
    }
    tracked_pose_.visible = true;
    tracked_pose_.confidence = pose.confidence;
  } else {
    ++pose_lost_count_;
    tracked_pose_.visible = false;
  }
  const int hits = static_cast<int>(
      std::count(pose_hit_history_.begin(), pose_hit_history_.end(), true));
  tracked_pose_.stable = hits >= std::max(1, config_.stable_min_hits);
}

bool GoalController::PoseReadyForFineAdjust() const {
  const int hits = static_cast<int>(
      std::count(fine_enter_history_.begin(), fine_enter_history_.end(), true));
  return tracked_pose_.stable && tracked_pose_.visible &&
         hits >= std::max(1, config_.stable_min_hits);
}

std::optional<GoalPoseObservation> GoalController::FinePoseMean() const {
  if (fine_pose_is_preaveraged_ && !fine_pose_window_.empty())
    return fine_pose_window_.front();
  if (fine_pose_window_.size() <
      static_cast<std::size_t>(std::max(1, config_.stable_window)))
    return std::nullopt;
  double x = 0.0, z = 0.0, sin_yaw = 0.0, cos_yaw = 0.0, confidence = 0.0;
  int count = 0;
  for (const auto &sample : fine_pose_window_) {
    if (!sample)
      continue;
    x += sample->x_m;
    z += sample->z_m;
    sin_yaw += std::sin(sample->yaw_rad);
    cos_yaw += std::cos(sample->yaw_rad);
    confidence += sample->confidence;
    ++count;
  }
  if (count < std::max(1, config_.stable_min_hits))
    return std::nullopt;
  const double divisor = static_cast<double>(count);
  return GoalPoseObservation{true, x / divisor, z / divisor,
                             std::atan2(sin_yaw, cos_yaw),
                             confidence / divisor};
}

double GoalController::ComputeShootYawRad(double x_m, double z_m,
                                          double yaw_rad) const {
  const double rim_x = x_m + config_.hoop_radius_m * std::sin(yaw_rad);
  const double rim_z = z_m - config_.hoop_radius_m * std::cos(yaw_rad);
  if (!std::isfinite(rim_x) || !std::isfinite(rim_z) ||
      std::hypot(rim_x, rim_z) <= kEpsilon)
    return 0.0;
  return Wrap(-std::atan2(rim_x, rim_z));
}

ActionRequest
GoalController::FineAction(const GoalPoseObservation &pose) const {
  if (!pose.valid || !std::isfinite(pose.x_m) || !std::isfinite(pose.z_m) ||
      pose.z_m <= 0.0 || !std::isfinite(pose.yaw_rad))
    return {};
  const double rim_x =
      pose.x_m + config_.hoop_radius_m * std::sin(pose.yaw_rad);
  const double rim_z =
      pose.z_m - config_.hoop_radius_m * std::cos(pose.yaw_rad);
  const double error = std::hypot(rim_x, rim_z) - config_.throwing_range_m;
  if (error > config_.position_tolerance_m) {
    return Locomotion(MissionAction::kStepForwardHalf);
  }
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
    const std::optional<ObjectTarget> &,
    const std::optional<ObjectTarget> &backboard,
    const GoalPoseObservation &pose, int image_width, int image_height,
    double now_sec, bool line_reference_valid, const CameraFeedback &camera,
    const ActionExecutionFeedback &feedback, bool allow_new_body_actions,
    std::optional<double> center_depth_m, double post_pickup_walk_sec) {
  const bool pose_valid = pose.valid && std::isfinite(pose.x_m) &&
                          std::isfinite(pose.z_m) && pose.z_m > 0.0 &&
                          std::isfinite(pose.yaw_rad);
  // Intermediate camera angles must not add object evidence to either tracker.
  const bool camera_switching = mode_ == GoalMode::kWaitCameraGoal ||
                                mode_ == GoalMode::kReturnCameraToLine;
  if (mode_ == GoalMode::kFineAdjust && feedback.action_done)
    ClearTracking();
  if (!camera_switching &&
      !(mode_ == GoalMode::kFineAdjust &&
        (feedback.action_active || feedback.action_done))) {
    UpdateGoalTracker(backboard, image_width, image_height);
    UpdatePoseTracker(backboard, pose);
  }

  if (mode_ == GoalMode::kLineFollow && has_ball_ && goal_entry_armed_ &&
      post_pickup_walk_sec + kEpsilon >= config_.post_pickup_wait_sec) {
    // Lock GOAL now to stop new LINE reservations; drain accepted walks first.
    mode_ = GoalMode::kWaitCameraGoal;
    camera_request_started_ = false;
  }

  GoalResult result;
  result.active =
      mode_ != GoalMode::kLineFollow && mode_ != GoalMode::kHeadingRecovery;
  result.mode = mode_;
  result.tracked = tracked_;
  result.pose = tracked_pose_;
  result.shoot_yaw_rad =
      tracked_pose_.visible
          ? ComputeShootYawRad(tracked_pose_.x_m, tracked_pose_.z_m,
                               tracked_pose_.yaw_rad)
          : shoot_yaw_rad_;
  switch (mode_) {
  case GoalMode::kLineFollow:
    return result;
  case GoalMode::kForwardApproach: {
    const double distance =
        center_depth_m.value_or(pose.valid ? pose.z_m : 0.0);
    const bool current_valid = tracked_.visible && tracked_.stable &&
                               std::isfinite(distance) && distance > 0.0;
    camera_trigger_latched_ =
        camera_trigger_latched_ ||
        (current_valid && distance <= config_.camera_goal_start_z_m);
    if (camera_trigger_latched_) {
      if (feedback.action_active && !feedback.action_done)
        return result;
      mode_ = GoalMode::kWaitCameraGoal;
      state_enter_sec_ = now_sec;
      result.mode = mode_;
      result.camera_request = CameraRequest::kGoal;
      return result;
    }
    if (!current_valid)
      return result;
    const auto d = SelectCruiseDecision(
        true, true, tracked_.center_px.u - config_.target_u_px,
        config_.approach_u_deadband_px);
    result.action = Locomotion(
        ApproachAction(d.direction, distance <= config_.approach_short_z_m));
    return result;
  }
  case GoalMode::kWaitCameraGoal:
    if (feedback.action_active && !feedback.action_done)
      return result;
    if (!camera_request_started_) {
      camera_request_started_ = true;
      state_enter_sec_ = now_sec;
    }
    result.camera_request = CameraRequest::kGoal;
    if (camera.actual_mode == CameraMode::kGoal && camera.settled) {
      mode_ = GoalMode::kSearch;
      ClearTracking();
      last_seen_u_px_ = config_.target_u_px;
      seen_after_camera_goal_ = false;
      recovery_visible_count_ = 0;
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
  case GoalMode::kSearch: {
    // No past GOAL-view sighting, or a centered last sighting: wait in place.
    if (!tracked_.visible && seen_after_camera_goal_ &&
        lost_count_ >= std::max(1, config_.lost_frames)) {
      mode_ = GoalMode::kRecoverySearch;
      stationary_recovery_.Begin(now_sec);
      state_enter_sec_ = now_sec;
      recovery_visible_count_ = 0;
      settle_until_sec_ = now_sec;
      result.mode = mode_;
      return result;
    }
    const bool center_near = center_depth_m && std::isfinite(*center_depth_m) &&
                             *center_depth_m > 0 &&
                             *center_depth_m <= config_.fine_adjust_start_z_m;
    if (PoseReadyForFineAdjust() ||
        (tracked_.stable && tracked_.visible && center_near)) {
      mode_ = GoalMode::kCoarseAdjust;
      settle_until_sec_ = now_sec + config_.fine_settle_duration_sec;
      result.mode = mode_;
    } else if (tracked_.stable && tracked_.visible) {
      mode_ = GoalMode::kApproach;
      result.mode = mode_;
      const auto decision = SelectCruiseDecision(
          true, true, tracked_.center_px.u - config_.target_u_px,
          config_.approach_u_deadband_px);
      result.action = Locomotion(ApproachAction(
          decision.direction, center_depth_m.value_or(tracked_pose_.z_m) > 0 &&
                                  center_depth_m.value_or(tracked_pose_.z_m) <=
                                      config_.approach_short_z_m));
    }
    return result;
  }
  case GoalMode::kApproach: {
    const bool center_near = center_depth_m && std::isfinite(*center_depth_m) &&
                             *center_depth_m > 0 &&
                             *center_depth_m <= config_.fine_adjust_start_z_m;
    fine_trigger_latched_ =
        fine_trigger_latched_ || PoseReadyForFineAdjust() ||
        (tracked_.visible && tracked_.stable && center_near);
    if (fine_trigger_latched_) {
      if (feedback.action_active && !feedback.action_done)
        return result;
      mode_ = GoalMode::kCoarseAdjust;
      settle_until_sec_ = now_sec + config_.fine_settle_duration_sec;
      result.mode = mode_;
      return result;
    }
    if (lost_count_ >= std::max(1, config_.lost_frames)) {
      mode_ = GoalMode::kRecoverySearch;
      stationary_recovery_.Begin(now_sec);
      {
        state_enter_sec_ = now_sec;
        recovery_visible_count_ = 0;
        settle_until_sec_ = now_sec;
      }
      ClearTracking();
      result.mode = mode_;
      return result;
    }
    const auto decision = SelectCruiseDecision(
        true, tracked_.visible, tracked_.center_px.u - config_.target_u_px,
        config_.approach_u_deadband_px);
    result.action = Locomotion(ApproachAction(
        decision.direction, center_depth_m.value_or(tracked_pose_.z_m) > 0 &&
                                center_depth_m.value_or(tracked_pose_.z_m) <=
                                    config_.approach_short_z_m));
    return result;
  }
  case GoalMode::kCoarseAdjust: {
    if (feedback.action_active && !feedback.action_done)
      return result;
    if (feedback.action_done) {
      settle_until_sec_ = now_sec + config_.fine_settle_duration_sec;
      return result;
    }
    if (now_sec < settle_until_sec_)
      return result;
    if (!pose_valid) {
      if (pose_lost_count_ >= config_.lost_frames) {
        mode_ = GoalMode::kCoarseRecovery;
        ClearTracking();
        stationary_recovery_.Begin(now_sec);
        result.mode = mode_;
      }
      return result;
    }
    const double rim_x =
        pose.x_m + config_.hoop_radius_m * std::sin(pose.yaw_rad);
    const double rim_z =
        pose.z_m - config_.hoop_radius_m * std::cos(pose.yaw_rad);
    const double error = std::hypot(rim_x, rim_z) - config_.throwing_range_m;
    result.fine_decision_pose = pose;
    result.fine_decision_pose_used = true;
    if (error >= config_.fine_one_step_range_error_m &&
        error > config_.position_tolerance_m) {
      result.action = Locomotion(error >= config_.fine_two_step_range_error_m
                                     ? MissionAction::kStepForwardTwo
                                     : MissionAction::kStepForwardOne);
    } else {
      mode_ = GoalMode::kWaitFinePose;
      result.mode = mode_;
      result.action = Locomotion(MissionAction::kDefaultPoseMode);
    }
    return result;
  }
  case GoalMode::kWaitFinePose:
    if (feedback.action_done) {
      mode_ = GoalMode::kFineAdjust;
      ClearTracking();
      settle_until_sec_ = now_sec + config_.fine_settle_duration_sec;
      result.mode = mode_;
      result.tracked = {};
      result.pose = {};
      return result;
    }
    result.action = Locomotion(MissionAction::kDefaultPoseMode);
    return result;
  case GoalMode::kCoarseRecovery:
  case GoalMode::kFineRecovery: {
    const bool was_fine = mode_ == GoalMode::kFineRecovery;
    const auto status = stationary_recovery_.Observe(
        backboard.has_value() && pose_valid, now_sec, config_.stable_window,
        config_.stable_min_hits, config_.recovery_observation_sec,
        config_.recovery_max_observations);
    if (status == StationaryObjectRecovery::Result::kFound) {
      mode_ = was_fine ? GoalMode::kFineAdjust : GoalMode::kCoarseAdjust;
      settle_until_sec_ = now_sec + config_.fine_settle_duration_sec;
    } else if (status == StationaryObjectRecovery::Result::kFailed)
      mode_ = GoalMode::kFailed;
    else if (status == StationaryObjectRecovery::Result::kRetry)
      ClearTracking();
    result.mode = mode_;
    return result;
  }
  case GoalMode::kFineAdjust: {
    if (feedback.action_active && !feedback.action_done)
      return result;
    if (feedback.action_done) {
      settle_until_sec_ = now_sec + config_.fine_settle_duration_sec;
      return result;
    }
    if (now_sec + kEpsilon < settle_until_sec_)
      return result;
    if (pose_lost_count_ >= std::max(1, config_.lost_frames)) {
      mode_ = GoalMode::kFineRecovery;
      ClearTracking();
      stationary_recovery_.Begin(now_sec);
      result.mode = mode_;
      return result;
    }
    const auto mean_pose = FinePoseMean();
    if (!mean_pose || !tracked_.visible || !tracked_pose_.visible)
      return result;
    // Only stationary observations since the last motion are eligible here.
    result.fine_decision_pose = *mean_pose;
    result.fine_decision_pose_used = true;
    result.action = FineAction(*mean_pose);
    if (test_execute_requires_stable_ &&
        result.action.action == MissionAction::kShoot &&
        !ExecutionObservationReady())
      result.action = {};
    if (allow_new_body_actions &&
        result.action.action == MissionAction::kShoot) {
      shoot_yaw_rad_ = ComputeShootYawRad(mean_pose->x_m, mean_pose->z_m,
                                          mean_pose->yaw_rad);
      mode_ = GoalMode::kShoot;
      result.mode = mode_;
      result.shoot_yaw_rad = shoot_yaw_rad_;
    }
    return result;
  }
  case GoalMode::kShoot:
    result.action = Mission(MissionAction::kShoot,
                            static_cast<std::int16_t>(std::lround(Clamp(
                                shoot_yaw_rad_ * 180.0 / kPi, -180.0, 180.0))));
    if (feedback.action_done) {
      has_ball_ = false;
      ball_consumed_ = true;
      mode_ = GoalMode::kPostShootBackAway;
      completed_back_steps_ = 0;
      back_away_issued_ = false;
      result.mode = mode_;
      result.action = {};
    }
    return result;
  case GoalMode::kPostShootBackAway:
    if (feedback.action_done && back_away_issued_) {
      ++completed_back_steps_;
      back_away_issued_ = false;
    }
    if (completed_back_steps_ >= config_.post_shoot_back_steps) {
      mode_ = GoalMode::kReturnCameraToLine;
      result.mode = mode_;
      result.action = {};
      result.camera_request = CameraRequest::kForward;
      return result;
    }
    result.action = Locomotion(MissionAction::kStepBack);
    if (allow_new_body_actions)
      back_away_issued_ = true;
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
  case GoalMode::kRecoverySearch: {
    if (feedback.action_active && !feedback.action_done)
      return result;
    const auto status = stationary_recovery_.Observe(
        backboard.has_value(), now_sec, config_.stable_window,
        config_.stable_min_hits, config_.recovery_observation_sec,
        config_.recovery_max_observations);
    if (status == StationaryObjectRecovery::Result::kFound)
      mode_ = GoalMode::kSearch;
    else if (status == StationaryObjectRecovery::Result::kFailed)
      mode_ = GoalMode::kFailed;
    else if (status == StationaryObjectRecovery::Result::kRetry)
      ClearTracking();
    result.mode = mode_;
    return result;
  }
  case GoalMode::kFailed:
    return result;
  }
  return result;
}

ActionRequest GoalController::SearchRecoveryAction() const {
  const double error = last_seen_u_px_ - config_.target_u_px;
  if (std::abs(error) <= config_.recovery_center_tolerance_px)
    return {};
  const MissionAction turn =
      error < 0.0 ? MissionAction::kTurnLeft : MissionAction::kTurnRight;
  return {turn, ActionCategory::kLocomotion, 15, false};
}

void GoalController::ObserveOnly(const std::optional<ObjectTarget> &target,
                                 const GoalPoseObservation &pose, int width,
                                 int height) {
  if (mode_ != GoalMode::kWaitCameraGoal &&
      mode_ != GoalMode::kReturnCameraToLine) {
    UpdateGoalTracker(target, width, height);
    UpdatePoseTracker(target, pose);
  }
}

void GoalController::ClearObservationHistory() { ClearTracking(); }

void GoalController::ClearTracking() {
  hit_history_.clear();
  pose_hit_history_.clear();
  fine_enter_history_.clear();
  fine_pose_window_.clear();
  fine_pose_mean_pending_ = false;
  fine_pose_is_preaveraged_ = false;
  lost_count_ = 0;
  pose_lost_count_ = 0;
  has_smoothed_ = false;
  has_pose_smoothed_ = false;
  tracked_ = {};
  tracked_pose_ = {};
}

void GoalController::Reset() {
  test_execute_requires_stable_ = false;
  stationary_recovery_.Begin(0.0);
  completed_back_steps_ = 0;
  back_away_issued_ = false;
  mode_ = GoalMode::kLineFollow;
  state_enter_sec_ = 0.0;
  settle_until_sec_ = 0.0;
  shoot_yaw_rad_ = 0.0;
  has_ball_ = false;
  ball_consumed_ = false;
  goal_entry_armed_ = false;
  post_pickup_line_wait_active_ = false;
  camera_trigger_latched_ = false;
  camera_request_started_ = false;
  fine_trigger_latched_ = false;
  last_seen_u_px_ = config_.target_u_px;
  seen_after_camera_goal_ = false;
  recovery_visible_count_ = 0;
  ClearTracking();
}

} // namespace vision_core
