#include "vision_core/ball_controller.hpp"

#include "vision_core/config_loader.hpp"
#include "vision_core/cruise_selector.hpp"

#include <algorithm>
#include <cmath>

namespace vision_core {
namespace {
constexpr double kEpsilon = 1e-9;
double Denom(double value) { return std::max(value, 1e-6); }
ActionRequest Locomotion(MissionAction action, std::int16_t yaw = 0) {
  return {action, ActionCategory::kLocomotion, yaw, false};
}
ActionRequest Mission(MissionAction action) {
  return {action, ActionCategory::kMission, 0, false};
}
} // namespace

BallController::BallController() {
  const auto config = LoadDefaultAlgorithmConfig();
  config_ = config.ball;
  camera_motion_timeout_sec_ = config.camera_motion_timeout_sec;
}
BallController::BallController(const BallConfig &config,
                               double camera_motion_timeout_sec)
    : config_(config), camera_motion_timeout_sec_(camera_motion_timeout_sec) {}

const char *BallController::ModeName(BallMode mode) {
  switch (mode) {
  case BallMode::kLineFollow: return "LINE_FOLLOW";
  case BallMode::kApproachBall: return "BALL_APPROACH";
  case BallMode::kWaitCameraDown: return "BALL_CAMERA_DOWN";
  case BallMode::kFineAdjustForPickup: return "BALL_FINE";
  case BallMode::kPickupBall: return "BALL_PICKUP";
  case BallMode::kVerifyPickup: return "BALL_VERIFY";
  case BallMode::kVerifyPickupObservation: return "BALL_VERIFY_OBSERVE";
  case BallMode::kStandUpAfterPickup: return "BALL_STAND_UP";
  case BallMode::kPostPickupBackAway: return "BALL_BACK_AWAY";
  case BallMode::kReturnCameraToLine: return "BALL_CAMERA_FORWARD";
  case BallMode::kPostPickupLineRecovery: return "BALL_LINE_RECOVERY";
  case BallMode::kBallRecoveryForward: return "BALL_RECOVERY_FORWARD";
  case BallMode::kBallRecoveryDown: return "BALL_RECOVERY_DOWN";
  case BallMode::kFailed: return "BALL_FAILED";
  }
  return "UNKNOWN";
}

double BallController::Clamp(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

void BallController::UpdateTracker(const std::optional<ObjectTarget> &target,
                                   int image_width, int image_height) {
  const bool detected = target && image_width > 1 && image_height > 1;
  hit_history_.push_back(detected);
  while (static_cast<int>(hit_history_.size()) >
         std::max(1, config_.stable_window)) hit_history_.pop_front();

  if (detected) {
    lost_count_ = 0;
    const Point2 center = target->center_rectified
                              ? target->rectified_center_px
                              : target->center_px;
    TrackedBall observed;
    observed.visible = true;
    observed.center_px = center;
    observed.u_norm = Clamp(center.u / Denom(image_width), 0.0, 1.0);
    observed.v_norm = Clamp(center.v / Denom(image_height), 0.0, 1.0);
    observed.h_norm = Clamp(target->height_px / Denom(image_height), 0.0, 1.0);
    observed.area_norm = Clamp(target->area_px /
                                   Denom(static_cast<double>(image_width) *
                                         image_height),
                               0.0, 1.0);
    observed.confidence = target->confidence;
    const double alpha = Clamp(config_.smooth_alpha, 0.0, 1.0);
    if (!has_smoothed_) {
      tracked_ = observed;
      has_smoothed_ = true;
    } else {
      tracked_.visible = true;
      tracked_.center_px = center;
      tracked_.u_norm = (1.0 - alpha) * tracked_.u_norm + alpha * observed.u_norm;
      tracked_.v_norm = (1.0 - alpha) * tracked_.v_norm + alpha * observed.v_norm;
      tracked_.h_norm = (1.0 - alpha) * tracked_.h_norm + alpha * observed.h_norm;
      tracked_.area_norm = observed.area_norm;
      tracked_.confidence = observed.confidence;
    }
    last_seen_u_norm_ = tracked_.u_norm;
  } else {
    ++lost_count_;
    tracked_.visible = false;
  }
  const int hits = static_cast<int>(
      std::count(hit_history_.begin(), hit_history_.end(), true));
  tracked_.stable = hits >= std::max(1, config_.stable_min_hits);

  const bool upper = detected &&
      target->center_px.v / Denom(image_height) <= config_.upper_acquire_v_norm;
  upper_acquire_history_.push_back(upper);
  while (static_cast<int>(upper_acquire_history_.size()) >
         std::max(1, config_.stable_window)) upper_acquire_history_.pop_front();
  const bool tilt = detected &&
      target->center_px.v / Denom(image_height) >= config_.tilt_down_v_norm;
  tilt_history_.push_back(tilt);
  while (static_cast<int>(tilt_history_.size()) >
         std::max(1, config_.tilt_down_window)) tilt_history_.pop_front();
}

ActionRequest BallController::FarAction() const {
  const CruiseDecision decision = SelectCruiseDecision(
      true, tracked_.visible,
      tracked_.u_norm - config_.far_u_des_norm,
      config_.approach_u_deadband);
  return Locomotion(CruiseAction(decision.direction));
}

ActionRequest BallController::FineAction() const {
  if (!tracked_.visible) return {};
  const double u_error = tracked_.u_norm - config_.fine_target_u_norm;
  if (u_error < -config_.fine_u_deadband)
    return Locomotion(MissionAction::kLeftSideStep);
  if (u_error > config_.fine_u_deadband)
    return Locomotion(MissionAction::kRightSideStep);
  const double v_error = tracked_.v_norm - config_.fine_target_v_norm;
  if (v_error < -config_.fine_v_deadband)
    return Locomotion(MissionAction::kStepForwardHalf);
  if (v_error > config_.fine_v_deadband)
    return Locomotion(MissionAction::kStepBack);
  return Mission(MissionAction::kPickBall);
}

ActionRequest BallController::RecoveryAction() const {
  const double error = last_seen_u_norm_ - config_.far_u_des_norm;
  if (std::abs(error) <= config_.recovery_center_tolerance_norm) return {};
  return Locomotion(error < 0.0 ? MissionAction::kTurnLeft
                                : MissionAction::kTurnRight,
                    15);
}

BallResult BallController::Compute(
    const std::optional<ObjectTarget> &target, int image_width,
    int image_height, double now_sec, const CameraFeedback &camera,
    const ActionExecutionFeedback &feedback) {
  if (mode_ == BallMode::kLineFollow && has_ball_) {
    BallResult result;
    result.has_ball = true;
    return result;
  }
  if (mode_ == BallMode::kLineFollow && now_sec < ignore_ball_until_sec_) {
    BallResult result;
    result.has_ball = has_ball_;
    return result;
  }
  UpdateTracker(target, image_width, image_height);

  if (mode_ == BallMode::kLineFollow) {
    const int upper_hits = static_cast<int>(std::count(
        upper_acquire_history_.begin(), upper_acquire_history_.end(), true));
    if (tracked_.stable && tracked_.visible &&
        upper_hits >= std::max(1, config_.stable_min_hits)) {
      mode_ = BallMode::kApproachBall;
      state_enter_sec_ = now_sec;
      pickup_attempt_count_ = 0;
      pickup_failed_ = false;
      has_ball_ = false;
    }
  }

  BallResult result;
  result.active = mode_ != BallMode::kLineFollow &&
                  mode_ != BallMode::kPostPickupLineRecovery;
  result.mode = mode_;
  result.tracked = tracked_;
  result.has_ball = has_ball_;
  result.pickup_failed = pickup_failed_;
  result.pickup_attempt_count = pickup_attempt_count_;

  switch (mode_) {
  case BallMode::kLineFollow:
    return result;
  case BallMode::kApproachBall: {
    if (lost_count_ >= std::max(1, config_.lost_frames)) {
      mode_ = BallMode::kBallRecoveryForward;
      state_enter_sec_ = now_sec;
      recovery_visible_count_ = 0;
      result.mode = mode_;
      return result;
    }
    const int tilt_hits = static_cast<int>(
        std::count(tilt_history_.begin(), tilt_history_.end(), true));
    camera_trigger_latched_ = camera_trigger_latched_ ||
        tilt_hits >= std::max(1, config_.tilt_down_min_hits);
    if (camera_trigger_latched_) {
      if (feedback.action_active && !feedback.action_done) return result;
      mode_ = BallMode::kWaitCameraDown;
      state_enter_sec_ = now_sec;
      result.mode = mode_;
      result.camera_request = CameraRequest::kDown;
      return result;
    }
    result.action = FarAction();
    return result;
  }
  case BallMode::kWaitCameraDown:
    result.camera_request = CameraRequest::kDown;
    if (camera.actual_mode == CameraMode::kDown && camera.settled) {
      mode_ = BallMode::kFineAdjustForPickup;
      settle_until_sec_ = now_sec + config_.fine_settle_duration_sec;
      result.mode = mode_;
      result.camera_request = CameraRequest::kNone;
    } else if (now_sec - state_enter_sec_ >= camera_motion_timeout_sec_) {
      mode_ = BallMode::kFailed;
      result.mode = mode_;
      result.camera_request = CameraRequest::kNone;
    }
    return result;
  case BallMode::kFineAdjustForPickup:
    if (feedback.action_active && !feedback.action_done) return result;
    if (feedback.action_done) {
      settle_until_sec_ = now_sec + config_.fine_settle_duration_sec;
      return result;
    }
    if (now_sec + kEpsilon < settle_until_sec_) return result;
    if (lost_count_ >= std::max(1, config_.lost_frames)) {
      mode_ = BallMode::kBallRecoveryDown;
      state_enter_sec_ = now_sec;
      recovery_visible_count_ = 0;
      result.mode = mode_;
      return result;
    }
    result.action = FineAction();
    if (result.action.action == MissionAction::kPickBall) {
      ++pickup_attempt_count_;
      result.pickup_attempt_count = pickup_attempt_count_;
      mode_ = BallMode::kPickupBall;
      result.mode = mode_;
    }
    return result;
  case BallMode::kPickupBall:
    result.action = Mission(MissionAction::kPickBall);
    if (feedback.action_done) {
      mode_ = BallMode::kVerifyPickup;
      ClearTracking();
      result.mode = mode_;
      result.action = Mission(MissionAction::kRecatch);
    }
    return result;
  case BallMode::kVerifyPickup:
    result.action = Mission(MissionAction::kRecatch);
    if (feedback.action_done) {
      mode_ = BallMode::kVerifyPickupObservation;
      ClearTracking();
      result.mode = mode_;
      result.action = {};
    }
    return result;
  case BallMode::kVerifyPickupObservation:
    if (static_cast<int>(hit_history_.size()) <
        std::max(1, config_.stable_window))
      return result;
    if (static_cast<int>(std::count(hit_history_.begin(),
                                    hit_history_.end(), true)) <
        std::max(1, config_.stable_min_hits)) {
      has_ball_ = true;
      mode_ = BallMode::kStandUpAfterPickup;
      result.mode = mode_;
      result.has_ball = true;
      result.action = Mission(MissionAction::kDefaultPosition);
    } else {
      if (pickup_attempt_count_ < std::max(1, config_.pickup_max_attempts)) {
        // 공이 이동했을 수 있으므로 새 위치에서 다시 fine 정렬한다.
        // attempt는 실제 다음 PICK을 발행할 때만 증가한다.
        mode_ = BallMode::kFineAdjustForPickup;
        ClearTracking();
        settle_until_sec_ = now_sec + config_.fine_settle_duration_sec;
        result.mode = mode_;
      } else {
        pickup_failed_ = true;
        result.pickup_failed = true;
        mode_ = BallMode::kStandUpAfterPickup;
        result.mode = mode_;
        result.action = Mission(MissionAction::kDefaultPosition);
      }
    }
    return result;
  case BallMode::kStandUpAfterPickup:
    result.action = Mission(MissionAction::kDefaultPosition);
    if (feedback.action_done) {
      if (has_ball_) {
        mode_ = BallMode::kPostPickupBackAway;
        back_away_issued_ = false;
        result.mode = mode_;
        result.action = Locomotion(MissionAction::kStepBack);
        back_away_issued_ = true;
      } else {
        mode_ = BallMode::kReturnCameraToLine;
        result.mode = mode_;
        result.action = {};
        result.camera_request = CameraRequest::kForward;
      }
    }
    return result;
  case BallMode::kPostPickupBackAway:
    if (feedback.action_done && back_away_issued_) {
      mode_ = BallMode::kReturnCameraToLine;
      result.mode = mode_;
      result.camera_request = CameraRequest::kForward;
      return result;
    }
    result.action = Locomotion(MissionAction::kStepBack);
    back_away_issued_ = true;
    return result;
  case BallMode::kReturnCameraToLine:
    result.camera_request = CameraRequest::kForward;
    if (camera.actual_mode == CameraMode::kForward && camera.settled) {
      mode_ = BallMode::kPostPickupLineRecovery;
      result.mode = mode_;
      result.active = false;
      result.camera_request = CameraRequest::kNone;
    }
    return result;
  case BallMode::kPostPickupLineRecovery:
    result.active = false;
    return result;
  case BallMode::kBallRecoveryForward:
  case BallMode::kBallRecoveryDown: {
    if (feedback.action_active && !feedback.action_done) return result;
    if (feedback.action_done) {
      settle_until_sec_ = now_sec + config_.recovery_settle_duration_sec;
      return result;
    }
    if (now_sec + kEpsilon < settle_until_sec_) return result;
    if (tracked_.visible) ++recovery_visible_count_;
    else recovery_visible_count_ = 0;
    if (recovery_visible_count_ >=
        std::max(1, config_.recovery_reacquire_min_hits)) {
      mode_ = mode_ == BallMode::kBallRecoveryDown
                  ? BallMode::kFineAdjustForPickup
                  : BallMode::kApproachBall;
      result.mode = mode_;
      settle_until_sec_ = now_sec + config_.fine_settle_duration_sec;
      return result;
    }
    if (now_sec - state_enter_sec_ >= config_.recovery_timeout_sec) {
      mode_ = BallMode::kFailed;
      result.mode = mode_;
      return result;
    }
    result.action = RecoveryAction();
    return result;
  }
  case BallMode::kFailed:
    return result;
  }
  return result;
}

void BallController::ClearTracking() {
  hit_history_.clear();
  upper_acquire_history_.clear();
  tilt_history_.clear();
  lost_count_ = 0;
  recovery_visible_count_ = 0;
  has_smoothed_ = false;
  tracked_ = {};
}

void BallController::ResetToLine(bool clear_ignore) {
  mode_ = BallMode::kLineFollow;
  state_enter_sec_ = 0.0;
  settle_until_sec_ = 0.0;
  camera_trigger_latched_ = false;
  back_away_issued_ = false;
  if (clear_ignore) ignore_ball_until_sec_ = 0.0;
  ClearTracking();
}

void BallController::ClearEntryEvidence() {
  if (mode_ == BallMode::kLineFollow) ClearTracking();
}

void BallController::CompleteLineRecovery(double now_sec) {
  if (mode_ != BallMode::kPostPickupLineRecovery) return;
  ignore_ball_until_sec_ = now_sec + config_.ball_ignore_duration_sec;
  ResetToLine(false);
}

void BallController::Reset() {
  has_ball_ = false;
  pickup_failed_ = false;
  pickup_attempt_count_ = 0;
  ResetToLine(true);
}

} // namespace vision_core
