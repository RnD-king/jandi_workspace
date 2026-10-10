#include "vision_core/hurdle_controller.hpp"

#include "vision_core/config_loader.hpp"

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

HurdleController::HurdleController() {
  const auto config = LoadDefaultAlgorithmConfig();
  config_ = config.hurdle;
  camera_motion_timeout_sec_ = config.camera_motion_timeout_sec;
}
HurdleController::HurdleController(const HurdleConfig &config,
                                   double camera_motion_timeout_sec)
    : config_(config), camera_motion_timeout_sec_(camera_motion_timeout_sec) {}

const char *HurdleController::ModeName(HurdleMode mode) {
  switch (mode) {
  case HurdleMode::kLineFollow: return "LINE_FOLLOW";
  case HurdleMode::kApproach: return "HURDLE_APPROACH";
  case HurdleMode::kWaitCameraDown: return "HURDLE_CAMERA_DOWN";
  case HurdleMode::kContactWalk: return "HURDLE_CONTACT";
  case HurdleMode::kCross: return "HURDLE_CROSS";
  case HurdleMode::kReturnCameraToLine: return "HURDLE_CAMERA_FORWARD";
  case HurdleMode::kRecoveryForward: return "HURDLE_RECOVERY_FORWARD";
  case HurdleMode::kFailed: return "HURDLE_FAILED";
  }
  return "UNKNOWN";
}

double HurdleController::Clamp(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

void HurdleController::UpdateTracker(const std::optional<ObjectTarget> &target,
                                     int image_width, int image_height) {
  const bool detected = target && image_width > 1 && image_height > 1;
  hit_history_.push_back(detected);
  while (static_cast<int>(hit_history_.size()) >
         std::max(1, config_.stable_window)) hit_history_.pop_front();
  bool acquire = false;
  bool close = false;
  if (detected) {
    lost_count_ = 0;
    const Point2 center = target->center_rectified
                              ? target->rectified_center_px
                              : target->center_px;
    TrackedHurdle observed;
    observed.visible = true;
    observed.u_norm = Clamp(center.u / Denom(image_width), 0.0, 1.0);
    observed.v_norm = Clamp(center.v / Denom(image_height), 0.0, 1.0);
    observed.h_norm = Clamp(target->height_px / Denom(image_height), 0.0, 1.0);
    observed.bottom_norm = Clamp((target->box_px.y + target->box_px.height) /
                                     Denom(image_height),
                                 0.0, 1.0);
    observed.confidence = target->confidence;
    const double raw_v = target->center_px.v / Denom(image_height);
    acquire = raw_v >= config_.acquire_min_v_norm;
    close = raw_v >= config_.tilt_trigger_v_norm;
    last_seen_u_norm_ = observed.u_norm;
    const double alpha = Clamp(config_.smooth_alpha, 0.0, 1.0);
    if (!has_smoothed_) {
      tracked_ = observed;
      has_smoothed_ = true;
    } else {
      tracked_.visible = true;
      tracked_.u_norm = (1.0 - alpha) * tracked_.u_norm + alpha * observed.u_norm;
      tracked_.v_norm = (1.0 - alpha) * tracked_.v_norm + alpha * observed.v_norm;
      tracked_.h_norm = observed.h_norm;
      tracked_.bottom_norm = observed.bottom_norm;
      tracked_.confidence = observed.confidence;
    }
  } else {
    ++lost_count_;
    tracked_.visible = false;
  }
  const int hits = static_cast<int>(
      std::count(hit_history_.begin(), hit_history_.end(), true));
  tracked_.stable = hits >= std::max(1, config_.stable_min_hits);
  acquire_history_.push_back(acquire);
  while (static_cast<int>(acquire_history_.size()) >
         std::max(1, config_.stable_window)) acquire_history_.pop_front();
  tilt_history_.push_back(close);
  while (static_cast<int>(tilt_history_.size()) >
         std::max(1, config_.tilt_trigger_window)) tilt_history_.pop_front();
}

ActionRequest HurdleController::RecoveryAction() const {
  const double error = last_seen_u_norm_ - 0.5;
  if (std::abs(error) <= config_.recovery_center_tolerance_norm) return {};
  return Locomotion(error < 0.0 ? MissionAction::kTurnLeft
                                : MissionAction::kTurnRight,
                    15);
}

HurdleResult HurdleController::Compute(
    const std::optional<ObjectTarget> &target, int image_width,
    int image_height, double now_sec, const CameraFeedback &camera,
    const ActionExecutionFeedback &feedback) {
  if (mode_ == HurdleMode::kLineFollow && now_sec < ignore_until_sec_)
    return {};
  // No tracking/near-evidence from the moving camera view.
  if (mode_ != HurdleMode::kWaitCameraDown &&
      mode_ != HurdleMode::kReturnCameraToLine)
    UpdateTracker(target, image_width, image_height);
  if (mode_ == HurdleMode::kLineFollow) {
    const int acquire_hits = static_cast<int>(std::count(
        acquire_history_.begin(), acquire_history_.end(), true));
    if (tracked_.stable && tracked_.visible &&
        acquire_hits >= std::max(1, config_.stable_min_hits)) {
      mode_ = HurdleMode::kApproach;
      state_enter_sec_ = now_sec;
    }
  }

  HurdleResult result;
  result.active = mode_ != HurdleMode::kLineFollow;
  result.mode = mode_;
  result.tracked = tracked_;
  switch (mode_) {
  case HurdleMode::kLineFollow:
    return result;
  case HurdleMode::kApproach: {
    if (lost_count_ >= std::max(1, config_.lost_frames)) {
      mode_ = HurdleMode::kRecoveryForward;
      state_enter_sec_ = now_sec;
      recovery_visible_count_ = 0;
      result.mode = mode_;
      return result;
    }
    const int close_hits = static_cast<int>(
        std::count(tilt_history_.begin(), tilt_history_.end(), true));
    close_trigger_latched_ = close_trigger_latched_ ||
        close_hits >= std::max(1, config_.tilt_trigger_min_hits);
    if (close_trigger_latched_) {
      if (feedback.action_active && !feedback.action_done) return result;
      mode_ = HurdleMode::kWaitCameraDown;
      state_enter_sec_ = now_sec;
      result.mode = mode_;
      result.camera_request = CameraRequest::kDown;
      return result;
    }
    result.action = Locomotion(MissionAction::kStepForwardFive);
    return result;
  }
  case HurdleMode::kWaitCameraDown:
    result.camera_request = CameraRequest::kDown;
    if (camera.actual_mode == CameraMode::kDown && camera.settled) {
      ClearTracking();  // Intermediate camera observations are discarded.
      last_seen_u_norm_ = 0.5;
      mode_ = HurdleMode::kContactWalk;
      result.mode = mode_;
      result.tracked = {};
      result.camera_request = CameraRequest::kNone;
      // CONTACT_WALK(20)은 protocol에만 예약한다. executor mapping 전까지 10.
      result.action = Locomotion(MissionAction::kStepForwardOne);
    } else if (now_sec - state_enter_sec_ >= camera_motion_timeout_sec_) {
      mode_ = HurdleMode::kFailed;
      result.mode = mode_;
      result.camera_request = CameraRequest::kNone;
    }
    return result;
  case HurdleMode::kContactWalk:
    result.action = Locomotion(MissionAction::kStepForwardOne);
    if (feedback.action_done) {
      mode_ = HurdleMode::kCross;
      result.mode = mode_;
      result.action = Mission(MissionAction::kHurdle);
    }
    return result;
  case HurdleMode::kCross:
    result.action = Mission(MissionAction::kHurdle);
    if (feedback.action_done) {
      mode_ = HurdleMode::kReturnCameraToLine;
      result.mode = mode_;
      result.action = {};
      result.camera_request = CameraRequest::kForward;
    }
    return result;
  case HurdleMode::kReturnCameraToLine:
    result.camera_request = CameraRequest::kForward;
    if (camera.actual_mode == CameraMode::kForward && camera.settled) {
      ignore_until_sec_ = now_sec + config_.hurdle_ignore_duration_sec;
      ResetToLine(false);
      result = {};
      result.mode = HurdleMode::kLineFollow;
    }
    return result;
  case HurdleMode::kRecoveryForward:
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
      mode_ = HurdleMode::kApproach;
      result.mode = mode_;
      return result;
    }
    if (now_sec - state_enter_sec_ >= config_.recovery_timeout_sec) {
      mode_ = HurdleMode::kFailed;
      result.mode = mode_;
      return result;
    }
    result.action = RecoveryAction();
    return result;
  case HurdleMode::kFailed:
    return result;
  }
  return result;
}

void HurdleController::ObserveOnly(const std::optional<ObjectTarget> &target,
                                   int width, int height) {
  if (mode_ != HurdleMode::kWaitCameraDown &&
      mode_ != HurdleMode::kReturnCameraToLine)
    UpdateTracker(target, width, height);
}

void HurdleController::ClearObservationHistory() {
  ClearTracking();
}

void HurdleController::ClearTracking() {
  hit_history_.clear();
  acquire_history_.clear();
  tilt_history_.clear();
  lost_count_ = 0;
  recovery_visible_count_ = 0;
  has_smoothed_ = false;
  tracked_ = {};
}

void HurdleController::ResetToLine(bool clear_ignore) {
  mode_ = HurdleMode::kLineFollow;
  ClearTracking();
  last_seen_u_norm_ = 0.5;
  state_enter_sec_ = 0.0;
  settle_until_sec_ = 0.0;
  close_trigger_latched_ = false;
  if (clear_ignore) ignore_until_sec_ = 0.0;
}

void HurdleController::Reset() { ResetToLine(true); }

} // namespace vision_core
