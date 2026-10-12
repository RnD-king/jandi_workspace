#include "vision_core/hurdle_controller.hpp"

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
  case HurdleMode::kDownSearch: return "HURDLE_DOWN_SEARCH";
  case HurdleMode::kFineAdjust: return "HURDLE_FINE";
  case HurdleMode::kRecoveryDown: return "HURDLE_RECOVERY_DOWN";
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
    observation_window_.Add(target, config_.stable_window);
    const auto mean = observation_window_.Mean().value();
    const Point2 center = mean.center_rectified ? mean.rectified_center_px : mean.center_px;
    tracked_.visible = true;
    tracked_.u_norm = Clamp(center.u/Denom(image_width),0.0,1.0);
    tracked_.v_norm = Clamp(center.v/Denom(image_height),0.0,1.0);
    tracked_.h_norm = Clamp(mean.height_px/Denom(image_height),0.0,1.0);
    tracked_.bottom_norm = Clamp((mean.box_px.y+mean.box_px.height)/Denom(image_height),0.0,1.0);
    tracked_.confidence = mean.confidence;
    const double raw_v = target->center_px.v/Denom(image_height);
    acquire = raw_v >= config_.acquire_min_v_norm;
    close = raw_v >= config_.tilt_trigger_v_norm;
    last_seen_u_norm_ = tracked_.u_norm;
  } else {
    observation_window_.Add(std::nullopt, config_.stable_window);
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
  return {}; // HURDLE horizontal alignment is deferred to LINE observations.
}

HurdleResult HurdleController::Compute(
    const std::optional<ObjectTarget> &target, int image_width,
    int image_height, double now_sec, const CameraFeedback &camera,
    const ActionExecutionFeedback &feedback,
    bool allow_new_body_actions) {
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
    if (lost_count_ >= std::max(1, config_.lost_frames)) {
      mode_ = HurdleMode::kRecoveryForward;
      state_enter_sec_ = now_sec;
      recovery_visible_count_ = 0;
      result.mode = mode_;
      return result;
    }
    if (tracked_.visible)
      result.action = Locomotion(ApproachAction(CruiseDirection::kStraight,
          tracked_.v_norm >= config_.approach_short_v_norm));
    return result;
  }
  case HurdleMode::kWaitCameraDown:
    result.camera_request = CameraRequest::kDown;
    if (camera.actual_mode == CameraMode::kDown && camera.settled) {
      ClearTracking();  // Intermediate camera observations are discarded.
      last_seen_u_norm_ = 0.5;
      mode_ = HurdleMode::kDownSearch;
      down_search_started_ = down_search_found_ = false;
      result.mode = mode_;
      result.tracked = {};
      result.camera_request = CameraRequest::kNone;
      result.action = {};
    } else if (now_sec - state_enter_sec_ >= camera_motion_timeout_sec_) {
      mode_ = HurdleMode::kFailed;
      result.mode = mode_;
      result.camera_request = CameraRequest::kNone;
    }
    return result;
  case HurdleMode::kDownSearch: {
    down_search_found_=down_search_found_ || (tracked_.visible && tracked_.stable);
    if (feedback.action_active && !feedback.action_done) return result;
    if (down_search_found_) {
      mode_=HurdleMode::kFineAdjust; ClearTracking();
      settle_until_sec_=now_sec+config_.recovery_settle_duration_sec;
      result.mode=mode_; result.action={}; return result;
    }
    if (!down_search_started_) {
      result.action=Locomotion(MissionAction::kStepForwardOne);
      if (!allow_new_body_actions) return result;
      down_search_started_=true; state_enter_sec_=now_sec;
    }
    if (now_sec-state_enter_sec_ >= config_.down_search_timeout_sec) {
      mode_=HurdleMode::kFailed; result.mode=mode_; result.action={}; return result;
    }
    result.action=Locomotion(MissionAction::kStepForwardOne); return result;
  }
  case HurdleMode::kFineAdjust: {
    if (feedback.action_active && !feedback.action_done) return result;
    if (now_sec < settle_until_sec_) return result;
    if (!tracked_.visible) {
      if (lost_count_>=config_.lost_frames) {
        mode_=HurdleMode::kRecoveryDown;
        ClearTracking(); stationary_recovery_.Begin(now_sec); result.mode=mode_;
      }
      return result;
    }
    if (tracked_.v_norm>=config_.contact_v_norm) {
      if (allow_new_body_actions) { mode_=HurdleMode::kContactWalk; completed_contact_steps_=0; }
      result.mode=mode_;
      result.action=Locomotion(MissionAction::kStepForwardHalf);
    } else {
      const double error=config_.fine_target_v_norm-tracked_.v_norm;
      result.action=Locomotion(error>=config_.fine_two_step_v_error_norm ? MissionAction::kStepForwardTwo :
          error>=config_.fine_one_step_v_error_norm ? MissionAction::kStepForwardOne : MissionAction::kStepForwardHalf);
    }
    return result;
  }
  case HurdleMode::kContactWalk:
    if (feedback.action_done) ++completed_contact_steps_;
    if (completed_contact_steps_>=config_.contact_steps) {
      mode_=HurdleMode::kCross; result.mode=mode_;
      result.action=Mission(MissionAction::kHurdle); return result;
    }
    result.action=Locomotion(MissionAction::kStepForwardHalf);
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
  case HurdleMode::kRecoveryDown: {
    if (feedback.action_active && !feedback.action_done) return result;
    const auto status = stationary_recovery_.Observe(target.has_value(), now_sec,
        config_.stable_window, config_.stable_min_hits,
        config_.recovery_observation_sec, config_.recovery_max_observations);
    if (status == StationaryObjectRecovery::Result::kFound) {
      mode_ = HurdleMode::kFineAdjust;
      settle_until_sec_ = now_sec + config_.recovery_settle_duration_sec;
    } else if (status == StationaryObjectRecovery::Result::kFailed) mode_ = HurdleMode::kFailed;
    else if (status == StationaryObjectRecovery::Result::kRetry) ClearTracking();
    result.mode = mode_; result.tracked = tracked_; return result;
  }
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
  observation_window_.Clear();
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
  down_search_started_=down_search_found_=false;
  stationary_recovery_.Begin(0.0);
  completed_contact_steps_=0;
  ClearTracking();
  last_seen_u_norm_ = 0.5;
  state_enter_sec_ = 0.0;
  settle_until_sec_ = 0.0;
  close_trigger_latched_ = false;
  if (clear_ignore) ignore_until_sec_ = 0.0;
}

void HurdleController::Reset() { ResetToLine(true); }

} // namespace vision_core
