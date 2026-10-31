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
  case HurdleMode::kApproachTwoBeforeDown: return "HURDLE_TWO_THEN_DOWN";
  case HurdleMode::kCoarseAdjust: return "HURDLE_DOWN_WALK";
  case HurdleMode::kWaitFinePose: return "HURDLE_TO_FINE_FEET";
  case HurdleMode::kRecoveryStep: return "HURDLE_RECOVERY_STEP";
  case HurdleMode::kRecoveryStepObserve: return "HURDLE_RECOVERY_STEP_OBSERVE";
  case HurdleMode::kFineLineAlign: return "HURDLE_FINE_LINE_ALIGN";
  }
  return "UNKNOWN";
}

double HurdleController::Clamp(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

void HurdleController::BeginFineLineObservation(double now_sec) {
  fine_line_history_.clear();
  fine_line_fresh_.reset();
  fine_line_qualified_ = false;
  fine_line_observing_ = true;
  fine_line_observation_started_ = true;
  fine_line_observation_start_sec_ = now_sec;
}

void HurdleController::ObserveFineLine(const LineGuide &guide, int image_width,
                                      double now_sec) {
  if (!fine_line_monitoring_ || !fine_line_observing_ || image_width <= 1)
    return;
  if (fine_line_last_sample_sec_ &&
      now_sec <= *fine_line_last_sample_sec_ + kEpsilon)
    return;
  fine_line_last_sample_sec_ = now_sec;
  std::optional<FineLineSample> sample;
  if (guide.valid && std::isfinite(guide.offset) &&
      std::isfinite(guide.heading_rad)) {
    constexpr double kDegreesPerRadian = 180.0 / 3.14159265358979323846;
    sample = FineLineSample{guide.offset * (0.5 * image_width),
                            guide.heading_rad * kDegreesPerRadian};
  }
  fine_line_history_.push_back(sample);
  while (fine_line_history_.size() >
         static_cast<std::size_t>(config_.fine_line_stable_window))
    fine_line_history_.pop_front();
  if (fine_line_history_.size() <
      static_cast<std::size_t>(config_.fine_line_stable_window))
    return;
  int hits = 0;
  double offset_sum = 0.0, heading_sum = 0.0;
  for (const auto &entry : fine_line_history_) {
    if (!entry) continue;
    ++hits;
    offset_sum += entry->offset_px;
    heading_sum += entry->heading_deg;
  }
  if (hits < config_.fine_line_stable_min_hits) {
    fine_line_qualified_ = false;
    return;
  }
  // The first complete 10/5 window may end in a missing frame. It still
  // qualifies, but later missing frames must not count as new evidence.
  const bool new_evidence = sample.has_value() || !fine_line_qualified_;
  fine_line_qualified_ = true;
  if (!new_evidence) return;
  fine_line_last_valid_ = FineLineSample{offset_sum / hits, heading_sum / hits};
  fine_line_stale_corrections_ = 0;
  if (mode_ == HurdleMode::kFineLineAlign && fine_line_observation_started_ &&
      now_sec + kEpsilon >= fine_line_observation_start_sec_)
    fine_line_fresh_ = fine_line_last_valid_;
}

void HurdleController::OnActualMotionDone(MissionAction action, double now_sec) {
  if (mode_ == HurdleMode::kFineLineAlign &&
      fine_line_pending_action_ == action && !fine_line_observation_started_)
    BeginFineLineObservation(now_sec);
}

void HurdleController::FinishFineLineAlign(HurdleResult &result) {
  fine_line_history_.clear();
  fine_line_fresh_.reset();
  fine_line_pending_action_ = MissionAction::kNone;
  fine_line_pending_stale_ = false;
  fine_line_monitoring_ = false;
  fine_line_observing_ = false;
  fine_line_observation_started_ = false;
  mode_ = HurdleMode::kFineAdjust;
  result.mode = mode_;
}

void HurdleController::UpdateTracker(const std::optional<ObjectTarget> &target,
                                     int image_width, int image_height) {
  const bool detected = target && image_width > 1 && image_height > 1;
  hit_history_.push_back(detected);
  while (static_cast<int>(hit_history_.size()) >
         std::max(1, config_.stable_window)) hit_history_.pop_front();
  bool acquire = false;
  if (detected) {
    lost_count_ = 0;
    observation_window_.Add(target, config_.stable_window);
    const auto mean = observation_window_.Mean(
        mode_ != HurdleMode::kFineAdjust).value();
    const Point2 center = mean.center_rectified ? mean.rectified_center_px : mean.center_px;
    tracked_.visible = true;
    tracked_.center_px = center;
    tracked_.u_norm = Clamp(center.u/Denom(image_width),0.0,1.0);
    tracked_.v_norm = Clamp(center.v/Denom(image_height),0.0,1.0);
    tracked_.h_norm = Clamp(mean.height_px/Denom(image_height),0.0,1.0);
    tracked_.bottom_norm = Clamp((mean.box_px.y+mean.box_px.height)/Denom(image_height),0.0,1.0);
    tracked_.confidence = mean.confidence;
    const double raw_v = target->center_px.v;
    acquire = raw_v <= config_.upper_acquire_v_px;
    last_seen_u_px_ = tracked_.center_px.u;
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

}

ActionRequest HurdleController::RecoveryAction() const {
  return {}; // HURDLE horizontal alignment is deferred to LINE observations.
}

void HurdleController::BeginStepRecovery(double now_sec, bool entry_loss) {
  entry_step_recovery_ = entry_loss;
  recovery_resume_mode_ = entry_loss ? HurdleMode::kApproach
      : (mode_ == HurdleMode::kApproach ? HurdleMode::kApproach
         : fine_pose_entered_ ? HurdleMode::kFineAdjust : HurdleMode::kCoarseAdjust);
  recovery_step_action_ = !entry_loss && fine_pose_entered_
      ? MissionAction::kStepForwardHalf : MissionAction::kStepForwardOne;
  recovery_step_limit_ = entry_loss ? config_.entry_recovery_max_steps : config_.recovery_max_steps;
  recovery_steps_completed_ = 0;
  recovery_step_pending_ = false;
  entry_observation_pending_ = false;
  last_approach_pending_ = awaiting_approach_observation_ = false;
  if (!entry_loss) down_search_.InvalidateDetection();
  ClearTracking();
  stationary_recovery_.Begin(now_sec);
  mode_ = recovery_step_limit_ > 0 || entry_loss
      ? HurdleMode::kRecoveryStep
      : recovery_resume_mode_ == HurdleMode::kApproach
          ? HurdleMode::kRecoveryForward : HurdleMode::kRecoveryDown;
}

void HurdleController::ExhaustStepRecovery(double now_sec, HurdleResult &result) {
  if (entry_step_recovery_) {
    ResetToLine(true);
    result = {};
    result.mode = HurdleMode::kLineFollow;
  } else {
    ClearTracking();
    stationary_recovery_.Begin(now_sec);
    mode_ = recovery_resume_mode_ == HurdleMode::kApproach
        ? HurdleMode::kRecoveryForward : HurdleMode::kRecoveryDown;
    result.mode = mode_;
    result.tracked = {};
  }
}

HurdleResult HurdleController::Compute(
    const std::optional<ObjectTarget> &target, int image_width,
    int image_height, double now_sec, const CameraFeedback &camera,
    const ActionExecutionFeedback &feedback,
    const CruiseDecision &line_decision,
    bool allow_new_body_actions, bool allow_mission_entry) {
  if (mode_ == HurdleMode::kLineFollow && now_sec < ignore_until_sec_)
    return {};
  // No tracking/near-evidence from the moving camera view.
  const bool verified = verified_observation_pending_;
  verified_observation_pending_ = false;
  if (!verified && mode_ == HurdleMode::kFineAdjust && feedback.action_done)
    ClearTracking();
  if (!verified && mode_ != HurdleMode::kWaitCameraDown &&
      mode_ != HurdleMode::kReturnCameraToLine &&
      !(mode_ == HurdleMode::kFineLineAlign &&
        (camera.actual_mode != CameraMode::kDown || !camera.settled)) &&
      !(mode_ == HurdleMode::kFineAdjust &&
        (feedback.action_active || feedback.action_done)))
    UpdateTracker(target, image_width, image_height);
  if (mode_ == HurdleMode::kLineFollow) {
    const int acquire_hits = static_cast<int>(std::count(
        acquire_history_.begin(), acquire_history_.end(), true));
    if (allow_mission_entry && tracked_.stable && tracked_.visible &&
        acquire_hits >= std::max(1, config_.stable_min_hits)) {
      mode_ = HurdleMode::kApproach;
      entry_observation_pending_ = true;
      state_enter_sec_ = now_sec;
    }
  }

  HurdleResult result;
  result.active = mode_ != HurdleMode::kLineFollow;
  result.mode = mode_;
  result.tracked = tracked_;
  result.recovery_steps_completed = recovery_steps_completed_;
  result.recovery_step_limit = recovery_step_limit_;
  result.entry_recovery = entry_step_recovery_;
  result.fine_line_fresh = fine_line_fresh_.has_value();
  result.fine_line_turns = fine_line_turns_;
  result.fine_line_side_steps = fine_line_side_steps_;
  result.fine_line_search_steps = fine_line_search_steps_;
  const auto &reported_line = fine_line_fresh_ ? fine_line_fresh_ : fine_line_last_valid_;
  if (reported_line) {
    result.fine_line_offset_px = reported_line->offset_px;
    result.fine_line_heading_deg = reported_line->heading_deg;
  }
  if (fine_line_monitoring_ && tracked_.visible &&
      tracked_.center_px.v >= config_.contact_v_px)
    fine_line_close_ = true;
  switch (mode_) {
  case HurdleMode::kLineFollow:
    return result;
  case HurdleMode::kApproach: {
    if (feedback.action_active && !feedback.action_done) return result;
    if (entry_observation_pending_ && feedback.observation_complete) {
      entry_observation_pending_ = false;
      if (!tracked_.visible || !tracked_.stable) {
        BeginStepRecovery(now_sec, entry_recovery_allowed_);
        result.mode = mode_;
        result.tracked = {};
        return result;
      }
    }
    if (feedback.action_done && last_approach_pending_) {
      last_approach_pending_=false;
      awaiting_approach_observation_=true;
    }
    if (awaiting_approach_observation_) {
      if (!feedback.observation_complete) return result;
      awaiting_approach_observation_=false;
      if (!tracked_.visible || !tracked_.stable) {
        if (last_approach_zone_ >= 1 && last_approach_zone_ <= 9) {
          mode_=HurdleMode::kWaitCameraDown; state_enter_sec_=now_sec;
          result.mode=mode_; result.camera_request=CameraRequest::kDown;
        } else {
          BeginStepRecovery(now_sec,entry_recovery_allowed_);result.mode=mode_;result.tracked={};
        }
        return result;
      }
    }
    if (!tracked_.visible) {
      if (lost_count_ >= config_.lost_frames || feedback.action_done) {
        if (last_approach_zone_ >= 1 && last_approach_zone_ <= 9) {
          mode_=HurdleMode::kWaitCameraDown; state_enter_sec_=now_sec;
          result.mode=mode_; result.camera_request=CameraRequest::kDown;
        } else {
          if (entry_observation_pending_) BeginStepRecovery(now_sec,entry_recovery_allowed_);
          else {mode_=HurdleMode::kRecoveryForward;stationary_recovery_.Begin(now_sec);state_enter_sec_=now_sec;}
          recovery_visible_count_=0;result.mode=mode_;result.tracked=tracked_;
        }
      }
      return result;
    }
    // Horizontal alignment comes from the shared LINE O/H steering score,
    // not from the HURDLE YOLO box's horizontal center.
    if (line_decision.direction == CruiseDirection::kNone)
      return result;  // HOLD on invalid line evidence.
    const double v=tracked_.center_px.v;
    const int steps=v>=config_.approach_two_step_v_px ? 2 :
        (v>=config_.approach_short_v_px ? 3 : 4);
    const bool straight = line_decision.direction == CruiseDirection::kStraight;
    if (!straight) {
      const bool left = line_decision.direction == CruiseDirection::kLeft;
      result.action = Locomotion(steps==4
          ? (left ? MissionAction::kStepForwardLeftTwo
                  : MissionAction::kStepForwardRightTwo)
          : steps==3
              ? (left ? MissionAction::kStepForwardLeftOne
                      : MissionAction::kStepForwardRightOne)
              : (left ? MissionAction::kTurnLeftInPlace
                      : MissionAction::kStepForwardRightOneHurdle));
      if (allow_new_body_actions) {
        entry_observation_pending_=false;
        last_approach_steps_=steps;
        last_approach_pending_=true;
        last_approach_zone_=left ? (steps==4 ? 1 : steps==3 ? 4 : 7)
                                 : (steps==4 ? 3 : steps==3 ? 6 : 9);
      }
      return result;
    }
    // Far-center also uses three straight steps; LEFT/RIGHT mappings unchanged.
    result.action=Locomotion(steps==2 ? MissionAction::kStepForwardTwo :
        MissionAction::kStepForwardThree);
    if (allow_new_body_actions) {
      entry_observation_pending_=false;
      last_approach_steps_=steps==4 ? 3 : steps;
      last_approach_pending_=true;
      last_approach_zone_=steps==4 ? 2 : steps==3 ? 5 : 8;
    }
    if (steps==2) {
      mode_=HurdleMode::kApproachTwoBeforeDown; approach_two_issued_=allow_new_body_actions;
      result.mode=mode_;
    }
    return result;
  }
  case HurdleMode::kApproachTwoBeforeDown:
    if (feedback.action_done && approach_two_issued_) {
      approach_two_issued_=false; mode_=HurdleMode::kWaitCameraDown; state_enter_sec_=now_sec;
      result.mode=mode_; result.camera_request=CameraRequest::kDown; return result;
    }
    result.action=Locomotion(MissionAction::kStepForwardTwo);
    if (allow_new_body_actions) approach_two_issued_=true;
    return result;

  case HurdleMode::kWaitCameraDown:
    if (test_camera_pending_) { state_enter_sec_=now_sec; test_camera_pending_=false; }
    result.camera_request = CameraRequest::kDown;
    if (camera.actual_mode == CameraMode::kDown && camera.settled) {
      ClearTracking();  // Intermediate camera observations are discarded.
      fine_line_monitoring_ = true;
      fine_line_observing_ = true;
      fine_line_history_.clear();
      fine_line_qualified_ = false;
      fine_line_last_sample_sec_.reset();
      fine_line_last_valid_.reset();
      fine_line_fresh_.reset();
      fine_line_close_ = false;
      last_seen_u_px_ = config_.target_u_px;
      mode_ = HurdleMode::kDownSearch;
      down_search_.Reset();
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
    const int hits=static_cast<int>(std::count(hit_history_.begin(),hit_history_.end(),true));
    const auto decision=down_search_.Observe(tracked_.visible,hits,
        config_.down_search_min_hits,config_.down_search_max_steps,
        feedback.action_active,feedback.action_done,
        now_sec,config_.down_search_inter_action_hold_sec);
    if (decision==DownSearchWalk::Decision::kMoving ||
        decision==DownSearchWalk::Decision::kWaiting) return result;
    if (decision==DownSearchWalk::Decision::kFound) {
      mode_=HurdleMode::kCoarseAdjust;
      ClearTracking();
      result.mode=mode_;
      return result;
    }
    if (decision==DownSearchWalk::Decision::kExhausted) {
      mode_=HurdleMode::kFailed;
      result.mode=mode_;
      return result;
    }
    result.action=Locomotion(MissionAction::kStepForwardOne);
    if (allow_new_body_actions) down_search_.Issued();
    return result;
  }
  case HurdleMode::kCoarseAdjust:
    if (feedback.action_active && !feedback.action_done) return result;
    if (feedback.action_done && !verified) return result;
    if (feedback.observation_complete && down_search_.Found()) {
      // Retry only the first stationary confirmation after search stopped.
      // Invalidate its found vote, not the completed-step budget or hold.
      down_search_.InvalidateDetection();
      if (!tracked_.visible || !tracked_.stable) {
        mode_ = HurdleMode::kDownSearch;
        ClearTracking();
        result.mode = mode_;
        result.tracked = {};
        return result;
      }
    }
    if (!tracked_.visible) {
      if (lost_count_>=config_.lost_frames) {
        BeginStepRecovery(now_sec,false);result.mode=mode_;result.tracked={};
      }
      return result;
    }
    if (tracked_.center_px.v >= config_.fine_mode_v_px) {
      mode_=HurdleMode::kWaitFinePose; result.mode=mode_;
      result.action=Locomotion(MissionAction::kDefaultPoseMode);
    } else result.action=Locomotion(tracked_.center_px.v < config_.down_one_step_v_px
        ? MissionAction::kStepForwardTwo : MissionAction::kStepForwardOne);
    return result;
  case HurdleMode::kWaitFinePose:
    if (feedback.action_done) {
      fine_pose_entered_=true; mode_=HurdleMode::kFineLineAlign; ClearTracking();
      fine_line_turns_ = fine_line_side_steps_ = 0;
      fine_line_stale_corrections_ = fine_line_search_steps_ = 0;
      fine_line_pending_action_ = MissionAction::kNone;
      BeginFineLineObservation(now_sec);

      result.mode=mode_; result.tracked={}; return result;
    }
    result.action=Locomotion(MissionAction::kDefaultPoseMode); return result;
  case HurdleMode::kFineLineAlign: {
    if (fine_line_pending_action_ != MissionAction::kNone) {
      if (!feedback.action_done) return result;
      const auto completed = fine_line_pending_action_;
      fine_line_pending_action_ = MissionAction::kNone;
      if (fine_line_pending_stale_) ++fine_line_stale_corrections_;
      fine_line_pending_stale_ = false;
      if (completed == MissionAction::kTurnLeft || completed == MissionAction::kTurnRight)
        ++fine_line_turns_;
      else if (completed == MissionAction::kLeftSideStep ||
               completed == MissionAction::kRightSideStep)
        ++fine_line_side_steps_;
      else if (completed == MissionAction::kStepForwardHalf)
        ++fine_line_search_steps_;
      if (!fine_line_observation_started_)
        BeginFineLineObservation(now_sec);
      result.fine_line_turns = fine_line_turns_;
      result.fine_line_side_steps = fine_line_side_steps_;
      result.fine_line_search_steps = fine_line_search_steps_;
    }
    if (feedback.action_active && !feedback.action_done) return result;
    if (camera.actual_mode != CameraMode::kDown || !camera.settled) {
      fine_line_history_.clear();
      fine_line_fresh_.reset();
      fine_line_qualified_ = false;
      fine_line_observing_ = false;
      fine_line_observation_started_ = false;
      return result;
    }
    if (!fine_line_observation_started_)
      BeginFineLineObservation(now_sec);
    if (!fine_line_observation_started_ ||
        now_sec + kEpsilon < fine_line_observation_start_sec_ +
                             config_.fine_line_observation_sec ||
        fine_line_history_.size() <
            static_cast<std::size_t>(config_.fine_line_stable_window))
      return result;

    const auto &line = fine_line_fresh_ ? fine_line_fresh_ : fine_line_last_valid_;
    const bool fresh = fine_line_fresh_.has_value();
    if (fresh &&
        std::abs(line->heading_deg) <= config_.fine_line_heading_tolerance_deg &&
        std::abs(line->offset_px) <= config_.fine_line_offset_tolerance_px) {
      FinishFineLineAlign(result);
      return result;
    }
    MissionAction correction = MissionAction::kNone;
    if (line && (fresh || fine_line_stale_corrections_ <
                                  config_.fine_line_max_stale_corrections)) {
      const bool heading_outside =
          std::abs(line->heading_deg) > config_.fine_line_heading_tolerance_deg;
      const bool offset_outside =
          std::abs(line->offset_px) > config_.fine_line_offset_tolerance_px;
      if (heading_outside && fine_line_turns_ < config_.fine_line_max_turns) {
        correction = line->heading_deg < 0.0 ? MissionAction::kTurnLeft
                                             : MissionAction::kTurnRight;
      } else if (offset_outside &&
                 fine_line_side_steps_ < config_.fine_line_max_side_steps) {
        correction = line->offset_px < 0.0 ? MissionAction::kLeftSideStep
                                            : MissionAction::kRightSideStep;
      } else if (heading_outside || offset_outside) {
        // Every remaining error has exhausted its own correction budget.
        FinishFineLineAlign(result);
        return result;
      }
    }
    if (correction == MissionAction::kNone) {
      if (fine_line_search_steps_ >= config_.fine_line_max_search_steps ||
          fine_line_close_) {
        FinishFineLineAlign(result);
        return result;
      }
      correction = MissionAction::kStepForwardHalf;
    }
    result.action = Locomotion(correction,
        correction == MissionAction::kTurnLeft || correction == MissionAction::kTurnRight
            ? static_cast<std::int16_t>(config_.fine_line_turn_yaw_deg) : 0);
    if (allow_new_body_actions) {
      fine_line_pending_action_ = correction;
      fine_line_pending_stale_ = !fresh && correction != MissionAction::kStepForwardHalf;
      fine_line_observing_ = false;
      fine_line_observation_started_ = false;
    }
    return result;
  }
  case HurdleMode::kFineAdjust: {
    if (feedback.action_active && !feedback.action_done) return result;
    if (!tracked_.visible) {
      if (lost_count_>=config_.lost_frames) {
        BeginStepRecovery(now_sec,false);result.mode=mode_;result.tracked={};
      }
      return result;
    }
    if (!ExecutionObservationReady()) return result;
    if (tracked_.center_px.v>=config_.contact_v_px) {
      if (allow_new_body_actions) {
        mode_=HurdleMode::kContactWalk;
        completed_contact_steps_=0;
        contact_step_pending_=true;
        contact_next_step_sec_=0.0;
      }
      result.mode=mode_;
      result.action=Locomotion(MissionAction::kStepForwardHalf);
    } else {
      result.action=Locomotion(MissionAction::kStepForwardHalf);
    }
    return result;
  }
  case HurdleMode::kContactWalk:
    if (feedback.action_done && contact_step_pending_) {
      contact_step_pending_=false;
      ++completed_contact_steps_;
      contact_next_step_sec_=now_sec+config_.contact_inter_action_hold_sec;
    }
    if (completed_contact_steps_>=config_.contact_steps) {
      contact_next_step_sec_=0.0;
      mode_=HurdleMode::kCross; result.mode=mode_;
      result.action=Mission(MissionAction::kHurdle); return result;
    }
    // Keep the repeat scope active, but request no body action during the
    // configured hold. DONE remains immediate; no extra observation is started.
    if (now_sec+kEpsilon<contact_next_step_sec_) return result;
    contact_next_step_sec_=0.0;
    result.action=Locomotion(MissionAction::kStepForwardHalf);
    if (allow_new_body_actions) contact_step_pending_=true;
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
  case HurdleMode::kRecoveryStep:
    if (feedback.action_active && !feedback.action_done) return result;
    if (recovery_step_pending_) {
      if (!feedback.action_done) return result;
      recovery_step_pending_ = false;
      ++recovery_steps_completed_;
      result.recovery_steps_completed = recovery_steps_completed_;
      ClearTracking();
      stationary_recovery_.Begin(now_sec);
      mode_ = HurdleMode::kRecoveryStepObserve;
      result.mode = mode_;
      result.tracked = {};
      return result;
    }
    if (recovery_steps_completed_ >= recovery_step_limit_) {
      ExhaustStepRecovery(now_sec,result);
      return result;
    }
    result.action = Locomotion(recovery_step_action_);
    if (allow_new_body_actions) recovery_step_pending_ = true;
    return result;
  case HurdleMode::kRecoveryStepObserve: {
    if (feedback.action_active && !feedback.action_done) return result;
    const auto status = stationary_recovery_.Observe(target.has_value(),now_sec,
        config_.stable_window,config_.stable_min_hits,config_.recovery_observation_sec,1);
    if (status == StationaryObjectRecovery::Result::kFound) {
      mode_ = recovery_resume_mode_;
      entry_step_recovery_ = false;
      recovery_steps_completed_ = 0;
      result.recovery_steps_completed = 0;
      result.entry_recovery = false;
      result.mode = mode_;
    } else if (status == StationaryObjectRecovery::Result::kFailed) {
      if (recovery_steps_completed_ >= recovery_step_limit_)
        ExhaustStepRecovery(now_sec,result);
      else {ClearTracking();mode_=HurdleMode::kRecoveryStep;result.mode=mode_;result.tracked={};}
    }
    return result;
  }
  case HurdleMode::kRecoveryForward: {
    if (feedback.action_active && !feedback.action_done) return result;
    const auto status=stationary_recovery_.Observe(target.has_value(),now_sec,
        config_.stable_window,config_.stable_min_hits,
        config_.recovery_observation_sec,config_.recovery_max_observations);
    if (status==StationaryObjectRecovery::Result::kFound) {
      mode_=HurdleMode::kApproach;
      recovery_steps_completed_=0;entry_step_recovery_=false;
    } else if (status==StationaryObjectRecovery::Result::kFailed) mode_=HurdleMode::kFailed;
    else if (status==StationaryObjectRecovery::Result::kRetry) ClearTracking();
    result.mode=mode_; result.tracked=tracked_; return result;
  }
  case HurdleMode::kFailed:
    return result;
  case HurdleMode::kRecoveryDown: {
    if (feedback.action_active && !feedback.action_done) return result;
    const auto status = stationary_recovery_.Observe(target.has_value(), now_sec,
        config_.stable_window, config_.stable_min_hits,
        config_.recovery_observation_sec, config_.recovery_max_observations);
    if (status == StationaryObjectRecovery::Result::kFound) {
      mode_ = fine_pose_entered_ ? HurdleMode::kFineAdjust : HurdleMode::kCoarseAdjust;
      recovery_steps_completed_=0;entry_step_recovery_=false;
    } else if (status == StationaryObjectRecovery::Result::kFailed) mode_ = HurdleMode::kFailed;
    else if (status == StationaryObjectRecovery::Result::kRetry) ClearTracking();
    result.mode = mode_; result.tracked = tracked_; return result;
  }
  }
  return result;
}

bool HurdleController::EntryReady() const {
  return tracked_.visible && tracked_.stable &&
      std::count(acquire_history_.begin(), acquire_history_.end(), true) >= config_.stable_min_hits;
}
void HurdleController::EnterVerified(const std::optional<ObjectTarget>& mean,
    const std::deque<bool>& hits, int width, int height, double now,
    bool allow_entry_recovery) {
  UseVerifiedObservation(mean, hits, width, height);
  mode_ = HurdleMode::kApproach;
  entry_observation_pending_ = true;
  entry_recovery_allowed_ = allow_entry_recovery;
  state_enter_sec_ = now;
}

void HurdleController::ObserveOnly(const std::optional<ObjectTarget> &target,
                                   int width, int height) {
  if (mode_ != HurdleMode::kWaitCameraDown &&
      mode_ != HurdleMode::kReturnCameraToLine)
    UpdateTracker(target, width, height);
}

void HurdleController::EnterContinuousRecovery(const ObjectTarget& target,
    const std::deque<bool>& hits,int width,int height,double now) {
  EnterVerified(target,hits,width,height,now,false);
  BeginStepRecovery(now,false);
}

void HurdleController::EnterContinuousNear(double now) {
  Reset();entry_recovery_allowed_=false;
  mode_=HurdleMode::kWaitCameraDown;state_enter_sec_=now;
}

void HurdleController::UseVerifiedObservation(
    const std::optional<ObjectTarget>& mean, const std::deque<bool>& hits,
    int width, int height) {
  ClearTracking();
  UpdateTracker(mean, width, height);
  hit_history_ = hits;
  while (hit_history_.size() > static_cast<std::size_t>(config_.stable_window))
    hit_history_.pop_front();
  tracked_.stable = hit_history_.size() >= static_cast<std::size_t>(config_.stable_window) &&
      std::count(hit_history_.begin(), hit_history_.end(), true) >= config_.stable_min_hits;
  lost_count_ = static_cast<int>(std::distance(hit_history_.rbegin(),
      std::find(hit_history_.rbegin(),hit_history_.rend(),true)));
  verified_observation_pending_ = true;
}

void HurdleController::ClearObservationHistory() {
  ClearTracking();
  stationary_recovery_.DiscardDetections();
}

void HurdleController::ClearTracking() {
  verified_observation_pending_ = false;
  observation_window_.Clear();
  hit_history_.clear();
  acquire_history_.clear();
  lost_count_ = 0;
  recovery_visible_count_ = 0;
  has_smoothed_ = false;
  tracked_ = {};
}

void HurdleController::ResetToLine(bool clear_ignore) {
  mode_ = HurdleMode::kLineFollow;
  entry_observation_pending_=entry_step_recovery_=recovery_step_pending_=false;
  entry_recovery_allowed_=true;
  recovery_steps_completed_=recovery_step_limit_=0;
  recovery_resume_mode_=HurdleMode::kApproach;
  recovery_step_action_=MissionAction::kStepForwardOne;
  down_search_.Reset();
  stationary_recovery_.Begin(0.0);
  completed_contact_steps_=0;
  contact_step_pending_=false;
  contact_next_step_sec_=0.0;
  ClearTracking();
  last_seen_u_px_ = config_.target_u_px;
  state_enter_sec_ = 0.0;
  fine_line_history_.clear();
  fine_line_qualified_ = false;
  fine_line_last_sample_sec_.reset();
  fine_line_last_valid_.reset();
  fine_line_fresh_.reset();
  fine_line_monitoring_ = fine_line_observing_ = fine_line_observation_started_ = false;
  fine_line_close_ = false;
  fine_line_pending_action_ = MissionAction::kNone;
  fine_line_pending_stale_ = false;
  fine_line_observation_start_sec_ = 0.0;
  fine_line_turns_ = fine_line_side_steps_ = fine_line_stale_corrections_ =
      fine_line_search_steps_ = 0;
  test_camera_pending_=false; fine_pose_entered_=false; approach_two_issued_=false;
  last_approach_steps_=0; awaiting_approach_observation_=false;
  last_approach_pending_=false;
  last_approach_zone_=0;
  if (clear_ignore) ignore_until_sec_ = 0.0;
}

void HurdleController::Reset() { ResetToLine(true); }

} // namespace vision_core
