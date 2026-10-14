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
  case BallMode::kVerifyPickupObservation: return "BALL_VERIFY_OBSERVE";
  case BallMode::kStandUpAfterPickup: return "BALL_STAND_UP";
  case BallMode::kPostPickupBackAway: return "BALL_BACK_AWAY";
  case BallMode::kReturnCameraToLine: return "BALL_CAMERA_FORWARD";
  case BallMode::kPostPickupLineRecovery: return "BALL_LINE_RECOVERY";
  case BallMode::kBallRecoveryForward: return "BALL_RECOVERY_FORWARD";
  case BallMode::kBallRecoveryWalkTurn: return "BALL_RECOVERY_WALK_TURN";
  case BallMode::kBallRecoveryDown: return "BALL_RECOVERY_DOWN";
  case BallMode::kFailed: return "BALL_FAILED";
  case BallMode::kDownSearch: return "BALL_DOWN_SEARCH";
  case BallMode::kApproachTwoBeforeDown: return "BALL_TWO_THEN_DOWN";
  case BallMode::kCoarseAdjust: return "BALL_DOWN_WALK";
  case BallMode::kWaitFinePose: return "BALL_TO_FINE_FEET";
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
  recent_ball_u_.push_back(detected ? std::optional<double>(
      target->center_rectified ? target->rectified_center_px.u :
                                 target->center_px.u) : std::nullopt);
  while (static_cast<int>(hit_history_.size()) >
         std::max(1, config_.stable_window)) hit_history_.pop_front();
  while (static_cast<int>(recent_ball_u_.size()) >
         std::max(1, config_.stable_window)) recent_ball_u_.pop_front();

  if (detected) {
    lost_count_ = 0;
    observation_window_.Add(target, config_.stable_window);
    const auto mean = observation_window_.Mean(
        mode_ != BallMode::kFineAdjustForPickup).value();
    const Point2 center = mean.center_rectified ? mean.rectified_center_px : mean.center_px;
    tracked_.visible = true;
    tracked_.center_px = center;
    tracked_.u_norm = Clamp(center.u / Denom(image_width), 0.0, 1.0);
    tracked_.v_norm = Clamp(center.v / Denom(image_height), 0.0, 1.0);
    tracked_.h_norm = Clamp(mean.height_px / Denom(image_height), 0.0, 1.0);
    tracked_.area_norm = Clamp(mean.area_px / Denom(double(image_width)*image_height),0.0,1.0);
    tracked_.confidence = mean.confidence;
    last_seen_u_px_ = tracked_.center_px.u;
  } else {
    observation_window_.Add(std::nullopt, config_.stable_window);
    ++lost_count_;
    tracked_.visible = false;
  }
  const int hits = static_cast<int>(
      std::count(hit_history_.begin(), hit_history_.end(), true));
  tracked_.stable = hits >= std::max(1, config_.stable_min_hits);
  // Freeze a stable multi-frame estimate, not the final moving-camera frame.
  if (tracked_.stable && tracked_.visible &&
      recent_ball_u_.size() >= static_cast<std::size_t>(config_.stable_window)) {
    double sum=0.0; int n=0;
    for (const auto &u : recent_ball_u_)
      if (u && std::isfinite(*u)) { sum+=*u; ++n; }
    if (n >= config_.stable_min_hits)
      last_stable_ball_u_ = sum/static_cast<double>(n);
  }

  const bool upper = detected &&
      target->center_px.v <= config_.upper_acquire_v_px;
  upper_acquire_history_.push_back(upper);
  while (static_cast<int>(upper_acquire_history_.size()) >
         std::max(1, config_.stable_window)) upper_acquire_history_.pop_front();

}

ActionRequest BallController::FarAction() const {
  const CruiseDecision decision = SelectCruiseDecision(
      true, tracked_.visible,
      tracked_.center_px.u - config_.far_u_des_px,
      config_.approach_u_deadband_px);
  if (decision.direction == CruiseDirection::kNone) return {};
  const double v = tracked_.center_px.v;
  // Near: rotate in WALK stance until the ball reaches center column;
  // only center/near issues the final TWO steps and camera DOWN.
  if (v >= config_.approach_two_step_v_px) {
    if (decision.direction == CruiseDirection::kLeft)
      return Locomotion(MissionAction::kStepForwardLeftOne);
    if (decision.direction == CruiseDirection::kRight)
      return Locomotion(MissionAction::kStepForwardRightOneHurdle);
    return Locomotion(MissionAction::kStepForwardTwo);
  }
  // 11/12/13 are reserved for normal LINE cruise, not object approach.
  if (decision.direction == CruiseDirection::kLeft)
    return Locomotion(MissionAction::kStepForwardLeftTwo);
  if (decision.direction == CruiseDirection::kRight)
    return Locomotion(MissionAction::kStepForwardRightTwo);
  return Locomotion(v >= config_.approach_short_v_px
                        ? MissionAction::kStepForwardThree
                        : MissionAction::kStepForwardFour);
}

ActionRequest BallController::FineAction() const {
  if (!tracked_.visible) return {};
  const double u = tracked_.center_px.u, v = tracked_.center_px.v;
  const double u_error = u - config_.PickupCenterU();
  const double v_error = v - config_.PickupCenterV();
  const double ru = std::max(0.0, std::abs(u_error) - config_.PickupHalfWidth()) /
                    config_.PickupHalfWidth();
  const double rv = std::max(0.0, std::abs(v_error) - config_.PickupHalfHeight()) /
                    config_.PickupHalfHeight();
  if (u >= config_.pickup_u1_px && u <= config_.pickup_u2_px &&
      v >= config_.pickup_v1_px && v <= config_.pickup_v2_px)
    return Mission(MissionAction::kPickBall);
  // Preserve the axis priority: relative excess beyond each rectangle edge.
  if (ru >= rv) {
    if (u < config_.pickup_u1_px) return Locomotion(MissionAction::kLeftSideStep);
    if (u > config_.pickup_u2_px) return Locomotion(MissionAction::kRightSideStep);
  }
  if (v < config_.pickup_v1_px) {
    return Locomotion(MissionAction::kStepForwardHalf);
  }
  if (v > config_.pickup_v2_px) return Locomotion(MissionAction::kStepBack);
  return {};
}

ActionRequest BallController::RecoveryAction() const {
  const double error = last_seen_u_px_ - config_.far_u_des_px;
  if (std::abs(error) <= config_.recovery_center_tolerance_px) return {};
  return Locomotion(error < 0.0 ? MissionAction::kTurnLeft
                                : MissionAction::kTurnRight,
                    15);
}

BallResult BallController::Compute(
    const std::optional<ObjectTarget> &target, int image_width,
    int image_height, double now_sec, const CameraFeedback &camera,
    const ActionExecutionFeedback &feedback,
    bool allow_new_body_actions) {
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
  // Ignore all intermediate DOWN/FORWARD camera frames.
  if (mode_ == BallMode::kFineAdjustForPickup && feedback.action_done)
    ClearTracking();
  if (mode_ != BallMode::kWaitCameraDown &&
      mode_ != BallMode::kReturnCameraToLine &&
      !(mode_ == BallMode::kFineAdjustForPickup &&
        (feedback.action_active || feedback.action_done)))
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
    if (feedback.action_active && !feedback.action_done) return result;
    if (lost_count_ >= std::max(1, config_.lost_frames)) {
      mode_ = BallMode::kBallRecoveryForward; stationary_recovery_.Begin(now_sec);
      state_enter_sec_ = now_sec;
      recovery_visible_count_ = 0;
      recovery_walk_turns_completed_=0;
      recovery_walk_turn_pending_=false;
      recovery_turn_search_exhausted_=false;
      recovery_turn_left_.reset();
      if (last_stable_ball_u_ && std::isfinite(*last_stable_ball_u_)) {
        const double offset=*last_stable_ball_u_ - config_.far_u_des_px;
        if (std::abs(offset) > config_.recovery_center_tolerance_px)
          recovery_turn_left_ = offset < 0.0;
      }
      result.mode = mode_;
      return result;
    }
    result.action = FarAction();
    if (result.action.action == MissionAction::kStepForwardTwo) {
      mode_ = BallMode::kApproachTwoBeforeDown;
      approach_two_issued_ = allow_new_body_actions;
      result.mode = mode_;
    }
    return result;
  }
  case BallMode::kApproachTwoBeforeDown:
    if (feedback.action_done && approach_two_issued_) {
      approach_two_issued_ = false;
      mode_ = BallMode::kWaitCameraDown; state_enter_sec_ = now_sec;
      result.mode = mode_; result.camera_request = CameraRequest::kDown;
      return result;
    }
    result.action = Locomotion(MissionAction::kStepForwardTwo);
    if (allow_new_body_actions) approach_two_issued_ = true;
    return result;
  case BallMode::kWaitCameraDown:
    if (test_camera_pending_) { state_enter_sec_=now_sec; test_camera_pending_=false; }
    result.camera_request = CameraRequest::kDown;
    if (camera.actual_mode == CameraMode::kDown && camera.settled) {
      ClearTracking();  // Start fine control with fresh DOWN-view evidence.
      last_seen_u_px_ = config_.far_u_des_px;
      mode_ = BallMode::kDownSearch;
      down_search_started_ = down_search_found_ = false;
      result.tracked = {};
      result.mode = mode_;
      result.camera_request = CameraRequest::kNone;
    } else if (now_sec - state_enter_sec_ >= camera_motion_timeout_sec_) {
      mode_ = BallMode::kFailed;
      result.mode = mode_;
      result.camera_request = CameraRequest::kNone;
    }
    return result;
  case BallMode::kDownSearch: {
    down_search_found_ = down_search_found_ || (tracked_.visible && tracked_.stable);
    if (feedback.action_active && !feedback.action_done) return result;
    if (down_search_found_) {
      mode_ = BallMode::kCoarseAdjust;
      ClearTracking();
      settle_until_sec_ = now_sec + config_.fine_settle_duration_sec;
      result.mode = mode_; result.action = {}; return result;
    }
    if (!down_search_started_) {
      result.action = Locomotion(MissionAction::kStepForwardOne);
      if (!allow_new_body_actions) return result;
      down_search_started_ = true;
      state_enter_sec_ = now_sec;
    }
    const bool expired = now_sec-state_enter_sec_ >= config_.down_search_timeout_sec;
    if (expired) {
      mode_ = BallMode::kFailed; result.mode = mode_; result.action = {}; return result;
    }
    result.action = Locomotion(MissionAction::kStepForwardOne);
    return result;
  }
  case BallMode::kCoarseAdjust:
    if (feedback.action_active && !feedback.action_done) return result;
    if (feedback.action_done) { settle_until_sec_=now_sec+config_.fine_settle_duration_sec; return result; }
    if (now_sec < settle_until_sec_) return result;
    if (!tracked_.visible) {
      if (lost_count_ >= config_.lost_frames) {
        mode_=BallMode::kBallRecoveryDown; ClearTracking(); stationary_recovery_.Begin(now_sec);
        result.mode=mode_;
      }
      return result;
    }
    if (tracked_.center_px.v >= config_.fine_mode_v_px ||
        tracked_.center_px.v >= config_.pickup_v1_px) {
      mode_=BallMode::kWaitFinePose; result.mode=mode_;
      result.action=Locomotion(MissionAction::kDefaultPoseMode);
    } else result.action=Locomotion(tracked_.center_px.v < config_.down_one_step_v_px
        ? MissionAction::kStepForwardTwo : MissionAction::kStepForwardOne);
    return result;
  case BallMode::kWaitFinePose:
    if (feedback.action_done) {
      mode_=BallMode::kFineAdjustForPickup; fine_pose_entered_=true;
      ClearTracking(); settle_until_sec_=now_sec+config_.fine_settle_duration_sec;
      result.mode=mode_; result.tracked={}; return result;
    }
    result.action=Locomotion(MissionAction::kDefaultPoseMode); return result;
  case BallMode::kFineAdjustForPickup:
    if (feedback.action_active && !feedback.action_done) return result;
    if (feedback.action_done) {
      settle_until_sec_ = now_sec + config_.fine_settle_duration_sec;
      return result;
    }
    if (now_sec + kEpsilon < settle_until_sec_) return result;
    if (lost_count_ >= std::max(1, config_.lost_frames)) {
      mode_ = BallMode::kBallRecoveryDown;
      ClearTracking();
      stationary_recovery_.Begin(now_sec);
      result.mode = mode_;
      return result;
    }
    if (!ExecutionObservationReady()) return result;
    result.action = FineAction();
    if (allow_new_body_actions &&
        result.action.action == MissionAction::kPickBall) {
      ++pickup_attempt_count_;
      result.pickup_attempt_count = pickup_attempt_count_;
      mode_ = BallMode::kPickupBall;
      result.mode = mode_;
    }
    return result;
  case BallMode::kPickupBall:
    result.action = Mission(MissionAction::kPickBall);
    if (feedback.action_done) {
      mode_ = BallMode::kVerifyPickupObservation;
      ClearTracking(); result.mode = mode_; result.action = {};
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
      if (has_ball_ && config_.post_pickup_back_steps > 0) {
        mode_ = BallMode::kPostPickupBackAway;
        back_away_issued_ = false;
        completed_back_steps_ = 0;
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
      ++completed_back_steps_;
      back_away_issued_ = false;
      if (completed_back_steps_ >= config_.post_pickup_back_steps) {
        mode_ = BallMode::kReturnCameraToLine;
        result.mode = mode_; result.camera_request = CameraRequest::kForward;
        result.action = {}; return result;
      }
    }
    result.action = Locomotion(MissionAction::kStepBack);
    if (allow_new_body_actions) back_away_issued_ = true;
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
  case BallMode::kBallRecoveryDown: {
    if (feedback.action_active && !feedback.action_done) return result;
    const auto status = stationary_recovery_.Observe(target.has_value(), now_sec,
        config_.stable_window, config_.stable_min_hits,
        config_.recovery_observation_sec, config_.recovery_max_observations);
    if (status == StationaryObjectRecovery::Result::kFound) {
      mode_ = fine_pose_entered_ ? BallMode::kFineAdjustForPickup : BallMode::kCoarseAdjust;
      settle_until_sec_ = now_sec + config_.fine_settle_duration_sec;
    } else if (status == StationaryObjectRecovery::Result::kFailed) {
      mode_ = BallMode::kFailed;
    } else if (status == StationaryObjectRecovery::Result::kRetry) ClearTracking();
    result.mode = mode_; result.tracked = tracked_;
    return result;
  }
  case BallMode::kBallRecoveryForward: {
    if (feedback.action_active && !feedback.action_done) return result;
    const bool can_turn = !recovery_turn_search_exhausted_ &&
        recovery_turn_left_.has_value() && config_.recovery_walk_turn_max>0;
    const auto status=stationary_recovery_.Observe(target.has_value(),now_sec,
        config_.stable_window,config_.stable_min_hits,
        config_.recovery_observation_sec,
        can_turn ? 1 : config_.recovery_max_observations);
    if (status==StationaryObjectRecovery::Result::kFound) {
      mode_=BallMode::kApproachBall;
      settle_until_sec_=now_sec+config_.recovery_settle_duration_sec;
    } else if (status==StationaryObjectRecovery::Result::kFailed) {
      if (can_turn) {
        mode_=BallMode::kBallRecoveryWalkTurn;
        recovery_walk_turns_completed_=0;
        recovery_walk_turn_pending_=false;
        ClearTracking();
      } else mode_=BallMode::kFailed;
    } else if (status==StationaryObjectRecovery::Result::kRetry) ClearTracking();
    result.mode=mode_; result.tracked=tracked_; return result;
  }
  case BallMode::kBallRecoveryWalkTurn: {
    // Reuse 27/28 WALK one-cycle turns. A completed turn is followed by
    // the existing 1s post-DONE observation before considering another turn.
    if (feedback.action_active && !feedback.action_done) return result;
    if (recovery_walk_turn_pending_) {
      if (!feedback.action_done || !feedback.observation_complete) return result;
      recovery_walk_turn_pending_=false;
      ++recovery_walk_turns_completed_;
      if (tracked_.visible && tracked_.stable) {
        mode_=BallMode::kApproachBall;
        settle_until_sec_=now_sec+config_.recovery_settle_duration_sec;
        result.mode=mode_;
        return result;
      }
      ClearTracking();
      if (recovery_walk_turns_completed_ >= config_.recovery_walk_turn_max) {
        recovery_turn_search_exhausted_=true;
        stationary_recovery_.Begin(now_sec);
        mode_=BallMode::kBallRecoveryForward;
      }
      result.mode=mode_; result.tracked=tracked_;
      return result;
    }
    result.action=Locomotion(*recovery_turn_left_
        ? MissionAction::kStepForwardLeftOne
        : MissionAction::kStepForwardRightOneHurdle);
    if (allow_new_body_actions) recovery_walk_turn_pending_=true;
    return result;
  }
  case BallMode::kFailed:
    return result;
  }
  return result;
}

void BallController::ObserveOnly(const std::optional<ObjectTarget> &target,
                                 int width, int height) {
  if (mode_ != BallMode::kWaitCameraDown &&
      mode_ != BallMode::kReturnCameraToLine)
    UpdateTracker(target, width, height);
}

void BallController::ClearObservationHistory() {
  ClearTracking();
}

void BallController::ClearTracking() {
  observation_window_.Clear();
  hit_history_.clear();
  upper_acquire_history_.clear();
  recent_ball_u_.clear();
  lost_count_ = 0;
  recovery_visible_count_ = 0;
  has_smoothed_ = false;
  tracked_ = {};
}

void BallController::ResetToLine(bool clear_ignore) {
  mode_ = BallMode::kLineFollow;
  last_stable_ball_u_.reset();
  recovery_turn_left_.reset();
  recovery_walk_turns_completed_=0;
  recovery_walk_turn_pending_=false;
  recovery_turn_search_exhausted_=false;
  state_enter_sec_ = 0.0;
  settle_until_sec_ = 0.0;
  test_camera_pending_=false; fine_pose_entered_=false; approach_two_issued_=false; last_approach_steps_=0;
  back_away_issued_ = false;
  completed_back_steps_ = 0;
  down_search_started_ = down_search_found_ = false;
  stationary_recovery_.Begin(0.0);
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
