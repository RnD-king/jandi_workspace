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
  case BallMode::kMidTurnBeforeRecovery:
    return "BALL_MID_TURN_THEN_RECOVERY";
  case BallMode::kFineSidePair:
    return "BALL_FINE_SIDE_PAIR";
  case BallMode::kFineMotionRepeat:
    return "BALL_FINE_MOTION_REPEAT";
  case BallMode::kLineFollow:
    return "LINE_FOLLOW";
  case BallMode::kApproachBall:
    return "BALL_APPROACH";
  case BallMode::kWaitCameraDown:
    return "BALL_CAMERA_DOWN";
  case BallMode::kFineAdjustForPickup:
    return "BALL_FINE";
  case BallMode::kPickupBall:
    return "BALL_PICKUP";
  case BallMode::kVerifyPickupObservation:
    return "BALL_VERIFY_OBSERVE";
  case BallMode::kStandUpAfterPickup:
    return "BALL_STAND_UP";
  case BallMode::kPostPickupBackAway:
    return "BALL_BACK_AWAY";
  case BallMode::kReturnCameraToLine:
    return "BALL_CAMERA_FORWARD";
  case BallMode::kPostPickupLineRecovery:
    return "BALL_LINE_RECOVERY";
  case BallMode::kBallRecoveryForward:
    return "BALL_RECOVERY_FORWARD";
  case BallMode::kBallRecoveryWalkTurn:
    return "BALL_RECOVERY_WALK_TURN";
  case BallMode::kBallRecoveryDown:
    return "BALL_RECOVERY_DOWN";
  case BallMode::kFailed:
    return "BALL_FAILED";
  case BallMode::kDownSearch:
    return "BALL_DOWN_SEARCH";
  case BallMode::kApproachTwoBeforeDown:
    return "BALL_WALK_THEN_DOWN";
  case BallMode::kNearTurnBeforeDown:
    return "BALL_TURN_REOBSERVE";
  case BallMode::kCoarseAdjust:
    return "BALL_DOWN_WALK";
  case BallMode::kWaitFinePose:
    return "BALL_TO_FINE_FEET";
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
  recent_ball_u_.push_back(
      detected ? std::optional<double>(target->center_rectified
                                           ? target->rectified_center_px.u
                                           : target->center_px.u)
               : std::nullopt);
  while (static_cast<int>(hit_history_.size()) >
         std::max(1, config_.stable_window))
    hit_history_.pop_front();
  while (static_cast<int>(recent_ball_u_.size()) >
         std::max(1, config_.stable_window))
    recent_ball_u_.pop_front();

  if (detected) {
    lost_count_ = 0;
    observation_window_.Add(target, config_.stable_window);
    const auto mean =
        observation_window_.Mean(mode_ != BallMode::kFineAdjustForPickup)
            .value();
    const Point2 center =
        mean.center_rectified ? mean.rectified_center_px : mean.center_px;
    tracked_.visible = true;
    tracked_.center_px = center;
    tracked_.u_norm = Clamp(center.u / Denom(image_width), 0.0, 1.0);
    tracked_.v_norm = Clamp(center.v / Denom(image_height), 0.0, 1.0);
    tracked_.h_norm = Clamp(mean.height_px / Denom(image_height), 0.0, 1.0);
    tracked_.area_norm = Clamp(
        mean.area_px / Denom(double(image_width) * image_height), 0.0, 1.0);
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
      recent_ball_u_.size() >=
          static_cast<std::size_t>(config_.stable_min_hits)) {
    double sum = 0.0;
    int n = 0;
    for (const auto &u : recent_ball_u_)
      if (u && std::isfinite(*u)) {
        sum += *u;
        ++n;
      }
    if (n >= config_.stable_min_hits)
      last_stable_ball_u_ = sum / static_cast<double>(n);
  }
}

ActionRequest BallController::FarAction(int image_height) const {
  const auto bounds=config_.ApproachBounds(tracked_.center_px.v,image_height);
  const double center=(bounds.first+bounds.second)*0.5;
  const double half_width=(bounds.second-bounds.first)*0.5;
  const CruiseDecision decision = SelectCruiseDecision(
      true, tracked_.visible, tracked_.center_px.u-center,half_width);
  if (decision.direction == CruiseDirection::kNone)
    return {};
  const double v = tracked_.center_px.v;
  // Near: rotate in WALK stance until the ball reaches center column;
  // Center/near issues TWO steps, or ONE step below the zone-8 boundary.
  // If a side turn loses the ball, Compute permits one extra same-direction
  // turn and a fresh stationary observation before deciding whether DOWN is necessary.
  if (v >= config_.approach_two_step_v_px) {
    if (decision.direction == CruiseDirection::kLeft)
      return Locomotion(MissionAction::kTurnLeftInPlace);
    if (decision.direction == CruiseDirection::kRight)
      return Locomotion(MissionAction::kStepForwardRightOneHurdle);
    return Locomotion(MissionAction::kStepForwardTwo);
  }
  // BALL keeps its own approach mapping; LINE curves 11/12 are not used here.
  const bool middle_row = v >= config_.approach_short_v_px;
  if (decision.direction == CruiseDirection::kLeft)
    return Locomotion(middle_row ? MissionAction::kStepForwardLeftOne
                                : MissionAction::kStepForwardLeftTwo);
  if (decision.direction == CruiseDirection::kRight)
    return Locomotion(middle_row ? MissionAction::kStepForwardRightOne
                                : MissionAction::kStepForwardRightTwo);
  // Both far-center (zone 2) and middle-center (zone 5) use three steps.
  return Locomotion(MissionAction::kStepForwardThree);
}

ActionRequest BallController::FineAction() const {
  if (!tracked_.visible)
    return {};
  const double u = tracked_.center_px.u, v = tracked_.center_px.v;
  const double u_error = u - config_.PickupCenterU();
  const double v_error = v - config_.PickupCenterV();

  const double ru =
      std::max(0.0, std::abs(u_error) - config_.PickupHalfWidth()) /
      config_.PickupHalfWidth();
  const double rv =
      std::max(0.0, std::abs(v_error) - config_.PickupHalfHeight()) /
      config_.PickupHalfHeight();
  if (u >= config_.pickup_u1_px && u <= config_.pickup_u2_px &&
      v >= config_.pickup_v1_px && v <= config_.pickup_v2_px)
    return Mission(MissionAction::kPickBall);
  // Preserve the axis priority: relative excess beyond each rectangle edge.
  if (ru >= rv) {
    if (u < config_.pickup_u1_px)
      return Locomotion(MissionAction::kLeftSideStep);
    if (u > config_.pickup_u2_px)
      return Locomotion(MissionAction::kRightSideStep);
  }
  if (v < config_.pickup_v1_px) {
    return Locomotion(MissionAction::kStepForwardHalf);
  }
  if (v > config_.pickup_v2_px)
    return Locomotion(MissionAction::kStepBack);
  return {};
}

ActionRequest BallController::RecoveryAction() const {
  const double error = last_seen_u_px_ - config_.far_u_des_px;
  if (std::abs(error) <= config_.recovery_center_tolerance_px)
    return {};
  return Locomotion(
      error < 0.0 ? MissionAction::kTurnLeft : MissionAction::kTurnRight, 15);
}

void BallController::BeginForwardRecovery(double now, std::optional<double> u,
                                           bool immediate_turn) {
  stationary_recovery_.Begin(now);
  state_enter_sec_ = now;
  recovery_walk_turns_completed_ = 0;
  recovery_walk_turn_limit_ = immediate_turn ? config_.entry_recovery_walk_turn_max
                                            : config_.recovery_walk_turn_max;
  recovery_walk_turn_pending_ = recovery_walk_turn_done_ = false;
  recovery_turn_search_exhausted_ = false;
  recovery_turn_left_.reset();
  if (u && std::isfinite(*u)) {
    const double offset = *u - config_.far_u_des_px;
    if (std::abs(offset) > config_.recovery_center_tolerance_px)
      recovery_turn_left_ = offset < 0.0;
  }
  mode_ = immediate_turn && recovery_turn_left_ && recovery_walk_turn_limit_ > 0
      ? BallMode::kBallRecoveryWalkTurn : BallMode::kBallRecoveryForward;
}

BallResult BallController::Compute(const std::optional<ObjectTarget> &target,
                                   int image_width, int image_height,
                                   double now_sec, const CameraFeedback &camera,
                                   const ActionExecutionFeedback &feedback,
                                   bool allow_new_body_actions,
                                   bool allow_mission_entry,
                                   std::optional<bool> entry_detected,
                                   std::optional<double> entry_center_u,
                                   bool allow_pickup,
                                   bool allow_fine_pair) {
  if (mode_ == BallMode::kLineFollow && has_ball_) {
    BallResult result;
    result.has_ball = true;
    return result;
  }
  // Ignore all intermediate DOWN/FORWARD camera frames.
  const bool verified = verified_observation_pending_;
  verified_observation_pending_ = false;
  if (!verified && mode_ == BallMode::kFineAdjustForPickup && feedback.action_done)
    ClearTracking();
  if (!verified && mode_ != BallMode::kWaitCameraDown &&
      mode_ != BallMode::kReturnCameraToLine &&
      !((mode_ == BallMode::kFineAdjustForPickup || mode_ == BallMode::kFineSidePair ||
         mode_ == BallMode::kFineMotionRepeat) &&
        (feedback.action_active || feedback.action_done)))
    UpdateTracker(target, image_width, image_height);

  if (mode_ == BallMode::kLineFollow) {
    // Count actual detections within the entry row, including misses.
    // Admission does not require a hit in the current frame or FINE stability.
    const bool entry_hit = entry_detected.value_or(
        target && target->center_px.v <= config_.upper_acquire_v_px);
    entry_hit_history_.push_back(entry_hit);
    const auto entry_u = entry_center_u ? entry_center_u :
        (target ? std::optional<double>(target->center_px.u) : std::nullopt);
    entry_u_history_.push_back(entry_hit && entry_u && std::isfinite(*entry_u)
        ? entry_u : std::nullopt);
    while (static_cast<int>(entry_u_history_.size()) > config_.stable_window)
      entry_u_history_.pop_front();
    while (static_cast<int>(entry_hit_history_.size()) > config_.stable_window)
      entry_hit_history_.pop_front();
    const int entry_hits = static_cast<int>(std::count(
        entry_hit_history_.begin(), entry_hit_history_.end(), true));
    if (allow_mission_entry && entry_hits >= config_.entry_min_hits) {
      entry_recovery_u_.reset();
      double sum_u = 0; int count_u = 0;
      for (const auto &u : entry_u_history_) if (u) { sum_u += *u; ++count_u; }
      if (count_u >= config_.entry_min_hits) entry_recovery_u_ = sum_u / count_u;
      entry_recovery_pending_ = true;
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
  result.entry_frames = static_cast<int>(entry_hit_history_.size());
  result.entry_hits = static_cast<int>(std::count(
      entry_hit_history_.begin(), entry_hit_history_.end(), true));

  switch (mode_) {
  case BallMode::kLineFollow:
    return result;
  case BallMode::kApproachBall: {
    if (feedback.action_active && !feedback.action_done)
      return result;
    // The first post-entry stationary window may fail before ordinary
    // tracking ever reaches seven hits. Retain the real six-hit direction.
    if (entry_recovery_pending_ && feedback.observation_complete) {
      entry_recovery_pending_ = false;
      if (!tracked_.visible || !tracked_.stable) {
        BeginForwardRecovery(now_sec, entry_recovery_u_, true);
        result.mode = mode_;
        return result;
      }
    }
    if (feedback.action_done && approach_action_pending_) {
      approach_action_pending_ = false;
      awaiting_approach_observation_ = true;
    }
    if (awaiting_approach_observation_) {
      if (!feedback.observation_complete) return result;
      awaiting_approach_observation_ = false;
      if (!tracked_.visible || !tracked_.stable) {
        if (last_approach_zone_ == 5) {
          mode_ = BallMode::kWaitCameraDown;
          state_enter_sec_ = now_sec;
          result.mode = mode_;
          result.camera_request = CameraRequest::kDown;
          return result;
        }
        if (last_approach_zone_ == 4 || last_approach_zone_ == 6) {
          if (config_.recovery_walk_turn_max == 0) {
            BeginForwardRecovery(now_sec,std::nullopt,false);
            result.mode=mode_;
            return result;
          }
          mid_recovery_turn_ = last_approach_zone_ == 4
              ? MissionAction::kTurnLeftInPlace : MissionAction::kStepForwardRightOneHurdle;
          mid_recovery_turn_pending_ = allow_new_body_actions;
          mode_ = BallMode::kMidTurnBeforeRecovery;
          result.mode = mode_;
          result.action = Locomotion(mid_recovery_turn_);
          return result;
        }
      }
    }
    if (near_turn_action_ != MissionAction::kNone) {
      // DONE may arrive before the next manual PREVIEW. Retain that edge
      // while waiting for the existing fresh stationary observation window.
      if (feedback.action_done) near_turn_done_ = true;
      if (!near_turn_done_ || !feedback.observation_complete)
        return result;
      near_turn_done_ = false;
      if (!tracked_.visible || !tracked_.stable) {
        mode_ = BallMode::kNearTurnBeforeDown;
        near_turn_retry_issued_ = allow_new_body_actions;
        result.mode = mode_;
        result.action = Locomotion(near_turn_action_);
        return result;
      }
      near_turn_action_ = MissionAction::kNone;
    }
    if (lost_count_ >= std::max(1, config_.lost_frames)) {
      const auto direction = entry_recovery_pending_ ? entry_recovery_u_ : last_stable_ball_u_;
      entry_recovery_pending_ = false;
      BeginForwardRecovery(now_sec, direction, false);
      result.mode = mode_;
      return result;
    }
    result.action = FarAction(image_height);
    // Only the straight near-cell (zone 8) is split. Side cells retain their
    // WALK turns; an active motion was already excluded at this case's entry.
    if (result.action.action == MissionAction::kStepForwardTwo &&
        tracked_.center_px.v >= config_.approach_direct_down_v_px) {
      result.action = Locomotion(MissionAction::kStepForwardOne);
    }
    if (allow_new_body_actions && result.action.action != MissionAction::kNone) {
      const auto action = result.action.action;
      const int row = tracked_.center_px.v >= config_.approach_two_step_v_px ? 2 :
          tracked_.center_px.v >= config_.approach_short_v_px ? 1 : 0;
      const bool left = action == MissionAction::kStepForwardLeftOne ||
          action == MissionAction::kStepForwardLeftTwo || action == MissionAction::kTurnLeftInPlace;
      const bool right = action == MissionAction::kStepForwardRightOne ||
          action == MissionAction::kStepForwardRightTwo || action == MissionAction::kStepForwardRightOneHurdle;
      last_approach_zone_ = row*3 + (left ? 1 : right ? 3 : 2);
      approach_action_pending_ = row == 1;
    }
    if (allow_new_body_actions && result.action.action != MissionAction::kNone)
      entry_recovery_pending_ = false;
    if (allow_new_body_actions &&
        (result.action.action == MissionAction::kTurnLeftInPlace ||
         result.action.action == MissionAction::kStepForwardRightOneHurdle)) {
      near_turn_action_ = result.action.action;
      near_turn_done_ = false;
    }
    if (result.action.action == MissionAction::kStepForwardTwo ||
        result.action.action == MissionAction::kStepForwardOne) {
      mode_ = BallMode::kApproachTwoBeforeDown;
      approach_before_down_action_ = result.action.action;
      approach_walk_issued_ = allow_new_body_actions;
      result.mode = mode_;
    }
    return result;
  }
  case BallMode::kMidTurnBeforeRecovery:
    if (tracked_.visible && tracked_.stable &&
        hit_history_.size() >= static_cast<std::size_t>(config_.stable_window)) {
      mode_=BallMode::kApproachBall;
      mid_recovery_turn_pending_=false;
      mid_recovery_turn_=MissionAction::kNone;
      recovery_walk_turns_completed_=0;
      recovery_turn_search_exhausted_=false;
      result.mode=mode_;
      return result;
    }
    if (feedback.action_active && !feedback.action_done) return result;
    if (mid_recovery_turn_pending_) {
      if (!feedback.action_done) return result;
      const bool left = mid_recovery_turn_ == MissionAction::kTurnLeftInPlace;
      mid_recovery_turn_pending_ = false;
      mid_recovery_turn_ = MissionAction::kNone;
      // Count this actual DONE in the existing total recovery-turn budget.
      BeginForwardRecovery(now_sec,std::nullopt,false);
      recovery_turn_left_ = left;
      recovery_walk_turns_completed_=1;
      recovery_turn_search_exhausted_=recovery_walk_turns_completed_>=config_.recovery_walk_turn_max;
      ClearTracking();
      result.mode = mode_;
      result.tracked = {};
      return result;
    }
    result.action = Locomotion(mid_recovery_turn_);
    if (allow_new_body_actions) mid_recovery_turn_pending_ = true;
    return result;
  case BallMode::kNearTurnBeforeDown:
    if (feedback.action_active && !feedback.action_done)
      return result;
    if (near_turn_retry_issued_) {
      if (feedback.action_done) near_turn_done_ = true;
      if (!near_turn_done_ || !feedback.observation_complete)
        return result;
      near_turn_done_ = false;
      near_turn_retry_issued_ = false;
      near_turn_action_ = MissionAction::kNone;
      if (tracked_.visible && tracked_.stable) {
        mode_ = BallMode::kApproachBall;
        result.mode = mode_;
        return result;
      }
      mode_ = BallMode::kWaitCameraDown;
      state_enter_sec_ = now_sec;
      result.mode = mode_;
      result.camera_request = CameraRequest::kDown;
      return result;
    }
    result.action = Locomotion(near_turn_action_);
    if (allow_new_body_actions) near_turn_retry_issued_ = true;
    return result;
  case BallMode::kApproachTwoBeforeDown:
    if (feedback.action_done && approach_walk_issued_) {
      approach_walk_issued_ = false;
      mode_ = BallMode::kWaitCameraDown;
      state_enter_sec_ = now_sec;
      result.mode = mode_;
      result.camera_request = CameraRequest::kDown;
      return result;
    }
    // Keep the originally chosen step count until its real DONE, even if
    // the ball is lost or the threshold is changed while waiting to execute.
    result.action = Locomotion(approach_before_down_action_);
    if (allow_new_body_actions)
      approach_walk_issued_ = true;
    return result;
  case BallMode::kWaitCameraDown:
    if (test_camera_pending_) {
      state_enter_sec_ = now_sec;
      test_camera_pending_ = false;
    }
    result.camera_request = CameraRequest::kDown;
    if (camera.actual_mode == CameraMode::kDown && camera.settled) {
      ClearTracking(); // Start fine control with fresh DOWN-view evidence.
      last_seen_u_px_ = config_.far_u_des_px;
      mode_ = BallMode::kDownSearch;
      down_search_.Reset();
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
    const int hits=static_cast<int>(std::count(hit_history_.begin(),hit_history_.end(),true));
    const auto decision=down_search_.Observe(tracked_.visible,hits,
        config_.down_search_min_hits,config_.down_search_max_steps,
        feedback.action_active,feedback.action_done,
        now_sec,config_.down_search_inter_action_hold_sec);
    if (decision==DownSearchWalk::Decision::kMoving ||
        decision==DownSearchWalk::Decision::kWaiting) return result;
    if (decision==DownSearchWalk::Decision::kFound) {
      mode_=BallMode::kCoarseAdjust;
      ClearTracking();
      result.mode=mode_;
      return result;
    }
    if (decision==DownSearchWalk::Decision::kExhausted) {
      mode_=BallMode::kFailed;
      result.mode=mode_;
      return result;
    }
    result.action=Locomotion(MissionAction::kStepForwardOne);
    if (allow_new_body_actions) down_search_.Issued();
    return result;
  }
  case BallMode::kCoarseAdjust:
    if (feedback.action_active && !feedback.action_done)
      return result;
    if (feedback.action_done && !verified)
      return result;
    if (feedback.observation_complete && down_search_.Found()) {
      // The moving-view vote only stopped search. Its first stationary
      // confirmation must reacquire the ball before ordinary DOWN approach.
      // Retain completed steps and the last DONE/hold when searching again.
      down_search_.InvalidateDetection();
      if (!tracked_.visible || !tracked_.stable) {
        mode_ = BallMode::kDownSearch;
        ClearTracking();
        result.mode = mode_;
        result.tracked = {};
        return result;
      }
    }
    if (!tracked_.visible) {
      if (lost_count_ >= config_.lost_frames) {
        mode_ = BallMode::kBallRecoveryDown;
        ClearTracking();
        stationary_recovery_.Begin(now_sec);
        result.mode = mode_;
      }
      return result;
    }
    if (tracked_.center_px.v >= config_.fine_mode_v_px ||
        tracked_.center_px.v >= config_.pickup_v1_px) {
      mode_ = BallMode::kWaitFinePose;
      result.mode = mode_;
      result.action = Locomotion(MissionAction::kDefaultPoseMode);
    } else
      result.action =
          Locomotion(tracked_.center_px.v < config_.down_one_step_v_px
                         ? MissionAction::kStepForwardTwo
                         : MissionAction::kStepForwardOne);
    return result;
  case BallMode::kWaitFinePose:
    if (feedback.action_done) {
      mode_ = BallMode::kFineAdjustForPickup;
      fine_pose_entered_ = true;
      ClearTracking();
      result.mode = mode_;
      result.tracked = {};
      return result;
    }
    result.action = Locomotion(MissionAction::kDefaultPoseMode);
    return result;
  case BallMode::kFineAdjustForPickup:
    if (feedback.action_active && !feedback.action_done)
      return result;
    if (feedback.action_done && !verified)
      return result;
    if (lost_count_ >= std::max(1, config_.lost_frames)) {
      mode_ = BallMode::kBallRecoveryDown;
      ClearTracking();
      stationary_recovery_.Begin(now_sec);
      result.mode = mode_;
      return result;
    }
    if (!ExecutionObservationReady())
      return result;
    result.action = FineAction();
    if (result.action.category==ActionCategory::kLocomotion && config_.fine_motion.HasCalibration()) {
      const auto plan=PlanBallFineMotion(config_.fine_motion,tracked_.center_px,
          config_.pickup_u1_px,config_.pickup_u2_px,config_.pickup_v1_px,config_.pickup_v2_px,
          image_width,image_height,allow_fine_pair ? config_.fine_motion.max_repetitions : 1);
      if (plan.action!=MissionAction::kNone) {
        result.action=Locomotion(plan.action);
        if (plan.repetitions>1 && allow_new_body_actions && allow_fine_pair) {
          fine_side_action_=plan.action;
          fine_side_remaining_=fine_repeat_total_=plan.repetitions;
          fine_repeat_predicted_center_=plan.predicted_center_px;
          fine_side_pending_=true;fine_side_done_=false;
          mode_=BallMode::kFineMotionRepeat;result.mode=mode_;
          return result;
        }
      }
      // No valid improving prediction: retain the original single action.
      // Never append unmeasured motions to a partially calibrated plan.
    } else if (result.action.action == MissionAction::kLeftSideStep ||
               result.action.action == MissionAction::kRightSideStep) {
      const double excess = result.action.action == MissionAction::kLeftSideStep
          ? config_.pickup_u1_px - tracked_.center_px.u
          : tracked_.center_px.u - config_.pickup_u2_px;
      if (excess >= config_.fine_side_two_step_margin_px &&
          allow_new_body_actions && allow_fine_pair && config_.fine_motion.max_repetitions>=2) {
        fine_side_action_ = result.action.action;
        fine_side_remaining_ = 2;
        fine_repeat_total_ = 2;
        fine_side_pending_ = allow_new_body_actions;
        fine_side_done_ = false;
        mode_ = BallMode::kFineSidePair;
        result.mode = mode_;
        return result;
      }
    }
    if (allow_new_body_actions && allow_pickup &&
        result.action.action == MissionAction::kPickBall) {
      ++pickup_attempt_count_;
      result.pickup_attempt_count = pickup_attempt_count_;
      mode_ = BallMode::kPickupBall;
      result.mode = mode_;
    }
    return result;
  case BallMode::kFineSidePair:
  case BallMode::kFineMotionRepeat:
    if (fine_side_pending_ && feedback.action_done) fine_side_done_ = true;
    if (feedback.action_active && !feedback.action_done) return result;
    if (fine_side_pending_) {
      if (!fine_side_done_) return result;
      fine_side_pending_ = fine_side_done_ = false;
      if (--fine_side_remaining_ == 0) {
        mode_ = BallMode::kFineAdjustForPickup;
        fine_side_action_ = MissionAction::kNone;
        fine_repeat_total_ = 0;
        fine_repeat_predicted_center_ = {};
        ClearTracking();
        result.mode = mode_;
        result.tracked = {};
        return result;
      }
    }
    result.action = Locomotion(fine_side_action_);
    if (allow_new_body_actions) fine_side_pending_ = true;
    return result;
  case BallMode::kPickupBall:
    result.action = Mission(MissionAction::kPickBall);
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
    if (static_cast<int>(
            std::count(hit_history_.begin(), hit_history_.end(), true)) <
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
        result.mode = mode_;
        result.camera_request = CameraRequest::kForward;
        result.action = {};
        return result;
      }
    }
    result.action = Locomotion(MissionAction::kStepBack);
    if (allow_new_body_actions)
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
  case BallMode::kBallRecoveryDown: {
    if (feedback.action_active && !feedback.action_done)
      return result;
    const auto status = stationary_recovery_.Observe(
        target.has_value(), now_sec, config_.stable_window,
        config_.stable_min_hits, config_.recovery_observation_sec,
        config_.recovery_max_observations);
    if (status == StationaryObjectRecovery::Result::kFound) {
      mode_ = fine_pose_entered_ ? BallMode::kFineAdjustForPickup
                                 : BallMode::kCoarseAdjust;
    } else if (status == StationaryObjectRecovery::Result::kFailed) {
      mode_ = BallMode::kFailed;
    } else if (status == StationaryObjectRecovery::Result::kRetry)
      ClearTracking();
    result.mode = mode_;
    result.tracked = tracked_;
    return result;
  }
  case BallMode::kBallRecoveryForward: {
    if (feedback.action_active && !feedback.action_done)
      return result;
    const bool can_turn = !recovery_turn_search_exhausted_ &&
                          recovery_turn_left_.has_value() &&
                          recovery_walk_turns_completed_ < recovery_walk_turn_limit_;
    const auto status = stationary_recovery_.Observe(
        target.has_value(), now_sec, config_.stable_window,
        config_.stable_min_hits, config_.recovery_observation_sec,
        can_turn ? 1 : config_.recovery_max_observations);
    if (status == StationaryObjectRecovery::Result::kFound) {
      mode_ = BallMode::kApproachBall;
      recovery_walk_turns_completed_ = 0;
      recovery_walk_turn_pending_ = false;
      recovery_walk_turn_done_ = false;
      recovery_turn_search_exhausted_ = false;
    } else if (status == StationaryObjectRecovery::Result::kFailed) {
      if (can_turn) {
        mode_ = BallMode::kBallRecoveryWalkTurn;
        // Preserve a completed zone-4/6 turn in this recovery episode.
        recovery_walk_turn_pending_ = false;
        recovery_walk_turn_done_ = false;
        ClearTracking();
      } else
        mode_ = BallMode::kFailed;
    } else if (status == StationaryObjectRecovery::Result::kRetry)
      ClearTracking();
    result.mode = mode_;
    result.tracked = tracked_;
    return result;
  }
  case BallMode::kBallRecoveryWalkTurn: {
    if (tracked_.visible && tracked_.stable &&
        hit_history_.size() >= static_cast<std::size_t>(config_.stable_window)) {
      mode_ = BallMode::kApproachBall;
      recovery_walk_turns_completed_ = 0;
      recovery_walk_turn_pending_ = false;
      recovery_walk_turn_done_ = false;
      recovery_turn_search_exhausted_ = false;
      result.mode = mode_;
      return result;
    }
    if (recovery_walk_turn_pending_ && feedback.action_done) recovery_walk_turn_done_ = true;
    // Use 31/28 in-place WALK turns. A completed turn is followed by
    // the existing 1s post-DONE observation before considering another turn.
    if (feedback.action_active && !feedback.action_done)
      return result;
    if (recovery_walk_turn_pending_) {
      if (!recovery_walk_turn_done_ || !feedback.observation_complete)
        return result;
      recovery_walk_turn_pending_ = false;
      recovery_walk_turn_done_ = false;
      ++recovery_walk_turns_completed_;
      ClearTracking();
      if (recovery_walk_turns_completed_ >= recovery_walk_turn_limit_) {
        recovery_turn_search_exhausted_ = true;
        stationary_recovery_.Begin(now_sec);
        mode_ = BallMode::kBallRecoveryForward;
      }
      result.mode = mode_;
      result.tracked = tracked_;
      return result;
    }
    // Retuning at a manual WAIT_PREVIEW boundary must respect the new budget.
    if (recovery_walk_turns_completed_ >= recovery_walk_turn_limit_) {
      recovery_turn_search_exhausted_ = true;
      stationary_recovery_.Begin(now_sec);
      mode_ = BallMode::kBallRecoveryForward;
      result.mode = mode_;
      return result;
    }
    result.action = Locomotion(*recovery_turn_left_
                                   ? MissionAction::kTurnLeftInPlace
                                   : MissionAction::kStepForwardRightOneHurdle);
    if (allow_new_body_actions) {
      recovery_walk_turn_pending_ = true;
      recovery_walk_turn_done_ = false;
    }
    return result;
  }
  case BallMode::kFailed:
    return result;
  }
  return result;
}

std::optional<double> BallController::EntryDirection() const {
  double sum = 0; int count = 0;
  for (const auto& u : entry_u_history_) if (u && std::isfinite(*u)) { sum += *u; ++count; }
  return count >= config_.entry_min_hits ? std::optional<double>(sum / count) : std::nullopt;
}
void BallController::EnterVerified(const std::optional<ObjectTarget>& mean,
    const std::deque<bool>& hits, int width, int height, double now) {
  UseVerifiedObservation(mean, hits, width, height);
  mode_ = BallMode::kApproachBall;
  state_enter_sec_ = now;
  pickup_attempt_count_ = 0; pickup_failed_ = false; has_ball_ = false;
  entry_recovery_u_ = mean ? std::optional<double>(mean->center_px.u) : std::nullopt;
  entry_recovery_pending_ = true;
}

void BallController::ObserveOnly(const std::optional<ObjectTarget> &target,
                                 int width, int height) {
  if (mode_ != BallMode::kWaitCameraDown &&
      mode_ != BallMode::kReturnCameraToLine)
    UpdateTracker(target, width, height);
}

void BallController::UseVerifiedObservation(
    const std::optional<ObjectTarget>& mean, const std::deque<bool>& hits,
    int width, int height) {
  ClearTracking();
  UpdateTracker(mean, width, height);
  hit_history_ = hits;
  while (hit_history_.size() > static_cast<std::size_t>(config_.stable_window))
    hit_history_.pop_front();
  tracked_.stable = hit_history_.size() >= static_cast<std::size_t>(config_.stable_window) &&
      std::count(hit_history_.begin(), hit_history_.end(), true) >= config_.stable_min_hits;
  if (mean && tracked_.stable) last_stable_ball_u_ = tracked_.center_px.u;
  lost_count_ = static_cast<int>(std::distance(hit_history_.rbegin(),
      std::find(hit_history_.rbegin(),hit_history_.rend(),true)));
  verified_observation_pending_ = true;
}

void BallController::ClearObservationHistory() {
  ClearTracking();
}

void BallController::ClearTracking() {
  verified_observation_pending_ = false;
  observation_window_.Clear();
  hit_history_.clear();
  entry_hit_history_.clear();
  entry_u_history_.clear();
  recent_ball_u_.clear();
  lost_count_ = 0;
  recovery_visible_count_ = 0;
  has_smoothed_ = false;
  tracked_ = {};
}

void BallController::ResetToLine(bool clear_ignore) {
  mode_ = BallMode::kLineFollow;
  entry_recovery_u_.reset();
  entry_recovery_pending_ = false;
  fine_side_action_ = MissionAction::kNone;
  fine_side_remaining_ = 0;
  fine_repeat_total_ = 0;
  fine_repeat_predicted_center_ = {};
  fine_side_pending_ = fine_side_done_ = false;
  last_stable_ball_u_.reset();
  recovery_turn_left_.reset();
  recovery_walk_turns_completed_ = 0;
  recovery_walk_turn_limit_ = config_.recovery_walk_turn_max;
  recovery_walk_turn_pending_ = false;
  recovery_walk_turn_done_ = false;
  recovery_turn_search_exhausted_ = false;
  state_enter_sec_ = 0.0;
  test_camera_pending_ = false;
  fine_pose_entered_ = false;
  approach_walk_issued_ = false;
  approach_before_down_action_ = MissionAction::kStepForwardTwo;
  near_turn_action_ = MissionAction::kNone;
  near_turn_done_ = false;
  near_turn_retry_issued_ = false;
  last_approach_steps_ = 0;
  last_approach_zone_ = 0;
  approach_action_pending_ = awaiting_approach_observation_ = false;
  mid_recovery_turn_ = MissionAction::kNone;
  mid_recovery_turn_pending_ = false;
  back_away_issued_ = false;
  completed_back_steps_ = 0;
  down_search_.Reset();
  stationary_recovery_.Begin(0.0);
  if (clear_ignore)
    ignore_ball_until_sec_ = 0.0;
  ClearTracking();
}

void BallController::ClearEntryEvidence() {
  if (mode_ == BallMode::kLineFollow)
    ClearTracking();
}

void BallController::CompleteLineRecovery(double now_sec) {
  if (mode_ != BallMode::kPostPickupLineRecovery)
    return;
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
