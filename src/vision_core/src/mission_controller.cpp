#include "vision_core/mission_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "vision_core/config_loader.hpp"
#include "vision_core/coordinate_rectifier.hpp"

namespace vision_core {
namespace {
bool IsLongLineAction(MissionAction action) {
  return action == MissionAction::kStepForwardLeft ||
         action == MissionAction::kStepForwardFour ||
         action == MissionAction::kStepForwardRight;
}
ActionRequest LineLocomotion(MissionAction action, bool queue_eligible) {
  if (action == MissionAction::kNone)
    return {};
  return {action, ActionCategory::kLocomotion, 0, queue_eligible};
}
ActionRequest RecoveryTurn(bool left, int yaw_deg) {
  // BALL/GOAL return to LINE in FINE stance; switch to WALK only after
  // the stationary LINE verification succeeds.
  return {left ? MissionAction::kTurnLeft : MissionAction::kTurnRight,
          ActionCategory::kLocomotion,static_cast<std::int16_t>(yaw_deg),false};
}
MissionControllerConfig ValidatedConfig(const MissionControllerConfig &config) {
  ValidateAlgorithmConfig(config);
  return config;
}
} // namespace

MissionController::MissionController()
    : MissionController(LoadDefaultAlgorithmConfig()) {}

MissionController::MissionController(const MissionControllerConfig &config)
    : config_(ValidatedConfig(config)),
      line_plan_(config_.line_p2p,config_.line.line_stable_window,config_.line.line_stable_min_hits),
      line_p2p_controller_(config_.line_p2p),
      line_guide_accumulator_(config_.line_p2p),
      ball_controller_(config_.ball, config_.camera_motion_timeout_sec),
      hurdle_controller_(config_.hurdle, config_.camera_motion_timeout_sec),
      goal_controller_(config_.goal, config_.camera_motion_timeout_sec),
      command_coordinator_(config_.command),
      ball_association_tracker_(config_.object_association),
      has_ball_(config_.initial_has_ball) {
  ball_mission_index_ = config_.test_ball_mission_index - 1;
  goal_mission_index_ = config_.test_goal_mission_index - 1;
  if (has_ball_)
    goal_controller_.StartAfterPickup(0.0);
  post_pickup_goal_line_gate_active_ =
      has_ball_ && config_.enable_goal && !config_.goal_trial_start_goal;
  if (config_.goal_trial_start_goal) {
    active_mission_ = MissionType::kGoal;
    has_ball_ = true;
    goal_mission_index_ = config_.test_goal_mission_index;
    goal_controller_.StartGoalTrial();
    goal_result_.mode = GoalMode::kWaitCameraGoal;
  }
  InitializeTest();
}

bool MissionController::StraightGoalGateReached(int additional_straight_blocks) const {
  const int required = config_.goal.post_pickup_straight_blocks_required;
  return required > 0 &&
      post_pickup_straight_blocks_completed_ + additional_straight_blocks >= required;
}

ActionRequest MissionController::CruiseAction(const LineGuide &guide,
                                              bool queue_eligible) {
  const CruiseDecision decision = line_p2p_controller_.Compute(guide);
  // Remember the signed decision, including straight/deadband and exact zero.
  // Never replace it with an invalid frame or the sparse-center recovery hint.
  // Virtual object paths must not replace the last actual LINE direction:
  // releasing a soft candidate must not turn LINE recovery into object search.
  if (decision.direction != CruiseDirection::kNone && continuous_.mission==MissionType::kNone)
    last_line_score_ = decision.error;
  MissionAction action = MissionAction::kNone;
  if (decision.direction != CruiseDirection::kNone &&
      std::abs(decision.error) > config_.line_p2p.in_place_turn_score) {
    if (line_guard_turns_completed_ >= config_.line_p2p.recovery_max_turns) {
      line_state_ = LineState::kFinalHold;
      return {};
    }
    line_state_ = LineState::kScoreRecovery;
    // B may be reserved using A's existing READY mean. B itself has no READY.
    return LineLocomotion(decision.error < 0
                              ? MissionAction::kTurnLeftInPlace
                              : MissionAction::kStepForwardRightOneHurdle,
                          false);
  }
  if (decision.direction != CruiseDirection::kNone) {
    line_guard_turns_completed_ = 0;
    if (line_state_ == LineState::kScoreRecovery)
      line_state_ = LineState::kNormal;
  }
  if (decision.direction == CruiseDirection::kLeft)
    action = MissionAction::kStepForwardLeft;
  else if (decision.direction == CruiseDirection::kStraight)
    action = MissionAction::kStepForwardFour;
  else if (decision.direction == CruiseDirection::kRight)
    action = MissionAction::kStepForwardRight;
  return decision.applicable ? LineLocomotion(action, queue_eligible)
                             : ActionRequest{};
}

void MissionController::BeginObservation(LineState state, double now_sec) {
  line_state_ = state;
  line_observation_start_sec_ = std::isfinite(now_sec) ? now_sec : 0.0;
  line_observation_active_ = true;
  observation_offset_sum_ = observation_heading_sum_ = 0.0;
  observation_confidence_sum_ = observation_curvature_sum_ = 0.0;
  observation_valid_count_ = observation_curvature_count_ = 0;
  line_observation_hits_.clear();
  line_observation_qualified_=false;
}

void MissionController::AddObservation(const LineGuide &guide) {
  if (!line_observation_active_) return;
  const bool valid=guide.valid && std::isfinite(guide.offset) &&
      std::isfinite(guide.heading_rad);
  line_observation_hits_.push_back(valid);
  while (line_observation_hits_.size()>static_cast<std::size_t>(config_.line.line_stable_window))
    line_observation_hits_.pop_front();
  if (line_observation_hits_.size()==static_cast<std::size_t>(config_.line.line_stable_window) &&
      std::count(line_observation_hits_.begin(),line_observation_hits_.end(),true)>=
          config_.line.line_stable_min_hits) line_observation_qualified_=true;
  if (!valid) return;
  observation_offset_sum_ += guide.offset;
  observation_heading_sum_ += guide.heading_rad;
  if (std::isfinite(guide.confidence))
    observation_confidence_sum_ += guide.confidence;
  ++observation_valid_count_;
  if (guide.curvature_valid && std::isfinite(guide.curvature_rad)) {
    observation_curvature_sum_ += guide.curvature_rad;
    ++observation_curvature_count_;
  }
}

std::optional<LineGuide> MissionController::FinishObservation() const {
  if (!line_observation_qualified_ || observation_valid_count_ <
      std::max(1, config_.line_p2p.failure_min_valid_samples))
    return std::nullopt;
  LineGuide guide;
  const double count = static_cast<double>(observation_valid_count_);
  guide.offset = observation_offset_sum_ / count;
  guide.heading_rad = observation_heading_sum_ / count;
  guide.confidence = observation_confidence_sum_ / count;
  guide.valid = true;
  if (observation_curvature_count_ > 0) {
    guide.curvature_rad = observation_curvature_sum_ /
                          static_cast<double>(observation_curvature_count_);
    guide.curvature_valid = true;
  }
  return guide;
}

ActionRequest MissionController::ComputeLineAction(
    const LineGuide &guide, const MissionFrameInput &input, bool action_active,
    bool action_done, bool *reference_valid, bool append_stability) {
  if (!line_observation_active_ && (line_state_ == LineState::kFailureObserve ||
                                    line_state_ == LineState::kRecoveryObserve))
    BeginObservation(line_state_, input.now_sec);

  const int stable_window = std::max(1, config_.line.line_stable_window);
  if (append_stability) {
    line_stability_history_.push_back(line_state_ == LineState::kNormal && guide.valid &&
                                      std::isfinite(guide.offset) && std::isfinite(guide.heading_rad));
    while (static_cast<int>(line_stability_history_.size()) > stable_window)
      line_stability_history_.pop_front();
  }
  const int hits = static_cast<int>(std::count(
      line_stability_history_.begin(), line_stability_history_.end(), true));
  if (reference_valid)
    *reference_valid = line_state_ == LineState::kNormal &&
                       line_stability_history_.size() == static_cast<std::size_t>(stable_window) &&
                       hits >= std::max(1, config_.line.line_stable_min_hits);

  if (line_state_ == LineState::kFinalHold)
    return {};
  if (line_state_ == LineState::kRecoveryTurn) {
    if (action_done) {
      recovery_turn_issued_ = false;
      BeginObservation(LineState::kRecoveryObserve,
                       std::max(input.now_sec, settle_until_sec_));
      if (input.now_sec + 1e-9 >= line_observation_start_sec_)
        AddObservation(guide);
      return {};
    }
    if (action_active || recovery_turn_issued_)
      return {};
    if (!input.allow_new_body_actions || !input.allow_new_commands ||
        !input.allow_new_line_action || !input.advance_line_fsm ||
        command_coordinator_.ActionAckTimedOut())
      return {};
    recovery_turn_issued_ = true;
    if (line_recovery_uses_visible_centers_) ++line_visible_recovery_turns_;
    else ++line_last_score_recovery_turns_;
    return {line_recovery_left_ ? MissionAction::kTurnLeftInPlace :
            MissionAction::kStepForwardRightOneHurdle,
            ActionCategory::kLocomotion,0,false};
  }

  if (line_state_ == LineState::kFailureObserve ||
      line_state_ == LineState::kRecoveryObserve) {
    if (action_active)
      return {};
    if (input.now_sec + 1e-9 < line_observation_start_sec_)
      return {};
    AddObservation(guide);
    const double wait =
        std::max(0.0, line_state_ == LineState::kRecoveryObserve
                          ? config_.line_p2p.recovery_observation_sec
                          : config_.line_p2p.failure_observation_sec);
    if (input.now_sec + 1e-9 < line_observation_start_sec_ + wait ||
        line_observation_hits_.size() < static_cast<std::size_t>(stable_window))
      return {};
    const auto observed = FinishObservation();
    line_observation_active_ = false;
    if (observed) {
      line_state_ = LineState::kNormal;
      line_visible_recovery_turns_ = line_last_score_recovery_turns_ = 0;
      line_no_evidence_observations_=0;
      return CruiseAction(*observed, true);
    }
    // Use this frame only, even if it has too few centers for a valid guide.
    // Finite centers in the image follow the same clamping/near-first cap as
    // the feature extractor, without requiring its four-point fitting gate.
    std::vector<Point2> centers;
    if (input.image_width > 1 && input.image_height > 1) {
      for (const auto &p : input.line_centers)
        if (std::isfinite(p.u) && std::isfinite(p.v))
          centers.push_back({std::clamp(p.u, 0.0, double(input.image_width - 1)),
                             std::clamp(p.v, 0.0, double(input.image_height - 1))});
    }
    std::sort(centers.begin(), centers.end(), [](const Point2 &a, const Point2 &b) {
      return a.v == b.v ? a.u < b.u : a.v > b.v;
    });
    centers.resize(std::min(centers.size(), static_cast<std::size_t>(
        std::min(8, std::max(1, config_.line_features.max_centers)))));
    double mean_u = 0.0;
    for (const auto &p : centers) mean_u += p.u;
    if (!centers.empty()) mean_u /= centers.size();
    line_recovery_uses_visible_centers_ = centers.size() >= 2 &&
        mean_u != last_line_image_center_u_;
    line_recovery_left_ = line_recovery_uses_visible_centers_
        ? mean_u < last_line_image_center_u_ : last_line_score_.value_or(0.0) < 0.0;
    if (!line_recovery_uses_visible_centers_ &&
        (!last_line_score_ || std::abs(*last_line_score_)<1e-9)) {
      if (++line_no_evidence_observations_>=5) line_state_=LineState::kFinalHold;
      else BeginObservation(LineState::kFailureObserve,input.now_sec);
      return {};
    }
    const int turns = line_recovery_uses_visible_centers_
        ? line_visible_recovery_turns_ : line_last_score_recovery_turns_;
    const int limit = line_recovery_uses_visible_centers_
        ? config_.line_p2p.visible_recovery_max_turns
        : config_.line_p2p.last_score_recovery_max_turns;
    if (turns < limit) {
      line_state_ = LineState::kRecoveryTurn;
      recovery_turn_issued_ = false;
      return ComputeLineAction(guide, input, false, false, reference_valid, false);
    }
    line_state_ = LineState::kFinalHold;
    return {};
  }

  if (action_active)
    return {};
  if (!guide.valid || !std::isfinite(guide.offset) ||
      !std::isfinite(guide.heading_rad)) {
    BeginObservation(LineState::kFailureObserve, input.now_sec);
    AddObservation(guide);
    return {};
  }
  return CruiseAction(guide, true);
}

void MissionController::BeginLineReacquisition() {
  line_guard_turns_completed_ = 0;
  mission_recovery_turns_ = 0;
  mission_recovery_issued_ = false;
  mission_recovery_failed_ = false;
  mission_recovery_found_ = false;
  mission_recovery_verifying_ = false;
  mission_recovery_qualified_ = false;
  mission_recovery_hits_.clear();
  last_line_score_.reset();
  actual_line_stability_history_.clear();
  line_recovery_uses_visible_centers_ = line_recovery_left_ = false;
  line_visible_recovery_turns_ = line_last_score_recovery_turns_ = 0;
  line_no_evidence_observations_=0;
  recovery_turn_issued_ = false;
  line_observation_active_ = false;
  line_observation_hits_.clear();
  line_observation_qualified_=false;
  line_failure_pending_done_ = false;
  line_state_ = LineState::kFailureObserve;
  line_stability_history_.clear();
}

void MissionController::EnterMission(MissionType mission) {
  active_mission_ = mission;
  line_guide_accumulator_.Reset();
  ready_line_action_id_ = 0;
  line_failure_pending_done_ = false;
  if (mission == MissionType::kBall) {
    // A new pickup starts a fresh GOAL walk gate only after line reacquisition.
    post_pickup_goal_line_gate_active_ = false;
    post_pickup_line_actions_issued_ = post_pickup_line_actions_completed_ = 0;
    post_pickup_straight_blocks_completed_ = 0;
    post_pickup_five_step_completed_ = false;
    post_pickup_line_actions_.clear();
    ++ball_mission_index_;
    hurdle_controller_.Reset();
    goal_controller_.Reset();
    hurdle_result_ = {};
    goal_result_ = {};
  } else if (mission == MissionType::kHurdle) {
    goal_controller_.Reset();
    ball_result_ = {};
    goal_result_ = {};
    has_ball_ = false;
  } else if (mission == MissionType::kGoal) {
    // Do not carry post-pickup booking limits into LINE after the later shot.
    post_pickup_goal_line_gate_active_ = false;
    post_pickup_line_actions_.clear();
    // Looking again after one LINE block is still the same carrying mission.
    if (!goal_controller_.EntryLineRetryPending())
      ++goal_mission_index_;
    hurdle_controller_.Reset();
    hurdle_result_ = {};
  }
}

void MissionController::FinishMission() {
  if (foot_pose_ != FootPose::kWalk) {
    mission_finish_pending_ = true;
    return;
  }
  last_line_score_.reset();recovering_approach_.reset();
  actual_line_stability_history_.clear();
  if (active_mission_ == MissionType::kBall && has_ball_ &&
      config_.enable_goal) {
    // Start counting only after the BALL mission's confirmed line recovery,
    // not while rotating in place or restoring WALK/FINE foot stance.
    post_pickup_goal_line_gate_active_ = true;
    post_pickup_line_actions_issued_ = 0;
    post_pickup_line_actions_completed_ = 0;
    post_pickup_straight_blocks_completed_ = 0;
    post_pickup_five_step_completed_ = false;
    post_pickup_line_actions_.clear();
  }
  if (config_.test_mission != MissionType::kNone ||
      (active_mission_ == MissionType::kGoal && config_.goal_trial_start_goal))
    goal_trial_complete_ = true;
  active_mission_ = MissionType::kLine;
  if (config_.test_mission == MissionType::kNone) {
    ball_result_ = {};
    hurdle_result_ = {};
    goal_result_ = {};
  }
}

void MissionController::TrackMovingObjectReadyWindow(
    const MissionFrameInput &input) {
  // The interactive tuning adapter owns its own READY window; do not
  // double-apply the production moving-window policy in manual mode.
  if (input.manual_observation_control || !input.command_transport_enabled ||
      input.image_width <= 1 || input.image_height <= 1 ||
      !std::isfinite(input.now_sec)) {
    moving_object_action_id_ = 0;
    moving_object_samples_.clear();
    moving_object_ready_mean_.reset();
    return;
  }
  const MissionAction action = last_command_.action;
  // Shared YAML allowlist, independent of LINE READY/queue eligibility.
  // Turning walks can be enabled explicitly, but their changing camera
  // heading can bias a moving pixel mean; default YAML excludes 24/25.
  const bool eligible_action = config_.UsesObjectReadyMean(action);
  const bool object_mission = active_mission_ == MissionType::kBall ||
                              active_mission_ == MissionType::kGoal ||
                              active_mission_ == MissionType::kHurdle;
  if (!object_mission || !eligible_action ||
      last_command_.action_category != ActionCategory::kLocomotion ||
      last_command_.mission != active_mission_ ||
      last_command_.action_id == 0) {
    moving_object_action_id_ = 0;
    moving_object_samples_.clear();
    moving_object_ready_mean_.reset();
    return;
  }
  if (moving_object_action_id_ != last_command_.action_id) {
    moving_object_action_id_ = last_command_.action_id;
    moving_object_camera_ = input.camera_feedback.actual_mode;
    moving_object_samples_.clear();
    moving_object_ready_mean_.reset();
  }
  if (!input.camera_feedback.settled ||
      input.camera_feedback.actual_mode == CameraMode::kTransition ||
      input.camera_feedback.actual_mode != moving_object_camera_) {
    moving_object_samples_.clear();
    moving_object_ready_mean_.reset();
    return;
  }
  if (moving_object_ready_mean_)
    return;
  const bool matching =
      input.delivery_feedback.action_id == moving_object_action_id_;
  const bool executing =
      last_command_.control_phase == ControlPhase::kWaitingActionDone ||
      (matching && input.delivery_feedback.acknowledged);
  if (!executing || input.delivery_feedback.done)
    return;

  const std::optional<ObjectTarget> *target =
      active_mission_ == MissionType::kBall     ? &input.ball_target
      : active_mission_ == MissionType::kHurdle ? &input.hurdle_target
                                                : &input.backboard_target;
  moving_object_samples_.push_back(
      {input.now_sec, *target,
       active_mission_ == MissionType::kGoal ? input.backboard_center_depth_m
                                             : std::optional<double>{},
       active_mission_ == MissionType::kGoal ? input.goal_pose
                                             : GoalPoseObservation{}});
  // Bound transient evidence if a broken executor delays READY indefinitely.
  if (moving_object_samples_.size() > 450)
    moving_object_samples_.erase(moving_object_samples_.begin());
  if (!matching || !input.delivery_feedback.ready)
    return;

  const double ready_sec = input.now_sec;
  const double start_sec = moving_object_samples_.front().now_sec;
  const double total_sec = std::max(0.0, ready_sec - start_sec) +
                           std::max(0.0, config_.line_p2p.ready_lead_sec);
  const double cutoff = start_sec + 0.40 * total_sec;
  std::vector<const MovingObjectSample *> window;
  for (const auto &sample : moving_object_samples_)
    if (sample.now_sec >= cutoff && sample.now_sec <= ready_sec)
      window.push_back(&sample);
  const int stable_window = active_mission_ == MissionType::kBall ? config_.ball.stable_window :
      active_mission_ == MissionType::kGoal ? config_.goal.stable_window : config_.hurdle.stable_window;
  const int minimum_hits = active_mission_ == MissionType::kBall ? config_.ball.stable_min_hits :
      active_mission_ == MissionType::kGoal ? config_.goal.stable_min_hits : config_.hurdle.stable_min_hits;
  if (window.size() < static_cast<std::size_t>(stable_window))
    return;
  std::optional<std::size_t> qualified_start;
  int recent_hits = 0;
  for (std::size_t i = 0; i < window.size(); ++i) {
    recent_hits += window[i]->target.has_value() ? 1 : 0;
    if (i >= static_cast<std::size_t>(stable_window))
      recent_hits -= window[i-stable_window]->target.has_value() ? 1 : 0;
    if (i+1 >= static_cast<std::size_t>(stable_window) && recent_hits >= minimum_hits)
      qualified_start = i+1-stable_window;
  }
  if (!qualified_start)
    return;

  MovingObjectReadyMean mean;
  mean.action_id = moving_object_action_id_;
  mean.mission = active_mission_;
  mean.camera = moving_object_camera_;
  for (std::size_t i = *qualified_start; i < *qualified_start+stable_window; ++i) {
    const auto &sample = *window[i];
    mean.hits.push_back(sample.target.has_value());
    mean.pose_hits.push_back(
        sample.target && sample.pose.valid && std::isfinite(sample.pose.x_m) &&
        std::isfinite(sample.pose.z_m) && sample.pose.z_m > 0 &&
        std::isfinite(sample.pose.yaw_rad));
    mean.near_hits.push_back(mean.pose_hits.back() &&
                             sample.pose.z_m <=
                                 config_.goal.fine_adjust_start_z_m);
  }
  double sum_u = 0.0, sum_v = 0.0, sum_ru = 0.0, sum_rv = 0.0;
  double sum_x = 0.0, sum_y = 0.0, sum_w = 0.0, sum_h = 0.0;
  double sum_conf = 0.0, total_weight = 0.0;
  double depth_sum = 0.0, pose_x = 0.0, pose_z = 0.0;
  double pose_sin = 0.0, pose_cos = 0.0;
  int hits = 0, depth_hits = 0, pose_hits = 0;
  for (const auto *sample : window) {
    if (!sample->target)
      continue;
    mean.target = *sample->target;
    ++hits;
    const double weight =
        active_mission_ == MissionType::kGoal
            ? 1.0
            : 1.0 + 2.0 * std::clamp((sample->now_sec - cutoff) /
                                         std::max(1e-9, ready_sec - cutoff),
                                     0.0, 1.0);
    total_weight += weight;
    const auto &t = *sample->target;
    sum_u += weight * t.center_px.u;
    sum_v += weight * t.center_px.v;
    // Some frames may lack IMU rectification; never average unset (0,0).
    const Point2 rect =
        t.center_rectified ? t.rectified_center_px : t.center_px;
    sum_ru += weight * rect.u;
    sum_rv += weight * rect.v;
    sum_x += weight * t.box_px.x;
    sum_y += weight * t.box_px.y;
    sum_w += weight * t.box_px.width;
    sum_h += weight * t.box_px.height;
    sum_conf += weight * t.confidence;
    if (active_mission_ == MissionType::kGoal) {
      if (sample->center_depth_m && std::isfinite(*sample->center_depth_m) &&
          *sample->center_depth_m > 0.0) {
        depth_sum += *sample->center_depth_m;
        ++depth_hits;
      }
      if (sample->pose.valid && std::isfinite(sample->pose.x_m) &&
          std::isfinite(sample->pose.z_m) &&
          std::isfinite(sample->pose.yaw_rad)) {
        pose_x += sample->pose.x_m;
        pose_z += sample->pose.z_m;
        pose_sin += std::sin(sample->pose.yaw_rad);
        pose_cos += std::cos(sample->pose.yaw_rad);
        ++pose_hits;
      }
    }
  }
  if (hits < minimum_hits || total_weight <= 0.0)
    return;
  if (active_mission_ == MissionType::kGoal && depth_hits < minimum_hits)
    return;
  mean.target.center_px = {sum_u / total_weight, sum_v / total_weight};
  mean.target.rectified_center_px = {sum_ru / total_weight,
                                     sum_rv / total_weight};
  mean.target.box_px = {sum_x / total_weight, sum_y / total_weight,
                        sum_w / total_weight, sum_h / total_weight};
  mean.target.width_px = mean.target.box_px.width;
  mean.target.height_px = mean.target.box_px.height;
  mean.target.area_px = mean.target.width_px * mean.target.height_px;
  mean.target.confidence = sum_conf / total_weight;
  if (depth_hits >= minimum_hits)
    mean.center_depth_m = depth_sum / static_cast<double>(depth_hits);
  if (pose_hits >= minimum_hits) {
    mean.pose.valid = true;
    mean.pose.x_m = pose_x / pose_hits;
    mean.pose.z_m = pose_z / pose_hits;
    mean.pose.yaw_rad = std::atan2(pose_sin, pose_cos);
    mean.pose.confidence = mean.target.confidence;
  }
  moving_object_ready_mean_ = mean;
}

bool MissionController::ShouldDeferDone(
    const CommandDeliveryFeedback &feedback) const {
  if (!feedback.done || config_.post_motion_observation_sec <= 0.0 ||
      feedback.action_id == 0 || feedback.action_id != last_command_.action_id)
    return false;
  if (RepeatScope() != 0)
    return false;
  // DOWN recovery owns its fresh post-DONE window and bounded FINE search.
  // Deliver real DONE now; a second generic window would delay every attempt.
  if (active_mission_==MissionType::kBall && ball_result_.mode==BallMode::kBallRecoveryDown)
    return false;
  if (active_mission_==MissionType::kGoal && goal_controller_.DepthRecoveryActive())
    return false;
  const MissionAction action = last_command_.action;
  const MissionType mission = last_command_.mission;
  // The READY queue and the existing 1s/2s LINE recovery observation
  // must not be delayed by the generic post-motion gate.
  // Long LINE READY skips the gate ONLY while the active mission remains LINE.
  // An object entered during that walk must still collect fresh post-DONE data.
  if (mission == MissionType::kLine && active_mission_ == MissionType::kLine &&
      (IsLongLineAction(action) || (line_state_ != LineState::kNormal &&
                                    line_state_ != LineState::kScoreRecovery)))
    return false;
  // If the mission has already committed to moving the camera, let the
  // camera command run; its DONE starts the fresh 1 s object-view window.
  if ((active_mission_ == MissionType::kBall &&
       ball_result_.mode == BallMode::kWaitCameraDown) ||
      (active_mission_ == MissionType::kHurdle &&
       hurdle_result_.mode == HurdleMode::kWaitCameraDown) ||
      (active_mission_ == MissionType::kGoal &&
       goal_result_.mode == GoalMode::kWaitCameraGoal))
    return false;
  if (action == MissionAction::kDefaultPoseMode ||
      action == MissionAction::kWalkMode ||
      (active_mission_ == MissionType::kBall &&
       ball_result_.mode == BallMode::kApproachTwoBeforeDown) ||
      (active_mission_ == MissionType::kHurdle &&
       hurdle_result_.mode == HurdleMode::kApproachTwoBeforeDown))
    return false;
  // Fixed multi-motion chains have no perception-dependent successor.
  if (action == MissionAction::kPickBall || action == MissionAction::kRecatch ||
      action == MissionAction::kDefaultPosition ||
      action == MissionAction::kShoot ||
      (mission == MissionType::kBall && action == MissionAction::kStepBack &&
       ball_result_.mode == BallMode::kPostPickupBackAway) ||
      (mission == MissionType::kHurdle && action == MissionAction::kHurdle))
    return false;
  return true;
}

void MissionController::BeginDecisionObservation(
    double now_sec, MissionType mission, bool camera_only,
    const CommandDeliveryFeedback &done) {
  decision_observation_active_ = true;
  decision_observation_camera_only_ = camera_only;
  decision_observation_first_frame_ = true;
  decision_observation_mission_ = mission;
  decision_observation_start_sec_ = std::max(now_sec, settle_until_sec_);
  deferred_done_ = done;
  decision_object_hits_.clear();
  decision_qualified_object_hits_.clear();decision_object_qualified_=false;
  decision_goal_pose_hits_.clear();
  decision_qualified_goal_pose_hits_.clear();decision_goal_pose_qualified_=false;
  decision_goal_near_hits_.clear();
  decision_observation_total_frames_ = 0;
  decision_object_weight_ = 0.0;
  decision_last_target_.reset();
  decision_target_u_sum_ = decision_target_v_sum_ = 0.0;
  decision_rect_u_sum_ = decision_rect_v_sum_ = 0.0;
  decision_target_count_ = 0;
  decision_pose_x_sum_ = decision_pose_z_sum_ = 0.0;
  decision_pose_sin_sum_ = decision_pose_cos_sum_ = 0.0;
  decision_pose_count_ = 0;
  decision_line_valid_count_ = 0;
  decision_line_hits_.clear();decision_line_qualified_=false;
  decision_line_offset_sum_ = decision_line_heading_sum_ = 0.0;
  decision_line_confidence_sum_ = 0.0;
  if (!camera_only) {
    // Never reuse target stability or O/H evidence from the moving action.
    if (!ball_target_locked_ && !ball_reacquiring_) ball_association_tracker_.Reset();
    if (candidate_.mission != MissionType::kHurdle && active_mission_ != MissionType::kHurdle)
      hurdle_target_tracker_.ClearUnpinned();
    if (active_mission_ != MissionType::kGoal) backboard_target_tracker_.ClearUnpinned();
    ball_controller_.ClearObservationHistory();
    hurdle_controller_.ClearObservationHistory();
    goal_controller_.ClearObservationHistory();
    line_stability_history_.clear();
    actual_line_stability_history_.clear();
  }
}

void MissionController::AddDecisionObservation(const MissionFrameInput &input) {
  if (!decision_observation_active_ ||
      input.now_sec + 1e-9 < decision_observation_start_sec_)
    return;
  ++decision_observation_total_frames_;
  const std::optional<ObjectTarget> *target = nullptr;
  switch (decision_observation_mission_) {
  case MissionType::kBall:
    target = &input.ball_target;
    break;
  case MissionType::kHurdle:
    target = &input.hurdle_target;
    break;
  case MissionType::kGoal:
    target = &input.backboard_target;
    break;
  default:
    break;
  }
  if (decision_observation_mission_ != MissionType::kLine) {
    const bool detected = target && target->has_value();
    decision_object_hits_.push_back(detected);
    const int window=decision_observation_mission_==MissionType::kBall ?
        config_.ball.stable_window : decision_observation_mission_==MissionType::kGoal ?
        config_.goal.stable_window : config_.hurdle.stable_window;
    while (decision_object_hits_.size()>static_cast<std::size_t>(window))
      decision_object_hits_.pop_front();
    const int minimum=decision_observation_mission_==MissionType::kBall ?
        config_.ball.stable_min_hits : decision_observation_mission_==MissionType::kGoal ?
        config_.goal.stable_min_hits : config_.hurdle.stable_min_hits;
    if (decision_object_hits_.size()==static_cast<std::size_t>(window) &&
        std::count(decision_object_hits_.begin(),decision_object_hits_.end(),true)>=minimum) {
      decision_object_qualified_=true;
      decision_qualified_object_hits_=decision_object_hits_;
    }
    if (decision_observation_mission_ == MissionType::kGoal) {
      goal_controller_.ObserveEntryBackboard(
          detected || input.backboard_image_without_depth);
      const bool pose_valid = detected && input.goal_pose.valid &&
                              std::isfinite(input.goal_pose.x_m) &&
                              std::isfinite(input.goal_pose.z_m) &&
                              std::isfinite(input.goal_pose.yaw_rad);
      decision_goal_pose_hits_.push_back(pose_valid);
      decision_goal_near_hits_.push_back(
          pose_valid &&
          input.goal_pose.z_m <= config_.goal.fine_adjust_start_z_m);
      while (decision_goal_near_hits_.size()>static_cast<std::size_t>(config_.goal.stable_window))
        decision_goal_near_hits_.pop_front();
      while (decision_goal_pose_hits_.size()>static_cast<std::size_t>(config_.goal.stable_window))
        decision_goal_pose_hits_.pop_front();
      if (decision_goal_pose_hits_.size()==static_cast<std::size_t>(config_.goal.stable_window) &&
          std::count(decision_goal_pose_hits_.begin(),decision_goal_pose_hits_.end(),true)>=
              config_.goal.stable_min_hits) {
        decision_goal_pose_qualified_=true;
        decision_qualified_goal_pose_hits_=decision_goal_pose_hits_;
      }
    }
    if (detected) {
      const ObjectTarget &t = target->value();
      decision_last_target_ = t;
      const bool fine_adjustment =
          (decision_observation_mission_ == MissionType::kBall &&
           ball_result_.mode == BallMode::kFineAdjustForPickup) ||
          (decision_observation_mission_ == MissionType::kHurdle &&
           hurdle_result_.mode == HurdleMode::kFineAdjust) ||
          decision_observation_mission_ == MissionType::kGoal;
      const double weight =
          fine_adjustment
              ? 1.0
              : 1.0 +
                    2.0 *
                        std::clamp(
                            (input.now_sec - decision_observation_start_sec_) /
                                std::max(1e-9,
                                         config_.post_motion_observation_sec),
                            0.0, 1.0);
      decision_object_weight_ += weight;
      decision_target_u_sum_ += weight * t.center_px.u;
      decision_target_v_sum_ += weight * t.center_px.v;
      const Point2 rect =
          t.center_rectified ? t.rectified_center_px : t.center_px;
      decision_rect_u_sum_ += weight * rect.u;
      decision_rect_v_sum_ += weight * rect.v;
      ++decision_target_count_;
    }
    if (decision_observation_mission_ == MissionType::kGoal &&
        input.goal_pose.valid && std::isfinite(input.goal_pose.x_m) &&
        std::isfinite(input.goal_pose.z_m) &&
        std::isfinite(input.goal_pose.yaw_rad)) {
      decision_pose_x_sum_ += input.goal_pose.x_m;
      decision_pose_z_sum_ += input.goal_pose.z_m;
      decision_pose_sin_sum_ += std::sin(input.goal_pose.yaw_rad);
      decision_pose_cos_sum_ += std::cos(input.goal_pose.yaw_rad);
      ++decision_pose_count_;
    }
    return;
  }
  const LineGuide guide =
      input.line_decision_guide_override
          ? *input.line_decision_guide_override
          : ComputeLineFeatures(input.line_centers, input.image_width,
                                input.image_height, config_.line_features)
                .guide;
  const bool valid=guide.valid && std::isfinite(guide.offset) &&
      std::isfinite(guide.heading_rad);
  decision_line_hits_.push_back(valid);
  while (decision_line_hits_.size()>static_cast<std::size_t>(config_.line.line_stable_window))
    decision_line_hits_.pop_front();
  if (decision_line_hits_.size()==static_cast<std::size_t>(config_.line.line_stable_window) &&
      std::count(decision_line_hits_.begin(),decision_line_hits_.end(),true)>=
          config_.line.line_stable_min_hits) decision_line_qualified_=true;
  if (valid) {
    ++decision_line_valid_count_;
    decision_line_offset_sum_ += guide.offset;
    decision_line_heading_sum_ += guide.heading_rad;
    if (std::isfinite(guide.confidence))
      decision_line_confidence_sum_ += guide.confidence;
  }
}

bool MissionController::DecisionObservationReady(double now_sec) const {
  if (!decision_observation_active_ ||
      now_sec - decision_observation_start_sec_ + 1e-9 <
          config_.post_motion_observation_sec)
    return false;
  if (decision_observation_mission_ == MissionType::kLine)
    return decision_line_hits_.size() >=
        static_cast<std::size_t>(config_.line.line_stable_window);
  // Time and real frames accumulate concurrently. The completed window
  // validates 10/7 once; controllers reuse that evidence without recollection.
  const int window=decision_observation_mission_==MissionType::kBall ? config_.ball.stable_window :
      decision_observation_mission_==MissionType::kGoal ? config_.goal.stable_window :
      config_.hurdle.stable_window;
  return decision_object_hits_.size()>=static_cast<std::size_t>(window);
}

std::optional<LineGuide> MissionController::DecisionLineGuide() const {
  if (!decision_line_qualified_ || decision_line_valid_count_ <
      std::max(1, config_.line_p2p.failure_min_valid_samples))
    return std::nullopt;
  LineGuide guide;
  guide.valid = true;
  guide.offset = decision_line_offset_sum_ / decision_line_valid_count_;
  guide.heading_rad = decision_line_heading_sum_ / decision_line_valid_count_;
  guide.confidence = decision_line_confidence_sum_ / decision_line_valid_count_;
  return guide;
}

MissionFrameResult MissionController::Step(const MissionFrameInput &input) {
  return StepWithLineImageCenter(input, std::nullopt);
}

MissionFrameResult MissionController::StepWithLineImageCenter(
    const MissionFrameInput &source, std::optional<double> line_image_center_u,
    bool evaluating_ready_successor) {
  MissionFrameInput input = source;
  if (decision_observation_restart_pending_) {
    decision_observation_restart_pending_ = false;
    if (decision_observation_active_)
      BeginDecisionObservation(input.now_sec, decision_observation_mission_,
                               decision_observation_camera_only_,
                               deferred_done_);
  }
  if (post_pickup_goal_line_gate_active_ && input.delivery_feedback.done &&
      input.delivery_feedback.action_id != 0) {
    const auto it =
        post_pickup_line_actions_.find(input.delivery_feedback.action_id);
    if (it != post_pickup_line_actions_.end()) {
      // The unique action ID is counted exactly once, only at physical DONE.
      ++post_pickup_line_actions_completed_;
      if (it->second == MissionAction::kStepForwardFour)
        ++post_pickup_straight_blocks_completed_;
      post_pickup_five_step_completed_ = StraightGoalGateReached();
      post_pickup_line_actions_.erase(it);
    }
  }
  // Normal LINE planning and pre-entry candidates share the existing
  // coordinator; object/test mission FSMs below retain their execution path.
  if (config_.goal_trial_start_goal &&
      config_.test_mission==MissionType::kNone && !input.manual_observation_control &&
      active_mission_==MissionType::kGoal && goal_result_.mode==GoalMode::kWaitCameraGoal) {
    active_mission_=MissionType::kLine;
    approach_goal_probe_=true;
    goal_start_camera_pending_=false; // The shared router now owns GOAL discovery.
  }
  if (active_mission_ == MissionType::kLine &&
      (config_.test_mission == MissionType::kNone || goal_controller_.EntryLineRetryPending()) &&
      !goal_trial_complete_ && !physical_reset_pending_)
    return StepPlannedLine(input, line_image_center_u);
  TrackMovingObjectReadyWindow(input);
  // Compute B on a disposable DONE simulation. Never advance A's live FSM
  // until its real DONE. Manual tuning keeps Preview/Commit as the only
  // admission.
  std::optional<ActionRequest> ready_object_successor;
  std::optional<MissionType> ready_object_successor_mission;
  if (!input.manual_observation_control && input.command_transport_enabled &&
      input.delivery_feedback.ready && !input.delivery_feedback.done &&
      moving_object_ready_mean_ &&
      moving_object_ready_mean_->action_id ==
          input.delivery_feedback.action_id &&
      active_mission_ == moving_object_ready_mean_->mission &&
      last_command_.action_id == moving_object_ready_mean_->action_id &&
      input.allow_new_commands && input.allow_new_body_actions) {
    MissionController speculative = *this;
    MissionFrameInput complete = input;
    complete.delivery_feedback = {input.delivery_feedback.action_id, true, true,
                                  false};
    const MissionFrameResult proposed = speculative.StepWithLineImageCenter(
        complete, line_image_center_u, true);
    const ControlCommand &next = proposed.command;
    if (next.command_type == CommandType::kAction && next.action_id != 0 &&
        next.action != MissionAction::kNone &&
        next.camera_request == CameraRequest::kNone) {
      ActionRequest b;
      b.action = next.action;
      b.category = next.action_category;
      b.target_yaw_deg = next.target_yaw_deg;
      b.line_queue_eligible = next.mission == MissionType::kLine
                                  ? IsLongLineAction(b.action)
                                  : config_.UsesObjectReadyMean(b.action);
      ready_object_successor = b;
      ready_object_successor_mission = next.mission;
    }
  }
  bool applied_object_ready_mean = false;
  // A READY mean alone is not a reserved successor. Without B in the
  // one-slot queue, retain the fresh 1s post-DONE observation gate.
  // Only the disposable candidate calculation may use the mean before B
  // exists; requiring B there would prevent the first reservation entirely.
  if (moving_object_ready_mean_ && input.delivery_feedback.done &&
      input.delivery_feedback.action_id ==
          moving_object_ready_mean_->action_id &&
      (evaluating_ready_successor ||
       (object_queued_parent_id_ == input.delivery_feedback.action_id &&
        object_queued_successor_.has_value())) &&
      active_mission_ == moving_object_ready_mean_->mission &&
      input.camera_feedback.settled &&
      input.camera_feedback.actual_mode == moving_object_ready_mean_->camera) {
    const auto mean = *moving_object_ready_mean_;
    // Import the actual window's hit/miss history with the cached mean.
    if (active_mission_ == MissionType::kBall) {
      input.ball_target = mean.target;
      ball_controller_.UseVerifiedObservation(
          input.ball_target, mean.hits, input.image_width, input.image_height);
    } else if (active_mission_ == MissionType::kHurdle) {
      input.hurdle_target = mean.target;
      hurdle_controller_.UseVerifiedObservation(input.hurdle_target, mean.hits,
                                                input.image_width,
                                                input.image_height);
    } else {
      input.backboard_target = mean.target;
      input.backboard_center_depth_m = mean.center_depth_m;
      input.goal_pose = mean.pose;
      goal_controller_.ClearMovingApproachTriggers();
      goal_controller_.UseVerifiedObservation(
          input.backboard_target, input.goal_pose, mean.hits, mean.pose_hits,
          input.image_width, input.image_height, mean.near_hits);
    }
    applied_object_ready_mean = true;
  }
  if (input.manual_observation_control && input.manual_ready_mean &&
      input.manual_object_hits.size() == 10 &&
      std::count(input.manual_object_hits.begin(),
                 input.manual_object_hits.end(), true) >= 7) {
    if (active_mission_ == MissionType::kBall)
      ball_controller_.UseVerifiedObservation(
          input.ball_target, input.manual_object_hits, input.image_width,
          input.image_height);
    else if (active_mission_ == MissionType::kHurdle)
      hurdle_controller_.UseVerifiedObservation(
          input.hurdle_target, input.manual_object_hits, input.image_width,
          input.image_height);
    else if (active_mission_ == MissionType::kGoal)
      goal_controller_.UseVerifiedObservation(
          input.backboard_target, input.goal_pose, input.manual_object_hits,
          input.manual_pose_hits, input.image_width, input.image_height,
          input.manual_near_hits);
    applied_object_ready_mean = true;
  }
  if (input.delivery_feedback.done) {
    moving_object_action_id_ = 0;
    moving_object_samples_.clear();
    moving_object_ready_mean_.reset();
  }
  // Consume the real completion edge for timing before any observation gate
  // defers it. Stationary post-DONE samples must never count as walking.
  command_coordinator_.ObserveExecutionTime(input.delivery_feedback,
                                            input.now_sec);
  const bool timing_done =
      input.delivery_feedback.done && last_command_.action_id != 0 &&
      input.delivery_feedback.action_id == last_command_.action_id &&
      last_settle_done_id_ != input.delivery_feedback.action_id;
  if (timing_done) {
    last_settle_done_id_ = input.delivery_feedback.action_id;
    if (last_command_.mission == MissionType::kLine &&
        (last_command_.action == MissionAction::kTurnLeftInPlace ||
         last_command_.action == MissionAction::kStepForwardRightOneHurdle))
      ++line_guard_turns_completed_;
    const auto action = last_command_.action;
    const int number = static_cast<int>(action);
    const bool transition = action == MissionAction::kDefaultPoseMode ||
                            action == MissionAction::kWalkMode ||
                            action == MissionAction::kHurdle;
    const bool intermediate_fine =
        active_mission_ == MissionType::kBall &&
        (ball_result_.mode == BallMode::kFineSidePair ||
         ball_result_.mode == BallMode::kFineMotionRepeat) &&
        ball_controller_.FineMotionActionsRemaining() > 1;
    const double delay = transition
                             ? config_.transition_settle_sec
                             : (number >= 3 && number <= 8 && !intermediate_fine
                                    ? config_.fine_motion_settle_sec
                                    : 0.0);
    settle_until_sec_ = std::max(settle_until_sec_, input.now_sec + delay);
    if (last_command_.mission == MissionType::kHurdle)
      hurdle_controller_.OnActualMotionDone(action, settle_until_sec_);
  }
  const bool pose_done =
      input.delivery_feedback.done && last_command_.action_id != 0 &&
      input.delivery_feedback.action_id == last_command_.action_id &&
      (last_command_.action == MissionAction::kDefaultPoseMode ||
       last_command_.action == MissionAction::kWalkMode);
  if (pose_done && !input.manual_observation_control)
    pose_observation_pending_ = true;
  if (pose_done)
    foot_pose_ = last_command_.action == MissionAction::kWalkMode
                     ? FootPose::kWalk
                     : FootPose::kFine;
  if (pose_change_pending_) {
    MissionFrameResult pose_result;
    pose_result.command = command_coordinator_.Compute(
        active_mission_, 901, {}, CameraRequest::kNone, input.delivery_feedback,
        input.now_sec);
    if (pose_result.command.action_id != 0 || pose_done)
      last_command_ = pose_result.command;
    if (pose_done) {
      pose_change_pending_ = false;
      const double pose_duration =
          std::max(0.0, input.now_sec - pose_started_sec_);
      ball_controller_.ShiftTimers(pose_duration);
      hurdle_controller_.ShiftTimers(pose_duration);
      goal_controller_.ShiftTimers(pose_duration);
      ClearObservationEvidence();
      if (mission_finish_pending_) {
        mission_finish_pending_ = false;
        FinishMission();
      }
    }
    pose_result.active_mission = active_mission_;
    pose_result.has_ball = has_ball_;
    pose_result.ball = ball_result_;
    pose_result.goal = goal_result_;
    pose_result.hurdle = hurdle_result_;
    pose_result.trial_complete = goal_trial_complete_;
    PopulateDiagnostics(pose_result, input.now_sec);
    return pose_result;
  }
  if (mission_finish_pending_) {
    MissionFrameResult pose_result;
    const ActionRequest pose_request =
        input.allow_new_commands && input.allow_new_body_actions
            ? ActionRequest{MissionAction::kWalkMode,
                            ActionCategory::kLocomotion, 0, false}
            : ActionRequest{};
    pose_result.command = command_coordinator_.Compute(
        active_mission_, 901, pose_request, CameraRequest::kNone,
        input.delivery_feedback, input.now_sec);
    if (pose_result.command.action_id != 0) {
      pose_started_sec_ = input.now_sec;
      pose_change_pending_ = true;
      pose_target_ = FootPose::kWalk;
    }
    last_command_ = pose_result.command;
    pose_result.active_mission = active_mission_;
    pose_result.has_ball = has_ball_;
    pose_result.ball = ball_result_;
    pose_result.goal = goal_result_;
    pose_result.hurdle = hurdle_result_;
    PopulateDiagnostics(pose_result, input.now_sec);
    return pose_result;
  }
  const int incoming_repeat_scope = RepeatScope();
  MissionFrameResult output;
  output.object_ready_mean_used = applied_object_ready_mean;
  if (goal_trial_complete_ || test_failed_) {
    output.trial_complete = goal_trial_complete_;
    output.goal_mission_index = std::max(1, goal_mission_index_);
    output.ball_mission_index = std::max(1, ball_mission_index_);
    output.ball = ball_result_;
    output.hurdle = hurdle_result_;
    output.goal = goal_result_;
    output.active_mission = active_mission_;
    output.has_ball = has_ball_;
    PopulateDiagnostics(output, input.now_sec);
    return output;
  }
  if (test_setup_pending_) {
    const CameraMode wanted =
        config_.test_stage == 0 || (config_.test_stage == 1 &&
                                    config_.test_mission != MissionType::kGoal)
            ? CameraMode::kForward
            : (config_.test_mission == MissionType::kGoal ? CameraMode::kGoal
                                                          : CameraMode::kDown);
    const bool camera_ready = input.camera_feedback.settled &&
                              input.camera_feedback.actual_mode == wanted;
    const MissionAction initial_pose = config_.test_stage == 3
                                           ? MissionAction::kDefaultPoseMode
                                           : MissionAction::kWalkMode;
    if (pose_done && last_command_.action == initial_pose)
      test_setup_pose_done_ = true;
    CameraRequest camera_request = CameraRequest::kNone;
    ActionRequest pose;
    if (!camera_ready && input.allow_new_commands) {
      if (!test_setup_camera_started_) {
        test_setup_camera_started_ = true;
        test_setup_camera_start_sec_ = input.now_sec;
      }
      if (input.now_sec - test_setup_camera_start_sec_ >=
          config_.camera_motion_timeout_sec)
        test_failed_ = true;
    }
    if (!camera_ready)
      camera_request = wanted == CameraMode::kGoal   ? CameraRequest::kGoal
                       : wanted == CameraMode::kDown ? CameraRequest::kDown
                                                     : CameraRequest::kForward;
    if (!test_setup_pose_done_)
      pose = {initial_pose, ActionCategory::kLocomotion, 0, false};
    if (camera_ready && test_setup_pose_done_) {
      test_setup_pending_ = false;
      ClearObservationEvidence();
      test_entry_pending_ =
          config_.test_stage <= 1 && config_.test_mission != MissionType::kGoal;
      if (config_.test_stage <= 1) {
        if (config_.test_mission == MissionType::kGoal) {
          if (config_.test_stage == 0) {
            goal_controller_.StartForwardTrial();
            goal_result_.mode = GoalMode::kForwardApproach;
          } else {
            goal_controller_.StartTestNear();
            goal_result_.mode = GoalMode::kWaitCameraGoal;
          }
        }
      } else if (config_.test_mission == MissionType::kBall) {
        if (config_.test_stage == 2)
          ball_controller_.StartTestNear();
        else
          ball_controller_.StartTestExecute();
        ball_result_.mode = config_.test_stage == 2
                                ? BallMode::kWaitCameraDown
                                : BallMode::kFineAdjustForPickup;
      } else if (config_.test_mission == MissionType::kHurdle) {
        if (config_.test_stage == 2)
          hurdle_controller_.StartTestNear();
        else
          hurdle_controller_.StartTestExecute();
        hurdle_result_.mode = config_.test_stage == 2
                                  ? HurdleMode::kWaitCameraDown
                                  : HurdleMode::kFineAdjust;
      } else {
        if (config_.test_stage <= 2)
          goal_controller_.StartTestNear();
        else
          goal_controller_.StartTestExecute();
        goal_result_.mode = config_.test_stage <= 2 ? GoalMode::kWaitCameraGoal
                                                    : GoalMode::kFineAdjust;
      }
    }
    if (!input.allow_new_body_actions && !input.allow_test_setup_body_actions)
      pose = {};
    if (!input.allow_new_commands || test_failed_) {
      pose = {};
      camera_request = CameraRequest::kNone;
    }
    output.command =
        command_coordinator_.Compute(active_mission_, 903, pose, camera_request,
                                     input.delivery_feedback, input.now_sec);
    last_command_ = output.command;
    output.active_mission = active_mission_;
    output.has_ball = has_ball_;
    output.ball = ball_result_;
    output.goal = goal_result_;
    output.hurdle = hurdle_result_;
    PopulateDiagnostics(output, input.now_sec);
    return output;
  }
  if (goal_start_camera_pending_) {
    // Explicit direct-GOAL START must issue its own camera command before
    // judging any backboard, even if cached feedback already says GOAL.
    const auto camera =
        input.allow_new_commands ? CameraRequest::kGoal : CameraRequest::kNone;
    output.command = command_coordinator_.Compute(
        active_mission_, static_cast<int>(GoalMode::kWaitCameraGoal), {},
        camera, input.delivery_feedback, input.now_sec);
    if (input.allow_new_commands) {
      goal_start_camera_pending_ = false;
      previous_camera_settled_ = false;
      previous_camera_mode_ = CameraMode::kTransition;
    }
    last_command_ = output.command;
    output.active_mission = active_mission_;
    output.has_ball = has_ball_;
    output.goal = goal_result_;
    PopulateDiagnostics(output, input.now_sec);
    return output;
  }
  if (input.image_width > 1)
    last_line_image_width_ = input.image_width;
  // Observe the camera transition edge even if no ActionCommand is pending.
  const bool camera_settle_edge =
      (!previous_camera_settled_ ||
       previous_camera_mode_ == CameraMode::kTransition) &&
      input.camera_feedback.settled &&
      (input.camera_feedback.actual_mode == CameraMode::kDown ||
       input.camera_feedback.actual_mode == CameraMode::kGoal);
  if (camera_settle_edge)
    settle_until_sec_ = std::max(settle_until_sec_,
                                 input.now_sec + config_.transition_settle_sec);
  previous_camera_settled_ = input.camera_feedback.settled;
  previous_camera_mode_ = input.camera_feedback.actual_mode;
  if (input.now_sec + 1e-9 < settle_until_sec_ && !timing_done &&
      !camera_settle_edge && !decision_observation_active_ &&
      !input.begin_manual_observation) {
    output.active_mission = active_mission_;
    output.has_ball = has_ball_;
    output.ball = ball_result_;
    output.goal = goal_result_;
    output.hurdle = hurdle_result_;
    output.command = command_coordinator_.Compute(
        active_mission_, 0, {}, CameraRequest::kNone, input.delivery_feedback,
        input.now_sec);
    PopulateDiagnostics(output, input.now_sec);
    return output;
  }
  if (active_mission_ == MissionType::kHurdle &&
      input.camera_feedback.actual_mode == CameraMode::kDown &&
      input.camera_feedback.settled &&
      input.now_sec + 1e-9 >= settle_until_sec_) {
    FeatureConfig hurdle_line_config = config_.line_features;
    hurdle_line_config.image_center_u = config_.hurdle.fine_line_reference_u_px;
    hurdle_line_config.guide_min_points = 3;
    hurdle_line_config.ransac_enabled = false;
    const auto guide = ComputeLineFeatures(
        input.line_centers, input.image_width, input.image_height,
        hurdle_line_config, 3).guide;
    hurdle_controller_.ObserveFineLine(guide, input.image_width, input.now_sec);
  }
  if (fresh_after_search_) {
    fresh_after_search_ = false;
    if (!input.manual_observation_control)
      BeginDecisionObservation(input.now_sec, active_mission_, false, {});
  }
  // Deliver the pose's actual DONE to its owning FSM first. Its fresh
  // observation begins on the next frame and must never swallow that edge.
  if (pose_observation_pending_ && !input.manual_observation_control &&
      !pose_done) {
    pose_observation_pending_ = false;
    BeginDecisionObservation(input.now_sec, active_mission_, false, {});
  }
  if (input.manual_observation_control && RepeatScope() == 0) {
    // DONE is always acknowledged by the coordinator immediately. Only a
    // deliberate PREVIEW may start the next one-second stationary window.
    if (input.begin_manual_observation)
      BeginDecisionObservation(input.now_sec, active_mission_, false, {});
  } else if (RepeatScope() == 0) {
    if (!decision_observation_active_ &&
        config_.post_motion_observation_sec > 0.0 && camera_settle_edge)
      BeginDecisionObservation(input.now_sec, active_mission_, true, {});
    if (!decision_observation_active_ && !applied_object_ready_mean &&
        ShouldDeferDone(input.delivery_feedback))
      BeginDecisionObservation(input.now_sec, active_mission_, false,
                               input.delivery_feedback);
  }
  // A deferred DONE remains pending at the coordinator while new, post-DONE
  // frames are collected. This makes the next action impossible before 1 s.
  bool observation_finished = false;
  bool observation_hold = decision_observation_active_;
  if (observation_hold) {
    if (!decision_observation_first_frame_ ||
        !decision_observation_camera_only_)
      AddDecisionObservation(input);
    if (DecisionObservationReady(input.now_sec)) {
      observation_finished = true;
      if (!decision_observation_camera_only_)
        input.delivery_feedback = deferred_done_;
      // Object acquisition 10/7 applies only on entry. After DONE one
      // second of stationary samples defines the next measurement.
      if (decision_observation_mission_ != MissionType::kLine) {
        const bool stable_object=decision_object_qualified_;
        // A single last-minute false detection is NOT a stationary mean.
        std::optional<ObjectTarget> mean;
        if (stable_object && decision_last_target_ && decision_target_count_ > 0) {
          mean = *decision_last_target_;
          const double count = decision_object_weight_;
          mean->center_px = {decision_target_u_sum_ / count,
                             decision_target_v_sum_ / count};
          if (mean->center_rectified)
            mean->rectified_center_px = {decision_rect_u_sum_ / count,
                                         decision_rect_v_sum_ / count};
        }
        if (decision_observation_mission_ == MissionType::kBall) {
          input.ball_target = mean;
          ball_controller_.UseVerifiedObservation(mean, decision_qualified_object_hits_,
                                                  input.image_width,
                                                  input.image_height);
        } else if (decision_observation_mission_ == MissionType::kHurdle) {
          input.hurdle_target = mean;
          hurdle_controller_.UseVerifiedObservation(mean, decision_qualified_object_hits_,
                                                    input.image_width,
                                                    input.image_height);
        } else if (decision_observation_mission_ == MissionType::kGoal) {
          input.backboard_target = mean;
          input.goal_pose = {};

          if (mean && decision_goal_pose_qualified_ && decision_pose_count_ > 0) {
            const double count = static_cast<double>(decision_pose_count_);
            input.goal_pose.valid = true;
            input.goal_pose.x_m = decision_pose_x_sum_ / count;
            input.goal_pose.z_m = decision_pose_z_sum_ / count;
            input.goal_pose.yaw_rad =
                std::atan2(decision_pose_sin_sum_, decision_pose_cos_sum_);
            input.goal_pose.confidence = mean->confidence;
          }
          goal_controller_.UseVerifiedObservation(
              mean, input.goal_pose, decision_qualified_object_hits_,
              decision_qualified_goal_pose_hits_, input.image_width, input.image_height,
              decision_goal_near_hits_);
        }
      }
      if (decision_observation_mission_ == MissionType::kLine) {
        const auto averaged = DecisionLineGuide();
        if (averaged)
          input.line_decision_guide_override = *averaged;
        else {
          LineGuide invalid;
          input.line_decision_guide_override = invalid;
        }
      }
      decision_observation_active_ = false;
      observation_hold = false;
    } else if (!decision_observation_camera_only_ &&
               deferred_done_.action_id != 0) {
      input.delivery_feedback = {deferred_done_.action_id, true, false, false};
    } else {
      input.delivery_feedback = {};
    }
  }
  // During a gate, collect fresh evidence without advancing the mission phase.
  // Only the first camera-settled frame transitions WAIT_CAMERA into the new
  // view; subsequent frames update tracking but do not choose an action.
  const bool camera_settle_transition_frame =
      observation_hold && decision_observation_camera_only_ &&
      decision_observation_first_frame_;
  if (observation_hold && !camera_settle_transition_frame) {
    output.line_features =
        ComputeLineFeatures(input.line_centers, input.image_width,
                            input.image_height, config_.line_features);
    output.line_computed = true;
    const bool collecting =
        input.now_sec + 1e-9 >= decision_observation_start_sec_;
    if (collecting && active_mission_ == MissionType::kBall)
      ball_controller_.ObserveOnly(input.ball_target, input.image_width,
                                   input.image_height);
    else if (collecting && active_mission_ == MissionType::kHurdle)
      hurdle_controller_.ObserveOnly(input.hurdle_target, input.image_width,
                                     input.image_height);
    else if (collecting && active_mission_ == MissionType::kGoal)
      goal_controller_.ObserveOnly(input.backboard_target, input.goal_pose,
                                   input.image_width, input.image_height);
    output.command = command_coordinator_.Compute(
        active_mission_, 0, {}, CameraRequest::kNone, input.delivery_feedback,
        input.now_sec);
    output.post_motion_observing = true;
    output.post_motion_frames = decision_observation_total_frames_;
    output.post_motion_window_frames = decision_object_hits_.size();
    output.post_motion_valid =
        decision_observation_mission_ == MissionType::kLine
            ? decision_line_valid_count_
            : static_cast<int>(std::count(decision_object_hits_.begin(),
                                          decision_object_hits_.end(), true));
    output.post_motion_elapsed_sec =
        std::max(0.0, input.now_sec - decision_observation_start_sec_);
    if (deferred_done_.action_id == 0 || output.command.action_id != 0)
      last_command_ = output.command;
    output.active_mission = active_mission_;
    output.line_in_recovery = line_state_ != LineState::kNormal;
    output.ball = ball_result_;
    output.hurdle = hurdle_result_;
    output.goal = goal_result_;
    output.has_ball = has_ball_;
    PopulateDiagnostics(output, input.now_sec);
    return output;
  }
  // The camera-settled transition also belongs to the observation gate,
  // although it advances WAIT_CAMERA without collecting a sample yet.
  output.post_motion_observing = observation_hold;
  decision_observation_first_frame_ = false;
  const int visible_turns_before = line_visible_recovery_turns_;
  const int last_score_turns_before = line_last_score_recovery_turns_;
  ActionExecutionFeedback action_feedback = input.action_feedback;
  const bool matching_feedback =
      last_command_.action_id != 0 &&
      input.delivery_feedback.action_id == last_command_.action_id;
  if (input.command_transport_enabled) {
    action_feedback.action_done =
        matching_feedback && input.delivery_feedback.done;
    action_feedback.action_active =
        last_command_.action_id != 0 && !action_feedback.action_done;
  }
  action_feedback.observation_complete =
      observation_finished || applied_object_ready_mean;
  bool line_reference_valid = false;
  ActionRequest line_action;
  const auto compute_line = [&]() {
    FeatureConfig cfg = config_.line_features;
    if (line_image_center_u && std::isfinite(*line_image_center_u) &&
        *line_image_center_u >= 0.0 &&
        *line_image_center_u < static_cast<double>(input.image_width))
      cfg.image_center_u = *line_image_center_u;
    last_line_image_center_u_ = cfg.image_center_u >= 0
        ? cfg.image_center_u : 0.5 * input.image_width;
    output.line_features = ComputeLineFeatures(
        input.line_centers, input.image_width, input.image_height, cfg);
    output.line_computed = true;
    if (!input.advance_line_fsm)
      return;
    const bool pending =
        last_command_.action_id != 0 &&
        last_command_.mission == MissionType::kLine &&
        last_command_.action_category == ActionCategory::kLocomotion;
    const bool queued_wait =
        pending &&
        (last_command_.control_phase == ControlPhase::kWaitingQueuedActionAck ||
         last_command_.control_phase ==
             ControlPhase::kWaitingQueuedActionStart);
    const bool feedback = pending && matching_feedback;
    const bool done = feedback && input.delivery_feedback.done;
    const bool active = pending && !done;
    // 첫 current action은 실제 ACK를 받은 시점부터 실행 관측을 모은다.
    // queued action의 선행 ACK는 저장 완료만 뜻하므로 queued_wait 동안은
    // 시작하지 않고, current DONE 뒤 pending으로 승격된 다음 frame에 시작한다.
    const bool execution_started =
        last_command_.control_phase == ControlPhase::kWaitingActionDone ||
        (feedback && input.delivery_feedback.acknowledged);
    LineGuide decision = input.line_decision_guide_override
                             ? *input.line_decision_guide_override
                             : output.line_features.guide;
    output.line_decision_guide = decision;
    if (pending && !queued_wait && execution_started &&
        IsLongLineAction(last_command_.action)) {
      if (!line_guide_accumulator_.ActiveFor(last_command_.action_id) &&
          ready_line_action_id_ != last_command_.action_id)
        line_guide_accumulator_.Begin(last_command_.action_id, input.now_sec);
      if (line_guide_accumulator_.ActiveFor(last_command_.action_id))
        line_guide_accumulator_.Add(last_command_.action_id, input.now_sec,
                                    output.line_features.guide);
      if (feedback && (input.delivery_feedback.ready || done) &&
          ready_line_action_id_ != last_command_.action_id) {
        const auto aggregate = line_guide_accumulator_.Finish(
            last_command_.action_id, input.now_sec,
            input.delivery_feedback.ready && !done
                ? config_.line_p2p.ready_lead_sec
                : 0.0);
        output.line_window_stats = line_guide_accumulator_.LastStats();
        if (aggregate) {
          last_ready_line_guide_ = aggregate;
          decision = *aggregate;
          output.line_decision_guide = decision;
          line_failure_pending_done_ = false;
        } else {
          last_ready_line_guide_.reset();
          decision.valid = false;
          output.line_decision_guide = decision;
          line_failure_pending_done_ = input.delivery_feedback.ready && !done;
        }
        if (input.delivery_feedback.ready)
          ready_line_action_id_ = last_command_.action_id;
        // READY는 current action의 종료가 아니다. invalid 집계의 2초
        // stationary observation은 실제 DONE 뒤에만 시작한다.
        if (line_failure_pending_done_) {
          line_action = {};
          return;
        }
        line_action = ComputeLineAction(decision, input, false, done,
                                        &line_reference_valid);
        return;
      }
    } else if (!pending) {
      line_guide_accumulator_.Reset();
    }
    if (done && line_failure_pending_done_) {
      line_failure_pending_done_ = false;
      BeginObservation(LineState::kFailureObserve, input.now_sec);
      AddObservation(output.line_features.guide);
      line_action = {};
      line_reference_valid = false;
      return;
    }
    line_action =
        ComputeLineAction(decision, input, active, done, &line_reference_valid);
  };

  const auto hurdle_line_decision = [&]() {
    // Compute LINE O/H steering during HURDLE without advancing LINE FSM.
    const auto features=ComputeLineFeatures(input.line_centers, input.image_width,
                                            input.image_height, config_.line_features);
    if (!output.line_computed) {
      output.line_features=features;
      output.line_computed=true;
    }
    const LineGuide guide = input.line_decision_guide_override.value_or(
        features.guide);
    return line_p2p_controller_.Compute(guide);
  };
  switch (active_mission_) {
  case MissionType::kLine:
    compute_line();
    ball_result_ = {};
    hurdle_result_ = {};
    goal_result_ = {};
    // FINAL HOLD는 명시적 Reset 전까지 완전한 terminal 상태다. 라인 action뿐
    // 아니라 새로운 object mission 진입도 허용하지 않는다.
    if (line_state_ == LineState::kFinalHold)
      break;
    // PAUSE cannot let incidental moving-frame detections switch missions.
    // Existing BALL/GOAL/HURDLE missions are still stepped to process DONE.
    if (!input.allow_new_mission_entry)
      break;
    if (has_ball_) {
      if (config_.enable_goal) {
        BallResult carrying;
        carrying.has_ball = true;
        goal_controller_.UpdateBallState(carrying);
        goal_result_ = goal_controller_.Compute(
            input.goal_target, input.backboard_target, input.goal_pose,
            input.image_width, input.image_height, input.now_sec,
            line_reference_valid, input.camera_feedback, action_feedback,
            input.allow_new_body_actions && input.allow_new_commands,
            input.backboard_center_depth_m,
            post_pickup_goal_line_gate_active_ && line_reference_valid &&
                line_state_ == LineState::kNormal &&
                (post_pickup_five_step_completed_ ||
                 post_pickup_line_actions_completed_ >=
                     goal_controller_.RequiredLineWalkActions()),
            input.backboard_image_without_depth);
        if (goal_result_.mode != GoalMode::kLineFollow)
          EnterMission(MissionType::kGoal);
      }
    } else {
      if (config_.enable_hurdle) {
        hurdle_result_ = hurdle_controller_.Compute(
            input.hurdle_target, input.image_width, input.image_height,
            input.now_sec, input.camera_feedback, action_feedback,
            hurdle_line_decision(),
            input.allow_new_body_actions && input.allow_new_commands);
        if (hurdle_result_.mode != HurdleMode::kLineFollow) {
          EnterMission(MissionType::kHurdle);
          break;
        }
      }
      if (config_.enable_ball) {
        ball_result_ = ball_controller_.Compute(
            input.ball_target, input.image_width, input.image_height,
            input.now_sec, input.camera_feedback, action_feedback,
            input.allow_new_body_actions && input.allow_new_commands,
            !command_coordinator_.HasQueuedAction(), input.ball_entry_detected,
            input.ball_entry_center_u);
        if (ball_result_.mode != BallMode::kLineFollow)
          EnterMission(MissionType::kBall);
      }
    }
    break;
  case MissionType::kBall: {
    const BallMode before = ball_result_.mode;
    const bool controller_allow =
        input.allow_new_body_actions && input.allow_new_commands;
    if (before == BallMode::kPostPickupLineRecovery)
      line_action = ComputeMissionLineRecovery(input, action_feedback,
                                               &line_reference_valid, &output);
    ball_result_ = ball_controller_.Compute(
        input.ball_target, input.image_width, input.image_height, input.now_sec,
        input.camera_feedback, action_feedback, controller_allow, true,
        std::nullopt, std::nullopt,
        !(config_.test_mission != MissionType::kNone &&
          config_.test_stage == 2),
        !(config_.test_mission != MissionType::kNone &&
          config_.test_stage == 3 && !test_execution_started_));
    if ((before == BallMode::kFineSidePair || before == BallMode::kFineMotionRepeat) &&
        ball_result_.mode != before &&
        !input.manual_observation_control &&
        config_.post_motion_observation_sec > 0.0) {
      BeginDecisionObservation(input.now_sec, active_mission_, false, {});
      output.post_motion_observing = true;
    }
    if (ball_result_.mode != BallMode::kLineFollow)
      test_entry_pending_ = false;
    has_ball_ = ball_result_.has_ball;
    if (ball_result_.mode == BallMode::kPostPickupLineRecovery)
      ball_result_.action = line_action;
    if (before != BallMode::kPostPickupLineRecovery &&
        ball_result_.mode == BallMode::kPostPickupLineRecovery)
      BeginLineReacquisition();
    if (ball_result_.mode == BallMode::kPostPickupLineRecovery &&
        line_reference_valid) {
      ball_controller_.CompleteLineRecovery();
      if (has_ball_ && config_.enable_goal)
        goal_controller_.UpdateBallState(ball_result_);
      FinishMission();
    } else if (ball_result_.mode == BallMode::kLineFollow &&
               !ball_result_.active && !test_entry_pending_) {
      BeginLineReacquisition();
      FinishMission();
    }
    break;
  }
  case MissionType::kHurdle: {
    const auto before = hurdle_result_.mode;
    const bool controller_allow =
        input.allow_new_body_actions && input.allow_new_commands &&
        !(config_.test_mission != MissionType::kNone &&
          config_.test_stage == 2 && before == HurdleMode::kFineAdjust);
    hurdle_result_ = hurdle_controller_.Compute(
        input.hurdle_target, input.image_width, input.image_height,
        input.now_sec, input.camera_feedback, action_feedback,
        hurdle_line_decision(), controller_allow);
    if (hurdle_result_.mode != HurdleMode::kLineFollow)
      test_entry_pending_ = false;
    if (before == HurdleMode::kDownSearch &&
        hurdle_result_.mode == HurdleMode::kCoarseAdjust)
      fresh_after_search_ = true;
    if (hurdle_result_.mode == HurdleMode::kLineFollow &&
        !hurdle_result_.active && !test_entry_pending_) {
      ball_controller_.ClearEntryEvidence();
      BeginLineReacquisition();
      FinishMission();
    }
    break;
  }
  case MissionType::kGoal: {
    const GoalMode before = goal_result_.mode;
    const bool controller_allow =
        input.allow_new_body_actions && input.allow_new_commands &&
        !(config_.test_mission != MissionType::kNone &&
          config_.test_stage == 2 && before == GoalMode::kFineAdjust);
    if (before == GoalMode::kHeadingRecovery)
      line_action = ComputeMissionLineRecovery(input, action_feedback,
                                               &line_reference_valid, &output);
    goal_result_ = goal_controller_.Compute(
        input.goal_target, input.backboard_target, input.goal_pose,
        input.image_width, input.image_height, input.now_sec,
        line_reference_valid, input.camera_feedback, action_feedback,
        controller_allow, input.backboard_center_depth_m, false,
        input.backboard_image_without_depth);
    // Fixed edge turns share the existing repeat gate. After the second actual
    // DONE, collect one fresh observation before resuming approach decisions.
    if (before == GoalMode::kEdgeTurn &&
        goal_result_.mode != GoalMode::kEdgeTurn &&
        !input.manual_observation_control &&
        config_.post_motion_observation_sec > 0.0) {
      BeginDecisionObservation(input.now_sec, active_mission_, false, {});
      output.post_motion_observing = true;
    }
    if (goal_result_.mode == GoalMode::kHeadingRecovery)
      goal_result_.action = line_action;
    has_ball_ = goal_controller_.HasBall();
    ball_controller_.SetHasBall(has_ball_);
    if (before != GoalMode::kHeadingRecovery &&
        goal_result_.mode == GoalMode::kHeadingRecovery)
      BeginLineReacquisition();
    if (goal_result_.mode == GoalMode::kLineFollow && !goal_result_.active) {
      if (before == GoalMode::kEntryRetryCameraForward) {
        // Camera FORWARD is physically settled. Reuse normal LINE recovery
        // and its READY/DONE planner, with no evidence from an earlier walk.
        active_mission_ = MissionType::kLine;
        BeginLineReacquisition();
        line_plan_.Reset();
        line_auxiliary_id_ = line_ready_handled_id_ = 0;
        line_auxiliary_done_ = line_auxiliary_observing_ = false;
        decision_observation_active_ = decision_observation_restart_pending_ = false;
        deferred_done_ = {};
        post_pickup_goal_line_gate_active_ = true;
        post_pickup_line_actions_issued_ = post_pickup_line_actions_completed_ = 0;
        post_pickup_straight_blocks_completed_ = 0;
        post_pickup_five_step_completed_ = false;
        post_pickup_line_actions_.clear();
      } else {
        FinishMission();
      }
    }
    break;
  }
  default:
    active_mission_ = MissionType::kLine;
    compute_line();
    break;
  }

  if (ResumeContinuousApproach(input))
    return StepWithLineImageCenter(input,line_image_center_u);
  // A near mission changes body/head geometry. A pre-entry LINE score must
  // never steer later recovery after that change, including legacy/manual paths.
  if ((active_mission_==MissionType::kBall &&
       ((ball_result_.mode==BallMode::kApproachBall && ball_result_.tracked.visible &&
         ball_result_.tracked.center_px.v>=config_.ball.approach_two_step_v_px) ||
        ball_result_.mode==BallMode::kWaitCameraDown || ball_result_.mode==BallMode::kDownSearch ||
        ball_result_.mode==BallMode::kCoarseAdjust || ball_result_.mode==BallMode::kWaitFinePose ||
        ball_result_.mode==BallMode::kFineAdjustForPickup)) ||
      (active_mission_==MissionType::kHurdle &&
       ((hurdle_result_.mode==HurdleMode::kApproach && hurdle_result_.tracked.visible &&
         hurdle_result_.tracked.center_px.v>=config_.hurdle.approach_two_step_v_px) ||
        hurdle_result_.mode==HurdleMode::kWaitCameraDown || hurdle_result_.mode==HurdleMode::kDownSearch ||
        hurdle_result_.mode==HurdleMode::kCoarseAdjust || hurdle_result_.mode==HurdleMode::kWaitFinePose ||
        hurdle_result_.mode==HurdleMode::kFineLineAlign ||
        hurdle_result_.mode==HurdleMode::kFineAdjust)) ||
      (active_mission_==MissionType::kGoal &&
       (goal_result_.mode==GoalMode::kCoarseAdjust || goal_result_.mode==GoalMode::kWaitFinePose ||
        goal_result_.mode==GoalMode::kFineAdjust)))
    last_line_score_.reset();
    actual_line_stability_history_.clear();
  ActionRequest request;
  CameraRequest camera_request = CameraRequest::kNone;
  int phase = 0;
  if (active_mission_ == MissionType::kBall) {
    request = ball_result_.action;
    camera_request = ball_result_.camera_request;
    phase = static_cast<int>(ball_result_.mode);
  } else if (active_mission_ == MissionType::kHurdle) {
    request = hurdle_result_.action;
    camera_request = hurdle_result_.camera_request;
    phase = static_cast<int>(hurdle_result_.mode);
  } else if (active_mission_ == MissionType::kGoal) {
    request = goal_result_.action;
    camera_request = goal_result_.camera_request;
    phase = static_cast<int>(goal_result_.mode);
  } else {
    request = line_action;
    phase = static_cast<int>(line_state_);
  }
  if (config_.test_mission != MissionType::kNone &&
      !action_feedback.action_active &&
      TestBoundaryReached(input.camera_feedback)) {
    goal_trial_complete_ = true;
    request = {};
    camera_request = CameraRequest::kNone;
  }
  // Alignment trials must never dispatch the mission primitive, including
  // while the full observation window is still accumulating.
  if (config_.test_mission != MissionType::kNone && config_.test_stage == 2 &&
      (request.action == MissionAction::kPickBall ||
       request.action == MissionAction::kShoot ||
       request.action == MissionAction::kHurdle ||
       (config_.test_mission == MissionType::kHurdle &&
        hurdle_result_.mode == HurdleMode::kFineAdjust &&
        hurdle_result_.tracked.center_px.v >= config_.hurdle.contact_v_px)))
    request = {};
  if (config_.test_mission != MissionType::kNone && config_.test_stage == 3 &&
      !test_execution_started_ && !TestExecutionReady())
    request = {};
  if (mission_finish_pending_) {
    request = {MissionAction::kWalkMode, ActionCategory::kLocomotion, 0, false};
    camera_request = CameraRequest::kNone;
    phase = 901;
  }
  // All recovery turns and resumed normal walking obey the same stance
  // contract.
  const int action_number = static_cast<int>(request.action);
  const bool wants_walk = action_number == 9 ||
                          (action_number >= 10 && action_number <= 13) ||
                          (action_number >= 21 && action_number <= 28) ||
                          request.action == MissionAction::kStepForwardOneAlternate ||
                          request.action == MissionAction::kTurnLeftInPlace;
  const bool wants_fine = (action_number >= 1 && action_number <= 8) ||
                          (action_number >= 14 && action_number <= 17);
  const bool implicit_pose =
      request.action != MissionAction::kDefaultPoseMode &&
      request.action != MissionAction::kWalkMode &&
      ((wants_walk && foot_pose_ != FootPose::kWalk) ||
       (wants_fine && foot_pose_ != FootPose::kFine));
  if (implicit_pose) {
    if (active_mission_ == MissionType::kLine && wants_fine) {
      recovery_turn_issued_ = false;
      line_visible_recovery_turns_ = visible_turns_before;
      line_last_score_recovery_turns_ = last_score_turns_before;
    }
    request = {wants_walk ? MissionAction::kWalkMode
                          : MissionAction::kDefaultPoseMode,
               ActionCategory::kLocomotion, 0, false};
    camera_request = CameraRequest::kNone;
    phase = 901;
  }
  // Reserve B by A's READY mean without stepping the real mission FSM.
  // Apply before permission gates so PAUSE and manual restrictions always win.
  MissionType command_mission = active_mission_;
  if (ready_object_successor && moving_object_ready_mean_ &&
      !observation_hold && camera_request == CameraRequest::kNone &&
      active_mission_ == moving_object_ready_mean_->mission) {
    request = *ready_object_successor;
    command_mission = ready_object_successor_mission.value_or(active_mission_);
  }
  // one-shot tuning gate는 perception/FSM/feedback 처리를 막거나 기존
  // pending/queued action을 취소하지 않는다. coordinator 직전의 새 LINE
  // request만 제거하여 READY/DONE 조기 반환 경로까지 동일하게 막는다.
  if (active_mission_ == MissionType::kLine && !input.allow_new_line_action)
    request = {};
  // Stop booking after either configured post-pickup LINE threshold is committed.
  // GOAL camera still follows the qualifying action's actual DONE.
  // Recovery turns remain available if the line itself becomes invalid.
  if (active_mission_ == MissionType::kLine &&
      post_pickup_goal_line_gate_active_ &&
      request.category == ActionCategory::kLocomotion &&
      IsLongLineAction(request.action) &&
      (post_pickup_line_actions_issued_ >=
           goal_controller_.RequiredLineWalkActions() ||
       StraightGoalGateReached(static_cast<int>(std::count_if(
           post_pickup_line_actions_.begin(), post_pickup_line_actions_.end(),
           [](const auto& entry) { return entry.second == MissionAction::kStepForwardFour; })))))
    request = {};
  // Manual tuning freezes NEW body actions but permits automatic camera
  // requests; unlike allow_new_commands, feedback still advances.
  if (!input.allow_new_body_actions ||
      (incoming_repeat_scope != 0 && RepeatScope() != incoming_repeat_scope))
    request = {};
  // Global start/pause gate: do not originate fresh action or camera
  // commands. The coordinator below must still consume pending feedback.
  if (!input.allow_new_commands) {
    request = {};
    camera_request = CameraRequest::kNone;
  }
  if (observation_hold || input.now_sec + 1e-9 < settle_until_sec_) {
    request = {};
    camera_request = CameraRequest::kNone;
  }
  if (command_mission != MissionType::kLine &&
      request.category == ActionCategory::kLocomotion &&
      config_.UsesObjectReadyMean(request.action))
    request.line_queue_eligible = true;
  // Transport can have consumed A DONE and even B DONE ahead of RGB processing.
  // Once B was pre-booked, A's late sensory DONE must not dispatch B again.
  const bool queued_parent_done =
      object_queued_parent_id_ != 0 && input.delivery_feedback.done &&
      input.delivery_feedback.action_id == object_queued_parent_id_;
  if (queued_parent_done)
    request = {};
  output.command = command_coordinator_.Compute(
      command_mission, phase, request, camera_request, input.delivery_feedback,
      input.now_sec);
  // A recovery turn is pending only after the coordinator actually issues it.
  // A stance transition, observation hold, or command gate must not consume
  // a turn before the executor's matching DONE arrives.
  const bool mission_line_recovery =
      (active_mission_ == MissionType::kBall &&
       ball_result_.mode == BallMode::kPostPickupLineRecovery) ||
      (active_mission_ == MissionType::kGoal &&
       goal_result_.mode == GoalMode::kHeadingRecovery);
  if (mission_line_recovery &&
      (request.action == MissionAction::kTurnLeft ||
       request.action == MissionAction::kTurnRight) &&
      output.command.command_type == CommandType::kAction &&
      output.command.action_id != 0 &&
      output.command.action == request.action)
    mission_recovery_issued_ = true;
  if (active_mission_ == MissionType::kLine &&
      post_pickup_goal_line_gate_active_ &&
      output.command.command_type == CommandType::kAction &&
      output.command.action_id != 0 &&
      output.command.action_category == ActionCategory::kLocomotion &&
      IsLongLineAction(output.command.action) &&
      post_pickup_line_actions_
          .emplace(output.command.action_id, output.command.action)
          .second)
    ++post_pickup_line_actions_issued_;
  if (ready_object_successor && moving_object_ready_mean_ &&
      output.command.command_type == CommandType::kAction &&
      output.command.control_phase == ControlPhase::kWaitingQueuedActionAck) {
    object_queued_parent_id_ = moving_object_ready_mean_->action_id;
    object_queued_successor_ = output.command;
  }
  if (config_.test_mission != MissionType::kNone && config_.test_stage == 3 &&
      output.command.command_type == CommandType::kAction &&
      TestExecutionReady())
    test_execution_started_ = true;
  if (output.command.action_id != 0 && phase == 901 &&
      (output.command.action == MissionAction::kDefaultPoseMode ||
       output.command.action == MissionAction::kWalkMode)) {
    pose_started_sec_ = input.now_sec;
    pose_change_pending_ = true;
    pose_target_ = output.command.action == MissionAction::kWalkMode
                       ? FootPose::kWalk
                       : FootPose::kFine;
  }
  if (input.advance_line_fsm && input.delivery_feedback.done &&
      input.delivery_feedback.action_id == ready_line_action_id_)
    ready_line_action_id_ = 0;
  // The transport may already have accepted DONE while an earlier ACK/READY
  // still awaits sensory processing. Keep its FSM ID until that DONE edge.
  // While B is reserved, last_command_ must still identify executing A.
  // Otherwise A's real DONE would be lost by object tracking and mission FSM.
  const bool object_queued_wait =
      object_queued_parent_id_ != 0 && !queued_parent_done &&
      (output.command.control_phase == ControlPhase::kWaitingQueuedActionAck ||
       output.command.control_phase == ControlPhase::kWaitingQueuedActionStart);
  if (queued_parent_done && output.command.action_id == 0 &&
      object_queued_successor_) {
    // B's DONE may have been processed by transport already. Retain B's
    // identity so the next sensory DONE edge still reaches its mission FSM.
    last_command_ = *object_queued_successor_;
    last_command_.command_type = CommandType::kHold;
    last_command_.control_phase = ControlPhase::kWaitingActionDone;
  } else if (!object_queued_wait &&
             (output.command.action_id != 0 || action_feedback.action_done ||
              last_command_.action_id == 0)) {
    last_command_ = output.command;
  }
  if (queued_parent_done)
    object_queued_parent_id_ = 0;
  if (object_queued_successor_ && input.delivery_feedback.done &&
      input.delivery_feedback.action_id == object_queued_successor_->action_id)
    object_queued_successor_.reset();
  output.active_mission = active_mission_;
  output.ball = ball_result_;
  output.hurdle = hurdle_result_;
  output.goal = goal_result_;
  output.has_ball = has_ball_;
  output.line_in_recovery = line_state_ != LineState::kNormal;
  output.trial_complete = goal_trial_complete_;
  output.repeat_scope = RepeatScope();
  output.repeat_sequence = output.repeat_scope != 0;
  output.ball_mission_index = std::max(1, ball_mission_index_);
  output.goal_mission_index = std::max(1, goal_mission_index_);
  output.recovery_turns_completed = mission_recovery_turns_;
  output.recovery_failed = mission_recovery_failed_;
  PopulateDiagnostics(output, input.now_sec);
  return output;
}

void MissionController::PopulateDiagnostics(MissionFrameResult &output,
                                            double now) const {
  PopulateApproachDiagnostics(output);
  output.remembered_line_score=last_line_score_;
  output.line_score_recovery =
      line_state_ == LineState::kScoreRecovery ||
      (line_state_ == LineState::kFinalHold &&
       line_guard_turns_completed_ > 0 &&
       line_guard_turns_completed_ >= config_.line_p2p.recovery_max_turns);
  output.line_score_recovery_turns_completed = line_guard_turns_completed_;
  output.test_stage = config_.test_stage;
  output.test_ready = TestExecutionReady();
  output.test_executing = test_execution_started_;
  output.terminal_hold = IsTerminalHold();
  output.foot_pose = foot_pose_;
  output.foot_transition =
      pose_change_pending_ ||
      (last_command_.action_id != 0 &&
       (last_command_.action == MissionAction::kDefaultPoseMode ||
        last_command_.action == MissionAction::kWalkMode));
  output.recovery_max_turns = config_.mission_recovery_max_turns;
  output.recovery_observation_frames =
      static_cast<int>(mission_recovery_hits_.size());
  output.repeat_scope = RepeatScope();
  output.repeat_sequence = output.repeat_scope != 0;
  output.ball_mission_index = std::max(
      1, (active_mission_ == MissionType::kBall ||
          (continuous_.mission==MissionType::kBall && continuous_.mission_started) ||
          (goal_trial_complete_ && config_.test_mission == MissionType::kBall))
             ? ball_mission_index_
             : ball_mission_index_ + 1);
  output.goal_mission_index = std::max(
      1,
      (active_mission_ == MissionType::kGoal || goal_controller_.EntryLineRetryPending() ||
       (continuous_.mission==MissionType::kGoal && continuous_.mission_started) ||
       (goal_trial_complete_ && (config_.test_mission == MissionType::kGoal ||
                                 (config_.test_mission == MissionType::kNone &&
                                  config_.goal_trial_start_goal))))
          ? goal_mission_index_
          : goal_mission_index_ + 1);
  output.recovery_turns_completed = mission_recovery_turns_;
  output.recovery_failed = mission_recovery_failed_;
  output.recovery_verifying = mission_recovery_verifying_;
  output.goal_walk_actions_completed = post_pickup_line_actions_completed_;
  output.goal_straight_blocks_completed = post_pickup_straight_blocks_completed_;
  output.goal_walk_actions_remaining =
      std::max(0, goal_controller_.RequiredLineWalkActions() -
                      post_pickup_line_actions_completed_);
  output.goal_five_step_completed = post_pickup_five_step_completed_;
  output.goal_walk_gate_active =
      has_ball_ && config_.enable_goal && !config_.goal_trial_start_goal &&
      (post_pickup_goal_line_gate_active_ ||
       (active_mission_ == MissionType::kGoal &&
        goal_result_.mode == GoalMode::kWaitCameraGoal));
  output.goal.recovery_observations_completed =
      goal_controller_.RecoveryObservationsCompleted();
  output.goal.entry_line_retries = goal_controller_.EntryLineRetries();
  output.goal.depth_recovery_active=goal_controller_.DepthRecoveryActive();
  output.goal.depth_recovery_steps=goal_controller_.DepthRecoverySteps();
  if (active_mission_==MissionType::kLine && config_.test_mission==MissionType::kNone)
    output.goal.entry_line_retries=approach_goal_empty_retries_;
  output.goal.recovery_observation_elapsed_sec =
      goal_controller_.RecoveryObservationElapsed(now);
  output.ball.recovery_observations_completed =
      ball_controller_.RecoveryObservationsCompleted();
  output.ball.recovery_observation_elapsed_sec =
      ball_controller_.RecoveryObservationElapsed(now);
  output.ball.recovery_steps_completed = ball_controller_.RecoveryStepsCompleted();
  output.ball.recovery_step_limit = ball_controller_.RecoveryStepLimit();
  output.ball.entry_recovery = ball_controller_.EntryRecovery();
  output.ball.down_search_steps_remaining =
      ball_controller_.DownSearchStepsRemaining();
  output.ball.down_search_found = ball_controller_.DownSearchFound();
  output.ball.detection_hits = ball_controller_.DetectionHits();
  output.ball.detection_frames = ball_controller_.DetectionFrames();
  output.ball.lost_frames = ball_controller_.LostFrames();
  output.hurdle.down_search_found = hurdle_controller_.DownSearchFound();
  output.hurdle.detection_hits = hurdle_controller_.DetectionHits();
  output.hurdle.detection_frames = hurdle_controller_.DetectionFrames();
  output.hurdle.lost_frames = hurdle_controller_.LostFrames();
  output.goal.detection_hits = goal_controller_.DetectionHits();
  output.goal.detection_frames = goal_controller_.DetectionFrames();
  output.goal.lost_frames = goal_controller_.LostFrames();
  output.goal.pose_lost_frames = goal_controller_.PoseLostFrames();
  output.ball.fine_motion_action=ball_controller_.FineMotionAction();
  output.ball.fine_motion_repetitions=ball_controller_.FineMotionRepetitions();
  output.ball.fine_motion_remaining=ball_controller_.FineMotionActionsRemaining();
  output.ball.fine_motion_prediction_valid=ball_result_.mode==BallMode::kFineMotionRepeat;
  output.ball.fine_motion_predicted_center_px=ball_controller_.FineMotionPredictedCenter();
  output.hurdle.recovery_observations_completed =
      hurdle_controller_.RecoveryObservationsCompleted();
  output.hurdle.recovery_observation_elapsed_sec =
      hurdle_controller_.RecoveryObservationElapsed(now);
  output.hurdle.recovery_steps_completed = hurdle_controller_.RecoveryStepsCompleted();
  output.hurdle.recovery_step_limit = hurdle_controller_.RecoveryStepLimit();
  output.hurdle.entry_recovery = hurdle_controller_.EntryRecovery();
  output.hurdle.down_search_steps_remaining =
      hurdle_controller_.DownSearchStepsRemaining();
}

PerceptionMissionFrameResult
MissionController::StepPerception(const PerceptionFrameInput &input) {
  PerceptionMissionFrameResult output;
  output.perception.detection_track_ids.resize(input.detections.size(),0);
  std::vector<Detection> detections;
  std::vector<Detection> image_detections;
  detections.reserve(input.detections.size());
  for (const auto &observation : input.detections) {
    image_detections.push_back(observation.detection);
    Detection detection = observation.detection;
    if (detection.class_id == config_.object_targets.backboard_class_id) {
      const double lo = std::min(config_.backboard_min_depth_m,
                                 config_.backboard_max_depth_m);
      const double hi = std::max(config_.backboard_min_depth_m,
                                 config_.backboard_max_depth_m);
      if (!observation.center_depth_m ||
          !std::isfinite(*observation.center_depth_m) ||
          *observation.center_depth_m < lo || *observation.center_depth_m > hi)
        detection.confidence = -std::numeric_limits<double>::infinity();
    }
    if (detection.class_id==config_.object_targets.ball_class_id ||
        detection.class_id==config_.object_targets.hurdle_class_id) {
      const auto& cfg=config_.object_targets;
      ObjectFilterDiagnostic diag;
      diag.class_id=detection.class_id;
      diag.center={detection.box.x+0.5*detection.box.width,detection.box.y+0.5*detection.box.height};
      diag.aspect_ratio=detection.box.height>0 ? detection.box.width/detection.box.height : 0;
      diag.aspect_passed=cfg.AcceptAspect(detection);
      diag.depth_valid_pixels=observation.depth_valid_pixels;
      if(observation.center_depth_m && std::isfinite(*observation.center_depth_m) && *observation.center_depth_m>0)
        diag.depth_m=observation.center_depth_m;
      const bool ball=detection.class_id==cfg.ball_class_id;
      const bool enabled=ball ? cfg.ball_depth_enabled : cfg.hurdle_depth_enabled;
      const auto mode=input.camera_feedback.actual_mode;
      if(enabled && (mode==CameraMode::kForward || mode==CameraMode::kDown)) {
        const double lo=ball ? (mode==CameraMode::kDown ? cfg.ball_down_min_depth_m : cfg.ball_forward_min_depth_m) : cfg.hurdle_min_depth_m;
        const double hi=ball ? (mode==CameraMode::kDown ? cfg.ball_down_max_depth_m : cfg.ball_forward_max_depth_m) : cfg.hurdle_max_depth_m;
        diag.depth_passed=diag.depth_m ? (*diag.depth_m>=lo && *diag.depth_m<=hi) :
            (ball ? cfg.ball_allow_missing_depth : cfg.hurdle_allow_missing_depth);
      }
      if(!diag.depth_passed) detection.confidence=-std::numeric_limits<double>::infinity();
      output.perception.object_filter_diagnostics.push_back(diag);
    }
    detections.push_back(detection);
  }
  const auto append_accepted_objects = [&](int class_id, double confidence) {
    for (const auto &candidate : ExtractObjectTargetCandidates(
             detections, class_id, confidence, config_.object_targets)) {
      output.perception.accepted_object_detections.push_back(
          detections[candidate.detection_index]);
      output.perception.accepted_object_indices.push_back(candidate.detection_index);
    }
  };
  append_accepted_objects(config_.object_targets.ball_class_id,
                          config_.object_targets.ball_confidence);
  append_accepted_objects(config_.object_targets.goal_class_id,
                          config_.object_targets.goal_confidence);
  append_accepted_objects(config_.object_targets.backboard_class_id,
                          config_.object_targets.backboard_confidence);
  append_accepted_objects(config_.object_targets.hurdle_class_id,
                          config_.object_targets.hurdle_confidence);
  output.perception.raw_line_centers =
      ExtractLineCenters(detections, config_.line_detection,
                         &output.perception.line_detection_indices);
  output.perception.line_centers = output.perception.raw_line_centers;
  // A valid READY window is already associated and 10/7-filtered by
  // the manual adapter. Reacquire its averaged single detection instead of
  // associating it to a raw mid-walk position.
  if (input.manual_observation_control && input.manual_ready_mean) {
    if (active_mission_ == MissionType::kBall) {
      if (!ball_target_locked_ && !ball_reacquiring_) ball_association_tracker_.Reset();
      ball_controller_.UseNextObservationAsMean();
    }
  }
  // Never carry an association identity across a camera movement.
  const bool camera_moving =
      !input.camera_feedback.settled ||
      input.camera_feedback.actual_mode == CameraMode::kTransition;
  const CameraMode current_mode = input.camera_feedback.actual_mode;
  if (camera_moving || object_tracking_camera_moving_ ||
      current_mode != object_tracking_camera_mode_) {
    ball_association_tracker_.Reset();
    ball_candidate_tracker_.Reset();
    ball_target_id_=0;
    ball_target_locked_=false;
    ball_reacquiring_=config_.ball_candidate_tracking.enabled && active_mission_==MissionType::kBall;
    backboard_target_tracker_.Reset();
    hurdle_target_tracker_.Reset();
    hurdle_controller_.InvalidateDetectionEvidence();
    goal_controller_.InvalidateDetectionEvidence();
    // An observation already in flight must also belong entirely to the new
    // camera view. Preserve deferred DONE, restarting only evidence and time.
    if (decision_observation_active_ &&
        (decision_observation_mission_ == MissionType::kHurdle ||
         decision_observation_mission_ == MissionType::kGoal)) {
      decision_object_hits_.clear(); decision_goal_pose_hits_.clear(); decision_goal_near_hits_.clear();
      decision_observation_total_frames_ = 0;
      decision_last_target_.reset(); decision_object_weight_ = 0; decision_target_count_ = 0;
      decision_target_u_sum_ = decision_target_v_sum_ = 0;
      decision_rect_u_sum_ = decision_rect_v_sum_ = 0;
      decision_pose_count_ = 0; decision_pose_x_sum_ = decision_pose_z_sum_ = 0;
      decision_pose_sin_sum_ = decision_pose_cos_sum_ = 0;
      decision_observation_start_sec_ = input.now_sec +
          (camera_moving ? 0.0 : config_.transition_settle_sec);
    }
    moving_object_samples_.clear(); moving_object_ready_mean_.reset();
  }
  object_tracking_camera_moving_ = camera_moving;
  if (!camera_moving)
    object_tracking_camera_mode_ = current_mode;
  ObjectTargetSelection selection;
  const auto lower_raw_rank = [](const ObjectTargetCandidate &a,
                                 const ObjectTargetCandidate &b) {
    return a.target.confidence == b.target.confidence
               ? a.target.area_px < b.target.area_px
               : a.target.confidence < b.target.confidence;
  };
  const auto goals = ExtractObjectTargetCandidates(
      detections, config_.object_targets.goal_class_id,
      config_.object_targets.goal_confidence, config_.object_targets);
  if (!camera_moving && !goals.empty()) {
    const auto best =
        std::max_element(goals.begin(), goals.end(), lower_raw_rank);
    selection.targets.goal = best->target;
    selection.indices.goal = best->detection_index;
  }
  const auto ball_candidates = ExtractObjectTargetCandidates(
      detections, config_.object_targets.ball_class_id,
      config_.object_targets.ball_confidence, config_.object_targets);
  const bool manual_ball_identity_valid=!input.manual_ready_mean || !config_.ball_candidate_tracking.enabled ||
      active_mission_!=MissionType::kBall || (input.ball_target_id && input.ball_target_id==ball_target_id_);
  ObjectAssociationSelection ball;
  std::optional<BallCandidateTrack> entry_track;
  const auto mode_before=ball_result_.mode;
  const bool recovery_mode=active_mission_==MissionType::kBall &&
      (mode_before==BallMode::kBallRecoveryDown || mode_before==BallMode::kBallRecoveryForward ||
       mode_before==BallMode::kBallRecoveryWalkTurn || mode_before==BallMode::kDownSearch ||
       mode_before==BallMode::kForwardRecoveryTurn || mode_before==BallMode::kForwardRecoveryStep ||
       mode_before==BallMode::kForwardRecoveryObserve);
  const bool candidate_search=candidate_.mission==MissionType::kBall &&
      (candidate_.phase==CandidatePhase::kSearch || candidate_.phase==CandidatePhase::kRestore);
  const bool acquisition_episode=recovery_mode || candidate_search;
  const bool needs_initial_target=active_mission_==MissionType::kBall && !ball_target_locked_ &&
      !ball_reacquiring_ && ball_target_id_==0 && mode_before!=BallMode::kVerifyPickupObservation;
  if (config_.ball_candidate_tracking.enabled &&
      ((acquisition_episode && !ball_recovery_was_active_ && !ball_reacquiring_) || needs_initial_target)) {
    ball_reacquiring_=true; ball_target_locked_=false; ball_target_id_=0;
    ball_candidate_tracker_.Reset(); ball_association_tracker_.Reset();
  }
  ball_recovery_was_active_=acquisition_episode;
  const auto ball_reference=ball_association_tracker_.Reference();
  if (camera_moving || !manual_ball_identity_valid) {
    // No intermediate camera frame or foreign READY mean can update IDs/hits.
  } else if (active_mission_ == MissionType::kBall &&
             ball_result_.mode == BallMode::kVerifyPickupObservation) {
    ball_target_locked_=ball_reacquiring_=false; ball_target_id_=0;
    ball_candidate_tracker_.Reset(); ball_association_tracker_.Reset();
    if (!ball_candidates.empty()) {
      const auto best=std::max_element(ball_candidates.begin(),ball_candidates.end(),lower_raw_rank);
      ball={best->target,best->detection_index};
    }
  } else if(config_.ball_candidate_tracking.enabled &&
      ((active_mission_==MissionType::kLine && candidate_.phase==CandidatePhase::kNone && !has_ball_ && !ball_target_locked_) || ball_reacquiring_)) {
    const bool entry=active_mission_==MissionType::kLine && candidate_.phase==CandidatePhase::kNone;
    ball_candidate_tracker_.Update(ball_candidates,config_.ball_candidate_tracking,config_.object_association,
        config_.ball.stable_window,entry ? config_.ball.upper_acquire_v_px : std::numeric_limits<double>::infinity());
    const auto* chosen=ball_reacquiring_ ? ball_candidate_tracker_.Find(ball_target_id_) : nullptr;
    if (!chosen && continuous_.mission==MissionType::kBall && continuous_.id)
      chosen=ball_candidate_tracker_.Find(continuous_.id);
    if(!chosen) chosen=ball_candidate_tracker_.Best();
    if(chosen) {
      entry_track=*chosen;
      if(ball_reacquiring_ && ball_target_id_!=chosen->id) {
        ball_target_id_=chosen->id;
        ball_controller_.InvalidateTargetIdentity();
        moving_object_samples_.clear(); moving_object_ready_mean_.reset();
        if(decision_observation_active_ && decision_observation_mission_==MissionType::kBall) {
          std::fill(decision_object_hits_.begin(),decision_object_hits_.end(),false);
          decision_object_weight_=0; decision_last_target_.reset();
          decision_target_u_sum_=decision_target_v_sum_=0;
          decision_rect_u_sum_=decision_rect_v_sum_=0;
          decision_target_count_=0;
        }
      }
      if(chosen->detection_index) ball={chosen->target,chosen->detection_index};
    }
  } else {
    // Locked targets retain the last reference after missing_frame_limit;
    // only explicit recovery/camera episodes may acquire another identity.
    if(input.manual_ready_mean && input.ball_target_id && input.ball_target_id==ball_target_id_ && ball_candidates.size()==1)
      ball_association_tracker_.Seed(ball_candidates.front().target);
    if(!(input.manual_ready_mean && input.ball_target_id && ball_target_locked_ && input.ball_target_id!=ball_target_id_))
      ball=ball_association_tracker_.Update(ball_candidates,input.image_width,input.image_height,!ball_target_locked_);
  }
  output.perception.ball_tracking_reason=camera_moving ? "camera moving" :
      !manual_ball_identity_valid ? "foreign READY identity" :
      mode_before==BallMode::kVerifyPickupObservation ? "pickup verification bypass" :
      ball_reacquiring_ ? "recovery/camera revalidation" :
      ball_target_locked_ ? (ball.target ? "locked match" :
          (ball_candidates.empty() ? "no detection" : "identity gate rejected")) : "per-ID entry evidence";
  if(ball_reference && !ball_candidates.empty()) {
    const ObjectTargetCandidate* nearest=nullptr;
    double best=std::numeric_limits<double>::infinity();
    const double diagonal=std::max(1.0,std::hypot(input.image_width,input.image_height));
    for(const auto& candidate:ball_candidates) {
      const double d=ObjectCenterDistancePixels(*ball_reference,candidate.target);
      const double size=ObjectSizeLogRatio(*ball_reference,candidate.target);
      const double score=config_.object_association.center_distance_weight*d/diagonal+
          config_.object_association.size_change_weight*size;
      if(score<best) {best=score;nearest=&candidate;}
    }
    if(nearest) {
      output.perception.ball_match_distance_px=ObjectCenterDistancePixels(*ball_reference,nearest->target);
      output.perception.ball_match_size_log_ratio=ObjectSizeLogRatio(*ball_reference,nearest->target);
    }
  }
  const bool hurdle_active = active_mission_ == MissionType::kHurdle ||
                             candidate_.mission == MissionType::kHurdle || continuous_.mission==MissionType::kHurdle;
  const bool hurdle_recovery =
      (active_mission_ == MissionType::kHurdle &&
       (hurdle_result_.mode == HurdleMode::kDownSearch ||
        hurdle_result_.mode == HurdleMode::kRecoveryDown ||
        hurdle_result_.mode == HurdleMode::kRecoveryForward ||
        hurdle_result_.mode == HurdleMode::kRecoveryStep ||
        hurdle_result_.mode == HurdleMode::kRecoveryStepObserve)) ||
      (candidate_.mission == MissionType::kHurdle &&
       (candidate_.phase == CandidatePhase::kSearch || candidate_.phase == CandidatePhase::kRestore));
  const bool board_recovery = active_mission_ == MissionType::kGoal &&
      (goal_result_.mode == GoalMode::kRecoveryTurn || goal_result_.mode == GoalMode::kRecoverySearch ||
       goal_result_.mode == GoalMode::kRecoveryForwardStep || goal_result_.mode == GoalMode::kRecoveryExtraTurn ||
       goal_result_.mode == GoalMode::kFineRecovery || goal_result_.mode == GoalMode::kCoarseRecovery);
  const bool manual_hurdle_identity_valid = !input.manual_ready_mean ||
      !config_.hurdle_candidate_tracking.enabled || active_mission_ != MissionType::kHurdle ||
      (input.hurdle_target_id && input.hurdle_target_id == hurdle_target_tracker_.TargetId());
  const bool manual_board_identity_valid = !input.manual_ready_mean ||
      !config_.backboard_candidate_tracking.enabled || active_mission_ != MissionType::kGoal ||
      (input.backboard_target_id && input.backboard_target_id == backboard_target_tracker_.TargetId());
  if (input.manual_observation_control && input.manual_ready_mean) {
    if (active_mission_ == MissionType::kHurdle && manual_hurdle_identity_valid)
      hurdle_controller_.UseNextObservationAsMean();
    if (active_mission_ == MissionType::kGoal && manual_board_identity_valid)
      goal_controller_.UseNextObservationAsMean();
  }
  const auto board = camera_moving || !manual_board_identity_valid ? ObjectAssociationSelection{} :
      backboard_target_tracker_.Observe(
          ExtractObjectTargetCandidates(detections, config_.object_targets.backboard_class_id,
              config_.object_targets.backboard_confidence, config_.object_targets),
          config_.backboard_candidate_tracking, config_.object_association,
          input.image_width, input.image_height, config_.goal.stable_window,
          std::numeric_limits<double>::infinity(), active_mission_ == MissionType::kGoal ||
              continuous_.mission==MissionType::kGoal, board_recovery,
          input.manual_ready_mean && active_mission_ == MissionType::kGoal, input.backboard_target_id);
  const auto hurdle = camera_moving || !manual_hurdle_identity_valid ? ObjectAssociationSelection{} :
      hurdle_target_tracker_.Observe(
          ExtractObjectTargetCandidates(detections, config_.object_targets.hurdle_class_id,
              config_.object_targets.hurdle_confidence, config_.object_targets),
          config_.hurdle_candidate_tracking, config_.object_association,
          input.image_width, input.image_height, config_.hurdle.stable_window,
          active_mission_ == MissionType::kLine ? config_.hurdle.upper_acquire_v_px :
              std::numeric_limits<double>::infinity(), hurdle_active, hurdle_recovery,
          input.manual_ready_mean && active_mission_ == MissionType::kHurdle, input.hurdle_target_id);
  // Evidence belongs to one identity; a replacement inherits clocks/retry
  // budgets but never detection counts, READY means, or coordinate averages.
  const auto invalidate_identity = [&](MissionType mission) {
    if (mission == MissionType::kHurdle) hurdle_controller_.InvalidateTargetIdentity();
    else goal_controller_.InvalidateDetectionEvidence();
    if (active_mission_ == mission || candidate_.mission == mission) {
      moving_object_samples_.clear(); moving_object_ready_mean_.reset();
    }
    if (decision_observation_active_ && decision_observation_mission_ == mission) {
      std::fill(decision_object_hits_.begin(), decision_object_hits_.end(), false);
      std::fill(decision_goal_pose_hits_.begin(), decision_goal_pose_hits_.end(), false);
      std::fill(decision_goal_near_hits_.begin(), decision_goal_near_hits_.end(), false);
      decision_object_weight_ = 0; decision_last_target_.reset(); decision_target_count_ = 0;
      decision_target_u_sum_ = decision_target_v_sum_ = 0;
      decision_rect_u_sum_ = decision_rect_v_sum_ = 0;
      decision_pose_count_ = 0; decision_pose_x_sum_ = decision_pose_z_sum_ = 0;
      decision_pose_sin_sum_ = decision_pose_cos_sum_ = 0;
    }
  };
  if (!camera_moving && manual_hurdle_identity_valid && hurdle_target_tracker_.IdentityChanged())
    invalidate_identity(MissionType::kHurdle);
  if (!camera_moving && manual_board_identity_valid && backboard_target_tracker_.IdentityChanged())
    invalidate_identity(MissionType::kGoal);
  selection.targets.ball = ball.target;
  selection.indices.ball = ball.detection_index;
  selection.targets.backboard = board.target;
  selection.indices.backboard = board.detection_index;
  selection.targets.hurdle = hurdle.target;
  selection.indices.hurdle = hurdle.detection_index;
  output.perception.targets = selection.targets;
  if(selection.indices.ball && *selection.indices.ball<input.detections.size())
    output.perception.ball_center_depth_m=input.detections[*selection.indices.ball].center_depth_m;
  if(selection.indices.hurdle && *selection.indices.hurdle<input.detections.size())
    output.perception.hurdle_center_depth_m=input.detections[*selection.indices.hurdle].center_depth_m;

  if (input.enable_imu_rectification && input.imu_valid) {
    output.perception.line_centers =
        RectifyPixelPoints(output.perception.raw_line_centers, input.intrinsics,
                           input.roll_rad, input.pitch_rad);
    const auto rectify = [&](std::optional<ObjectTarget> &target) {
      if (!target)
        return;
      const auto point =
          RectifyPixelPoints({target->center_px}, input.intrinsics,
                             input.roll_rad, input.pitch_rad);
      if (!point.empty()) {
        target->rectified_center_px = point.front();
        target->center_rectified = true;
      }
    };
    rectify(output.perception.targets.ball);
    rectify(output.perception.targets.goal);
    rectify(output.perception.targets.backboard);
    rectify(output.perception.targets.hurdle);
    output.perception.imu_rectification_applied = true;
  }
  if (selection.indices.backboard &&
      *selection.indices.backboard < input.detections.size()) {
    const auto &depth = input.detections[*selection.indices.backboard];
    const auto &backboard = output.perception.targets.backboard;
    output.perception.backboard_center_depth_m = depth.center_depth_m;
    output.perception.backboard_left_depth_m = depth.left_depth_m;
    output.perception.backboard_right_depth_m = depth.right_depth_m;
    output.perception.backboard_left_depth_u_px = depth.left_depth_u_px;
    output.perception.backboard_right_depth_u_px = depth.right_depth_u_px;
    const bool fine = active_mission_ == MissionType::kGoal &&
                      (goal_result_.mode == GoalMode::kFineAdjust ||
                       goal_result_.mode == GoalMode::kCoarseAdjust ||
                       goal_result_.mode == GoalMode::kFineRecovery ||
                       goal_result_.mode == GoalMode::kCoarseRecovery ||
                       goal_result_.mode == GoalMode::kShoot ||
                       ((goal_result_.mode == GoalMode::kRecoveryTurn ||
                         goal_result_.mode == GoalMode::kRecoveryForwardStep ||
                         goal_result_.mode == GoalMode::kRecoveryExtraTurn) &&
                        foot_pose_ == FootPose::kFine));
    const bool near =
        depth.center_depth_m &&
        *depth.center_depth_m <= config_.goal.fine_adjust_start_z_m;
    if (backboard && depth.center_depth_m && depth.left_depth_m &&
        depth.right_depth_m && (near || fine)) {
      const double center =
          std::round(backboard->box_px.x + 0.5 * backboard->box_px.width);
      const double left = std::round(backboard->box_px.x);
      const double right =
          std::round(backboard->box_px.x + backboard->box_px.width - 1.0);
      output.perception.goal_pose = EstimateGoalPoseFromBackboardDepths(
          center, *depth.center_depth_m, depth.left_depth_u_px.value_or(left),
          *depth.left_depth_m, depth.right_depth_u_px.value_or(right),
          *depth.right_depth_m, input.intrinsics, backboard->confidence);
    }
  }
  MissionFrameInput prepared;
  prepared.line_centers = output.perception.line_centers;
  prepared.intrinsics=input.intrinsics;
  prepared.rectify_approach=input.enable_imu_rectification && input.imu_valid;
  prepared.approach_roll_rad=input.roll_rad;prepared.approach_pitch_rad=input.pitch_rad;
  prepared.ball_track_id=entry_track ? entry_track->id : ball_target_id_;
  prepared.hurdle_track_id=hurdle_target_tracker_.TargetId();
  prepared.backboard_track_id=backboard_target_tracker_.TargetId();
  if (backboard_target_tracker_.Selected())
    prepared.backboard_entry_track_hits=backboard_target_tracker_.Selected()->HitCount();
  prepared.ball_target = output.perception.targets.ball;
  if(config_.ball_candidate_tracking.enabled && active_mission_==MissionType::kLine && candidate_.phase==CandidatePhase::kNone) {
    prepared.ball_entry_track_hits=entry_track ? entry_track->HitCount() : 0;
    prepared.ball_entry_track_frames=entry_track ? static_cast<int>(entry_track->hits.size()) : 0;
    prepared.ball_entry_detected=entry_track && entry_track->detection_index && entry_track->target.center_px.v<=config_.ball.upper_acquire_v_px;
    if(entry_track) prepared.ball_entry_center_u=entry_track->MeanU();
  } else if(config_.ball_candidate_tracking.enabled && candidate_.mission==MissionType::kBall) {
    // During draining, only the pinned identity may update search direction;
    // an unrelated raw detection must not overwrite the selected A evidence.
    prepared.ball_entry_detected=ball.target && ball.target->center_px.v<=config_.ball.upper_acquire_v_px;
    if(ball.target) prepared.ball_entry_center_u=ball.target->center_px.u;
  } else {
    prepared.ball_entry_detected = false;
    const ObjectTargetCandidate *entry_candidate = nullptr;
    if (!camera_moving)
      for (const auto &candidate : ball_candidates) {
        if (candidate.target.center_px.v <= config_.ball.upper_acquire_v_px &&
            (!entry_candidate || lower_raw_rank(*entry_candidate, candidate)))
          entry_candidate = &candidate;
      }
    if (entry_candidate) {
      prepared.ball_entry_detected = true;
      prepared.ball_entry_center_u = entry_candidate->target.center_px.u;
    }
  }
  prepared.hurdle_target = output.perception.targets.hurdle;
  if (config_.hurdle_candidate_tracking.enabled && active_mission_ == MissionType::kLine &&
      candidate_.phase == CandidatePhase::kNone) {
    const auto& track = hurdle_target_tracker_.Selected();
    prepared.hurdle_entry_track_hits = track ? track->HitCount() : 0;
    if (track) prepared.hurdle_entry_center_u = track->MeanU();
  }
  prepared.goal_target = output.perception.targets.goal;
  prepared.backboard_target = output.perception.targets.backboard;
  const bool backboard_in_image = !ExtractObjectTargetCandidates(
      image_detections, config_.object_targets.backboard_class_id,
      config_.object_targets.backboard_confidence, config_.object_targets).empty();
  if (input.camera_feedback.settled &&
      input.camera_feedback.actual_mode == CameraMode::kGoal)
    goal_controller_.ObserveEntryBackboard(backboard_in_image);
  prepared.backboard_image_without_depth =
      input.camera_feedback.settled && backboard_in_image &&
      ExtractObjectTargetCandidates(
          detections, config_.object_targets.backboard_class_id,
          config_.object_targets.backboard_confidence, config_.object_targets)
          .empty();
  prepared.goal_pose = output.perception.goal_pose;
  prepared.backboard_center_depth_m =
      output.perception.backboard_center_depth_m;
  prepared.image_width = input.image_width;
  prepared.image_height = input.image_height;
  prepared.now_sec = input.now_sec;
  prepared.camera_feedback = input.camera_feedback;
  prepared.command_transport_enabled = input.command_transport_enabled;
  prepared.action_feedback = input.action_feedback;
  prepared.delivery_feedback = input.delivery_feedback;
  prepared.allow_new_line_action = input.allow_new_line_action;
  prepared.allow_new_line_plan = input.allow_new_line_plan;
  prepared.allow_new_commands = input.allow_new_commands;
  const bool manual_evidence_valid =
      manual_ball_identity_valid && manual_hurdle_identity_valid && manual_board_identity_valid &&
      input.manual_object_hits.size() == 10 &&
      std::count(input.manual_object_hits.begin(),
                 input.manual_object_hits.end(), true) >= 7;
  prepared.allow_new_body_actions =
      input.allow_new_body_actions &&
      (!input.manual_ready_mean || manual_evidence_valid);
  prepared.allow_test_setup_body_actions = input.allow_test_setup_body_actions;
  prepared.allow_new_mission_entry = input.allow_new_mission_entry;
  prepared.advance_line_fsm = input.advance_line_fsm;
  prepared.manual_observation_control = input.manual_observation_control;
  prepared.begin_manual_observation = input.begin_manual_observation;
  prepared.manual_ready_mean = input.manual_ready_mean;
  prepared.manual_object_hits = input.manual_object_hits;
  if(!manual_ball_identity_valid || !manual_hurdle_identity_valid || !manual_board_identity_valid)
    std::fill(prepared.manual_object_hits.begin(),prepared.manual_object_hits.end(),false);
  prepared.manual_pose_hits = input.manual_pose_hits;
  prepared.manual_near_hits = input.manual_near_hits;
  prepared.line_decision_guide_override = input.line_decision_guide_override;
  // Line robot-reference is independent from the camera optical cx.
  std::optional<double> center = config_.line_features.image_center_u;
  output.mission = StepWithLineImageCenter(prepared, center);
  if(config_.ball_candidate_tracking.enabled) {
    const bool continuous_ball=continuous_.mission==MissionType::kBall && continuous_.locked;
    if (continuous_ball && !ball_target_locked_ && continuous_.id && continuous_.last) {
      ball_target_id_=continuous_.id;ball_target_locked_=true;ball_reacquiring_=false;
      ball_association_tracker_.Seed(*continuous_.last);ball_candidate_tracker_.Reset();
    }
    const bool selected_ball=candidate_.mission==MissionType::kBall || active_mission_==MissionType::kBall || continuous_ball;
    if(selected_ball && entry_track && !ball_target_locked_ &&
        ((!ball_reacquiring_ && candidate_.mission==MissionType::kBall) ||
         (active_mission_==MissionType::kBall && ball_result_.mode!=BallMode::kWaitCameraDown &&
          ball_result_.mode!=BallMode::kDownSearch && ball_result_.mode!=BallMode::kBallRecoveryDown &&
          ball_result_.mode!=BallMode::kBallRecoveryForward && ball_result_.mode!=BallMode::kBallRecoveryWalkTurn &&
          ball_result_.mode!=BallMode::kForwardRecoveryTurn && ball_result_.mode!=BallMode::kForwardRecoveryStep &&
          ball_result_.mode!=BallMode::kForwardRecoveryObserve &&
          ball_result_.mode!=BallMode::kVerifyPickupObservation))) {
      ball_target_id_=entry_track->id; ball_target_locked_=true; ball_reacquiring_=false;
      ball_association_tracker_.Seed(entry_track->target);
      ball_candidate_tracker_.Reset();
      if(candidate_.mission==MissionType::kBall) ball_controller_.ClearEntryEvidence();
    }
    if(!selected_ball && ball_target_locked_) {
      ball_target_locked_=ball_reacquiring_=false; ball_target_id_=0;
      ball_candidate_tracker_.Reset(); ball_association_tracker_.Reset();
    }
  }
  output.perception.ball_tracks=ball_candidate_tracker_.Tracks();
  if(entry_track && output.perception.ball_tracks.empty()) output.perception.ball_tracks.push_back(*entry_track);
  output.perception.ball_target_id=ball_target_id_;
  output.perception.ball_target_locked=ball_target_locked_;
  output.perception.ball_reacquiring=ball_reacquiring_;
  output.perception.ball_missing_frames=ball_target_locked_ ? ball_association_tracker_.MissingFrames() :
      (entry_track ? entry_track->missing_frames : 0);
  if (config_.hurdle_candidate_tracking.enabled) {
    const bool continuous_hurdle=continuous_.mission==MissionType::kHurdle && continuous_.locked;
    const bool selected = candidate_.mission == MissionType::kHurdle || active_mission_ == MissionType::kHurdle ||
        continuous_.mission==MissionType::kHurdle;
    if (continuous_hurdle && hurdle_target_tracker_.TargetId()==continuous_.id) hurdle_target_tracker_.Lock();
    if (selected && ((candidate_.mission == MissionType::kHurdle && !hurdle_target_tracker_.Reacquiring()) ||
        (active_mission_ == MissionType::kHurdle && hurdle_result_.mode != HurdleMode::kWaitCameraDown &&
         hurdle_result_.mode != HurdleMode::kDownSearch && hurdle_result_.mode != HurdleMode::kRecoveryDown &&
         hurdle_result_.mode != HurdleMode::kRecoveryForward &&
         hurdle_result_.mode != HurdleMode::kRecoveryStep && hurdle_result_.mode != HurdleMode::kRecoveryStepObserve &&
         hurdle_result_.tracked.stable)))
      hurdle_target_tracker_.Lock();
    if (!selected && (hurdle_target_tracker_.Locked() || hurdle_target_tracker_.Reacquiring()))
      hurdle_target_tracker_.Reset();
  }
  if (config_.backboard_candidate_tracking.enabled) {
    if (continuous_.mission==MissionType::kGoal && continuous_.locked &&
        backboard_target_tracker_.TargetId()==continuous_.id) backboard_target_tracker_.Lock();
    if (active_mission_ == MissionType::kGoal && goal_result_.tracked.stable &&
        goal_result_.mode != GoalMode::kWaitCameraGoal && goal_result_.mode != GoalMode::kRecoveryTurn &&
        goal_result_.mode != GoalMode::kRecoveryForwardStep && goal_result_.mode != GoalMode::kRecoveryExtraTurn &&
        goal_result_.mode != GoalMode::kRecoverySearch && goal_result_.mode != GoalMode::kFineRecovery &&
        goal_result_.mode != GoalMode::kCoarseRecovery)
      backboard_target_tracker_.Lock();
    if (active_mission_ != MissionType::kGoal && continuous_.mission!=MissionType::kGoal &&
        (backboard_target_tracker_.Locked() || backboard_target_tracker_.Reacquiring()))
      backboard_target_tracker_.Reset();
  }
  output.perception.hurdle_tracks = hurdle_target_tracker_.Tracks();
  output.perception.backboard_tracks = backboard_target_tracker_.Tracks();
  output.perception.hurdle_target_id = hurdle_target_tracker_.TargetId();
  output.perception.backboard_target_id = backboard_target_tracker_.TargetId();
  output.perception.hurdle_target_locked = hurdle_target_tracker_.Locked();
  output.perception.backboard_target_locked = backboard_target_tracker_.Locked();
  output.perception.hurdle_reacquiring = hurdle_target_tracker_.Reacquiring();
  output.perception.backboard_reacquiring = backboard_target_tracker_.Reacquiring();
  const auto record_track_ids = [&](const std::vector<BallCandidateTrack>& tracks) {
    for (const auto& track:tracks)
      if (track.detection_index && *track.detection_index<input.detections.size())
        output.perception.detection_track_ids[*track.detection_index]=track.id;
  };
  record_track_ids(output.perception.ball_tracks);
  record_track_ids(output.perception.hurdle_tracks);
  record_track_ids(output.perception.backboard_tracks);
  const auto record_selected_id = [&](std::optional<std::size_t> index, std::uint64_t id) {
    if (index && *index<input.detections.size() && id)
      output.perception.detection_track_ids[*index]=id;
  };
  record_selected_id(selection.indices.ball,output.perception.ball_target_id);
  record_selected_id(selection.indices.hurdle,output.perception.hurdle_target_id);
  record_selected_id(selection.indices.backboard,output.perception.backboard_target_id);
  return output;
}

PerceptionMissionFrameResult
MissionController::PreviewPerception(const PerceptionFrameInput &input) const {
  MissionController preview(*this);
  PerceptionFrameInput proposed = input;
  proposed.allow_new_body_actions = true;
  return preview.StepPerception(proposed);
}

int MissionController::RepeatScope() const {
  if (active_mission_ == MissionType::kLine &&
      (line_plan_.Active() || candidate_.phase != CandidatePhase::kNone)) return 101;
  if (mission_recovery_failed_ || mission_finish_pending_ ||
      pose_change_pending_)
    return 0;
  if (active_mission_ == MissionType::kBall) {
    // One committed side-recovery cycle is TURN -> STEP -> fresh observation.
    // Share a scope across the two motions; the controller owns post-step data.
    if (ball_result_.mode == BallMode::kForwardRecoveryTurn ||
        ball_result_.mode == BallMode::kForwardRecoveryStep)
      return 200 + static_cast<int>(BallMode::kForwardRecoveryTurn);
    if (ball_result_.mode == BallMode::kMidTurnBeforeRecovery ||
        ball_result_.mode == BallMode::kFineSidePair ||
        ball_result_.mode == BallMode::kFineMotionRepeat ||
        ball_result_.mode == BallMode::kDownSearch ||
        ball_result_.mode == BallMode::kPostPickupBackAway ||
        ball_result_.mode == BallMode::kPostPickupLineRecovery)
      return 200 + static_cast<int>(ball_result_.mode);
  } else if (active_mission_ == MissionType::kHurdle) {
    if (hurdle_result_.mode == HurdleMode::kDownSearch ||
        hurdle_result_.mode == HurdleMode::kRecoveryStep ||
        hurdle_result_.mode == HurdleMode::kContactWalk)
      return 400 + static_cast<int>(hurdle_result_.mode);
  } else if (active_mission_ == MissionType::kGoal) {
    if (goal_result_.mode == GoalMode::kPostShootBackAway ||
        goal_result_.mode == GoalMode::kEntryRetryCameraForward ||
        goal_result_.mode == GoalMode::kHeadingRecovery ||
        goal_result_.mode == GoalMode::kEdgeTurn)
      return 300 + static_cast<int>(goal_result_.mode);
  }
  return 0;
}

bool MissionController::NeedsStationaryObservation() const {
  if (candidate_.phase == CandidatePhase::kObserve) return true;
  return (active_mission_ == MissionType::kBall &&
          (ball_result_.mode == BallMode::kBallRecoveryDown ||
           ball_result_.mode == BallMode::kBallRecoveryForward ||
           ball_result_.mode == BallMode::kForwardRecoveryObserve)) ||
         (active_mission_ == MissionType::kHurdle &&
          (hurdle_result_.mode == HurdleMode::kRecoveryDown ||
           hurdle_result_.mode == HurdleMode::kRecoveryForward ||
           hurdle_result_.mode == HurdleMode::kRecoveryStepObserve)) ||
         (active_mission_ == MissionType::kGoal &&
          (goal_result_.mode == GoalMode::kFineRecovery ||
           goal_result_.mode == GoalMode::kCoarseRecovery ||
           goal_result_.mode == GoalMode::kRecoverySearch)) ||
         mission_recovery_verifying_;
}

bool MissionController::SetTestMissionIndices(int ball_index, int goal_index) {
  if (ball_index < 1 || goal_index < 1 || last_command_.action_id != 0)
    return false;
  config_.test_ball_mission_index = ball_index;
  config_.test_goal_mission_index = goal_index;
  ball_mission_index_ =
      active_mission_ == MissionType::kBall ? ball_index : ball_index - 1;
  goal_mission_index_ =
      active_mission_ == MissionType::kGoal ? goal_index : goal_index - 1;
  return true;
}

ActionRequest MissionController::ComputeMissionLineRecovery(
    const MissionFrameInput &input, const ActionExecutionFeedback &feedback,
    bool *valid, MissionFrameResult *output) {
  *valid = false;
  if (mission_recovery_failed_)
    return {};
  output->line_features =
      ComputeLineFeatures(input.line_centers, input.image_width,
                          input.image_height, config_.line_features);
  output->line_computed = true;
  const auto &guide = output->line_features.guide;
  if (mission_recovery_verifying_) {
    mission_recovery_hits_.push_back(guide.valid);
    while (mission_recovery_hits_.size() >
           static_cast<std::size_t>(config_.line.line_stable_window))
      mission_recovery_hits_.pop_front();
    if (mission_recovery_hits_.size()==static_cast<std::size_t>(config_.line.line_stable_window) &&
        std::count(mission_recovery_hits_.begin(),mission_recovery_hits_.end(),true)>=
            config_.line.line_stable_min_hits)
      mission_recovery_qualified_=true;
    if (input.now_sec - mission_recovery_verify_start_sec_ + 1e-9 <
            config_.line_p2p.recovery_observation_sec ||
        mission_recovery_hits_.size() <
            static_cast<std::size_t>(config_.line.line_stable_window))
      return {};
    const bool confirmed = mission_recovery_qualified_;
    mission_recovery_verifying_ = false;
    mission_recovery_qualified_=false;
    if (confirmed) {
      *valid = true;
      line_state_ = LineState::kNormal;
      line_stability_history_ = mission_recovery_hits_;
      return {};
    }
    mission_recovery_found_ = false;
    mission_recovery_hits_.clear();
    if (mission_recovery_turns_ >= config_.mission_recovery_max_turns)
      mission_recovery_failed_ = true;
    return {};
  }
  mission_recovery_hits_.push_back(guide.valid);
  while (mission_recovery_hits_.size() >
         static_cast<std::size_t>(config_.line.line_stable_window))
    mission_recovery_hits_.pop_front();
  const int hits = std::count(mission_recovery_hits_.begin(),
                              mission_recovery_hits_.end(), true);
  mission_recovery_found_ =
      mission_recovery_found_ || hits >= config_.line.line_stable_min_hits;
  if (feedback.action_done && mission_recovery_issued_ &&
      (last_command_.action == MissionAction::kTurnLeft ||
       last_command_.action == MissionAction::kTurnRight)) {
    ++mission_recovery_turns_;
    mission_recovery_issued_ = false;
  }
  *valid = false;
  if (feedback.action_active && !feedback.action_done)
    return {};
  if (mission_recovery_found_ ||
      mission_recovery_turns_ >= config_.mission_recovery_max_turns) {
    mission_recovery_verifying_ = true;
    mission_recovery_qualified_=false;
    mission_recovery_verify_start_sec_ = input.now_sec;
    mission_recovery_hits_.clear();
    return {};
  }
  if (mission_recovery_turns_ >= config_.mission_recovery_max_turns) {
    mission_recovery_failed_ = true;
    return {};
  }
  if (mission_recovery_failed_)
    return {};
  const int index = active_mission_ == MissionType::kBall ? ball_mission_index_
                                                          : goal_mission_index_;
  return RecoveryTurn(index % 2 == 0, config_.mission_recovery_turn_yaw_deg);
}

double MissionController::ManualPreviewDurationSec() const {
  const bool uses_line_recovery =
      active_mission_ == MissionType::kLine ||
      (active_mission_ == MissionType::kBall &&
       ball_result_.mode == BallMode::kPostPickupLineRecovery) ||
      (active_mission_ == MissionType::kGoal &&
       goal_result_.mode == GoalMode::kHeadingRecovery);
  if (uses_line_recovery) {
    if (line_state_ == LineState::kFailureObserve)
      return std::max(config_.post_motion_observation_sec,
                      config_.line_p2p.failure_observation_sec);
    if (line_state_ == LineState::kRecoveryObserve)
      return std::max(config_.post_motion_observation_sec,
                      config_.line_p2p.recovery_observation_sec);
  }
  return config_.post_motion_observation_sec;
}

void MissionController::SetBallFineSideTwoStepMargin(double margin_px) {
  auto candidate = config_;
  candidate.ball.fine_side_two_step_margin_px = margin_px;
  ValidateAlgorithmConfig(candidate);
  config_.ball.fine_side_two_step_margin_px = margin_px;
  ball_controller_.UpdateTuning(config_.ball);
}

void MissionController::SetLivePerceptionTuning(
    const MissionControllerConfig &candidate) {
  auto updated = config_;
  updated.ball.approach_direct_down_v_px =
      candidate.ball.approach_direct_down_v_px;
  updated.ball.approach_left_top_u_px = candidate.ball.approach_left_top_u_px;
  updated.ball.approach_right_top_u_px = candidate.ball.approach_right_top_u_px;
  updated.ball.fine_motion = candidate.ball.fine_motion;
  updated.ball.down_search_max_steps = candidate.ball.down_search_max_steps;
  updated.ball.down_search_inter_action_hold_sec =
      candidate.ball.down_search_inter_action_hold_sec;
  updated.hurdle.down_search_max_steps = candidate.hurdle.down_search_max_steps;
  updated.hurdle.down_search_inter_action_hold_sec =
      candidate.hurdle.down_search_inter_action_hold_sec;
  updated.hurdle.contact_inter_action_hold_sec =
      candidate.hurdle.contact_inter_action_hold_sec;
  updated.ball.down_search_min_hits = candidate.ball.down_search_min_hits;
  updated.hurdle.down_search_min_hits = candidate.hurdle.down_search_min_hits;
  updated.line_p2p.in_place_turn_score = candidate.line_p2p.in_place_turn_score;
  updated.line_detection.min_box_width = candidate.line_detection.min_box_width;
  updated.line_detection.min_box_height =
      candidate.line_detection.min_box_height;
  updated.object_targets=candidate.object_targets;
  updated.ball_candidate_tracking=candidate.ball_candidate_tracking;
  updated.hurdle_candidate_tracking=candidate.hurdle_candidate_tracking;
  updated.backboard_candidate_tracking=candidate.backboard_candidate_tracking;
  ValidateAlgorithmConfig(updated);
  const bool changed =
      !config_.object_targets.SamePerceptionFilters(updated.object_targets) ||
      !(config_.ball_candidate_tracking==updated.ball_candidate_tracking) ||
      !(config_.hurdle_candidate_tracking==updated.hurdle_candidate_tracking) ||
      !(config_.backboard_candidate_tracking==updated.backboard_candidate_tracking) ||
      config_.line_detection.min_box_width !=
          updated.line_detection.min_box_width ||
      config_.line_detection.min_box_height !=
          updated.line_detection.min_box_height;
  config_ = updated;
  ball_controller_.UpdateTuning(config_.ball);
  hurdle_controller_.UpdateTuning(config_.hurdle);
  if (changed) {
    hurdle_target_tracker_.Reset(); backboard_target_tracker_.Reset();
    if(!config_.ball_candidate_tracking.enabled) {
      ball_target_locked_=ball_reacquiring_=false; ball_association_tracker_.Reset();
    }
    ball_candidate_tracker_.Reset();
    if(!ball_target_locked_) ball_target_id_=0;
    ClearObservationEvidence();
    ball_controller_.InvalidateDetectionEvidence();
    hurdle_controller_.InvalidateDetectionEvidence();
    goal_controller_.InvalidateDetectionEvidence();
    decision_observation_restart_pending_ = decision_observation_active_;
  }
}

void MissionController::SetGoalEdgeMargin(double margin_px) {
  auto candidate = config_;
  candidate.goal.edge_margin_px = margin_px;
  ValidateAlgorithmConfig(candidate);
  config_.goal.edge_margin_px = margin_px;
  goal_controller_.UpdateTuning(config_.goal);
}

bool MissionController::UpdateTuning(const MissionControllerConfig &candidate) {
  if (last_command_.action_id != 0)
    return false;
  // Parameter edits are allowed while a stationary preview is frozen.
  MissionControllerConfig updated = config_;
  updated.line_features.image_center_u = candidate.line_features.image_center_u;
  updated.line_p2p = candidate.line_p2p;
  updated.transition_settle_sec = candidate.transition_settle_sec;
  updated.walk_inter_action_hold_sec = candidate.walk_inter_action_hold_sec;
  updated.fine_motion_settle_sec = candidate.fine_motion_settle_sec;
  updated.ball = candidate.ball;
  updated.hurdle = candidate.hurdle;
  updated.goal = candidate.goal;
  updated.object_targets = candidate.object_targets;
  updated.ball_candidate_tracking=candidate.ball_candidate_tracking;
  updated.hurdle_candidate_tracking=candidate.hurdle_candidate_tracking;
  updated.backboard_candidate_tracking=candidate.backboard_candidate_tracking;
  updated.line_detection.min_box_width = candidate.line_detection.min_box_width;
  updated.line_detection.min_box_height =
      candidate.line_detection.min_box_height;
  updated.backboard_min_depth_m = candidate.backboard_min_depth_m;
  updated.backboard_max_depth_m = candidate.backboard_max_depth_m;
  updated.mission_recovery_max_turns = candidate.mission_recovery_max_turns;
  updated.backboard_depth_strip_width_px =
      candidate.backboard_depth_strip_width_px;
  ValidateAlgorithmConfig(updated);
  const bool box_filter_changed =
      !config_.object_targets.SamePerceptionFilters(updated.object_targets) ||
      !(config_.ball_candidate_tracking==updated.ball_candidate_tracking) ||
      !(config_.hurdle_candidate_tracking==updated.hurdle_candidate_tracking) ||
      !(config_.backboard_candidate_tracking==updated.backboard_candidate_tracking) ||
      config_.line_detection.min_box_width !=
          updated.line_detection.min_box_width ||
      config_.line_detection.min_box_height !=
          updated.line_detection.min_box_height;
  const bool depth_filter_changed =
      config_.backboard_min_depth_m != updated.backboard_min_depth_m ||
      config_.backboard_max_depth_m != updated.backboard_max_depth_m;
  // All existing offset accumulators were measured relative to the OLD
  // reference. Rebase in normalized coordinates without altering heading.
  if (last_line_image_width_ > 1) {
    const double delta = (config_.line_features.image_center_u -
                          updated.line_features.image_center_u) /
                         (0.5 * static_cast<double>(last_line_image_width_));
    if (delta != 0.0) {
      observation_offset_sum_ += delta * observation_valid_count_;
      decision_line_offset_sum_ += delta * decision_line_valid_count_;
      if (last_ready_line_guide_ && last_ready_line_guide_->valid)
        last_ready_line_guide_->offset += delta;
    }
  }
  config_ = updated;
  line_guide_accumulator_ =
      LineGuideAccumulator(config_.line_p2p);
  line_p2p_controller_ = LineP2pController(config_.line_p2p);
  line_plan_.UpdateConfig(config_.line_p2p);
  ball_controller_.UpdateTuning(config_.ball);
  goal_controller_.UpdateTuning(config_.goal);
  hurdle_controller_.UpdateTuning(config_.hurdle);
  if (box_filter_changed) {
    hurdle_target_tracker_.Reset(); backboard_target_tracker_.Reset();
    if(!config_.ball_candidate_tracking.enabled) {
      ball_target_locked_=ball_reacquiring_=false; ball_association_tracker_.Reset();
    }
    ball_candidate_tracker_.Reset();
    if(!ball_target_locked_) ball_target_id_=0;
    ClearObservationEvidence();
    ball_controller_.InvalidateDetectionEvidence();
    hurdle_controller_.InvalidateDetectionEvidence();
    goal_controller_.InvalidateDetectionEvidence();
    decision_observation_restart_pending_ = decision_observation_active_;
  }
  if (depth_filter_changed) {
    backboard_target_tracker_.Reset();
    goal_controller_.ClearObservationHistory();
  }
  return true;
}

bool MissionController::UpdateLineP2pTuning(const LineP2pConfig &line_p2p) {
  if (last_command_.action_id != 0 || line_observation_active_)
    return false;
  MissionControllerConfig candidate = config_;
  candidate.line_p2p = line_p2p;
  ValidateAlgorithmConfig(candidate);
  config_ = candidate;
  line_p2p_controller_ = LineP2pController(line_p2p);
  line_plan_.UpdateConfig(line_p2p);
  line_guide_accumulator_ =
      LineGuideAccumulator(line_p2p);
  return true;
}

void MissionController::ClearObservationEvidence() {
  approach_evidence_={};
  continuous_.hits.clear();continuous_.pose_hits.clear();continuous_.near_hits.clear();
  continuous_.missing=continuous_.blend=0;
  line_plan_.ClearEvidence();
  if (candidate_.phase == CandidatePhase::kObserve)
    decision_observation_restart_pending_ = true;
  line_guide_accumulator_.Reset();
  moving_object_action_id_ = 0;
  moving_object_samples_.clear();
  moving_object_ready_mean_.reset();
  line_stability_history_.clear();
  actual_line_stability_history_.clear();
  if(!ball_target_locked_ && !ball_reacquiring_) {
    ball_association_tracker_.Reset(); ball_candidate_tracker_.Reset(); ball_target_id_=0;
  }
  hurdle_target_tracker_.ClearUnpinned();
  backboard_target_tracker_.ClearUnpinned();
  ball_controller_.ClearObservationHistory();
  hurdle_controller_.ClearObservationHistory();
  goal_controller_.ClearObservationHistory();
  // Keep deferred DONE and the mission phase intact for RESUME.
}

void MissionController::Reset() { Reset(false); }

void MissionController::Reset(bool preserve_action_ids) {
  recovering_approach_.reset();
  continuous_={};approach_evidence_={};
  approach_goal_probe_=approach_return_forward_=false;
  approach_goal_empty_frames_=approach_goal_retry_after_=0;
  approach_goal_empty_retries_=0;
  approach_goal_search_failed_=false;
  approach_goal_depth_failed_=false;approach_goal_bad_depth_frames_=0;
  approach_goal_depth_action_id_=0;
  approach_camera_started_=-1;
  line_plan_.Reset();
  candidate_ = {};
  line_ready_handled_id_ = line_auxiliary_id_ = 0;
  line_auxiliary_done_ = line_auxiliary_observing_ = false;
  line_auxiliary_action_ = MissionAction::kNone;
  if (!preserve_action_ids)
    foot_pose_ = FootPose::kWalk;
  pose_change_pending_ = mission_finish_pending_ = physical_reset_pending_ =
      pose_observation_pending_ = false;
  reset_camera_command_sent_ = false;
  reset_walk_done_ = false;
  test_setup_camera_started_ = test_failed_ = false;
  test_setup_pose_done_ = false;
  fresh_after_search_ = test_setup_pending_ = test_entry_pending_ =
      test_execution_started_ = false;
  goal_trial_complete_ = false;
  ball_mission_index_ = config_.test_ball_mission_index - 1;
  goal_mission_index_ = config_.test_goal_mission_index - 1;
  mission_recovery_turns_ = 0;
  mission_recovery_issued_ = false;
  mission_recovery_failed_ = mission_recovery_found_ = false;
  mission_recovery_hits_.clear();
  mission_recovery_verifying_ = false;
  mission_recovery_qualified_ = false;
  post_pickup_goal_line_gate_active_ = false;
  post_pickup_line_actions_issued_ = post_pickup_line_actions_completed_ = 0;
  post_pickup_straight_blocks_completed_ = 0;
  post_pickup_five_step_completed_ = false;
  post_pickup_line_actions_.clear();
  active_mission_ = MissionType::kLine;
  has_ball_ = config_.initial_has_ball;
  line_state_ = LineState::kNormal;
  last_line_score_.reset();
  actual_line_stability_history_.clear();
  line_recovery_uses_visible_centers_ = line_recovery_left_ = false;
  line_observation_active_ = false;
  line_observation_hits_.clear();
  line_observation_qualified_=false;
  line_no_evidence_observations_=0;
  line_visible_recovery_turns_ = line_last_score_recovery_turns_ = 0;
  recovery_turn_issued_ = false;
  line_stability_history_.clear();
  line_guide_accumulator_.Reset();
  moving_object_action_id_ = 0;
  moving_object_samples_.clear();
  moving_object_ready_mean_.reset();
  object_queued_parent_id_ = 0;
  object_queued_successor_.reset();
  last_ready_line_guide_.reset();
  ready_line_action_id_ = 0;
  line_failure_pending_done_ = false;
  decision_observation_active_ = false;
  decision_observation_restart_pending_ = false;
  line_guard_turns_completed_ = 0;
  settle_until_sec_ = 0.0;
  last_settle_done_id_ = 0;
  decision_observation_camera_only_ = false;
  decision_observation_first_frame_ = false;
  deferred_done_ = {};
  decision_object_hits_.clear();
  decision_qualified_object_hits_.clear();
  decision_object_qualified_=false;
  decision_goal_pose_hits_.clear();
  decision_qualified_goal_pose_hits_.clear();
  decision_goal_pose_qualified_=false;
  decision_goal_near_hits_.clear();
  decision_line_hits_.clear();
  decision_line_qualified_=false;
  decision_observation_total_frames_ = 0;
  decision_last_target_.reset();
  decision_target_u_sum_ = decision_target_v_sum_ = 0.0;
  decision_rect_u_sum_ = decision_rect_v_sum_ = 0.0;
  decision_target_count_ = 0;
  decision_pose_x_sum_ = decision_pose_z_sum_ = 0.0;
  decision_pose_sin_sum_ = decision_pose_cos_sum_ = 0.0;
  decision_pose_count_ = 0;
  decision_line_valid_count_ = 0;
  decision_line_offset_sum_ = decision_line_heading_sum_ = 0.0;
  decision_line_confidence_sum_ = 0.0;
  previous_camera_settled_ = true;
  previous_camera_mode_ = CameraMode::kForward;
  object_tracking_camera_moving_ = false;
  object_tracking_camera_mode_ = CameraMode::kForward;
  ball_controller_.Reset();
  hurdle_controller_.Reset();
  goal_controller_.Reset();
  command_coordinator_.Reset(preserve_action_ids);
  ball_candidate_tracker_.Reset(); ball_target_id_=0;
  ball_target_locked_=ball_reacquiring_=ball_recovery_was_active_=false;
  ball_association_tracker_.Reset();
  backboard_target_tracker_.Reset();
  hurdle_target_tracker_.Reset();
  ball_result_ = {};
  hurdle_result_ = {};
  goal_result_ = {};
  last_command_ = {};
  if (has_ball_)
    goal_controller_.StartAfterPickup(0.0);
  post_pickup_goal_line_gate_active_ =
      has_ball_ && config_.enable_goal && !config_.goal_trial_start_goal;
  if (config_.goal_trial_start_goal) {
    active_mission_ = MissionType::kGoal;
    has_ball_ = true;
    goal_mission_index_ = config_.test_goal_mission_index;
    goal_controller_.StartGoalTrial();
    goal_result_.mode = GoalMode::kWaitCameraGoal;
  }
  InitializeTest();
}
void MissionController::BeginPhysicalReset() {
  Reset(true);
  physical_reset_pending_ = true;
  reset_walk_done_ = false;
  reset_camera_command_sent_ = false;
}
MissionFrameResult
MissionController::StepPhysicalReset(const CommandDeliveryFeedback &feedback,
                                     const CameraFeedback &camera,
                                     double now_sec) {
  MissionFrameResult result;
  if (!physical_reset_pending_) {
    result.physical_reset_complete = true;
    return result;
  }
  if (feedback.done && last_command_.action_id != 0 &&
      feedback.action_id == last_command_.action_id &&
      last_command_.action == MissionAction::kWalkMode) {
    reset_walk_done_ = true;
    foot_pose_ = FootPose::kWalk;
  }
  // Direct GOAL trials reset to their starting view. Dedicated staged tests
  // keep their existing FORWARD reset and prepare their own camera on START.
  const bool start_goal = config_.goal_trial_start_goal &&
                          config_.test_mission == MissionType::kNone;
  const auto wanted = start_goal ? CameraMode::kGoal : CameraMode::kForward;
  const auto request =
      start_goal ? CameraRequest::kGoal : CameraRequest::kForward;
  // Always force a NEW camera command, even when cached feedback matches.
  const bool camera_ready = reset_camera_command_sent_ && camera.settled &&
                            camera.actual_mode == wanted;
  result.command = command_coordinator_.Compute(
      MissionType::kLine, 902,
      reset_walk_done_ ? ActionRequest{}
                       : ActionRequest{MissionAction::kWalkMode,
                                       ActionCategory::kLocomotion, 0, false},
      camera_ready ? CameraRequest::kNone : request, feedback, now_sec);
  if (result.command.camera_request == request)
    reset_camera_command_sent_ = true;
  last_command_ = result.command;
  if (reset_walk_done_ && camera_ready && last_command_.action_id == 0) {
    physical_reset_pending_ = false;
    Reset(true);
    result.physical_reset_complete = true;
  }
  PopulateDiagnostics(result, now_sec);
  return result;
}

void MissionController::InitializeTest() {
  goal_start_camera_pending_ = config_.test_mission == MissionType::kNone &&
                               config_.goal_trial_start_goal;
  if (config_.test_mission == MissionType::kNone)
    return;
  post_pickup_goal_line_gate_active_ = false;
  active_mission_ = config_.test_mission;
  ball_result_ = {};
  hurdle_result_ = {};
  goal_result_ = {};
  test_execution_started_ = false;
  // GOAL stage 1 begins in the GOAL camera and approaches until close-walk
  // entry.
  test_setup_pending_ = true;
  test_entry_pending_ =
      !test_setup_pending_ && active_mission_ != MissionType::kGoal;
  has_ball_ = active_mission_ == MissionType::kGoal;
  if (active_mission_ == MissionType::kBall)
    ball_mission_index_ = config_.test_ball_mission_index;
  if (active_mission_ == MissionType::kGoal) {
    goal_mission_index_ = config_.test_goal_mission_index;
    if (!test_setup_pending_) {
      goal_controller_.StartForwardTrial();
      goal_result_.mode = GoalMode::kForwardApproach;
    }
  }
}
bool MissionController::SetTestStage(MissionType mission, int stage) {
  if (last_command_.action_id != 0 || pose_change_pending_ ||
      physical_reset_pending_ ||
      (mission != MissionType::kBall && mission != MissionType::kGoal &&
       mission != MissionType::kHurdle) ||
      stage < 0 || stage > 3)
    return false;
  config_.test_mission = mission;
  config_.test_stage = stage;
  Reset(true);
  return true;
}
bool MissionController::TestExecutionReady() const {
  if (config_.test_mission == MissionType::kBall)
    return ball_controller_.ExecutionObservationReady() &&
           ball_result_.action.action == MissionAction::kPickBall;
  if (config_.test_mission == MissionType::kGoal)
    return goal_controller_.ExecutionObservationReady() &&
           goal_result_.action.action == MissionAction::kShoot;
  if (config_.test_mission == MissionType::kHurdle)
    return hurdle_controller_.ExecutionObservationReady() &&
           hurdle_result_.tracked.center_px.v >= config_.hurdle.contact_v_px &&
           (hurdle_result_.mode == HurdleMode::kFineAdjust ||
            hurdle_result_.mode == HurdleMode::kContactWalk);
  return false;
}
bool MissionController::TestBoundaryReached(
    const CameraFeedback &camera) const {
  if (config_.test_stage == 2)
    return TestExecutionReady();
  if (config_.test_stage != 1 || !camera.settled)
    return false;
  if (config_.test_mission == MissionType::kBall)
    return camera.actual_mode == CameraMode::kDown &&
           ball_result_.mode == BallMode::kDownSearch;
  if (config_.test_mission == MissionType::kHurdle)
    return camera.actual_mode == CameraMode::kDown &&
           hurdle_result_.mode == HurdleMode::kDownSearch;
  return config_.test_mission == MissionType::kGoal &&
         camera.actual_mode == CameraMode::kGoal &&
         goal_result_.mode == GoalMode::kWaitFinePose;
}
ControlCommand MissionController::AcceptDeliveryFeedback(
    const CommandDeliveryFeedback &feedback, double now_sec) {
  // Keep last_command_ as the sensory FSM's matching edge until its next frame.
  if (feedback.done && feedback.action_id != 0 &&
      feedback.action_id == last_command_.action_id) {
    if (last_command_.action == MissionAction::kDefaultPoseMode)
      foot_pose_ = FootPose::kFine;
    if (last_command_.action == MissionAction::kWalkMode)
      foot_pose_ = FootPose::kWalk;
  }
  ObserveLineFeedback(feedback, now_sec);
  return command_coordinator_.Compute(active_mission_, 0, {},
                                      CameraRequest::kNone, feedback, now_sec);
}

} // namespace vision_core
