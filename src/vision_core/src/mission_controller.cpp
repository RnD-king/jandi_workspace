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
         action == MissionAction::kStepForwardFive ||
         action == MissionAction::kStepForwardRight;
}
ActionRequest LineLocomotion(MissionAction action, bool queue_eligible) {
  if (action == MissionAction::kNone) return {};
  return {action, ActionCategory::kLocomotion, 0, queue_eligible};
}
ActionRequest RecoveryTurn(bool left, int yaw_deg) {
  return {left ? MissionAction::kTurnLeft : MissionAction::kTurnRight,
          ActionCategory::kLocomotion,
          static_cast<std::int16_t>(std::max(0, yaw_deg)), false};
}
MissionControllerConfig ValidatedConfig(
    const MissionControllerConfig &config) {
  ValidateAlgorithmConfig(config);
  return config;
}
} // namespace

MissionController::MissionController()
    : MissionController(LoadDefaultAlgorithmConfig()) {}

MissionController::MissionController(const MissionControllerConfig &config)
    : config_(ValidatedConfig(config)),
      line_p2p_controller_(config_.line_p2p),
      line_guide_accumulator_(config_.line_p2p.failure_min_valid_samples),
      ball_controller_(config_.ball, config_.camera_motion_timeout_sec),
      hurdle_controller_(config_.hurdle, config_.camera_motion_timeout_sec),
      goal_controller_(config_.goal, config_.camera_motion_timeout_sec),
      command_coordinator_(config_.command),
      ball_association_tracker_(config_.object_association),
      backboard_association_tracker_(config_.object_association),
      hurdle_association_tracker_(config_.object_association),
      has_ball_(config_.initial_has_ball) {
  ball_mission_index_=config_.test_ball_mission_index-1;
  goal_mission_index_=config_.test_goal_mission_index-1;
  if (has_ball_) goal_controller_.StartAfterPickup(0.0);
  post_pickup_goal_line_gate_active_ =
      has_ball_ && config_.enable_goal && !config_.goal_trial_start_forward;
  if (config_.goal_trial_start_forward) {
    active_mission_=MissionType::kGoal; has_ball_=true;
    goal_mission_index_=config_.test_goal_mission_index;
    goal_controller_.StartForwardTrial();
    goal_result_.mode=GoalMode::kForwardApproach;
  }
  InitializeTest();
}

ActionRequest MissionController::CruiseAction(const LineGuide &guide,
                                               bool queue_eligible) {
  const CruiseDecision decision = line_p2p_controller_.Compute(guide);
  MissionAction action = MissionAction::kNone;
  if (decision.direction == CruiseDirection::kLeft)
    action = MissionAction::kStepForwardLeft;
  else if (decision.direction == CruiseDirection::kStraight)
    action = MissionAction::kStepForwardFive;
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
}

void MissionController::AddObservation(const LineGuide &guide) {
  if (!line_observation_active_ || !guide.valid ||
      !std::isfinite(guide.offset) || !std::isfinite(guide.heading_rad))
    return;
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
  if (observation_valid_count_ <
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
    const LineGuide &guide, const MissionFrameInput &input,
    bool action_active, bool action_done, bool *reference_valid) {
  if (!line_observation_active_ &&
      (line_state_ == LineState::kFailureObserve ||
       line_state_ == LineState::kRecoveryObserve))
    BeginObservation(line_state_, input.now_sec);

  const int stable_window = std::max(1, config_.line.line_stable_window);
  line_stability_history_.push_back(
      line_state_ == LineState::kNormal && guide.valid);
  while (static_cast<int>(line_stability_history_.size()) > stable_window)
    line_stability_history_.pop_front();
  const int hits = static_cast<int>(std::count(
      line_stability_history_.begin(), line_stability_history_.end(), true));
  if (reference_valid)
    *reference_valid = line_state_ == LineState::kNormal &&
        hits >= std::max(1, config_.line.line_stable_min_hits);

  if (line_state_ == LineState::kFinalHold) return {};
  if (line_state_ == LineState::kRecoveryTurn) {
    if (action_done) {
      recovery_turn_issued_ = false;
      BeginObservation(LineState::kRecoveryObserve, input.now_sec);
      AddObservation(guide);
      return {};
    }
    if (action_active || recovery_turn_issued_) return {};
    if (!input.allow_new_body_actions) return {};
    recovery_turn_issued_ = true;
    ++line_recovery_turns_;
    return RecoveryTurn(line_direction_evidence_ == DirectionEvidence::kLeft,
                        config_.line_p2p.recovery_turn_yaw_deg);
  }

  if (line_state_ == LineState::kFailureObserve ||
      line_state_ == LineState::kRecoveryObserve) {
    if (action_active) return {};
    AddObservation(guide);
    const double wait = std::max(0.0,
        line_state_ == LineState::kRecoveryObserve
            ? config_.line_p2p.recovery_observation_sec
            : config_.line_p2p.failure_observation_sec);
    if (input.now_sec + 1e-9 < line_observation_start_sec_ + wait) return {};
    const auto observed = FinishObservation();
    line_observation_active_ = false;
    if (observed) {
      line_state_ = LineState::kNormal;
      line_no_evidence_retries_ = line_recovery_turns_ = 0;
      const CruiseDecision d = line_p2p_controller_.Compute(*observed);
      if (d.direction == CruiseDirection::kLeft)
        line_direction_evidence_ = DirectionEvidence::kLeft;
      else if (d.direction == CruiseDirection::kRight)
        line_direction_evidence_ = DirectionEvidence::kRight;
      else if (d.direction == CruiseDirection::kStraight)
        line_direction_evidence_ = DirectionEvidence::kNone;
      return CruiseAction(*observed, true);
    }
    if (line_direction_evidence_ == DirectionEvidence::kNone) {
      if (line_no_evidence_retries_ <
          std::max(0, config_.line_p2p.no_evidence_max_retries)) {
        ++line_no_evidence_retries_;
        BeginObservation(LineState::kFailureObserve, input.now_sec);
      } else {
        line_state_ = LineState::kFinalHold;
      }
      return {};
    }
    if (line_recovery_turns_ <
        std::max(0, config_.line_p2p.recovery_max_turns)) {
      line_state_ = LineState::kRecoveryTurn;
      recovery_turn_issued_ = false;
      return ComputeLineAction(guide, input, false, false, reference_valid);
    }
    line_state_ = LineState::kFinalHold;
    return {};
  }

  if (action_active) return {};
  if (!guide.valid || !std::isfinite(guide.offset) ||
      !std::isfinite(guide.heading_rad)) {
    BeginObservation(LineState::kFailureObserve, input.now_sec);
    AddObservation(guide);
    return {};
  }
  const CruiseDecision d = line_p2p_controller_.Compute(guide);
  if (d.direction == CruiseDirection::kLeft)
    line_direction_evidence_ = DirectionEvidence::kLeft;
  else if (d.direction == CruiseDirection::kRight)
    line_direction_evidence_ = DirectionEvidence::kRight;
  else if (d.direction == CruiseDirection::kStraight)
    line_direction_evidence_ = DirectionEvidence::kNone;
  return CruiseAction(guide, true);
}

void MissionController::BeginLineReacquisition() {
  mission_recovery_turns_=0; mission_recovery_issued_=false;
  mission_recovery_failed_=false; mission_recovery_found_=false;
  mission_recovery_verifying_=false;
  mission_recovery_hits_.clear();
  line_direction_evidence_ = DirectionEvidence::kNone;
  line_no_evidence_retries_ = line_recovery_turns_ = 0;
  recovery_turn_issued_ = false;
  line_observation_active_ = false;
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
    ++ball_mission_index_;
    hurdle_controller_.Reset(); goal_controller_.Reset();
    hurdle_result_ = {}; goal_result_ = {};
  } else if (mission == MissionType::kHurdle) {
    goal_controller_.Reset(); ball_result_ = {}; goal_result_ = {};
    has_ball_ = false;
  } else if (mission == MissionType::kGoal) {
    ++goal_mission_index_;
    hurdle_controller_.Reset(); hurdle_result_ = {};
  }
}

void MissionController::FinishMission() {
  if (foot_pose_ != FootPose::kWalk) { mission_finish_pending_=true; return; }
  if (active_mission_ == MissionType::kBall && has_ball_ && config_.enable_goal) {
    // Start counting only after the BALL mission's confirmed line recovery,
    // not while rotating in place or restoring WALK/FINE foot stance.
    post_pickup_goal_line_gate_active_ = true;
    post_pickup_line_actions_issued_ = 0;
    post_pickup_line_actions_completed_ = 0;
    post_pickup_five_step_completed_ = false;
    post_pickup_line_actions_.clear();
  }
  if (config_.test_mission!=MissionType::kNone ||
      (active_mission_==MissionType::kGoal && config_.goal_trial_start_forward)) goal_trial_complete_=true;
  active_mission_ = MissionType::kLine;
  if (config_.test_mission==MissionType::kNone) {
    ball_result_ = {}; hurdle_result_ = {}; goal_result_ = {};
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
  const bool object_mission =
      active_mission_ == MissionType::kBall ||
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
  if (moving_object_ready_mean_) return;
  const bool matching =
      input.delivery_feedback.action_id == moving_object_action_id_;
  const bool executing =
      last_command_.control_phase == ControlPhase::kWaitingActionDone ||
      (matching && input.delivery_feedback.acknowledged);
  if (!executing || input.delivery_feedback.done) return;

  const std::optional<ObjectTarget> *target =
      active_mission_ == MissionType::kBall ? &input.ball_target :
      active_mission_ == MissionType::kHurdle ? &input.hurdle_target :
      &input.backboard_target;
  moving_object_samples_.push_back({
      input.now_sec, *target,
      active_mission_ == MissionType::kGoal
          ? input.backboard_center_depth_m : std::optional<double>{},
      active_mission_ == MissionType::kGoal ? input.goal_pose
                                           : GoalPoseObservation{}});
  // Bound transient evidence if a broken executor delays READY indefinitely.
  if (moving_object_samples_.size() > 450)
    moving_object_samples_.erase(moving_object_samples_.begin());
  if (!matching || !input.delivery_feedback.ready) return;

  const double ready_sec = input.now_sec;
  const double start_sec = moving_object_samples_.front().now_sec;
  const double total_sec = std::max(0.0, ready_sec - start_sec) +
                           std::max(0.0, config_.line_p2p.ready_lead_sec);
  const double cutoff = start_sec + 0.40 * total_sec;
  std::vector<const MovingObjectSample *> window;
  for (const auto &sample : moving_object_samples_)
    if (sample.now_sec >= cutoff && sample.now_sec <= ready_sec)
      window.push_back(&sample);
  // Preserve the same 10/7 and recency gate as the manual READY adapter.
  if (window.size() < 10) return;
  int last_ten_hits = 0;
  for (std::size_t i = window.size() - 10; i < window.size(); ++i)
    if (window[i]->target) ++last_ten_hits;
  if (last_ten_hits < 7) return;

  MovingObjectReadyMean mean;
  mean.action_id = moving_object_action_id_;
  mean.mission = active_mission_;
  mean.camera = moving_object_camera_;
  double sum_u=0.0, sum_v=0.0, sum_ru=0.0, sum_rv=0.0;
  double sum_x=0.0, sum_y=0.0, sum_w=0.0, sum_h=0.0;
  double sum_conf=0.0, total_weight=0.0, last_seen=-1.0;
  double depth_sum=0.0, pose_x=0.0, pose_z=0.0;
  double pose_sin=0.0, pose_cos=0.0, pose_last=-1.0;
  int hits=0, depth_hits=0, pose_hits=0;
  for (const auto *sample : window) {
    if (!sample->target) continue;
    mean.target = *sample->target;
    ++hits;
    last_seen = sample->now_sec;
    const double weight = active_mission_ == MissionType::kGoal ? 1.0 :
        1.0 + 2.0 * std::clamp(
            (sample->now_sec - cutoff) /
                std::max(1e-9, ready_sec - cutoff), 0.0, 1.0);
    total_weight += weight;
    const auto &t = *sample->target;
    sum_u += weight*t.center_px.u; sum_v += weight*t.center_px.v;
    // Some frames may lack IMU rectification; never average unset (0,0).
    const Point2 rect=t.center_rectified ? t.rectified_center_px : t.center_px;
    sum_ru += weight*rect.u; sum_rv += weight*rect.v;
    sum_x += weight*t.box_px.x; sum_y += weight*t.box_px.y;
    sum_w += weight*t.box_px.width; sum_h += weight*t.box_px.height;
    sum_conf += weight*t.confidence;
    if (active_mission_ == MissionType::kGoal) {
      if (sample->center_depth_m &&
          std::isfinite(*sample->center_depth_m) &&
          *sample->center_depth_m > 0.0) {
        depth_sum += *sample->center_depth_m;
        ++depth_hits;
      }
      if (sample->pose.valid && std::isfinite(sample->pose.x_m) &&
          std::isfinite(sample->pose.z_m) &&
          std::isfinite(sample->pose.yaw_rad)) {
        pose_x += sample->pose.x_m; pose_z += sample->pose.z_m;
        pose_sin += std::sin(sample->pose.yaw_rad);
        pose_cos += std::cos(sample->pose.yaw_rad);
        pose_last = sample->now_sec;
        ++pose_hits;
      }
    }
  }
  if (hits < 7 || total_weight <= 0.0 ||
      ready_sec - last_seen > 0.30) return;
  if (active_mission_ == MissionType::kGoal && depth_hits < 7) return;
  mean.target.center_px={sum_u/total_weight, sum_v/total_weight};
  mean.target.rectified_center_px={sum_ru/total_weight, sum_rv/total_weight};
  mean.target.box_px={sum_x/total_weight, sum_y/total_weight,
                      sum_w/total_weight, sum_h/total_weight};
  mean.target.width_px=mean.target.box_px.width;
  mean.target.height_px=mean.target.box_px.height;
  mean.target.area_px=mean.target.width_px*mean.target.height_px;
  mean.target.confidence=sum_conf/total_weight;
  if (depth_hits >= 7)
    mean.center_depth_m = depth_sum/static_cast<double>(depth_hits);
  if (pose_hits >= 7 && ready_sec-pose_last <= 0.30) {
    mean.pose.valid=true;
    mean.pose.x_m=pose_x/pose_hits;
    mean.pose.z_m=pose_z/pose_hits;
    mean.pose.yaw_rad=std::atan2(pose_sin,pose_cos);
    mean.pose.confidence=mean.target.confidence;
  }
  moving_object_ready_mean_ = mean;
}

bool MissionController::ShouldDeferDone(
    const CommandDeliveryFeedback &feedback) const {
  if (!feedback.done || config_.post_motion_observation_sec <= 0.0 ||
      feedback.action_id == 0 || feedback.action_id != last_command_.action_id)
    return false;
  if (RepeatScope()!=0) return false;
  const MissionAction action = last_command_.action;
  const MissionType mission = last_command_.mission;
  // The READY queue and the existing 1s/2s LINE recovery observation
  // must not be delayed by the generic post-motion gate.
  // Long LINE READY skips the gate ONLY while the active mission remains LINE.
  // An object entered during that walk must still collect fresh post-DONE data.
  if (mission == MissionType::kLine && active_mission_ == MissionType::kLine &&
      (IsLongLineAction(action) || line_state_ != LineState::kNormal))
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
  if (action == MissionAction::kDefaultPoseMode || action == MissionAction::kWalkMode ||
      (active_mission_ == MissionType::kBall && ball_result_.mode == BallMode::kApproachTwoBeforeDown) ||
      (active_mission_ == MissionType::kHurdle && hurdle_result_.mode == HurdleMode::kApproachTwoBeforeDown)) return false;
  // Fixed multi-motion chains have no perception-dependent successor.
  if (action == MissionAction::kPickBall ||
      action == MissionAction::kRecatch ||
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
  decision_observation_start_sec_ = now_sec;
  deferred_done_ = done;
  decision_object_hits_.clear();
  decision_goal_pose_hits_.clear();
  decision_observation_total_frames_ = 0;
  decision_object_weight_=0.0;
  decision_last_target_.reset();
  decision_target_u_sum_ = decision_target_v_sum_ = 0.0;
  decision_rect_u_sum_ = decision_rect_v_sum_ = 0.0;
  decision_target_count_ = 0;
  decision_last_seen_sec_ = 0.0;
  decision_pose_x_sum_ = decision_pose_z_sum_ = 0.0;
  decision_pose_sin_sum_ = decision_pose_cos_sum_ = 0.0;
  decision_pose_count_ = 0;
  decision_pose_last_seen_sec_ = 0.0;
  decision_line_valid_count_ = 0;
  decision_line_offset_sum_ = decision_line_heading_sum_ = 0.0;
  decision_line_confidence_sum_ = 0.0;
  if (!camera_only) {
    // Never reuse target stability or O/H evidence from the moving action.
    ball_association_tracker_.Reset();
    hurdle_association_tracker_.Reset();
    backboard_association_tracker_.Reset();
    ball_controller_.ClearObservationHistory();
    hurdle_controller_.ClearObservationHistory();
    goal_controller_.ClearObservationHistory();
    line_stability_history_.clear();
  }
}

void MissionController::AddDecisionObservation(const MissionFrameInput &input) {
  if (!decision_observation_active_) return;
  ++decision_observation_total_frames_;
  const std::optional<ObjectTarget> *target = nullptr;
  switch (decision_observation_mission_) {
  case MissionType::kBall: target = &input.ball_target; break;
  case MissionType::kHurdle: target = &input.hurdle_target; break;
  case MissionType::kGoal: target = &input.backboard_target; break;
  default: break;
  }
  if (decision_observation_mission_ != MissionType::kLine) {
    const bool detected = target && target->has_value();
    decision_object_hits_.push_back(detected);
    while (decision_object_hits_.size() > 10) decision_object_hits_.pop_front();
    if (decision_observation_mission_ == MissionType::kGoal) {
      const bool pose_valid = detected && input.goal_pose.valid &&
          std::isfinite(input.goal_pose.x_m) &&
          std::isfinite(input.goal_pose.z_m) &&
          std::isfinite(input.goal_pose.yaw_rad);
      decision_goal_pose_hits_.push_back(pose_valid);
      while (decision_goal_pose_hits_.size() > 10)
        decision_goal_pose_hits_.pop_front();
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
      const double weight=fine_adjustment ? 1.0 :
          1.0+2.0*std::clamp((input.now_sec-decision_observation_start_sec_)/
              std::max(1e-9,config_.post_motion_observation_sec),0.0,1.0);
      decision_object_weight_+=weight;
      decision_target_u_sum_ += weight*t.center_px.u;
      decision_target_v_sum_ += weight*t.center_px.v;
      const Point2 rect = t.center_rectified ? t.rectified_center_px : t.center_px;
      decision_rect_u_sum_ += weight*rect.u;
      decision_rect_v_sum_ += weight*rect.v;
      ++decision_target_count_;
      decision_last_seen_sec_ = input.now_sec;
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
      decision_pose_last_seen_sec_ = input.now_sec;
    }
    return;
  }
  const LineGuide guide = input.line_decision_guide_override
      ? *input.line_decision_guide_override
      : ComputeLineFeatures(input.line_centers, input.image_width,
                            input.image_height, config_.line_features).guide;
  if (guide.valid && std::isfinite(guide.offset) &&
      std::isfinite(guide.heading_rad)) {
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
          config_.post_motion_observation_sec) return false;
  if (decision_observation_mission_ == MissionType::kLine) return true;
  // Object 10/7 evidence is reserved for initial acquisition.
  // After DONE, end at one second; missing targets go to recovery FSM.
  return decision_object_hits_.size() >= 10;
}

std::optional<LineGuide> MissionController::DecisionLineGuide() const {
  if (decision_line_valid_count_ <
      std::max(1, config_.line_p2p.failure_min_valid_samples)) return std::nullopt;
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
    const MissionFrameInput &source,
    std::optional<double> line_image_center_u) {
  MissionFrameInput input = source;
  if (post_pickup_goal_line_gate_active_ &&
      input.delivery_feedback.done && input.delivery_feedback.action_id != 0) {
    const auto it = post_pickup_line_actions_.find(input.delivery_feedback.action_id);
    if (it != post_pickup_line_actions_.end()) {
      // The unique action ID is counted exactly once, only at physical DONE.
      ++post_pickup_line_actions_completed_;
      if (it->second == MissionAction::kStepForwardFive)
        post_pickup_five_step_completed_ = true;
      post_pickup_line_actions_.erase(it);
    }
  }
  TrackMovingObjectReadyWindow(input);
  // Compute B on a disposable DONE simulation. Never advance A's live FSM
  // until its real DONE. Manual tuning keeps Preview/Commit as the only admission.
  std::optional<ActionRequest> ready_object_successor;
  std::optional<MissionType> ready_object_successor_mission;
  if (!input.manual_observation_control && input.command_transport_enabled &&
      input.delivery_feedback.ready && !input.delivery_feedback.done &&
      moving_object_ready_mean_ &&
      moving_object_ready_mean_->action_id == input.delivery_feedback.action_id &&
      active_mission_ == moving_object_ready_mean_->mission &&
      last_command_.action_id == moving_object_ready_mean_->action_id &&
      input.allow_new_commands && input.allow_new_body_actions) {
    MissionController speculative = *this;
    MissionFrameInput complete = input;
    complete.delivery_feedback = {input.delivery_feedback.action_id, true, true, false};
    const MissionFrameResult proposed =
        speculative.StepWithLineImageCenter(complete, line_image_center_u);
    const ControlCommand &next = proposed.command;
    if (next.command_type == CommandType::kAction &&
        next.action_id != 0 &&
        next.action != MissionAction::kNone &&
        next.camera_request == CameraRequest::kNone) {
      ActionRequest b;
      b.action = next.action;
      b.category = next.action_category;
      b.target_yaw_deg = next.target_yaw_deg;
      b.line_queue_eligible = next.mission == MissionType::kLine
          ? IsLongLineAction(b.action) : config_.UsesObjectReadyMean(b.action);
      ready_object_successor = b;
      ready_object_successor_mission = next.mission;
    }
  }
  bool applied_object_ready_mean = false;
  if (moving_object_ready_mean_ &&
      input.delivery_feedback.done &&
      input.delivery_feedback.action_id == moving_object_ready_mean_->action_id &&
      active_mission_ == moving_object_ready_mean_->mission &&
      input.camera_feedback.settled &&
      input.camera_feedback.actual_mode == moving_object_ready_mean_->camera) {
    const auto mean = *moving_object_ready_mean_;
    // The cached mean already passed ten/7 detection checks. Reconstruct
    // tracker stability using that verified evidence, not a single raw DONE
    // frame, and let the owning mission FSM consume the real DONE immediately.
    if (active_mission_ == MissionType::kBall) {
      input.ball_target = mean.target;
      ball_controller_.ClearObservationHistory();
      for (int i=1; i<std::max(1,config_.ball.stable_min_hits); ++i)
        ball_controller_.ObserveOnly(input.ball_target,input.image_width,input.image_height);
      ball_controller_.UseNextObservationAsMean();
    } else if (active_mission_ == MissionType::kHurdle) {
      input.hurdle_target = mean.target;
      hurdle_controller_.ClearObservationHistory();
      for (int i=1; i<std::max(1,config_.hurdle.stable_min_hits); ++i)
        hurdle_controller_.ObserveOnly(input.hurdle_target,input.image_width,input.image_height);
      hurdle_controller_.UseNextObservationAsMean();
    } else {
      input.backboard_target = mean.target;
      input.backboard_center_depth_m = mean.center_depth_m;
      input.goal_pose = mean.pose;
      goal_controller_.ClearMovingApproachTriggers();
      goal_controller_.ClearObservationHistory();
      for (int i=1; i<std::max(1,config_.goal.stable_min_hits); ++i)
        goal_controller_.ObserveOnly(input.backboard_target,input.goal_pose,
                                     input.image_width,input.image_height);
      goal_controller_.UseNextObservationAsMean();
    }
    applied_object_ready_mean = true;
  }
  if (input.delivery_feedback.done) {
    moving_object_action_id_=0;
    moving_object_samples_.clear();
    moving_object_ready_mean_.reset();
  }
  // Consume the real completion edge for timing before any observation gate
  // defers it. Stationary post-DONE samples must never count as walking.
  command_coordinator_.ObserveExecutionTime(input.delivery_feedback, input.now_sec);
  const bool pose_done = input.delivery_feedback.done && last_command_.action_id != 0 &&
      input.delivery_feedback.action_id == last_command_.action_id &&
      (last_command_.action == MissionAction::kDefaultPoseMode || last_command_.action == MissionAction::kWalkMode);
  if (pose_done && !input.manual_observation_control) pose_observation_pending_=true;
  if (pose_done) foot_pose_ = last_command_.action == MissionAction::kWalkMode ? FootPose::kWalk : FootPose::kFine;
  if (pose_change_pending_) {
    MissionFrameResult pose_result;
    pose_result.command = command_coordinator_.Compute(active_mission_, 901, {},
        CameraRequest::kNone, input.delivery_feedback, input.now_sec);
    if (pose_result.command.action_id!=0 || pose_done) last_command_ = pose_result.command;
    if (pose_done) {
      pose_change_pending_ = false;
      const double pose_duration=std::max(0.0,input.now_sec-pose_started_sec_);
      ball_controller_.ShiftTimers(pose_duration); hurdle_controller_.ShiftTimers(pose_duration);
      goal_controller_.ShiftTimers(pose_duration);
      ClearObservationEvidence();
      if (mission_finish_pending_) { mission_finish_pending_=false; FinishMission(); }
    }
    pose_result.active_mission=active_mission_; pose_result.has_ball=has_ball_;
    pose_result.ball=ball_result_; pose_result.goal=goal_result_; pose_result.hurdle=hurdle_result_;
    pose_result.trial_complete=goal_trial_complete_;
    PopulateDiagnostics(pose_result,input.now_sec); return pose_result;
  }
  if (mission_finish_pending_) {
    MissionFrameResult pose_result;
    const ActionRequest pose_request = input.allow_new_commands && input.allow_new_body_actions
        ? ActionRequest{MissionAction::kWalkMode,ActionCategory::kLocomotion,0,false} : ActionRequest{};
    pose_result.command=command_coordinator_.Compute(active_mission_,901,pose_request,
        CameraRequest::kNone,input.delivery_feedback,input.now_sec);
    if (pose_result.command.action_id != 0) { pose_started_sec_=input.now_sec; pose_change_pending_=true; pose_target_=FootPose::kWalk; }
    last_command_=pose_result.command;
    pose_result.active_mission=active_mission_; pose_result.has_ball=has_ball_;
    pose_result.ball=ball_result_; pose_result.goal=goal_result_; pose_result.hurdle=hurdle_result_;
    PopulateDiagnostics(pose_result,input.now_sec); return pose_result;
  }
  const int incoming_repeat_scope=RepeatScope();
  MissionFrameResult output;
  output.object_ready_mean_used=applied_object_ready_mean;
  if (goal_trial_complete_ || test_failed_) {
    output.trial_complete=goal_trial_complete_; output.goal_mission_index=std::max(1,goal_mission_index_);
    output.ball_mission_index=std::max(1,ball_mission_index_);
    output.ball=ball_result_; output.hurdle=hurdle_result_; output.goal=goal_result_;
    output.active_mission=active_mission_; output.has_ball=has_ball_;
    PopulateDiagnostics(output,input.now_sec); return output;
  }
  if (test_setup_pending_) {
    const CameraMode wanted = config_.test_stage == 0 ||
            (config_.test_stage == 1 && config_.test_mission != MissionType::kGoal)
        ? CameraMode::kForward
        : (config_.test_mission == MissionType::kGoal
            ? CameraMode::kGoal : CameraMode::kDown);
    const bool camera_ready=input.camera_feedback.settled && input.camera_feedback.actual_mode==wanted;
    const MissionAction initial_pose = config_.test_stage == 3
        ? MissionAction::kDefaultPoseMode : MissionAction::kWalkMode;
    if (pose_done && last_command_.action == initial_pose)
      test_setup_pose_done_ = true;
    CameraRequest camera_request=CameraRequest::kNone;
    ActionRequest pose;
    if (!camera_ready && input.allow_new_commands) {
      if (!test_setup_camera_started_) { test_setup_camera_started_=true; test_setup_camera_start_sec_=input.now_sec; }
      if (input.now_sec-test_setup_camera_start_sec_>=config_.camera_motion_timeout_sec) test_failed_=true;
    }
    if (!camera_ready)
      camera_request=wanted==CameraMode::kGoal ? CameraRequest::kGoal :
          wanted==CameraMode::kDown ? CameraRequest::kDown : CameraRequest::kForward;
    if (!test_setup_pose_done_)
      pose={initial_pose,ActionCategory::kLocomotion,0,false};
    if (camera_ready && test_setup_pose_done_) {
      test_setup_pending_=false; ClearObservationEvidence();
      test_entry_pending_ = config_.test_stage <= 1 &&
          config_.test_mission != MissionType::kGoal;
      if (config_.test_stage <= 1) {
        if (config_.test_mission == MissionType::kGoal) {
          if (config_.test_stage == 0) {
            goal_controller_.StartForwardTrial();
            goal_result_.mode=GoalMode::kForwardApproach;
          } else {
            goal_controller_.StartTestNear();
            goal_result_.mode=GoalMode::kWaitCameraGoal;
          }
        }
      } else if (config_.test_mission==MissionType::kBall) {
        if (config_.test_stage==2) ball_controller_.StartTestNear(); else ball_controller_.StartTestExecute();
        ball_result_.mode=config_.test_stage==2 ? BallMode::kWaitCameraDown : BallMode::kFineAdjustForPickup;
      } else if (config_.test_mission==MissionType::kHurdle) {
        if (config_.test_stage==2) hurdle_controller_.StartTestNear(); else hurdle_controller_.StartTestExecute();
        hurdle_result_.mode=config_.test_stage==2 ? HurdleMode::kWaitCameraDown : HurdleMode::kFineAdjust;
      } else {
        if (config_.test_stage<=2) goal_controller_.StartTestNear(); else goal_controller_.StartTestExecute();
        goal_result_.mode=config_.test_stage<=2 ? GoalMode::kWaitCameraGoal : GoalMode::kFineAdjust;
      }
    }
    if (!input.allow_new_body_actions && !input.allow_test_setup_body_actions) pose={};
    if (!input.allow_new_commands || test_failed_) { pose={}; camera_request=CameraRequest::kNone; }
    output.command=command_coordinator_.Compute(active_mission_,903,pose,camera_request,
        input.delivery_feedback,input.now_sec);
    last_command_=output.command;
    output.active_mission=active_mission_; output.has_ball=has_ball_;
    output.ball=ball_result_; output.goal=goal_result_; output.hurdle=hurdle_result_;
    PopulateDiagnostics(output,input.now_sec); return output;
  }
  if (input.image_width > 1) last_line_image_width_ = input.image_width;
  // Observe the camera transition edge even if no ActionCommand is pending.
  const bool camera_settle_edge =
      (!previous_camera_settled_ ||
       previous_camera_mode_ == CameraMode::kTransition) &&
      input.camera_feedback.settled &&
      (input.camera_feedback.actual_mode == CameraMode::kDown ||
       input.camera_feedback.actual_mode == CameraMode::kGoal);
  previous_camera_settled_ = input.camera_feedback.settled;
  previous_camera_mode_ = input.camera_feedback.actual_mode;
  if (fresh_after_search_) {
    fresh_after_search_=false;
    if (!input.manual_observation_control)
      BeginDecisionObservation(input.now_sec,active_mission_,false,{});
  }
  // Deliver the pose's actual DONE to its owning FSM first. Its fresh
  // observation begins on the next frame and must never swallow that edge.
  if (pose_observation_pending_ && !input.manual_observation_control && !pose_done) {
    pose_observation_pending_=false;
    BeginDecisionObservation(input.now_sec,active_mission_,false,{});
  }
  if (input.manual_observation_control && RepeatScope()==0) {
    // DONE is always acknowledged by the coordinator immediately. Only a
    // deliberate PREVIEW may start the next one-second stationary window.
    if (input.begin_manual_observation)
      BeginDecisionObservation(input.now_sec, active_mission_, false, {});
  } else if (RepeatScope()==0) {
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
  bool observation_finished=false;
  bool observation_hold = decision_observation_active_;
  if (observation_hold) {
    if (!decision_observation_first_frame_ || !decision_observation_camera_only_)
      AddDecisionObservation(input);
    if (DecisionObservationReady(input.now_sec)) {
      observation_finished=true;
      if (!decision_observation_camera_only_)
        input.delivery_feedback = deferred_done_;
      // Object acquisition 10/7 applies only on entry. After DONE one
      // second of stationary samples defines the next measurement.
      if (decision_observation_mission_ != MissionType::kLine) {
        const int stable_hits = static_cast<int>(std::count(
            decision_object_hits_.begin(), decision_object_hits_.end(), true));
        const bool stable_object = decision_object_hits_.size() == 10 &&
            stable_hits >= 7;
        const bool recently_seen = decision_last_target_ &&
            decision_target_count_ > 0 &&
            input.now_sec - decision_last_seen_sec_ <= 0.30;
        // A single last-minute false detection is NOT a stationary mean.
        std::optional<ObjectTarget> mean;
        if (stable_object && recently_seen) {
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
          if (mean) ball_controller_.UseNextObservationAsMean();
        } else if (decision_observation_mission_ == MissionType::kHurdle) {
          input.hurdle_target = mean;
          if (mean) hurdle_controller_.UseNextObservationAsMean();
        } else if (decision_observation_mission_ == MissionType::kGoal) {
          input.backboard_target = mean;
          input.goal_pose = {};
          if (mean) goal_controller_.UseNextObservationAsMean();
          const int stable_pose_hits = static_cast<int>(std::count(
              decision_goal_pose_hits_.begin(), decision_goal_pose_hits_.end(), true));
          if (mean && decision_goal_pose_hits_.size() == 10 &&
              stable_pose_hits >= 7 && decision_pose_count_ > 0 &&
              input.now_sec - decision_pose_last_seen_sec_ <= 0.30) {
            const double count = static_cast<double>(decision_pose_count_);
            input.goal_pose.valid = true;
            input.goal_pose.x_m = decision_pose_x_sum_ / count;
            input.goal_pose.z_m = decision_pose_z_sum_ / count;
            input.goal_pose.yaw_rad = std::atan2(
                decision_pose_sin_sum_, decision_pose_cos_sum_);
            input.goal_pose.confidence = mean->confidence;
          }
        }
      }
      if (decision_observation_mission_ == MissionType::kLine) {
        const auto averaged = DecisionLineGuide();
        if (averaged) input.line_decision_guide_override = *averaged;
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
    output.line_features = ComputeLineFeatures(input.line_centers,
        input.image_width, input.image_height, config_.line_features);
    output.line_computed = true;
    if (active_mission_ == MissionType::kBall)
      ball_controller_.ObserveOnly(input.ball_target, input.image_width,
                                    input.image_height);
    else if (active_mission_ == MissionType::kHurdle)
      hurdle_controller_.ObserveOnly(input.hurdle_target, input.image_width,
                                      input.image_height);
    else if (active_mission_ == MissionType::kGoal)
      goal_controller_.ObserveOnly(input.backboard_target, input.goal_pose,
                                    input.image_width, input.image_height);
    output.command = command_coordinator_.Compute(active_mission_, 0, {},
        CameraRequest::kNone, input.delivery_feedback, input.now_sec);
    output.post_motion_observing = true;
    output.post_motion_frames = decision_observation_total_frames_;
    output.post_motion_window_frames = decision_object_hits_.size();
    output.post_motion_valid = decision_observation_mission_ == MissionType::kLine
        ? decision_line_valid_count_
        : static_cast<int>(std::count(decision_object_hits_.begin(),
                                      decision_object_hits_.end(), true));
    output.post_motion_elapsed_sec =
        std::max(0.0, input.now_sec - decision_observation_start_sec_);
    if (deferred_done_.action_id==0 || output.command.action_id!=0)
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
  decision_observation_first_frame_ = false;
  const int line_turns_before=line_recovery_turns_;
  ActionExecutionFeedback action_feedback = input.action_feedback;
  const bool matching_feedback = last_command_.action_id != 0 &&
      input.delivery_feedback.action_id == last_command_.action_id;
  if (input.command_transport_enabled) {
    action_feedback.action_done = matching_feedback &&
                                  input.delivery_feedback.done;
    action_feedback.action_active = last_command_.action_id != 0 &&
                                    !action_feedback.action_done;
  }
  action_feedback.observation_complete=
      observation_finished || applied_object_ready_mean;
  bool line_reference_valid = false;
  ActionRequest line_action;
  const auto compute_line = [&]() {
    FeatureConfig cfg = config_.line_features;
    if (line_image_center_u && std::isfinite(*line_image_center_u) &&
        *line_image_center_u >= 0.0 &&
        *line_image_center_u < static_cast<double>(input.image_width))
      cfg.image_center_u = *line_image_center_u;
    output.line_features = ComputeLineFeatures(
        input.line_centers, input.image_width, input.image_height, cfg);
    output.line_computed = true;
    if (!input.advance_line_fsm) return;
    const bool pending = last_command_.action_id != 0 &&
        last_command_.mission == MissionType::kLine &&
        last_command_.action_category == ActionCategory::kLocomotion;
    const bool queued_wait = pending &&
        (last_command_.control_phase ==
             ControlPhase::kWaitingQueuedActionAck ||
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
        ? *input.line_decision_guide_override : output.line_features.guide;
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
                ? config_.line_p2p.ready_lead_sec : 0.0);
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
    line_action = ComputeLineAction(decision, input, active, done,
                                    &line_reference_valid);
  };

  const auto hurdle_line_decision = [&]() {
    // Compute LINE O/H steering during HURDLE without advancing LINE FSM.
    const LineGuide guide = input.line_decision_guide_override.value_or(
        ComputeLineFeatures(input.line_centers, input.image_width,
                            input.image_height, config_.line_features).guide);
    return line_p2p_controller_.Compute(guide);
  };
  switch (active_mission_) {
  case MissionType::kLine:
    compute_line(); ball_result_ = {}; hurdle_result_ = {}; goal_result_ = {};
    // FINAL HOLD는 명시적 Reset 전까지 완전한 terminal 상태다. 라인 action뿐
    // 아니라 새로운 object mission 진입도 허용하지 않는다.
    if (line_state_ == LineState::kFinalHold) break;
    // PAUSE cannot let incidental moving-frame detections switch missions.
    // Existing BALL/GOAL/HURDLE missions are still stepped to process DONE.
    if (!input.allow_new_mission_entry) break;
    if (has_ball_) {
      if (config_.enable_goal) {
        BallResult carrying; carrying.has_ball = true;
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
                 config_.goal.post_pickup_walk_fallback_actions));
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
          EnterMission(MissionType::kHurdle); break;
        }
      }
      if (config_.enable_ball) {
        ball_result_ = ball_controller_.Compute(
            input.ball_target, input.image_width, input.image_height,
            input.now_sec, input.camera_feedback, action_feedback,
            input.allow_new_body_actions && input.allow_new_commands);
        if (ball_result_.mode != BallMode::kLineFollow)
          EnterMission(MissionType::kBall);
      }
    }
    break;
  case MissionType::kBall: {
    const BallMode before = ball_result_.mode;
    const bool controller_allow=input.allow_new_body_actions && input.allow_new_commands && !(config_.test_mission!=MissionType::kNone && config_.test_stage==2 && before==BallMode::kFineAdjustForPickup);
    if (before == BallMode::kPostPickupLineRecovery)
      line_action=ComputeMissionLineRecovery(input,action_feedback,&line_reference_valid,&output);
    ball_result_ = ball_controller_.Compute(
        input.ball_target, input.image_width, input.image_height,
        input.now_sec, input.camera_feedback, action_feedback,
            controller_allow);
    if (ball_result_.mode!=BallMode::kLineFollow) test_entry_pending_=false;
    if (before==BallMode::kDownSearch && ball_result_.mode==BallMode::kCoarseAdjust)
      fresh_after_search_=true;
    has_ball_ = ball_result_.has_ball;
    if (ball_result_.mode==BallMode::kPostPickupLineRecovery) ball_result_.action=line_action;
    if (before != BallMode::kPostPickupLineRecovery &&
        ball_result_.mode == BallMode::kPostPickupLineRecovery)
      BeginLineReacquisition();
    if (ball_result_.mode == BallMode::kPostPickupLineRecovery &&
        line_reference_valid) {
      ball_controller_.CompleteLineRecovery(input.now_sec);
      if (has_ball_ && config_.enable_goal)
        goal_controller_.UpdateBallState(ball_result_);
      FinishMission();
    } else if (ball_result_.mode == BallMode::kLineFollow &&
               !ball_result_.active && !test_entry_pending_) {
      BeginLineReacquisition(); FinishMission();
    }
    break;
  }
  case MissionType::kHurdle: {
    const auto before=hurdle_result_.mode;
    const bool controller_allow=input.allow_new_body_actions && input.allow_new_commands && !(config_.test_mission!=MissionType::kNone && config_.test_stage==2 && before==HurdleMode::kFineAdjust);
    hurdle_result_ = hurdle_controller_.Compute(
        input.hurdle_target, input.image_width, input.image_height,
        input.now_sec, input.camera_feedback, action_feedback,
        hurdle_line_decision(), controller_allow);
    if (hurdle_result_.mode!=HurdleMode::kLineFollow) test_entry_pending_=false;
    if (before==HurdleMode::kDownSearch && hurdle_result_.mode==HurdleMode::kCoarseAdjust)
      fresh_after_search_=true;
    if (hurdle_result_.mode == HurdleMode::kLineFollow &&
        !hurdle_result_.active && !test_entry_pending_) {
      ball_controller_.ClearEntryEvidence();
      BeginLineReacquisition(); FinishMission();
    }
    break;
  }
  case MissionType::kGoal: {
    const GoalMode before = goal_result_.mode;
    const bool controller_allow=input.allow_new_body_actions && input.allow_new_commands && !(config_.test_mission!=MissionType::kNone && config_.test_stage==2 && before==GoalMode::kFineAdjust);
    if (before == GoalMode::kHeadingRecovery)
      line_action=ComputeMissionLineRecovery(input,action_feedback,&line_reference_valid,&output);
    goal_result_ = goal_controller_.Compute(
        input.goal_target, input.backboard_target, input.goal_pose,
        input.image_width, input.image_height, input.now_sec,
        line_reference_valid, input.camera_feedback, action_feedback,
            controller_allow, input.backboard_center_depth_m);
    if (goal_result_.mode==GoalMode::kHeadingRecovery) goal_result_.action=line_action;
    has_ball_ = goal_controller_.HasBall();
    ball_controller_.SetHasBall(has_ball_);
    if (before != GoalMode::kHeadingRecovery &&
        goal_result_.mode == GoalMode::kHeadingRecovery)
      BeginLineReacquisition();
    if (goal_result_.mode == GoalMode::kLineFollow && !goal_result_.active)
      FinishMission();
    break;
  }
  default:
    active_mission_ = MissionType::kLine; compute_line(); break;
  }

  ActionRequest request;
  CameraRequest camera_request = CameraRequest::kNone;
  int phase = 0;
  if (active_mission_ == MissionType::kBall) {
    request = ball_result_.action; camera_request = ball_result_.camera_request;
    phase = static_cast<int>(ball_result_.mode);
  } else if (active_mission_ == MissionType::kHurdle) {
    request = hurdle_result_.action;
    camera_request = hurdle_result_.camera_request;
    phase = static_cast<int>(hurdle_result_.mode);
  } else if (active_mission_ == MissionType::kGoal) {
    request = goal_result_.action; camera_request = goal_result_.camera_request;
    phase = static_cast<int>(goal_result_.mode);
  } else {
    request = line_action; phase = static_cast<int>(line_state_);
  }
  if (config_.test_mission!=MissionType::kNone && !action_feedback.action_active &&
      TestBoundaryReached(input.camera_feedback)) {
    goal_trial_complete_=true; request={}; camera_request=CameraRequest::kNone;
  }
  // Alignment trials must never dispatch the mission primitive, including
  // while the full observation window is still accumulating.
  if (config_.test_mission!=MissionType::kNone && config_.test_stage==2 &&
      (request.action==MissionAction::kPickBall || request.action==MissionAction::kShoot ||
       request.action==MissionAction::kHurdle ||
       (config_.test_mission==MissionType::kHurdle &&
        hurdle_result_.mode==HurdleMode::kFineAdjust &&
        hurdle_result_.tracked.center_px.v>=config_.hurdle.contact_v_px))) request={};
  if (config_.test_mission!=MissionType::kNone && config_.test_stage==3 &&
      !test_execution_started_ && !TestExecutionReady()) request={};
  if (mission_finish_pending_) {
    request={MissionAction::kWalkMode,ActionCategory::kLocomotion,0,false};
    camera_request=CameraRequest::kNone; phase=901;
  }
  // All recovery turns and resumed normal walking obey the same stance contract.
  const int action_number=static_cast<int>(request.action);
  const bool wants_walk=action_number==9 || (action_number>=10 && action_number<=13) ||
      (action_number>=21 && action_number<=28);
  const bool wants_fine=(action_number>=1 && action_number<=8) ||
      (action_number>=14 && action_number<=17);
  const bool implicit_pose=request.action!=MissionAction::kDefaultPoseMode &&
      request.action!=MissionAction::kWalkMode &&
      ((wants_walk && foot_pose_!=FootPose::kWalk) || (wants_fine && foot_pose_!=FootPose::kFine));
  if (implicit_pose) {
    if (active_mission_==MissionType::kLine && wants_fine) {
      recovery_turn_issued_=false; line_recovery_turns_=line_turns_before;
    }
    request={wants_walk ? MissionAction::kWalkMode : MissionAction::kDefaultPoseMode,
             ActionCategory::kLocomotion,0,false};
    camera_request=CameraRequest::kNone; phase=901;
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
  if (active_mission_ == MissionType::kLine &&
      !input.allow_new_line_action)
    request = {};
  // Stop booking another cruise once the current 13 (five-step) or the
  // third post-pickup LINE walk is committed. GOAL camera follows its DONE.
  // Recovery turns remain available if the line itself becomes invalid.
  if (active_mission_ == MissionType::kLine &&
      post_pickup_goal_line_gate_active_ &&
      request.category == ActionCategory::kLocomotion &&
      IsLongLineAction(request.action) &&
      (post_pickup_line_actions_issued_ >=
           config_.goal.post_pickup_walk_fallback_actions ||
       (last_command_.mission == MissionType::kLine &&
        last_command_.action_id != 0 &&
        last_command_.action == MissionAction::kStepForwardFive)))
    request = {};
  // Manual tuning freezes NEW body actions but permits automatic camera
  // requests; unlike allow_new_commands, feedback still advances.
  if (!input.allow_new_body_actions ||
      (incoming_repeat_scope!=0 && RepeatScope()!=incoming_repeat_scope)) request = {};
  // Global start/pause gate: do not originate fresh action or camera
  // commands. The coordinator below must still consume pending feedback.
  if (!input.allow_new_commands) {
    request = {};
    camera_request = CameraRequest::kNone;
  }
  if (observation_hold) {
    request = {};
    camera_request = CameraRequest::kNone;
  }
  if (command_mission != MissionType::kLine &&
      request.category == ActionCategory::kLocomotion &&
      config_.UsesObjectReadyMean(request.action))
    request.line_queue_eligible = true;
  // Transport can have consumed A DONE and even B DONE ahead of RGB processing.
  // Once B was pre-booked, A's late sensory DONE must not dispatch B again.
  const bool queued_parent_done = object_queued_parent_id_ != 0 &&
      input.delivery_feedback.done &&
      input.delivery_feedback.action_id == object_queued_parent_id_;
  if (queued_parent_done) request = {};
  output.command = command_coordinator_.Compute(
      command_mission, phase, request, camera_request,
      input.delivery_feedback, input.now_sec);
  if (active_mission_ == MissionType::kLine &&
      post_pickup_goal_line_gate_active_ &&
      output.command.command_type == CommandType::kAction &&
      output.command.action_id != 0 &&
      output.command.action_category == ActionCategory::kLocomotion &&
      IsLongLineAction(output.command.action) &&
      post_pickup_line_actions_.emplace(output.command.action_id,
                                        output.command.action).second)
    ++post_pickup_line_actions_issued_;
  if (ready_object_successor && moving_object_ready_mean_ &&
      output.command.command_type == CommandType::kAction &&
      output.command.control_phase == ControlPhase::kWaitingQueuedActionAck) {
    object_queued_parent_id_ = moving_object_ready_mean_->action_id;
    object_queued_successor_ = output.command;
  }
  if (config_.test_mission!=MissionType::kNone && config_.test_stage==3 &&
      output.command.command_type==CommandType::kAction && TestExecutionReady())
    test_execution_started_=true;
  if (output.command.action_id != 0 && phase==901 &&
      (output.command.action==MissionAction::kDefaultPoseMode || output.command.action==MissionAction::kWalkMode)) {
    pose_started_sec_=input.now_sec; pose_change_pending_=true;
    pose_target_=output.command.action==MissionAction::kWalkMode ? FootPose::kWalk : FootPose::kFine;
  }
  if (input.advance_line_fsm && input.delivery_feedback.done &&
      input.delivery_feedback.action_id == ready_line_action_id_)
    ready_line_action_id_ = 0;
  // The transport may already have accepted DONE while an earlier ACK/READY
  // still awaits sensory processing. Keep its FSM ID until that DONE edge.
  // While B is reserved, last_command_ must still identify executing A.
  // Otherwise A's real DONE would be lost by object tracking and mission FSM.
  const bool object_queued_wait = object_queued_parent_id_ != 0 &&
      !queued_parent_done &&
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
             (output.command.action_id!=0 || action_feedback.action_done ||
              last_command_.action_id==0)) {
    last_command_ = output.command;
  }
  if (queued_parent_done) object_queued_parent_id_ = 0;
  if (object_queued_successor_ && input.delivery_feedback.done &&
      input.delivery_feedback.action_id == object_queued_successor_->action_id)
    object_queued_successor_.reset();
  output.active_mission = active_mission_;
  output.ball = ball_result_; output.hurdle = hurdle_result_;
  output.goal = goal_result_; output.has_ball = has_ball_;
  output.line_in_recovery = line_state_ != LineState::kNormal;
  output.trial_complete=goal_trial_complete_;
  output.repeat_scope=RepeatScope(); output.repeat_sequence=output.repeat_scope!=0;
  output.ball_mission_index=std::max(1,ball_mission_index_);
  output.goal_mission_index=std::max(1,goal_mission_index_);
  output.recovery_turns_completed=mission_recovery_turns_;
  output.recovery_failed=mission_recovery_failed_;
  PopulateDiagnostics(output, input.now_sec);
  return output;
}

void MissionController::PopulateDiagnostics(MissionFrameResult &output, double now) const {
  output.test_stage=config_.test_stage; output.test_ready=TestExecutionReady(); output.test_executing=test_execution_started_;
  output.terminal_hold=IsTerminalHold();
  output.foot_pose=foot_pose_;
  output.foot_transition=pose_change_pending_ ||
      (last_command_.action_id!=0 && (last_command_.action==MissionAction::kDefaultPoseMode || last_command_.action==MissionAction::kWalkMode));
  output.recovery_max_turns=config_.mission_recovery_max_turns;
  output.recovery_observation_frames=static_cast<int>(mission_recovery_hits_.size());
  output.repeat_scope = RepeatScope(); output.repeat_sequence = output.repeat_scope != 0;
  output.ball_mission_index = std::max(1, (active_mission_==MissionType::kBall || (goal_trial_complete_ && config_.test_mission==MissionType::kBall)) ? ball_mission_index_ : ball_mission_index_+1);
  output.goal_mission_index = std::max(1, (active_mission_==MissionType::kGoal || (goal_trial_complete_ &&
      (config_.test_mission==MissionType::kGoal ||
       (config_.test_mission==MissionType::kNone && config_.goal_trial_start_forward)))) ? goal_mission_index_ : goal_mission_index_+1);
  output.recovery_turns_completed = mission_recovery_turns_;
  output.recovery_failed = mission_recovery_failed_;
  output.recovery_verifying = mission_recovery_verifying_;
  output.goal_walk_actions_completed = post_pickup_line_actions_completed_;
  output.goal_walk_actions_remaining = std::max(0,
      config_.goal.post_pickup_walk_fallback_actions -
          post_pickup_line_actions_completed_);
  output.goal_five_step_completed = post_pickup_five_step_completed_;
  output.goal_walk_gate_active = has_ball_ && config_.enable_goal &&
      !config_.goal_trial_start_forward && (post_pickup_goal_line_gate_active_ ||
      (active_mission_ == MissionType::kGoal && goal_result_.mode == GoalMode::kWaitCameraGoal));
  output.goal.recovery_observations_completed=goal_controller_.RecoveryObservationsCompleted();
  output.goal.recovery_observation_elapsed_sec=goal_controller_.RecoveryObservationElapsed(now);
  output.ball.recovery_observations_completed = ball_controller_.RecoveryObservationsCompleted();
  output.ball.recovery_observation_elapsed_sec = ball_controller_.RecoveryObservationElapsed(now);
  output.ball.down_search_remaining_sec = ball_controller_.DownSearchRemaining(now);
  output.hurdle.recovery_observations_completed = hurdle_controller_.RecoveryObservationsCompleted();
  output.hurdle.recovery_observation_elapsed_sec = hurdle_controller_.RecoveryObservationElapsed(now);
  output.hurdle.down_search_remaining_sec = hurdle_controller_.DownSearchRemaining(now);
}

PerceptionMissionFrameResult MissionController::StepPerception(
    const PerceptionFrameInput &input) {
  PerceptionMissionFrameResult output;
  std::vector<Detection> detections;
  detections.reserve(input.detections.size());
  for (const auto &observation : input.detections) {
    Detection detection = observation.detection;
    if (detection.class_id == config_.object_targets.backboard_class_id) {
      const double lo = std::min(config_.backboard_min_depth_m,
                                 config_.backboard_max_depth_m);
      const double hi = std::max(config_.backboard_min_depth_m,
                                 config_.backboard_max_depth_m);
      if (!observation.center_depth_m ||
          !std::isfinite(*observation.center_depth_m) ||
          *observation.center_depth_m < lo ||
          *observation.center_depth_m > hi)
        detection.confidence = -std::numeric_limits<double>::infinity();
    }
    detections.push_back(detection);
  }
  output.perception.raw_line_centers =
      ExtractLineCenters(detections, config_.line_detection);
  output.perception.line_centers = output.perception.raw_line_centers;
  // A valid READY window is already associated and 10/7-filtered by
  // the manual adapter. Reacquire its averaged single detection instead of
  // associating it to a raw mid-walk position.
  if (input.manual_observation_control && input.manual_ready_mean) {
    if (active_mission_ == MissionType::kBall) {
      ball_association_tracker_.Reset();
      ball_controller_.UseNextObservationAsMean();
    } else if (active_mission_ == MissionType::kGoal) {
      backboard_association_tracker_.Reset();
      goal_controller_.UseNextObservationAsMean();
    } else if (active_mission_ == MissionType::kHurdle) {
      hurdle_association_tracker_.Reset();
      hurdle_controller_.UseNextObservationAsMean();
    }
  }
  // Never carry an association identity across a camera movement.
  const bool camera_moving = !input.camera_feedback.settled ||
      input.camera_feedback.actual_mode == CameraMode::kTransition;
  const CameraMode current_mode = input.camera_feedback.actual_mode;
  if (camera_moving || object_tracking_camera_moving_ ||
      current_mode != object_tracking_camera_mode_) {
    ball_association_tracker_.Reset();
    backboard_association_tracker_.Reset();
    hurdle_association_tracker_.Reset();
  }
  object_tracking_camera_moving_ = camera_moving;
  if (!camera_moving) object_tracking_camera_mode_ = current_mode;
  ObjectTargetSelection selection;
  const auto lower_raw_rank =
      [](const ObjectTargetCandidate &a, const ObjectTargetCandidate &b) {
        return a.target.confidence == b.target.confidence
            ? a.target.area_px < b.target.area_px
            : a.target.confidence < b.target.confidence;
      };
  const auto goals = ExtractObjectTargetCandidates(
      detections, config_.object_targets.goal_class_id,
      config_.object_targets.goal_confidence, config_.object_targets);
  if (!camera_moving && !goals.empty()) {
    const auto best = std::max_element(
        goals.begin(), goals.end(), lower_raw_rank);
    selection.targets.goal = best->target;
    selection.indices.goal = best->detection_index;
  }
  const auto ball_candidates = ExtractObjectTargetCandidates(
      detections, config_.object_targets.ball_class_id,
      config_.object_targets.ball_confidence, config_.object_targets);
  ObjectAssociationSelection ball;
  if (camera_moving) {
    // No candidates may become tracked targets until the camera settles.
  } else if (active_mission_ == MissionType::kBall &&
      ball_result_.mode == BallMode::kVerifyPickupObservation) {
    // Verification asks whether a ball remains, not whether its old identity
    // matches. Reset so the next normal frame can acquire the new position.
    ball_association_tracker_.Reset();
    if (!ball_candidates.empty()) {
      const auto best = std::max_element(
          ball_candidates.begin(), ball_candidates.end(), lower_raw_rank);
      ball = {best->target, best->detection_index};
    }
  } else {
    ball = ball_association_tracker_.Update(
        ball_candidates, input.image_width, input.image_height);
  }
  const auto board = camera_moving ? ObjectAssociationSelection{} :
      backboard_association_tracker_.Update(
      ExtractObjectTargetCandidates(detections,
          config_.object_targets.backboard_class_id,
          config_.object_targets.backboard_confidence, config_.object_targets),
      input.image_width, input.image_height);
  const auto hurdle = camera_moving ? ObjectAssociationSelection{} :
      hurdle_association_tracker_.Update(
      ExtractObjectTargetCandidates(detections,
          config_.object_targets.hurdle_class_id,
          config_.object_targets.hurdle_confidence, config_.object_targets),
      input.image_width, input.image_height);
  selection.targets.ball = ball.target; selection.indices.ball = ball.detection_index;
  selection.targets.backboard = board.target; selection.indices.backboard = board.detection_index;
  selection.targets.hurdle = hurdle.target; selection.indices.hurdle = hurdle.detection_index;
  output.perception.targets = selection.targets;

  if (input.enable_imu_rectification && input.imu_valid) {
    output.perception.line_centers = RectifyPixelPoints(
        output.perception.raw_line_centers, input.intrinsics,
        input.roll_rad, input.pitch_rad);
    const auto rectify = [&](std::optional<ObjectTarget> &target) {
      if (!target) return;
      const auto point = RectifyPixelPoints({target->center_px},
          input.intrinsics, input.roll_rad, input.pitch_rad);
      if (!point.empty()) {
        target->rectified_center_px = point.front();
        target->center_rectified = true;
      }
    };
    rectify(output.perception.targets.ball); rectify(output.perception.targets.goal);
    rectify(output.perception.targets.backboard); rectify(output.perception.targets.hurdle);
    output.perception.imu_rectification_applied = true;
  }
  if (selection.indices.backboard &&
      *selection.indices.backboard < input.detections.size()) {
    const auto &depth = input.detections[*selection.indices.backboard];
    const auto &backboard = output.perception.targets.backboard;
    output.perception.backboard_center_depth_m = depth.center_depth_m;
    output.perception.backboard_left_depth_m = depth.left_depth_m;
    output.perception.backboard_right_depth_m = depth.right_depth_m;
    output.perception.backboard_left_depth_u_px=depth.left_depth_u_px;
    output.perception.backboard_right_depth_u_px=depth.right_depth_u_px;
    const bool fine = active_mission_ == MissionType::kGoal &&
        (goal_result_.mode == GoalMode::kFineAdjust || goal_result_.mode == GoalMode::kCoarseAdjust ||
         goal_result_.mode == GoalMode::kFineRecovery || goal_result_.mode == GoalMode::kCoarseRecovery ||
         goal_result_.mode == GoalMode::kShoot);
    const bool near = depth.center_depth_m &&
        *depth.center_depth_m <= config_.goal.fine_adjust_start_z_m;
    if (backboard && depth.center_depth_m && depth.left_depth_m &&
        depth.right_depth_m && (near || fine)) {
      const double center = std::round(backboard->box_px.x +
                                       0.5 * backboard->box_px.width);
      const double left = std::round(backboard->box_px.x);
      const double right = std::round(backboard->box_px.x +
                                      backboard->box_px.width - 1.0);
      output.perception.goal_pose = EstimateGoalPoseFromBackboardDepths(
          center, *depth.center_depth_m, depth.left_depth_u_px.value_or(left), *depth.left_depth_m,
          depth.right_depth_u_px.value_or(right), *depth.right_depth_m, input.intrinsics,
          backboard->confidence);
    }
  }
  MissionFrameInput prepared;
  prepared.line_centers = output.perception.line_centers;
  prepared.ball_target = output.perception.targets.ball;
  prepared.hurdle_target = output.perception.targets.hurdle;
  prepared.goal_target = output.perception.targets.goal;
  prepared.backboard_target = output.perception.targets.backboard;
  prepared.goal_pose = output.perception.goal_pose;
  prepared.backboard_center_depth_m=output.perception.backboard_center_depth_m;
  prepared.image_width = input.image_width; prepared.image_height = input.image_height;
  prepared.now_sec = input.now_sec; prepared.camera_feedback = input.camera_feedback;
  prepared.command_transport_enabled = input.command_transport_enabled;
  prepared.action_feedback = input.action_feedback;
  prepared.delivery_feedback = input.delivery_feedback;
  prepared.allow_new_line_action = input.allow_new_line_action;
  prepared.allow_new_commands = input.allow_new_commands;
  prepared.allow_new_body_actions = input.allow_new_body_actions;
  prepared.allow_test_setup_body_actions = input.allow_test_setup_body_actions;
  prepared.allow_new_mission_entry = input.allow_new_mission_entry;
  prepared.advance_line_fsm = input.advance_line_fsm;
  prepared.manual_observation_control = input.manual_observation_control;
  prepared.begin_manual_observation = input.begin_manual_observation;
  prepared.line_decision_guide_override = input.line_decision_guide_override;
  // Line robot-reference is independent from the camera optical cx.
  std::optional<double> center = config_.line_features.image_center_u;
  output.mission = StepWithLineImageCenter(prepared, center);
  return output;
}

PerceptionMissionFrameResult MissionController::PreviewPerception(
    const PerceptionFrameInput &input) const {
  MissionController preview(*this);
  PerceptionFrameInput proposed = input;
  proposed.allow_new_body_actions = true;
  return preview.StepPerception(proposed);
}


int MissionController::RepeatScope() const {
  if (mission_recovery_failed_ || mission_finish_pending_ || pose_change_pending_) return 0;
  if (active_mission_==MissionType::kBall) {
    if (ball_result_.mode==BallMode::kDownSearch ||
        ball_result_.mode==BallMode::kPostPickupBackAway ||
        ball_result_.mode==BallMode::kPostPickupLineRecovery)
      return 200+static_cast<int>(ball_result_.mode);
  } else if (active_mission_==MissionType::kHurdle) {
    if (hurdle_result_.mode==HurdleMode::kDownSearch || hurdle_result_.mode==HurdleMode::kContactWalk)
      return 400+static_cast<int>(hurdle_result_.mode);
  } else if (active_mission_==MissionType::kGoal) {
    if (goal_result_.mode==GoalMode::kPostShootBackAway || goal_result_.mode==GoalMode::kHeadingRecovery)
      return 300+static_cast<int>(goal_result_.mode);
  }
  return 0;
}

bool MissionController::NeedsStationaryObservation() const {
  return (active_mission_ == MissionType::kBall &&
      (ball_result_.mode == BallMode::kBallRecoveryDown || ball_result_.mode == BallMode::kBallRecoveryForward)) ||
      (active_mission_ == MissionType::kHurdle &&
      (hurdle_result_.mode == HurdleMode::kRecoveryDown || hurdle_result_.mode == HurdleMode::kRecoveryForward)) ||
      (active_mission_ == MissionType::kGoal &&
       (goal_result_.mode==GoalMode::kFineRecovery || goal_result_.mode==GoalMode::kCoarseRecovery || goal_result_.mode==GoalMode::kRecoverySearch)) ||
      mission_recovery_verifying_;
}

bool MissionController::SetTestMissionIndices(int ball_index, int goal_index) {
  if (ball_index<1 || goal_index<1 || last_command_.action_id!=0) return false;
  config_.test_ball_mission_index=ball_index; config_.test_goal_mission_index=goal_index;
  ball_mission_index_=active_mission_==MissionType::kBall ? ball_index : ball_index-1;
  goal_mission_index_=active_mission_==MissionType::kGoal ? goal_index : goal_index-1;
  return true;
}

ActionRequest MissionController::ComputeMissionLineRecovery(
    const MissionFrameInput& input, const ActionExecutionFeedback& feedback,
    bool* valid, MissionFrameResult* output) {
  *valid=false;
  if (mission_recovery_failed_) return {};
  output->line_features=ComputeLineFeatures(input.line_centers,input.image_width,
      input.image_height,config_.line_features);
  output->line_computed=true;
  const auto& guide=output->line_features.guide;
  if (mission_recovery_verifying_) {
    mission_recovery_hits_.push_back(guide.valid);
    while (mission_recovery_hits_.size()>static_cast<std::size_t>(config_.line.line_stable_window))
      mission_recovery_hits_.pop_front();
    if (input.now_sec - mission_recovery_verify_start_sec_ + 1e-9 <
        config_.line_p2p.recovery_observation_sec ||
        mission_recovery_hits_.size()<static_cast<std::size_t>(config_.line.line_stable_window)) return {};
    const bool confirmed = guide.valid && mission_recovery_hits_.size() ==
        static_cast<std::size_t>(config_.line.line_stable_window) &&
        std::count(mission_recovery_hits_.begin(), mission_recovery_hits_.end(), true) >=
            config_.line.line_stable_min_hits;
    mission_recovery_verifying_ = false;
    if (confirmed) {
      *valid = true; line_state_ = LineState::kNormal;
      line_stability_history_ = mission_recovery_hits_; return {};
    }
    mission_recovery_found_ = false; mission_recovery_hits_.clear();
    if (mission_recovery_turns_ >= config_.mission_recovery_max_turns)
      mission_recovery_failed_ = true;
    return {};
  }
  mission_recovery_hits_.push_back(guide.valid);
  while (mission_recovery_hits_.size()>static_cast<std::size_t>(config_.line.line_stable_window))
    mission_recovery_hits_.pop_front();
  const int hits=std::count(mission_recovery_hits_.begin(),mission_recovery_hits_.end(),true);
  mission_recovery_found_=mission_recovery_found_ || hits>=config_.line.line_stable_min_hits;
  if (feedback.action_done && mission_recovery_issued_) {
    ++mission_recovery_turns_; mission_recovery_issued_=false;
  }
  *valid=false;
  if (feedback.action_active && !feedback.action_done) return {};
  if (mission_recovery_found_ || mission_recovery_turns_>=config_.mission_recovery_max_turns) {
    mission_recovery_verifying_ = true;
    mission_recovery_verify_start_sec_ = input.now_sec;
    mission_recovery_hits_.clear(); return {};
  }
  if (mission_recovery_turns_>=config_.mission_recovery_max_turns) {
    mission_recovery_failed_=true; return {};
  }
  if (mission_recovery_failed_) return {};
  if (input.allow_new_body_actions && input.allow_new_commands) mission_recovery_issued_=true;
  const int index=active_mission_==MissionType::kBall ? ball_mission_index_ : goal_mission_index_;
  return RecoveryTurn(index%2==0,config_.mission_recovery_yaw_deg);
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

bool MissionController::UpdateTuning(const MissionControllerConfig &candidate) {
  if (last_command_.action_id != 0) return false;
  // Parameter edits are allowed while a stationary preview is frozen.
  MissionControllerConfig updated = config_;
  updated.line_features.image_center_u = candidate.line_features.image_center_u;
  updated.line_p2p = candidate.line_p2p;
  updated.ball=candidate.ball;
  updated.hurdle=candidate.hurdle;
  updated.goal=candidate.goal;
  updated.backboard_min_depth_m=candidate.backboard_min_depth_m;
  updated.backboard_max_depth_m=candidate.backboard_max_depth_m;
  updated.mission_recovery_max_turns=candidate.mission_recovery_max_turns;
  updated.backboard_depth_strip_width_px=candidate.backboard_depth_strip_width_px;
  ValidateAlgorithmConfig(updated);
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
  line_guide_accumulator_ = LineGuideAccumulator(config_.line_p2p.failure_min_valid_samples);
  line_p2p_controller_ = LineP2pController(config_.line_p2p);
  ball_controller_.UpdateTuning(config_.ball);
  goal_controller_.UpdateTuning(config_.goal);
  hurdle_controller_.UpdateTuning(config_.hurdle);
  if (depth_filter_changed) {
    backboard_association_tracker_.Reset();
    goal_controller_.ClearObservationHistory();
  }
  return true;
}

bool MissionController::UpdateLineP2pTuning(const LineP2pConfig &line_p2p) {
  if (last_command_.action_id != 0 || line_observation_active_) return false;
  MissionControllerConfig candidate = config_;
  candidate.line_p2p = line_p2p;
  ValidateAlgorithmConfig(candidate);
  config_ = candidate;
  line_p2p_controller_ = LineP2pController(line_p2p);
  line_guide_accumulator_ = LineGuideAccumulator(line_p2p.failure_min_valid_samples);
  return true;
}

void MissionController::ClearObservationEvidence() {
  line_guide_accumulator_.Reset();
  moving_object_action_id_=0;
  moving_object_samples_.clear();
  moving_object_ready_mean_.reset();
  line_stability_history_.clear();
  ball_association_tracker_.Reset();
  hurdle_association_tracker_.Reset();
  backboard_association_tracker_.Reset();
  ball_controller_.ClearObservationHistory();
  hurdle_controller_.ClearObservationHistory();
  goal_controller_.ClearObservationHistory();
  // Keep deferred DONE and the mission phase intact for RESUME.
}

void MissionController::Reset() { Reset(false); }

void MissionController::Reset(bool preserve_action_ids) {
  if (!preserve_action_ids) foot_pose_=FootPose::kWalk;
  pose_change_pending_=mission_finish_pending_=physical_reset_pending_=pose_observation_pending_=false;
  reset_camera_command_sent_=false;
  reset_walk_done_=false;
  test_setup_camera_started_=test_failed_=false;
  test_setup_pose_done_=false;
  fresh_after_search_=test_setup_pending_=test_entry_pending_=test_execution_started_=false;
  goal_trial_complete_=false;
  ball_mission_index_=config_.test_ball_mission_index-1;
  goal_mission_index_=config_.test_goal_mission_index-1;
  mission_recovery_turns_=0; mission_recovery_issued_=false;
  mission_recovery_failed_=mission_recovery_found_=false; mission_recovery_hits_.clear();
  mission_recovery_verifying_ = false;
  post_pickup_goal_line_gate_active_ = false;
  post_pickup_line_actions_issued_ = post_pickup_line_actions_completed_ = 0;
  post_pickup_five_step_completed_ = false;
  post_pickup_line_actions_.clear();
  active_mission_ = MissionType::kLine;
  has_ball_ = config_.initial_has_ball;
  line_state_ = LineState::kNormal;
  line_direction_evidence_ = DirectionEvidence::kNone;
  line_observation_active_ = false;
  line_no_evidence_retries_ = line_recovery_turns_ = 0;
  recovery_turn_issued_ = false;
  line_stability_history_.clear(); line_guide_accumulator_.Reset();
  moving_object_action_id_=0;
  moving_object_samples_.clear();
  moving_object_ready_mean_.reset();
  object_queued_parent_id_ = 0;
  object_queued_successor_.reset();
  last_ready_line_guide_.reset();
  ready_line_action_id_ = 0;
  line_failure_pending_done_ = false;
  decision_observation_active_ = false;
  decision_observation_camera_only_ = false;
  decision_observation_first_frame_ = false;
  deferred_done_ = {};
  decision_object_hits_.clear();
  decision_goal_pose_hits_.clear();
  decision_observation_total_frames_ = 0;
  decision_last_target_.reset();
  decision_target_u_sum_ = decision_target_v_sum_ = 0.0;
  decision_rect_u_sum_ = decision_rect_v_sum_ = 0.0;
  decision_target_count_ = 0;
  decision_last_seen_sec_ = 0.0;
  decision_pose_x_sum_ = decision_pose_z_sum_ = 0.0;
  decision_pose_sin_sum_ = decision_pose_cos_sum_ = 0.0;
  decision_pose_count_ = 0;
  decision_pose_last_seen_sec_ = 0.0;
  decision_line_valid_count_ = 0;
  decision_line_offset_sum_ = decision_line_heading_sum_ = 0.0;
  decision_line_confidence_sum_ = 0.0;
  previous_camera_settled_ = true;
  previous_camera_mode_ = CameraMode::kForward;
  object_tracking_camera_moving_ = false;
  object_tracking_camera_mode_ = CameraMode::kForward;
  ball_controller_.Reset(); hurdle_controller_.Reset(); goal_controller_.Reset();
  command_coordinator_.Reset(preserve_action_ids); ball_association_tracker_.Reset();
  backboard_association_tracker_.Reset(); hurdle_association_tracker_.Reset();
  ball_result_ = {}; hurdle_result_ = {}; goal_result_ = {}; last_command_ = {};
  if (has_ball_) goal_controller_.StartAfterPickup(0.0);
  post_pickup_goal_line_gate_active_ =
      has_ball_ && config_.enable_goal && !config_.goal_trial_start_forward;
  if (config_.goal_trial_start_forward) {
    active_mission_=MissionType::kGoal; has_ball_=true;
    goal_mission_index_=config_.test_goal_mission_index;
    goal_controller_.StartForwardTrial(); goal_result_.mode=GoalMode::kForwardApproach;
  }
  InitializeTest();
}
void MissionController::BeginPhysicalReset() {
  Reset(true);
  physical_reset_pending_=true;
  reset_walk_done_=false;
  reset_camera_command_sent_=false;
}
MissionFrameResult MissionController::StepPhysicalReset(
    const CommandDeliveryFeedback &feedback,const CameraFeedback &camera,double now_sec) {
  MissionFrameResult result;
  if (!physical_reset_pending_) { result.physical_reset_complete=true; return result; }
  if (feedback.done && last_command_.action_id!=0 && feedback.action_id==last_command_.action_id &&
      last_command_.action==MissionAction::kWalkMode) { reset_walk_done_=true; foot_pose_=FootPose::kWalk; }
  // Force a NEW FORWARD command on every physical RESET even if the cached
  // mode already says FORWARD. In hardware the pending command sets camera
  // feedback to TRANSITION until its own new ID receives DONE.
  const bool camera_ready = reset_camera_command_sent_ &&
      camera.settled && camera.actual_mode==CameraMode::kForward;
  result.command=command_coordinator_.Compute(MissionType::kLine,902,
      reset_walk_done_ ? ActionRequest{} : ActionRequest{MissionAction::kWalkMode,ActionCategory::kLocomotion,0,false},
      camera_ready ? CameraRequest::kNone : CameraRequest::kForward,feedback,now_sec);
  if (result.command.camera_request == CameraRequest::kForward)
    reset_camera_command_sent_=true;
  last_command_=result.command;
  if (reset_walk_done_ && camera_ready && last_command_.action_id==0) {
    physical_reset_pending_=false; Reset(true); result.physical_reset_complete=true;
  }
  PopulateDiagnostics(result,now_sec); return result;
}

void MissionController::InitializeTest() {
  if (config_.test_mission==MissionType::kNone) return;
  active_mission_=config_.test_mission;
  ball_result_={}; hurdle_result_={}; goal_result_={};
  test_execution_started_=false;
  // GOAL stage 1 begins in the GOAL camera and approaches until close-walk entry.
  test_setup_pending_=true;
  test_entry_pending_=!test_setup_pending_ && active_mission_!=MissionType::kGoal;
  has_ball_=active_mission_==MissionType::kGoal;
  if (active_mission_==MissionType::kBall) ball_mission_index_=config_.test_ball_mission_index;
  if (active_mission_==MissionType::kGoal) {
    goal_mission_index_=config_.test_goal_mission_index;
    if (!test_setup_pending_) { goal_controller_.StartForwardTrial(); goal_result_.mode=GoalMode::kForwardApproach; }
  }
}
bool MissionController::SetTestStage(MissionType mission,int stage) {
  if (last_command_.action_id!=0 || pose_change_pending_ || physical_reset_pending_ ||
      (mission!=MissionType::kBall && mission!=MissionType::kGoal && mission!=MissionType::kHurdle) ||
      stage<0 || stage>3) return false;
  config_.test_mission=mission; config_.test_stage=stage; Reset(true); return true;
}
bool MissionController::TestExecutionReady() const {
  if (config_.test_mission==MissionType::kBall)
    return ball_controller_.ExecutionObservationReady() &&
        ball_result_.action.action==MissionAction::kPickBall;
  if (config_.test_mission==MissionType::kGoal)
    return goal_controller_.ExecutionObservationReady() &&
        goal_result_.action.action==MissionAction::kShoot;
  if (config_.test_mission==MissionType::kHurdle)
    return hurdle_controller_.ExecutionObservationReady() &&
        hurdle_result_.tracked.center_px.v>=config_.hurdle.contact_v_px &&
        (hurdle_result_.mode==HurdleMode::kFineAdjust || hurdle_result_.mode==HurdleMode::kContactWalk);
  return false;
}
bool MissionController::TestBoundaryReached(const CameraFeedback& camera) const {
  if (config_.test_stage==2) return TestExecutionReady();
  if (config_.test_stage!=1 || !camera.settled) return false;
  if (config_.test_mission==MissionType::kBall)
    return camera.actual_mode==CameraMode::kDown && ball_result_.mode==BallMode::kDownSearch;
  if (config_.test_mission==MissionType::kHurdle)
    return camera.actual_mode==CameraMode::kDown && hurdle_result_.mode==HurdleMode::kDownSearch;
  return config_.test_mission==MissionType::kGoal && camera.actual_mode==CameraMode::kGoal &&
      goal_result_.mode==GoalMode::kCoarseAdjust;
}
ControlCommand MissionController::AcceptDeliveryFeedback(const CommandDeliveryFeedback& feedback,double now_sec) {
  // Keep last_command_ as the sensory FSM's matching edge until its next frame.
  if (feedback.done && feedback.action_id!=0 && feedback.action_id==last_command_.action_id) {
    if (last_command_.action==MissionAction::kDefaultPoseMode) foot_pose_=FootPose::kFine;
    if (last_command_.action==MissionAction::kWalkMode) foot_pose_=FootPose::kWalk;
  }
  return command_coordinator_.Compute(active_mission_,0,{},CameraRequest::kNone,feedback,now_sec);
}

} // namespace vision_core
