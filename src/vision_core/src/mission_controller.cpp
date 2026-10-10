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
      ball_controller_(config_.ball, config_.camera_motion_timeout_sec),
      hurdle_controller_(config_.hurdle, config_.camera_motion_timeout_sec),
      goal_controller_(config_.goal, config_.camera_motion_timeout_sec),
      command_coordinator_(config_.command),
      ball_association_tracker_(config_.object_association),
      backboard_association_tracker_(config_.object_association),
      hurdle_association_tracker_(config_.object_association),
      has_ball_(config_.initial_has_ball) {
  if (has_ball_) goal_controller_.StartAfterPickup(0.0);
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
    hurdle_controller_.Reset(); goal_controller_.Reset();
    hurdle_result_ = {}; goal_result_ = {};
  } else if (mission == MissionType::kHurdle) {
    goal_controller_.Reset(); ball_result_ = {}; goal_result_ = {};
    has_ball_ = false;
  } else if (mission == MissionType::kGoal) {
    hurdle_controller_.Reset(); hurdle_result_ = {};
  }
}

void MissionController::FinishMission() {
  active_mission_ = MissionType::kLine;
  ball_result_ = {}; hurdle_result_ = {}; goal_result_ = {};
}

bool MissionController::ShouldDeferDone(
    const CommandDeliveryFeedback &feedback) const {
  if (!feedback.done || config_.post_motion_observation_sec <= 0.0 ||
      feedback.action_id == 0 || feedback.action_id != last_command_.action_id)
    return false;
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
  // Fixed multi-motion chains have no perception-dependent successor.
  if (action == MissionAction::kPickBall ||
      action == MissionAction::kRecatch ||
      action == MissionAction::kDefaultPosition ||
      action == MissionAction::kShoot ||
      (mission == MissionType::kBall && action == MissionAction::kStepBack &&
       ball_result_.mode == BallMode::kPostPickupBackAway) ||
      (mission == MissionType::kHurdle &&
       (action == MissionAction::kStepForwardOne ||
        action == MissionAction::kHurdle)))
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
  decision_observation_total_frames_ = 0;
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
  bool detected = false;
  switch (decision_observation_mission_) {
  case MissionType::kBall: detected = input.ball_target.has_value(); break;
  case MissionType::kHurdle: detected = input.hurdle_target.has_value(); break;
  case MissionType::kGoal: detected = input.backboard_target.has_value(); break;
  default: break;
  }
  if (decision_observation_mission_ != MissionType::kLine) {
    decision_object_hits_.push_back(detected);
    while (decision_object_hits_.size() > 10) decision_object_hits_.pop_front();
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
  // DOWN-camera HURDLE contact is a fixed sequence, not a new target choice.
  if (decision_observation_camera_only_ &&
      decision_observation_mission_ == MissionType::kHurdle) return true;
  return decision_object_hits_.size() >= 10 &&
      std::count(decision_object_hits_.begin(),
                 decision_object_hits_.end(), true) >= 7;
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
  MissionFrameResult output;
  // Observe the camera transition edge even if no ActionCommand is pending.
  const bool camera_settle_edge =
      (!previous_camera_settled_ ||
       previous_camera_mode_ == CameraMode::kTransition) &&
      input.camera_feedback.settled &&
      (input.camera_feedback.actual_mode == CameraMode::kDown ||
       input.camera_feedback.actual_mode == CameraMode::kGoal);
  previous_camera_settled_ = input.camera_feedback.settled;
  previous_camera_mode_ = input.camera_feedback.actual_mode;
  if (!decision_observation_active_ &&
      config_.post_motion_observation_sec > 0.0 && camera_settle_edge)
    BeginDecisionObservation(input.now_sec, active_mission_, true, {});
  if (!decision_observation_active_ && ShouldDeferDone(input.delivery_feedback))
    BeginDecisionObservation(input.now_sec, active_mission_, false,
                             input.delivery_feedback);
  // A deferred DONE remains pending at the coordinator while new, post-DONE
  // frames are collected. This makes the next action impossible before 1 s.
  bool observation_hold = decision_observation_active_;
  if (observation_hold) {
    if (!decision_observation_first_frame_ || !decision_observation_camera_only_)
      AddDecisionObservation(input);
    if (DecisionObservationReady(input.now_sec)) {
      if (!decision_observation_camera_only_)
        input.delivery_feedback = deferred_done_;
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
    last_command_ = output.command;
    output.active_mission = active_mission_;
    output.line_in_recovery = line_state_ != LineState::kNormal;
    output.ball = ball_result_;
    output.hurdle = hurdle_result_;
    output.goal = goal_result_;
    output.has_ball = has_ball_;
    return output;
  }
  decision_observation_first_frame_ = false;
  ActionExecutionFeedback action_feedback = input.action_feedback;
  const bool matching_feedback = last_command_.action_id != 0 &&
      input.delivery_feedback.action_id == last_command_.action_id;
  if (input.command_transport_enabled) {
    action_feedback.action_done = matching_feedback &&
                                  input.delivery_feedback.done;
    action_feedback.action_active = last_command_.action_id != 0 &&
                                    !action_feedback.action_done;
  }
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
          decision = *aggregate;
          line_failure_pending_done_ = false;
        } else {
          decision.valid = false;
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

  switch (active_mission_) {
  case MissionType::kLine:
    compute_line(); ball_result_ = {}; hurdle_result_ = {}; goal_result_ = {};
    // FINAL HOLD는 명시적 Reset 전까지 완전한 terminal 상태다. 라인 action뿐
    // 아니라 새로운 object mission 진입도 허용하지 않는다.
    if (line_state_ == LineState::kFinalHold) break;
    if (has_ball_) {
      if (config_.enable_goal) {
        BallResult carrying; carrying.has_ball = true;
        goal_controller_.UpdateBallState(carrying);
        goal_result_ = goal_controller_.Compute(
            input.goal_target, input.backboard_target, input.goal_pose,
            input.image_width, input.image_height, input.now_sec,
            line_reference_valid, input.camera_feedback, action_feedback);
        if (goal_result_.mode != GoalMode::kLineFollow)
          EnterMission(MissionType::kGoal);
      }
    } else {
      if (config_.enable_hurdle) {
        hurdle_result_ = hurdle_controller_.Compute(
            input.hurdle_target, input.image_width, input.image_height,
            input.now_sec, input.camera_feedback, action_feedback);
        if (hurdle_result_.mode != HurdleMode::kLineFollow) {
          EnterMission(MissionType::kHurdle); break;
        }
      }
      if (config_.enable_ball) {
        ball_result_ = ball_controller_.Compute(
            input.ball_target, input.image_width, input.image_height,
            input.now_sec, input.camera_feedback, action_feedback);
        if (ball_result_.mode != BallMode::kLineFollow)
          EnterMission(MissionType::kBall);
      }
    }
    break;
  case MissionType::kBall: {
    const BallMode before = ball_result_.mode;
    if (before == BallMode::kPostPickupLineRecovery) compute_line();
    ball_result_ = ball_controller_.Compute(
        input.ball_target, input.image_width, input.image_height,
        input.now_sec, input.camera_feedback, action_feedback);
    has_ball_ = ball_result_.has_ball;
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
               !ball_result_.active) {
      BeginLineReacquisition(); FinishMission();
    }
    break;
  }
  case MissionType::kHurdle:
    hurdle_result_ = hurdle_controller_.Compute(
        input.hurdle_target, input.image_width, input.image_height,
        input.now_sec, input.camera_feedback, action_feedback);
    if (hurdle_result_.mode == HurdleMode::kLineFollow &&
        !hurdle_result_.active) {
      ball_controller_.ClearEntryEvidence();
      BeginLineReacquisition(); FinishMission();
    }
    break;
  case MissionType::kGoal: {
    const GoalMode before = goal_result_.mode;
    if (before == GoalMode::kHeadingRecovery) compute_line();
    goal_result_ = goal_controller_.Compute(
        input.goal_target, input.backboard_target, input.goal_pose,
        input.image_width, input.image_height, input.now_sec,
        line_reference_valid, input.camera_feedback, action_feedback);
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
  // one-shot tuning gate는 perception/FSM/feedback 처리를 막거나 기존
  // pending/queued action을 취소하지 않는다. coordinator 직전의 새 LINE
  // request만 제거하여 READY/DONE 조기 반환 경로까지 동일하게 막는다.
  if (active_mission_ == MissionType::kLine &&
      !input.allow_new_line_action)
    request = {};
  if (observation_hold) {
    request = {};
    camera_request = CameraRequest::kNone;
  }
  output.command = command_coordinator_.Compute(
      active_mission_, phase, request, camera_request,
      input.delivery_feedback, input.now_sec);
  if (input.advance_line_fsm && input.delivery_feedback.done &&
      input.delivery_feedback.action_id == ready_line_action_id_)
    ready_line_action_id_ = 0;
  last_command_ = output.command;
  output.active_mission = active_mission_;
  output.ball = ball_result_; output.hurdle = hurdle_result_;
  output.goal = goal_result_; output.has_ball = has_ball_;
  output.line_in_recovery = line_state_ != LineState::kNormal;
  return output;
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
    const bool fine = active_mission_ == MissionType::kGoal &&
        (goal_result_.mode == GoalMode::kFineAdjust ||
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
          center, *depth.center_depth_m, left, *depth.left_depth_m,
          right, *depth.right_depth_m, input.intrinsics,
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
  prepared.image_width = input.image_width; prepared.image_height = input.image_height;
  prepared.now_sec = input.now_sec; prepared.camera_feedback = input.camera_feedback;
  prepared.command_transport_enabled = input.command_transport_enabled;
  prepared.action_feedback = input.action_feedback;
  prepared.delivery_feedback = input.delivery_feedback;
  prepared.allow_new_line_action = input.allow_new_line_action;
  prepared.advance_line_fsm = input.advance_line_fsm;
  prepared.line_decision_guide_override = input.line_decision_guide_override;
  std::optional<double> center;
  if (std::isfinite(input.intrinsics.cx) && input.intrinsics.cx >= 0.0 &&
      input.intrinsics.cx < static_cast<double>(input.image_width))
    center = input.intrinsics.cx;
  output.mission = StepWithLineImageCenter(prepared, center);
  return output;
}

bool MissionController::UpdateLineP2pTuning(const LineP2pConfig &line_p2p) {
  if (last_command_.action_id != 0 || line_observation_active_) return false;
  MissionControllerConfig candidate = config_;
  candidate.line_p2p = line_p2p;
  ValidateAlgorithmConfig(candidate);
  config_ = candidate;
  line_p2p_controller_ = LineP2pController(line_p2p);
  return true;
}

void MissionController::Reset() {
  active_mission_ = MissionType::kLine;
  has_ball_ = config_.initial_has_ball;
  line_state_ = LineState::kNormal;
  line_direction_evidence_ = DirectionEvidence::kNone;
  line_observation_active_ = false;
  line_no_evidence_retries_ = line_recovery_turns_ = 0;
  recovery_turn_issued_ = false;
  line_stability_history_.clear(); line_guide_accumulator_.Reset();
  ready_line_action_id_ = 0;
  line_failure_pending_done_ = false;
  decision_observation_active_ = false;
  decision_observation_camera_only_ = false;
  decision_observation_first_frame_ = false;
  deferred_done_ = {};
  decision_object_hits_.clear();
  decision_observation_total_frames_ = 0;
  decision_line_valid_count_ = 0;
  decision_line_offset_sum_ = decision_line_heading_sum_ = 0.0;
  decision_line_confidence_sum_ = 0.0;
  previous_camera_settled_ = true;
  previous_camera_mode_ = CameraMode::kForward;
  object_tracking_camera_moving_ = false;
  object_tracking_camera_mode_ = CameraMode::kForward;
  ball_controller_.Reset(); hurdle_controller_.Reset(); goal_controller_.Reset();
  command_coordinator_.Reset(); ball_association_tracker_.Reset();
  backboard_association_tracker_.Reset(); hurdle_association_tracker_.Reset();
  ball_result_ = {}; hurdle_result_ = {}; goal_result_ = {}; last_command_ = {};
  if (has_ball_) goal_controller_.StartAfterPickup(0.0);
}
} // namespace vision_core
