#include "vision_core/mission_controller.hpp"

#include <algorithm>
#include <cmath>

namespace vision_core {
namespace {
ActionRequest WalkTurn(bool left) {
  return {left ? MissionAction::kTurnLeftInPlace : MissionAction::kStepForwardRightOneHurdle,
          ActionCategory::kLocomotion, 0, false};
}
bool SameIssuedAction(MissionAction requested, MissionAction issued) {
  const auto one_step = [](MissionAction action) {
    return action == MissionAction::kStepForwardOne ||
        action == MissionAction::kStepForwardOneAlternate;
  };
  return requested == issued || (one_step(requested) && one_step(issued));
}
}

void MissionController::ObserveLineFeedback(const CommandDeliveryFeedback& feedback, double now) {
  line_plan_.Feedback(feedback, now, config_.walk_inter_action_hold_sec);
  if (feedback.done && feedback.action_id != 0) {
    if (feedback.action_id == candidate_.turn_id) {
      if (candidate_.turn_is_restore) ++candidate_.restores; else ++candidate_.searches;
      candidate_.observe_after_turn = !candidate_.turn_is_restore;
      candidate_.turn_id = 0; // Duplicate DONE cannot consume another turn.
    }
    if (feedback.action_id == line_auxiliary_id_ && !line_auxiliary_done_) {
      line_auxiliary_done_ = true;
      if (line_auxiliary_action_ == MissionAction::kWalkMode ||
          line_auxiliary_action_ == MissionAction::kDefaultPoseMode) {
        foot_pose_ = line_auxiliary_action_ == MissionAction::kWalkMode ? FootPose::kWalk : FootPose::kFine;
        settle_until_sec_ = std::max(settle_until_sec_, now + config_.transition_settle_sec);
      } else if (line_auxiliary_action_ == MissionAction::kTurnLeft ||
                 line_auxiliary_action_ == MissionAction::kTurnRight) {
        settle_until_sec_ = std::max(settle_until_sec_, now + config_.fine_motion_settle_sec);
      }
    }
  }
}

void MissionController::BeginCandidateObservation(double now) {
  candidate_.phase = CandidatePhase::kObserve;
  candidate_.observe_after_turn = false;
  // This clears all moving observation/association histories, while the
  // candidate's entry-window direction and completed turns remain independent.
  BeginDecisionObservation(now, candidate_.mission, false, {});
}

void MissionController::ReleaseCandidate() {
  if (candidate_.mission == MissionType::kHurdle) hurdle_target_tracker_.Reset();
  candidate_ = {};
  ball_candidate_tracker_.Reset(); ball_association_tracker_.Reset();
  ball_target_id_=0; ball_target_locked_=ball_reacquiring_=ball_recovery_was_active_=false;
  decision_observation_active_ = false;
  decision_observation_restart_pending_ = false;
  ball_controller_.ClearEntryEvidence();
  hurdle_controller_.ClearObservationHistory();
  line_plan_.Reset();
  line_auxiliary_id_ = 0;
  line_auxiliary_done_ = line_auxiliary_observing_ = false;
  BeginLineReacquisition();
}

MissionFrameResult MissionController::StepPlannedLine(
    const MissionFrameInput& input, std::optional<double> image_center) {
  MissionFrameResult output;
  ObserveLineFeedback(input.delivery_feedback, input.now_sec);
  // Full AUTO has one approach policy. Manual approval and isolated stages
  // retain their stationary, one-command-at-a-time execution contract. The
  // candidate direction search below is unnecessary in ordinary AUTO.
  const bool automatic_approach = !input.manual_observation_control &&
                                  config_.test_mission == MissionType::kNone;
  if (automatic_approach && has_ball_ && config_.enable_goal) {
    BallResult carrying;carrying.has_ball=true;
    goal_controller_.UpdateBallState(carrying);
  }
  for (const auto& block : line_plan_.TakeCompletedBlocks()) {
    if (automatic_approach && block.owner && block.owner==continuous_.epoch) {
      if (block.probe) {
        continuous_.probe_active=false;
        continuous_.loss_pending=true;
        continue;
      } else if (block.object_qualified) {
        ++continuous_.blocks;
        continuous_.locked=continuous_.blocks>=config_.object_approach.lock_blocks;
      } else continuous_.loss_pending=true;
    }
    if (!post_pickup_goal_line_gate_active_) continue;
    ++post_pickup_line_actions_completed_;
    if (block.cruise == MissionAction::kStepForwardFour)
      ++post_pickup_straight_blocks_completed_;
    post_pickup_five_step_completed_ = StraightGoalGateReached();
  }
  FeatureConfig features = config_.line_features;
  if (image_center && std::isfinite(*image_center) && *image_center >= 0 &&
      *image_center < input.image_width) features.image_center_u = *image_center;
  output.line_features = ComputeLineFeatures(input.line_centers, input.image_width,
                                            input.image_height, features);
  output.line_computed = true;
  auto raw = output.line_features.guide;
  const auto actual_line_guide=raw;
  LineGuide decision = input.line_decision_guide_override.value_or(raw);
  output.line_decision_guide = decision;
  last_line_image_width_ = input.image_width;
  last_line_image_center_u_ = features.image_center_u >= 0
      ? features.image_center_u : 0.5 * input.image_width;
  const bool may_advance = input.allow_new_mission_entry;
  const bool commands_allowed = input.allow_new_commands && input.allow_new_body_actions &&
      !command_coordinator_.ActionAckTimedOut();
  const int visible_turns_before = line_visible_recovery_turns_;
  const int last_score_turns_before = line_last_score_recovery_turns_;
  const bool completing_current = input.delivery_feedback.done &&
      command_coordinator_.PendingActionId() == input.delivery_feedback.action_id &&
      !command_coordinator_.HasQueuedAction();
  const bool idle = (!command_coordinator_.HasPendingAction() || completing_current) &&
      !line_plan_.Outstanding();
  RouteDecision route;
  if (automatic_approach) {
    route=PrepareContinuousApproach(input,output.line_features,idle);
    if (route.entered) return StepWithLineImageCenter(input,image_center);
    if (route.guide) {
      raw=*route.guide;decision=raw;
      output.line_decision_guide=decision;
      if (route.object_visible && raw.valid && !continuous_.near &&
          (continuous_.mission == MissionType::kBall ||
           continuous_.mission == MissionType::kGoal)) {
        output.approach_path_guide = route.display_guide.value_or(raw);
        output.approach_path_reference_px = {
            last_line_image_center_u_,
            output.line_features.guide.valid
                ? output.line_features.guide_reference_v_px
                : config_.object_approach.reference_v_ratio * input.image_height};
      }
    }
  }
  if (input.advance_line_fsm) {
    actual_line_stability_history_.push_back(line_state_==LineState::kNormal &&
        actual_line_guide.valid && std::isfinite(actual_line_guide.offset) &&
        std::isfinite(actual_line_guide.heading_rad));
    while (actual_line_stability_history_.size() >
        static_cast<std::size_t>(config_.line.line_stable_window))
      actual_line_stability_history_.pop_front();
    line_stability_history_.push_back(line_state_ == LineState::kNormal && raw.valid &&
                                      std::isfinite(raw.offset) && std::isfinite(raw.heading_rad));
    while (line_stability_history_.size() > static_cast<std::size_t>(config_.line.line_stable_window))
      line_stability_history_.pop_front();
    line_plan_.Observe(input.now_sec, raw);
    if (automatic_approach && continuous_.mission!=MissionType::kNone) {
      const int minimum=continuous_.mission==MissionType::kBall ? config_.ball.entry_min_hits :
          continuous_.mission==MissionType::kGoal ? config_.goal.stable_min_hits : config_.hurdle.stable_min_hits;
      const int window=continuous_.mission==MissionType::kBall ? config_.ball.stable_window :
          continuous_.mission==MissionType::kGoal ? config_.goal.stable_window : config_.hurdle.stable_window;
      line_plan_.ObserveObject(input.now_sec,continuous_.epoch,route.object_visible,window,minimum);
    }
  }
  const bool line_valid = line_state_ == LineState::kNormal &&
      line_stability_history_.size()==static_cast<std::size_t>(config_.line.line_stable_window) &&
      std::count(line_stability_history_.begin(), line_stability_history_.end(), true) >=
          config_.line.line_stable_min_hits;
  // Remember only stable, actual floor LINE evidence. The router may have
  // replaced raw with a BALL/GOAL virtual path (or a HURDLE held path).
  const bool actual_line_valid=actual_line_stability_history_.size()==
      static_cast<std::size_t>(config_.line.line_stable_window) &&
      std::count(actual_line_stability_history_.begin(),
                 actual_line_stability_history_.end(),true)>=config_.line.line_stable_min_hits;
  if (automatic_approach && !continuous_.near && line_state_==LineState::kNormal &&
      actual_line_valid && actual_line_guide.valid &&
      std::isfinite(actual_line_guide.offset) &&
      std::isfinite(actual_line_guide.heading_rad)) {
    const auto real=line_p2p_controller_.Compute(actual_line_guide);
    if (real.direction!=CruiseDirection::kNone) last_line_score_=real.error;
  }
  ActionRequest request;
  CameraRequest camera = route.camera;
  bool manual_line_hold = false;
  if (input.manual_observation_control && input.begin_manual_observation &&
      candidate_.phase == CandidatePhase::kNone && !line_plan_.Active() &&
      line_state_ == LineState::kNormal)
    BeginDecisionObservation(input.now_sec, MissionType::kLine, false, {});

  // Observe entry evidence BEFORE considering READY. A qualified BALL candidate
  // wins the same sensory cycle even when A's READY is also delivered now.
  if (!automatic_approach && may_advance && !has_ball_ && line_state_ != LineState::kFinalHold &&
      input.camera_feedback.settled && input.camera_feedback.actual_mode == CameraMode::kForward &&
      (candidate_.phase == CandidatePhase::kNone || candidate_.phase == CandidatePhase::kDrain)) {
    if (config_.enable_hurdle && (candidate_.mission == MissionType::kNone ||
                                  candidate_.mission == MissionType::kHurdle)) {
      hurdle_result_ = hurdle_controller_.Compute(input.hurdle_target, input.image_width,
          input.image_height, input.now_sec, input.camera_feedback, {},
          line_p2p_controller_.Compute(raw), false, false);
      if (hurdle_result_.tracked.visible &&
          (input.hurdle_entry_track_hits ? *input.hurdle_entry_track_hits >= config_.hurdle.stable_min_hits :
                                           hurdle_controller_.EntryReady())) {
        candidate_.mission = MissionType::kHurdle;
        candidate_.direction_u = input.hurdle_entry_center_u.value_or(hurdle_result_.tracked.center_px.u);
      }
    }
    if (config_.enable_ball && (candidate_.mission == MissionType::kNone ||
                                candidate_.mission == MissionType::kBall)) {
      ball_result_ = ball_controller_.Compute(input.ball_target, input.image_width,
          input.image_height, input.now_sec, input.camera_feedback, {}, false, false,
          input.ball_entry_detected, input.ball_entry_center_u);
      if(input.ball_entry_track_hits) {
        ball_result_.entry_hits=*input.ball_entry_track_hits;
        ball_result_.entry_frames=input.ball_entry_track_frames.value_or(0);
      }
      if (ball_result_.entry_hits >= config_.ball.entry_min_hits) {
        candidate_.mission = MissionType::kBall;
        const bool new_hit = input.ball_entry_detected.value_or(
            input.ball_target && input.ball_target->center_px.v <= config_.ball.upper_acquire_v_px);
        const auto direction = input.ball_entry_track_hits ? input.ball_entry_center_u : ball_controller_.EntryDirection();
        if (new_hit && direction) {
          candidate_.direction_u = direction;
        }
      }
    }
    if (candidate_.phase == CandidatePhase::kNone && candidate_.mission != MissionType::kNone) {
      candidate_.phase = CandidatePhase::kDrain;
      line_plan_.Abort(); // Unissued repeats are discarded, issued IDs remain owned by coordinator.
      line_failure_pending_done_ = false;
      line_auxiliary_observing_ = false;
      decision_observation_active_ = false;
    }
  }

  if (candidate_.phase == CandidatePhase::kNone && input.manual_observation_control &&
      decision_observation_active_ && decision_observation_mission_ == MissionType::kLine &&
      !line_auxiliary_observing_) {
    AddDecisionObservation(input);
    manual_line_hold = !DecisionObservationReady(input.now_sec);
    output.post_motion_observing = manual_line_hold;
    if (!manual_line_hold) {
      decision = DecisionLineGuide().value_or(LineGuide{});
      decision_observation_active_ = false;
      output.line_decision_guide = decision;
    }
  }
  if (candidate_.phase != CandidatePhase::kNone) {
    if (may_advance) {
      if (candidate_.phase == CandidatePhase::kDrain && idle) BeginCandidateObservation(input.now_sec);
      if (candidate_.phase == CandidatePhase::kSearch && candidate_.turn_id == 0 && idle) {
        if (candidate_.observe_after_turn) BeginCandidateObservation(input.now_sec);
        else if (candidate_.search_left) request = WalkTurn(*candidate_.search_left);
      }
      if (candidate_.phase == CandidatePhase::kObserve) {
        AddDecisionObservation(input);
        output.post_motion_observing = true;
        output.post_motion_frames = decision_observation_total_frames_;
        output.post_motion_window_frames = decision_object_hits_.size();
        output.post_motion_valid = std::count(decision_object_hits_.begin(), decision_object_hits_.end(), true);
        output.post_motion_elapsed_sec = std::max(0.0, input.now_sec - decision_observation_start_sec_);
        if (DecisionObservationReady(input.now_sec) && input.allow_new_commands) {
          const bool verified = decision_object_qualified_ &&
              decision_last_target_ && decision_object_weight_ > 0;
          decision_observation_active_ = false;
          output.post_motion_observing = false;
          if (verified) {
            auto mean = *decision_last_target_;
            mean.center_px = {decision_target_u_sum_ / decision_object_weight_,
                              decision_target_v_sum_ / decision_object_weight_};
            if (mean.center_rectified)
              mean.rectified_center_px = {decision_rect_u_sum_ / decision_object_weight_,
                                         decision_rect_v_sum_ / decision_object_weight_};
            const auto mission = candidate_.mission;
            if (mission == MissionType::kBall)
              ball_controller_.EnterVerified(mean, decision_qualified_object_hits_, input.image_width,
                                              input.image_height, input.now_sec);
            else
              hurdle_controller_.EnterVerified(mean, decision_qualified_object_hits_, input.image_width,
                                                input.image_height, input.now_sec);
            candidate_ = {};
            line_plan_.Reset();
            line_auxiliary_id_ = line_ready_handled_id_ = 0;
            line_auxiliary_done_ = line_auxiliary_observing_ = false;
            EnterMission(mission);
            if (mission == MissionType::kBall) {
              ball_result_.mode = BallMode::kApproachBall; ball_result_.active = true;
            } else {
              hurdle_result_.mode = HurdleMode::kApproach; hurdle_result_.active = true;
            }
          } else {
            // Fix the saved candidate direction when search starts. It
            // survives DONE/observation waits and remains paired with the
            // inverse restoration, regardless of how long draining took.
            if (!candidate_.search_left && candidate_.direction_u) {
              const double center = candidate_.mission == MissionType::kBall ?
                  config_.ball.far_u_des_px : config_.hurdle.target_u_px;
              const double tolerance = candidate_.mission == MissionType::kBall ?
                  config_.ball.recovery_center_tolerance_px : config_.hurdle.recovery_center_tolerance_px;
              const double offset = *candidate_.direction_u - center;
              if (std::abs(offset) > tolerance) candidate_.search_left = offset < 0;
            }
            if (candidate_.search_left && candidate_.searches < config_.candidate_search_max_turns) {
              request = WalkTurn(*candidate_.search_left);
              candidate_.phase = CandidatePhase::kSearch;
            } else {
              candidate_.phase = CandidatePhase::kRestore;
            }
          }
        }
      }
      if (candidate_.phase == CandidatePhase::kRestore && idle && candidate_.turn_id == 0) {
        if (candidate_.restores < candidate_.searches && candidate_.search_left)
          request = WalkTurn(!*candidate_.search_left);
        else ReleaseCandidate();
      }
    }
  } else if (input.advance_line_fsm && !manual_line_hold && !route.hold) {
    // A block remains valid if any rolling 10-frame window qualifies before
    // its final READY; a single missed frame never cancels the next unit.
    if (line_failure_pending_done_ && idle) {
      line_failure_pending_done_ = false;
      BeginObservation(LineState::kFailureObserve, input.now_sec);
    }
    // GOAL admission is evaluated only after physically completed full blocks,
    // with the same carrying/armed/LINE validity gates as the old coordinator.
    if (!automatic_approach && has_ball_ && config_.enable_goal && may_advance) {
      BallResult carrying; carrying.has_ball = true;
      goal_controller_.UpdateBallState(carrying);
      ActionExecutionFeedback execution;
      execution.action_active = !idle;
      goal_result_ = goal_controller_.Compute(input.goal_target, input.backboard_target,
          input.goal_pose, input.image_width, input.image_height, input.now_sec,
          line_valid, input.camera_feedback, execution, commands_allowed,
          input.backboard_center_depth_m,
          idle && post_pickup_goal_line_gate_active_ && line_valid && line_state_ == LineState::kNormal &&
              (post_pickup_five_step_completed_ || post_pickup_line_actions_completed_ >=
                   goal_controller_.RequiredLineWalkActions()), input.backboard_image_without_depth,
          idle && input.allow_new_commands);
      if (goal_result_.mode != GoalMode::kLineFollow && idle && input.allow_new_commands) {
        line_plan_.Abort();
        EnterMission(MissionType::kGoal);
        request = goal_result_.action;
        camera = goal_result_.camera_request;
      }
    }
    if (active_mission_ == MissionType::kLine) {
      const auto id = input.delivery_feedback.action_id;
      const bool ready = input.delivery_feedback.ready && !input.delivery_feedback.done &&
          command_coordinator_.PendingActionId() == id && !line_plan_.Done(id) &&
          !command_coordinator_.HasQueuedAction();
      const bool final_edge = line_plan_.LastUnit(id) &&
          (input.delivery_feedback.ready || input.delivery_feedback.done);
      const int completing_straight = final_edge && !line_plan_.Done(id) &&
          line_plan_.CruiseFor(id) == MissionAction::kStepForwardFour ? 1 : 0;
      // The previous block may not have reached sensory DONE yet; count the
      // qualifying block prospectively ONLY to suppress its final READY queue.
      const bool goal_will_enter = has_ball_ && config_.enable_goal &&
          (!automatic_approach || (input.camera_feedback.actual_mode==CameraMode::kForward &&
            post_pickup_line_actions_completed_+(line_plan_.Done(id)?0:1)>=approach_goal_retry_after_)) &&
          goal_controller_.CanEnterFromLine() &&
          post_pickup_goal_line_gate_active_ && line_valid && final_edge &&
          (post_pickup_five_step_completed_ ||
           post_pickup_line_actions_completed_ + (line_plan_.Done(id) ? 0 : 1) >=
               goal_controller_.RequiredLineWalkActions());
      // Only the configured number of completed straight blocks satisfies the shortcut.
      const bool straight_goal = has_ball_ && config_.enable_goal &&
          (!automatic_approach || (input.camera_feedback.actual_mode==CameraMode::kForward &&
            post_pickup_line_actions_completed_+(line_plan_.Done(id)?0:1)>=approach_goal_retry_after_)) &&
          goal_controller_.CanEnterFromLine() &&
          post_pickup_goal_line_gate_active_ && line_valid && final_edge &&
          StraightGoalGateReached(completing_straight);
      if (final_edge) {
        const auto mean = line_plan_.Finish(id, input.now_sec,
            input.delivery_feedback.ready && !input.delivery_feedback.done ? config_.line_p2p.ready_lead_sec : 0.0);
        last_ready_line_guide_ = mean;
        output.line_window_stats = line_plan_.Stats(id);
        decision = mean.value_or(LineGuide{});
        output.line_decision_guide = decision;
        if (!decision.valid && !idle) line_failure_pending_done_ = true;
        if (automatic_approach && continuous_.mission!=MissionType::kNone &&
            line_plan_.ObjectOwned(id,continuous_.epoch) &&
            !line_plan_.IsProbe(id) &&
            !line_plan_.ObjectQualified(id)) {
          continuous_.loss_pending=true;
          line_plan_.Abort();
          route.hold=true;
        }
      }
      if (line_auxiliary_id_ && line_auxiliary_done_ && idle) {
        const bool pose = line_auxiliary_action_ == MissionAction::kWalkMode ||
                          line_auxiliary_action_ == MissionAction::kDefaultPoseMode;
        if ((line_state_ == LineState::kScoreRecovery || pose) && !line_auxiliary_observing_) {
          if (!pose) ++line_guard_turns_completed_;
          BeginDecisionObservation(input.now_sec, MissionType::kLine, false, {});
          line_auxiliary_observing_ = true;
        }
      }
      bool observation_wait = false;
      if (line_auxiliary_observing_) {
        if (automatic_approach && route.guide) {
          auto observation=input;
          observation.line_decision_guide_override=raw;
          AddDecisionObservation(observation);
        } else AddDecisionObservation(input);
        observation_wait = !DecisionObservationReady(input.now_sec);
        output.post_motion_observing = observation_wait;
        if (!observation_wait) {
          decision = DecisionLineGuide().value_or(LineGuide{});
          decision_observation_active_ = false;
          line_auxiliary_observing_ = false;
          line_auxiliary_id_ = 0;
        }
      }
      if (!observation_wait && !line_failure_pending_done_ && line_state_ != LineState::kFinalHold) {
        if (automatic_approach && ready && !line_plan_.Active() && !route.hold &&
            decision.valid && !goal_will_enter && !straight_goal && line_ready_handled_id_!=id) {
          bool reference=false;
          request=ComputeLineAction(decision,input,false,false,&reference,false);
        } else if (ready && !final_edge && line_ready_handled_id_ != id) {
          request = line_plan_.Continue(id);
        } else if ((ready && final_edge && line_ready_handled_id_ != id &&
                    !goal_will_enter && !straight_goal) || idle) {
          request = line_plan_.Continue();
          if (request.action == MissionAction::kNone && !line_plan_.Active() &&
              !goal_will_enter && !straight_goal && input.allow_new_line_plan) {
            ActionExecutionFeedback execution;
            execution.action_done = line_auxiliary_done_ &&
                line_auxiliary_action_ != MissionAction::kWalkMode &&
                line_auxiliary_action_ != MissionAction::kDefaultPoseMode;
            execution.action_active = !idle && !ready;
            bool reference = false;
            // Final READY deliberately evaluates the next block while its
            // current last unit is still running, exactly like legacy READY.
            request = ComputeLineAction(decision, input, false,
                                         execution.action_done, &reference, false);
            line_auxiliary_done_ = false;
            if (line_state_ != LineState::kScoreRecovery) line_auxiliary_id_ = 0;
          } else if (ready && final_edge && request.action == MissionAction::kNone &&
                     !goal_will_enter && !straight_goal && input.allow_new_line_plan) {
            // A new block is chosen once at the LAST READY, not at every step.
            bool reference = false;
            request = ComputeLineAction(decision, input, false, false, &reference, false);
          }
        }
      }
      if (request.action == MissionAction::kNone && idle && final_edge && !decision.valid &&
          !goal_will_enter && !straight_goal && line_state_ == LineState::kNormal)
        BeginObservation(LineState::kFailureObserve, input.now_sec);
    }
  }

  if (route.action.action!=MissionAction::kNone) request=route.action;
  if (!commands_allowed || manual_line_hold || (route.hold && route.action.action==MissionAction::kNone) || input.now_sec + 1e-9 < settle_until_sec_ || (active_mission_ == MissionType::kLine &&
      (!input.advance_line_fsm || (!input.allow_new_line_action && !route.near_final_action)))) request = {};
  const bool wants_fine = request.action == MissionAction::kTurnLeft || request.action == MissionAction::kTurnRight;
  const bool wants_walk = LinePlan::IsCruise(request.action) || LinePlan::IsUnit(request.action) ||
      request.action==MissionAction::kStepForwardOne ||
      request.action==MissionAction::kStepForwardOneAlternate ||
      request.action==MissionAction::kStepForwardTwo ||
      request.action == MissionAction::kTurnLeftInPlace || request.action == MissionAction::kStepForwardRightOneHurdle;
  bool pose_transition = false;
  if ((wants_fine && foot_pose_ != FootPose::kFine) || (wants_walk && foot_pose_ != FootPose::kWalk)) {
    if (wants_fine) {
      recovery_turn_issued_ = false;
      line_visible_recovery_turns_ = visible_turns_before;
      line_last_score_recovery_turns_ = last_score_turns_before;
    }
    request = {wants_walk ? MissionAction::kWalkMode : MissionAction::kDefaultPoseMode,
               ActionCategory::kLocomotion, 0, false};
    pose_transition = true;
  }
  if (LinePlan::IsCruise(request.action)) {
    if (input.allow_new_line_plan) request = line_plan_.Begin(request.action,
        automatic_approach ? continuous_.epoch : 0,route.probe_start);
    else request = {};
  }
  if (!input.allow_new_commands) camera = CameraRequest::kNone;
  const auto parent_id = command_coordinator_.PendingActionId();
  output.command = command_coordinator_.Compute(active_mission_,
      candidate_.phase == CandidatePhase::kNone ? static_cast<int>(line_state_) :
          1000 + static_cast<int>(candidate_.phase), request, camera,
      input.delivery_feedback, input.now_sec);
  const auto& command = output.command;
  if (LinePlan::IsUnit(command.action)) line_plan_.Issued(command,
      command.control_phase == ControlPhase::kWaitingQueuedActionAck ? parent_id : 0);
  if (route.probe_start) {
    if (command.command_type==CommandType::kAction && command.action_id &&
        LinePlan::IsUnit(command.action)) {
      continuous_.probe_active=true;
      continuous_.probe_used=true;
      continuous_.loss_pending=false;
    } else line_plan_.Abort();
  }
  if (route.near_final_action && command.command_type==CommandType::kAction &&
      command.action_id && SameIssuedAction(route.action.action, command.action))
    continuous_.near_motion_id=command.action_id;
  if (command.command_type == CommandType::kAction && command.action_id &&
      request.action != MissionAction::kNone && SameIssuedAction(request.action, command.action) &&
      !LinePlan::IsUnit(command.action)) {
    if (!pose_transition && (candidate_.phase == CandidatePhase::kSearch || candidate_.phase == CandidatePhase::kRestore)) {
      candidate_.turn_id = command.action_id;
      candidate_.turn_is_restore = candidate_.phase == CandidatePhase::kRestore;
    } else if (route.depth_recovery_step) {
      approach_goal_depth_action_id_=command.action_id;
    } else if (line_auxiliary_id_ != command.action_id) {
      line_auxiliary_id_ = command.action_id;
      line_auxiliary_action_ = command.action;
      line_auxiliary_done_ = false;
    }
  }
  if (input.delivery_feedback.ready && command.control_phase == ControlPhase::kWaitingQueuedActionAck)
    line_ready_handled_id_ = input.delivery_feedback.action_id;
  last_command_ = command;
  output.active_mission = active_mission_;
  output.has_ball = has_ball_;
  output.ball = ball_result_; output.hurdle = hurdle_result_; output.goal = goal_result_;
  output.line_in_recovery = line_state_ != LineState::kNormal;
  output.candidate_mission = candidate_.mission;
  output.candidate_phase = static_cast<int>(candidate_.phase);
  output.candidate_search_turns = candidate_.searches;
  output.candidate_restore_turns = candidate_.restores;
  output.line_plan_active = line_plan_.Active();
  PopulateApproachDiagnostics(output);
  PopulateDiagnostics(output, input.now_sec);
  return output;
}
} // namespace vision_core
