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

void MissionController::RetryLineHeadSearch() {
  auto& scan=line_head_search_;using Phase=LineHeadSearch::Phase;
  if(++scan.completed_pairs<std::min(config_.line_recovery.scan_max_pairs,config_.scan_correction.max_attempts)) {
    scan.first_left=scan.last_found_left.value_or(scan.first_left);
    scan.phase=scan.first_left ? Phase::kPendingLeft : Phase::kPendingRight;
    scan.observation.Reset();
  } else { scan.phase=Phase::kIdle;line_state_=LineState::kFinalHold; }
}
void MissionController::RetryGoalHeadSearch() {
  auto& scan=goal_head_search_;using Phase=GoalHeadSearch::Phase;
  scan.observation.Reset();scan.observed_target.reset();scan.started=false;
  if(scan.attempts<config_.scan_correction.max_attempts) {
    scan.first_left=scan.last_found_left.value_or(scan.first_left);
    scan.phase=scan.first_left ? Phase::kPendingLeft : Phase::kPendingRight;
  } else { scan.phase=Phase::kIdle;scan.exhausted=true;approach_goal_empty_frames_=0; }
}
void MissionController::RetryCurveHeadSearch() {
  using Phase=CurveSearchState::Phase;
  curve_.observation.Reset();curve_.started=false;
  if(curve_.head_attempts<std::min(config_.curve_search.head_search_max_attempts,config_.scan_correction.max_attempts)) {
    // Reacquire fresh FORWARD LINE evidence, then perform exactly one normal
    // LINE unit before retrying the head scan. Recovery turns do not count.
    curve_.phase=Phase::kProbeWalk;curve_.probe_action_id=0;
    line_plan_.Abort();
    last_ready_line_guide_.reset();
    line_auxiliary_id_=0;line_auxiliary_done_=line_auxiliary_observing_=false;
    decision_observation_active_=decision_observation_restart_pending_=false;
    BeginLineReacquisition();
  } else {
    curve_.phase=Phase::kLocked;curve_.unlock_blocks=config_.curve_search.failure_lock_line_blocks;
    curve_.skip_pending=true;
    if(course_curve_index_==3) { course_curve_index_=5;curve_.unlock_blocks=0; }
    BeginLineReacquisition();
  }
}
void MissionController::AdvanceScanDeadlines(double now) {
  // A transport tick supplies time only: no RGB vote, body request or fake DONE.
  const auto failed=[](StationaryScanObservation::Result value) {
    return value==StationaryScanObservation::Result::kEmpty ||
        value==StationaryScanObservation::Result::kRejected;
  };
  if(line_head_search_.phase==LineHeadSearch::Phase::kReobserve &&
     failed(line_head_search_.observation.Observe(1,false,now,false,
         config_.line_recovery.scan_hold_sec,config_.line_recovery.scan_observation_timeout_sec,
         config_.line.line_stable_window,config_.line.line_stable_min_hits))) RetryLineHeadSearch();
  if(goal_head_search_.phase==GoalHeadSearch::Phase::kReobserve &&
     failed(goal_head_search_.observation.Observe(1,false,now,false,
         config_.goal.head_scan_hold_sec,config_.goal.head_scan_observation_timeout_sec,
         config_.goal.stable_window,config_.goal.stable_min_hits))) RetryGoalHeadSearch();
  if(curve_.phase==CurveSearchState::Phase::kBodyObserve &&
     failed(curve_.observation.Observe(1,false,now,false,
         config_.curve_search.scan_hold_sec,config_.curve_search.scan_observation_timeout_sec,
         config_.ball.stable_window,config_.ball.stable_min_hits))) RetryCurveHeadSearch();
}

void MissionController::ObserveLineFeedback(const CommandDeliveryFeedback& feedback, double now) {
  if(feedback.done && feedback.action_id!=0 && continuous_.mission==MissionType::kGoal &&
     line_plan_.ObjectOwned(feedback.action_id,continuous_.epoch) &&
     !line_plan_.IsProbe(feedback.action_id))
    goal_approach_walk_done_=true;
  line_plan_.Feedback(feedback, now, config_.walk_inter_action_hold_sec);
  if (feedback.done && feedback.action_id != 0) {
    if(curve_.phase==CurveSearchState::Phase::kProbeWalk &&
       feedback.action_id==curve_.probe_action_id) {
      curve_.probe_action_id=0;
      curve_.phase=CurveSearchState::Phase::kDrain;
    }
    if(line_head_search_.phase==LineHeadSearch::Phase::kAlignBody &&
       feedback.action_id==line_head_search_.align_action_id) {
      auto& scan=line_head_search_;scan.align_action_id=0;
      if(scan.align_remaining>0) --scan.align_remaining;
      if(scan.align_remaining==0) {
        scan.phase=LineHeadSearch::Phase::kReobserve;scan.observation.Begin(1,now);scan.observed_line.reset();
      }
    }
    if(goal_head_search_.phase==GoalHeadSearch::Phase::kBodyTurn &&
       feedback.action_id==goal_head_search_.body_action_id) {
      auto& scan=goal_head_search_;scan.body_action_id=0;
      if(scan.turns_remaining>0) --scan.turns_remaining;
      if(scan.turns_remaining==0) {
        scan.phase=GoalHeadSearch::Phase::kReobserve;scan.observation.Begin(1,now);
        scan.observed_target.reset();scan.reobserve_started_sec=now;
      }
    }
    if(curve_.phase==CurveSearchState::Phase::kBodyTurn &&
       feedback.action_id==curve_.correction_action_id) {
      curve_.correction_action_id=0;if(curve_.turns_remaining>0) --curve_.turns_remaining;
      if(curve_.turns_remaining==0) {
        curve_.phase=CurveSearchState::Phase::kBodyObserve;curve_.observation.Begin(1,now);curve_.observed_frames=0;
      }
    }
    if (feedback.action_id == candidate_.turn_id) {
      if (candidate_.turn_is_restore) ++candidate_.restores; else ++candidate_.searches;
      candidate_.observe_after_turn = !candidate_.turn_is_restore;
      candidate_.turn_id = 0; // Duplicate DONE cannot consume another turn.
    }
    if (feedback.action_id == line_auxiliary_id_ && !line_auxiliary_done_) {
      line_auxiliary_done_ = true;
      if (finish_line_started_ && (line_auxiliary_action_==MissionAction::kTurnLeftInPlace ||
          line_auxiliary_action_==MissionAction::kStepForwardRightOneHurdle))
        ++line_guard_turns_completed_; // Count actual DONE even if floor evidence vanished.
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
                                  UsesAutomaticMissionPolicy();
  const bool finish_course=FinishCourseEnabled(input);
  finish_loss_request_ = false;
  if (finish_course && !finish_line_started_ && input.advance_line_fsm &&
      input.camera_feedback.settled && input.camera_feedback.actual_mode==CameraMode::kForward) {
    finish_line_started_=true;
    if (line_state_!=LineState::kFinalHold) line_state_=LineState::kNormal;
    line_failure_pending_done_=false;
    line_observation_active_=false;
  }
  if (automatic_approach && !finish_course && has_ball_ && config_.enable_goal) {
    BallResult carrying;carrying.has_ball=true;
    goal_controller_.UpdateBallState(carrying);
  }
  const auto completed_blocks=line_plan_.TakeCompletedBlocks();
  for (const auto& block : completed_blocks) {
    if (automatic_approach && block.owner==0 && !block.ball_entry_epoch && !block.probe &&
        config_.curve_search.enabled && config_.enable_ball) {
      int left_hits=0,right_hits=0;
      for(const int d:curve_.frame_directions) {
        if(d<0) ++left_hits;
        if(d>0) ++right_hits;
      }
      const int detected=curve_.block_curve_direction;
      curve_.frame_directions.clear();
      curve_.block_curve_direction=0;
      if(curve_.phase==CurveSearchState::Phase::kLocked &&
         curve_.unlock_blocks>0) {
        if(--curve_.unlock_blocks==0)
          curve_.phase=CurveSearchState::Phase::kIdle;
      } else if(curve_.phase==CurveSearchState::Phase::kDelay) {
        if(--curve_.delay_remaining<=0) {
          curve_.phase=CurveSearchState::Phase::kDrain;
          line_plan_.Abort(); // Never cancel already-issued motors.
        }
      } else if(curve_.phase==CurveSearchState::Phase::kBodyForward) {
        curve_.phase=CurveSearchState::Phase::kBodyObserve;
        curve_.observed_frames=0;
      } else if((curve_.phase==CurveSearchState::Phase::kIdle ||
                 curve_.phase==CurveSearchState::Phase::kConfirm) &&
                !has_ball_ && continuous_.mission==MissionType::kNone &&
                course_curve_index_<=3) {
        if(detected!=0) {
          if(curve_.phase!=CurveSearchState::Phase::kConfirm ||
             detected!=curve_.direction) {
            curve_.direction=detected;curve_.confirmations=0;
            curve_.phase=CurveSearchState::Phase::kConfirm;
          } else ++curve_.confirmations;
          if(curve_.confirmations>=config_.curve_search.confirm_extra_sequences) {
            bool eligible=true;
            if(curve_.skip_pending) {
              if(course_curve_index_==1 && detected==1) {
                course_curve_index_=2;eligible=false;
              } else if((course_curve_index_==1 || course_curve_index_==2) &&
                        detected==-1) {
                course_curve_index_=3;curve_.skip_pending=false;
              } else eligible=false;
            } else if(course_curve_index_!=1 && course_curve_index_!=3) {
              eligible=false;
            }
            if(eligible) {
              curve_.head_attempts=curve_.body_attempts=0;
              curve_.turns_remaining=0;curve_.correction_action_id=0;curve_.found_yaw.reset();
              curve_.delay_remaining=config_.curve_search.scan_delay_walk_sequences;
              curve_.phase=curve_.delay_remaining>0 ?
                  CurveSearchState::Phase::kDelay : CurveSearchState::Phase::kDrain;
              if(curve_.phase==CurveSearchState::Phase::kDrain) line_plan_.Abort();
            } else {curve_.phase=CurveSearchState::Phase::kIdle;curve_.confirmations=0;}
          }
        } else {curve_.phase=CurveSearchState::Phase::kIdle;curve_.confirmations=0;}
      }
    }
    if (automatic_approach && continuous_.epoch &&
        (block.owner==continuous_.epoch ||
         block.ball_entry_epoch==continuous_.epoch)) {
      if (block.probe) {
        if (continuous_.mission==MissionType::kBall &&
            continuous_.probe_return_to_line) {
          FinishPrelockProbeToLine(); // Keep the already-ACKed LINE successor.
        } else if (continuous_.probe_active) {
          // The probe finished with no rediscovery and no LINE successor.
          continuous_.probe_active=false;
          continuous_.loss_pending=true;
        }
        // A rediscovered BALL turned the probe into its final issued unit.
        // It is NOT a lock-qualified normal approach block.
        continue;
      } else if (block.object_qualified ||
                 block.ball_entry_epoch==continuous_.epoch) {
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
  features.image_center_u = EffectiveLineReferenceU(input, features.image_center_u);
  output.line_reference_u_px = features.image_center_u;
  output.line_features = ComputeLineFeaturesForFrame(input, features);
  output.line_computed = true;
  // The normal LINE guide is untouched; this only records independent curve
  // votes during an executing logical LINE block.
  const auto curve_evidence=ComputeCurveEvidence(input.line_centers,input.image_width,
      input.image_height,features,config_.curve_search);
  const bool curve_frame_eligible=automatic_approach && !finish_course && !has_ball_ &&
      continuous_.mission==MissionType::kNone &&
      input.camera_feedback.settled &&
      input.camera_feedback.actual_mode==CameraMode::kForward &&
      line_plan_.Active() && input.advance_line_fsm &&
      (curve_.phase==CurveSearchState::Phase::kIdle ||
       curve_.phase==CurveSearchState::Phase::kConfirm);
  if(curve_frame_eligible) {
    curve_.frame_directions.push_back(curve_evidence.direction);
    while(curve_.frame_directions.size()>
          static_cast<std::size_t>(config_.curve_search.stable_window))
      curve_.frame_directions.pop_front();
    if(curve_.block_curve_direction==0 &&
       curve_.frame_directions.size()==
          static_cast<std::size_t>(config_.curve_search.stable_window)) {
      const int right=std::count(curve_.frame_directions.begin(),
                                  curve_.frame_directions.end(),1);
      const int left=std::count(curve_.frame_directions.begin(),
                                 curve_.frame_directions.end(),-1);
      if(right>=config_.curve_search.stable_min_hits)
        curve_.block_curve_direction=1;
      else if(left>=config_.curve_search.stable_min_hits)
        curve_.block_curve_direction=-1;
    }
  }
  auto raw = output.line_features.guide;
  const auto actual_line_guide=raw;
  const bool finish_floor_usable=actual_line_guide.valid && output.line_features.n_visible>=3;
  finish_sparse_line_ = finish_course && output.line_features.n_visible < 3;
  if (finish_course && input.advance_line_fsm && input.camera_feedback.settled &&
      input.camera_feedback.actual_mode == CameraMode::kForward) {
    finish_line_hits_.push_back(finish_floor_usable);
    while (finish_line_hits_.size() > static_cast<std::size_t>(config_.line.line_stable_window))
      finish_line_hits_.pop_front();
    const bool stable = finish_line_hits_.size() == static_cast<std::size_t>(config_.line.line_stable_window) &&
        std::count(finish_line_hits_.begin(), finish_line_hits_.end(), true) >= config_.line.line_stable_min_hits;
    if (stable && finish_floor_usable) {
      finish_last_stable_guide_ = actual_line_guide;
      if (!finish_loss_action_id_ || line_plan_.Done(finish_loss_action_id_))
        finish_loss_used_ = false;
    }
  }
  if (finish_course && !finish_floor_usable) {
    // Finish steering needs >=3 valid floor LINE points. Objects, including a
    // finish gate detected as HURDLE, must never substitute for missing floor evidence.
    // ComputeLineAction keeps the existing straight fallback for this case.
    raw={};
  }
  // Sparse/invalid finish evidence bypasses visual recovery, without cancelling issued IDs.
  if (finish_course && !finish_floor_usable) {
    if (line_state_==LineState::kScoreRecovery) line_state_=LineState::kNormal;
    line_auxiliary_observing_=false;
    decision_observation_active_=false;
  }
  LineGuide decision = input.line_decision_guide_override.value_or(raw);
  if (finish_course) decision=raw;
  else if (mission_recovery_line_guide_ && !command_coordinator_.HasPendingAction())
    decision = *mission_recovery_line_guide_;
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
  // Scan-side hints are actual settled-view targets, never a virtual LINE path.
  if(input.camera_feedback.settled && input.camera_scan.phase==0) {
    if(input.ball_target && input.camera_feedback.actual_mode==CameraMode::kForward &&
       std::isfinite(input.ball_target->center_px.u)) last_ball_scan_u_=input.ball_target->center_px.u;
    if(input.backboard_target && input.camera_feedback.actual_mode==CameraMode::kGoal &&
       std::isfinite(input.backboard_target->center_px.u)) last_goal_scan_u_=input.backboard_target->center_px.u;
  }
  RouteDecision route;
  bool curve_special=curve_.phase==CurveSearchState::Phase::kDrain ||
      curve_.phase==CurveSearchState::Phase::kScan ||
      curve_.phase==CurveSearchState::Phase::kHold ||
      curve_.phase==CurveSearchState::Phase::kReturnFound ||
      curve_.phase==CurveSearchState::Phase::kReturnMiss ||
      // Before issue, use normal LINE observation/recovery and BALL admission.
      // After issue, hold all successors until this exact unit's real DONE.
      (curve_.phase==CurveSearchState::Phase::kProbeWalk &&
       (curve_.probe_action_id!=0 || !input.camera_feedback.settled ||
        input.camera_feedback.actual_mode!=CameraMode::kForward)) ||
      curve_.phase==CurveSearchState::Phase::kBodyTurn ||
      curve_.phase==CurveSearchState::Phase::kBodyForward ||
      curve_.phase==CurveSearchState::Phase::kBodyObserve;
  // A qualified BALL must be allowed to enter even when a previously
  // scheduled curve has reached DRAIN but the head scan has not started.
  const bool ball_ready_during_drain=curve_.phase==CurveSearchState::Phase::kDrain &&
      input.camera_feedback.settled &&
      input.camera_feedback.actual_mode==CameraMode::kForward &&
      input.ball_target && input.ball_entry_track_hits &&
      *input.ball_entry_track_hits>=config_.ball.entry_min_hits &&
      input.ball_target->center_px.v<=config_.ball.upper_acquire_v_px;
  if (automatic_approach && !finish_course && (!curve_special ||
      ball_ready_during_drain)) {
    route=PrepareContinuousApproach(input,output.line_features,idle);
    if (continuous_.mission==MissionType::kBall &&
        (curve_.phase==CurveSearchState::Phase::kConfirm ||
         curve_.phase==CurveSearchState::Phase::kDelay ||
         curve_.phase==CurveSearchState::Phase::kDrain ||
         curve_.phase==CurveSearchState::Phase::kProbeWalk)) {
      // BALL admission wins over an UNSTARTED camera scan; never cancel an
      // already running scan or an already issued motor action.
      curve_.phase=CurveSearchState::Phase::kIdle;
      curve_.delay_remaining=curve_.confirmations=0;
      curve_.frame_directions.clear();curve_.block_curve_direction=0;
      curve_special=false;
    }
    if (route.entered) {
      // Far-object loss must not lock out the next legitimate curve.
      if(continuous_.mission==MissionType::kBall && continuous_.near) {
        curve_.phase=CurveSearchState::Phase::kLocked;
        curve_.unlock_blocks=0;
      } else curve_.phase=CurveSearchState::Phase::kIdle;
      return StepWithLineImageCenter(input,image_center);
    }
    // Reaching BALL near is the actual lock condition, independently of
    // whether the object was acquired before or during a curve scan.
    if (continuous_.mission==MissionType::kBall && continuous_.near) {
      curve_.phase=CurveSearchState::Phase::kLocked;
      curve_.unlock_blocks=0;
    }
    if (continuous_.mission==MissionType::kHurdle && continuous_.near &&
        curve_.phase==CurveSearchState::Phase::kLocked &&
        curve_.unlock_blocks==config_.curve_search.failure_lock_line_blocks)
      curve_.phase=CurveSearchState::Phase::kIdle;
    if (route.guide) {
      raw=*route.guide;decision=raw;
      output.line_decision_guide=decision;
    }
  }
  // Read only the LAST extra pre-DOWN LINE unit's own moving O/H through
  // READY. A queued ACK can precede the previous unit's DONE: start only
  // once the extra unit becomes the physically pending action.
  if (automatic_approach && input.advance_line_fsm &&
      continuous_.mission==MissionType::kHurdle && continuous_.near &&
      continuous_.near_motion_id!=0 && !continuous_.hurdle_down_ready_finalized) {
    const auto id=continuous_.near_motion_id;
    const auto& fb=input.delivery_feedback;
    if (fb.action_id==id && fb.acknowledged && !fb.done)
      continuous_.hurdle_down_final_acknowledged=true;
    if (continuous_.hurdle_down_final_acknowledged &&
        command_coordinator_.PendingActionId()==id) {
      if (!hurdle_down_ready_accumulator_.ActiveFor(id))
        hurdle_down_ready_accumulator_.Begin(id,input.now_sec);
      hurdle_down_ready_accumulator_.Add(id,input.now_sec,actual_line_guide);
      if (fb.action_id==id && fb.ready && !fb.done) {
        const auto mean=hurdle_down_ready_accumulator_.Finish(
            id,input.now_sec,config_.line_p2p.ready_lead_sec);
        if (mean && mean->valid) {
          const auto dir=SteeringDecision(*mean).direction;
          if (dir==CruiseDirection::kLeft)
            continuous_.hurdle_down_ready_action=MissionAction::kLineLeftUnit;
          else if (dir==CruiseDirection::kRight)
            continuous_.hurdle_down_ready_action=MissionAction::kLineRightUnitTurnFirst; // 36
          else if (dir==CruiseDirection::kStraight)
            continuous_.hurdle_down_ready_action=MissionAction::kLineStraightUnit;
        }
        // An invalid READY window falls back straight, not to the OLD block.
        continuous_.hurdle_down_ready_finalized=true;
      }
    }
  }
  if (input.advance_line_fsm) {
    actual_line_stability_history_.push_back(line_state_==LineState::kNormal &&
        !curve_special && input.camera_feedback.settled &&
        (input.camera_feedback.actual_mode==CameraMode::kForward ||
         input.camera_feedback.actual_mode==CameraMode::kBallView) &&
        actual_line_guide.valid && std::isfinite(actual_line_guide.offset) &&
        std::isfinite(actual_line_guide.heading_rad));
    while (actual_line_stability_history_.size() >
        static_cast<std::size_t>(config_.line.line_stable_window))
      actual_line_stability_history_.pop_front();
    line_stability_history_.push_back(line_state_ == LineState::kNormal &&
                                      (!automatic_approach || continuous_.mission==MissionType::kNone ||
                                       route.guide_observed) && raw.valid &&
                                      std::isfinite(raw.offset) && std::isfinite(raw.heading_rad));
    while (line_stability_history_.size() > static_cast<std::size_t>(config_.line.line_stable_window))
      line_stability_history_.pop_front();
    LineGuide object_observation;
    if (continuous_.mission==MissionType::kHurdle)
      object_observation=actual_line_guide;
    else if (route.guide_observed && route.guide)
      object_observation=*route.guide;
    const bool pixel_owner=automatic_approach &&
        (continuous_.mission==MissionType::kBall || continuous_.mission==MissionType::kGoal);
    const std::optional<Point2> object_center =
        pixel_owner && route.object_visible && continuous_.last
        ? std::optional<Point2>(continuous_.last->center_px) : std::nullopt;
    // The first BALL admission may occur inside an already-issued LINE
    // block. Keep that block's LINE ownership and samples, but preserve its
    // independently qualified BALL entry mean through its final READY.
    if (automatic_approach && continuous_.mission==MissionType::kBall &&
        !continuous_.near && continuous_.blocks==0) {
      const auto pending=command_coordinator_.PendingActionId();
      if (pending && !line_plan_.ObjectOwned(pending,continuous_.epoch))
        line_plan_.AttachBallEntry(pending,continuous_.epoch,
                                   input.ball_entry_mean_center,input.now_sec,config_.ball.ready_observation_end_weight);
    }
    line_plan_.Observe(input.now_sec,automatic_approach ? actual_line_guide : raw,
                       object_observation,continuous_.epoch,object_center,pixel_owner);
    if (automatic_approach && continuous_.mission==MissionType::kBall)
      line_plan_.ObserveBallEntry(input.now_sec,continuous_.epoch,
          route.object_visible && input.ball_target
              ? std::optional<Point2>(input.ball_target->center_px)
              : std::nullopt);
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
    if(continuous_.mission==MissionType::kNone && line_recovery_memory_.budget<0 &&
       !curve_special && input.camera_feedback.settled &&
       (input.camera_feedback.actual_mode==CameraMode::kForward ||
        input.camera_feedback.actual_mode==CameraMode::kBallView)) {
      line_recovery_memory_.normal=actual_line_guide;
      if(real.direction==CruiseDirection::kLeft) line_recovery_memory_.last_left=true;
      else if(real.direction==CruiseDirection::kRight) line_recovery_memory_.last_left=false;
    }
  }
  ActionRequest request;
  CameraRequest camera = route.camera;
  const bool line_scan_override=automatic_approach && !finish_course &&
      line_state_==LineState::kRecoveryHeadScan &&
      line_head_search_.phase!=LineHeadSearch::Phase::kIdle;
  const bool curve_override=automatic_approach && !finish_course && curve_special && !line_scan_override;
  if (line_scan_override) {
    // No body motion while a scan transaction is pending. Both arms are
    // extended by the executor before the LINE scan's first yaw command.
    using Phase=LineHeadSearch::Phase;
    auto &scan=line_head_search_;
    route.hold=false;
    const auto &sf=input.camera_scan;
    auto start=[&](Phase scanning,CameraRequest request) {
      if (idle && commands_allowed && input.camera_feedback.settled &&
          input.camera_feedback.actual_mode==CameraMode::kForward) {
        scan.phase=scanning;scan.scan_started=false;
        scan.pause_hits.clear();scan.verify_hits.clear();
        scan.observation.Reset();
        camera=request;
      }
    };
    if (scan.phase==Phase::kPendingLeft)
      start(Phase::kScanningLeft,CameraRequest::kLineScanLeft);
    else if (scan.phase==Phase::kPendingRight)
      start(Phase::kScanningRight,CameraRequest::kLineScanRight);
    else if (scan.phase==Phase::kScanningLeft || scan.phase==Phase::kScanningRight ||
             scan.phase==Phase::kHoldingLeft || scan.phase==Phase::kHoldingRight) {
      if (sf.phase!=0) scan.scan_started=true;
      if ((sf.phase==2 || sf.phase==3) && sf.stop_id) {
        FeatureConfig hint=config_.line_features;
        hint.ransac_enabled=false;hint.guide_min_points=2;
        const auto guide=ComputeLineFeatures(input.line_centers,input.image_width,
            input.image_height,hint,2).guide;
        const bool valid_guide=guide.valid && std::isfinite(guide.offset) &&
            std::isfinite(guide.heading_rad);
        const auto verdict=scan.observation.Observe(sf.stop_id,sf.frame_is_fresh,
            input.now_sec,valid_guide,config_.line_recovery.scan_hold_sec,
            config_.line_recovery.scan_observation_timeout_sec,
            config_.line_recovery.scan_verify_window,config_.line_recovery.scan_verify_hits,
            std::nullopt,sf.stationary_since_sec,input.frame_capture_sec);
        using Verdict=StationaryScanObservation::Result;
        if(verdict==Verdict::kConfirmed) {
          scan.align_left=sf.yaw_deg>0.0;
          if(std::abs(sf.yaw_deg)>1.0) scan.last_found_left=scan.align_left;
          scan.align_remaining=std::abs(sf.yaw_deg)>1.0 ? 3 : 0;
          camera=CameraRequest::kScanReturn;scan.phase=Phase::kReturnFound;
        } else if(verdict==Verdict::kEmpty || verdict==Verdict::kRejected)
          camera=CameraRequest::kScanResume;
      }
      if (sf.finished && scan.scan_started &&
          (scan.phase==Phase::kScanningLeft || scan.phase==Phase::kScanningRight ||
           scan.phase==Phase::kHoldingLeft || scan.phase==Phase::kHoldingRight)) {
        if(++scan.completed_pairs<std::min(config_.line_recovery.scan_max_pairs,
                                          config_.scan_correction.max_attempts))
          scan.phase=scan.first_left ? Phase::kPendingLeft : Phase::kPendingRight;
        else {
          scan.phase=Phase::kReleaseArms;
          scan.release_requested=false;scan.release_seen_transition=false;
          scan.release_align_body=false;
        }
      }
    } else if (scan.phase==Phase::kReturnFound) {
      if (sf.finished && input.camera_feedback.settled &&
          input.camera_feedback.actual_mode==CameraMode::kForward) {
        scan.phase=Phase::kReleaseArms;
        scan.release_requested=false;
        scan.release_seen_transition=false;
        scan.release_align_body=true;
      }
    } else if (scan.phase==Phase::kReleaseArms) {
      if (!scan.release_requested) {
        // The executor retains both elbows between LINE_SCAN commands.
        // FORWARD returns the head and explicitly restores both elbows;
        // its DONE is delayed until the elbow interpolation is finished.
        if (idle && commands_allowed && input.camera_feedback.settled &&
            input.camera_feedback.actual_mode==CameraMode::kForward) {
          camera=CameraRequest::kForward;
          scan.release_requested=true;
        }
      } else if (!input.camera_feedback.settled ||
                 input.camera_feedback.actual_mode==CameraMode::kTransition) {
        scan.release_seen_transition=true;
      } else if (scan.release_seen_transition && input.camera_feedback.settled &&
                 input.camera_feedback.actual_mode==CameraMode::kForward) {
        if (scan.release_align_body) scan.phase=Phase::kAlignBody;
        else { scan.phase=Phase::kIdle;line_state_=LineState::kFinalHold; }
      }
    } else if (scan.phase==Phase::kAlignBody) {
      if (scan.align_action_id!=0 && input.delivery_feedback.done &&
          input.delivery_feedback.action_id==scan.align_action_id) {
        scan.align_action_id=0;
        if (scan.align_remaining>0) --scan.align_remaining;
      }
      if (scan.align_remaining==0 && scan.align_action_id==0 && idle) {
        scan.phase=Phase::kReobserve;scan.observation.Begin(1,input.now_sec);scan.observed_line.reset();
      } else if (scan.align_action_id==0 && idle && commands_allowed &&
                 input.allow_new_line_action) {
        request=WalkTurn(scan.align_left);
      }
    } else if(scan.phase==Phase::kReobserve) {
      const auto normal=ComputeLineFeaturesForFrame(input,features).guide;
      if(normal.valid && std::isfinite(normal.offset) && std::isfinite(normal.heading_rad)) scan.observed_line=normal;
      const auto verdict=scan.observation.Observe(1,true,input.now_sec,
          normal.valid && std::isfinite(normal.offset) && std::isfinite(normal.heading_rad),
          config_.line_recovery.scan_hold_sec,config_.line_recovery.scan_observation_timeout_sec,
          config_.line.line_stable_window,config_.line.line_stable_min_hits,
          std::nullopt,std::nullopt,input.frame_capture_sec);
      using Verdict=StationaryScanObservation::Result;
      if(verdict==Verdict::kConfirmed) {
        if(scan.observed_line) {
          line_recovery_memory_.normal=scan.observed_line;
          const auto dir=line_p2p_controller_.Compute(*scan.observed_line).direction;
          if(dir==CruiseDirection::kLeft) line_recovery_memory_.last_left=true;
          else if(dir==CruiseDirection::kRight) line_recovery_memory_.last_left=false;
        }
        scan={};line_state_=LineState::kNormal;
        line_visible_recovery_turns_=line_last_score_recovery_turns_=0;
        line_recovery_memory_.budget=-1;line_recovery_memory_.frozen.reset();line_recovery_memory_.frozen_left.reset();
        line_forward_recovery_issued_=0;line_forward_recovery_pending_=false;
        line_failure_pending_done_=false;
        // The usual normal LINE accumulator resumes on subsequent fresh frames.
      } else if(verdict!=Verdict::kWaiting) {
        RetryLineHeadSearch();
      }
    }
  }
  if(curve_override) {
    using Phase=CurveSearchState::Phase;
    const auto phase=curve_.phase;
    if(phase==Phase::kDrain && idle && commands_allowed) {
      if(last_ready_line_guide_ && last_ready_line_guide_->valid)
        curve_.saved_line=last_ready_line_guide_;
      curve_.observation.Reset();curve_.phase=Phase::kScan;curve_.started=false;
      ++curve_.head_attempts;
      // The course-index side is authoritative: a pixel hint may not scan
      // the forbidden side or change the existing arm/curve policy.
      const bool allowed_left=curve_.direction>0;
      const bool hinted=ScanFirstLeft(last_ball_scan_u_,input.intrinsics.cx,
          config_.curve_search.center_half_width_px,allowed_left);
      const bool first_left=hinted==allowed_left ? hinted : allowed_left;
      camera=first_left ? CameraRequest::kScanLeft : CameraRequest::kScanRight;
    } else if(phase==Phase::kScan || phase==Phase::kHold) {
      const auto& sf=input.camera_scan;
      if(sf.phase!=0) curve_.started=true;
      if((sf.phase==2 || sf.phase==3) && sf.stop_id) {
        const auto verdict=curve_.observation.Observe(sf.stop_id,sf.frame_is_fresh,
            input.now_sec,input.scan_ball_u && std::isfinite(*input.scan_ball_u),
            config_.curve_search.scan_hold_sec,config_.curve_search.scan_observation_timeout_sec,
            config_.curve_search.verify_window,config_.curve_search.verify_hits,
            input.scan_ball_u,sf.stationary_since_sec,input.frame_capture_sec);
        using Verdict=StationaryScanObservation::Result;
        if(verdict==Verdict::kConfirmed) {
          const auto correction=ObjectScanCorrection(sf.yaw_deg,curve_.observation.MeanU(),
                                                     input.intrinsics,config_.scan_correction);
          curve_.found_yaw=sf.yaw_deg;
          if(correction) {
            curve_.turn_left=correction->left;curve_.turns_remaining=correction->turns;
            curve_.phase=Phase::kReturnFound;
          } else curve_.phase=Phase::kReturnMiss;
          camera=CameraRequest::kScanReturn;
        } else if(verdict==Verdict::kEmpty || verdict==Verdict::kRejected) {
          camera=CameraRequest::kScanReturn;curve_.phase=Phase::kReturnMiss;
        }
      }
      if(sf.finished && curve_.started &&
         (curve_.phase==Phase::kScan || curve_.phase==Phase::kHold)) curve_.phase=Phase::kReturnMiss;
    } else if(phase==Phase::kReturnFound &&
              input.camera_feedback.settled && input.camera_feedback.actual_mode==CameraMode::kForward) {
      ++curve_.body_attempts; // Finding a target never resets head_attempts.
      curve_.phase=curve_.turns_remaining ? Phase::kBodyTurn : Phase::kBodyObserve;
      curve_.observation.Begin(1,input.now_sec);
    } else if(phase==Phase::kReturnMiss &&
              input.camera_feedback.settled && input.camera_feedback.actual_mode==CameraMode::kForward) {
      RetryCurveHeadSearch();
    } else if(phase==Phase::kBodyTurn && idle && commands_allowed && curve_.correction_action_id==0) {
      request=WalkTurn(curve_.turn_left);
    } else if(phase==Phase::kBodyObserve && idle && input.camera_feedback.settled &&
              input.camera_feedback.actual_mode==CameraMode::kForward) {
      const bool valid=input.ball_target && std::isfinite(input.ball_target->center_px.u) &&
          std::isfinite(input.ball_target->center_px.v);
      const auto verdict=curve_.observation.Observe(1,true,input.now_sec,valid,
          config_.curve_search.scan_hold_sec,config_.curve_search.scan_observation_timeout_sec,
          config_.ball.stable_window,config_.ball.stable_min_hits,
          std::nullopt,std::nullopt,input.frame_capture_sec);
      using Verdict=StationaryScanObservation::Result;
      if(verdict==Verdict::kConfirmed) {
        curve_.phase=Phase::kIdle;
        // Admission resumes through the existing BALL course/lock policy.
      } else if(verdict!=Verdict::kWaiting) RetryCurveHeadSearch();
    }
  }
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
    if (config_.enable_hurdle && hurdle_crosses_completed_<2 && (candidate_.mission == MissionType::kNone ||
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
                  config_.ball.ApproachCenter(decision_last_target_ ? decision_last_target_->center_px.v : 0.0,input.image_height) : config_.hurdle.target_u_px;
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
  } else if (!curve_override && !line_scan_override && input.advance_line_fsm && !manual_line_hold && !route.hold) {
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
      const bool goal_will_enter = !finish_course && has_ball_ && config_.enable_goal &&
          (!automatic_approach || ((input.camera_feedback.actual_mode==CameraMode::kForward ||
             input.camera_feedback.actual_mode==CameraMode::kBallView) &&
            post_pickup_line_actions_completed_+(line_plan_.Done(id)?0:1)>=approach_goal_retry_after_)) &&
          goal_controller_.CanEnterFromLine() &&
          post_pickup_goal_line_gate_active_ && line_valid && final_edge &&
          (post_pickup_five_step_completed_ ||
           post_pickup_line_actions_completed_ + (line_plan_.Done(id) ? 0 : 1) >=
               goal_controller_.RequiredLineWalkActions());
      // Only the configured number of completed straight blocks satisfies the shortcut.
      const bool straight_goal = !finish_course && has_ball_ && config_.enable_goal &&
          (!automatic_approach || ((input.camera_feedback.actual_mode==CameraMode::kForward ||
             input.camera_feedback.actual_mode==CameraMode::kBallView) &&
            post_pickup_line_actions_completed_+(line_plan_.Done(id)?0:1)>=approach_goal_retry_after_)) &&
          goal_controller_.CanEnterFromLine() &&
          post_pickup_goal_line_gate_active_ && line_valid && final_edge &&
          StraightGoalGateReached(completing_straight);
      if (final_edge) {
        const double ready_remaining =
            input.delivery_feedback.ready && !input.delivery_feedback.done
                ? config_.line_p2p.ready_lead_sec : 0.0;
        const bool pixel_owner=automatic_approach &&
            (continuous_.mission==MissionType::kBall ||
             continuous_.mission==MissionType::kGoal) &&
            line_plan_.ObjectOwned(id,continuous_.epoch);
        const bool ball_entry_owner=automatic_approach &&
            continuous_.mission==MissionType::kBall &&
            line_plan_.BallEntryOwned(id,continuous_.epoch);
        std::optional<Point2> averaged_center;
        std::optional<LineGuide> mean;
        if (pixel_owner) {
          const bool ball_owner=continuous_.mission==MissionType::kBall;
          const bool rediscovered_ball_probe=ball_owner &&
              line_plan_.IsProbe(id) && route.object_visible &&
              !continuous_.probe_active;
          averaged_center=line_plan_.FinishObjectCenter(
              id,input.now_sec,ready_remaining,
              !ball_owner, !ball_owner || line_plan_.ObjectQualified(id) ||
                  rediscovered_ball_probe);
          if (averaged_center && continuous_.last) {
            auto averaged_target=*continuous_.last;
            averaged_target.center_px=*averaged_center;
            LineGuide guide;
            if(continuous_.mission==MissionType::kGoal)
              guide=ObjectZoneGuide(averaged_target,
                  config_.goal.auto_approach_center_u_px-config_.goal.auto_approach_half_width_px,
                  config_.goal.auto_approach_center_u_px+config_.goal.auto_approach_half_width_px,
                  config_.object_approach.steering_deadband_deg,input.image_width);
            else {
              const auto bounds=config_.ball.ApproachBounds(averaged_target.center_px.v,input.image_height);
              guide=ObjectZoneGuide(averaged_target,bounds.first,bounds.second,
                  config_.object_approach.steering_deadband_deg,input.image_width);
            }
            if (guide.valid) mean=guide;
          }
        } else {
          mean=line_plan_.Finish(id,input.now_sec,ready_remaining);
        }
        last_ready_line_guide_=mean;
        if (finish_course && finish_floor_usable && mean && mean->valid)
          finish_last_stable_guide_ = mean;
        output.line_window_stats=line_plan_.Stats(id);
        decision=mean.value_or(LineGuide{});
        if (finish_course && (!finish_floor_usable || !decision.valid)) decision=raw;
        if (automatic_approach && continuous_.mission!=MissionType::kNone) {
          if (line_plan_.ObjectOwned(id,continuous_.epoch)) {
            if (((!line_plan_.IsProbe(id) && line_plan_.ObjectQualified(id)) ||
                 (line_plan_.IsProbe(id) && continuous_.mission==MissionType::kBall &&
                  route.object_visible && !continuous_.probe_active)) &&
                mean && mean->valid) {
              if (pixel_owner)
                continuous_.last_ready_center=averaged_center;
              else if (continuous_.mission==MissionType::kHurdle)
                continuous_.hurdle_last_mean_guide=*mean;
            }
          } else if (continuous_.mission==MissionType::kBall ||
                     continuous_.mission==MissionType::kGoal) {
            // The preceding floor LINE mean must not steer the virtual path.
            decision=route.guide_observed && route.guide ? *route.guide : LineGuide{};
          }
        }
        // First admitted BALL can still be on the preceding physical LINE
        // block. Finish its independent center mean BEFORE reading/steering
        // from the saved center. Never use the prior mean in this READY.
        if (ball_entry_owner) {
          const auto entry_center=line_plan_.FinishBallEntryCenter(
              id,input.now_sec,ready_remaining);
          if (entry_center && continuous_.last) {
            continuous_.last_ready_center=*entry_center;
            auto averaged_target=*continuous_.last;
            averaged_target.center_px=*entry_center;
            const auto bounds=config_.ball.ApproachBounds(
                entry_center->v,input.image_height);
            const auto entry_guide=ObjectZoneGuide(
                averaged_target,bounds.first,bounds.second,
                config_.object_approach.steering_deadband_deg,input.image_width);
            if (entry_guide.valid && !continuous_.near) {
              decision=entry_guide;
              route.guide=entry_guide;
              route.guide_observed=true;
              route.hold=false;
              line_failure_pending_done_=false;
            }
          }
        }
        output.line_decision_guide = decision;
        if (automatic_approach && ready && pixel_owner &&
            continuous_.mission==MissionType::kBall &&
            line_plan_.IsProbe(id) && !continuous_.probe_active &&
            route.object_visible && averaged_center && decision.valid) {
          // A previously lost BALL was rediscovered during the one permitted
          // probe. Its CURRENT, real-READY weighted mean selects the next
          // 32/33/36 block. Do not force the generic floor LINE failure FSM
          // to process a virtual BALL guide or delay this READY handoff.
          const auto direction=SteeringDecision(decision).direction;
          const MissionAction cruise=direction==CruiseDirection::kLeft ? MissionAction::kStepForwardLeft :
              direction==CruiseDirection::kStraight ? MissionAction::kStepForwardFour :
              direction==CruiseDirection::kRight ? MissionAction::kStepForwardRight :
              MissionAction::kNone;
          if(cruise!=MissionAction::kNone) {
            route.action={cruise,ActionCategory::kLocomotion,0,false};
            line_failure_pending_done_=false;
            line_state_=LineState::kNormal;
          }
        }
        const bool object_loss=automatic_approach &&
            continuous_.mission!=MissionType::kNone &&
            line_plan_.ObjectOwned(id,continuous_.epoch) &&
            !line_plan_.IsProbe(id) && !line_plan_.ObjectQualified(id);
        if (!finish_course && !decision.valid && !idle && !object_loss &&
            !route.resume_line_at_ready &&
            !(automatic_approach && continuous_.mission==MissionType::kHurdle &&
              continuous_.near))
          line_failure_pending_done_ = true;
        if (automatic_approach && continuous_.mission!=MissionType::kNone &&
            !(continuous_.mission==MissionType::kHurdle && continuous_.near) &&
            line_plan_.ObjectOwned(id,continuous_.epoch) &&
            !line_plan_.IsProbe(id) &&
            !line_plan_.ObjectQualified(id)) {
          continuous_.loss_pending=true;
          line_plan_.Abort(); // Real DONE still belongs to the original block.
          route.hold=true;
          if (continuous_.mission==MissionType::kBall && ready &&
              !continuous_.locked && !continuous_.near &&
              !continuous_.probe_used &&
              config_.object_approach.prelock_loss_extra_blocks>0 &&
              continuous_.last_ready_center && continuous_.last) {
            // A failed first BALL approach can queue exactly ONE 32/33/36
            // search unit at A's READY rather than waiting idle after DONE.
            auto remembered=*continuous_.last;
            remembered.center_px=*continuous_.last_ready_center;
            const auto bounds=config_.ball.ApproachBounds(
                remembered.center_px.v,input.image_height);
            const auto guide=ObjectZoneGuide(remembered,bounds.first,bounds.second,
                config_.object_approach.steering_deadband_deg,input.image_width);
            const auto dir=SteeringDecision(guide).direction;
            const MissionAction cruise=dir==CruiseDirection::kLeft ? MissionAction::kStepForwardLeft :
                dir==CruiseDirection::kStraight ? MissionAction::kStepForwardFour :
                dir==CruiseDirection::kRight ? MissionAction::kStepForwardRight :
                MissionAction::kNone;
            if (cruise!=MissionAction::kNone) {
              route.action={cruise,ActionCategory::kLocomotion,0,false};
              route.probe_start=true;
              route.hold=false;
            }
          }
        }
      }
      // HURDLE near votes only commit the NUMBER of remaining LINE units.
      // Keep collecting normal real-floor O/H until this unit's READY; then
      // reserve ONE final 32/33/36, even inside a multi-unit LINE block.
      if (automatic_approach && continuous_.mission==MissionType::kHurdle &&
          continuous_.near && ready &&
          continuous_.near_motion.action==MissionAction::kNone &&
          continuous_.near_motion_id==0) {
        const auto mean=line_plan_.Finish(id,input.now_sec,
                                         config_.line_p2p.ready_lead_sec);
        if (mean && mean->valid) {
          const auto direction=SteeringDecision(*mean).direction;
          const auto unit=direction==CruiseDirection::kLeft ?
              MissionAction::kLineLeftUnit :
              direction==CruiseDirection::kRight ?
                  static_cast<MissionAction>(config_.line_p2p.right_unit_action) :
              direction==CruiseDirection::kStraight ?
                  MissionAction::kLineStraightUnit : MissionAction::kNone;
          if (unit!=MissionAction::kNone) {
            continuous_.hurdle_last_mean_guide=*mean;
            last_ready_line_guide_=mean;
            decision=*mean;
            output.line_decision_guide=*mean;
            continuous_.near_motion={unit,ActionCategory::kLocomotion,0,true};
            route.action=continuous_.near_motion;
            route.near_final_action=true;
            request=route.action;
            line_plan_.Abort(); // Stop unissued repeats; retain physical IDs.
            line_failure_pending_done_=false;
          }
        }
      }
      if (line_auxiliary_id_ && line_auxiliary_done_ && idle) {
        const bool pose = line_auxiliary_action_ == MissionAction::kWalkMode ||
                          line_auxiliary_action_ == MissionAction::kDefaultPoseMode;
        if ((line_state_ == LineState::kScoreRecovery || pose) && !line_auxiliary_observing_) {
          if (!pose && !finish_course) ++line_guard_turns_completed_;
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
      // CONFIRM becomes DRAIN only on the second actual LINE block DONE.
      // With zero delay, do not prebook a third block at its final READY.
      // If the next block is not confirmed, LINE resumes normally on DONE.
      const bool defer_curve_confirm_ready=automatic_approach &&
          config_.curve_search.enabled &&
          config_.curve_search.scan_delay_walk_sequences==0 &&
          curve_.phase==CurveSearchState::Phase::kConfirm && !has_ball_ &&
          continuous_.mission==MissionType::kNone && ready && final_edge &&
          line_plan_.CruiseFor(id)!=MissionAction::kNone;
      if (!observation_wait && !line_failure_pending_done_ &&
          !route.hold && !route.near_final_action &&
          line_state_ != LineState::kFinalHold &&
          !defer_curve_confirm_ready) {
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
      if (!finish_course && request.action == MissionAction::kNone && idle && final_edge && !decision.valid &&
          !goal_will_enter && !straight_goal && line_state_ == LineState::kNormal)
        BeginObservation(LineState::kFailureObserve, input.now_sec);
    }
  }

  if (!curve_override && route.action.action!=MissionAction::kNone) request=route.action;
  if (finish_course && finish_sparse_line_ && !finish_loss_used_ && finish_last_stable_guide_ &&
      input.advance_line_fsm && commands_allowed && input.allow_new_line_action && input.allow_new_line_plan &&
      input.delivery_feedback.ready && !input.delivery_feedback.done &&
      command_coordinator_.PendingActionId() == input.delivery_feedback.action_id &&
      line_plan_.Contains(input.delivery_feedback.action_id) && !command_coordinator_.HasQueuedAction()) {
    line_plan_.Abort(); // Drop only unissued repeats of the interrupted normal block.
    bool reference = false;
    request = ComputeLineAction({}, input, false, false, &reference, false);
  }
  if (finish_course && finish_loss_action_id_ &&
      command_coordinator_.PendingActionId() == finish_loss_action_id_ &&
      input.delivery_feedback.action_id == finish_loss_action_id_ &&
      input.delivery_feedback.ready && !input.delivery_feedback.done &&
      !line_plan_.Done(finish_loss_action_id_) &&
      !command_coordinator_.HasQueuedAction() && input.allow_new_line_plan)
    request = LineLocomotion(MissionAction::kStepForwardFour, true);
  if(automatic_approach && !finish_course && curve_.phase==CurveSearchState::Phase::kDelay &&
     curve_.delay_remaining==1 && input.delivery_feedback.ready &&
     line_plan_.LastUnit(input.delivery_feedback.action_id))
    request={};
  if (!commands_allowed || manual_line_hold || (route.hold && route.action.action==MissionAction::kNone) || input.now_sec + 1e-9 < settle_until_sec_ || (active_mission_ == MissionType::kLine &&
      (!input.advance_line_fsm || (!input.allow_new_line_action && !route.near_final_action)))) request = {};
  if (finish_course && (!input.camera_feedback.settled ||
      input.camera_feedback.actual_mode!=CameraMode::kForward)) request={};
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
        automatic_approach && !route.resume_line_at_ready ? continuous_.epoch : 0,
        route.probe_start,
        continuous_.mission==MissionType::kBall ? config_.ball.ready_observation_end_weight :
        continuous_.mission==MissionType::kGoal ? config_.goal.ready_observation_end_weight :
        continuous_.mission==MissionType::kHurdle ? config_.hurdle.ready_observation_end_weight :
        config_.line_p2p.ready_observation_end_weight);
    else request = {};
  }
  if (!input.allow_new_commands) camera = CameraRequest::kNone;
  const auto parent_id = command_coordinator_.PendingActionId();
  output.command = command_coordinator_.Compute(active_mission_,
      candidate_.phase == CandidatePhase::kNone ? static_cast<int>(line_state_) :
          1000 + static_cast<int>(candidate_.phase), request, camera,
      input.delivery_feedback, input.now_sec);
  const auto& command = output.command;
  if (command.command_type == CommandType::kAction && command.action_id &&
      request.action != MissionAction::kNone && command.action == request.action)
    mission_recovery_line_guide_.reset();
  if(curve_.phase==CurveSearchState::Phase::kBodyTurn &&
     request.action!=MissionAction::kNone &&
     command.command_type==CommandType::kAction &&
     command.action_id!=0 && command.action==request.action)
    curve_.correction_action_id=command.action_id;
  if (LinePlan::IsUnit(command.action)) {
    line_plan_.Issued(command,
      command.control_phase == ControlPhase::kWaitingQueuedActionAck ? parent_id : 0);
    if (finish_course && finish_loss_request_ && !finish_loss_used_ &&
        request.action == command.action && command.command_type == CommandType::kAction && command.action_id) {
      finish_loss_used_ = true;
      finish_loss_action_id_ = command.action_id;
      finish_line_hits_.clear();
      // Exactly one physical unit regardless of configured LINE repetitions.
      line_plan_.Abort();
    }
    if(curve_.phase==CurveSearchState::Phase::kProbeWalk &&
       curve_.probe_action_id==0 && continuous_.mission==MissionType::kNone &&
       line_state_==LineState::kNormal && request.action==command.action &&
       command.command_type==CommandType::kAction && command.action_id!=0) {
      curve_.probe_action_id=command.action_id;
      // Exactly one physical 32/33/36, even if LINE repetitions are increased.
      // Abort only unissued repeats; the coordinator retains this ID to DONE.
      line_plan_.Abort();
    }
    if (line_state_==LineState::kRecoveryForward && line_forward_recovery_pending_ &&
        command.command_type==CommandType::kAction && command.action_id!=0) {
      ++line_forward_recovery_issued_;
      line_forward_recovery_pending_=false;
      line_failure_pending_done_=true;
      line_plan_.Abort(); // Recovery is exactly one physical unit; issued IDs still drain DONE.
    }
  }
  if (line_scan_override && line_head_search_.phase==LineHeadSearch::Phase::kAlignBody &&
      line_head_search_.align_action_id==0 && request.action!=MissionAction::kNone &&
      command.command_type==CommandType::kAction && command.action_id!=0 &&
      command.action==request.action)
    line_head_search_.align_action_id=command.action_id;
  output.approach_action_committed=automatic_approach &&
      continuous_.mission!=MissionType::kNone &&
      line_plan_.ObjectOwned(command.action_id,continuous_.epoch);
  if (automatic_approach && route.object_visible && route.guide_observed &&
      route.guide && route.guide->valid && !continuous_.near &&
      (continuous_.mission==MissionType::kBall || continuous_.mission==MissionType::kGoal) &&
      output.approach_action_committed) {
    output.approach_path_guide=route.display_guide.value_or(*route.guide);
    output.approach_path_reference_px={
         (continuous_.mission==MissionType::kGoal ?
             config_.goal.auto_approach_center_u_px : config_.ball.ApproachCenter(
                 output.line_features.guide.valid ? output.line_features.guide_reference_v_px :
                 config_.object_approach.reference_v_ratio*input.image_height,input.image_height)),
        output.line_features.guide.valid
            ? output.line_features.guide_reference_v_px
            : config_.object_approach.reference_v_ratio * input.image_height};
  }
  if (route.resume_line_at_ready && command.command_type==CommandType::kAction &&
      command.action_id && LinePlan::IsUnit(command.action))
    continuous_.probe_return_to_line=true;
  if (route.probe_start) {
    if (command.command_type==CommandType::kAction && command.action_id &&
        LinePlan::IsUnit(command.action)) {
      continuous_.probe_active=true;
      continuous_.probe_used=true;
      continuous_.loss_pending=false;
    } else line_plan_.Abort();
  }
  if (goal_head_search_.phase==GoalHeadSearch::Phase::kBodyTurn &&
      route.action.action!=MissionAction::kNone &&
      goal_head_search_.body_action_id==0 &&
      command.command_type==CommandType::kAction && command.action_id!=0 &&
      command.action==route.action.action)
    goal_head_search_.body_action_id=command.action_id;
  if (route.near_final_action && command.command_type==CommandType::kAction &&
      command.action_id && SameIssuedAction(route.action.action, command.action))
    continuous_.near_motion_id=command.action_id;
  if (command.command_type == CommandType::kAction && command.action_id &&
      request.action != MissionAction::kNone && SameIssuedAction(request.action, command.action) &&
      !LinePlan::IsUnit(command.action)) {
    if (!pose_transition && (candidate_.phase == CandidatePhase::kSearch || candidate_.phase == CandidatePhase::kRestore)) {
      candidate_.turn_id = command.action_id;
      candidate_.turn_is_restore = candidate_.phase == CandidatePhase::kRestore;
    } else if (route.edge_turn_step) {
      goal_edge_guard_.action_id=command.action_id;
    } else if (route.depth_recovery_step) {
      approach_goal_depth_action_id_=command.action_id;
      far_goal_depth_probe_.phase=FarGoalDepthProbe::Phase::kMotion;
    } else if (line_auxiliary_id_ != command.action_id) {
      line_auxiliary_id_ = command.action_id;
      line_auxiliary_action_ = command.action;
      line_auxiliary_done_ = false;
    }
  }
  if (input.delivery_feedback.ready && command.control_phase == ControlPhase::kWaitingQueuedActionAck)
    line_ready_handled_id_ = input.delivery_feedback.action_id;
  if(automatic_approach && !finish_course && input.raw_backboard_observation_available && has_ball_ &&
     command.command_type==CommandType::kAction && command.action_id!=0 &&
     !route.edge_turn_step && goal_edge_guard_.phase==GoalEdgeGuard::Phase::kIdle)
    ClearGoalEdgeObservation();
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
