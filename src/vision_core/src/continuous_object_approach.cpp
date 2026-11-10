#include "vision_core/mission_controller.hpp"
#include <algorithm>
#include <cmath>

namespace vision_core {
namespace {
void Push(std::deque<bool>& history,bool hit,int window) {
  history.push_back(hit);
  while (history.size()>static_cast<std::size_t>(window)) history.pop_front();
}
int Hits(const std::deque<bool>& history) {
  return std::count(history.begin(),history.end(),true);
}
// Apply GOAL-style near voting to valid BALL/HURDLE detections.
bool NearPixelWindowReady(const std::deque<bool>& detection_hits,
                          const std::deque<bool>& near_hits, int window,
                          int stable_min_hits, int near_min_hits) {
  const auto size=static_cast<std::size_t>(window);
  if (detection_hits.size()!=size || near_hits.size()!=size) return false;
  int valid=0, near=0;
  for (std::size_t j=0;j<size;++j) {
    if (!detection_hits[j]) continue;
    ++valid;
    if (near_hits[j]) ++near;
  }
  return valid>=stable_min_hits && near>=near_min_hits;
}
bool SamePosition(const ObjectTarget& a,const ObjectTarget& b,double max_distance,double max_size) {
  if (std::hypot(a.center_px.u-b.center_px.u,a.center_px.v-b.center_px.v)>max_distance) return false;
  if (a.area_px>0 && b.area_px>0 && std::abs(std::log(a.area_px/b.area_px))>2*max_size) return false;
  return true;
}
}

void MissionController::ReleaseContinuousApproach() {
  const auto mission=continuous_.mission;
  continuous_={};
  approach_evidence_={};
  // Discard only unissued repeats; ACKed/queued commands retain their owner.
  line_plan_.Abort();
  line_failure_pending_done_=false;
  last_ready_line_guide_.reset();
  line_stability_history_.clear();
  actual_line_stability_history_.clear();
  if (mission==MissionType::kBall) {
    ball_candidate_tracker_.Reset(); ball_association_tracker_.Reset();
    ball_target_id_=0;ball_target_locked_=ball_reacquiring_=false;
    ball_controller_.ClearEntryEvidence();
  } else if (mission==MissionType::kHurdle) hurdle_target_tracker_.Reset();
  else if (mission==MissionType::kGoal) {
    goal_edge_guard_={};
    goal_approach_walk_done_=false;
    far_goal_depth_probe_={};
    backboard_target_tracker_.Reset();
    goal_controller_.ClearDepthRecovery();
    approach_goal_depth_action_id_=0;
    approach_goal_bad_depth_frames_=0;
    goal_rgb_missing_frames_=goal_tracking_unusable_frames_=goal_out_of_range_frames_=0;
  }
}

// The failed pre-lock BALL probe may have already reserved a floor-LINE
// successor at READY. Preserve its action ID and LinePlan block on DONE.
void MissionController::FinishPrelockProbeToLine() {
  if (continuous_.mission!=MissionType::kBall) return;
  continuous_={};approach_evidence_={};
  ball_candidate_tracker_.Reset();ball_association_tracker_.Reset();
  ball_target_id_=0;ball_target_locked_=ball_reacquiring_=false;
  ball_controller_.ClearEntryEvidence();
  last_ready_line_guide_.reset();
  line_stability_history_.clear();
  // Keep actual floor-LINE direction, queued LINE block and transport IDs.
}

void MissionController::EnterContinuousMission(const MissionFrameInput& input,bool lost) {
  const auto route=continuous_;
  // Retain the raw last target only for identity/metadata; recovery direction
  // must use the most recent qualified final-READY coordinate mean.
  auto recovery_target=route.last.value_or(ObjectTarget{});
  if (route.last_ready_center) {
    recovery_target.center_px=*route.last_ready_center;
  } else if (!route.near && route.mission!=MissionType::kHurdle) {
    // No qualified READY mean: do not steer from a single last sighting.
    recovery_target.center_px.u=route.mission==MissionType::kBall
        ? config_.ball.ApproachCenter(recovery_target.center_px.v,input.image_height) : config_.goal.target_u_px;
  }
  // All IDs have physically completed. Consume that real feedback before the
  // mission controller starts; never give a LINE DONE to a pickup/shoot FSM.
  command_coordinator_.Compute(MissionType::kLine,0,{},CameraRequest::kNone,
                               input.delivery_feedback,input.now_sec);
  const int goal_index=goal_mission_index_;
  const int ball_index=ball_mission_index_;
  EnterMission(route.mission);
  if (route.mission_started) {
    goal_mission_index_=goal_index;ball_mission_index_=ball_index;
  }
  // A direct GOAL trial already acquired its mission index at startup.
  if (route.mission==MissionType::kGoal && config_.goal_trial_start_goal)
    goal_mission_index_=goal_index;
  if (route.mission==MissionType::kBall) {
    if (route.near) ball_controller_.EnterContinuousNear(input.now_sec);
    else ball_controller_.EnterContinuousRecovery(recovery_target,route.good_hits,input.image_width,input.image_height,input.now_sec);
    ball_result_.mode=route.near ? BallMode::kWaitCameraDown : BallMode::kForwardRecoveryTurn;
    ball_result_.active=true;
    if (route.id) {
      ball_target_id_=route.id;ball_target_locked_=true;ball_reacquiring_=false;
      ball_association_tracker_.Seed(*route.last);
    }
  } else if (route.mission==MissionType::kHurdle) {
    if (route.near) hurdle_controller_.EnterContinuousNear(
        input.now_sec,route.hurdle_down_ready_action);
    else hurdle_controller_.EnterContinuousRecovery(recovery_target,route.good_hits,input.image_width,input.image_height,input.now_sec);
    hurdle_result_.mode=route.near ? HurdleMode::kWaitCameraDown : HurdleMode::kRecoveryStep;
    hurdle_result_.active=true;
    hurdle_target_tracker_.Lock();
  } else {
    if (route.near) {
      // A FINE view needs its own fresh edge evidence, not far-WALK votes.
      goal_edge_guard_ = {};
      goal_controller_.EnterContinuousNear(recovery_target,route.pose,
          lost ? route.good_hits : route.hits,route.pose_hits,route.near_hits,
          input.image_width,input.image_height,input.now_sec,lost);
    } else goal_controller_.EnterContinuousApproach(recovery_target,route.pose,
          route.good_hits,route.pose_hits,route.near_hits,
          input.image_width,input.image_height,input.now_sec,true);
    goal_result_.mode=route.near ? GoalMode::kWaitFinePose : GoalMode::kApproach;
    goal_result_.active=true;
    // GOAL retains candidate tracking throughout recovery; no permanent ID lock.
  }
  if (!route.near) {
    recovering_approach_=route;recovering_approach_->mission_started=true;
  } else {
    recovering_approach_.reset();last_line_score_.reset();
    actual_line_stability_history_.clear();
  }
  continuous_={};approach_evidence_={};
  hurdle_down_ready_accumulator_.Reset();
  approach_goal_probe_=approach_return_ball_view_=false;
  approach_goal_empty_retries_=0;
  approach_goal_bad_depth_frames_=0;
  goal_rgb_missing_frames_=goal_tracking_unusable_frames_=goal_out_of_range_frames_=0;
  approach_goal_depth_action_id_=0;
  far_goal_depth_probe_={};
  approach_camera_started_=-1;
  line_plan_.Reset();line_ready_handled_id_=line_auxiliary_id_=0;
  line_auxiliary_done_=line_auxiliary_observing_=false;
  decision_observation_active_=decision_observation_restart_pending_=false;
  deferred_done_={};
  // The head is already settled in this view. The next fresh object window
  // belongs after the real stance DONE, not to a stale LINE/camera edge.
  previous_camera_mode_=input.camera_feedback.actual_mode;
  previous_camera_settled_=input.camera_feedback.settled;
  last_command_={};
}

bool MissionController::ResumeContinuousApproach(const MissionFrameInput& input) {
  if (!recovering_approach_) return false;
  auto& saved=*recovering_approach_;
  std::optional<ObjectTarget> target;
  bool found=false;
  if (saved.mission==MissionType::kBall) {
    target=input.ball_target;found=ball_result_.mode==BallMode::kApproachBall;
    saved.hits=ball_controller_.ObservationHits();
    if (target) {
      if (input.ball_track_id) saved.id=input.ball_track_id;
    }
    if (found || saved.near) ball_result_.action={};
  } else if (saved.mission==MissionType::kHurdle) {
    target=input.hurdle_target;found=hurdle_result_.mode==HurdleMode::kApproach;
    saved.hits=hurdle_controller_.ObservationHits();
    if (target) {
      if (input.hurdle_track_id) saved.id=input.hurdle_track_id;
    }
    if (found || saved.near) hurdle_result_.action={};
  } else {
    target=input.backboard_target;
    found=goal_result_.mode==GoalMode::kSearch || goal_result_.mode==GoalMode::kApproach ||
        goal_result_.mode==GoalMode::kCoarseAdjust || goal_result_.mode==GoalMode::kWaitFinePose ||
        goal_result_.mode==GoalMode::kFineAdjust;
    saved.hits=goal_controller_.ObservationHits();
    saved.pose_hits=goal_controller_.PoseObservationHits();
    saved.near_hits=goal_controller_.NearObservationHits();
    if (target) {
      saved.depth=input.backboard_center_depth_m;saved.pose=input.goal_pose;
      saved.near=saved.near || GoalNearWindowReady(saved.hits,saved.near_hits,config_.goal);
      if (input.backboard_track_id) saved.id=input.backboard_track_id;
    }
    if (found || saved.near) goal_result_.action={};
  }
  if (target) saved.last=target;
  if (!saved.near && (!found || !target)) return false;
  const bool completing=input.delivery_feedback.done &&
      input.delivery_feedback.action_id==command_coordinator_.PendingActionId() &&
      !command_coordinator_.HasQueuedAction();
  if ((command_coordinator_.HasPendingAction() && !completing) ||
      command_coordinator_.HasQueuedAction() || command_coordinator_.ActionAckTimedOut() ||
      !input.allow_new_commands || !input.allow_new_body_actions ||
      !input.allow_new_mission_entry || !input.camera_feedback.settled) return false;
  if (!saved.near && input.camera_feedback.actual_mode!=
      (saved.mission==MissionType::kGoal ? CameraMode::kGoal : CameraMode::kForward)) return false;
  auto resumed=saved;
  command_coordinator_.Compute(active_mission_,0,{},CameraRequest::kNone,input.delivery_feedback,input.now_sec);
  recovering_approach_.reset();continuous_=std::move(resumed);
  // Require fresh near votes after a successful far-object recovery.
  if (!continuous_.near && (continuous_.mission==MissionType::kBall ||
                            continuous_.mission==MissionType::kHurdle)) {
    continuous_.hits.clear();continuous_.near_hits.clear();
  }
  continuous_.missing=0;continuous_.good_hits=continuous_.hits;
  continuous_.loss_pending=false;
  line_plan_.Reset();line_state_=LineState::kNormal;
  line_guard_turns_completed_=0;line_failure_pending_done_=false;
  line_stability_history_.clear();line_ready_handled_id_=line_auxiliary_id_=0;
  actual_line_stability_history_.clear();
  line_auxiliary_done_=line_auxiliary_observing_=line_observation_active_=false;
  decision_observation_active_=decision_observation_restart_pending_=false;
  deferred_done_={};last_command_={};
  if (continuous_.near) {
    EnterContinuousMission(input,!target);
  } else {
    // This is the same object mission, not completion or a new admission.
    // Keep its lock, completed blocks, identity, and mission sequence number.
    active_mission_=MissionType::kLine;
    approach_goal_probe_=approach_return_ball_view_=false;
  }
  return true;
}

bool MissionController::PrepareGoalDepthRecovery(const MissionFrameInput& input,
    bool visible,bool usable,bool idle,RouteDecision& result) {
  // The FIRST GOAL sighting without usable central Depth is an acquisition
  // failure. No blind forward step: let the normal left/right head scan run.
  if(!goal_approach_walk_done_) {
    approach_goal_bad_depth_frames_=0;
    far_goal_depth_probe_={};
    return false;
  }
  auto& probe=far_goal_depth_probe_;
  const bool done=approach_goal_depth_action_id_ && input.delivery_feedback.done &&
      input.delivery_feedback.action_id==approach_goal_depth_action_id_;
  if(probe.phase==FarGoalDepthProbe::Phase::kMotion) {
    result.hold=true;
    if(done) {
      approach_goal_depth_action_id_=0;
      ++probe.attempts;
      probe.phase=FarGoalDepthProbe::Phase::kObserve;
      probe.observe_start_sec=input.now_sec;
      probe.hits.clear();probe.qualified=false;
    }
    return true;
  }
  if(probe.phase==FarGoalDepthProbe::Phase::kObserve) {
    result.hold=true;
    probe.hits.push_back(visible&&usable);
    while(probe.hits.size()>static_cast<std::size_t>(config_.goal.stable_window))
      probe.hits.pop_front();
    if(probe.hits.size()==static_cast<std::size_t>(config_.goal.stable_window) &&
       std::count(probe.hits.begin(),probe.hits.end(),true)>=config_.goal.stable_min_hits)
      probe.qualified=true;
    if(input.now_sec+1e-9<probe.observe_start_sec+config_.post_motion_observation_sec ||
       probe.hits.size()<static_cast<std::size_t>(config_.goal.stable_window))return true;
    if(probe.qualified) {
      probe={};approach_goal_bad_depth_frames_=0;
      return false;
    }
    if(!visible) {
      probe={};
      if(continuous_.mission==MissionType::kGoal)continuous_.loss_pending=true;
      return false; // Lost YOLO uses the ordinary GOAL head scan.
    }
    if(probe.attempts>=config_.goal.far_depth_probe_max_attempts){
      // Depth-only failure: no neck scan, no fourth body step.
      line_plan_.Abort();
      ReleaseContinuousApproach();
      approach_return_ball_view_=true;
      post_pickup_goal_line_gate_active_=true;
      approach_goal_retry_after_=post_pickup_line_actions_completed_+1;
      result.camera=CameraRequest::kBallViewLine;
      return true;
    }
    probe.phase=FarGoalDepthProbe::Phase::kIdle;
    approach_goal_bad_depth_frames_=config_.goal.lost_frames; // next bounded probe
    return true;
  }
  if(!visible){approach_goal_bad_depth_frames_=0;return false;}
  if(usable){approach_goal_bad_depth_frames_=0;probe={};return false;}
  result.hold=true;line_plan_.Abort();
  if(++approach_goal_bad_depth_frames_<config_.goal.lost_frames || !idle)return true;
  if(!input.raw_backboard_u)return true; // Never invent a direction from stale pixels.
  if(!input.allow_new_commands || !input.allow_new_body_actions ||
     !input.allow_new_line_action || command_coordinator_.ActionAckTimedOut() ||
     input.now_sec+1e-9<settle_until_sec_)return true;
  const double u=*input.raw_backboard_u;
  const double left=config_.goal.auto_approach_center_u_px-config_.goal.auto_approach_half_width_px;
  const double right=config_.goal.auto_approach_center_u_px+config_.goal.auto_approach_half_width_px;
  const auto action=u<left ? MissionAction::kStepForwardLeftOne :
                    u>right ? MissionAction::kStepForwardRightOne :
                              MissionAction::kStepForwardOne;
  result.action={action,ActionCategory::kLocomotion,0,false};
  result.depth_recovery_step=true;
  return true;
}


void MissionController::ClearGoalEdgeObservation() {
  auto& e=goal_edge_guard_;
  if(e.phase!=GoalEdgeGuard::Phase::kIdle)return;
  if(active_mission_==MissionType::kLine && foot_pose_==FootPose::kWalk)return;
  e.observing=false;e.ready=false;
  e.left_qualified=e.right_qualified=false;
  e.left_hits.clear();e.right_hits.clear();
}

// Shared configurable raw RGB edge evidence. Reuse the post-FINE DONE window
// instead of starting a separate 1s timer after the same stationary frame.
void MissionController::ObserveGoalEdgeSample(const MissionFrameInput& input) {
  auto& e=goal_edge_guard_;
  if(!e.observing){
    e.observing=true;e.observation_start_sec=input.now_sec;
    e.left_hits.clear();e.right_hits.clear();
    e.left_qualified=e.right_qualified=false;
  }
  const double stamp=input.frame_capture_sec.value_or(input.now_sec);
  if (!std::isfinite(stamp) || stamp<=e.last_sample_sec) return;
  e.last_sample_sec=stamp;
  e.seen_hits.push_back(input.raw_backboard_target.has_value());
  while(e.seen_hits.size()>static_cast<std::size_t>(config_.goal.edge_stable_window))
    e.seen_hits.pop_front();
  bool left=false,right=false;
  if(input.raw_backboard_target){
    const auto& box=input.raw_backboard_target->box_px;
    left=box.width>0&&box.height>0&&box.x<=config_.goal.edge_margin_px;
    right=box.width>0&&box.height>0&&box.x+box.width>=
          static_cast<double>(input.image_width)-config_.goal.edge_margin_px;
    if(left&&right)left=right=false; // Ambiguous clipped/oversized box.
  }
  e.left_hits.push_back(left);e.right_hits.push_back(right);
  const auto window=static_cast<std::size_t>(config_.goal.edge_stable_window);
  while(e.left_hits.size()>window)e.left_hits.pop_front();
  while(e.right_hits.size()>window)e.right_hits.pop_front();
  // Latch a qualifying full window within this observation.
  if(e.left_hits.size()==window){
    if(std::count(e.left_hits.begin(),e.left_hits.end(),true)>=config_.goal.edge_stable_min_hits)
      e.left_qualified=true;
    if(std::count(e.right_hits.begin(),e.right_hits.end(),true)>=config_.goal.edge_stable_min_hits)
      e.right_qualified=true;
  }
}

bool MissionController::PrepareGoalEdgeGuard(const MissionFrameInput& input,
    bool idle,ActionRequest& action) {
  auto& e=goal_edge_guard_;
  if(!input.raw_backboard_observation_available ||
     !input.camera_feedback.settled ||
     input.camera_feedback.actual_mode!=CameraMode::kGoal) {
    if(e.phase==GoalEdgeGuard::Phase::kIdle)e={};
    return false;
  }
  const auto select_turn=[&]() {
    return e.turn_left ?
        (foot_pose_==FootPose::kFine ? MissionAction::kTurnLeft :
                                      MissionAction::kTurnLeftInPlace) :
        (foot_pose_==FootPose::kFine ? MissionAction::kTurnRight :
                                      MissionAction::kStepForwardRightOneHurdle);
  };
  // FINE TURN_LEFT/RIGHT (7/8) use the requested yaw to select JSON repeats.
  // WALK in-place turns (31/28) play their fixed asset once; keep yaw at zero.
  const int edge_turn_yaw_deg=foot_pose_==FootPose::kFine ? 15 : 0;
  if(e.phase==GoalEdgeGuard::Phase::kTurning) {
    if(e.action_id && input.delivery_feedback.done &&
       input.delivery_feedback.action_id==e.action_id) {
      e.action_id=0;
      if(++e.turns_completed>=2){
        e.phase=GoalEdgeGuard::Phase::kReobserve;
        e.observing=false;e.ready=false;
        e.left_qualified=e.right_qualified=false;
        e.left_hits.clear();e.right_hits.clear();e.seen_hits.clear();
      }
    }
    if(e.phase==GoalEdgeGuard::Phase::kTurning && e.action_id==0 && idle &&
       input.allow_new_commands && input.allow_new_body_actions) {
      action={select_turn(),ActionCategory::kLocomotion,edge_turn_yaw_deg,false};
    }
    return true;
  }
  const bool far=active_mission_==MissionType::kLine && foot_pose_==FootPose::kWalk;
  const auto begin_turns=[&]() {
    e.turn_left=e.left_qualified;
    e.turns_completed=0;e.action_id=0;
    e.phase=GoalEdgeGuard::Phase::kPending;
    if(far) line_plan_.Abort(); // Only unissued repeats; queued ACKs remain real.
  };
  if(e.phase==GoalEdgeGuard::Phase::kPending) {
    if(!idle || line_plan_.Outstanding()) return true;
    e.phase=GoalEdgeGuard::Phase::kTurning;
    if(input.allow_new_commands && input.allow_new_body_actions)
      action={select_turn(),ActionCategory::kLocomotion,edge_turn_yaw_deg,false};
    return true;
  }
  if(far && e.phase==GoalEdgeGuard::Phase::kIdle) {
    // One physical unit's ACK/start..READY window. Queued storage ACK is not a start.
    const auto id=command_coordinator_.PendingActionId();
    const auto start=id ? line_plan_.StartEstimate(id) : std::optional<double>{};
    if(id && (!start || input.now_sec+1e-9<*start)) return false;
    if(e.sample_action_id!=id) {
      e.sample_action_id=id;e.sample_ready_closed=false;
      e.observing=false;e.ready=false;
      e.left_hits.clear();e.right_hits.clear();e.seen_hits.clear();
      e.left_qualified=e.right_qualified=false;
    }
    const bool done=id && input.delivery_feedback.action_id==id && input.delivery_feedback.done;
    if(!e.sample_ready_closed && !done &&
        (!start || input.frame_capture_sec.value_or(input.now_sec)+1e-9>=*start))
      ObserveGoalEdgeSample(input);
    const bool ready=id && input.delivery_feedback.action_id==id && input.delivery_feedback.ready;
    if(ready || done) e.sample_ready_closed=true;
    // Initial acquisition may qualify at rest. Moving evidence commits only at READY.
    if((ready || idle) && e.left_qualified!=e.right_qualified) {
      begin_turns();
      if(idle && !line_plan_.Outstanding()) {
        e.phase=GoalEdgeGuard::Phase::kTurning;
        if(input.allow_new_commands && input.allow_new_body_actions)
          action={select_turn(),ActionCategory::kLocomotion,edge_turn_yaw_deg,false};
      }
      return true;
    }
    return false; // Normal far approach never waits for an edge-only observation.
  }
  if(e.ready)return false;
  if(!idle)return false;
  ObserveGoalEdgeSample(input);
  const bool full=e.left_hits.size()>=static_cast<std::size_t>(config_.goal.edge_stable_window);
  const bool qualified=e.left_qualified!=e.right_qualified ||
      std::count(e.seen_hits.begin(),e.seen_hits.end(),true)>=config_.goal.stable_min_hits;
  const bool fast_ready=config_.race_round2_fast_enabled && full && qualified;
  if(!full || (!fast_ready && input.now_sec+1e-9<
      e.observation_start_sec+config_.post_motion_observation_sec)) return true;
  e.observing=false;e.ready=true;
  if(e.left_qualified!=e.right_qualified) {
    begin_turns();
    e.phase=GoalEdgeGuard::Phase::kTurning;
    if(input.allow_new_commands && input.allow_new_body_actions)
      action={select_turn(),ActionCategory::kLocomotion,edge_turn_yaw_deg,false};
    return true;
  }
  if(far && std::count(e.seen_hits.begin(),e.seen_hits.end(),true)==0 &&
      continuous_.mission==MissionType::kGoal) {
    // A failed fresh recheck is RGB loss, never evidence of an unclipped board.
    continuous_.loss_pending=true;
    goal_rgb_missing_frames_=config_.goal.lost_frames;
  }

  e.phase=GoalEdgeGuard::Phase::kIdle;
  return false;
}

MissionController::RouteDecision MissionController::PrepareContinuousApproach(
    const MissionFrameInput& input,const Features& line,bool idle) {
  RouteDecision result;
  if (line_state_ == LineState::kFinalHold) {
    result.hold = true;
    return result;
  }
  const auto& camera=input.camera_feedback;
  const auto& cfg=config_.object_approach;
  const bool advance=input.advance_line_fsm && input.allow_new_mission_entry;
  const bool settled=camera.settled && camera.actual_mode!=CameraMode::kTransition;
  const bool goal_view=camera.actual_mode==CameraMode::kGoal;
  // Adopt a newly stable, Depth-valid GOAL identity before edge/scan states
  // can return early. NEVER inherit stale READY coordinates or old ID votes.
  const bool stable_board = goal_view && camera.settled &&
      input.backboard_target && input.backboard_track_id &&
      input.backboard_entry_track_hits.value_or(0)>=config_.goal.stable_min_hits &&
      input.backboard_entry_track_frames.value_or(0)>=config_.goal.stable_window &&
      input.backboard_center_depth_m && std::isfinite(*input.backboard_center_depth_m) &&
      *input.backboard_center_depth_m>=config_.backboard_min_depth_m &&
      *input.backboard_center_depth_m<=config_.backboard_max_depth_m;
  if (continuous_.mission==MissionType::kGoal && stable_board &&
      input.backboard_track_id!=continuous_.id) {
    continuous_.id=input.backboard_track_id;
    continuous_.last=input.backboard_target;
    continuous_.last_ready_center.reset();
    continuous_.hits.clear();continuous_.near_hits.clear();
    continuous_.pose_hits.clear();continuous_.good_hits.clear();
    continuous_.missing=0;continuous_.loss_pending=false;
    goal_rgb_missing_frames_=goal_tracking_unusable_frames_=0;
    approach_evidence_[1]={};
    // An already-issued camera/body motion is not interrupted. At a settled
    // post-turn observation, a fresh stable ID releases the obsolete search.
    if (idle && input.camera_scan.phase==0 &&
        (goal_head_search_.phase==GoalHeadSearch::Phase::kReobserve ||
         goal_head_search_.phase==GoalHeadSearch::Phase::kBodyTurn))
      goal_head_search_={};
  }
  // The raw-bbox edge guard precedes Depth recovery; far evidence stays in motion.
  if(goal_view && input.advance_line_fsm && input.raw_backboard_observation_available && has_ball_ &&
     !input.backboard_image_out_of_depth_range &&
     !(continuous_.mission==MissionType::kGoal && continuous_.near)) {
    ActionRequest edge_action;
    if(PrepareGoalEdgeGuard(input,idle,edge_action)) {
      result.hold=true;result.action=edge_action;
      result.edge_turn_step=edge_action.action!=MissionAction::kNone;
      return result;
    }
  }
  const auto camera_request=[&](CameraRequest request) {
    result.hold=true;
    if (!idle || !input.allow_new_commands) return;
    if (approach_camera_started_<0) approach_camera_started_=input.now_sec;
    if (input.now_sec-approach_camera_started_>=config_.camera_motion_timeout_sec) {
      // No fabricated completion or automatic body command after camera failure.
      line_state_=LineState::kFinalHold;
      return;
    }
    result.camera=request;
  };
  // Dedicated stationary GOAL-view search. Only fresh captures from
  // encoder-confirmed stops with valid center Depth may qualify.
  if (goal_head_search_.phase!=GoalHeadSearch::Phase::kIdle) {
    using Phase=GoalHeadSearch::Phase;
    auto &scan=goal_head_search_;
    result.hold=true;
    const auto &sf=input.camera_scan;
    const auto start=[&](Phase phase,CameraRequest request) {
      if(idle && input.allow_new_commands && input.allow_new_body_actions &&
         camera.settled && camera.actual_mode==CameraMode::kGoal) {
        scan.phase=phase;scan.started=false;
        scan.pause_hits.clear();scan.verify_hits.clear();
        scan.observation.Reset();scan.exhausted=false;
        result.camera=request;
      }
    };
    if(scan.phase==Phase::kPendingLeft)
      start(Phase::kScanningLeft,CameraRequest::kGoalScanLeft);
    else if(scan.phase==Phase::kPendingRight)
      start(Phase::kScanningRight,CameraRequest::kGoalScanRight);
    else if(scan.phase==Phase::kScanningLeft || scan.phase==Phase::kScanningRight ||
            scan.phase==Phase::kHoldingLeft || scan.phase==Phase::kHoldingRight) {
      if(sf.phase!=0) scan.started=true;
      const bool stopped=(sf.phase==2 || sf.phase==3) && sf.stop_id;
      if(stopped) {
        const bool centered=input.scan_backboard_u &&
            std::abs(*input.scan_backboard_u-config_.goal.auto_approach_center_u_px)<=
                config_.goal.head_scan_center_half_width_px;
        const auto verdict=scan.observation.Observe(sf.stop_id,sf.frame_is_fresh,
            input.now_sec,centered,config_.goal.head_scan_hold_sec,
            config_.goal.head_scan_observation_timeout_sec,
            config_.goal.head_scan_verify_window,config_.goal.head_scan_verify_hits);
        using Verdict=StationaryScanObservation::Result;
        if(verdict==Verdict::kConfirmed) {
          const double yaw=sf.yaw_deg;
          scan.turn_left=yaw>=0;
          scan.resume_right=yaw>1.0;
          scan.turns_remaining=std::abs(yaw)<=
                  config_.goal.head_scan_body_turn_one_threshold_deg ? 0 :
              std::abs(yaw)<=config_.goal.head_scan_body_turn_two_threshold_deg ? 1 : 2;
          scan.phase=Phase::kReturnFound;
          result.camera=CameraRequest::kScanReturn;
        } else if(verdict==Verdict::kEmpty || verdict==Verdict::kRejected) {
          result.camera=CameraRequest::kScanResume;
        }
      }
      if(sf.finished && scan.started) {
        scan.phase=Phase::kIdle;scan.exhausted=true;approach_goal_empty_frames_=0;
      }
    } else if(scan.phase==Phase::kReturnFound) {
      if(sf.finished && camera.settled && camera.actual_mode==CameraMode::kGoal)
        scan.phase=Phase::kBodyTurn;
    } else if(scan.phase==Phase::kBodyTurn) {
      if(scan.body_action_id && input.delivery_feedback.done &&
         input.delivery_feedback.action_id==scan.body_action_id) {
        scan.body_action_id=0;
        if(scan.turns_remaining>0) --scan.turns_remaining;
      }
      if(scan.turns_remaining==0 && !scan.body_action_id && idle) {
        scan.phase=Phase::kReobserve;scan.reobserve_started_sec=input.now_sec;
        scan.observation.Reset();scan.observed_target.reset();
      } else if(!scan.body_action_id && idle &&
                input.allow_new_commands && input.allow_new_body_actions) {
        result.action={scan.turn_left ? MissionAction::kTurnLeftInPlace :
                                        MissionAction::kStepForwardRightOneHurdle,
                       ActionCategory::kLocomotion,0,false};
      }
    } else if(scan.phase==Phase::kReobserve) {
      // Reconfirm with a full fresh window, retaining success even if its
      // final frame misses. Never let an old lost ID override this result.
      if(!idle || !camera.settled || camera.actual_mode!=CameraMode::kGoal) return result;
      const bool valid=input.backboard_target && input.backboard_center_depth_m &&
          std::isfinite(*input.backboard_center_depth_m) &&
          *input.backboard_center_depth_m>=config_.backboard_min_depth_m &&
          *input.backboard_center_depth_m<=config_.backboard_max_depth_m &&
          std::abs(input.backboard_target->center_px.u-config_.goal.auto_approach_center_u_px)<=
              config_.goal.head_scan_center_half_width_px;
      if(valid) { scan.observed_target=input.backboard_target;scan.observed_id=input.backboard_track_id; }
      const auto verdict=scan.observation.Observe(1,true,input.now_sec,valid,
          config_.goal.head_scan_hold_sec,config_.goal.head_scan_observation_timeout_sec,
          config_.goal.head_scan_verify_window,config_.goal.head_scan_verify_hits);
      using Verdict=StationaryScanObservation::Result;
      if(verdict==Verdict::kWaiting)return result;
      if(verdict==Verdict::kConfirmed) {
        if(continuous_.mission==MissionType::kGoal) {
          continuous_.id=scan.observed_id;continuous_.last=scan.observed_target;
          continuous_.last_ready_center.reset();continuous_.good_hits.clear();
          continuous_.loss_pending=false;continuous_.missing=0;
          continuous_.hits.clear();continuous_.near_hits.clear();continuous_.pose_hits.clear();
          line_plan_.Reset();continuous_.probe_used=false;
        }
        goal_rgb_missing_frames_=goal_tracking_unusable_frames_=goal_out_of_range_frames_=0;
        scan.phase=Phase::kIdle;scan.used=false;scan.exhausted=false;
        approach_goal_empty_frames_=0;approach_evidence_={};
        return result;
      }
      scan.observation.Reset();scan.observed_target.reset();
      if(scan.resume_right) {
        scan.phase=Phase::kPendingRight;scan.resume_right=false;
      } else { scan.phase=Phase::kIdle;scan.exhausted=true; }
      return result;
    }
    if(scan.phase!=Phase::kIdle) return result;
    // If a complete scan failed, let the original bounded return-to-LINE
    // fallback resume rather than endlessly rescanning the same GOAL view.
    if(!input.backboard_target) return result;
  }
  if (approach_return_ball_view_) {
    if (!settled || camera.actual_mode!=CameraMode::kBallView) {
      camera_request(CameraRequest::kBallViewLine);return result;
    }
    approach_return_ball_view_=false;approach_camera_started_=-1;
    line_stability_history_.clear();
    actual_line_stability_history_.clear();
  }
  // Preserve the carrying/post-pickup gate. Camera transitions still drain all
  // issued locomotion before requesting GOAL, even in continuous mode.
  const bool goal_gate=has_ball_ && config_.enable_goal && post_pickup_goal_line_gate_active_ &&
      (post_pickup_five_step_completed_ || post_pickup_line_actions_completed_>=goal_controller_.RequiredLineWalkActions()) &&
      post_pickup_line_actions_completed_>=approach_goal_retry_after_;
  // Fast branch: the last required LINE unit was approved while actual
  // floor LINE was valid, BEFORE moving the head. Its physical DONE (not
  // current GOAL-view floor pixels) unlocks the GOAL camera observation.
  const bool early_goal_complete=config_.race_round2_fast_enabled && goal_early_camera_armed_ &&
      goal_early_camera_action_id_!=0 &&
      line_plan_.Done(goal_early_camera_action_id_) &&
      goal_gate && continuous_.mission==MissionType::kNone;
  if (early_goal_complete ||
      (advance && settled &&
       (camera.actual_mode==CameraMode::kForward ||
        camera.actual_mode==CameraMode::kBallView) &&
       goal_gate && continuous_.mission==MissionType::kNone &&
       line_state_==LineState::kNormal)) {
    approach_goal_probe_=true;line_plan_.Abort();
    if (early_goal_complete) {
      goal_early_camera_armed_=false;
      goal_early_camera_action_id_=0;
    }
  }
  if (approach_goal_probe_) {
    if (!settled || !goal_view) { camera_request(CameraRequest::kGoal);return result; }
    approach_goal_probe_=false;approach_camera_started_=-1;
    approach_goal_empty_frames_=0;approach_evidence_={};
    goal_approach_walk_done_=false;far_goal_depth_probe_={};
    goal_edge_guard_={}; // A re-entered GOAL view gets fresh edge evidence.
    goal_head_search_={};
    line_stability_history_.clear();
    actual_line_stability_history_.clear();
  }
  if (!settled || !advance || line_state_==LineState::kFinalHold) {
    result.hold=true;return result;
  }
  LineGuide goal_guide;
  bool goal_depth_usable=false;
  if (goal_view && input.backboard_target) {
    const double z=input.backboard_center_depth_m.value_or(input.goal_pose.valid ? input.goal_pose.z_m : 0);
    goal_depth_usable=std::isfinite(z) && z>0.05;
    goal_guide=ObjectZoneGuide(*input.backboard_target,
        config_.goal.auto_approach_center_u_px-config_.goal.auto_approach_half_width_px,
        config_.goal.auto_approach_center_u_px+config_.goal.auto_approach_half_width_px,
        cfg.steering_deadband_deg,input.image_width);
  }
  const std::array<MissionType,3> missions{MissionType::kBall,MissionType::kGoal,MissionType::kHurdle};
  const std::array<std::optional<ObjectTarget>,3> targets{input.ball_target,input.backboard_target,input.hurdle_target};
  const std::array<std::uint64_t,3> ids{input.ball_track_id,input.backboard_track_id,input.hurdle_track_id};
  // Each object uses the SAME permissive identity gate for both its
  // candidate tracker and the continuous-approach router. An additional
  // common 200px / size-ratio gate must not reject a still-tracked target.
  const auto same_target=[&](const ObjectTarget& previous,const ObjectTarget& current,std::size_t index) {
    const double max_distance=index==0 ? config_.ball_candidate_tracking.max_center_distance_px :
        index==1 ? config_.backboard_candidate_tracking.max_center_distance_px :
                   config_.hurdle_candidate_tracking.max_center_distance_px;
    const double max_size=index==0 ? config_.ball_candidate_tracking.max_size_log_ratio :
        index==1 ? config_.backboard_candidate_tracking.max_size_log_ratio :
                   config_.hurdle_candidate_tracking.max_size_log_ratio;
    return SamePosition(previous,current,max_distance,max_size);
  };
  const std::array<int,3> windows{config_.ball.stable_window,config_.goal.stable_window,config_.hurdle.stable_window};
  const std::array<int,3> entry_hits{config_.ball.entry_min_hits,config_.goal.stable_min_hits,config_.hurdle.stable_min_hits};
  const std::array<bool,3> eligible{
      !has_ball_ && config_.enable_ball && camera.actual_mode==CameraMode::kForward,
      has_ball_ && config_.enable_goal && goal_view,
      !has_ball_ && config_.enable_hurdle && hurdle_crosses_completed_<2 && camera.actual_mode==CameraMode::kForward &&
          !hurdle_controller_.IgnoringUntil(input.now_sec)};
  for (std::size_t i=0;i<3;++i) {
    auto& evidence=approach_evidence_[i];
    bool visible=eligible[i] && targets[i].has_value();
    if (visible && i==0) visible=targets[i]->center_px.v<=config_.ball.upper_acquire_v_px;
    if (visible && i==2) visible=targets[i]->center_px.v<=config_.hurdle.upper_acquire_v_px;
    if (visible && evidence.last && ((ids[i] && evidence.id && ids[i]!=evidence.id) ||
        !same_target(*evidence.last,*targets[i],i))) evidence={};
    Push(evidence.hits,visible,windows[i]);
    if (i==1) Push(evidence.near_hits,visible &&
        GoalNearDistanceSample(input.backboard_center_depth_m,input.goal_pose,
                               config_.goal.approach_short_z_m),windows[i]);
    else Push(evidence.near_hits,visible &&
        targets[i]->center_px.v >= (i==0 ? config_.ball.approach_two_step_v_px
                                        : config_.hurdle.approach_two_step_v_px),windows[i]);
    if (visible) { evidence.id=ids[i];evidence.last=targets[i]; }
  }
  bool acquired=false;
  if (continuous_.mission==MissionType::kNone) {
    // Same priority as legacy LINE admission: HURDLE before BALL.
    for (std::size_t i:{std::size_t{2},std::size_t{0},std::size_t{1}}) {
      const auto& evidence=approach_evidence_[i];
      const int hits=i==0 ? input.ball_entry_track_hits.value_or(Hits(evidence.hits)) :
          i==2 ? input.hurdle_entry_track_hits.value_or(Hits(evidence.hits)) :
                 input.backboard_entry_track_hits.value_or(Hits(evidence.hits));
      if (!eligible[i] || !targets[i] || !evidence.last || evidence.hits.empty() ||
          !evidence.hits.back() || hits<entry_hits[i]) continue;
      continuous_.mission=missions[i];continuous_.id=ids[i];
      continuous_.epoch=next_approach_epoch_++;
      continuous_.last=targets[i];continuous_.hits=evidence.hits;
      continuous_.near_hits=evidence.near_hits;
      // The first object-owned READY does not exist yet. Preserve the real
      // per-ID LINE admission mean for the single pre-lock loss probe.
      if(i==0) continuous_.last_ready_center=input.ball_entry_mean_center;
      acquired=true;
      if (i==1) {
        approach_goal_empty_retries_=0;
        goal_head_search_.used=false; // Re-arm search for a later acquired GOAL loss.
      }
      break;
    }
  }
  if (continuous_.mission==MissionType::kNone) {
    if (goal_view) {
      // No physical GOAL approach has DONE yet. Image-only sightings are
      // treated as unseen for head-scan and bounded LINE retry purposes.
      result.hold=true;
      if (input.backboard_target) approach_goal_empty_frames_=0;
      else if (++approach_goal_empty_frames_>=config_.goal.lost_frames) {
        if (config_.goal.head_scan_enabled && !goal_head_search_.used) {
          goal_head_search_.used=true;
          goal_head_search_.phase=GoalHeadSearch::Phase::kPendingLeft;
          return result;
        }
        if (approach_goal_empty_retries_>=cfg.goal_empty_max_retries) {
          // Terminal failure: keep draining issued IDs; only explicit Reset
          // may restart this search. No further head or walking requests.
          line_plan_.Abort();line_state_=LineState::kFinalHold;
          approach_goal_search_failed_=true;
          return result;
        }
        ++approach_goal_empty_retries_;
        approach_return_ball_view_=true;
        post_pickup_goal_line_gate_active_=true;
        approach_goal_retry_after_=post_pickup_line_actions_completed_+1;
        line_plan_.Abort();camera_request(CameraRequest::kBallViewLine);
      }
    }
    return result;
  }
  const std::size_t i=continuous_.mission==MissionType::kBall ? 0 :
                      continuous_.mission==MissionType::kGoal ? 1 : 2;
  auto target=targets[i];
  // All GOAL replacement decisions were made before early scan returns.
  // An unqualified alternate ID cannot steer an ongoing approach.
  if (i==1 && target && ids[i] && continuous_.id && ids[i]!=continuous_.id)
    target.reset();
  if (target && continuous_.last &&
      ((ids[i] && continuous_.id && ids[i]!=continuous_.id) ||
       !same_target(*continuous_.last,*target,i))) target.reset();
  if (target) {
    continuous_.last=target;continuous_.missing=0;
    if (ids[i]) continuous_.id=ids[i];
    if (i==1) {
      continuous_.depth=input.backboard_center_depth_m;
      continuous_.pose=input.goal_pose;
    }
  } else if (i==1 && input.backboard_image_without_depth && !continuous_.near) {
    // The image is still present: rejected Depth is coordinate loss, not an
    // unconfirmed object disappearance. Keep its bounded Depth recovery budget.
    continuous_.missing=0;
  } else ++continuous_.missing;
  result.object_visible=target.has_value();
  if (target && !continuous_.locked && !continuous_.near &&
      (continuous_.probe_active || continuous_.loss_pending)) {
    // A fresh sighting starts a new detection episode, but already completed
    // physical approach blocks remain part of the same target's lock.
    continuous_.hits.clear();continuous_.good_hits.clear();
    continuous_.pose_hits.clear();continuous_.near_hits.clear();
    approach_evidence_[i].hits.clear();approach_evidence_[i].near_hits.clear();
    Push(approach_evidence_[i].hits,true,windows[i]);
    if (i==1) Push(approach_evidence_[i].near_hits,
        GoalNearDistanceSample(input.backboard_center_depth_m,input.goal_pose,
                               config_.goal.approach_short_z_m),windows[i]);
    if (continuous_.probe_active)
      line_plan_.TruncateUnissuedProbeRepeats(continuous_.epoch); // Retain this unit's READY.
    continuous_.missing=0;continuous_.loss_pending=false;
    continuous_.probe_active=false;continuous_.probe_used=false;
  }
  if (!acquired) Push(continuous_.hits,target.has_value(),windows[i]);
  Push(continuous_.pose_hits,target && input.goal_pose.valid,windows[i]);
  if (i==1) {
    if (!acquired) Push(continuous_.near_hits,target &&
        GoalNearDistanceSample(input.backboard_center_depth_m,input.goal_pose,
                               config_.goal.approach_short_z_m),windows[i]);
    continuous_.near=continuous_.near || (target &&
        GoalNearWindowReady(continuous_.hits,continuous_.near_hits,config_.goal));
  } else {
    const double near_v=i==0 ? config_.ball.approach_two_step_v_px
                              : config_.hurdle.approach_two_step_v_px;
    const int stable_hits=i==0 ? config_.ball.stable_min_hits
                               : config_.hurdle.stable_min_hits;
    const int near_hits=i==0 ? config_.ball.near_v_min_hits
                             : config_.hurdle.near_v_min_hits;
    if (!acquired) Push(continuous_.near_hits,target && target->center_px.v>=near_v,windows[i]);
    continuous_.near=continuous_.near || (target &&
        NearPixelWindowReady(continuous_.hits,continuous_.near_hits,windows[i],
                             stable_hits,std::min(near_hits,windows[i])));
  }
  if (target) continuous_.good_hits=continuous_.hits;
  // An object may disappear before any object-owned block has been issued.
  // READY evidence cannot mark that episode lost: after the last physical
  // LINE unit drains, expire the pre-lock ID instead of holding forever.
  if(i==0 && !target && !continuous_.locked && !continuous_.near &&
     !continuous_.loss_pending && !line_plan_.Active() && !line_plan_.Outstanding() &&
     continuous_.missing>config_.ball_candidate_tracking.missing_frame_limit)
    continuous_.loss_pending=true;
  // A true RGB loss is independent of tracker IDs and Depth validity.
  // A separate bounded counter handles persistent visible but unusable
  // candidates (e.g. repeated unstable replacements) rather than HOLD forever.
  if(i==1 && !continuous_.near) {
    const bool rgb_visible=input.raw_backboard_target.has_value() ||
        input.raw_backboard_u.has_value() || input.backboard_target.has_value() ||
        input.backboard_image_without_depth || input.backboard_image_out_of_depth_range;
    goal_rgb_missing_frames_=rgb_visible ? 0 :
        std::min(config_.goal.lost_frames,goal_rgb_missing_frames_+1);
    const bool tracking_unusable=input.raw_backboard_target && !target &&
        !input.backboard_image_without_depth &&
        !input.backboard_image_out_of_depth_range;
    goal_tracking_unusable_frames_=tracking_unusable ?
        std::min(config_.goal.lost_frames,goal_tracking_unusable_frames_+1) : 0;
    // A missing object must not truncate the remainder of a physically
    // executing 32/33/36 block. Its last READY or actual DONE decides
    // loss; once all issued units drain, a persistent RGB miss can scan.
    if((goal_rgb_missing_frames_>=config_.goal.lost_frames ||
        goal_tracking_unusable_frames_>=config_.goal.lost_frames) &&
       !line_plan_.Active() && !line_plan_.Outstanding())
      continuous_.loss_pending=true;
  }
  const bool lost=continuous_.loss_pending;
  if(i==1 && !continuous_.near && input.backboard_image_out_of_depth_range) {
    // A measured but forbidden Depth is NOT a missing-Depth search request.
    // Discard unissued movements, then return to LINE after bounded evidence.
    result.hold=true;
    line_plan_.Abort();
    goal_out_of_range_frames_=std::min(
        config_.goal.lost_frames,goal_out_of_range_frames_+1);
    if(goal_out_of_range_frames_>=config_.goal.lost_frames &&
       idle && !line_plan_.Outstanding()) {
      ReleaseContinuousApproach();
      approach_return_ball_view_=true;
      post_pickup_goal_line_gate_active_=true;
      approach_goal_retry_after_=post_pickup_line_actions_completed_+1;
      result.camera=CameraRequest::kBallViewLine;
    }
    return result;
  }
  if(i==1) goal_out_of_range_frames_=0;
  // A completed GOAL approach with visible RGB but invalid Depth takes the
  // direction-aware 3x probe path BEFORE generic object-loss handling.
  if(i==1 && !continuous_.near && input.backboard_image_without_depth &&
     PrepareGoalDepthRecovery(input,true,false,idle,result))return result;
  if(i==1 && !continuous_.near && far_goal_depth_probe_.phase!=FarGoalDepthProbe::Phase::kIdle &&
     PrepareGoalDepthRecovery(input,input.backboard_target.has_value() || input.backboard_image_without_depth,
                              input.backboard_target.has_value(),idle,result))return result;
  // A GOAL already acquired during approach uses the same head scan as the
  // first empty GOAL view before falling back to BALL_VIEW_LINE. All issued
  // locomotion IDs must drain before a scan camera request can start.
  if (i==1 && !continuous_.near && lost && !target &&
      !input.backboard_image_without_depth && goal_view &&
      !goal_head_search_.used && config_.goal.head_scan_enabled) {
    line_plan_.Abort();
    result.hold=true;
    if (idle && !line_plan_.Outstanding()) {
      goal_head_search_.used=true;
      goal_head_search_.phase=GoalHeadSearch::Phase::kPendingLeft;
    }
    return result;
  }
  if (lost && i==1 && !continuous_.near && goal_head_search_.used && goal_head_search_.exhausted &&
      !target && !input.backboard_image_without_depth &&
      idle && !line_plan_.Outstanding()) {
    // Completed left/right scan found no board: do not enter a stale
    // locked-object recovery loop; return to BALL_VIEW_LINE.
    ReleaseContinuousApproach();
    approach_return_ball_view_=true;
    post_pickup_goal_line_gate_active_=true;
    approach_goal_retry_after_=post_pickup_line_actions_completed_+1;
    result.hold=true;
    result.camera=CameraRequest::kBallViewLine;
    return result;
  }
  if (lost && !continuous_.locked && !continuous_.near) {
    line_plan_.Abort();result.hold=true;
    if (!idle) return result;
    const bool have_hurdle_guide=continuous_.hurdle_last_mean_guide.valid ||
        continuous_.hurdle_last_guide.valid;
    const bool probe_available=!continuous_.probe_used &&
        cfg.prelock_loss_extra_blocks>0 &&
        (i==2 ? have_hurdle_guide :
            continuous_.last_ready_center.has_value() && continuous_.last.has_value());
    if (probe_available) {
      if (!input.allow_new_commands || !input.allow_new_body_actions ||
          !input.allow_new_line_action || !input.allow_new_line_plan ||
          command_coordinator_.ActionAckTimedOut() ||
          input.now_sec+1e-9<settle_until_sec_) return result;
      LineGuide probe_guide;
      if (i==2) {
        probe_guide=continuous_.hurdle_last_mean_guide.valid ?
            continuous_.hurdle_last_mean_guide : continuous_.hurdle_last_guide;
      } else {
        auto averaged_target=*continuous_.last;
        averaged_target.center_px=*continuous_.last_ready_center;
        if(i==1)
          probe_guide=ObjectZoneGuide(averaged_target,
              config_.goal.auto_approach_center_u_px-config_.goal.auto_approach_half_width_px,
              config_.goal.auto_approach_center_u_px+config_.goal.auto_approach_half_width_px,
              cfg.steering_deadband_deg,input.image_width);
        else {
          const auto bounds=config_.ball.ApproachBounds(averaged_target.center_px.v,input.image_height);
          probe_guide=ObjectZoneGuide(averaged_target,bounds.first,bounds.second,
              cfg.steering_deadband_deg,input.image_width);
        }
      }
      const auto direction=SteeringDecision(probe_guide).direction;
      const auto cruise=direction==CruiseDirection::kLeft ? MissionAction::kStepForwardLeft :
          direction==CruiseDirection::kStraight ? MissionAction::kStepForwardFour :
          direction==CruiseDirection::kRight ? MissionAction::kStepForwardRight : MissionAction::kNone;
      if (cruise!=MissionAction::kNone) {
        result.action={cruise,ActionCategory::kLocomotion,0,false};
        result.probe_start=true;
        return result;
      }
    }
    const bool restore=goal_view;
    ReleaseContinuousApproach();
    if (restore) {
      approach_return_ball_view_=true;
      post_pickup_goal_line_gate_active_=true;
      approach_goal_retry_after_=post_pickup_line_actions_completed_+1;
      camera_request(CameraRequest::kBallViewLine);
    }
    return result;
  }
  if (continuous_.near || (lost && continuous_.locked)) {
    // If normal LINE already reserved B at A's READY before HURDLE near
    // qualified, B is exactly the required one extra unit. Never reserve C.
    if (i==2 && continuous_.near && continuous_.near_motion_id==0 &&
        continuous_.near_motion.action==MissionAction::kNone &&
        command_coordinator_.HasQueuedAction() &&
        LinePlan::IsUnit(command_coordinator_.QueuedAction())) {
      continuous_.near_motion_id=command_coordinator_.QueuedActionId();
      continuous_.near_motion={command_coordinator_.QueuedAction(),
                               ActionCategory::kLocomotion,0,true};
      // A normal LINE READY had already reserved the required extra unit.
      // Its ACK is sufficient to allow DOWN to begin in parallel.
      if (config_.race_round2_fast_enabled) hurdle_down_camera_armed_=true;
    }
    // A newly committed HURDLE waits for the executing LINE unit's READY
    // window; unlike BALL/GOAL, do not abort its O/H accumulator yet.
    const bool hurdle_ready_tail = i==2 && continuous_.near &&
        continuous_.near_motion_id==0 &&
        continuous_.near_motion.action==MissionAction::kNone;
    if (!hurdle_ready_tail) line_plan_.Abort();
    result.hold=true;
    if (!continuous_.near) {
      if (idle && input.allow_new_commands && input.allow_new_body_actions) {
        EnterContinuousMission(input,true);result.entered=true;
      }
      return result;
    }
    // HURDLE: commit exactly ONE successor at the current LINE READY.
    // Direction is chosen from the normal block O/H mean, not from the
    // near-trigger frame. Without READY, fallback occurs only after DONE.
    if (i==2) {
      if (!idle && hurdle_ready_tail) {
        result.hold=false; // Let StepPlannedLine collect and finish at READY.
        return result;
      }
      if (!idle) return result;
      if (continuous_.near_motion_id) {
        if (!input.delivery_feedback.done ||
            input.delivery_feedback.action_id!=continuous_.near_motion_id ||
            !input.allow_new_commands || !input.allow_new_body_actions)return result;
        continuous_.near_motion_id=0;
        continuous_.near_motion={};
        EnterContinuousMission(input,false);result.entered=true;
        return result;
      }
      if (continuous_.near_motion.action!=MissionAction::kNone) {
        result.action=continuous_.near_motion;result.near_final_action=true;
        return result;
      }
      const auto guide=continuous_.hurdle_last_mean_guide.valid ?
          continuous_.hurdle_last_mean_guide :
          continuous_.hurdle_last_guide.valid ?
          continuous_.hurdle_last_guide : line.guide;
      if(!guide.valid)return result;
      const auto dir=SteeringDecision(guide).direction;
      const auto unit=dir==CruiseDirection::kLeft ? MissionAction::kLineLeftUnit :
          dir==CruiseDirection::kRight ?
              static_cast<MissionAction>(config_.line_p2p.right_unit_action) :
          dir==CruiseDirection::kStraight ? MissionAction::kLineStraightUnit :
                                           MissionAction::kNone;
      if(unit==MissionAction::kNone)return result;
      continuous_.near_motion={unit,ActionCategory::kLocomotion,0,true};
      result.action=continuous_.near_motion;result.near_final_action=true;
      return result;
    }
    // GOAL: near is already stable (10/7 + depth votes). Drain the
    // current WALK and request FINE stance directly, without a pre-FINE
    // 1s observation. Fresh post-pose samples handle edge and next action.
    if (i==1) {
      if (idle && input.allow_new_commands && input.allow_new_body_actions) {
        EnterContinuousMission(input,false);result.entered=true;
      }
      return result;
    }
    last_line_score_.reset();
    actual_line_stability_history_.clear();
    if (!idle) return result;
    if (continuous_.near_motion_id) {
      if (!input.delivery_feedback.done ||
          input.delivery_feedback.action_id!=continuous_.near_motion_id ||
          !input.allow_new_commands || !input.allow_new_body_actions)
        return result;
      // Never issue the next step/turn until this exact command's real DONE.
      continuous_.near_motion_id=0;
      continuous_.near_motion={};
      if (continuous_.near_actions.empty()) {
        if (continuous_.near_reobserve_after_turns) {
          // LOST near BALL: both in-place turns have physically DONE.
          // Reacquire fresh stationary samples; never lower the camera yet.
          continuous_.near_reobserve_after_turns=false;
          continuous_.near_observing=false;
          continuous_.near_qualified=false;
          continuous_.near_observation_hits.clear();
          continuous_.near_sum_u=continuous_.near_sum_v=0.0;
          continuous_.near_valid_count=0;
          return result;
        }
        // Fixed turn/forward chain is complete; only now change the camera.
        EnterContinuousMission(input,false);result.entered=true;
        return result;
      }
      const auto next=continuous_.near_actions.front();
      continuous_.near_actions.pop_front();
      continuous_.near_motion={next,ActionCategory::kLocomotion,0,false};
      result.action=continuous_.near_motion;result.near_final_action=true;
      return result;
    }
    if (continuous_.near_motion.action!=MissionAction::kNone) {
      result.action=continuous_.near_motion;result.near_final_action=true;
      return result;
    }
    if (!continuous_.near_observing) {
      continuous_.near_observing=true;
      continuous_.near_observation_started_sec=input.now_sec;
      return result; // First post-DONE frame starts, but never votes in, the window.
    }
    continuous_.near_observation_hits.push_back(target.has_value());
    while (continuous_.near_observation_hits.size()>static_cast<std::size_t>(windows[i]))
      continuous_.near_observation_hits.pop_front();
    if (target) {
      continuous_.near_sum_u+=target->center_px.u;
      continuous_.near_sum_v+=target->center_px.v;
      ++continuous_.near_valid_count;
    }
    const int minimum=i==0 ? config_.ball.stable_min_hits :
        i==1 ? config_.goal.stable_min_hits : config_.hurdle.stable_min_hits;
    if (continuous_.near_observation_hits.size()==static_cast<std::size_t>(windows[i]) &&
        Hits(continuous_.near_observation_hits)>=minimum) {
      continuous_.near_qualified=true;
      continuous_.hits=continuous_.near_observation_hits;
    }
    if (input.now_sec+1e-9 < continuous_.near_observation_started_sec+
            config_.post_motion_observation_sec ||
        continuous_.near_observation_hits.size()<static_cast<std::size_t>(windows[i]))
      return result;
    if (!continuous_.near_qualified) {
      ++continuous_.near_attempts;
      // Use the existing, already time-weighted last qualified BALL READY
      // center WITHOUT averaging the partially failed stationary window.
      // Recover at most once: two same-direction WALK turns, each real DONE,
      // followed by one fresh 1-second observation. A central remembered
      // center retains the original stationary retry path.
      if (i==0 && continuous_.near_turn_batches==0 &&
          continuous_.last_ready_center && continuous_.last &&
          input.allow_new_commands && input.allow_new_body_actions) {
        const auto center=*continuous_.last_ready_center;
        const auto bounds=config_.ball.ApproachBounds(center.v,input.image_height);
        const MissionAction turn=center.u<bounds.first ? MissionAction::kTurnLeftInPlace :
            center.u>bounds.second ? MissionAction::kStepForwardRightOneHurdle :
            MissionAction::kNone;
        if (turn!=MissionAction::kNone) {
          continuous_.near_turn_batches=1;
          continuous_.near_reobserve_after_turns=true;
          continuous_.near_actions.clear();
          continuous_.near_actions.push_back(turn);
          continuous_.near_motion={turn,ActionCategory::kLocomotion,0,false};
          result.action=continuous_.near_motion;
          result.near_final_action=true;
          continuous_.near_observation_hits.clear();
          continuous_.near_sum_u=continuous_.near_sum_v=0.0;
          continuous_.near_valid_count=0;
          return result;
        }
      }
      const int limit=i==0 ? config_.ball.recovery_max_observations :
          i==1 ? config_.goal.recovery_max_observations : config_.hurdle.recovery_max_observations;
      if (continuous_.near_attempts>=limit) {
        if (input.allow_new_commands && input.allow_new_body_actions) {
          // A completed turn recovery uses the same final path as center
          // loss: enter BALL_VIEW; do not FINAL_HOLD merely for turning.
          EnterContinuousMission(input,true);result.entered=true;
        }
      } else {
        continuous_.near_observation_started_sec=input.now_sec;
        continuous_.near_observation_hits.clear();
        continuous_.near_sum_u=continuous_.near_sum_v=0.0;
        continuous_.near_valid_count=0;
      }
      return result;
    }
    if (continuous_.near_valid_count>0 && continuous_.last) {
      continuous_.last->center_px={continuous_.near_sum_u/continuous_.near_valid_count,
                                   continuous_.near_sum_v/continuous_.near_valid_count};
    }
    if (i==1) {
      if (input.allow_new_commands && input.allow_new_body_actions) {
        EnterContinuousMission(input,false);result.entered=true;
      }
      return result;
    }
    const double v=continuous_.last->center_px.v;
    if (i==0) {
      const auto bounds=config_.ball.ApproachBounds(v,input.image_height);
      const auto outer=config_.ball.ExtremeApproachBounds(v,input.image_height);
      const double u=continuous_.last->center_px.u;
      const bool left=u<bounds.first, right=u>bounds.second;
      if (left || right) {
        const auto turn=left ? MissionAction::kTurnLeftInPlace :
                               MissionAction::kStepForwardRightOneHurdle;
        const bool extreme=left ? u<outer.first : u>outer.second;
        continuous_.near_actions.push_back(turn);
        if(extreme) continuous_.near_actions.push_back(turn);
        // All near-grid columns share the same v split: turn(s), then
        // two or one physical forward step(s), then camera DOWN.
        continuous_.near_actions.push_back(
            v<config_.ball.approach_direct_down_v_px ?
                MissionAction::kStepForwardTwo : MissionAction::kStepForwardOne);
      } else {
        // Center uses the same lower horizontal split as both side zones.
        continuous_.near_actions.push_back(
            v<config_.ball.approach_direct_down_v_px ?
                MissionAction::kStepForwardTwo : MissionAction::kStepForwardOne);
      }
    } else {
      // HURDLE already had two near steps; add the configured physical steps
      // before DOWN without inserting a new observation between actions.
      continuous_.near_actions.push_back(MissionAction::kStepForwardTwo);
      for(int n=0;n<config_.hurdle.pre_down_extra_forward_steps/2;++n)
        continuous_.near_actions.push_back(MissionAction::kStepForwardTwo);
      if(config_.hurdle.pre_down_extra_forward_steps%2)
        continuous_.near_actions.push_back(MissionAction::kStepForwardOne);
    }
    const auto first=continuous_.near_actions.front();
    continuous_.near_actions.pop_front();
    continuous_.near_motion={first,ActionCategory::kLocomotion,0,false};
    result.action=continuous_.near_motion;result.near_final_action=true;
    return result;
  }
  // Near commitment and soft-loss release take precedence over coordinate
  // recovery. A visible near board must never issue another WALK search step.
  // Far-depth retry is handled above, before any LOST/scan transition.
  const bool probe_final_edge=continuous_.probe_active &&
      line_plan_.LastUnit(input.delivery_feedback.action_id) &&
      (input.delivery_feedback.ready || input.delivery_feedback.done);
  if (!target) {
    if (i==0 && probe_final_edge && !continuous_.locked &&
        continuous_.probe_active && !continuous_.probe_return_to_line) {
      // The sole recovery unit is at READY. Return to the last known
      // REAL floor LINE direction without waiting for its physical DONE.
      // Virtual BALL steering is never used as the floor fallback.
      std::optional<double> remembered=last_line_score_;
      if (!remembered && line.guide.valid) {
        const auto floor=line_p2p_controller_.Compute(line.guide);
        if (floor.direction!=CruiseDirection::kNone) remembered=floor.error;
      }
      if (remembered && std::isfinite(*remembered)) {
        const auto deadband=config_.line_p2p.steering_deadband;
        const MissionAction cruise=*remembered < -deadband ? MissionAction::kStepForwardLeft :
            *remembered > deadband ? MissionAction::kStepForwardRight :
            MissionAction::kStepForwardFour;
        result.action={cruise,ActionCategory::kLocomotion,0,false};
        result.resume_line_at_ready=true;
        result.hold=false;
        return result;
      }
    }
    // Missing frames inside a walking block retain the last path. Only that
    // block's READY evidence may declare loss and stop its successor.
    LineGuide saved_guide;
    if (i!=2 && continuous_.last_ready_center && continuous_.last) {
      auto averaged_target=*continuous_.last;
      averaged_target.center_px=*continuous_.last_ready_center;
      if(i==1)
        saved_guide=ObjectZoneGuide(averaged_target,
            config_.goal.auto_approach_center_u_px-config_.goal.auto_approach_half_width_px,
            config_.goal.auto_approach_center_u_px+config_.goal.auto_approach_half_width_px,
            cfg.steering_deadband_deg,input.image_width);
      else {
        const auto bounds=config_.ball.ApproachBounds(averaged_target.center_px.v,input.image_height);
        saved_guide=ObjectZoneGuide(averaged_target,bounds.first,bounds.second,
            cfg.steering_deadband_deg,input.image_width);
      }
    }
    const LineGuide hurdle_fallback=continuous_.hurdle_last_guide.valid ?
        continuous_.hurdle_last_guide : line.guide;
    result.guide=saved_guide.valid ? saved_guide : (i==2 ? hurdle_fallback : LineGuide{});
    // The already-issued LINE block may still queue its next unit at READY;
    // absence of a READY pixel mean must not truncate that physical block.
    // Final READY cannot start a new block without a valid guide.
    const bool may_finish_block=line_plan_.Active() && !probe_final_edge;
    result.hold=(!result.guide->valid && !may_finish_block) || probe_final_edge;
    return result;
  }
  LineGuide guide=line.guide;
  if (i==1) {
    result.display_guide=goal_guide;
    guide=goal_guide;
  } else if (i==0) {
    // On the FIRST admission, use the weighted candidate history rather than
    // a single last YOLO frame to select the initial 32/33/36 zone. Once an
    // object-owned READY exists, that later measured mean takes priority.
    auto steering_target=*target;
    if(continuous_.blocks==0 && continuous_.last_ready_center)
      steering_target.center_px=*continuous_.last_ready_center;
    const auto bounds=config_.ball.ApproachBounds(
        steering_target.center_px.v,input.image_height);
    guide=ObjectZoneGuide(steering_target,bounds.first,bounds.second,
                          cfg.steering_deadband_deg,input.image_width);
  }
  result.guide=guide;
  result.guide_observed=guide.valid;
  if (i==2 && guide.valid) continuous_.hurdle_last_guide=guide;
  // HURDLE follows the actual LINE, including its bounded loss recovery.
  // Holding the router on an invalid floor guide would bypass that FSM forever.
  result.hold=(i!=2 && !guide.valid) || probe_final_edge;
  return result;
}

void MissionController::PopulateApproachDiagnostics(MissionFrameResult& output) const {
  output.approach_goal_search_failed=approach_goal_search_failed_;
  output.approach_goal_depth_failed=approach_goal_depth_failed_;
  const auto& state=recovering_approach_ ? *recovering_approach_ : continuous_;
  output.approach_mission=state.mission;
  output.approach_track_id=state.id;
  output.approach_completed_blocks=state.blocks;
  output.approach_missing_frames=state.missing;
  output.approach_locked=state.locked;
  output.approach_loss_pending=state.loss_pending;
  output.approach_probe_active=state.probe_active;
  output.approach_probe_used=state.probe_used;
  output.approach_near=state.near;
  output.approach_near_observing=state.near && state.near_observing &&
      state.near_motion.action==MissionAction::kNone && state.near_motion_id==0;
  output.approach_near_final_motion=state.near && state.near_observing &&
      (state.near_motion.action!=MissionAction::kNone || state.near_motion_id!=0);
  if (const auto evidence=line_plan_.CurrentObjectEvidence(state.epoch)) {
    output.approach_sequence_hits=evidence->hits;
    output.approach_sequence_frames=evidence->frames;
    output.approach_sequence_qualified=evidence->qualified;
  }
}
}
