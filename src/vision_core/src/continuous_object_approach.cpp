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
    backboard_target_tracker_.Reset();
    goal_controller_.ClearDepthRecovery();
    approach_goal_depth_action_id_=0;
    approach_goal_bad_depth_frames_=0;
  }
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
        ? config_.ball.far_u_des_px : config_.goal.target_u_px;
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
    if (route.near) hurdle_controller_.EnterContinuousNear(input.now_sec);
    else hurdle_controller_.EnterContinuousRecovery(recovery_target,route.good_hits,input.image_width,input.image_height,input.now_sec);
    hurdle_result_.mode=route.near ? HurdleMode::kWaitCameraDown : HurdleMode::kRecoveryStep;
    hurdle_result_.active=true;
    hurdle_target_tracker_.Lock();
  } else {
    if (route.near)
      goal_controller_.EnterContinuousNear(recovery_target,route.pose,
          lost ? route.good_hits : route.hits,route.pose_hits,route.near_hits,
          input.image_width,input.image_height,input.now_sec,lost);
    else goal_controller_.EnterContinuousApproach(recovery_target,route.pose,
          route.good_hits,route.pose_hits,route.near_hits,
          input.image_width,input.image_height,input.now_sec,true);
    goal_result_.mode=route.near ? GoalMode::kWaitFinePose : GoalMode::kApproach;
    goal_result_.active=true;
    backboard_target_tracker_.Lock();
  }
  if (!route.near) {
    recovering_approach_=route;recovering_approach_->mission_started=true;
  } else {
    recovering_approach_.reset();last_line_score_.reset();
    actual_line_stability_history_.clear();
  }
  continuous_={};approach_evidence_={};
  approach_goal_probe_=approach_return_forward_=false;
  approach_goal_empty_retries_=0;
  approach_goal_bad_depth_frames_=0;
  approach_goal_depth_action_id_=0;
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
    approach_goal_probe_=approach_return_forward_=false;
  }
  return true;
}

bool MissionController::PrepareGoalDepthRecovery(const MissionFrameInput& input,
    bool visible,bool usable,bool idle,RouteDecision& result) {
  if (visible && !usable && continuous_.probe_active) {
    // Depth loss is a separate GOAL recovery episode. The current motor ID
    // still drains normally, but an aborted probe must not remain armed.
    continuous_.probe_active=false;
  }
  if (usable || !visible) approach_goal_bad_depth_frames_=0;
  else ++approach_goal_bad_depth_frames_;
  if (!goal_controller_.DepthRecoveryActive()) {
    if (!visible || usable) return false;
    result.hold=true;
    line_plan_.Abort();
    if (approach_goal_bad_depth_frames_<config_.goal.lost_frames) return true;
    if (!idle) return true;
    goal_controller_.BeginDepthRecovery(false);
  }
  result.hold=true;
  const bool done=approach_goal_depth_action_id_ && input.delivery_feedback.done &&
      input.delivery_feedback.action_id==approach_goal_depth_action_id_;
  const bool can_issue=idle && input.allow_new_commands && input.allow_new_body_actions &&
      input.allow_new_line_action && !command_coordinator_.ActionAckTimedOut() &&
      input.now_sec+1e-9>=settle_until_sec_;
  if (foot_pose_!=FootPose::kWalk && !approach_goal_depth_action_id_) {
    if (can_issue) result.action={MissionAction::kWalkMode,ActionCategory::kLocomotion,0,false};
    return true;
  }
  const auto recovery=goal_controller_.StepDepthRecovery(visible,usable,input.now_sec,
      {done,command_coordinator_.HasPendingAction() && !done,false},can_issue);
  if (done) approach_goal_depth_action_id_=0;
  if (recovery.failed) {
    line_plan_.Abort();line_state_=LineState::kFinalHold;approach_goal_depth_failed_=true;
  } else if (recovery.missing && continuous_.mission==MissionType::kGoal) {
    if (continuous_.locked) {
      EnterContinuousMission(input,true);
      result.entered=true;
    } else {
      ReleaseContinuousApproach();
      approach_return_forward_=true;
      post_pickup_goal_line_gate_active_=true;
      approach_goal_retry_after_=post_pickup_line_actions_completed_+1;
      if (idle && input.allow_new_commands) result.camera=CameraRequest::kForward;
    }
  } else if (recovery.recovered) {
    approach_goal_bad_depth_frames_=0;
    continuous_.probe_used=false;
    line_plan_.Reset();line_auxiliary_id_=0;
    line_auxiliary_done_=line_auxiliary_observing_=false;
    line_stability_history_.clear();
    actual_line_stability_history_.clear();
  } else if (can_issue && recovery.action.action!=MissionAction::kNone) {
    result.action=recovery.action;result.depth_recovery_step=true;
  }
  return true;
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
  if (approach_return_forward_) {
    if (!settled || camera.actual_mode!=CameraMode::kForward) {
      camera_request(CameraRequest::kForward);return result;
    }
    approach_return_forward_=false;approach_camera_started_=-1;
    line_stability_history_.clear();
    actual_line_stability_history_.clear();
  }
  // Preserve the carrying/post-pickup gate. Camera transitions still drain all
  // issued locomotion before requesting GOAL, even in continuous mode.
  const bool goal_gate=has_ball_ && config_.enable_goal && post_pickup_goal_line_gate_active_ &&
      (post_pickup_five_step_completed_ || post_pickup_line_actions_completed_>=goal_controller_.RequiredLineWalkActions()) &&
      post_pickup_line_actions_completed_>=approach_goal_retry_after_;
  if (advance && settled && camera.actual_mode==CameraMode::kForward && goal_gate &&
      continuous_.mission==MissionType::kNone && line_state_==LineState::kNormal) {
    approach_goal_probe_=true;line_plan_.Abort();
  }
  if (approach_goal_probe_) {
    if (!settled || !goal_view) { camera_request(CameraRequest::kGoal);return result; }
    approach_goal_probe_=false;approach_camera_started_=-1;
    approach_goal_empty_frames_=0;approach_evidence_={};
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
    goal_guide=ObjectApproachGuide(*input.backboard_target,false,0,input.intrinsics,{},
        input.image_width,input.image_height,last_line_image_center_u_,
        line.guide.valid ? line.guide_reference_v_px : cfg.reference_v_ratio*input.image_height,
        input.rectify_approach,input.approach_roll_rad,input.approach_pitch_rad);
  }
  const std::array<MissionType,3> missions{MissionType::kBall,MissionType::kGoal,MissionType::kHurdle};
  const std::array<std::optional<ObjectTarget>,3> targets{input.ball_target,input.backboard_target,input.hurdle_target};
  const std::array<std::uint64_t,3> ids{input.ball_track_id,input.backboard_track_id,input.hurdle_track_id};
  const std::array<int,3> windows{config_.ball.stable_window,config_.goal.stable_window,config_.hurdle.stable_window};
  const std::array<int,3> entry_hits{config_.ball.entry_min_hits,config_.goal.stable_min_hits,config_.hurdle.stable_min_hits};
  const std::array<bool,3> eligible{
      !has_ball_ && config_.enable_ball && camera.actual_mode==CameraMode::kForward,
      has_ball_ && config_.enable_goal && goal_view,
      !has_ball_ && config_.enable_hurdle && camera.actual_mode==CameraMode::kForward &&
          !hurdle_controller_.IgnoringUntil(input.now_sec)};
  for (std::size_t i=0;i<3;++i) {
    auto& evidence=approach_evidence_[i];
    bool visible=eligible[i] && targets[i].has_value();
    if (visible && i==0) visible=targets[i]->center_px.v<=config_.ball.upper_acquire_v_px;
    if (visible && i==2) visible=targets[i]->center_px.v<=config_.hurdle.upper_acquire_v_px;
    if (visible && evidence.last && ((ids[i] && evidence.id && ids[i]!=evidence.id) ||
        !SamePosition(*evidence.last,*targets[i],config_.object_association.max_center_distance_norm*
            std::hypot(input.image_width,input.image_height),config_.object_association.max_size_log_ratio))) evidence={};
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
      acquired=true;
      if (i==1) approach_goal_empty_retries_=0;
      break;
    }
  }
  if (continuous_.mission==MissionType::kNone) {
    if (goal_view) {
      if (PrepareGoalDepthRecovery(input, input.backboard_target.has_value() ||
          input.backboard_image_without_depth, goal_depth_usable && goal_guide.valid,
          idle, result)) return result;
      result.hold=true;
      if (input.backboard_target || input.backboard_image_without_depth) approach_goal_empty_frames_=0;
      else if (++approach_goal_empty_frames_>=config_.goal.lost_frames) {
        if (approach_goal_empty_retries_>=cfg.goal_empty_max_retries) {
          // Terminal failure: keep draining issued IDs; only explicit Reset
          // may restart this search. No further head or walking requests.
          line_plan_.Abort();line_state_=LineState::kFinalHold;
          approach_goal_search_failed_=true;
          return result;
        }
        ++approach_goal_empty_retries_;
        approach_return_forward_=true;
        post_pickup_goal_line_gate_active_=true;
        approach_goal_retry_after_=post_pickup_line_actions_completed_+1;
        line_plan_.Abort();camera_request(CameraRequest::kForward);
      }
    }
    return result;
  }
  const std::size_t i=continuous_.mission==MissionType::kBall ? 0 :
                      continuous_.mission==MissionType::kGoal ? 1 : 2;
  auto target=targets[i];
  if (target && continuous_.last && ((ids[i] && continuous_.id && ids[i]!=continuous_.id) ||
      !SamePosition(*continuous_.last,*target,config_.object_association.max_center_distance_norm*
          std::hypot(input.image_width,input.image_height),config_.object_association.max_size_log_ratio))) target.reset();
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
      line_plan_.Abort(); // Issued IDs drain; only unissued probe repeats stop.
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
  const bool lost=continuous_.loss_pending;
  if (lost && !continuous_.locked && !continuous_.near) {
    line_plan_.Abort();result.hold=true;
    if (!idle) return result;
    const bool probe_available=!continuous_.probe_used &&
        cfg.prelock_loss_extra_blocks>0 &&
        continuous_.last_ready_center.has_value() && continuous_.last.has_value();
    if (probe_available) {
      if (!input.allow_new_commands || !input.allow_new_body_actions ||
          !input.allow_new_line_action || !input.allow_new_line_plan ||
          command_coordinator_.ActionAckTimedOut() ||
          input.now_sec+1e-9<settle_until_sec_) return result;
      auto averaged_target=*continuous_.last;
      averaged_target.center_px=*continuous_.last_ready_center;
      const auto probe_guide=ObjectApproachGuide(averaged_target,false,0,
          input.intrinsics,ApproachCameraPose{},input.image_width,input.image_height,
          last_line_image_center_u_,line.guide.valid ?
              line.guide_reference_v_px : cfg.reference_v_ratio*input.image_height,
          input.rectify_approach,input.approach_roll_rad,input.approach_pitch_rad);
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
      approach_return_forward_=true;
      post_pickup_goal_line_gate_active_=true;
      approach_goal_retry_after_=post_pickup_line_actions_completed_+1;
      camera_request(CameraRequest::kForward);
    }
    return result;
  }
  if (continuous_.near || (lost && continuous_.locked)) {
    line_plan_.Abort();result.hold=true;
    if (!continuous_.near) {
      if (idle && input.allow_new_commands && input.allow_new_body_actions) {
        EnterContinuousMission(input,true);result.entered=true;
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
      const int limit=i==0 ? config_.ball.recovery_max_observations :
          i==1 ? config_.goal.recovery_max_observations : config_.hurdle.recovery_max_observations;
      if (continuous_.near_attempts>=limit) {
        if (input.allow_new_commands && input.allow_new_body_actions) {
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
      const double u=continuous_.last->center_px.u;
      const bool left=u<bounds.first, right=u>bounds.second;
      if (left || right) {
        const auto turn=left ? MissionAction::kTurnLeftInPlace :
                               MissionAction::kStepForwardRightOneHurdle;
        const double excess=left ? bounds.first-u : u-bounds.second;
        continuous_.near_actions.push_back(turn);
        if (excess>=config_.ball.approach_extreme_turn_margin_px)
          continuous_.near_actions.push_back(turn);
        continuous_.near_actions.push_back(MissionAction::kStepForwardOne);
      } else {
        continuous_.near_actions.push_back(
            v<config_.ball.approach_direct_down_v_px ?
                MissionAction::kStepForwardTwo : MissionAction::kStepForwardOne);
      }
    } else {
      continuous_.near_actions.push_back(MissionAction::kStepForwardTwo);
    }
    const auto first=continuous_.near_actions.front();
    continuous_.near_actions.pop_front();
    continuous_.near_motion={first,ActionCategory::kLocomotion,0,false};
    result.action=continuous_.near_motion;result.near_final_action=true;
    return result;
  }
  // Near commitment and soft-loss release take precedence over coordinate
  // recovery. A visible near board must never issue another WALK search step.
  if (i == 1 && PrepareGoalDepthRecovery(input, target.has_value() ||
      input.backboard_image_without_depth,
      target && goal_depth_usable && goal_guide.valid, idle, result)) return result;
  const bool probe_final_edge=continuous_.probe_active &&
      line_plan_.LastUnit(input.delivery_feedback.action_id) &&
      (input.delivery_feedback.ready || input.delivery_feedback.done);
  if (!target) {
    // Missing frames inside a walking block retain the last path. Only that
    // block's READY evidence may declare loss and stop its successor.
    LineGuide saved_guide;
    if (i!=2 && continuous_.last_ready_center && continuous_.last) {
      auto averaged_target=*continuous_.last;
      averaged_target.center_px=*continuous_.last_ready_center;
      saved_guide=ObjectApproachGuide(averaged_target,false,0,input.intrinsics,
          ApproachCameraPose{},input.image_width,input.image_height,
          last_line_image_center_u_,line.guide.valid ?
              line.guide_reference_v_px : cfg.reference_v_ratio*input.image_height,
          input.rectify_approach,input.approach_roll_rad,input.approach_pitch_rad);
    }
    result.guide=saved_guide.valid ? saved_guide : (i==2 ? line.guide : LineGuide{});
    result.hold=!result.guide->valid || probe_final_edge;
    return result;
  }
  LineGuide guide=line.guide;
  if (i==1) {
    result.display_guide=goal_guide;
    guide=goal_guide;
  } else if (i==0) {
    const double reference_u=last_line_image_center_u_;
    const double reference_v=line.guide.valid ? line.guide_reference_v_px :
        cfg.reference_v_ratio*input.image_height;
    guide=ObjectApproachGuide(*target,false,0,
        input.intrinsics,ApproachCameraPose{},
        input.image_width,input.image_height,reference_u,reference_v,
        input.rectify_approach,input.approach_roll_rad,input.approach_pitch_rad);
  }
  result.guide=guide;
  result.guide_observed=guide.valid;
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
