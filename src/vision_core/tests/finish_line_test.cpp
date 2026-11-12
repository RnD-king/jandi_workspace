// Seed completed course stages without running unrelated BALL/GOAL missions.
// Dependencies are included first; only MissionController access is exposed.
#include <algorithm>
#include <cstddef>
#include <deque>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>
#include "vision_core/ball_controller.hpp"
#include "vision_core/ball_candidate_tracker.hpp"
#include "vision_core/control_command.hpp"
#include "vision_core/goal_controller.hpp"
#include "vision_core/hurdle_controller.hpp"
#include "vision_core/line_detection_extractor.hpp"
#include "vision_core/line_feature_extractor.hpp"
#include "vision_core/line_p2p_controller.hpp"
#include "vision_core/line_plan.hpp"
#include "vision_core/object_approach.hpp"
#include <array>
#include "vision_core/object_target_extractor.hpp"
#include "vision_core/object_association_tracker.hpp"
#include "vision_core/object_target_tracker.hpp"
#define private public
#include "vision_core/mission_controller.hpp"
#undef private
#include "vision_core/config_loader.hpp"
#include <stdexcept>
#include <limits>

using namespace vision_core;
using LineState = MissionController::LineState;
namespace {
void Check(bool value,const char* message) { if(!value) throw std::runtime_error(message); }
MissionControllerConfig Config() {
  auto c=LoadDefaultAlgorithmConfig();
  c.enable_ball=c.enable_goal=false;c.enable_hurdle=true;
  c.initial_has_ball=c.goal_trial_start_goal=false;c.test_mission=MissionType::kNone;
  c.curve_search.enabled=false;
  c.line_features.image_center_u=c.hurdle.target_u_px=320;
  c.line_p2p.offset_gain=1;c.line_p2p.heading_gain=0;
  c.line_p2p.finish_offset_gain=1;c.line_p2p.finish_heading_gain=0;
  c.line_p2p.steering_deadband=.1;c.line_p2p.in_place_turn_score=1;
  c.line_p2p.left_repetitions=c.line_p2p.right_repetitions=c.line_p2p.straight_repetitions=2;
  c.transition_settle_sec=0;c.hurdle.contact_steps=1;
  c.hurdle.contact_inter_action_hold_sec=0;
  return c;
}
ObjectTarget Finish(double u=450) {
  ObjectTarget t;t.center_px={u,400};t.box_px={u-50,390,100,20};
  t.width_px=100;t.height_px=20;t.area_px=2000;t.confidence=.95;return t;
}
struct Driver {
  MissionController core;
  MissionFrameInput in;
  MissionFrameResult out;
  explicit Driver(int crosses=2,int index=5,MissionControllerConfig config=Config()):core(config) {
    core.hurdle_crosses_completed_=crosses;core.course_curve_index_=index;
    in.image_width=640;in.image_height=480;in.command_transport_enabled=true;
    in.camera_feedback={CameraMode::kForward,true};
  }
  void Tick(CommandDeliveryFeedback feedback={},double dt=.1) {
    in.now_sec+=dt;in.delivery_feedback=feedback;out=core.Step(in);
    Check(out.command.action!=MissionAction::kHurdle,"finish emitted a crossing action");
    if(core.FinishCourseEnabled(in) && in.camera_feedback.settled &&
        in.camera_feedback.actual_mode==CameraMode::kForward)
      Check(out.command.camera_request==CameraRequest::kNone,"finish started a recovery head scan");
  }
  void Floor(double u,int count=5) {
    in.line_centers.clear();
    for(int i=0;i<count;++i)in.line_centers.push_back({u,450.0-40*i});
  }
};
void FinishGains() {
  auto config=Config();
  config.line_p2p.offset_gain=.2;config.line_p2p.heading_gain=.4;
  config.line_p2p.finish_offset_gain=.3;config.line_p2p.finish_heading_gain=.6;
  config.line_p2p.steering_deadband=.08;
  LineGuide guide;guide.valid=true;guide.offset=-.3;
  Driver finish(2,5,config);
  Check(finish.core.SteeringDecision(guide).direction==CruiseDirection::kStraight,
        "finish gains leaked into ordinary steering");
  finish.Floor(224);finish.Tick();
  Check(finish.out.command.action==MissionAction::kLineLeftUnit,
        "finish did not correct an offset inside the ordinary deadband");
  config.line_p2p.finish_offset_gain=.2;config.line_p2p.finish_heading_gain=.4;
  Driver baseline(2,5,config);baseline.Floor(224);baseline.Tick();
  Check(baseline.out.command.action==MissionAction::kLineStraightUnit,
        "finish ignored its separately configured gains");
  guide.offset=0;guide.heading_rad=.15;
  Check(finish.core.SteeringDecision(guide,true).direction==CruiseDirection::kRight &&
        finish.core.SteeringDecision(guide).direction==CruiseDirection::kStraight,
        "finish heading gain was not applied independently");
}
void PriorityAndGates() {
  Driver floor;floor.Floor(200);floor.in.hurdle_target=Finish(450);floor.Tick();
  Check(floor.out.command.action==MissionAction::kLineLeftUnit,"valid LINE did not win over finish bbox");
  for(int points:{0,1,2}) {
    Driver bbox;bbox.Floor(200,points);bbox.in.hurdle_target=Finish(450);bbox.Tick();
    Check(bbox.out.command.action==MissionAction::kLineStraightUnit,
          "sparse LINE steered toward an object instead of going straight");
    Check(bbox.out.line_decision_guide && !bbox.out.line_decision_guide->valid,
          "finish synthesized a floor guide from an object");
    Check(bbox.out.active_mission==MissionType::kLine && bbox.out.approach_mission==MissionType::kNone,
          "finish entered HURDLE/FINE instead of retaining LINE transport");
  }
  Driver three;three.Floor(200,3);three.in.hurdle_target=Finish(450);three.Tick();
  Check(three.out.line_decision_guide && three.out.line_decision_guide->valid &&
        three.out.command.action==MissionAction::kLineLeftUnit,
        "three valid floor points must steer with O/H in FINISH mode");
  Driver missing;missing.Tick();
  Check(missing.out.command.action==MissionAction::kLineStraightUnit,"missing LINE and bbox did not go straight");
  Driver outlier;outlier.Floor(200,4);outlier.in.line_centers.back().u=600;
  outlier.core.config_.line_features.ransac_enabled=true;
  outlier.core.config_.line_features.ransac_min_input_points=4;
  outlier.core.config_.line_features.ransac_residual_threshold_px=20;
  outlier.core.config_.line_features.ransac_min_sample_v_gap_px=20;
  outlier.in.hurdle_target=Finish(450);outlier.Tick();
  Check(outlier.out.line_features.n_visible==3 &&
        outlier.out.command.action==MissionAction::kLineLeftUnit,
        "three RANSAC inliers did not steer from floor O/H");
  Driver bad;bad.in.hurdle_target=Finish(std::numeric_limits<double>::quiet_NaN());bad.Tick();
  Check(bad.out.command.action==MissionAction::kLineStraightUnit,"invalid finish center was used");
  Driver score;score.core.config_.line_p2p.in_place_turn_score=.01;
  // Recreate the steering controller as a real matching configuration would.
  score.core.line_p2p_controller_=LineP2pController(score.core.config_.line_p2p);
  score.Floor(600);score.Tick();
  Check(score.out.command.action==MissionAction::kStepForwardRightOneHurdle,
        "finish did not reuse ordinary right score recovery");
  const auto turn=score.out.command.action_id;
  score.Tick({turn,true,false,false});
  score.Tick({turn,true,true,false});
  Check(score.core.line_guard_turns_completed_==1 && score.out.post_motion_observing,
        "finish turn lost its DONE count or fresh observation");
  score.Floor(600,2);score.Tick();
  Check(score.out.command.action==MissionAction::kLineStraightUnit && !score.out.post_motion_observing,
        "sparse finish evidence did not leave score observation for straight walking");
  Driver sparse_done;sparse_done.core.config_.line_p2p.in_place_turn_score=.01;
  sparse_done.Floor(600);sparse_done.Tick();const auto sparse_turn=sparse_done.out.command.action_id;
  sparse_done.Tick({sparse_turn,true,false,false});sparse_done.Floor(600,2);
  sparse_done.Tick({sparse_turn,true,true,false});
  Check(sparse_done.core.line_guard_turns_completed_==1 &&
        sparse_done.out.command.action==MissionAction::kLineStraightUnit,
        "sparse finish evidence erased an actually completed turn");
  Driver exhausted;exhausted.Floor(600);
  exhausted.core.line_guard_turns_completed_=exhausted.core.config_.line_p2p.recovery_max_turns;
  exhausted.core.config_.line_p2p.in_place_turn_score=.01;
  exhausted.Tick();
  Check(exhausted.out.command.action==MissionAction::kLineStraightUnit &&
        exhausted.core.finish_line_straight_only_,"finish turn exhaustion did not latch straight");
  Driver flat;flat.in.line_centers={{100,300},{200,302},{300,304}};flat.Tick();
  Check(flat.out.command.action==MissionAction::kLineStraightUnit,
        "invalid three-point fit did not use finish straight fallback");
  for(const auto stage:std::vector<std::pair<int,int>>{{0,5},{1,5},{2,1},{2,3},{2,4}}) {
    Driver normal(stage.first,stage.second);normal.Floor(200,3);normal.Tick();
    Check(!normal.core.FinishCourseEnabled(normal.in),"partial finish gate was accepted");
    Check(normal.out.command.action_id==0 && normal.out.line_in_recovery,
          "ordinary LINE loss became blind forward walking");
  }
  Driver manual;manual.in.manual_observation_control=true;manual.Tick();
  Check(!manual.core.FinishCourseEnabled(manual.in) && manual.out.command.action_id==0,
        "finish exception leaked into manual mode");
  Driver isolated;isolated.core.config_.test_mission=MissionType::kHurdle;
  Check(!isolated.core.FinishCourseEnabled(isolated.in),"finish exception leaked into standalone HURDLE");
  Driver camera;camera.in.camera_feedback={CameraMode::kTransition,false};camera.Tick();
  Check(camera.out.command.action_id==0,"finish moved during camera transition");
  Driver settled_down;settled_down.in.camera_feedback={CameraMode::kDown,true};settled_down.Tick();
  Check(settled_down.out.command.action_id==0,"finish used a DOWN camera observation");
}
void ActivateRealCooldown(Driver& d) {
  auto& h=d.core.hurdle_controller_;
  h.StartTestExecute();
  const auto target=Finish(320);
  const std::deque<std::optional<double>> raw_v(10,target.center_px.v);
  h.UseVerifiedObservation(target,std::deque<bool>(10,true),640,480,&raw_v);
  const CameraFeedback down{CameraMode::kDown,true};
  auto r=h.Compute(target,640,480,.1,down,{},{});
  Check(r.mode==HurdleMode::kContactWalk,"cooldown setup did not enter contact walk");
  ActionExecutionFeedback done;done.action_done=true;
  r=h.Compute(target,640,480,.2,down,done,{});
  Check(r.action.action==MissionAction::kHurdle,"cooldown setup did not cross");
  h.Compute(target,640,480,.3,down,done,{});
  h.Compute({},640,480,.4,{CameraMode::kForward,true},{},{});
  Check(h.IgnoringUntil(.5),"cooldown fixture did not start real post-return ignore timer");
  d.in.now_sec=.4;
}
void CooldownScope() {
  Driver second;ActivateRealCooldown(second);second.in.hurdle_target=Finish(450);second.Tick();
  Check(second.out.command.action==MissionAction::kLineStraightUnit,
        "finish object affected steering during second crossing cooldown");
  Driver first(1,5);ActivateRealCooldown(first);first.Floor(320);first.in.hurdle_target=Finish(450);
  for(int i=0;i<15;++i)first.Tick();
  Check(first.core.hurdle_controller_.IgnoringUntil(first.in.now_sec),"first cooldown ended too soon");
  Check(first.out.approach_mission==MissionType::kNone,"first crossing lost its cooldown protection");
}
void AllObjectsAreIgnored() {
  for (bool carrying : {false,true}) for (int points : {0,3,5}) {
    Driver baseline,visible;
    for (auto* d : {&baseline,&visible}) {
      d->core.config_.enable_ball=d->core.config_.enable_goal=true;
      d->core.config_.curve_search.enabled=true;
      d->core.has_ball_=carrying;
      d->core.post_pickup_goal_line_gate_active_=true;
      d->core.post_pickup_line_actions_completed_=100;
      d->core.post_pickup_straight_blocks_completed_=100;
      d->core.post_pickup_five_step_completed_=true;
      d->Floor(200,points);
    }
    const auto tick=[&](CommandDeliveryFeedback feedback=CommandDeliveryFeedback{}) {
      // Qualifying objects move across the image, including a clipped board
      // and missing Depth. None may affect finish steering or cause a camera
      // change, object admission, recovery search, or observation hold.
      const double u=visible.in.now_sec<1 ? 10 : visible.in.now_sec<2 ? 630 : 320;
      visible.in.ball_target=Finish(u);
      visible.in.hurdle_target=Finish(u);
      visible.in.goal_target=Finish(u);
      visible.in.backboard_target=Finish(u);
      visible.in.raw_backboard_target=Finish(u);
      visible.in.raw_backboard_observation_available=true;
      visible.in.raw_backboard_u=u;
      visible.in.backboard_image_without_depth=true;
      visible.in.ball_entry_detected=true;
      visible.in.ball_entry_track_hits=10;
      visible.in.ball_entry_track_frames=10;
      visible.in.ball_entry_center_u=u;
      visible.in.ball_entry_mean_center=Point2{u,400};
      visible.in.hurdle_entry_track_hits=10;
      visible.in.backboard_entry_track_hits=10;
      visible.in.scan_ball_u=u;
      visible.in.scan_backboard_u=u;
      baseline.Tick(feedback);visible.Tick(feedback);
      const auto& a=baseline.out.command;
      const auto& b=visible.out.command;
      Check(a.action==b.action && a.action_id==b.action_id &&
            a.command_type==b.command_type && a.camera_request==b.camera_request &&
            a.target_yaw_deg==b.target_yaw_deg,
            "objects changed finish commands or camera requests");
      Check(visible.out.active_mission==MissionType::kLine &&
            visible.out.approach_mission==MissionType::kNone &&
            visible.out.candidate_mission==MissionType::kNone &&
            visible.out.candidate_phase==0 && !visible.out.post_motion_observing,
            "finish objects started an object decision or observation");
    };
    tick();
    for (int block=0;block<3;++block) {
      const auto a=baseline.out.command.action_id;
      Check(a!=0,"finish test did not start a walk");
      tick({a,true,false,false});
      for (int frame=0;frame<12;++frame) tick();
      tick({a,true,false,true});
      const auto b=baseline.out.command.action_id;
      Check(b!=0 && b!=a,"finish objects blocked the READY successor");
      tick({b,true,false,false});
      tick({a,true,true,false});
    }
  }
}
void ReadyDonePauseReset() {
  Driver d;d.Tick();const auto a=d.out.command.action_id;
  Check(a!=0,"finish did not issue initial walk");
  d.Tick({a,true,false,false});
  for(int i=0;i<12;++i)d.Tick();
  Check(d.out.command.action_id==a && !d.core.command_coordinator_.HasQueuedAction(),
        "finish issued successor before READY");
  d.Tick({a,true,false,true});const auto b=d.out.command.action_id;
  Check(b!=0 && b!=a && d.out.command.action==MissionAction::kLineStraightUnit,
        "finish READY failed to reserve one successor");
  d.Tick({b,true,false,false});
  Check(!d.core.line_plan_.Done(a),"READY/queued ACK fabricated predecessor DONE");
  d.Tick({a,true,true,false});
  Check(d.core.line_plan_.Done(a) && !d.core.line_plan_.Done(b),"physical DONE ownership was lost");
  for(int i=0;i<12;++i)d.Tick();
  d.Tick({b,true,false,true});const auto c=d.out.command.action_id;
  Check(c && c!=b && d.out.command.action==MissionAction::kLineStraightUnit,
        "empty finish block started loss recovery at final READY");
  d.Tick({c,true,false,false});d.Tick({b,true,true,false});
  d.Tick({b,true,true,false});
  Check(!d.core.line_plan_.Done(c),"duplicate DONE completed a new finish walk");
  d.in.allow_new_commands=d.in.allow_new_body_actions=d.in.allow_new_line_action=false;
  d.in.allow_new_line_plan=d.in.allow_new_mission_entry=d.in.advance_line_fsm=false;
  d.Tick({c,true,false,true});
  Check(!d.core.command_coordinator_.HasQueuedAction(),"PAUSE allowed a finish successor");
  d.Tick({c,true,true,false});
  Check(d.core.line_plan_.Done(c),"PAUSE discarded actual DONE");
  d.in.allow_new_commands=d.in.allow_new_body_actions=d.in.allow_new_line_action=true;
  d.in.allow_new_line_plan=d.in.allow_new_mission_entry=d.in.advance_line_fsm=true;
  d.Tick();
  Check(d.out.command.action==MissionAction::kLineStraightUnit && d.out.command.action_id>c,
        "RESUME did not continue the existing finish walk plan");
  d.core.Reset(false);d.in.delivery_feedback={};d.Tick();
  Check(d.core.hurdle_crosses_completed_==0 && d.out.course_curve_index==1 &&
        d.out.command.action_id==0,"RESET retained finish-only forward policy");
}
void SingleSparseRecoveryAndContinuousStraight() {
  for(int points : {0,1,2}) for(double offset : {-.4,0.,.4}) {
    auto cfg=Config();
    cfg.line_p2p.left_repetitions=cfg.line_p2p.right_repetitions=cfg.line_p2p.straight_repetitions=3;
    Driver d(2,5,cfg);
    LineGuide memory;memory.valid=true;memory.offset=offset;
    d.core.finish_last_stable_guide_=memory;
    d.Floor(320,points);d.Tick();
    const auto recovery=d.out.command.action_id;
    const auto expected=offset<0 ? MissionAction::kLineLeftUnit : offset>0 ?
        static_cast<MissionAction>(cfg.line_p2p.right_unit_action) : MissionAction::kLineStraightUnit;
    Check(recovery && d.out.command.action==expected && d.core.finish_loss_used_,
          "sparse FINISH did not use its last stable direction once");
    Check(!d.core.line_plan_.Active(),"loss walk retained unissued repetitions");
    d.Tick({recovery,true,false,false});
    d.in.allow_new_commands=d.in.allow_new_body_actions=false;
    d.Tick({recovery,true,false,true});
    Check(!d.core.command_coordinator_.HasQueuedAction(),"PAUSE booked a FINISH successor");
    d.in.allow_new_commands=d.in.allow_new_body_actions=true;
    d.Tick({recovery,true,false,true});
    const auto straight=d.out.command.action_id;
    Check(straight && straight!=recovery && d.out.command.action==MissionAction::kLineStraightUnit &&
          d.core.command_coordinator_.HasQueuedAction(),"recovery READY did not reserve straight continuation");
    Check(!d.core.line_plan_.Done(recovery) && !d.out.post_motion_observing,
          "READY finished recovery or added a stationary observation");
    d.Tick({straight,true,false,false});
    d.Tick({recovery,true,false,true});
    Check(d.core.command_coordinator_.QueuedActionId()==straight,"duplicate READY replaced successor");
    d.Tick({recovery,true,true,false});
    Check(d.core.line_plan_.Done(recovery) && !d.core.line_plan_.Done(straight) &&
          !d.out.post_motion_observing,"real DONE lost the straight successor");
    for(int i=0;i<12;++i) {
      d.Tick();Check(d.core.finish_loss_used_,"persistent loss rearmed another recovery");
    }
    d.Tick({straight,true,false,true});
    Check(d.out.command.action==MissionAction::kLineStraightUnit && d.out.command.action_id!=recovery,
          "persistent sparse FINISH repeated the correction instead of continuing straight");
    d.core.Reset(true);
    Check(!d.core.finish_last_stable_guide_ && !d.core.finish_loss_used_ && !d.core.finish_loss_action_id_,
          "RESET retained sparse FINISH memory or action ownership");
  }
  // Reacquisition must use a complete fresh stability window before another loss.
  Driver fresh;fresh.Floor(200);
  fresh.Tick();const auto original=fresh.out.command.action_id;
  fresh.Tick({original,true,false,false});
  for(int i=0;i<12;++i)fresh.Tick();
  Check(fresh.core.finish_last_stable_guide_.has_value(),"stable floor did not build FINISH memory");
  fresh.Floor(320,0);fresh.Tick({original,true,false,true});
  Check(fresh.core.finish_loss_used_ && fresh.out.command.action==MissionAction::kLineLeftUnit,
        "real pre-loss O/H did not select recovery at READY");
}
void SparseRecoveryResumesRealThreePointSteering() {
  Driver d;
  LineGuide memory;memory.valid=true;memory.offset=-.4;
  d.core.finish_last_stable_guide_=memory;
  d.Floor(100,2);d.Tick();
  const auto recovery=d.out.command.action_id;
  Check(recovery && d.out.command.action==MissionAction::kLineLeftUnit,
        "FINISH did not begin its remembered-left recovery");
  d.Tick({recovery,true,false,false});
  // Three valid floor LINE points reappear while the one-shot unit is active.
  // Its READY successor must use fresh real steering instead of blind straight.
  d.Floor(600,3);d.Tick({recovery,true,false,true});
  Check(d.out.command.action==
        static_cast<MissionAction>(d.core.config_.line_p2p.right_unit_action) &&
        d.out.command.action_id!=recovery &&
        d.core.command_coordinator_.HasQueuedAction(),
        "FINISH recovery READY ignored newly valid three-point floor LINE");
}
}
int main() { SparseRecoveryResumesRealThreePointSteering();SingleSparseRecoveryAndContinuousStraight();FinishGains();PriorityAndGates();CooldownScope();AllObjectsAreIgnored();ReadyDonePauseReset(); }
