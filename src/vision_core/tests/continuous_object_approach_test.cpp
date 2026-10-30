#include "vision_core/config_loader.hpp"
#include "vision_core/mission_controller.hpp"
#include <cmath>
#include <stdexcept>
#include <limits>

using namespace vision_core;
namespace {
void Check(bool value,const char* message) { if(!value) throw std::runtime_error(message); }
ObjectTarget Target(double u=320,double v=100,int cls=1) {
  ObjectTarget t;t.class_id=cls;t.confidence=.95;t.center_px={u,v};
  t.box_px={u-20,v-20,40,40};t.width_px=t.height_px=40;t.area_px=1600;return t;
}
MissionControllerConfig Config() {
  auto c=LoadDefaultAlgorithmConfig();
  c.enable_ball=true;c.enable_goal=c.enable_hurdle=false;
  c.initial_has_ball=c.goal_trial_start_goal=false;
  c.test_mission=MissionType::kNone;
  c.line_features.image_center_u=320;
  c.line_p2p.left_repetitions=c.line_p2p.straight_repetitions=c.line_p2p.right_repetitions=2;
  c.ball.entry_min_hits=4;c.ball.stable_window=10;c.ball.stable_min_hits=7;
  c.ball.approach_short_v_px=100;c.ball.approach_two_step_v_px=240;c.ball.approach_direct_down_v_px=355;
  c.hurdle.approach_short_v_px=100;c.hurdle.approach_two_step_v_px=240;
  c.object_association.max_center_distance_norm=.5;
  c.object_approach.lock_blocks=3;
  c.object_approach.forward_camera={0,.45,0,20,0};
  c.object_approach.goal_camera={0,.45,0,0,0};
  return c;
}
struct Driver {
  MissionController core;
  MissionFrameInput in;
  MissionFrameResult out;
  std::uint64_t current{}, last_done{};
  explicit Driver(const MissionControllerConfig& c):core(c) {
    in.image_width=640;in.image_height=480;
    in.camera_feedback={CameraMode::kForward,true};in.command_transport_enabled=true;
    in.line_centers={{320,450},{320,400},{320,350},{320,300},{320,250}};
    in.ball_target=Target();in.ball_track_id=41;
  }
  void Tick(CommandDeliveryFeedback fb={},double dt=.05) {
    in.now_sec+=dt;in.delivery_feedback=fb;out=core.Step(in);
  }
  void Warm() {
    Tick();current=out.command.action_id;
    Check(current!=0,"initial LINE unit missing");
    Tick({current,true,false,false});
    for(int i=0;i<12;++i) Tick();
  }
  void Block() {
    for(int n=0;n<2;++n) {
      Tick({current,true,false,false});
      for(int i=0;i<12;++i) Tick();
      Tick({current,true,false,true});
      const auto next=out.command.action_id;
      Check(next && next!=current && LinePlan::IsUnit(out.command.action),
            "continuous READY did not reserve the next shared unit");
      Tick({next,true,false,false});
      last_done=current;
      Tick({current,true,true,false});current=next;
    }
  }
  void MissingBlock() {
    in.ball_target.reset();
    for(int i=0;i<12;++i) Tick();
    Check(out.approach_mission==MissionType::kBall,
          "one missing frame released an object before block READY");
    Tick({current,true,false,true});
    const auto second=out.command.action_id;
    Check(second && second!=current && LinePlan::IsUnit(out.command.action),
          "missing first unit incorrectly cancelled the rest of its 2-unit block");
    Tick({second,true,false,false});
    Tick({current,true,true,false});current=second;
    for(int i=0;i<12;++i) Tick();
    Tick({current,true,false,true});
    Check(out.command.action_id==current && !out.approach_sequence_qualified,
          "unqualified object block queued a successor at final READY");
    Tick({current,true,true,false});
  }
};
void Geometry() {
  const Intrinsics k{600,600,320,240};const ApproachCameraPose c{0,.45,0,20,0};
  auto t=Target(320,150);
  auto guide=ObjectApproachGuide(t,false,0,k,c,640,480,320,400,false,0,0);
  Check(guide.valid && std::abs(guide.offset)<1e-9 && std::abs(guide.heading_rad)<1e-9,
        "centered floor path should be straight");
  t=Target(430,150);
  auto right=ObjectApproachGuide(t,false,0,k,c,640,480,320,400,false,0,0);
  t=Target(210,150);
  auto left=ObjectApproachGuide(t,false,0,k,c,640,480,320,400,false,0,0);
  Check(right.valid && left.valid && right.offset>0 && right.heading_rad>0 &&
        left.offset<0 && left.heading_rad<0,"virtual path O/H signs differ from LINE");
  auto shifted=c;shifted.right_m=.1;
  const auto offset=ObjectApproachGuide(Target(320,150),false,0,k,shifted,640,480,320,400,false,0,0);
  Check(offset.valid && std::abs(offset.offset-guide.offset)<1e-9,
        "BALL RGB path unexpectedly depended on unmeasured camera offset");
  Check(!ObjectApproachGuide(Target(),true,0,k,c,640,480,320,400,false,0,0).valid,
        "missing board depth generated a valid path");
  auto bad=k;bad.fx=0;
  Check(ObjectApproachGuide(Target(),false,0,bad,c,640,480,320,400,false,0,0).valid,
        "BALL RGB path unexpectedly required depth intrinsics");
  const auto goal=ObjectApproachGuide(Target(),true,2,k,{0,.45,0,0,0},640,480,320,400,false,0,0);
  Check(goal.valid && std::abs(goal.offset)<1e-9,"GOAL view could not project an elevated target onto the floor");
}
void SoftLoss(int blocks) {
  Driver d(Config());d.Warm();
  d.Block(); // The already-running LINE block belongs to LINE, not the object.
  Check(d.out.approach_completed_blocks==0,"pre-existing LINE block counted towards object lock");
  for(int i=0;i<blocks;++i) d.Block();
  Check(d.out.approach_completed_blocks==blocks && !d.out.approach_locked,
        "soft target locked before three completed blocks");
  const auto before=d.out.approach_completed_blocks;
  d.Tick({d.last_done,true,true,false});
  Check(d.out.approach_completed_blocks==before,"duplicate DONE counted a second block");
  d.MissingBlock();
  Check(d.out.active_mission==MissionType::kLine && d.out.approach_mission==MissionType::kNone &&
        d.out.approach_completed_blocks==0 && !d.out.candidate_phase,
        "lost soft target started stationary confirmation/search instead of releasing to LINE");
  d.in.ball_target=Target();
  for(int i=0;i<12;++i)d.Tick();
  Check(LinePlan::IsUnit(d.out.command.action) && d.out.approach_completed_blocks==0,
        "soft loss failed to reconnect LINE without retaining old object lock");
}
void LockAndLoss() {
  Driver d(Config());d.Warm();d.Block();
  d.Block();d.Block();
  Check(!d.out.approach_locked,"two blocks already locked the target");
  // No READY/ACK, including the final block's last READY, may commit early.
  d.Tick({d.current,true,false,true});
  Check(!d.out.approach_locked,"READY counted as physical completion");
  auto next=d.out.command.action_id;
  Check(next && next!=d.current,"third block first successor missing");
  d.Tick({next,true,false,false});d.Tick({d.current,true,true,false});d.current=next;
  for(int i=0;i<12;++i)d.Tick();
  d.Tick({d.current,true,false,true});
  Check(!d.out.approach_locked,"third block final READY committed too early");
  next=d.out.command.action_id;d.Tick({next,true,false,false});
  d.Tick({d.current,true,true,false});d.current=next;
  Check(d.out.approach_completed_blocks==3 && d.out.approach_locked,
        "third complete same-identity block did not lock");
  d.in.ball_target.reset();for(int i=0;i<3;++i)d.Tick();
  Check(d.out.approach_locked && d.out.active_mission==MissionType::kLine,
        "locked loss released the object or entered recovery before issued motion DONE");
  d.MissingBlock();
  Check(d.out.active_mission==MissionType::kBall && d.out.ball.active,
        "locked loss did not hand off to the existing BALL recovery");
  Check(!d.out.ball.entry_recovery && d.out.ball.recovery_step_limit==Config().ball.recovery_max_steps,
        "locked BALL reused the pre-lock entry-loss budget");
}

void EmptyGoalRetryLimit() {
  auto c=Config();c.enable_ball=false;c.enable_goal=true;c.initial_has_ball=true;
  c.goal_trial_start_goal=true;c.goal.lost_frames=3;
  c.object_approach.goal_empty_max_retries=3;
  c.goal.post_pickup_walk_fallback_actions=1;
  Driver d(c);d.in.ball_target.reset();d.in.camera_feedback={CameraMode::kGoal,true};
  int returns=0,age=0;std::uint64_t active=0,queued=0,done=0;
  bool ready=false;
  for(int frame=0;frame<600 && !d.out.terminal_hold;++frame) {
    CommandDeliveryFeedback fb;
    const auto id=d.out.command.action_id;
    if(id && LinePlan::IsUnit(d.out.command.action) && id!=done && id!=active && id!=queued) {
      if(!active) { active=id;age=0;ready=false; }
      else queued=id;
      fb={id,true,false,false};
    } else if(active && ++age>=12 && !ready) {
      ready=true;fb={active,true,false,true};
    } else if(active && age>=14) {
      done=active;fb={active,true,true,false};
      active=queued;queued=0;age=0;ready=false;
    }
    if(d.out.command.camera_request==CameraRequest::kForward &&
        d.in.camera_feedback.actual_mode!=CameraMode::kForward) {
      ++returns;d.in.camera_feedback={CameraMode::kForward,true};
    } else if(d.out.command.camera_request==CameraRequest::kGoal)
      d.in.camera_feedback={CameraMode::kGoal,true};
    d.Tick(fb,.1);
  }
  Check(returns==3 && d.out.terminal_hold && d.out.approach_goal_search_failed,
        "empty GOAL view did not stop after exactly three LINE/recheck retries");
  Check(d.out.goal.entry_line_retries==3,"GOAL retry diagnostics lost the consumed budget");
  d.in.backboard_target=Target();d.in.backboard_center_depth_m=2;
  for(int i=0;i<20;++i) {
    d.Tick();Check(d.out.terminal_hold && d.out.command.command_type!=CommandType::kAction &&
        d.out.command.camera_request==CameraRequest::kNone,
        "GOAL retry exhaustion resumed automatically");
  }
  d.core.Reset(true);d.in.backboard_target.reset();d.in.backboard_center_depth_m.reset();d.Tick();
  Check(!d.out.approach_goal_search_failed && !d.out.goal.entry_line_retries,
        "RESET retained empty GOAL failure or retry count");
}
void NearAndIdentity() {
  Driver d(Config());d.Warm();d.Block();d.Block();
  d.in.ball_target=Target(320,260);d.Tick();
  Check(d.out.approach_near && !d.out.approach_locked,"near approach unnecessarily waited for three blocks");
  d.Tick({d.current,true,false,true});
  Check(d.out.command.action_id==d.current && d.out.command.camera_request==CameraRequest::kNone,
        "near boundary queued another walk or moved the camera before DONE");
  d.Tick({d.current,true,true,false});
  Check(d.out.active_mission==MissionType::kLine && d.out.approach_near,
        "near BALL skipped the fresh stopped observation");
  for(int n=0;n<30 && d.out.command.action_id==0;++n)d.Tick();
  const auto near_step=d.out.command.action_id;
  Check(near_step && d.out.command.action==MissionAction::kStepForwardTwo,
        "near BALL did not choose the grid's upper-center two-step action");
  d.Tick({near_step,true,false,false});
  d.Tick({near_step,true,true,false});
  Check(d.out.active_mission==MissionType::kBall,"near object did not use the original near BALL controller");
  Check(d.out.ball.mode==BallMode::kWaitCameraDown && d.out.command.camera_request==CameraRequest::kDown &&
      !d.out.remembered_line_score,"near BALL did not discard LINE memory and enter DOWN immediately");
  Driver replacement(Config());replacement.Warm();replacement.Block();replacement.Block();
  replacement.in.ball_track_id=99;
  for(int i=0;i<3;++i) replacement.Tick();
  Check(replacement.out.approach_completed_blocks==1,
        "replacement identity cancelled a walking block before its evidence window");
  replacement.MissingBlock();
  replacement.in.ball_target=Target();
  for(int i=0;i<12;++i) replacement.Tick();
  Check(replacement.out.approach_completed_blocks==0 && !replacement.out.approach_locked,
        "a replacement identity inherited completed object walks");
}
void HurdleAndGoal() {
  auto c=Config();c.enable_ball=false;c.enable_hurdle=true;
  Driver h(c);h.in.ball_target.reset();h.in.hurdle_target=Target();h.in.hurdle_track_id=5;
  h.Warm();h.Block();h.Block();
  Check(h.out.approach_mission==MissionType::kHurdle && h.out.approach_completed_blocks==1,
        "HURDLE did not use the common LINE block planner");
  h.in.hurdle_target=Target(320,260);h.Tick();
  h.Tick({h.current,true,false,true});
  Check(h.out.command.action_id==h.current,"HURDLE boundary booked another shared unit");
  h.Tick({h.current,true,true,false});
  Check(h.out.active_mission==MissionType::kLine && h.out.approach_near,
        "near HURDLE skipped fresh observation");
  for(int n=0;n<30 && h.out.command.action_id==0;++n)h.Tick();
  const auto hurdle_step=h.out.command.action_id;
  Check(hurdle_step && h.out.command.action==MissionAction::kStepForwardTwo,
        "near HURDLE did not use its horizontal-line two-step action");
  h.Tick({hurdle_step,true,false,false});h.Tick({hurdle_step,true,true,false});
  Check(h.out.active_mission==MissionType::kHurdle,"HURDLE did not hand off to the existing DOWN approach");

  c=Config();c.enable_ball=false;c.enable_goal=true;c.initial_has_ball=true;c.goal_trial_start_goal=true;
  Driver g(c);g.in.ball_target.reset();g.in.camera_feedback={CameraMode::kGoal,true};
  g.in.line_centers.clear();g.in.backboard_target=Target();g.in.backboard_track_id=9;
  g.in.backboard_center_depth_m=2;g.in.goal_pose={true,0,2,0,1};
  for(int i=0;i<12;++i)g.Tick();
  Check(g.out.approach_mission==MissionType::kGoal && LinePlan::IsUnit(g.out.command.action) &&
        g.out.line_decision_guide && g.out.line_decision_guide->valid && !g.out.line_features.guide.valid,
        "GOAL approach required floor-line detections or failed to generate a common O/H guide");
  g.current=g.out.command.action_id;g.Block();
  g.in.backboard_center_depth_m=.5;g.in.goal_pose.z_m=.5;g.Tick();
  g.Tick({g.current,true,false,true});
  Check(g.out.command.action_id==g.current,"near GOAL reserved another shared unit");
  g.Tick({g.current,true,true,false});
  for(int n=0;n<30 && g.out.active_mission==MissionType::kLine;++n)g.Tick();
  Check(g.out.active_mission==MissionType::kGoal,"near GOAL did not enter existing fine approach");
  Check(g.out.goal_mission_index==c.test_goal_mission_index,"direct GOAL handoff incremented an already-started trial index");
}
void ManualAndReset() {
  auto c=Config();
  Driver d(c);d.in.manual_observation_control=true;d.Warm();
  Check(d.out.candidate_mission==MissionType::kBall && !d.out.approach_locked &&
        d.out.approach_mission==MissionType::kNone,"manual approval lost stationary candidate verification");
  Driver reset(Config());reset.Warm();reset.Block();reset.Block();reset.Block();reset.Block();
  Check(reset.out.approach_locked,"reset fixture did not reach lock");
  reset.core.Reset(true);reset.in.ball_target.reset();reset.Tick();
  Check(!reset.out.approach_locked && !reset.out.approach_completed_blocks &&
        reset.out.approach_mission==MissionType::kNone,"RESET retained approach commitment");
}
void CameraDrainAndProbe() {
  auto c=Config();c.enable_ball=false;c.enable_goal=true;c.initial_has_ball=true;
  c.goal.post_pickup_walk_fallback_actions=3;
  Driver d(c);d.in.ball_target.reset();d.Warm();
  d.Tick({d.current,true,false,true});
  const auto second=d.out.command.action_id;
  Check(second && second!=d.current,"probe fixture did not reserve second LINE unit");
  d.Tick({second,true,false,false});d.Tick({d.current,true,true,false});d.current=second;
  for(int i=0;i<12;++i)d.Tick();
  d.Tick({d.current,true,false,true});
  Check(d.out.command.action_id==d.current && d.out.command.camera_request==CameraRequest::kNone,
        "GOAL camera moved at READY or a new block was booked across the carrying gate");
  d.Tick({d.current,true,true,false});
  Check(d.out.command.camera_request==CameraRequest::kGoal && d.out.command.command_type!=CommandType::kAction,
        "GOAL probe did not wait for all issued locomotion DONE");
  d.in.line_centers.clear();d.in.camera_feedback={CameraMode::kTransition,false};
  for(int i=0;i<3;++i) {
    d.Tick();Check(d.out.command.command_type!=CommandType::kAction,"camera transition issued locomotion");
  }
  d.in.camera_feedback={CameraMode::kGoal,true};d.in.backboard_target=Target();d.in.backboard_track_id=9;
  d.in.backboard_center_depth_m=2;d.in.goal_pose={true,0,2,0,1};
  for(int i=0;i<10;++i)d.Tick();
  Check(LinePlan::IsUnit(d.out.command.action) && !d.out.line_in_recovery,
        "GOAL camera acquisition was incorrectly routed into missing-floor-LINE recovery");
  d.current=d.out.command.action_id;d.Tick({d.current,true,false,false});
  d.in.backboard_target.reset();d.in.backboard_center_depth_m.reset();d.in.goal_pose={};
  for(int i=0;i<3;++i)d.Tick();
  Check(d.out.command.camera_request==CameraRequest::kNone,"soft GOAL loss moved the head during locomotion");
  d.Tick({d.current,true,false,true});
  const auto second_goal_unit=d.out.command.action_id;
  Check(second_goal_unit && second_goal_unit!=d.current && LinePlan::IsUnit(d.out.command.action),
        "one missing GOAL window cancelled the remainder of its walking block");
  d.Tick({second_goal_unit,true,false,false});
  d.Tick({d.current,true,true,false});d.current=second_goal_unit;
  for(int i=0;i<12;++i)d.Tick();
  d.Tick({d.current,true,false,true});
  Check(d.out.command.action_id==d.current,"unqualified GOAL block booked another walk");
  d.Tick({d.current,true,true,false});
  Check(d.out.command.camera_request==CameraRequest::kForward && d.out.command.command_type!=CommandType::kAction,
        "soft GOAL loss did not restore FORWARD after actual DONE");
}
void GoalTurnObservationUsesSelectedPath() {
  auto c=Config();c.enable_ball=false;c.enable_goal=true;c.initial_has_ball=true;
  c.line_p2p.offset_gain=c.line_p2p.heading_gain=1;
  c.line_p2p.steering_deadband=.05;c.line_p2p.in_place_turn_score=.1;
  Driver d(c);d.in.ball_target.reset();d.in.line_centers.clear();
  d.in.camera_feedback={CameraMode::kGoal,true};d.in.backboard_target=Target(500,100);
  d.in.backboard_track_id=7;d.in.backboard_center_depth_m=2;d.in.goal_pose={true,.6,2,0,1};
  for(int i=0;i<12;++i)d.Tick();
  Check(d.out.command.action==MissionAction::kStepForwardRightOneHurdle,
        "GOAL path did not reuse the LINE in-place steering guard");
  const auto turn=d.out.command.action_id;d.Tick({turn,true,false,false});
  d.in.backboard_target=Target();d.in.goal_pose.x_m=0;
  d.Tick({turn,true,true,false});
  for(int i=0;i<20;++i)d.Tick({},.10);
  Check(LinePlan::IsUnit(d.out.command.action) && !d.out.line_in_recovery &&
        !d.out.approach_completed_blocks,
        "GOAL post-turn observation used invisible floor lines or counted the turn as an object block");
}

void FarGoalDepthRecovery(bool found,bool acquired=false) {
  auto c=Config();c.enable_ball=false;c.enable_goal=true;c.initial_has_ball=true;
  c.goal_trial_start_goal=true;c.goal.lost_frames=3;c.goal.depth_recovery_max_steps=3;
  Driver d(c);d.in.ball_target.reset();d.in.line_centers.clear();
  d.in.camera_feedback={CameraMode::kGoal,true};d.in.backboard_image_without_depth=true;
  if (acquired) {
    d.in.backboard_target=Target();d.in.backboard_track_id=9;
    d.in.backboard_center_depth_m=2;
    for(int i=0;i<12;++i)d.Tick();
    const auto unit=d.out.command.action_id;
    Check(unit && d.out.approach_mission==MissionType::kGoal,"Depth fixture did not acquire GOAL");
    d.Tick({unit,true,false,false});
    d.in.backboard_target.reset();d.in.backboard_center_depth_m.reset();
    for(int i=0;i<4;++i)d.Tick();
    Check(d.out.approach_mission==MissionType::kGoal && !d.out.approach_missing_frames,
          "visible image/invalid Depth was mistaken for a soft object disappearance");
    d.Tick({unit,true,true,false});
  }
  std::uint64_t completed=0;
  for(int n=1;n<=3;++n) {
    for(int i=0;i<40 && (d.out.command.action_id==0 || d.out.command.action_id==completed);++i)d.Tick();
    const auto id=d.out.command.action_id;
    Check(id && id!=completed && d.out.command.action==MissionAction::kStepForwardOne,
          "far GOAL with visible image/invalid Depth did not request one bounded WALK step");
    d.Tick({id,true,false,false});
    Check(d.out.goal.depth_recovery_steps==n-1,"Depth recovery ACK consumed a step");
    d.Tick({id,true,true,false});completed=id;
    d.Tick({id,true,true,false});
    Check(d.out.goal.depth_recovery_steps==n,"far GOAL duplicate DONE changed recovery count");
    if(found) {
      d.in.backboard_image_without_depth=false;d.in.backboard_target=Target();d.in.backboard_track_id=9;
      d.in.backboard_center_depth_m=2;d.in.goal_pose={true,0,2,0,1};
    }
    for(int i=0;i<25 && d.out.goal.depth_recovery_active;++i)d.Tick({},.1);
    if(found) {
      for(int i=0;i<20;++i)d.Tick();
      Check(LinePlan::IsUnit(d.out.command.action) && d.out.approach_mission==MissionType::kGoal &&
          !d.out.approach_goal_depth_failed,"recovered far GOAL coordinates did not resume shared virtual-line approach");
      return;
    }
  }
  Check(d.out.terminal_hold && d.out.approach_goal_depth_failed &&
      d.out.goal.depth_recovery_steps==3 && d.out.approach_completed_blocks==0,
      "far Depth recovery did not terminate after three steps or counted recovery as object LINE blocks");
}

void NearLossBeforeLock(MissionType mission) {
  auto c=Config();c.enable_ball=mission==MissionType::kBall;
  c.enable_hurdle=mission==MissionType::kHurdle;c.enable_goal=mission==MissionType::kGoal;
  c.initial_has_ball=c.goal_trial_start_goal=mission==MissionType::kGoal;
  Driver d(c);
  if(mission==MissionType::kBall) d.Warm();
  else if(mission==MissionType::kHurdle) {
    d.in.ball_target.reset();d.in.hurdle_target=Target();d.in.hurdle_track_id=5;d.Warm();
  } else {
    d.in.ball_target.reset();d.in.camera_feedback={CameraMode::kGoal,true};
    d.in.line_centers.clear();d.in.backboard_target=Target();d.in.backboard_track_id=9;
    d.in.backboard_center_depth_m=2;d.in.goal_pose={true,0,2,0,1};
    for(int i=0;i<12;++i)d.Tick();d.current=d.out.command.action_id;
    Check(d.current!=0,"near GOAL fixture did not start common approach");
    d.Tick({d.current,true,false,false});
  }
  if(mission==MissionType::kBall)d.in.ball_target=Target(320,260);
  else if(mission==MissionType::kHurdle)d.in.hurdle_target=Target(320,260);
  else { d.in.backboard_center_depth_m=.5;d.in.goal_pose.z_m=.5; }
  d.Tick();Check(d.out.approach_near && !d.out.approach_locked,"pre-lock near boundary was not latched");
  Check(!d.out.remembered_line_score,"near boundary retained pre-entry LINE steering memory");
  // Even a visible next frame outside the boundary cannot undo near commitment.
  if(mission==MissionType::kBall)d.in.ball_target=Target();
  else if(mission==MissionType::kHurdle)d.in.hurdle_target=Target();
  else { d.in.backboard_center_depth_m=2;d.in.goal_pose.z_m=2; }
  d.Tick();Check(d.out.approach_near,"near boundary was undone by image/depth jitter");
  d.in.ball_target.reset();d.in.hurdle_target.reset();d.in.backboard_target.reset();
  d.in.backboard_center_depth_m.reset();d.in.goal_pose={};
  for(int i=0;i<3;++i)d.Tick();
  Check(d.out.approach_near && d.out.approach_mission==mission,
        "pre-lock near loss was discarded as a far soft candidate");
  d.Tick({d.current,true,true,false});
  for(int i=0;i<120 && d.out.active_mission==MissionType::kLine;++i)d.Tick({},.1);
  Check(d.out.active_mission==mission && !d.out.remembered_line_score,
        "near loss returned to LINE instead of the committed object mission");
  if(mission!=MissionType::kGoal) {
    Check(d.out.command.camera_request==CameraRequest::kDown,"BALL/HURDLE near loss did not request DOWN");
    d.in.camera_feedback={CameraMode::kDown,true};
  } else Check(d.out.command.camera_request!=CameraRequest::kForward,
               "near GOAL loss restored FORWARD/LINE");
  d.in.allow_new_body_actions=false;
  for(int i=0;i<30;++i)d.Tick();
  Check(d.out.active_mission==mission,"near recovery escaped to LINE after camera/observation changes");
}

void NearGoalPreemptsDepthWalk() {
  auto c=Config();c.enable_ball=false;c.enable_goal=true;
  c.initial_has_ball=c.goal_trial_start_goal=true;c.goal.lost_frames=3;
  Driver d(c);d.in.ball_target.reset();d.in.line_centers.clear();
  d.in.camera_feedback={CameraMode::kGoal,true};d.in.backboard_image_without_depth=true;
  for(int i=0;i<30 && !d.out.command.action_id;++i)d.Tick();
  const auto walk=d.out.command.action_id;
  Check(walk && d.out.command.action==MissionAction::kStepForwardOne,
        "near preemption fixture did not start a real Depth recovery WALK");
  d.Tick({walk,true,false,false});
  d.in.backboard_target=Target();d.in.backboard_track_id=9;
  d.in.backboard_image_without_depth=false;d.in.backboard_center_depth_m=.5;
  d.in.goal_pose={}; // A near center depth is sufficient even without edge pose.
  for(int i=0;i<12;++i) {
    d.Tick();
    Check(d.out.command.action_id==walk,"near sighting canceled an accepted WALK");
  }
  Check(d.out.approach_near,"near board remained behind coordinate-recovery routing");
  d.Tick({walk,true,true,false});
  Check(d.out.active_mission==MissionType::kLine && d.out.approach_near,
        "near board bypassed its fresh stopped observation");
  for(int i=0;i<30 && d.out.active_mission==MissionType::kLine;++i)d.Tick();
  Check(d.out.active_mission==MissionType::kGoal &&
        d.out.command.action==MissionAction::kDefaultPoseMode && !d.out.goal.depth_recovery_active,
        "near board issued another Depth WALK instead of FINE feet after real DONE");
}

void SoftGoalLossClearsDepthEpisode() {
  auto c=Config();c.enable_ball=false;c.enable_goal=true;
  c.initial_has_ball=c.goal_trial_start_goal=true;c.goal.lost_frames=3;
  Driver d(c);d.in.ball_target.reset();d.in.line_centers.clear();
  d.in.camera_feedback={CameraMode::kGoal,true};d.in.backboard_target=Target();
  d.in.backboard_track_id=9;d.in.backboard_center_depth_m=2;
  for(int i=0;i<12;++i)d.Tick();
  const auto unit=d.out.command.action_id;
  Check(unit && d.out.approach_mission==MissionType::kGoal,"soft-loss fixture did not acquire GOAL");
  d.Tick({unit,true,false,false});d.in.backboard_center_depth_m.reset();
  for(int i=0;i<4;++i)d.Tick();
  d.Tick({unit,true,true,false});
  for(int i=0;i<10 && d.out.command.action!=MissionAction::kStepForwardOne;++i)d.Tick();
  const auto walk=d.out.command.action_id;
  Check(walk && d.out.command.action==MissionAction::kStepForwardOne,"soft-loss fixture missed Depth WALK");
  d.Tick({walk,true,false,false});d.in.backboard_target.reset();
  for(int i=0;i<3;++i)d.Tick();
  Check(d.out.approach_mission==MissionType::kGoal && d.out.command.action_id==walk,
        "a few missed frames cancelled the accepted Depth WALK");
  d.Tick({walk,true,true,false});
  bool returned=false;
  for(int i=0;i<40 && !returned;++i) {
    d.Tick({},.1);
    returned=d.out.command.camera_request==CameraRequest::kForward;
  }
  Check(returned && d.out.approach_mission==MissionType::kNone,
        "soft Depth loss did not restore FORWARD after stopped observation");
}

void HurdleLineLossUsesLineRecovery() {
  auto c=Config();c.enable_ball=false;c.enable_hurdle=true;
  Driver d(c);d.in.ball_target.reset();d.in.hurdle_target=Target();d.in.hurdle_track_id=5;
  d.Warm();d.in.line_centers.clear();
  for(int i=0;i<15;++i)d.Tick();
  d.Tick({d.current,true,true,false});
  d.current=d.out.command.action_id;
  Check(d.current && LinePlan::IsUnit(d.out.command.action),
        "HURDLE fixture did not finish the first unit of its walking block");
  d.Tick({d.current,true,false,false});
  for(int i=0;i<15;++i)d.Tick();
  d.Tick({d.current,true,false,true});
  d.Tick({d.current,true,true,false});
  d.current=d.out.command.action_id;
  Check(d.current && LinePlan::IsUnit(d.out.command.action),
        "qualified earlier LINE block did not start the next block");
  for(int n=0;n<2;++n) {
    d.Tick({d.current,true,false,false});
    for(int i=0;i<15;++i)d.Tick();
    d.Tick({d.current,true,false,true});
    const auto next=d.out.command.action_id;
    d.Tick({d.current,true,true,false});
    if(n==0) {
      Check(next && next!=d.current,"first unit did not continue its block");
      d.current=next;
    }
  }
  bool recovering=d.out.line_in_recovery;
  for(int i=0;i<35 && !recovering;++i) {d.Tick({},.1);recovering=d.out.line_in_recovery;}
  Check(recovering && !d.out.terminal_hold,
        "visible far HURDLE with missing floor LINE stayed in router HOLD forever");
}

void FarReacquisitionReturnsToSharedPath(MissionType mission) {
  auto c=Config();c.enable_ball=mission==MissionType::kBall;
  c.enable_hurdle=mission==MissionType::kHurdle;c.enable_goal=mission==MissionType::kGoal;
  c.initial_has_ball=c.goal_trial_start_goal=mission==MissionType::kGoal;
  Driver d(c);
  auto visible=[&](bool value) {
    d.in.ball_target.reset();d.in.hurdle_target.reset();d.in.backboard_target.reset();
    if(mission==MissionType::kBall) {if(value)d.in.ball_target=Target();}
    else if(mission==MissionType::kHurdle) {if(value)d.in.hurdle_target=Target();d.in.hurdle_track_id=5;}
    else {
      if(value)d.in.backboard_target=Target();d.in.backboard_track_id=9;
      d.in.backboard_center_depth_m=value ? std::optional<double>(2) : std::nullopt;
      d.in.goal_pose=value ? GoalPoseObservation{true,0,2,0,1} : GoalPoseObservation{};
    }
  };
  visible(true);
  if(mission==MissionType::kGoal) {
    d.in.camera_feedback={CameraMode::kGoal,true};d.in.line_centers.clear();
    for(int i=0;i<12;++i)d.Tick();d.current=d.out.command.action_id;
    Check(d.current!=0,"GOAL far recovery fixture never started");d.Tick({d.current,true,false,false});
  } else d.Warm();
  for(int i=0;i<5 && !d.out.approach_locked;++i)d.Block();
  Check(d.out.approach_locked,"far recovery fixture did not lock the object");
  const auto fail_block=[&] {
    visible(false);
    for(int i=0;i<12;++i)d.Tick();
    d.Tick({d.current,true,false,true});
    const auto second=d.out.command.action_id;
    Check(second && second!=d.current && LinePlan::IsUnit(d.out.command.action),
          "missing far object cancelled a walking block after one unit");
    d.Tick({second,true,false,false});d.Tick({d.current,true,true,false});d.current=second;
    for(int i=0;i<12;++i)d.Tick();
    d.Tick({d.current,true,false,true});
    Check(d.out.command.action_id==d.current,
          "missing far object booked another block without 10/n evidence");
    d.Tick({d.current,true,true,false});
  };
  fail_block();
  Check(d.out.active_mission==mission,"locked far loss did not start bounded object recovery");
  const int ball_index=d.out.ball_mission_index,goal_index=d.out.goal_mission_index;
  visible(true);
  std::uint64_t pending=0,done=0;bool resumed=false;
  for(int i=0;i<160;++i) {
    CommandDeliveryFeedback fb;
    const auto id=d.out.command.action_id;
    if(pending) {
      done=pending;fb={pending,true,true,false};pending=0;
    } else if(id && id!=done && d.out.command.command_type==CommandType::kAction && !LinePlan::IsUnit(d.out.command.action)) {
      pending=id;fb={id,true,false,false};
    }
    d.Tick(fb,.1);
    if(d.out.active_mission==MissionType::kLine && d.out.approach_mission==mission && LinePlan::IsUnit(d.out.command.action)) {
      resumed=true;break;
    }
  }
  Check(resumed && d.out.approach_locked && d.out.approach_completed_blocks>=3,
        "confirmed far reacquisition did not resume 32/33/34 with the same lock/block history");
  Check(d.out.ball_mission_index==ball_index && d.out.goal_mission_index==goal_index,
        "resuming virtual approach changed the object's mission sequence number");
  d.current=d.out.command.action_id;d.Tick({d.current,true,false,false});
  fail_block();
  Check(d.out.active_mission==mission && d.out.ball_mission_index==ball_index && d.out.goal_mission_index==goal_index,
        "a repeated far loss admitted a new mission instead of preserving the existing mission");
}
}
int main() {
  Geometry();SoftLoss(1);SoftLoss(2);LockAndLoss();NearAndIdentity();HurdleAndGoal();ManualAndReset();CameraDrainAndProbe();GoalTurnObservationUsesSelectedPath();EmptyGoalRetryLimit();FarGoalDepthRecovery(false);FarGoalDepthRecovery(true);
  NearGoalPreemptsDepthWalk();SoftGoalLossClearsDepthEpisode();HurdleLineLossUsesLineRecovery();
  FarGoalDepthRecovery(false,true);FarGoalDepthRecovery(true,true);
  for(auto mission:{MissionType::kBall,MissionType::kHurdle,MissionType::kGoal}) {
    NearLossBeforeLock(mission);FarReacquisitionReturnsToSharedPath(mission);
  }
}
