#include "vision_core/config_loader.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <limits>
using namespace vision_core;
namespace {
ObjectTarget Target(double u, double v=100, double width=20) {
  ObjectTarget t; t.center_px={u,v}; t.box_px={u-width/2,v-10,width,20};
  t.width_px=width;t.height_px=20;t.area_px=width*20;t.confidence=1;return t;
}
void VerifiedFineWindows() {
  const auto c=LoadDefaultAlgorithmConfig();
  const std::deque<bool> hits{true,false,true,true,false,true,true,false,true,true};
  const ActionExecutionFeedback done{true,false,true};
  BallController ball(c.ball,3); ball.StartTestExecute();
  ball.UseVerifiedObservation(Target(345,100),hits,640,480);
  auto b=ball.Compute(Target(0,0),640,480,1,{CameraMode::kDown,true},done);
  assert(b.action.action==MissionAction::kPickBall && b.tracked.stable);
  BallController single(c.ball,3);single.StartTestExecute();
  single.UseVerifiedObservation(Target(345,100),{true},640,480);
  b=single.Compute(Target(345,100),640,480,1,{CameraMode::kDown,true},done);
  assert(b.action.action==MissionAction::kNone && !single.ExecutionObservationReady());
  HurdleController hurdle(c.hurdle,3);hurdle.StartTestExecute();
  hurdle.UseVerifiedObservation(Target(320,270),hits,640,480);
  auto h=hurdle.Compute({},640,480,1,{CameraMode::kDown,true},done,{});
  assert(h.mode==HurdleMode::kContactWalk && h.action.action==MissionAction::kStepForwardHalf);
  GoalController goal(c.goal,3);goal.StartTestExecute();
  GoalPoseObservation pose{true,-.08,.5,0,1};
  goal.UseVerifiedObservation(Target(320),pose,hits,hits,640,480);
  auto g=goal.Compute({},{},{},640,480,1,true,{CameraMode::kGoal,true},done);
  assert(g.action.action==MissionAction::kShoot && g.fine_decision_pose_used);
}
void InvalidAndNearWindows() {
  auto c=LoadDefaultAlgorithmConfig();
  const std::deque<bool> hits(10,true), missing(10,false);
  const ActionExecutionFeedback done{true,false,true};
  BallController ball(c.ball,3);ball.StartTestExecute();
  ball.UseVerifiedObservation({},missing,640,480);
  auto b=ball.Compute({},640,480,1,{CameraMode::kDown,true},done);
  assert(b.mode==BallMode::kBallRecoveryDown);
  HurdleController hurdle(c.hurdle,3);hurdle.StartTestExecute();
  hurdle.UseVerifiedObservation({},missing,640,480);
  auto h=hurdle.Compute({},640,480,1,{CameraMode::kDown,true},done,{});
  assert(h.mode==HurdleMode::kRecoveryDown);
  GoalController goal(c.goal,3);goal.StartTestNear();
  const CameraFeedback camera{CameraMode::kGoal,true};
  goal.Compute({},{},{},640,480,0,true,camera,{});
  goal.UseVerifiedObservation(Target(320),{true,0,.7,0,1},hits,hits,640,480,hits);
  auto g=goal.Compute({},{},{},640,480,1,true,camera,{});
  assert(g.mode==GoalMode::kCoarseAdjust); // no second near-detection window
}
void GoalRecovery(bool fine, bool left) {
  auto c=LoadDefaultAlgorithmConfig().goal;
  c.stable_window=c.stable_min_hits=c.lost_frames=1;c.smooth_alpha=1;
  GoalController goal(c,3);
  if(fine)goal.StartTestExecute();else goal.StartTestNear();
  const CameraFeedback camera{CameraMode::kGoal,true};
  const auto target=Target(left?150:550);
  if(!fine)goal.Compute({},{},{},640,480,0,true,camera,{},false);
  goal.Compute({},target,{true,0,1.5,0,1},640,480,.1,true,camera,{},false);
  auto g=goal.Compute({},{},{},640,480,.2,true,camera,{});
  assert(g.mode==GoalMode::kRecoveryTurn);
  g=goal.Compute({},{},{},640,480,.3,true,camera,{});
  const auto turn=fine ? (left?MissionAction::kTurnLeft:MissionAction::kTurnRight)
      : (left?MissionAction::kTurnLeftInPlace:MissionAction::kStepForwardRightOneHurdle);
  assert(g.action.action==turn && goal.RecoveryTurnsCompleted()==0);
  g=goal.Compute({},{},{},640,480,.4,true,camera,{false,true,false});
  assert(g.action.action==MissionAction::kNone && goal.RecoveryTurnsCompleted()==0);
  // In-motion reacquisition resets the budget. The late DONE cannot count it.
  g=goal.Compute({},target,{true,0,1.5,0,1},640,480,.5,true,camera,{false,true,false});
  assert(g.mode==(fine?GoalMode::kFineAdjust:GoalMode::kApproach));
  assert(goal.RecoveryTurnsCompleted()==0);
  goal.Compute({},{},{},640,480,.6,true,camera,{true,false,false});
  assert(goal.RecoveryTurnsCompleted()==0);
  // A fresh loss has exactly three completed turns before stationary fallback.
  goal.Reset();
  if(fine)goal.StartTestExecute();else goal.StartTestNear();
  if(!fine)goal.Compute({},{},{},640,480,1,true,camera,{},false);
  goal.Compute({},target,{true,0,1.5,0,1},640,480,1.1,true,camera,{},false);
  goal.Compute({},{},{},640,480,1.2,true,camera,{});
  g=goal.Compute({},{},{},640,480,1.3,true,camera,{});
  for(int i=1;i<=3;++i) {
    assert(g.action.action==turn);
    g=goal.Compute({},{},{},640,480,1.3+.2*i,true,camera,{true,false,true});
    assert(goal.RecoveryTurnsCompleted()==i);
  }
  assert(g.mode==(fine?GoalMode::kFineRecovery:GoalMode::kRecoverySearch));
  // Visible board + failed pose uses stationary pose recovery, never a turn.
  if(fine) {
    goal.StartTestExecute();
    goal.Compute({},target,{true,0,1,0,1},640,480,3,true,camera,{},false);
    g=goal.Compute({},target,{},640,480,3.1,true,camera,{});
    assert(g.mode==GoalMode::kFineRecovery && g.action.action==MissionAction::kNone);
  }
}
void GoalEdges() {
  auto c=LoadDefaultAlgorithmConfig().goal;c.stable_window=c.stable_min_hits=1;
  const CameraFeedback camera{CameraMode::kGoal,true};
  for(double margin:{0.0,35.0,50.0,80.0}) for(bool left:{false,true}) {
    c.edge_margin_px=margin;
    GoalController goal(c,3);goal.StartTestNear();
    goal.Compute({},{},{},640,480,0,true,camera,{});
    const auto edge=Target(left?margin+10:640-margin-10,100,20);
    auto g=goal.Compute({},edge,{},640,480,.1,true,camera,{});
    const auto turn=left?MissionAction::kTurnLeftInPlace:MissionAction::kStepForwardRightOneHurdle;
    assert(g.mode==GoalMode::kEdgeTurn && g.action.action==turn);
    // Stable reacquisition during either cycle cannot stop a fixed pair.
    g=goal.Compute({},Target(320),{},640,480,.2,true,camera,{false,true,false});
    assert(g.mode==GoalMode::kEdgeTurn && g.action.action==MissionAction::kNone);
    g=goal.Compute({},Target(320),{},640,480,.4,true,camera,{false,true,false});
    assert(g.mode==GoalMode::kEdgeTurn && g.action.action==MissionAction::kNone);
    // First actual DONE immediately requests the second single turn, even
    // without observation_complete and despite stable centered reacquisition.
    g=goal.Compute({},Target(320),{},640,480,.5,true,camera,{true,false,false});
    assert(g.mode==GoalMode::kEdgeTurn && g.action.action==turn);
    g=goal.Compute({},Target(320),{},640,480,.6,true,camera,{false,true,false});
    assert(g.mode==GoalMode::kEdgeTurn && g.action.action==MissionAction::kNone);
    g=goal.Compute({},Target(320),{},640,480,.7,true,camera,{true,false,false});
    assert(g.mode==GoalMode::kSearch && g.action.action==MissionAction::kNone);
    assert(goal.RecoveryTurnsCompleted()==0);
    // A later edge event starts a fresh two-turn budget.
    g=goal.Compute({},edge,{},640,480,.8,true,camera,{});
    assert(g.mode==GoalMode::kEdgeTurn && g.action.action==turn);
    g=goal.Compute({},Target(320),{},640,480,.9,true,camera,{true,false,false});
    assert(g.mode==GoalMode::kEdgeTurn && g.action.action==turn);
    g=goal.Compute({},Target(320),{},640,480,1.0,true,camera,{true,false,false});
    assert(g.mode==GoalMode::kSearch && g.action.action==MissionAction::kNone);
    // A box one pixel outside the margin does not trigger a pair.
    GoalController outside(c,3);outside.StartTestNear();
    outside.Compute({},{},{},640,480,0,true,camera,{});
    g=outside.Compute({},Target(left?margin+11:640-margin-11),{},640,480,.1,true,camera,{});
    assert(g.mode!=GoalMode::kEdgeTurn);
  }
  c.edge_margin_px=50;
  GoalController both(c,3);both.StartTestNear();
  both.Compute({},{},{},640,480,0,true,camera,{});
  auto g=both.Compute({},Target(320,100,600),{},640,480,.1,true,camera,{});
  assert(g.action.action==MissionAction::kNone && g.mode==GoalMode::kSearch);
  // Retuning a not-yet-committed pair re-evaluates the new boundary.
  GoalController preview(c,3);preview.StartTestNear();
  preview.Compute({},{},{},640,480,0,true,camera,{});
  g=preview.Compute({},Target(60),{},640,480,.1,true,camera,{},false);
  assert(g.action.action==MissionAction::kTurnLeftInPlace);
  c.edge_margin_px=20;preview.UpdateTuning(c);
  g=preview.Compute({},Target(60),{},640,480,.2,true,camera,{},false);
  assert(g.mode!=GoalMode::kEdgeTurn);
  // PAUSE can consume A DONE without authorizing B yet. Retuning in that
  // idle gap must preserve the accepted pair and its one remaining turn.
  c.edge_margin_px=50;
  GoalController paused(c,3);paused.StartTestNear();
  paused.Compute({},{},{},640,480,0,true,camera,{});
  g=paused.Compute({},Target(60),{},640,480,.1,true,camera,{});
  assert(g.action.action==MissionAction::kTurnLeftInPlace);
  g=paused.Compute({},Target(320),{},640,480,.2,true,camera,{true,false,false},false);
  assert(g.mode==GoalMode::kEdgeTurn);
  c.edge_margin_px=0;paused.UpdateTuning(c);
  g=paused.Compute({},Target(320),{},640,480,.3,true,camera,{},false);
  assert(g.mode==GoalMode::kEdgeTurn);
  g=paused.Compute({},Target(320),{},640,480,.4,true,camera,{});
  assert(g.action.action==MissionAction::kTurnLeftInPlace);
  g=paused.Compute({},Target(320),{},640,480,.5,true,camera,{true,false,false});
  assert(g.mode==GoalMode::kSearch && g.action.action==MissionAction::kNone);
}
MissionFrameInput Frame(double t) {
  MissionFrameInput f;f.now_sec=t;f.image_width=640;f.image_height=480;
  f.command_transport_enabled=true;f.advance_line_fsm=false;return f;
}
void GoalPairHasOneObservation(bool left) {
  auto c=LoadDefaultAlgorithmConfig();
  c.test_mission=MissionType::kGoal;c.test_stage=1;
  c.enable_ball=c.enable_hurdle=false;c.initial_has_ball=true;
  c.transition_settle_sec=0;
  c.goal.stable_window=c.goal.stable_min_hits=1;
  MissionController core(c);
  auto f=Frame(0);f.camera_feedback={CameraMode::kGoal,true};
  f.backboard_target=Target(left?60:580);f.goal_pose={true,0,1.2,0,1};
  f.backboard_center_depth_m=1.2;
  const auto turn=left?MissionAction::kTurnLeftInPlace:MissionAction::kStepForwardRightOneHurdle;
  MissionFrameResult r;
  for(int i=0;i<100;++i) {
    f.now_sec=.1*i;f.delivery_feedback={};
    if(r.command.action_id)f.delivery_feedback={r.command.action_id,true,true,false};
    r=core.Step(f);
    if(r.command.action==turn)break;
  }
  assert(r.command.action==turn && r.repeat_sequence);
  const auto first_id=r.command.action_id;
  const int scope=r.repeat_scope;
  f.backboard_target=Target(320);
  for(int i=0;i<5;++i) {
    f.now_sec+=.1;f.delivery_feedback={first_id,true,false,false};r=core.Step(f);
    assert(!r.post_motion_observing && r.repeat_scope==scope);
  }
  // A's actual DONE dispatches B with a new ID in this same frame.
  f.now_sec+=.1;f.delivery_feedback={first_id,true,true,false};r=core.Step(f);
  assert(!r.post_motion_observing && r.repeat_scope==scope);
  assert(r.command.command_type==CommandType::kAction && r.command.action==turn);
  const auto second_id=r.command.action_id;
  assert(second_id!=0 && second_id!=first_id);
  // A repeated/late A DONE must not finish B or issue a third turn.
  for(int i=0;i<5;++i) {
    f.now_sec+=.1;
    f.delivery_feedback=i==0 ? CommandDeliveryFeedback{first_id,true,true,false}
        : CommandDeliveryFeedback{second_id,true,false,false};
    r=core.Step(f);
    assert(!r.post_motion_observing && r.repeat_scope==scope);
    assert(r.command.action_id==second_id);
  }
  f.now_sec+=.1;const double done_at=f.now_sec;
  // PAUSE may receive physical DONE before its next sensory frame. Changing
  // only the margin must keep that edge available for the normal observation.
  core.AcceptDeliveryFeedback({second_id,true,true,false},f.now_sec);
  core.SetGoalEdgeMargin(40);
  f.delivery_feedback={second_id,true,true,false};r=core.Step(f);
  assert(r.post_motion_observing && !r.repeat_sequence && r.command.action_id==0);
  for(int i=1;i<10;++i) {
    f.now_sec=done_at+.1*i;f.delivery_feedback={};r=core.Step(f);
    assert(r.post_motion_observing && !r.repeat_sequence && r.command.action_id==0);
  }
  f.now_sec=done_at+1;f.delivery_feedback={};r=core.Step(f);
  assert(!r.post_motion_observing && r.goal.mode!=GoalMode::kEdgeTurn);
}
void PoseSettleAndConcurrentObservation() {
  auto c=LoadDefaultAlgorithmConfig();c.test_mission=MissionType::kBall;c.test_stage=3;
  c.enable_goal=c.enable_hurdle=false;
  MissionController core(c);
  auto f=Frame(0);f.camera_feedback={CameraMode::kDown,true};
  auto r=core.Step(f);assert(r.command.action==MissionAction::kDefaultPoseMode);
  const auto id=r.command.action_id;
  f.now_sec=.1;f.delivery_feedback={id,true,true,false};r=core.Step(f);
  for(int i=2;i<=6;++i) {
    f.now_sec=.1*i;f.delivery_feedback={};f.ball_target=Target(345,100);r=core.Step(f);
    assert(r.command.action_id==0 && !r.post_motion_observing);
  }
  // Startup's fresh near-camera FSM settles before a manual/automatic window.
  for(int i=7;i<=35 && r.command.action!=MissionAction::kPickBall;++i) {
    f.now_sec=.1*i;f.delivery_feedback={};f.ball_target=Target(345,100);r=core.Step(f);
  }
  assert(r.command.action==MissionAction::kPickBall);
}
void FineObservationTiming(double extra_settle, bool manual=false) {
  auto c=LoadDefaultAlgorithmConfig();c.test_mission=MissionType::kBall;c.test_stage=2;
  c.enable_goal=c.enable_hurdle=false;c.fine_motion_settle_sec=extra_settle;
  MissionController core(c);
  auto f=Frame(0);f.camera_feedback={CameraMode::kDown,true};
  MissionFrameResult r;
  std::uint64_t setup_done=0;
  for(int i=0;i<=160;++i) {
    f.now_sec=.05*i;f.ball_target=Target(345,50);f.delivery_feedback={};
    if(r.command.action_id!=0 && r.command.action_id!=setup_done) {
      setup_done=r.command.action_id;f.delivery_feedback={setup_done,true,true,false};
    }
    r=core.Step(f);
    if(r.command.action==MissionAction::kStepForwardHalf)break;
  }
  assert(r.command.action==MissionAction::kStepForwardHalf);
  const auto motion_id=r.command.action_id;
  f.now_sec+=.1;const double done_at=f.now_sec;
  f.manual_observation_control=manual;
  f.delivery_feedback={motion_id,true,true,false};f.ball_target=Target(345,150);
  r=core.Step(f);if(!manual)assert(r.post_motion_observing);
  const double decision_at=done_at+std::max(extra_settle,manual?.05:0.0)+1;
  bool decided=false;
  for(int i=1;i<=40;++i) {
    f.now_sec=done_at+.05*i;
    f.begin_manual_observation=manual && i==1;
    f.delivery_feedback=i==2 ? CommandDeliveryFeedback{motion_id,true,true,false} : CommandDeliveryFeedback{};
    r=core.Step(f);
    if(f.now_sec+1e-9<decision_at) {
      assert(r.command.action!=MissionAction::kStepBack);
    } else {
      assert(r.command.action==MissionAction::kStepBack);
      assert(f.now_sec<=decision_at+.051);
      decided=true;break;
    }
  }
  assert(decided); // No extra ten-frame/FINE wait after the completed window.
}
void ImageDetectionWithMissingDepth() {
  auto c=LoadDefaultAlgorithmConfig();c.test_mission=MissionType::kGoal;c.test_stage=3;
  c.enable_ball=c.enable_hurdle=false;c.goal.lost_frames=1;
  MissionController core(c);
  PerceptionFrameInput f;f.image_width=640;f.image_height=480;
  f.intrinsics={600,600,320,240};f.camera_feedback={CameraMode::kGoal,true};
  f.command_transport_enabled=true;f.allow_new_body_actions=false;f.allow_test_setup_body_actions=true;
  PerceptionDetection d;d.detection.class_id=c.object_targets.backboard_class_id;
  d.detection.confidence=1;d.detection.box={140,90,20,20};
  d.center_depth_m=d.left_depth_m=d.right_depth_m=.5;f.detections={d};
  auto out=core.StepPerception(f);const auto pose=out.mission.command.action_id;
  f.now_sec=.1;f.delivery_feedback={pose,true,true,false};core.StepPerception(f);
  for(int i=2;i<=40;++i) {f.now_sec=.1*i;f.delivery_feedback={};out=core.StepPerception(f);}
  assert(out.mission.goal.mode==GoalMode::kFineAdjust && out.mission.goal.tracked.stable);
  f.detections[0].center_depth_m.reset();
  f.detections[0].left_depth_m.reset();f.detections[0].right_depth_m.reset();
  f.now_sec+=.1;out=core.StepPerception(f);
  assert(!out.perception.targets.backboard);
  assert(out.mission.goal.mode==GoalMode::kFineRecovery);
  assert(out.mission.goal.action.action==MissionAction::kNone);
}
void Defaults() {
  auto c=LoadDefaultAlgorithmConfig();
  assert(c.transition_settle_sec==.6 && c.fine_motion_settle_sec==0 && c.walk_inter_action_hold_sec==0);
  assert(c.ball.down_one_step_v_px==0 && c.ball.fine_mode_v_px==10);
  assert(c.ball.recovery_walk_turn_max==3 && c.goal.recovery_turn_max==3);
  for(int field=0;field<4;++field) {
    auto bad=c;
    if(field==0)bad.transition_settle_sec=-1;
    if(field==1)bad.fine_motion_settle_sec=std::numeric_limits<double>::quiet_NaN();
    if(field==2)bad.walk_inter_action_hold_sec=std::numeric_limits<double>::infinity();
    if(field==3)bad.goal.recovery_turn_max=-1;
    bool rejected=false;try{ValidateAlgorithmConfig(bad);}catch(...){rejected=true;}
    assert(rejected);
  }
}
}
int main() {
  Defaults();VerifiedFineWindows();InvalidAndNearWindows();GoalEdges();ImageDetectionWithMissingDepth();
  for(bool left:{false,true})GoalPairHasOneObservation(left);
  for(bool fine:{false,true})for(bool left:{false,true})GoalRecovery(fine,left);
  PoseSettleAndConcurrentObservation();
  FineObservationTiming(0);FineObservationTiming(.25);FineObservationTiming(.25,true);
}
