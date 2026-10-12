#include "vision_core/config_loader.hpp"
#include "vision_core/object_observation_window.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>

using namespace vision_core;
namespace {
ObjectTarget Target(double u=.5,double v=.7) {
  ObjectTarget t; t.center_px={u*640,v*480};
  t.box_px={u*640-10,v*480-10,20,20}; t.width_px=t.height_px=20;
  t.area_px=400; t.confidence=1; return t;
}
struct Trial {
  MissionController core;
  MissionFrameInput in;
  MissionFrameResult out;
  int permit=0;
  explicit Trial(MissionControllerConfig c):core(c) {
    in.image_width=640; in.image_height=480; in.command_transport_enabled=true;
    in.advance_line_fsm=false; in.allow_new_body_actions=false;
  }
  void Tick(bool done=false,std::uint64_t duplicate=0) {
    in.now_sec+=.1;
    const auto previous=out.command.action_id;
    in.delivery_feedback={};
    if (done && previous) in.delivery_feedback={previous,true,true,false};
    if (duplicate) in.delivery_feedback={duplicate,true,true,false};
    in.allow_new_body_actions=permit!=0 && core.RepeatScope()==permit;
    out=core.Step(in);
    if (out.repeat_scope!=permit) permit=0;
  }
  MissionFrameResult Preview() const {
    auto copy=core; auto proposed=in;
    proposed.delivery_feedback={}; proposed.allow_new_body_actions=true;
    return copy.Step(proposed);
  }
  void Commit() {
    const auto expected=Preview();
    assert(expected.command.command_type==CommandType::kAction);
    auto committed=in; committed.delivery_feedback={}; committed.allow_new_body_actions=true;
    out=core.Step(committed); permit=out.repeat_scope;
    assert(out.command.action_id==expected.command.action_id);
    assert(out.command.action==expected.command.action);
    assert(out.repeat_scope==expected.repeat_scope);
  }
};
MissionControllerConfig Config() {
  auto c=LoadDefaultAlgorithmConfig();
  c.enable_hurdle=c.enable_goal=false; c.post_motion_observation_sec=0;
  c.ball.upper_acquire_v_norm=1;
  c.ball.tilt_down_window=c.ball.tilt_down_min_hits=1;
  c.ball.fine_settle_duration_sec=0;
  c.goal.fine_settle_duration_sec=0;
  return c;
}
Trial BallSearch() {
  Trial t(Config()); t.in.ball_target=Target(.5,.85);
  for(int i=0;i<7;++i)t.Tick();
  assert(t.out.ball.mode==BallMode::kWaitCameraDown);
  assert(t.out.command.camera_request==CameraRequest::kDown);
  t.in.camera_feedback={CameraMode::kDown,true}; t.in.ball_target.reset(); t.Tick(); t.Tick();
  assert(t.out.ball.mode==BallMode::kDownSearch);
  assert(t.out.command.action_id==0);
  return t;
}
void TestCameraLatchSurvivesLoss() {
  auto c=Config(); c.ball.stable_window=c.ball.stable_min_hits=1; c.ball.lost_frames=1;
  BallController ball(c.ball,3); ActionExecutionFeedback busy; busy.action_active=true;
  ActionExecutionFeedback done; done.action_done=true;
  ball.Compute(Target(.5,.3),640,480,0,{},{});
  auto b=ball.Compute(Target(.5,.85),640,480,.1,{},busy);
  assert(b.mode==BallMode::kApproachBall && b.camera_request==CameraRequest::kNone);
  b=ball.Compute({},640,480,.2,{},busy); assert(b.mode==BallMode::kApproachBall);
  b=ball.Compute({},640,480,.3,{},done); assert(b.camera_request==CameraRequest::kDown);
  c.hurdle.stable_window=c.hurdle.stable_min_hits=1; c.hurdle.lost_frames=1;
  c.hurdle.acquire_min_v_norm=0; c.hurdle.tilt_trigger_window=c.hurdle.tilt_trigger_min_hits=1;
  HurdleController hurdle(c.hurdle,3);
  hurdle.Compute(Target(.5,.3),640,480,0,{},{});
  hurdle.Compute(Target(.5,.85),640,480,.1,{},busy);
  auto h=hurdle.Compute({},640,480,.2,{},busy); assert(h.mode==HurdleMode::kApproach);
  h=hurdle.Compute({},640,480,.3,{},done); assert(h.camera_request==CameraRequest::kDown);
  c.goal.stable_window=c.goal.stable_min_hits=1;
  GoalController goal(c.goal,3); goal.StartForwardTrial();
  goal.Compute({},Target(),{},640,480,.1,true,{},busy,true,.7);
  auto g=goal.Compute({}, {}, {},640,480,.2,true,{},busy);
  assert(g.mode==GoalMode::kForwardApproach);
  g=goal.Compute({}, {}, {},640,480,.3,true,{},done);
  assert(g.camera_request==CameraRequest::kGoal);
}
void TestWindowAndDeadline() {
  ObjectObservationWindow window;
  window.Add(Target(.1),10); window.Add(std::nullopt,10); window.Add(Target(.9),10);
  assert(std::abs(window.Mean()->center_px.u/640-.7)<1e-9);
  auto t=BallSearch(); const auto p=t.Preview();
  assert(p.command.action==MissionAction::kStepForwardOne);
  assert(t.Preview().command.action_id==p.command.action_id);
  assert(t.core.RepeatScope()==p.repeat_scope);
  // Human PREVIEW time must not start the real deadline.
  t.in.now_sec+=100; t.Commit(); assert(t.out.command.action==MissionAction::kStepForwardOne);
  for(int i=0;i<9;++i)t.Tick(true);
  assert(t.out.ball.mode==BallMode::kDownSearch);
  t.in.now_sec+=10; t.Tick(true);
  assert(t.out.ball.mode==BallMode::kFailed && t.out.command.action_id==0 && t.permit==0);
  t.in.ball_target=Target(); for(int i=0;i<10;++i)t.Tick();
  assert(t.out.ball.mode==BallMode::kFailed && t.out.command.action_id==0);
}
Trial PickAndBack() {
  auto t=BallSearch(); t.Commit();
  std::uint64_t old_id=0;
  for(int i=0;i<7;++i) {
    t.in.ball_target=Target(); old_id=t.out.command.action_id;
    t.Tick(i%2==1);
    assert(t.out.ball.mode==BallMode::kDownSearch); // Finish accepted motion first.
  }
  t.Tick(true); assert(t.out.ball.mode==BallMode::kFineAdjustForPickup);
  assert(t.out.command.action_id==0 && t.permit==0);
  t.Tick(false,old_id); // Old DONE cannot advance a new phase.
  const auto before=t.out.ball.pickup_attempt_count;
  assert(t.Preview().command.action==MissionAction::kPickBall);
  assert(t.Preview().ball.pickup_attempt_count==before+1);
  assert(t.out.ball.pickup_attempt_count==before);
  t.Commit(); assert(t.out.command.action==MissionAction::kPickBall);
  t.in.ball_target.reset(); t.Tick(true);
  assert(t.out.ball.mode==BallMode::kVerifyPickupObservation);
  assert(t.out.command.action_id==0); // RECATCH must not precede verification.
  for(int i=0;i<10;++i)t.Tick();
  assert(t.out.ball.has_ball && t.out.command.action_id==0);
  assert(t.Preview().command.action==MissionAction::kRecatch);
  t.Commit(); t.Tick(true);
  assert(t.out.command.action_id==0); // DEFAULT requires another commit.
  assert(t.Preview().command.action==MissionAction::kDefaultPosition);
  t.Commit(); t.Tick(true);
  assert(t.out.ball.mode==BallMode::kPostPickupBackAway && t.permit==0);
  t.Commit(); assert(t.out.command.action==MissionAction::kStepBack);
  old_id=t.out.command.action_id;
  t.Tick(true); assert(t.out.command.action==MissionAction::kStepBack);
  const auto second=t.out.command.action_id; assert(second!=old_id);
  t.Tick(false,old_id); assert(t.out.command.action_id==second);
  t.Tick(true); assert(t.out.command.action==MissionAction::kStepBack);
  t.Tick(true); assert(t.out.command.action_id==0 && t.permit==0);
  assert(t.out.command.camera_request==CameraRequest::kForward);
  t.in.camera_feedback={CameraMode::kForward,true}; t.Tick();
  assert(t.out.ball.mode==BallMode::kPostPickupLineRecovery);
  return t;
}
void TestAlternatingBoundedRecovery() {
  auto right=PickAndBack(); auto left=right;
  assert(left.core.SetTestMissionIndices(2,1));
  left.Commit(); assert(left.out.command.action==MissionAction::kTurnLeft);
  right.Commit(); assert(right.out.command.action==MissionAction::kTurnRight);
  auto found_on_fifth=right;
  for(int i=0;i<5;++i)right.Tick(true);
  assert(right.out.recovery_turns_completed==5 && right.out.recovery_failed);
  assert(right.out.command.action_id==0 && right.permit==0);
  right.in.line_centers={{420,450},{420,350},{420,250}};
  for(int i=0;i<10;++i)right.Tick();
  assert(right.out.recovery_failed && right.out.command.action_id==0);
  for(int i=0;i<4;++i)found_on_fifth.Tick(true);
  found_on_fifth.in.line_centers={{420,450},{420,350},{420,250}};
  for(int i=0;i<7;++i)found_on_fifth.Tick();
  assert(found_on_fifth.out.command.action_id!=0);
  found_on_fifth.Tick(true);
  assert(!found_on_fifth.out.recovery_failed);
  assert(found_on_fifth.out.recovery_verifying);
  assert(found_on_fifth.out.active_mission==MissionType::kBall);
  auto lost_after_rotation = found_on_fifth;
  lost_after_rotation.in.line_centers.clear();
  for (int i=0;i<11;++i) lost_after_rotation.Tick();
  assert(lost_after_rotation.out.recovery_failed);
  assert(lost_after_rotation.out.recovery_turns_completed==5);
  assert(lost_after_rotation.out.command.action_id==0);
  for (int i=0;i<11;++i) found_on_fifth.Tick();
  assert(found_on_fifth.out.active_mission==MissionType::kLine);
  assert(found_on_fifth.out.command.action_id==0 && found_on_fifth.permit==0);
}
void TestGoalForwardTrial() {
  auto c=Config(); c.enable_ball=false; c.enable_goal=true;
  c.goal_trial_start_forward=c.initial_has_ball=true;
  c.test_goal_mission_index=2;
  Trial t(c); t.in.backboard_target=Target(.5,.3); t.in.backboard_center_depth_m=2;
  for(int i=0;i<7;++i)t.Tick();
  assert(t.out.goal.mode==GoalMode::kForwardApproach);
  assert(t.out.command.camera_request==CameraRequest::kNone);
  t.Commit(); assert(t.out.command.action==MissionAction::kStepForwardFour);
  t.in.backboard_center_depth_m=.7; t.Tick(true);
  assert(t.out.command.camera_request==CameraRequest::kGoal);
  t.in.camera_feedback={CameraMode::kGoal,true}; t.Tick();
  t.in.goal_pose={true,0,.5,0,1};
  for(int i=0;i<8;++i)t.Tick();
  assert(t.out.goal.mode==GoalMode::kFineAdjust);
  assert(t.Preview().command.action==MissionAction::kShoot);
  t.Commit(); t.Tick(true); assert(t.out.command.action_id==0);
  t.Commit(); assert(t.out.command.action==MissionAction::kStepBack);
  t.Tick(true); assert(t.out.command.action==MissionAction::kStepBack);
  t.Tick(true); assert(t.out.command.action_id==0 && t.permit==0);
  assert(t.out.command.camera_request==CameraRequest::kForward);
  t.in.camera_feedback={CameraMode::kForward,true}; t.Tick();
  t.Commit(); assert(t.out.command.action==MissionAction::kTurnLeft);
  t.in.line_centers={{420,450},{420,350},{420,250}};
  for(int i=0;i<7;++i)t.Tick(); t.Tick(true);
  assert(t.out.recovery_verifying && !t.out.trial_complete);
  for(int i=0;i<11;++i)t.Tick();
  assert(t.out.trial_complete && t.out.command.action_id==0);
  t.Tick(); assert(t.out.trial_complete && t.out.command.action_id==0);
  assert(t.core.SetTestMissionIndices(1,3));
  t.core.Reset(true); t.Tick(); assert(!t.out.trial_complete && t.out.goal_mission_index==3);
}
void TestHurdleSearchAndContact() {
  auto c=Config(); c.enable_ball=false; c.enable_hurdle=true;
  c.hurdle.acquire_min_v_norm=0; c.hurdle.tilt_trigger_window=c.hurdle.tilt_trigger_min_hits=1;
  c.hurdle.recovery_settle_duration_sec=0;
  Trial t(c); t.in.hurdle_target=Target(.5,.85);
  for(int i=0;i<7;++i)t.Tick();
  assert(t.out.command.camera_request==CameraRequest::kDown);
  t.in.hurdle_target.reset(); t.in.camera_feedback={CameraMode::kDown,true}; t.Tick(); t.Tick();
  t.Commit(); assert(t.out.command.action==MissionAction::kStepForwardOne);
  for(int i=0;i<7;++i) {t.in.hurdle_target=Target(.5,.9); t.Tick(i%2==1);}
  t.Tick(true); assert(t.out.hurdle.mode==HurdleMode::kFineAdjust && t.permit==0);
  t.Tick(); t.Commit(); assert(t.out.hurdle.mode==HurdleMode::kContactWalk);
  assert(t.out.command.action==MissionAction::kStepForwardHalf);
  for(int i=0;i<3;++i)t.Tick(true);
  assert(t.out.hurdle.mode==HurdleMode::kCross && t.out.command.action_id==0 && t.permit==0);
  t.Commit(); assert(t.out.command.action==MissionAction::kHurdle);
}
} // namespace
int main() {
  // Distance thresholds keep raw GOAL pose geometry, changing only primitive length.
  auto gc=LoadDefaultAlgorithmConfig().goal;
  gc.stable_window=gc.stable_min_hits=1; gc.fine_settle_duration_sec=0;
  GoalController goal(gc,3); goal.StartForwardTrial();
  goal.Compute({},Target(),{true,0,.7,0,1},640,480,0,true,{}, {},true,.7);
  goal.Compute({}, {}, {},640,480,.1,true,{CameraMode::kGoal,true},{});
  goal.Compute({},Target(),{true,0,.7,0,1},640,480,.2,true,{CameraMode::kGoal,true},{});
  auto large=goal.Compute({},Target(),{true,0,.8,0,1},640,480,.3,true,{CameraMode::kGoal,true},{});
  assert(large.fine_decision_pose_used && large.action.action==MissionAction::kStepForwardTwo);
  auto medium=goal.Compute({},Target(),{true,0,.65,0,1},640,480,.4,true,{CameraMode::kGoal,true},{});
  assert(medium.fine_decision_pose_used && medium.action.action==MissionAction::kStepForwardOne);
  TestCameraLatchSurvivesLoss(); TestWindowAndDeadline(); TestAlternatingBoundedRecovery();
  TestGoalForwardTrial(); TestHurdleSearchAndContact();
}
