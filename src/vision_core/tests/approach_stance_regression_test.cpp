#include "vision_core/config_loader.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
using namespace vision_core;
static CruiseDecision StraightLine() { CruiseDecision d; d.applicable=true; d.direction=CruiseDirection::kStraight; return d; }
namespace {
ObjectTarget Target(double u,double v) {
  ObjectTarget t; t.center_px={u,v}; t.box_px={u-10,v-10,20,20};
  t.width_px=t.height_px=20; t.area_px=400; t.confidence=1; return t;
}
void BallStages() {
  auto c=LoadDefaultAlgorithmConfig().ball;
  c.stable_window=c.stable_min_hits=1;
  c.lost_frames=1;
  // Isolate legacy two-step / one-step / fine transitions from live tuning values.
  c.down_one_step_v_px=216.0; c.fine_mode_v_px=278.4;
  c.pickup_v1_px=307.2; c.pickup_v2_px=364.8;
  BallController ball(c,3);
  auto r=ball.Compute(Target(380,400),640,480,0,{},{});
  assert(r.mode==BallMode::kApproachTwoBeforeDown && r.action.action==MissionAction::kStepForwardTwo);
  ActionExecutionFeedback busy; busy.action_active=true;
  r=ball.Compute({},640,480,.1,{},busy); assert(r.camera_request==CameraRequest::kNone);
  ActionExecutionFeedback done; done.action_done=true;
  r=ball.Compute({},640,480,.2,{},done); assert(r.camera_request==CameraRequest::kDown);
  CameraFeedback down{CameraMode::kDown,true};
  r=ball.Compute({},640,480,.3,down,{}); assert(r.mode==BallMode::kDownSearch);
  r=ball.Compute(Target(320,200),640,480,.4,down,{}); assert(r.mode==BallMode::kCoarseAdjust);
  r=ball.Compute(Target(320,200),640,480,.5,down,{}); assert(r.action.action==MissionAction::kStepForwardTwo);
  r=ball.Compute(Target(320,250),640,480,.6,down,{}); assert(r.action.action==MissionAction::kStepForwardOne);
  r=ball.Compute(Target(320,320),640,480,.7,down,{});
  assert(r.mode==BallMode::kWaitFinePose && r.action.action==MissionAction::kDefaultPoseMode);
  r=ball.Compute(Target(320,320),640,480,.8,down,done); assert(r.mode==BallMode::kFineAdjustForPickup);
  r=ball.Compute(Target(320,200),640,480,.9,down,{}); assert(r.action.action==MissionAction::kStepForwardHalf);
}
void HurdlePostWalkPolicy(int steps) {
  auto c=LoadDefaultAlgorithmConfig().hurdle;
  c.stable_window=c.stable_min_hits=1; c.lost_frames=1;
  HurdleController hurdle(c,3);
  const double v=steps==4 ? c.approach_short_v_px-20 :
      steps==3 ? (c.approach_short_v_px+c.approach_two_step_v_px)/2 : c.approach_two_step_v_px+20;
  auto r=hurdle.Compute(Target(100,v),640,480,0,{},{},StraightLine());
  assert(r.action.action==(steps==4 ? MissionAction::kStepForwardFour :
      steps==3 ? MissionAction::kStepForwardThree : MissionAction::kStepForwardTwo));
  ActionExecutionFeedback done; done.action_done=true;
  r=hurdle.Compute({},640,480,.1,{},done,StraightLine());
  if (steps==2) { assert(r.camera_request==CameraRequest::kDown); return; }
  assert(r.camera_request==CameraRequest::kNone);
  ActionExecutionFeedback observed; observed.observation_complete=true;
  r=hurdle.Compute({},640,480,.2,{},observed,StraightLine());
  assert(r.mode==HurdleMode::kWaitCameraDown && r.camera_request==CameraRequest::kDown);
}
void ResetWaitsForBoth() {
  MissionController core; core.BeginPhysicalReset();
  auto r=core.StepPhysicalReset({}, {CameraMode::kDown,true},0);
  const auto id=r.command.action_id;
  assert(id && r.command.action==MissionAction::kWalkMode && r.command.camera_request==CameraRequest::kForward);
  r=core.StepPhysicalReset({id,true,false,false},{CameraMode::kForward,true},.1);
  assert(!r.physical_reset_complete && !core.FeetReadyToStart());
  r=core.StepPhysicalReset({id,true,true,false},{CameraMode::kForward,true},.2);
  assert(r.physical_reset_complete && core.FeetReadyToStart());
}
void LowFpsDoesNotConsumeAttempts() {
  StationaryObjectRecovery observer; observer.Begin(0);
  for(int i=0;i<9;++i) {
    assert(observer.Observe(true,.2*i,10,7,1,5)==StationaryObjectRecovery::Result::kWaiting);
    assert(observer.Completed()==0);
  }
  assert(observer.Observe(true,1.8,10,7,1,5)==StationaryObjectRecovery::Result::kFound);
}
}
int main() {
  const auto c=LoadDefaultAlgorithmConfig();
  assert(c.ball.upper_acquire_v_px==470 && c.hurdle.upper_acquire_v_px==470);
  assert(c.mission_recovery_max_turns==10);
  BallStages(); for(int steps:{4,3,2}) HurdlePostWalkPolicy(steps);
  ResetWaitsForBoth(); LowFpsDoesNotConsumeAttempts();
}
