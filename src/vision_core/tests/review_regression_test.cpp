#include "vision_core/config_loader.hpp"
#include "vision_core/mission_controller.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>

using namespace vision_core;
namespace {
ObjectTarget Target(double u, double v) {
  ObjectTarget t; t.center_px={u*640,v*480}; t.rectified_center_px=t.center_px;
  t.box_px={u*640-10,v*480-10,20,20}; t.width_px=t.height_px=20;
  t.confidence=1; t.area_px=400; return t;
}
MissionFrameInput Frame(double now) {
  MissionFrameInput f; f.now_sec=now; f.image_width=640; f.image_height=480;
  f.command_transport_enabled=true;
  f.line_centers={{420,450},{420,350},{420,250}}; return f;
}
void TestClockAndGoalReservationDrain() {
  auto cfg=LoadDefaultAlgorithmConfig(); cfg.initial_has_ball=true;
  cfg.enable_ball=cfg.enable_hurdle=false;
  MissionController core(cfg);
  auto f=Frame(0); auto r=core.Step(f); const auto a=r.command.action_id;
  assert(a!=0 && r.goal_walk_remaining_sec==5);
  f=Frame(20); f.delivery_feedback={a,true,false,false}; r=core.Step(f);
  assert(r.goal_walk_elapsed_sec==0); // Dispatch/ACK wait is not execution.
  for(int i=1;i<=24;++i) { f=Frame(20+.1*i); r=core.Step(f); }
  f=Frame(22.5); f.delivery_feedback={a,true,false,true}; r=core.Step(f);
  const auto b=r.command.action_id; assert(b!=a && b!=0);
  f=Frame(22.6); f.delivery_feedback={b,true,false,false}; r=core.Step(f);
  assert(std::abs(r.goal_walk_elapsed_sec-2.6)<1e-8); // No queued-ACK double count.
  f=Frame(23); f.delivery_feedback={a,true,true,false}; r=core.Step(f);
  assert(r.command.action_id==b);
  for(int i=1;i<20;++i) { f=Frame(23+.1*i); r=core.Step(f); }
  f=Frame(25); r=core.Step(f);
  assert(r.active_mission==MissionType::kGoal && r.goal.mode==GoalMode::kWaitCameraGoal);
  assert(r.goal_walk_remaining_sec==0 && r.command.camera_request==CameraRequest::kNone);
  f=Frame(25.1); f.delivery_feedback={b,true,false,true}; r=core.Step(f);
  assert(r.command.action_id==b); // READY cannot reserve a third walk after threshold.
  f=Frame(25.3); f.delivery_feedback={b,true,true,false}; r=core.Step(f);
  assert(r.command.action_id==0 && r.command.camera_request==CameraRequest::kGoal);
  f=Frame(25.4); f.camera_feedback={CameraMode::kGoal,true}; r=core.Step(f);
  for(int i=0;i<30;++i) { f=Frame(25.5+.1*i); f.camera_feedback={CameraMode::kGoal,true}; r=core.Step(f); }
  assert(r.command.action_id==0 && r.goal.mode==GoalMode::kSearch); // Missing GOAL intentionally holds.
  assert(std::abs(r.goal_walk_elapsed_sec-5.3)<1e-8); // No camera/observation time.
}
void TestCommonLineEvidence() {
  LineGuide g; g.valid=true;
  LineGuideAccumulator a; a.Begin(1,0);
  for(int i=0;i<4;++i)a.Add(1,.5+.1*i,g);
  assert(!a.Finish(1,1));
  a.Begin(2,0); for(int i=0;i<4;++i)a.Add(2,.1*i,g);
  assert(!a.FinishAll(2));
  a.Begin(3,0); for(int i=0;i<5;++i)a.Add(3,.5+.1*i,g);
  assert(a.Finish(3,1));
  a.Begin(4,0); for(int i=0;i<5;++i)a.Add(4,.1*i,g);
  assert(a.FinishAll(4));
}
void TestClockExcludesIdleAndDeferredDone() {
  ControlCommandCoordinator clock({1,10});
  ActionRequest walk{MissionAction::kStepForwardFive,ActionCategory::kLocomotion,0,false};
  clock.Compute(MissionType::kLine,0,walk,CameraRequest::kNone,{},0);
  clock.Compute(MissionType::kLine,0,{},CameraRequest::kNone,{1,true,false,false},.2);
  clock.ObserveExecutionTime({1,true,true,false},3.2);
  clock.Compute(MissionType::kLine,0,{},CameraRequest::kNone,{1,true,false,false},4.2);
  assert(std::abs(clock.LineExecutionSeconds()-3)<1e-8);
  clock.Compute(MissionType::kLine,0,{},CameraRequest::kNone,{1,true,true,false},4.3);
  clock.Compute(MissionType::kLine,0,{},CameraRequest::kNone,{},100);
  assert(std::abs(clock.LineExecutionSeconds()-3)<1e-8);
  clock.Compute(MissionType::kLine,0,walk,CameraRequest::kNone,{},100.1);
  clock.Compute(MissionType::kLine,0,{},CameraRequest::kNone,{2,true,false,false},100.2);
  clock.Compute(MissionType::kLine,0,{},CameraRequest::kNone,{2,true,true,false},102.2);
  assert(std::abs(clock.LineExecutionSeconds()-5)<1e-8);
}
void TestManualPerceptionStationaryRecovery() {
  auto cfg=LoadDefaultAlgorithmConfig(); cfg.enable_goal=cfg.enable_hurdle=false;
  cfg.ball.upper_acquire_v_norm=1; cfg.ball.tilt_down_min_hits=1;
  MissionController core(cfg);
  PerceptionFrameInput in; in.image_width=640; in.image_height=480;
  in.manual_observation_control=true; in.command_transport_enabled=true;
  in.allow_new_body_actions=false; in.advance_line_fsm=false;
  const auto detection=[](double v) {
    PerceptionDetection d; d.detection.class_id=1; d.detection.confidence=1;
    d.detection.box={310,v*480-10,20,20}; return d;
  };
  PerceptionMissionFrameResult out;
  for(int i=0;i<10;++i) { in.now_sec+=.1; in.detections={detection(.85)}; out=core.StepPerception(in); }
  assert(out.mission.ball.mode==BallMode::kWaitCameraDown);
  in.camera_feedback={CameraMode::kDown,true}; in.detections.clear(); in.now_sec+=.1;
  core.StepPerception(in);
  for(int i=0;i<20;++i) { in.now_sec+=.1; in.detections={detection(.5)}; out=core.StepPerception(in); }
  assert(out.mission.ball.mode==BallMode::kFineAdjustForPickup);
  in.detections.clear();
  for(int i=0;i<10 && !core.NeedsStationaryObservation();++i) { in.now_sec+=.1; out=core.StepPerception(in); }
  assert(core.NeedsStationaryObservation());
  auto reacquired=core; auto found_input=in;
  for(int i=0;i<60 && out.mission.ball.mode!=BallMode::kFailed;++i) {
    in.now_sec+=.1; out=core.StepPerception(in);
    assert(!out.mission.post_motion_observing && out.mission.command.action_id==0);
  }
  assert(out.mission.ball.mode==BallMode::kFailed && out.mission.ball.recovery_observations_completed==5);
  for(int i=0;i<12;++i) {
    found_input.now_sec+=.1; found_input.detections={detection(.7)};
    out=reacquired.StepPerception(found_input);
    assert(out.mission.command.action_id==0);
  }
  assert(out.mission.ball.mode==BallMode::kFineAdjustForPickup);
  assert(out.mission.ball.recovery_observations_completed==1);
  assert(!reacquired.NeedsStationaryObservation());
}
void TestStationaryRecoveryAndNoHurdleCentering() {
  const auto c=LoadDefaultAlgorithmConfig();
  BallController ball(c.ball,3);
  CameraFeedback forward{}, down{CameraMode::kDown,true};
  double now=0; BallResult b;
  for(int i=0;i<10;++i) b=ball.Compute(Target(.5,.3),640,480,now+=.1,forward,{},false);
  for(int i=0;i<10;++i) b=ball.Compute(Target(.5,.85),640,480,now+=.1,forward,{},false);
  assert(b.mode==BallMode::kWaitCameraDown);
  ball.Compute({},640,480,now+=.1,down,{},false);
  for(int i=0;i<10;++i) b=ball.Compute(Target(.5,.5),640,480,now+=.1,down,{},false);
  for(int i=0;i<10;++i)b=ball.Compute({},640,480,now+=.1,down,{},false);
  assert(b.mode==BallMode::kBallRecoveryDown);
  for(int i=0;i<60 && b.mode!=BallMode::kFailed;++i) {
    b=ball.Compute({},640,480,now+=.1,down,{},false);
    assert(b.action.action==MissionAction::kNone);
  }
  assert(b.mode==BallMode::kFailed && ball.RecoveryObservationsCompleted()==5);

  HurdleController hurdle(c.hurdle,3); HurdleResult h; now=0;
  for(int i=0;i<10;++i)h=hurdle.Compute(Target(.05,.62),640,480,now+=.1,forward,{},false);
  assert(h.action.action==MissionAction::kStepForwardFour); // Strongly off-center still walks straight.
  for(int i=0;i<10;++i)h=hurdle.Compute(Target(.05,.85),640,480,now+=.1,forward,{},false);
  hurdle.Compute({},640,480,now+=.1,down,{},false);
  for(int i=0;i<18;++i)h=hurdle.Compute(Target(.05,.9),640,480,now+=.1,down,{},false);
  assert(h.mode==HurdleMode::kFineAdjust && h.action.action==MissionAction::kStepForwardHalf);
  for(int i=0;i<6;++i)h=hurdle.Compute({},640,480,now+=.1,down,{},false);
  assert(h.mode==HurdleMode::kRecoveryDown);
  for(int i=0;i<12;++i)h=hurdle.Compute(Target(.95,.7),640,480,now+=.1,down,{},false);
  assert(h.mode==HurdleMode::kFineAdjust && hurdle.RecoveryObservationsCompleted()==1);
  for(int i=0;i<10;++i)h=hurdle.Compute({},640,480,now+=.1,down,{},false);
  assert(h.mode==HurdleMode::kRecoveryDown);
  for(int i=0;i<60 && h.mode!=HurdleMode::kFailed;++i) {
    h=hurdle.Compute({},640,480,now+=.1,down,{},false);
    assert(h.action.action==MissionAction::kNone);
  }
  assert(h.mode==HurdleMode::kFailed && hurdle.RecoveryObservationsCompleted()==5);
}
} // namespace
int main() {
  TestClockAndGoalReservationDrain(); TestCommonLineEvidence();
  TestClockExcludesIdleAndDeferredDone(); TestManualPerceptionStationaryRecovery();
  TestStationaryRecoveryAndNoHurdleCentering();
}
