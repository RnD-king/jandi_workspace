#include "vision_core/config_loader.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>

using namespace vision_core;
namespace {
ObjectTarget Target(double u,double v) {
  ObjectTarget t;t.center_px={u,v};t.box_px={u-10,v-10,20,20};
  t.width_px=t.height_px=20;t.area_px=400;t.confidence=1;return t;
}
BallFineMotionPlan Plan(const BallFineMotionConfig& c,Point2 p) {
  return PlanBallFineMotion(c,p,310,385,80,130,640,480);
}
BallConfig Config() {
  auto c=LoadDefaultAlgorithmConfig().ball;
  c.pickup_u1_px=310;c.pickup_u2_px=385;c.pickup_v1_px=80;c.pickup_v2_px=130;
  c.stable_window=10;c.stable_min_hits=7;c.down_search_min_hits=7;
  c.fine_motion={};return c;
}
void CountsAndCrossAxis() {
  BallFineMotionConfig c;c.right_delta_px={-50,0};
  assert(Plan(c,{420,105}).repetitions==1);
  assert(Plan(c,{470,105}).repetitions==2);
  auto p=Plan(c,{500,105});
  assert(p.action==MissionAction::kRightSideStep && p.repetitions==3);
  assert(p.predicted_center_px.u==350 && p.predicted_center_px.v==105);
  c.max_repetitions=2;assert(Plan(c,{500,105}).repetitions==2);
  c.max_repetitions=1;assert(Plan(c,{500,105}).repetitions==1);
  // Both coordinates choose the direction, not just the largest raw axis.
  c={};c.right_delta_px={-50,-20};c.forward_delta_px={-40,25};
  p=Plan(c,{500,30});
  assert(p.action==MissionAction::kStepForwardHalf && p.repetitions==3);
  assert(p.predicted_center_px.u==380 && p.predicted_center_px.v==105);
  // Equal final error prefers fewer actions.
  c={};c.right_delta_px={-50,0};c.left_delta_px={-120,0};
  p=Plan(c,{500,105});assert(p.action==MissionAction::kLeftSideStep && p.repetitions==1);
}
void BoundsAndUncalibrated() {
  BallFineMotionConfig c;
  assert(!c.HasCalibration() && Plan(c,{500,105}).repetitions==0);
  c.right_delta_px={50,0};assert(Plan(c,{500,105}).repetitions==0); // Wrong sign.
  c.right_delta_px={-110,0};assert(Plan(c,{500,105}).repetitions==1); // Second would cross left edge.
  c.right_delta_px={-50,-20};assert(Plan(c,{500,85}).repetitions==0); // Damages an aligned axis.
  assert(Plan(c,{500,10}).repetitions==0); // Leaves image before a useful step.
  c.right_delta_px={-50,0};assert(Plan(c,{347.5,105}).repetitions==0); // Already in pickup rectangle.
  assert(PlanBallFineMotion(c,{500,105},310,385,80,130,640,480,1).repetitions==1);
}
void DirectRepeat(MissionAction action) {
  auto c=Config();Point2 start;
  switch (action) {
    case MissionAction::kStepForwardHalf:start={347.5,20};c.fine_motion.forward_delta_px={0,20};break;
    case MissionAction::kStepBack:start={347.5,190};c.fine_motion.backward_delta_px={0,-20};break;
    case MissionAction::kLeftSideStep:start={200,105};c.fine_motion.left_delta_px={40,0};break;
    default:start={500,105};c.fine_motion.right_delta_px={-50,0};break;
  }
  BallController ball(c,3);ball.StartTestExecute();
  const CameraFeedback down{CameraMode::kDown,true};
  ball.UseVerifiedObservation(Target(start.u,start.v),std::deque<bool>(10,true),640,480);
  auto b=ball.Compute({},640,480,0,down,{},false);
  assert(b.mode==BallMode::kFineAdjustForPickup && b.action.action==action);
  assert(ball.FineMotionRepetitions()==0); // Uncommitted evidence cannot freeze a plan.
  auto preview=ball;b=preview.Compute(Target(start.u,start.v),640,480,.01,down,{});
  assert(b.mode==BallMode::kFineMotionRepeat && preview.FineMotionRepetitions()==3);
  assert(ball.FineMotionRepetitions()==0);
  b=ball.Compute(Target(start.u,start.v),640,480,.02,down,{});
  assert(b.mode==BallMode::kFineMotionRepeat && b.action.action==action);
  ActionExecutionFeedback busy;busy.action_active=true;
  b=ball.Compute(Target(347.5,105),640,480,.1,down,busy);
  assert(b.action.action==MissionAction::kNone && ball.FineSideActionsRemaining()==3);
  // Retuning during an accepted chain cannot change its direction/count.
  auto changed=c;changed.fine_motion={};changed.fine_motion.max_repetitions=1;
  ball.UpdateTuning(changed);
  ActionExecutionFeedback done;done.action_done=true;
  ball.Compute({},640,480,.2,down,done,false);
  assert(ball.FineSideActionsRemaining()==2);
  ball.Compute({},640,480,.21,down,done,false); // Duplicate while paused.
  assert(ball.FineSideActionsRemaining()==2);
  b=ball.Compute({},640,480,.3,down,{});
  assert(b.action.action==action && ball.FineMotionRepetitions()==3);
  b=ball.Compute({},640,480,.4,down,done);
  assert(b.action.action==action && ball.FineSideActionsRemaining()==1);
  b=ball.Compute({},640,480,.5,down,done);
  assert(b.mode==BallMode::kFineAdjustForPickup && b.action.action==MissionAction::kNone);
  assert(!ball.ExecutionObservationReady() && ball.FineMotionRepetitions()==0);
  ball.Reset();assert(ball.FineSideActionsRemaining()==0);
}
void LegacyFallback() {
  auto c=Config();BallController ball(c,3);ball.StartTestExecute();
  ball.UseVerifiedObservation(Target(500,105),std::deque<bool>(10,true),640,480);
  auto b=ball.Compute({},640,480,0,{CameraMode::kDown,true},{});
  assert(b.mode==BallMode::kFineSidePair && b.action.action==MissionAction::kRightSideStep);
  c.fine_motion.max_repetitions=1;BallController single(c,3);single.StartTestExecute();
  single.UseVerifiedObservation(Target(500,105),std::deque<bool>(10,true),640,480);
  b=single.Compute({},640,480,0,{CameraMode::kDown,true},{});
  assert(b.mode==BallMode::kFineAdjustForPickup && b.action.action==MissionAction::kRightSideStep);
  // An unmeasured side never inherits the legacy blind pair when a partial
  // calibration is active and the measured direction cannot help.
  c.fine_motion.max_repetitions=3;c.fine_motion.forward_delta_px={0,20};
  BallController partial(c,3);partial.StartTestExecute();
  partial.UseVerifiedObservation(Target(500,105),std::deque<bool>(10,true),640,480);
  b=partial.Compute({},640,480,0,{CameraMode::kDown,true},{});
  assert(b.mode==BallMode::kFineAdjustForPickup && b.action.action==MissionAction::kRightSideStep);
}
void Transport(bool manual,bool pause) {
  auto c=LoadDefaultAlgorithmConfig();c.ball=Config();
  c.test_mission=MissionType::kBall;c.test_stage=2;c.enable_goal=c.enable_hurdle=false;
  c.fine_motion_settle_sec=.2;c.ball.fine_motion.right_delta_px={-50,0};
  MissionController core(c);MissionFrameInput f;
  f.image_width=640;f.image_height=480;f.command_transport_enabled=true;
  f.camera_feedback={CameraMode::kDown,true};f.ball_target=Target(500,105);
  MissionFrameResult r;std::uint64_t setup_done=0;
  for (int i=0;i<=200;++i) {
    f.now_sec=.05*i;f.delivery_feedback={};
    if (r.command.action_id && r.command.action_id!=setup_done) {
      setup_done=r.command.action_id;f.delivery_feedback={setup_done,true,true,false};
    }
    r=core.Step(f);
    if (r.ball.mode==BallMode::kFineMotionRepeat) break;
  }
  assert(r.command.action==MissionAction::kRightSideStep && r.ball.fine_motion_repetitions==3);
  const auto a=r.command.action_id;const int scope=r.repeat_scope;
  assert(a!=0 && scope!=0 && r.repeat_sequence);
  f.manual_observation_control=manual;f.ball_target=Target(347.5,105);
  f.allow_new_body_actions=f.allow_new_commands=!pause;
  f.now_sec+=.1;f.delivery_feedback={a,true,true,false};r=core.Step(f);
  if (pause) {
    assert(r.command.action_id==0 && r.repeat_scope==scope && r.ball.fine_motion_remaining==2);
    f.now_sec+=.01;r=core.Step(f); // Duplicate A cannot consume another step.
    assert(r.ball.fine_motion_remaining==2);
    auto changed=c;changed.ball.fine_motion.max_repetitions=1;
    core.SetLivePerceptionTuning(changed);
    f.allow_new_body_actions=f.allow_new_commands=true;f.delivery_feedback={};
    f.now_sec+=.01;r=core.Step(f);
  }
  const auto b=r.command.action_id;
  assert(b!=0 && b!=a && r.command.action==MissionAction::kRightSideStep);
  assert(!r.post_motion_observing && r.repeat_scope==scope);
  f.now_sec+=.01;f.delivery_feedback={a,true,true,false};r=core.Step(f);
  assert(r.command.action_id==b && r.ball.fine_motion_remaining==2);
  f.now_sec+=.05;f.delivery_feedback={b,true,true,false};r=core.Step(f);
  const auto third=r.command.action_id;
  assert(third!=0 && third!=a && third!=b && r.command.action==MissionAction::kRightSideStep);
  assert(!r.post_motion_observing && r.repeat_scope==scope && r.ball.fine_motion_remaining==1);
  f.now_sec+=.01;f.delivery_feedback={b,true,true,false};r=core.Step(f);
  assert(r.command.action_id==third && r.ball.fine_motion_remaining==1);
  f.now_sec+=.05;const auto done_at=f.now_sec;f.delivery_feedback={third,true,true,false};r=core.Step(f);
  assert(r.command.action_id==0 && !r.repeat_sequence && r.repeat_scope==0);
  bool finished=false;
  for (int i=1;i<=40;++i) {
    f.now_sec=done_at+.05*i;f.delivery_feedback={};f.begin_manual_observation=manual && i==1;
    r=core.Step(f);
    assert(r.command.action_id==0); // No fourth body action or automatic pickup.
    if (f.now_sec+1e-9<done_at+.2+c.post_motion_observation_sec) assert(!r.trial_complete);
    if (r.trial_complete) {finished=true;break;}
  }
  assert(finished); // One fresh observation after the entire chain.
  core.Reset(true);f.delivery_feedback={third,true,true,false};f.now_sec+=.1;r=core.Step(f);
  assert(!r.repeat_sequence && r.ball.fine_motion_remaining==0);
  assert(r.command.action_id==0 || r.command.action_id>third);
}
}
int main() {
  CountsAndCrossAxis();BoundsAndUncalibrated();LegacyFallback();
  for (auto action : {MissionAction::kStepForwardHalf,MissionAction::kStepBack,
      MissionAction::kLeftSideStep,MissionAction::kRightSideStep}) DirectRepeat(action);
  Transport(false,false);Transport(true,false);Transport(false,true);
}
