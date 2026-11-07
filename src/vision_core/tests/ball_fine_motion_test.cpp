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
void TaperedPickupBounds() {
  auto c=Config();
  c.pickup_top_inset_px=5.0;
  // Preserve the original outer rectangle while narrowing only the top.
  assert(c.PickupContains({310,130}));
  assert(c.PickupContains({385,130}));
  assert(c.PickupContains({315,80}));
  assert(c.PickupContains({385-5,80}));
  assert(!c.PickupContains({310,80}));
  assert(!c.PickupContains({385,80}));
  assert(c.PickupLeftAt(105)>310 && c.PickupRightAt(105)<385);
  BallFineMotionConfig motion;
  motion.right_delta_px={-5,0};
  const auto plan=PlanBallFineMotion(motion,{386,80},
      310,385,80,130,640,480,3,c.pickup_top_inset_px);
  assert(plan.repetitions>0 && plan.action==MissionAction::kRightSideStep);
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
  switch(action) {
    case MissionAction::kStepForwardHalf:start={347.5,20};c.fine_motion.forward_delta_px={0,20};break;
    case MissionAction::kStepBack:start={347.5,190};c.fine_motion.backward_delta_px={0,-20};break;
    case MissionAction::kLeftSideStep:start={200,105};c.fine_motion.left_delta_px={40,0};break;
    default:start={500,105};c.fine_motion.right_delta_px={-50,0};break;
  }
  const int execute=action==MissionAction::kStepForwardHalf ? 3 : 2;
  BallController ball(c,3);ball.StartTestExecute();
  const CameraFeedback down{CameraMode::kBallView,true};
  ball.UseVerifiedObservation(Target(start.u,start.v),std::deque<bool>(10,true),640,480);
  auto out=ball.Compute({},640,480,0,down,{},false);
  assert(out.mode==BallMode::kFineAdjustForPickup && out.action.action==action);
  assert(ball.FineMotionRepetitions()==0);
  auto preview=ball;out=preview.Compute(Target(start.u,start.v),640,480,.01,down,{});
  assert(out.mode==BallMode::kFineMotionRepeat &&
         preview.FineMotionRepetitions()==execute);
  assert(ball.FineMotionRepetitions()==0);
  out=ball.Compute(Target(start.u,start.v),640,480,.02,down,{});
  assert(out.mode==BallMode::kFineMotionRepeat &&
         out.action.action==action && ball.FineMotionRepetitions()==execute);
  ActionExecutionFeedback busy;busy.action_active=true;
  out=ball.Compute(Target(347.5,105),640,480,.1,down,busy);
  assert(out.action.action==MissionAction::kNone &&
         ball.FineSideActionsRemaining()==execute);
  auto changed=c;changed.fine_motion={};changed.fine_motion.max_repetitions=1;
  ball.UpdateTuning(changed); // A committed batch must not be retuned.
  ActionExecutionFeedback done;done.action_done=true;
  ball.Compute({},640,480,.2,down,done,false);
  assert(ball.FineSideActionsRemaining()==execute-1);
  ball.Compute({},640,480,.21,down,done,false);
  assert(ball.FineSideActionsRemaining()==execute-1);
  out=ball.Compute({},640,480,.3,down,{});
  assert(out.action.action==action && ball.FineMotionRepetitions()==execute);
  out=ball.Compute({},640,480,.4,down,done);
  if(execute==3) {
    assert(out.action.action==action && ball.FineSideActionsRemaining()==1);
    out=ball.Compute({},640,480,.5,down,done);
  }
  assert(out.mode==BallMode::kFineAdjustForPickup &&
         out.action.action==MissionAction::kNone);
  assert(!ball.ExecutionObservationReady() && ball.FineMotionRepetitions()==0);
  ball.Reset();assert(ball.FineSideActionsRemaining()==0);
}
void LegacyFallback() {
  auto c=Config();BallController ball(c,3);ball.StartTestExecute();
  ball.UseVerifiedObservation(Target(500,105),std::deque<bool>(10,true),640,480);
  auto b=ball.Compute({},640,480,0,{CameraMode::kBallView,true},{});
  assert(b.mode==BallMode::kFineAdjustForPickup &&
         b.action.action==MissionAction::kRightSideStep);
  c.fine_motion.max_repetitions=1;BallController single(c,3);single.StartTestExecute();
  single.UseVerifiedObservation(Target(500,105),std::deque<bool>(10,true),640,480);
  b=single.Compute({},640,480,0,{CameraMode::kBallView,true},{});
  assert(b.mode==BallMode::kFineAdjustForPickup && b.action.action==MissionAction::kRightSideStep);
  // An unmeasured side never inherits the legacy blind pair when a partial
  // calibration is active and the measured direction cannot help.
  c.fine_motion.max_repetitions=3;c.fine_motion.forward_delta_px={0,20};
  BallController partial(c,3);partial.StartTestExecute();
  partial.UseVerifiedObservation(Target(500,105),std::deque<bool>(10,true),640,480);
  b=partial.Compute({},640,480,0,{CameraMode::kBallView,true},{});
  assert(b.mode==BallMode::kFineAdjustForPickup && b.action.action==MissionAction::kRightSideStep);
}
void Transport(bool manual,bool pause) {
  auto c=LoadDefaultAlgorithmConfig();c.ball=Config();
  c.test_mission=MissionType::kBall;c.test_stage=2;
  c.enable_goal=c.enable_hurdle=false;
  c.fine_motion_settle_sec=.2;c.ball.fine_motion.right_delta_px={-50,0};
  MissionController core(c);MissionFrameInput f;
  f.image_width=640;f.image_height=480;f.command_transport_enabled=true;
  f.camera_feedback={CameraMode::kBallView,true};f.ball_target=Target(500,105);
  MissionFrameResult out;std::uint64_t setup_done=0;
  for(int i=0;i<=200;++i) {
    f.now_sec=.05*i;f.delivery_feedback={};
    if(out.command.action_id && out.command.action_id!=setup_done) {
      setup_done=out.command.action_id;
      f.delivery_feedback={setup_done,true,true,false};
    }
    out=core.Step(f);
    if(out.ball.mode==BallMode::kFineMotionRepeat) break;
  }
  assert(out.command.action==MissionAction::kRightSideStep &&
         out.ball.fine_motion_repetitions==2 && out.repeat_sequence);
  const auto a=out.command.action_id;
  f.manual_observation_control=manual;
  f.allow_new_commands=f.allow_new_body_actions=!pause;
  f.now_sec+=.1;f.delivery_feedback={a,true,true,false};out=core.Step(f);
  if(pause) {
    assert(out.command.action_id==0 && out.ball.fine_motion_remaining==1);
    f.now_sec+=.01;out=core.Step(f);
    assert(out.ball.fine_motion_remaining==1);
    f.allow_new_commands=f.allow_new_body_actions=true;
    f.delivery_feedback={};f.now_sec+=.01;out=core.Step(f);
  }
  const auto b=out.command.action_id;
  assert(b && b!=a && out.command.action==MissionAction::kRightSideStep);
  f.now_sec+=.05;f.delivery_feedback={b,true,true,false};out=core.Step(f);
  assert(out.command.action_id==0 && !out.repeat_sequence &&
         out.ball.fine_motion_remaining==0);
  const auto done_at=f.now_sec;
  for(int i=1;i<=10;++i) {
    f.now_sec=done_at+.05*i;f.delivery_feedback={};
    f.begin_manual_observation=manual && i==1;
    out=core.Step(f);
    assert(out.command.action_id==0 && !out.repeat_sequence);
  }
  core.Reset(true);
}

}
int main() {
  CountsAndCrossAxis();BoundsAndUncalibrated();TaperedPickupBounds();LegacyFallback();
  for (auto action : {MissionAction::kStepForwardHalf,MissionAction::kStepBack,
      MissionAction::kLeftSideStep,MissionAction::kRightSideStep}) DirectRepeat(action);
  Transport(false,false);Transport(true,false);Transport(false,true);
}
