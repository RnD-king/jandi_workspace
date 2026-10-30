#include "vision_core/config_loader.hpp"
#include <stdexcept>

using namespace vision_core;
namespace {
void Check(bool ok,const char* message) { if(!ok) throw std::runtime_error(message); }
ObjectTarget Target(Point2 p) {
  ObjectTarget t;t.center_px=p;t.box_px={p.u-10,p.v-10,20,20};
  t.width_px=t.height_px=20;t.area_px=400;t.confidence=1;return t;
}
BallConfig Config() {
  auto c=LoadDefaultAlgorithmConfig().ball;
  c.stable_window=10;c.stable_min_hits=7;c.lost_frames=3;
  c.pickup_u1_px=300;c.pickup_u2_px=400;c.pickup_v1_px=100;c.pickup_v2_px=150;
  c.down_recovery_max_actions=3;c.recovery_observation_sec=1;
  // FORWARD centerline deliberately disagrees with the pickup rectangle.
  c.far_u_des_px=100;c.fine_side_two_step_margin_px=1e6;
  return c;
}
struct Driver {
  BallController core;
  BallResult out;
  double now{};
  explicit Driver():core(Config(),30) { core.StartTestExecute(); }
  void Tick(std::optional<ObjectTarget> t={},ActionExecutionFeedback fb={},bool allow=true) {
    now+=.1;out=core.Compute(t,640,480,now,{CameraMode::kDown,true},fb,allow);
  }
  void Lose(Point2 p) {
    for(int i=0;i<10;++i) Tick(Target(p),{},false);
    for(int i=0;i<3;++i) Tick({}, {},false);
    Check(out.mode==BallMode::kBallRecoveryDown,"DOWN loss did not enter recovery");
  }
  void Observe(std::optional<ObjectTarget> t={}) {
    for(int i=0;i<12 && out.mode==BallMode::kBallRecoveryDown;++i) Tick(t,{},false);
  }
};
void DirectionAndLimit(Point2 p,MissionAction expected) {
  Driver d;d.Lose(p);
  for(int n=1;n<=3;++n) {
    d.Tick({}, {},false);
    Check(d.out.action.action==expected && d.core.RecoveryStepsCompleted()==n-1,
          "DOWN search ignored pickup rectangle or counted an unissued action");
    d.Tick();Check(d.out.action.action==expected,"DOWN search changed frozen direction");
    for(int i=0;i<10;++i)d.Tick(Target({350,120}),{false,true,false});
    Check(d.out.mode==BallMode::kBallRecoveryDown && d.core.RecoveryStepsCompleted()==n-1,
          "moving sighting bypassed DONE or consumed a recovery attempt");
    d.Tick({}, {true,false,false});
    Check(d.core.RecoveryStepsCompleted()==n,"actual FINE DONE did not count once");
    d.Tick({}, {true,false,false});
    Check(d.core.RecoveryStepsCompleted()==n,"duplicate DONE consumed a second attempt");
    d.Observe();
  }
  Check(d.out.mode==BallMode::kFailed,"DOWN search did not fail after three real actions and observations");
  for(int i=0;i<15;++i) {
    d.Tick(Target(p));Check(d.out.mode==BallMode::kFailed && d.out.action.action==MissionAction::kNone,
                           "failed DOWN search restarted without Reset");
  }
}
void ReacquireAndReset() {
  Driver d;d.Lose({250,125});d.Tick();d.Tick({}, {true,false,false});
  d.Observe(Target({350,80}));
  Check(d.out.mode==BallMode::kFineAdjustForPickup,"stationary reacquisition did not resume FINE");
  d.Tick(Target({350,80}));
  Check(d.out.action.action==MissionAction::kStepForwardHalf,"reacquired position did not replace lost position");
  d.core.Reset();d.core.StartTestExecute();
  for(int i=0;i<3;++i)d.Tick({}, {},false);
  d.Tick();Check(d.out.action.action==MissionAction::kNone,"Reset retained the previous DOWN search direction");
}
void LastSeenInsideBox() {
  Driver d;d.Lose({350,125});
  for(int i=0;i<40 && d.out.mode!=BallMode::kFailed;++i) {
    d.Tick();Check(d.out.action.action==MissionAction::kNone,
                   "lost ball inside pickup rectangle triggered blind movement/pickup");
  }
  Check(d.out.mode==BallMode::kFailed,"inside-box loss did not exhaust three stationary attempts");
}
}
int main() {
  DirectionAndLimit({250,125},MissionAction::kLeftSideStep);
  DirectionAndLimit({450,125},MissionAction::kRightSideStep);
  DirectionAndLimit({350,50},MissionAction::kStepForwardHalf);
  DirectionAndLimit({350,200},MissionAction::kStepBack);
  // Both axes outside: normalized excess determines the same priority as FINE.
  DirectionAndLimit({250,0},MissionAction::kStepForwardHalf);
  ReacquireAndReset();LastSeenInsideBox();
}
