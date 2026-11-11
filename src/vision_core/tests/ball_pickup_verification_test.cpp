#include "vision_core/config_loader.hpp"
#include <stdexcept>
using namespace vision_core;
namespace {
void Check(bool ok,const char* message) { if(!ok) throw std::runtime_error(message); }
ObjectTarget Ball(double u=320,double v=340) {
  ObjectTarget t;t.center_px={u,v};t.box_px={u-10,v-10,20,20};
  t.width_px=t.height_px=20;t.area_px=400;t.confidence=1;return t;
}
BallConfig Config() {
  auto c=LoadDefaultAlgorithmConfig().ball;
  c.stable_window=10;c.stable_min_hits=7;c.pickup_verify_min_hits=3;
  c.pickup_verification_sec=1;c.pickup_polygon={{280,300},{360,300},{360,380},{280,380}};
  c.pickup_initial_shrink_percent=0;c.lost_frames=5;
  return c;
}
struct Trial {
  BallConfig config=Config();BallController core{config,3};BallResult out;
  double now=0;
  Trial() { core.StartTestExecute(); }
  void Tick(std::optional<ObjectTarget> target=Ball(),bool allow=false,
            ActionExecutionFeedback feedback={}) {
    now+=.1;out=core.Compute(target,640,480,now,{CameraMode::kBallView,true},feedback,
                            allow,false,std::nullopt,std::nullopt,true,false);
  }
  void Candidate() {
    for(int i=0;i<10;++i) Tick();
    Check(out.pickup_verifying && out.action.action==MissionAction::kNone,"candidate did not start fresh verification");
    Check(out.pickup_attempt_count==0,"candidate consumed an attempt");
  }
};
void FreshSecondAndManualCommit() {
  Trial t;t.Candidate();
  for(int i=0;i<9;++i) {
    t.Tick();Check(t.out.action.action==MissionAction::kNone,"PICK before a complete extra second");
    Check(t.out.pickup_attempt_count==0,"observation consumed an attempt");
  }
  t.Tick();Check(t.out.action.action==MissionAction::kPickBall,"fresh verified PICK was not previewable");
  Check(t.out.pickup_attempt_count==0,"PREVIEW consumed an attempt");
  Check(t.core.PickupVerificationReady() && !t.core.PickupVerificationActive(),"manual observation never completed");
  t.core.ClearObservationHistory(); // A later explicit PREVIEW replaces its coordinates.
  for(int i=0;i<10;++i)t.Tick();
  Check(t.out.action.action==MissionAction::kPickBall && t.out.pickup_attempt_count==0,
        "fresh manual PREVIEW restarted the extra second indefinitely");
  auto speculative=t.core;
  const auto preview=speculative.Compute(Ball(),640,480,t.now,{CameraMode::kBallView,true},{},true);
  Check(preview.pickup_attempt_count==0 && t.core.PickupVerificationReady(),"copy PREVIEW mutated the real controller");
  speculative.NotifyPickupIssued(100);
  Check(speculative.PickupAttemptCount()==1 && t.core.PickupAttemptCount()==0,"speculative emission mutated real attempts");
  t.Tick(Ball(),true);
  Check(t.out.action.action==MissionAction::kPickBall && t.out.pickup_attempt_count==0,"unissued PICK consumed an attempt");
  t.Tick(Ball(),true,{true,false,false});
  Check(t.out.mode==BallMode::kPickupBall && t.core.PickupAttemptCount()==0,"unissued PICK accepted a DONE");
  t.core.CancelUnissuedPickup();
  Check(t.core.PickupVerificationReady() && t.core.PickupAttemptCount()==0,"blocked PICK consumed its verified candidate");
  t.Tick(Ball(),true);
  t.core.NotifyPickupIssued(101);t.core.NotifyPickupIssued(101);
  Check(t.core.PickupAttemptCount()==1,"actual PICK was not counted exactly once");
  t.Tick(Ball(),true,{false,true,false});
  Check(t.out.pickup_attempt_count==1,"active PICK was counted twice");
  t.Tick(Ball(),true,{true,false,false});
  Check(t.out.mode==BallMode::kVerifyPickupObservation,"actual PICK DONE lost post-pickup verification");
}
void NewCoordinatesLossAndPause() {
  Trial t;t.Candidate();
  for(int i=0;i<10;++i) t.Tick(Ball(180,340));
  Check(!t.out.pickup_verifying && t.out.action.action!=MissionAction::kPickBall,
        "old inside-polygon coordinates survived the fresh window");
  Check(t.out.pickup_attempt_count==0,"outside-polygon verification counted a PICK");
  Trial lost;lost.Candidate();
  for(int i=0;i<9;++i) lost.Tick(std::nullopt);
  Check(lost.out.pickup_verifying,"loss bypassed the requested stationary second");
  lost.Tick(std::nullopt);
  Check(lost.out.mode==BallMode::kBallRecoveryDown,"lost ball did not enter existing recovery");
  Trial paused;paused.Candidate();paused.core.ShiftTimers(10);paused.now+=10;
  for(int i=0;i<9;++i) paused.Tick();
  Check(paused.out.action.action==MissionAction::kNone,"paused time satisfied verification");
  paused.Tick();Check(paused.out.action.action==MissionAction::kPickBall,"shifted verification deadline lost");
  paused.core.Reset();Check(!paused.core.PickupVerificationActive(),"RESET retained verification");
}
}
int main() { FreshSecondAndManualCommit();NewCoordinatesLossAndPause(); }
