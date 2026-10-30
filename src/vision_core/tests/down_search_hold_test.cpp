#include "vision_core/config_loader.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>

using namespace vision_core;
namespace {
MissionControllerConfig Config(double ball_hold, double hurdle_hold) {
  auto c=LoadDefaultAlgorithmConfig();
  c.ball.down_search_inter_action_hold_sec=ball_hold;
  c.hurdle.down_search_inter_action_hold_sec=hurdle_hold;
  c.ball.stable_window=c.ball.stable_min_hits=c.ball.entry_min_hits=c.ball.down_search_min_hits=1;
  c.ball.pickup_verify_min_hits=1; // This fixture uses a one-frame window.
  c.hurdle.stable_window=c.hurdle.stable_min_hits=c.hurdle.down_search_min_hits=1;
  return c;
}
struct SearchPair {
  BallController ball;
  HurdleController hurdle;
  BallResult b;
  HurdleResult h;
  explicit SearchPair(const MissionControllerConfig& c):ball(c.ball,3),hurdle(c.hurdle,3) {
    ball.StartTestNear();hurdle.StartTestNear();Tick(0);
    assert(b.mode==BallMode::kDownSearch && h.mode==HurdleMode::kDownSearch);
  }
  void Tick(double now, bool active=false, bool done=false,
            const std::optional<ObjectTarget>& target=std::nullopt) {
    ActionExecutionFeedback feedback;feedback.action_active=active;feedback.action_done=done;
    const CameraFeedback down{CameraMode::kDown,true};
    b=ball.Compute(target,640,480,now,down,feedback);
    h=hurdle.Compute(target,640,480,now,down,feedback,{});
  }
  void NoStep() const {
    assert((b.action.action==MissionAction::kNone ||
        (b.mode==BallMode::kWaitFinePose && b.action.action==MissionAction::kDefaultPoseMode)) &&
        h.action.action==MissionAction::kNone);
  }
  void Verify(double now,const std::optional<ObjectTarget>& mean,const std::deque<bool>& hits) {
    ball.UseVerifiedObservation(mean,hits,640,480);
    hurdle.UseVerifiedObservation(mean,hits,640,480);
    ActionExecutionFeedback feedback;feedback.observation_complete=true;
    const CameraFeedback down{CameraMode::kDown,true};
    b=ball.Compute({},640,480,now,down,feedback);
    h=hurdle.Compute({},640,480,now,down,feedback,{});
  }
};
void IndependentWaits() {
  auto c=Config(1,2);
  SearchPair s(c);s.Tick(.1);
  assert(s.b.action.action==MissionAction::kStepForwardOne && s.h.action.action==s.b.action.action);
  s.Tick(100,true);s.NoStep(); // Elapsed time cannot consume an unfinished action.
  assert(s.ball.DownSearchStepsRemaining()==c.ball.down_search_max_steps);
  s.Tick(101,false,true);s.NoStep();
  assert(s.ball.DownSearchStepsRemaining()==c.ball.down_search_max_steps-1);
  s.Tick(101.5,false,true);s.NoStep(); // Duplicate DONE must not move the deadline.
  s.Tick(101.99);s.NoStep();
  s.Tick(102);
  assert(s.b.action.action==MissionAction::kStepForwardOne && s.h.action.action==MissionAction::kNone);
  // Lower HURDLE's setting during the wait; retain original DONE and count.
  c.hurdle.down_search_inter_action_hold_sec=.5;s.hurdle.UpdateTuning(c.hurdle);
  auto h=s.hurdle.Compute({},640,480,102.01,{CameraMode::kDown,true},{},{});
  assert(h.action.action==MissionAction::kStepForwardOne);
  assert(s.hurdle.DownSearchStepsRemaining()==c.hurdle.down_search_max_steps-1);
}
void RetuneAndDetection() {
  auto c=Config(.5,.5);SearchPair s(c);s.Tick(.1);s.Tick(1,false,true);s.NoStep();
  c.ball.down_search_inter_action_hold_sec=2;c.hurdle.down_search_inter_action_hold_sec=2;
  s.ball.UpdateTuning(c.ball);s.hurdle.UpdateTuning(c.hurdle);
  s.Tick(1.6);s.NoStep(); // Increasing the wait is relative to DONE, not retuning time.
  ObjectTarget target;target.center_px={320,200};target.box_px={310,190,20,20};
  target.width_px=target.height_px=20;target.area_px=400;target.confidence=1;
  s.Tick(1.7,false,false,target);
  assert(s.b.mode==BallMode::kWaitFinePose && s.h.mode==HurdleMode::kCoarseAdjust);
  s.NoStep(); // Detection while stationary needs no additional search step.
  s.ball.StartTestNear();s.hurdle.StartTestNear();s.Tick(1.8);s.Tick(1.9);
  assert(s.b.action.action==MissionAction::kStepForwardOne && s.h.action.action==s.b.action.action);
}
void ExhaustedAndZero() {
  auto c=Config(10,10);c.ball.down_search_max_steps=c.hurdle.down_search_max_steps=1;
  SearchPair s(c);s.Tick(.1);s.Tick(.2,false,true);
  assert(s.b.mode==BallMode::kFailed && s.h.mode==HurdleMode::kFailed);
  c=Config(0,0);SearchPair zero(c);zero.Tick(.1);zero.Tick(.2,false,true);
  assert(zero.b.action.action==MissionAction::kStepForwardOne && zero.h.action.action==zero.b.action.action);
}
void ShiftAndReset() {
  DownSearchWalk s;
  using D=DownSearchWalk::Decision;
  assert(s.Observe(false,0,1,2,false,false,0,1)==D::kStep);
  s.Issued();assert(s.Observe(false,0,1,2,false,true,1,1)==D::kWaiting);
  s.ShiftTimers(3);
  assert(s.Observe(false,0,1,2,false,false,4.5,1)==D::kWaiting);
  assert(s.Observe(false,0,1,2,false,false,5,1)==D::kStep);
  s.Reset();assert(s.Remaining(2)==2);
  assert(s.Observe(false,0,1,2,false,false,5,10)==D::kStep);
}
void FoundSurvivesIdentityChangeUntilDone() {
  auto c=Config(.5,.5);
  c.ball.stable_window=c.hurdle.stable_window=10;
  c.ball.stable_min_hits=c.hurdle.stable_min_hits=7;
  c.ball.down_search_min_hits=3;c.hurdle.down_search_min_hits=5;
  SearchPair s(c);s.Tick(.1);
  ObjectTarget target;target.center_px={320,200};target.box_px={300,180,40,40};
  target.width_px=target.height_px=40;target.area_px=1600;target.confidence=1;
  for(int i=0;i<7;++i)s.Tick(.2+.1*i,true,false,target);
  assert(s.ball.DownSearchFound() && s.hurdle.DownSearchFound());
  s.ball.InvalidateTargetIdentity();s.hurdle.InvalidateTargetIdentity();
  for(int i=0;i<6;++i){s.Tick(1+.1*i,true);s.NoStep();}
  assert(s.ball.DownSearchFound() && s.hurdle.DownSearchFound());
  s.Tick(2,false,true);
  assert(s.b.mode==BallMode::kWaitFinePose && s.h.mode==HurdleMode::kCoarseAdjust);
  s.NoStep();
  // Filter invalidation and a new search still discard old-view decisions.
  s.ball.InvalidateDetectionEvidence();s.hurdle.InvalidateDetectionEvidence();
  assert(!s.ball.DownSearchFound() && !s.hurdle.DownSearchFound());
  s.ball.StartTestNear();s.hurdle.StartTestNear();
  assert(!s.ball.DownSearchFound() && !s.hurdle.DownSearchFound());
}
void FailedSearchConfirmationResumes(bool partial_hits) {
  // HURDLE retains DOWN WALK confirmation/retry; BALL now commits to FINE.
  auto c=Config(.5,.5);auto hc=c.hurdle;
  hc.stable_window=10;hc.stable_min_hits=7;hc.down_search_min_hits=5;
  hc.down_search_max_steps=3;hc.lost_frames=1;
  hc.down_one_step_v_px=100;hc.fine_mode_v_px=180;
  HurdleController h(hc,3);HurdleResult r;
  const CameraFeedback down{CameraMode::kDown,true};
  auto tick=[&](double now,std::optional<ObjectTarget> t={},ActionExecutionFeedback fb={}) {
    r=h.Compute(t,640,480,now,down,fb,{});
  };
  ObjectTarget t;t.center_px={320,50};t.box_px={300,30,40,40};
  t.width_px=t.height_px=40;t.area_px=1600;t.confidence=1;
  h.StartTestNear();tick(0);tick(.1);
  for(int i=0;i<7;++i)tick(.2+.1*i,t,{false,true,false});
  tick(1,{}, {true,false,false});assert(r.mode==HurdleMode::kCoarseAdjust);
  std::deque<bool> failed(10,false);if(partial_hits)for(int i=0;i<5;++i)failed[i]=true;
  h.UseVerifiedObservation({},failed,640,480);tick(2.1,{}, {false,false,true});
  assert(r.mode==HurdleMode::kDownSearch && h.DownSearchStepsRemaining()==2);
  tick(2.2);assert(r.action.action==MissionAction::kStepForwardOne);
  for(int i=0;i<7;++i)tick(2.4+.1*i,t,{false,true,false});
  tick(3.2,{}, {true,false,false});assert(r.mode==HurdleMode::kCoarseAdjust);
  h.UseVerifiedObservation(t,std::deque<bool>(10,true),640,480);tick(4.3,{}, {false,false,true});
  assert(r.action.action==MissionAction::kStepForwardTwo && h.DownSearchStepsRemaining()==1);
  h.UseVerifiedObservation({},std::deque<bool>(10,false),640,480);tick(5.4,{}, {false,false,true});
  assert(r.mode==HurdleMode::kRecoveryStep);

  auto bc=c.ball;bc.stable_window=10;bc.stable_min_hits=7;bc.down_search_min_hits=3;bc.lost_frames=1;
  bc.pickup_u1_px=300;bc.pickup_u2_px=340;bc.pickup_v1_px=100;bc.pickup_v2_px=150;
  BallController ball(bc,3);ball.StartTestNear();ball.Compute({},640,480,0,down,{});
  ball.Compute({},640,480,.1,down,{});
  for(int i=0;i<7;++i)ball.Compute(t,640,480,.2+.1*i,down,{false,true,false});
  auto bres=ball.Compute({},640,480,1,down,{true,false,false});
  assert(bres.mode==BallMode::kWaitFinePose && bres.action.action==MissionAction::kDefaultPoseMode);
  bres=ball.Compute({},640,480,1.1,down,{false,true,false});
  assert(bres.mode==BallMode::kWaitFinePose);
  bres=ball.Compute({},640,480,1.2,down,{true,false,false});
  assert(bres.mode==BallMode::kFineAdjustForPickup);
  ball.UseVerifiedObservation({},failed,640,480);
  bres=ball.Compute({},640,480,2.3,down,{false,false,true});
  assert(bres.mode==BallMode::kBallRecoveryDown);
  bres=ball.Compute({},640,480,2.4,down,{});
  assert(bres.action.action==MissionAction::kStepForwardHalf);
  assert(ball.DownSearchStepsRemaining()==bc.down_search_max_steps-1); // No WALK search restart.
}
void FailedSearchConfirmationRetainsLimitAndHold() {
  auto c=Config(.5,.5);
  c.ball.down_search_max_steps=c.hurdle.down_search_max_steps=1;
  SearchPair s(c);s.Tick(.1);
  s.Tick(.2,true,false);s.Tick(1,false,true);s.NoStep();
  s.Verify(2,{},std::deque<bool>(10,false));s.NoStep();s.Tick(2.1);s.NoStep();
  assert(s.b.mode==BallMode::kFailed && s.h.mode==HurdleMode::kFailed);
  assert(s.ball.DownSearchStepsRemaining()==0 && s.hurdle.DownSearchStepsRemaining()==0);
  // Even an early manual confirmation cannot discard the last DONE's hold.
  c.ball.down_search_max_steps=c.hurdle.down_search_max_steps=2;
  SearchPair held(c);held.Tick(.1);held.Tick(.2,true,false);held.Tick(1,false,true);
  held.Verify(1.1,{},std::deque<bool>(10,false));held.Tick(1.49);held.NoStep();
  held.Tick(1.5);
  assert(held.b.action.action==MissionAction::kStepForwardOne && held.h.action.action==held.b.action.action);
}
}
int main() {
  IndependentWaits();RetuneAndDetection();ExhaustedAndZero();ShiftAndReset();
  FoundSurvivesIdentityChangeUntilDone();
  FailedSearchConfirmationResumes(false);FailedSearchConfirmationResumes(true);
  FailedSearchConfirmationRetainsLimitAndHold();
}
