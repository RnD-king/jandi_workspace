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
    assert(b.action.action==MissionAction::kNone && h.action.action==MissionAction::kNone);
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
  assert(s.b.mode==BallMode::kCoarseAdjust && s.h.mode==HurdleMode::kCoarseAdjust);
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
  assert(s.b.mode==BallMode::kCoarseAdjust && s.h.mode==HurdleMode::kCoarseAdjust);
  s.NoStep();
  // Filter invalidation and a new search still discard old-view decisions.
  s.ball.InvalidateDetectionEvidence();s.hurdle.InvalidateDetectionEvidence();
  assert(!s.ball.DownSearchFound() && !s.hurdle.DownSearchFound());
  s.ball.StartTestNear();s.hurdle.StartTestNear();
  assert(!s.ball.DownSearchFound() && !s.hurdle.DownSearchFound());
}
void FailedSearchConfirmationResumes(bool partial_hits) {
  auto c=Config(.5,.5);
  c.ball.stable_window=c.hurdle.stable_window=10;
  c.ball.stable_min_hits=c.hurdle.stable_min_hits=7;
  c.ball.down_search_min_hits=3;c.hurdle.down_search_min_hits=5;
  c.ball.down_search_max_steps=c.hurdle.down_search_max_steps=3;
  c.ball.lost_frames=c.hurdle.lost_frames=1;
  c.ball.down_one_step_v_px=c.hurdle.down_one_step_v_px=100;
  c.ball.fine_mode_v_px=c.hurdle.fine_mode_v_px=180;
  c.ball.pickup_v1_px=200;
  c.ball.pickup_v2_px=250;
  SearchPair s(c);s.Tick(.1);
  ObjectTarget target;target.center_px={320,50};target.box_px={300,30,40,40};
  target.width_px=target.height_px=40;target.area_px=1600;target.confidence=1;
  for(int i=0;i<7;++i)s.Tick(.2+.1*i,true,false,target);
  s.Tick(1,false,true);s.NoStep();
  assert(s.b.mode==BallMode::kCoarseAdjust && s.h.mode==HurdleMode::kCoarseAdjust);
  std::deque<bool> failed(10,false);
  if(partial_hits)for(int i=0;i<5;++i)failed[i]=true;
  // Core passes no stationary mean both for total loss and insufficient 7/10.
  s.Verify(2.1,{},failed);s.NoStep();
  assert(s.b.mode==BallMode::kDownSearch && s.h.mode==HurdleMode::kDownSearch);
  assert(!s.ball.DownSearchFound() && !s.hurdle.DownSearchFound());
  assert(s.ball.DownSearchStepsRemaining()==2 && s.hurdle.DownSearchStepsRemaining()==2);
  s.Tick(2.2);
  assert(s.b.action.action==MissionAction::kStepForwardOne && s.h.action.action==s.b.action.action);
  s.Tick(2.3,true);s.NoStep();
  assert(s.ball.DownSearchStepsRemaining()==2 && s.hurdle.DownSearchStepsRemaining()==2);
  for(int i=0;i<7;++i)s.Tick(2.4+.1*i,true,false,target);
  s.Tick(3.2,false,true);s.NoStep();
  assert(s.b.mode==BallMode::kCoarseAdjust && s.h.mode==HurdleMode::kCoarseAdjust);
  s.Verify(4.3,target,std::deque<bool>(10,true));
  assert(s.b.action.action==MissionAction::kStepForwardTwo && s.h.action.action==s.b.action.action);
  assert(!s.ball.DownSearchFound() && !s.hurdle.DownSearchFound());
  assert(s.ball.DownSearchStepsRemaining()==1 && s.hurdle.DownSearchStepsRemaining()==1);
  // A later ordinary approach loss still uses the existing recovery FSM.
  s.Verify(5.4,{},std::deque<bool>(10,false));s.NoStep();
  assert(s.b.mode==BallMode::kBallRecoveryDown && s.h.mode==HurdleMode::kRecoveryDown);
}
void FailedSearchConfirmationRetainsLimitAndHold() {
  auto c=Config(.5,.5);
  c.ball.down_search_max_steps=c.hurdle.down_search_max_steps=1;
  SearchPair s(c);s.Tick(.1);
  ObjectTarget target;target.center_px={320,50};target.box_px={300,30,40,40};
  target.width_px=target.height_px=40;target.area_px=1600;target.confidence=1;
  s.Tick(.2,true,false,target);s.Tick(1,false,true);s.NoStep();
  s.Verify(2,{},std::deque<bool>(10,false));s.NoStep();s.Tick(2.1);s.NoStep();
  assert(s.b.mode==BallMode::kFailed && s.h.mode==HurdleMode::kFailed);
  assert(s.ball.DownSearchStepsRemaining()==0 && s.hurdle.DownSearchStepsRemaining()==0);
  // Even an early manual confirmation cannot discard the last DONE's hold.
  c.ball.down_search_max_steps=c.hurdle.down_search_max_steps=2;
  SearchPair held(c);held.Tick(.1);held.Tick(.2,true,false,target);held.Tick(1,false,true);
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
