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
}
int main() { IndependentWaits();RetuneAndDetection();ExhaustedAndZero();ShiftAndReset();FoundSurvivesIdentityChangeUntilDone(); }
