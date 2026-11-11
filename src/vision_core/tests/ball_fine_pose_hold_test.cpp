#include "vision_core/config_loader.hpp"
#include <stdexcept>

using namespace vision_core;
namespace {
void Check(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
}
ObjectTarget Ball() {
  ObjectTarget t;
  t.center_px={320,200}; t.box_px={300,180,40,40};
  t.width_px=t.height_px=40; t.area_px=1600; t.confidence=1;
  return t;
}
BallConfig Config(double hold) {
  auto c=LoadDefaultAlgorithmConfig().ball;
  c.fine_pose_pre_hold_sec=hold;
  c.stable_window=c.stable_min_hits=c.entry_min_hits=c.down_search_min_hits=1;
  c.pickup_verify_min_hits=1;
  return c;
}
struct Trial {
  BallController core;
  BallResult result;
  explicit Trial(double hold):core(Config(hold),3) {
    core.StartTestNear();
    Tick(0);
    Check(result.mode==BallMode::kDownSearch,"camera did not enter DOWN search");
    Tick(10);
    Check(result.mode==BallMode::kWaitFinePose,"ball discovery did not latch FINE preparation");
  }
  void Tick(double now, ActionExecutionFeedback feedback={}, bool allow=true) {
    result=core.Compute(Ball(),640,480,now,{CameraMode::kBallView,true},feedback,allow);
  }
  void Holding() const {
    Check(result.mode==BallMode::kWaitFinePose && result.action.action==MissionAction::kNone,
          "FINE pose was issued before the stationary deadline");
  }
};
void DelayAndActualDone() {
  Trial t(.5); t.Holding();
  ActionExecutionFeedback previous_done; previous_done.action_done=true;
  t.Tick(10.49,previous_done); t.Holding();
  // PREVIEW may recommend the pose but cannot mark it as executed.
  t.Tick(10.5,{},false);
  Check(t.result.action.action==MissionAction::kDefaultPoseMode,"pose not available at 0.5 seconds");
  t.Tick(10.51,previous_done,false);
  Check(t.result.mode==BallMode::kWaitFinePose,"unissued pose consumed an earlier walking DONE");
  t.Tick(10.52);
  Check(t.result.action.action==MissionAction::kDefaultPoseMode,"COMMIT lost the prepared pose");
  ActionExecutionFeedback moving; moving.action_active=true;
  t.Tick(11,moving);
  Check(t.result.mode==BallMode::kWaitFinePose,"time completed a moving FINE pose");
  t.Tick(11.1,previous_done);
  Check(t.result.mode==BallMode::kFineAdjustForPickup,"actual FINE pose DONE was ignored");
}
void PausedClockAndReset() {
  Trial t(.5); t.core.ShiftTimers(4);
  t.Tick(14.49); t.Holding();
  t.Tick(14.5);
  Check(t.result.action.action==MissionAction::kDefaultPoseMode,"shifted deadline was lost");
  t.core.Reset(); t.core.StartTestNear();
  t.Tick(20); t.Tick(21); t.Holding();
  t.Tick(21.49); t.Holding();
  t.Tick(21.5);
  Check(t.result.action.action==MissionAction::kDefaultPoseMode,"reset did not start a new hold");
}
void ZeroDelay() {
  Trial t(0);
  Check(t.result.action.action==MissionAction::kDefaultPoseMode,"zero-delay configuration changed behavior");
}
}
int main() { DelayAndActualDone(); PausedClockAndReset(); ZeroDelay(); }
