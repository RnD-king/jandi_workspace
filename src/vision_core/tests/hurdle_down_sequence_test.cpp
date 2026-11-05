#include "vision_core/hurdle_controller.hpp"
#include "vision_core/config_loader.hpp"
#include <stdexcept>
#include <optional>
#include <string>

using namespace vision_core;

namespace {
void Require(bool condition,const char* why) {
  if (!condition) throw std::runtime_error(why);
}
std::optional<ObjectTarget> Seen(double v) {
  ObjectTarget target;
  target.center_px={380,v};
  target.box_px={360,v-10,40,20};
  target.width_px=40;target.height_px=20;
  target.area_px=800;target.confidence=.95;
  return target;
}
struct Trial {
  HurdleController controller;
  CameraFeedback down{CameraMode::kDown,true};
  double now{0.0};
  explicit Trial(HurdleConfig config):controller(config,3.0) {
    controller.StartTestNear(true);
  }
  HurdleResult Tick(std::optional<ObjectTarget> target={},
                    ActionExecutionFeedback fb={},double dt=.1,bool allow=true) {
    now+=dt;
    return controller.Compute(target,640,480,now,down,fb,{},allow);
  }
  void Start() {
    Require(Tick().mode==HurdleMode::kDownSearch,"DOWN did not enter search");
    Require(Tick().action.action==MissionAction::kLineStraightUnit,
            "initial DOWN two-step is not motion 33");
  }
};
ActionExecutionFeedback Running() {
  ActionExecutionFeedback f;f.action_active=true;return f;
}
ActionExecutionFeedback Done() {
  ActionExecutionFeedback f;f.action_done=true;return f;
}
void FirstTwoMovingVotes(double v,MissionAction expected) {
  auto config=LoadDefaultAlgorithmConfig().hurdle;
  config.stable_window=10;config.stable_min_hits=7;
  config.down_search_min_hits=3;
  config.down_one_step_v_px=120;
  config.fine_mode_v_px=240;
  Trial t(config);
  t.Start();
  for(int i=0;i<10;++i) {
    const auto r=t.Tick(i<3 ? Seen(v) : std::nullopt,Running());
    Require(r.action.action==MissionAction::kNone,
            "initial 33 acted before DONE");
  }
  t.Tick({},Done());
  const auto decision=t.Tick();
  Require(decision.action.action==expected,
          "first 33 dropped its moving 10/3 observation at DONE");
}
void DistanceStepsOnlyReobserveAfterDone() {
  auto config=LoadDefaultAlgorithmConfig().hurdle;
  config.stable_window=10;config.stable_min_hits=7;
  config.down_search_min_hits=3;
  config.down_one_step_v_px=120;
  config.fine_mode_v_px=240;
  config.down_stationary_observation_sec=1.0;
  Trial t(config);t.Start();
  for(int i=0;i<10;++i)t.Tick(Seen(100),Running());
  t.Tick({},Done());
  auto r=t.Tick();
  Require(r.action.action==MissionAction::kLineStraightUnit,
          "v<120 should issue distance-based 33");
  // These moving high-v samples MUST NOT end the distance-based 33 early.
  for(int i=0;i<11;++i) {
    r=t.Tick(Seen(300),Running());
    Require(r.mode==HurdleMode::kDownSearch &&
            r.action.action==MissionAction::kNone,
            "distance-based 33 chose FINE during motion");
  }
  r=t.Tick(Seen(300),Done());
  Require(r.action.action==MissionAction::kNone,
          "distance-based 33 chose its successor at DONE");
  for(int i=0;i<9;++i) {
    r=t.Tick(Seen(120));
    Require(r.action.action==MissionAction::kNone,
            "stationary 1s window was shortened");
  }
  r=t.Tick(Seen(120),{},.12);
  Require(r.action.action==MissionAction::kStepForwardOne,
          "fresh stationary v=120 should use 10, not old moving v=300");
  // A distance-based 10 ALSO waits for a fresh post-DONE stationary mean.
  for(int i=0;i<10;++i)t.Tick(Seen(100),Running());
  t.Tick(Seen(100),Done());
  for(int i=0;i<9;++i) {
    r=t.Tick(Seen(240));
    Require(r.action.action==MissionAction::kNone,
            "distance-based 10 used moving pixels or skipped its 1s window");
  }
  r=t.Tick(Seen(240),{},.12);
  Require(r.mode==HurdleMode::kWaitFinePose &&
          r.action.action==MissionAction::kDefaultPoseMode,
          "stationary v>=240 did not enter FINE without a final 10");
}
void MissingSearchContinuouslyObservesAndHoldsHalfSecond() {
  auto config=LoadDefaultAlgorithmConfig().hurdle;
  config.stable_window=10;config.stable_min_hits=7;
  config.down_search_min_hits=3;
  config.down_search_inter_action_hold_sec=.5;
  config.down_one_step_v_px=120;
  config.fine_mode_v_px=240;
  Trial t(config);t.Start();
  for(int i=0;i<10;++i)t.Tick({},Running());
  t.Tick({},Done());
  auto r=t.Tick();
  Require(r.action.action==MissionAction::kStepForwardOne,
          "missing first 33 should search with motion 10");
  for(int i=0;i<10;++i)
    r=t.Tick(i>=7 ? Seen(239) : std::nullopt,Running());
  Require(r.action.action==MissionAction::kNone,
          "missing search chose a new step while 10 still active");
  t.Tick(Seen(239),Done());
  for(int i=0;i<4;++i) {
    r=t.Tick(Seen(239));
    Require(r.action.action==MissionAction::kNone,
            "missing search skipped the 0.5s continuous observation hold");
  }
  r=t.Tick(Seen(239));
  Require(r.action.action==MissionAction::kStepForwardOne,
          "moving search missed a qualified 120<=v<240 target after hold");
}
} // namespace

int main() {
  FirstTwoMovingVotes(100,MissionAction::kLineStraightUnit);
  FirstTwoMovingVotes(120,MissionAction::kStepForwardOne);
  FirstTwoMovingVotes(240,MissionAction::kDefaultPoseMode);
  DistanceStepsOnlyReobserveAfterDone();
  MissingSearchContinuouslyObservesAndHoldsHalfSecond();
}
