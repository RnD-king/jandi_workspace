#include "vision_core/hurdle_controller.hpp"
#include "vision_core/config_loader.hpp"
#include "vision_core/mission_controller.hpp"
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
void InitialDownMotionUsesLastReadyAction() {
  auto config=LoadDefaultAlgorithmConfig().hurdle;
  for (const auto first : {MissionAction::kLineLeftUnit,
                           MissionAction::kLineStraightUnit,
                           MissionAction::kLineRightUnitTurnFirst}) {
    HurdleController controller(config,3.0);
    controller.EnterContinuousNear(0.0,first);
    const CameraFeedback down{CameraMode::kDown,true};
    auto result=controller.Compute({},640,480,0.1,down,{}, {},true);
    Require(result.mode==HurdleMode::kDownSearch,
            "HURDLE did not enter DOWN");
    result=controller.Compute({},640,480,0.2,down,{}, {},true);
    Require(result.action.action==first,
            "HURDLE first DOWN unit ignored final extra LINE READY");
  }
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
void DownIgnoresOutOfRangeDepthButForwardDoesNot() {
  auto c=LoadDefaultAlgorithmConfig();
  c.enable_hurdle=true; c.enable_ball=c.enable_goal=false;
  c.object_targets.hurdle_depth_enabled=true;
  c.object_targets.hurdle_min_depth_m=.25;
  c.object_targets.hurdle_max_depth_m=2.0;
  MissionController controller(c);
  PerceptionFrameInput f; f.image_width=640; f.image_height=480;
  f.manual_observation_control=true; f.advance_line_fsm=false;
  PerceptionDetection d;
  d.detection.class_id=c.object_targets.hurdle_class_id;
  d.detection.confidence=.95;
  d.detection.box={130,180,240,30};
  d.center_depth_m=.10; // Rejected by FORWARD [0.25,2m].
  f.detections={d};
  f.camera_feedback={CameraMode::kForward,true};f.now_sec=.1;
  auto out=controller.StepPerception(f);
  Require(!out.perception.object_filter_diagnostics.back().depth_passed &&
          out.perception.accepted_object_detections.empty(),
          "FORWARD accidentally stopped enforcing HURDLE Depth range");
  f.camera_feedback={CameraMode::kDown,true};f.now_sec=.2;
  out=controller.StepPerception(f);
  Require(out.perception.object_filter_diagnostics.back().depth_passed &&
          out.perception.accepted_object_detections.size()==1,
          "DOWN should accept HURDLE pixels regardless of Depth range");
}
} // namespace

int main() {
  InitialDownMotionUsesLastReadyAction();
  FirstTwoMovingVotes(100,MissionAction::kStepForwardOne); // Only 3 far votes: insufficient for 2 steps.
  FirstTwoMovingVotes(120,MissionAction::kStepForwardOne);
  FirstTwoMovingVotes(240,MissionAction::kDefaultPoseMode);
  DistanceStepsOnlyReobserveAfterDone();
  MissingSearchContinuouslyObservesAndHoldsHalfSecond();
  DownIgnoresOutOfRangeDepthButForwardDoesNot();
}
