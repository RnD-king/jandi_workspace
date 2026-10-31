#include "vision_core/config_loader.hpp"
#include "vision_core/hurdle_controller.hpp"
#include "vision_core/line_feature_extractor.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>

using namespace vision_core;
namespace {
constexpr double kPi = 3.14159265358979323846;
ObjectTarget Hurdle(double v) {
  ObjectTarget t;
  t.center_px = {320, v};
  t.box_px = {220, v - 8, 200, 16};
  t.height_px = 16;
  t.confidence = 1;
  return t;
}
LineGuide Guide(double offset_px, double heading_deg) {
  LineGuide guide;
  guide.valid = true;
  guide.offset = offset_px / 320.0;
  guide.heading_rad = heading_deg * kPi / 180.0;
  return guide;
}
struct Trial {
  HurdleController controller;
  double now{0};
  const CameraFeedback down{CameraMode::kDown, true};
  explicit Trial(bool close = false) : controller([] {
    auto c = LoadDefaultAlgorithmConfig().hurdle;
    c.stable_window = c.stable_min_hits = c.down_search_min_hits = 1;
    return c;
  }(), 3.0) {
    controller.StartTestNear();
    const auto target = Hurdle(close ? 260 : 220);
    auto out = controller.Compute({}, 640, 480, now, down, {}, {});
    assert(out.mode == HurdleMode::kDownSearch);
    out = controller.Compute(target, 640, 480, now += .1, down, {}, {});
    assert(out.mode == HurdleMode::kCoarseAdjust);
    out = controller.Compute(target, 640, 480, now += .1, down, {}, {});
    assert(out.mode == HurdleMode::kWaitFinePose &&
           out.action.action == MissionAction::kDefaultPoseMode);
  }
  void Remember(const LineGuide &guide) {
    for (int i = 0; i < 10; ++i)
      controller.ObserveFineLine(guide, 640, now += .11);
  }
  void PoseDone() {
    ActionExecutionFeedback done;
    done.action_done = true;
    const auto out = controller.Compute({}, 640, 480, now += .1, down, done, {});
    assert(out.mode == HurdleMode::kFineLineAlign);
  }
  HurdleResult Observe(const LineGuide &guide = {}) {
    HurdleResult out;
    for (int i = 0; i < 10; ++i) {
      now += .11;
      controller.ObserveFineLine(guide, 640, now);
      out = controller.Compute({}, 640, 480, now, down, {}, {});
      if (i < 9) assert(out.action.action == MissionAction::kNone);
    }
    return out;
  }
  void Done(MissionAction action) {
    ActionExecutionFeedback done;
    done.action_done = true;
    now += .1;
    controller.OnActualMotionDone(action, now);
    const auto out = controller.Compute({}, 640, 480, now, down, done, {});
    assert(out.mode == HurdleMode::kFineLineAlign &&
           out.action.action == MissionAction::kNone);
  }
};
void ThreePointGate() {
  auto cfg = LoadDefaultAlgorithmConfig().line_features;
  const std::vector<Point2> three{{380, 400}, {380, 300}, {380, 200}};
  assert(!ComputeLineFeatures(three, 640, 480, cfg).guide.valid);
  cfg.ransac_enabled = false;
  cfg.image_center_u = 380;
  assert(ComputeLineFeatures(three, 640, 480, cfg, 3).guide.valid);
  assert(!ComputeLineFeatures({{380, 400}, {380, 300}},
                              640, 480, cfg, 3).guide.valid);
}
void HeadingThenOffset() {
  Trial t;
  t.PoseDone();
  auto out = t.Observe(Guide(30, 10));
  assert(out.action.action == MissionAction::kTurnRight &&
         out.action.target_yaw_deg == 15);
  t.Done(MissionAction::kTurnRight);
  out = t.Observe(Guide(30, 0));
  assert(out.fine_line_turns == 1 &&
         out.action.action == MissionAction::kRightSideStep);
  t.Done(MissionAction::kRightSideStep);
  out = t.Observe(Guide(0, 0));
  assert(out.mode == HurdleMode::kFineAdjust && out.fine_line_side_steps == 1);
}
void StaleMemoryAndBoundedSearch() {
  Trial t;
  t.Remember(Guide(-40, -10));
  t.PoseDone();
  for (int i = 0; i < 2; ++i) {
    const auto out = t.Observe();
    assert(out.action.action == MissionAction::kTurnLeft &&
           out.action.target_yaw_deg == 15);
    t.Done(MissionAction::kTurnLeft);
  }
  for (int i = 0; i < 3; ++i) {
    const auto out = t.Observe();
    assert(out.action.action == MissionAction::kStepForwardHalf &&
           out.fine_line_turns == 2);
    t.Done(MissionAction::kStepForwardHalf);
  }
  const auto out = t.Observe();
  assert(out.mode == HurdleMode::kFineAdjust && out.fine_line_search_steps == 3);
}
void NoSearchWhenClose() {
  Trial t(true);
  t.PoseDone();
  const auto out = t.Observe();
  assert(out.mode == HurdleMode::kFineAdjust &&
         out.action.action == MissionAction::kNone);
}
void FiveOfTenIncludesMissingLastFrame() {
  Trial t;
  t.PoseDone();
  HurdleResult out;
  for (int i = 0; i < 10; ++i) {
    t.now += .11;
    t.controller.ObserveFineLine(i < 5 ? Guide(0, 0) : LineGuide{}, 640, t.now);
    out = t.controller.Compute({}, 640, 480, t.now, t.down, {}, {});
  }
  assert(out.mode == HurdleMode::kFineAdjust);
}
void IndependentCorrectionLimits() {
  Trial turns;
  turns.PoseDone();
  for (int i = 0; i < 4; ++i) {
    const auto out = turns.Observe(Guide(30, 10));
    assert(out.action.action == MissionAction::kTurnRight);
    turns.Done(MissionAction::kTurnRight);
  }
  for (int i = 0; i < 4; ++i) {
    const auto out = turns.Observe(Guide(30, 10));
    assert(out.mode == HurdleMode::kFineLineAlign &&
           out.fine_line_turns == 4 && out.fine_line_side_steps == i &&
           out.action.action == MissionAction::kRightSideStep);
    turns.Done(MissionAction::kRightSideStep);
  }
  auto out = turns.Observe(Guide(30, 10));
  assert(out.mode == HurdleMode::kFineAdjust && out.fine_line_turns == 4 &&
         out.fine_line_side_steps == 4);

  Trial turns_only;
  turns_only.PoseDone();
  for (int i = 0; i < 4; ++i) {
    out = turns_only.Observe(Guide(0, 10));
    assert(out.action.action == MissionAction::kTurnRight);
    turns_only.Done(MissionAction::kTurnRight);
  }
  out = turns_only.Observe(Guide(0, 10));
  assert(out.mode == HurdleMode::kFineAdjust && out.fine_line_turns == 4 &&
         out.fine_line_side_steps == 0);

  Trial sides;
  sides.PoseDone();
  for (int i = 0; i < 4; ++i) {
    out = sides.Observe(Guide(30, 0));
    assert(out.action.action == MissionAction::kRightSideStep);
    sides.Done(MissionAction::kRightSideStep);
  }
  out = sides.Observe(Guide(30, 10));
  assert(out.mode == HurdleMode::kFineLineAlign &&
         out.fine_line_side_steps == 4 && out.fine_line_turns == 0 &&
         out.action.action == MissionAction::kTurnRight);
  sides.Done(MissionAction::kTurnRight);
  out = sides.Observe(Guide(30, 0));
  assert(out.mode == HurdleMode::kFineAdjust && out.fine_line_side_steps == 4 &&
         out.fine_line_turns == 1);
}
void StaleAlignedIsNotSuccess() {
  Trial t;
  t.Remember(Guide(0, 0));
  t.PoseDone();
  const auto out = t.Observe();
  assert(out.mode == HurdleMode::kFineLineAlign &&
         out.action.action == MissionAction::kStepForwardHalf);
}
void CameraTransitionRequiresFreshObservation() {
  Trial t;
  t.PoseDone();
  t.now += .2;
  auto out = t.controller.Compute({}, 640, 480, t.now,
                                  {CameraMode::kTransition, false}, {}, {});
  assert(out.mode == HurdleMode::kFineLineAlign &&
         out.action.action == MissionAction::kNone);
  out = t.controller.Compute({}, 640, 480, t.now += .1, t.down, {}, {});
  assert(out.mode == HurdleMode::kFineLineAlign);
  out = t.Observe(Guide(0, 0));
  assert(out.mode == HurdleMode::kFineAdjust);
}
} // namespace
int main() {
  ThreePointGate();
  HeadingThenOffset();
  StaleMemoryAndBoundedSearch();
  NoSearchWhenClose();
  FiveOfTenIncludesMissingLastFrame();
  IndependentCorrectionLimits();
  StaleAlignedIsNotSuccess();
  CameraTransitionRequiresFreshObservation();
}
