#include "vision_core/hurdle_controller.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
using namespace vision_core;
static ObjectTarget H(double v) {
  ObjectTarget t; t.center_px = {50.0, v * 100.0}; t.box_px = {40, v*100-5, 20, 10};
  t.height_px = 10; t.confidence = 1; return t;
}
int main() {
  HurdleConfig c{}; c.stable_window = c.stable_min_hits = 1;
  c.lost_frames = 2; c.smooth_alpha = 1; c.acquire_min_v_norm = 0;
  c.tilt_trigger_v_norm = 0.75; c.tilt_trigger_window = c.tilt_trigger_min_hits = 1;
  HurdleController controller(c, 3.0);
  auto r = controller.Compute(H(0.5), 100, 100, 0, {}, {});
  assert(r.mode == HurdleMode::kApproach);
  assert(r.action.action == MissionAction::kStepForwardFive);
  ActionExecutionFeedback active; active.action_active = true;
  r = controller.Compute(H(0.8), 100, 100, .1, {}, active);
  assert(r.camera_request == CameraRequest::kNone);
  ActionExecutionFeedback done; done.action_done = true;
  r = controller.Compute(H(0.8), 100, 100, .2, {}, done);
  assert(r.camera_request == CameraRequest::kDown);
  r = controller.Compute(H(0.8), 100, 100, .3,
                         {CameraMode::kDown, true}, {});
  assert(r.action.action == MissionAction::kStepForwardOne);
  assert(r.action.action != MissionAction::kContactWalk);
  r = controller.Compute(H(0.8), 100, 100, .4,
                         {CameraMode::kDown, true}, done);
  assert(r.action.action == MissionAction::kHurdle);
  // Camera-transition observations cannot contaminate the DOWN-view evidence.
  HurdleController fresh_down(c,3.0);
  fresh_down.Compute(H(.5),100,100,0,{},{});
  r=fresh_down.Compute(H(.8),100,100,.1,{},done);
  assert(r.mode==HurdleMode::kWaitCameraDown);
  r=fresh_down.Compute(H(.8),100,100,.15,
                       {CameraMode::kTransition,false},{});
  assert(r.mode==HurdleMode::kWaitCameraDown);
  r=fresh_down.Compute(H(.8),100,100,.2,
                       {CameraMode::kDown,true},{});
  assert(r.mode==HurdleMode::kContactWalk && !r.tracked.visible);
  return 0;
}
