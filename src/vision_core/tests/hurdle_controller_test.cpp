#include "vision_core/hurdle_controller.hpp"
#include "vision_core/config_loader.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
using namespace vision_core;
static ObjectTarget H(double v) {
  ObjectTarget t; t.center_px = {320.0, v * 480.0}; t.box_px = {310, v*480-5, 20, 10};
  t.height_px = 10; t.confidence = 1; return t;
}
int main() {
  HurdleConfig c=LoadDefaultAlgorithmConfig().hurdle; c.recovery_settle_duration_sec=0; c.contact_steps=1; c.stable_window = c.stable_min_hits = 1;
  c.lost_frames = 2; c.smooth_alpha = 1; c.upper_acquire_v_px=480;
  c.approach_two_step_v_px = 360.0;
  HurdleController controller(c, 3.0);
  auto r = controller.Compute(H(0.5), 640,480, 0, {}, {});
  assert(r.mode == HurdleMode::kApproach);
  assert(r.action.action == MissionAction::kStepForwardFour);
  ActionExecutionFeedback active; active.action_active = true;
  r = controller.Compute(H(0.8), 640,480, .1, {}, active);
  assert(r.camera_request == CameraRequest::kNone);
  ActionExecutionFeedback done; done.action_done = true;
  r = controller.Compute(H(0.8), 640,480, .2, {}, done);
  assert(r.camera_request==CameraRequest::kNone);
  ActionExecutionFeedback observed; observed.observation_complete=true;
  r=controller.Compute(H(.8),640,480,.21,{},observed);
  assert(r.action.action==MissionAction::kStepForwardTwo);
  r=controller.Compute({},640,480,.22,{},done);
  assert(r.camera_request == CameraRequest::kDown);
  r = controller.Compute(H(0.8), 640,480, .3,
                         {CameraMode::kDown, true}, {});
  assert(r.mode==HurdleMode::kDownSearch && !r.tracked.visible);
  r=controller.Compute(H(.9),640,480,.31,{CameraMode::kDown,true},{});
  assert(r.mode==HurdleMode::kCoarseAdjust);
  r=controller.Compute(H(.9),640,480,.32,{CameraMode::kDown,true},{});
  assert(r.action.action==MissionAction::kDefaultPoseMode);
  r=controller.Compute({},640,480,.33,{CameraMode::kDown,true},done);
  assert(r.mode==HurdleMode::kFineAdjust);
  r=controller.Compute(H(.9),640,480,.34,{CameraMode::kDown,true},{});
  assert(r.mode==HurdleMode::kContactWalk);
  assert(r.action.action == MissionAction::kStepForwardHalf);
  assert(r.action.action != MissionAction::kContactWalk);
  r = controller.Compute(H(0.8), 640,480, .4,
                         {CameraMode::kDown, true}, done);
  assert(r.action.action == MissionAction::kHurdle);
  // Camera-transition observations cannot contaminate the DOWN-view evidence.
  HurdleController fresh_down(c,3.0);
  fresh_down.Compute(H(.5),640,480,0,{},{});
  r=fresh_down.Compute(H(.8),640,480,.1,{},done);
  r=fresh_down.Compute(H(.8),640,480,.11,{},observed);
  assert(r.mode==HurdleMode::kApproachTwoBeforeDown);
  r=fresh_down.Compute({},640,480,.12,{},done);
  assert(r.mode==HurdleMode::kWaitCameraDown);
  r=fresh_down.Compute(H(.8),640,480,.15,
                       {CameraMode::kTransition,false},{});
  assert(r.mode==HurdleMode::kWaitCameraDown);
  r=fresh_down.Compute(H(.8),640,480,.2,
                       {CameraMode::kDown,true},{});
  assert(r.mode==HurdleMode::kDownSearch && !r.tracked.visible);
  return 0;
}
