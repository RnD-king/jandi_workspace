#include "vision_core/hurdle_controller.hpp"
#include "vision_core/config_loader.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <utility>
#include <vector>
using namespace vision_core;
static ObjectTarget H(double v) {
  ObjectTarget t; t.center_px = {320.0, v * 480.0}; t.box_px = {310, v*480-5, 20, 10};
  t.height_px = 10; t.confidence = 1; return t;
}
int main() {
  HurdleConfig c=LoadDefaultAlgorithmConfig().hurdle;  c.contact_steps=1; c.down_search_min_hits=1; c.stable_window = c.stable_min_hits = 1;
  c.lost_frames = 2; c.smooth_alpha = 1; c.upper_acquire_v_px=480;
  c.approach_two_step_v_px = 360.0;
  c.approach_short_v_px = 312.0;
  CruiseDecision straight; straight.applicable=true;
  straight.direction=CruiseDirection::kStraight;
  CruiseDecision left = straight, right = straight, missing = straight;
  left.direction=CruiseDirection::kLeft;
  right.direction=CruiseDirection::kRight;
  missing.direction=CruiseDirection::kNone;
  struct GridCase { double v; CruiseDirection direction; MissionAction action; };
  for (const auto& scenario : std::vector<GridCase>{
      {.5,CruiseDirection::kLeft,MissionAction::kStepForwardLeftTwo},
      {.5,CruiseDirection::kStraight,MissionAction::kStepForwardFour},
      {.5,CruiseDirection::kRight,MissionAction::kStepForwardRightTwo},
      {.7,CruiseDirection::kLeft,MissionAction::kStepForwardLeftOne},
      {.7,CruiseDirection::kStraight,MissionAction::kStepForwardThree},
      {.7,CruiseDirection::kRight,MissionAction::kStepForwardRightOne},
      {.8,CruiseDirection::kLeft,MissionAction::kTurnLeftInPlace},
      {.8,CruiseDirection::kStraight,MissionAction::kStepForwardTwo},
      {.8,CruiseDirection::kRight,MissionAction::kStepForwardRightOneHurdle},
      {312.0/480,CruiseDirection::kRight,MissionAction::kStepForwardRightOne},
      {360.0/480,CruiseDirection::kLeft,MissionAction::kTurnLeftInPlace}}) {
    HurdleController grid(c,3.0);
    auto steering=straight; steering.direction=scenario.direction;
    const auto v=grid.Compute(H(scenario.v),640,480,0,{}, {},steering);
    assert(v.action.action==scenario.action);
    assert(v.action.category==ActionCategory::kLocomotion);
    assert(v.camera_request==CameraRequest::kNone);
  }
  HurdleController right_near(c,3.0);
  assert(right_near.Compute(H(.8),640,480,0,{}, {},right).action.action==
         MissionAction::kStepForwardRightOneHurdle);
  HurdleController invalid_line(c,3.0);
  assert(invalid_line.Compute(H(.5),640,480,0,{}, {},missing).action.action==
         MissionAction::kNone);
  HurdleController controller(c, 3.0);
  auto r = controller.Compute(H(0.5), 640,480, 0, {}, {}, straight);
  assert(r.mode == HurdleMode::kApproach);
  assert(r.action.action == MissionAction::kStepForwardFour);
  ActionExecutionFeedback active; active.action_active = true;
  r = controller.Compute(H(0.8), 640,480, .1, {}, active, straight);
  assert(r.camera_request == CameraRequest::kNone);
  ActionExecutionFeedback done; done.action_done = true;
  r = controller.Compute(H(0.8), 640,480, .2, {}, done, straight);
  assert(r.camera_request==CameraRequest::kNone);
  ActionExecutionFeedback observed; observed.observation_complete=true;
  r=controller.Compute(H(.8),640,480,.21,{},observed, straight);
  assert(r.action.action==MissionAction::kStepForwardTwo);
  r=controller.Compute({},640,480,.22,{},done, straight);
  assert(r.camera_request == CameraRequest::kDown);
  r = controller.Compute(H(0.8), 640,480, .3,
                         {CameraMode::kDown, true}, {}, straight);
  assert(r.mode==HurdleMode::kDownSearch && !r.tracked.visible);
  r=controller.Compute(H(.9),640,480,.31,{CameraMode::kDown,true},{}, straight);
  assert(r.mode==HurdleMode::kCoarseAdjust);
  r=controller.Compute(H(.9),640,480,.32,{CameraMode::kDown,true},{}, straight);
  assert(r.action.action==MissionAction::kDefaultPoseMode);
  r=controller.Compute({},640,480,.33,{CameraMode::kDown,true},done, straight);
  assert(r.mode==HurdleMode::kFineAdjust);
  r=controller.Compute(H(.9),640,480,.34,{CameraMode::kDown,true},{}, straight);
  assert(r.mode==HurdleMode::kContactWalk);
  assert(r.action.action == MissionAction::kStepForwardHalf);
  assert(r.action.action != MissionAction::kContactWalk);
  r = controller.Compute(H(0.8), 640,480, .4,
                         {CameraMode::kDown, true}, done, straight);
  assert(r.action.action == MissionAction::kHurdle);
  // Camera-transition observations cannot contaminate the DOWN-view evidence.
  HurdleController fresh_down(c,3.0);
  fresh_down.Compute(H(.5),640,480,0,{},{}, straight);
  r=fresh_down.Compute(H(.8),640,480,.1,{},done, straight);
  r=fresh_down.Compute(H(.8),640,480,.11,{},observed, straight);
  assert(r.mode==HurdleMode::kApproachTwoBeforeDown);
  r=fresh_down.Compute({},640,480,.12,{},done, straight);
  assert(r.mode==HurdleMode::kWaitCameraDown);
  r=fresh_down.Compute(H(.8),640,480,.15,
                       {CameraMode::kTransition,false},{}, straight);
  assert(r.mode==HurdleMode::kWaitCameraDown);
  r=fresh_down.Compute(H(.8),640,480,.2,
                       {CameraMode::kDown,true},{}, straight);
  assert(r.mode==HurdleMode::kDownSearch && !r.tracked.visible);
  // Camera fallback follows the original region, independently of action ID:
  // all recorded regions, including bottom-row 7/9, go DOWN on loss.
  for(const auto direction : {CruiseDirection::kLeft,CruiseDirection::kRight}) {
    CruiseDecision steering=straight; steering.direction=direction;
    for(const auto pair : std::vector<std::pair<double,bool>>{
        {.5,true}, {.7,true}, {.8,true}}) {
      HurdleController zone(c,3.0);
      const auto selected=zone.Compute(H(pair.first),640,480,4.0,{}, {},steering);
      assert(selected.action.action!=MissionAction::kNone);
      auto missing_done=zone.Compute({},640,480,4.1,{}, done,steering);
      assert(missing_done.camera_request==CameraRequest::kNone);
      ActionExecutionFeedback post_done; post_done.observation_complete=true;
      auto settled=zone.Compute({},640,480,5.2,{}, post_done,steering);
      if(pair.second) {
        assert(settled.mode==HurdleMode::kWaitCameraDown);
        assert(settled.camera_request==CameraRequest::kDown);
      } else {
        assert(settled.mode==HurdleMode::kRecoveryForward);
        assert(settled.camera_request==CameraRequest::kNone);
      }
    }
  }
  // Ten completed 1-step motions in DOWN search, never eleven; no time limit.
  HurdleConfig count_cfg=c;
  count_cfg.down_search_max_steps=10;
  HurdleController step_search(count_cfg,3.0);
  step_search.StartTestNear();
  auto step_result=step_search.Compute({},640,480,100.,{CameraMode::kDown,true},{},straight);
  assert(step_result.mode==HurdleMode::kDownSearch);
  for(int n=1;n<=10;++n) {
    step_result=step_search.Compute({},640,480,100.+10.*n, {CameraMode::kDown,true},{},straight);
    assert(step_result.action.action==MissionAction::kStepForwardOne);
    step_result=step_search.Compute({},640,480,100.+10.*n+.1,
        {CameraMode::kDown,true},done,straight);
    assert(step_search.DownSearchStepsRemaining()==10-n);
    if(n<10) assert(step_result.mode==HurdleMode::kDownSearch);
  }
  assert(step_result.mode==HurdleMode::kFailed);
  assert(step_result.action.action==MissionAction::kNone);
  // Seeing the obstacle after step ten wins over exhausted counter.
  HurdleController final_found(count_cfg,3.0);
  final_found.StartTestNear();
  final_found.Compute({},640,480,0,{CameraMode::kDown,true},{},straight);
  for(int n=1;n<=10;++n) {
    final_found.Compute({},640,480,n*2.,{CameraMode::kDown,true},{},straight);
    step_result=final_found.Compute(n==10 ? std::optional<ObjectTarget>(H(.8)) : std::nullopt,
        640,480,n*2.+.1,{CameraMode::kDown,true},done,straight);
  }
  assert(step_result.mode==HurdleMode::kCoarseAdjust);

  return 0;
}
