#include "vision_core/goal_controller.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
using namespace vision_core;
static ObjectTarget Board(double u=.5) {
  ObjectTarget t; t.center_px={u*100,50}; t.box_px={u*100-10,40,20,20};
  t.height_px=20; t.confidence=1; return t;
}
static GoalPoseObservation Pose(double x,double z,double yaw=0) {
  return {true,x,z,yaw,1};
}
int main() {
  Intrinsics k{100,100,50,50};
  const auto p = EstimateGoalPoseFromBackboardDepths(50,1,40,1.1,60,.9,k);
  assert(p.valid && std::abs(p.x_m) < 1e-12);
  GoalConfig c{}; c.stable_window=c.stable_min_hits=1; c.lost_frames=2;
  c.smooth_alpha=0; c.post_pickup_wait_sec=0;
  c.target_u_norm=.5; c.approach_u_deadband=.05; c.fine_adjust_start_z_m=.8;
  c.hoop_radius_m=.1; c.throwing_range_m=.5; c.position_tolerance_m=.05;
  c.fine_settle_duration_sec=0; c.shoot_yaw_limit_deg=30;
  GoalController controller(c, 3.0); controller.StartAfterPickup(0);
  BallResult carrying; carrying.has_ball=true; carrying.mode=BallMode::kLineFollow;
  controller.UpdateBallState(carrying);
  ActionExecutionFeedback active; active.action_active=true;
  auto r=controller.Compute({}, {}, {},100,100,0,true,{},active);
  assert(r.camera_request==CameraRequest::kNone);
  r=controller.Compute({}, {}, {},100,100,.1,true,{},{});
  assert(r.camera_request==CameraRequest::kGoal);
  r=controller.Compute({}, {}, {},100,100,.2,true,{CameraMode::kGoal,true},{});
  assert(r.mode==GoalMode::kSearch);
  r=controller.Compute({},Board(),Pose(0,.6),100,100,.3,true,{CameraMode::kGoal,true},{});
  assert(r.mode==GoalMode::kFineAdjust);
  r=controller.Compute({},Board(),Pose(0,.8),100,100,.35,true,{CameraMode::kGoal,true},{});
  assert(r.action.action==MissionAction::kStepForwardHalf);
  ActionExecutionFeedback fine_done; fine_done.action_done=true;
  r=controller.Compute({},Board(),Pose(0,.8),100,100,.36,true,{CameraMode::kGoal,true},fine_done);
  assert(r.action.action==MissionAction::kNone);
  assert(r.action.action!=MissionAction::kDefaultPoseMode);
  r=controller.Compute({},Board(),Pose(.3,.5),100,100,.37,true,{CameraMode::kGoal,true},{});
  assert(r.action.action==MissionAction::kRightSideStep);
  r=controller.Compute({},Board(),Pose(-.3,.5),100,100,.38,true,{CameraMode::kGoal,true},{});
  assert(r.action.action==MissionAction::kLeftSideStep);
  // smoothing에는 직전 .6m가 남아 있어도 fine은 settle 뒤 현재 raw RGB-D
  // pose(.3m)를 사용해야 한다.
  r=controller.Compute({},Board(),Pose(0,.3),100,100,.4,true,{CameraMode::kGoal,true},{});
  assert(r.action.action==MissionAction::kStepBack);
  r=controller.Compute({},Board(),Pose(0,.6),100,100,.5,true,{CameraMode::kGoal,true},{});
  assert(r.action.action==MissionAction::kShoot);
  assert(std::abs(r.action.target_yaw_deg)<=30);

  GoalController timeout_controller(c, 3.0);
  timeout_controller.StartAfterPickup(0);
  timeout_controller.UpdateBallState(carrying);
  r=timeout_controller.Compute({}, {}, {},100,100,0,true,{},{});
  assert(r.mode==GoalMode::kWaitCameraGoal);
  r=timeout_controller.Compute({}, {}, {},100,100,3.1,true,{},{});
  assert(r.mode==GoalMode::kFailed);
  assert(r.camera_request==CameraRequest::kNone);
  return 0;
}
