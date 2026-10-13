#include "vision_core/goal_controller.hpp"
#include "vision_core/config_loader.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
using namespace vision_core;
static ObjectTarget Board(double u=.5) {
  ObjectTarget t; t.center_px={u*640,240}; t.box_px={u*640-10,230,20,20};
  t.height_px=20; t.confidence=1; return t;
}
static GoalPoseObservation Pose(double x,double z,double yaw=0) {
  return {true,x,z,yaw,1};
}
int main() {
  Intrinsics k{640,480,50,50};
  const auto p = EstimateGoalPoseFromBackboardDepths(50,1,40,1.1,60,.9,k);
  assert(p.valid && std::abs(p.x_m) < 1e-12);
  GoalConfig c=LoadDefaultAlgorithmConfig().goal; c.stable_window=c.stable_min_hits=1; c.lost_frames=2;
  c.smooth_alpha=0; c.post_pickup_wait_sec=0;
  c.fine_one_step_range_error_m=c.fine_two_step_range_error_m=1.0; // Isolate near fine geometry.
  c.target_u_px=320; c.approach_u_deadband_px=32; c.fine_adjust_start_z_m=.8;
  c.hoop_radius_m=.1; c.throw_landing_x_m=0; c.throw_landing_z_m=.5; c.position_tolerance_m=.05;
  c.fine_settle_duration_sec=0; c.shoot_yaw_limit_deg=30;
  c.recovery_timeout_sec=5; c.recovery_reacquire_min_hits=3;
  c.recovery_center_tolerance_px=76.8; c.recovery_settle_duration_sec=.6;
  GoalController controller(c, 3.0); controller.StartAfterPickup(0);
  BallResult carrying; carrying.has_ball=true; carrying.mode=BallMode::kLineFollow;
  controller.UpdateBallState(carrying);
  ActionExecutionFeedback active; active.action_active=true;
  auto r=controller.Compute({}, Board(), Pose(0,.7),640,480,0,true,{},active);
  assert(r.camera_request==CameraRequest::kNone);
  r=controller.Compute({}, Board(), Pose(0,.7),640,480,.1,true,{},{});
  assert(r.camera_request==CameraRequest::kGoal);
  r=controller.Compute({}, {}, {},640,480,.2,true,{CameraMode::kGoal,true},{});
  assert(r.mode==GoalMode::kSearch);
  r=controller.Compute({},Board(),Pose(0,.6),640,480,.3,true,{CameraMode::kGoal,true},{});
  assert(r.mode==GoalMode::kCoarseAdjust);
  r=controller.Compute({},Board(),Pose(0,.6),640,480,.31,true,{CameraMode::kGoal,true},{});
  assert(r.action.action==MissionAction::kDefaultPoseMode);
  ActionExecutionFeedback pose_done; pose_done.action_done=true;
  r=controller.Compute({},Board(),Pose(0,.6),640,480,.32,true,{CameraMode::kGoal,true},pose_done);
  assert(r.mode==GoalMode::kFineAdjust);
  r=controller.Compute({},Board(),Pose(0,.8),640,480,.35,true,{CameraMode::kGoal,true},{});
  assert(r.action.action==MissionAction::kStepForwardHalf);
  ActionExecutionFeedback fine_done; fine_done.action_done=true;
  r=controller.Compute({},Board(),Pose(0,.8),640,480,.36,true,{CameraMode::kGoal,true},fine_done);
  assert(r.action.action==MissionAction::kNone);
  assert(r.action.action!=MissionAction::kDefaultPoseMode);
  r=controller.Compute({},Board(),Pose(.3,.5),640,480,.37,true,{CameraMode::kGoal,true},{});
  assert(r.action.action==MissionAction::kRightSideStep);
  r=controller.Compute({},Board(),Pose(-.3,.5),640,480,.38,true,{CameraMode::kGoal,true},{});
  assert(r.action.action==MissionAction::kLeftSideStep);
  // A one-frame fine window uses the new stationary pose, not the older EMA.
  r=controller.Compute({},Board(),Pose(0,.3),640,480,.4,true,{CameraMode::kGoal,true},{});
  assert(r.action.action==MissionAction::kStepBack);
  r=controller.Compute({},Board(),Pose(0,.6),640,480,.5,true,{CameraMode::kGoal,true},{});
  assert(r.action.action==MissionAction::kShoot);
  assert(std::abs(r.action.target_yaw_deg)<=30);

  // Calibrated camera-frame landing point (-8cm,48cm):
  // rim straight ahead at the measured radial range requires a NEGATIVE yaw.
  GoalConfig calibrated_cfg = c;
  calibrated_cfg.throw_landing_x_m = -.08;
  calibrated_cfg.throw_landing_z_m = .48;
  GoalController calibrated(calibrated_cfg,3.0);
  calibrated.StartTestExecute();
  const double radial_range=std::hypot(-.08,.48);
  auto calibrated_shot=calibrated.Compute({},Board(),Pose(0,radial_range+.1),
      640,480,1.0,true,{CameraMode::kGoal,true},{});
  assert(calibrated_shot.action.action == MissionAction::kShoot);
  assert(calibrated_shot.action.target_yaw_deg == -9);
  // Offset/angle-limit violations must not accidentally fire.
  GoalController off_axis(calibrated_cfg,3.0);
  off_axis.StartTestExecute();
  const auto off_axis_result=off_axis.Compute({},Board(),Pose(.3,radial_range+.1),
      640,480,1.0,true,{CameraMode::kGoal,true},{});
  assert(off_axis_result.action.action != MissionAction::kShoot);

  GoalController timeout_controller(c, 3.0);
  timeout_controller.StartAfterPickup(0);
  timeout_controller.UpdateBallState(carrying);
  r=timeout_controller.Compute({}, Board(), Pose(0,.7),640,480,0,true,{},{});
  assert(r.mode==GoalMode::kWaitCameraGoal);
  r=timeout_controller.Compute({}, {}, {},640,480,3.1,true,{},{});
  assert(r.mode==GoalMode::kFailed);
  assert(r.camera_request==CameraRequest::kNone);

  // No GOAL-camera observation yet: stay still despite intermediate detections.
  GoalController unseen(c, 3.0);
  unseen.StartAfterPickup(0); unseen.UpdateBallState(carrying);
  unseen.Compute({}, Board(), Pose(0,.7),640,480,0,true,{},{});
  r=unseen.Compute({},Board(.9),Pose(0,1.2),640,480,.05,true,
                   {CameraMode::kTransition,false},{});
  assert(r.mode==GoalMode::kWaitCameraGoal);
  r=unseen.Compute({}, {}, {},640,480,.1,true,{CameraMode::kGoal,true},{});
  assert(r.mode==GoalMode::kSearch && !r.tracked.visible);
  r=unseen.Compute({}, {}, {},640,480,.8,true,{CameraMode::kGoal,true},{});
  assert(r.mode==GoalMode::kSearch && r.action.action==MissionAction::kNone);

  // A previously centered target remains stationary when lost.
  GoalController centered(c, 3.0);
  centered.StartAfterPickup(0); centered.UpdateBallState(carrying);
  centered.Compute({}, Board(), Pose(0,.7),640,480,0,true,{},{});
  centered.Compute({}, {}, {},640,480,.1,true,{CameraMode::kGoal,true},{});
  r=centered.Compute({},Board(.52),Pose(0,1.2),640,480,.2,true,
                     {CameraMode::kGoal,true},{});
  assert(r.mode==GoalMode::kApproach);
  centered.Compute({}, {}, {},640,480,.3,true,{CameraMode::kGoal,true},{});
  centered.Compute({}, {}, {},640,480,.4,true,{CameraMode::kGoal,true},{});
  centered.Compute({}, {}, {},640,480,.5,true,{CameraMode::kGoal,true},{});
  r=centered.Compute({}, {}, {},640,480,.6,true,{CameraMode::kGoal,true},{});
  assert(r.mode==GoalMode::kRecoverySearch && r.action.action==MissionAction::kNone);

  // Off-center loss also remains stationary and uses a complete recovery window.
  GoalController left(c, 3.0);
  left.StartAfterPickup(0); left.UpdateBallState(carrying);
  left.Compute({}, Board(), Pose(0,.7),640,480,0,true,{},{});
  left.Compute({}, {}, {},640,480,.1,true,{CameraMode::kGoal,true},{});
  r=left.Compute({},Board(.2),Pose(0,1.2),640,480,.2,true,
                 {CameraMode::kGoal,true},{});
  assert(r.mode==GoalMode::kApproach);
  left.Compute({}, {}, {},640,480,.3,true,{CameraMode::kGoal,true},{});
  r=left.Compute({}, {}, {},640,480,.4,true,{CameraMode::kGoal,true},{});
  assert(r.mode==GoalMode::kRecoverySearch);
  r=left.Compute({}, {}, {},640,480,.5,true,{CameraMode::kGoal,true},{});
  assert(r.action.action==MissionAction::kNone);
  ActionExecutionFeedback busy; busy.action_active=true;
  r=left.Compute({}, {}, {},640,480,.55,true,{CameraMode::kGoal,true},busy);
  assert(r.action.action==MissionAction::kNone);
  ActionExecutionFeedback finished; finished.action_done=true;
  r=left.Compute({}, {}, {},640,480,.6,true,{CameraMode::kGoal,true},finished);
  assert(r.action.action==MissionAction::kNone);
  r=left.Compute({}, {}, {},640,480,.8,true,{CameraMode::kGoal,true},{});
  assert(r.action.action==MissionAction::kNone);
  r=left.Compute({},Board(.25),Pose(0,1.2),640,480,1.3,true,
                 {CameraMode::kGoal,true},{});
  assert(r.mode==GoalMode::kRecoverySearch && r.action.action==MissionAction::kNone);
  r=left.Compute({},Board(.25),Pose(0,1.2),640,480,1.4,true,
               {CameraMode::kGoal,true},{});
  assert(r.mode==GoalMode::kSearch && r.action.action==MissionAction::kNone);
  r=left.Compute({},Board(.25),Pose(0,1.2),640,480,1.5,true,
                 {CameraMode::kGoal,true},{});
  assert(r.mode==GoalMode::kApproach);
  r=left.Compute({},Board(.25),Pose(0,1.2),640,480,1.6,true,
                 {CameraMode::kGoal,true},{});
  assert(r.mode==GoalMode::kApproach);

  GoalController right(c, 3.0);
  right.StartAfterPickup(0); right.UpdateBallState(carrying);
  right.Compute({}, Board(), Pose(0,.7),640,480,0,true,{},{});
  right.Compute({}, {}, {},640,480,.1,true,{CameraMode::kGoal,true},{});
  right.Compute({},Board(.8),Pose(0,1.2),640,480,.2,true,
                {CameraMode::kGoal,true},{});
  right.Compute({}, {}, {},640,480,.3,true,{CameraMode::kGoal,true},{});
  r=right.Compute({}, {}, {},640,480,.4,true,{CameraMode::kGoal,true},{});
  assert(r.mode==GoalMode::kRecoverySearch);
  r=right.Compute({}, {}, {},640,480,.5,true,{CameraMode::kGoal,true},{});
  assert(r.action.action==MissionAction::kNone);
  for (int i=1;i<=5;++i)
    r=right.Compute({}, {}, {},640,480,.5+i,true,{CameraMode::kGoal,true},{});
  assert(right.RecoveryObservationsCompleted()==5);
  assert(r.mode==GoalMode::kFailed && r.action.action==MissionAction::kNone);
  return 0;
}
