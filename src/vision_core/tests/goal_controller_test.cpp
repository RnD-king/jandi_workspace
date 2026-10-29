#include "vision_core/goal_controller.hpp"
#include "vision_core/config_loader.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <vector>
#include <utility>
using namespace vision_core;
static ObjectTarget Board(double u=.5) {
  ObjectTarget t; t.center_px={u*640,240}; t.box_px={u*640-10,230,20,20};
  t.height_px=20; t.confidence=1; return t;
}
static GoalPoseObservation Pose(double x,double z,double yaw=0) {
  return {true,x,z,yaw,1};
}
static void EmptyEntryViewRetriesLine() {
  auto c=LoadDefaultAlgorithmConfig().goal;
  c.stable_window=10;c.stable_min_hits=7;
  c.post_pickup_walk_fallback_actions=0; // Must not bypass the fresh retry walk.
  c.entry_line_retry_max_attempts=2;
  const CameraFeedback forward{CameraMode::kForward,true}, goal_view{CameraMode::kGoal,true};
  const std::deque<bool> misses(10,false);
  GoalController goal(c,3);goal.StartGoalTrial();
  double now=0;
  for(int attempt=0;attempt<=2;++attempt) {
    auto g=goal.Compute({},{},{},640,480,now,true,goal_view,{});
    assert(g.mode==GoalMode::kSearch);
    assert(goal.EntryLineRetries()==attempt && !goal.EntryLineRetryPending());
    g=goal.Compute({},{},{},640,480,now+.1,true,goal_view,{});
    assert(g.mode==GoalMode::kSearch); // No transition before a completed window.
    goal.UseVerifiedObservation({}, {},misses,misses,640,480);
    g=goal.Compute({},{},{},640,480,now+1.1,true,goal_view,{false,false,true});
    assert(g.action.action==MissionAction::kNone && goal.HasBall());
    if(attempt==2) {
      assert(g.mode==GoalMode::kFailed && g.camera_request==CameraRequest::kNone);
      assert(goal.EntryLineRetries()==2);
      break;
    }
    assert(g.mode==GoalMode::kEntryRetryCameraForward);
    assert(g.camera_request==CameraRequest::kForward);
    assert(goal.EntryLineRetries()==attempt+1 && goal.EntryLineRetryPending());
    g=goal.Compute({},{},{},640,480,now+1.2,true,{CameraMode::kTransition,false},{});
    assert(g.mode==GoalMode::kEntryRetryCameraForward);
    g=goal.Compute({},{},{},640,480,now+1.3,true,forward,{});
    assert(g.mode==GoalMode::kLineFollow && !g.active && goal.HasBall());
    assert(goal.RequiredLineWalkActions()==1);
    g=goal.Compute({},{},{},640,480,now+1.4,true,forward,{},true,{},false);
    assert(g.mode==GoalMode::kLineFollow); // Old/zero fallback is not a completed retry block.
    g=goal.Compute({},{},{},640,480,now+1.5,true,forward,{},true,{},true);
    assert(g.mode==GoalMode::kWaitCameraGoal && g.camera_request==CameraRequest::kGoal);
    now+=2;
  }
  goal.Reset();
  assert(goal.EntryLineRetries()==0 && !goal.EntryLineRetryPending());

  // An intermittent valid board or image-only Depth failure is not an empty
  // view, even when the last frame and the final stationary mean are absent.
  for(bool image_only:{false,true}) {
    GoalController partial(c,3);partial.StartGoalTrial();
    partial.Compute({},{},{},640,480,0,true,goal_view,{});
    if(image_only) partial.ObserveEntryBackboard(true);
    else partial.ObserveOnly(Board(),{},640,480);
    partial.UseVerifiedObservation({}, {},misses,misses,640,480);
    auto g=partial.Compute({},{},{},640,480,1.1,true,goal_view,{false,false,true},true,{},false,image_only);
    assert(g.mode==(image_only?GoalMode::kRecoverySearch:GoalMode::kRecoveryForwardStep));
    assert(partial.EntryLineRetries()==0);
    assert(g.camera_request==CameraRequest::kNone);
  }
}
static void CoarseTurnAlignment(double yaw_limit_deg) {
  auto c=LoadDefaultAlgorithmConfig().goal;
  c.stable_window=c.stable_min_hits=1;
  c.smooth_alpha=0;
  c.throw_landing_x_m=0;
  c.throw_landing_z_m=.4;
  c.shoot_yaw_limit_deg=yaw_limit_deg;
  const CameraFeedback camera{CameraMode::kGoal,true};
  const auto enter_coarse=[&](GoalController& goal,const GoalPoseObservation& pose) {
    goal.StartTestNear();
    goal.Compute({},{},{},640,480,0,true,camera,{});
    const auto r=goal.Compute({},Board(),pose,640,480,.1,true,camera,{},true,pose.z_m);
    assert(r.mode==GoalMode::kCoarseAdjust);
  };
  for(double sign:{-1.0,1.0}) {
    const auto off_axis=Pose(sign*.4,.7);
    GoalController goal(c,3);
    enter_coarse(goal,off_axis);
    auto r=goal.Compute({},Board(),off_axis,640,480,.2,true,camera,{},false);
    const auto turn=sign<0 ? MissionAction::kTurnLeftInPlace
                          : MissionAction::kStepForwardRightOneHurdle;
    // Preview and execution choose the same in-place WALK motion.
    assert(r.mode==GoalMode::kCoarseAdjust && r.action.action==turn);
    r=goal.Compute({},Board(),off_axis,640,480,.21,true,camera,{});
    assert(r.action.action==turn && goal.RecoveryTurnsCompleted()==0);
    ActionExecutionFeedback busy;busy.action_active=true;
    r=goal.Compute({},Board(),off_axis,640,480,.3,true,camera,busy);
    assert(r.action.action==MissionAction::kNone);
    ActionExecutionFeedback done;done.action_done=true;
    r=goal.Compute({},Board(),off_axis,640,480,.4,true,camera,done);
    assert(r.action.action==MissionAction::kNone);
    // The verified post-DONE pose is reused to resume forward approach.
    const auto aligned=Pose(0,.7);
    goal.UseVerifiedObservation(Board(),aligned,{true},{true},640,480);
    r=goal.Compute({},Board(),aligned,640,480,1.4,true,camera,done);
    assert(r.action.action==MissionAction::kStepForwardOne);

    // Below 0.15m, switch feet before handling the same angular violation.
    GoalController close(c,3);
    // Keep the rim at the target radial range so distance correction does
    // not take priority over the side-step this scenario is testing.
    const auto close_pose=Pose(sign*.3,c.hoop_radius_m+
        std::sqrt(c.throw_landing_z_m*c.throw_landing_z_m-.3*.3));
    enter_coarse(close,close_pose);
    r=close.Compute({},Board(),close_pose,640,480,.2,true,camera,{});
    assert(r.mode==GoalMode::kWaitFinePose);
    assert(r.action.action==MissionAction::kDefaultPoseMode);
    r=close.Compute({},Board(),close_pose,640,480,.3,true,camera,done);
    assert(r.mode==GoalMode::kFineAdjust);
    r=close.Compute({},Board(),close_pose,640,480,.4,true,camera,{});
    assert(r.action.action==(sign<0 ? MissionAction::kLeftSideStep
                                  : MissionAction::kRightSideStep));
  }
  // Both forward step sizes remain available inside the angular limit.
  GoalController aligned(c,3);
  enter_coarse(aligned,Pose(.1,.8));
  auto r=aligned.Compute({},Board(),Pose(.1,.8),640,480,.2,true,camera,{});
  assert(r.action.action==MissionAction::kStepForwardTwo);
  r=aligned.Compute({},Board(),Pose(0,.7),640,480,.3,true,camera,{});
  assert(r.action.action==MissionAction::kStepForwardOne);

  // Check both sides of the configured 0.15m stance boundary while off-axis.
  for(double margin:{-1e-6,1e-6}) {
    const double range=c.throw_landing_z_m+c.fine_one_step_range_error_m+margin;
    const auto pose=Pose(.4,c.hoop_radius_m+std::sqrt(range*range-.4*.4));
    GoalController goal(c,3);enter_coarse(goal,pose);
    r=goal.Compute({},Board(),pose,640,480,.2,true,camera,{});
    assert(r.action.action==(margin<0 ? MissionAction::kDefaultPoseMode
                                     : MissionAction::kStepForwardRightOneHurdle));
  }

  // The existing configurable FINE angular limit also governs WALK alignment.
  const auto off_axis=Pose(.4,.7);
  const double angle=std::atan2(.4,.7-c.hoop_radius_m)*180.0/std::acos(-1.0);
  for(double margin:{-1e-6,1e-6}) {
    auto threshold=c;threshold.shoot_yaw_limit_deg=angle+margin;
    GoalController goal(threshold,3);enter_coarse(goal,off_axis);
    r=goal.Compute({},Board(),off_axis,640,480,.2,true,camera,{});
    assert(r.action.action==(margin<0 ? MissionAction::kStepForwardRightOneHurdle
                                     : MissionAction::kStepForwardTwo));
  }
}
static GoalPoseObservation BoardPoseFromPhysicalRim(double rim_z,double phi) {
  // Construct actual board edge positions from an independent tangent/normal
  // geometry, then pass their pinhole pixels/depths through the real estimator.
  const double radius=.1,half_width=.1;
  const double bx=radius*std::sin(phi),bz=rim_z+radius*std::cos(phi);
  const double lx=bx-half_width*std::cos(phi),lz=bz+half_width*std::sin(phi);
  const double rx=bx+half_width*std::cos(phi),rz=bz-half_width*std::sin(phi);
  const Intrinsics intrinsics{600,600,320,240};
  const auto u=[](double x,double z) {return 320+600*x/z;};
  const auto pose=EstimateGoalPoseFromBackboardDepths(u(bx,bz),bz,u(lx,lz),lz,u(rx,rz),rz,intrinsics);
  assert(pose.valid && std::abs(pose.yaw_rad-phi)<1e-12);
  return pose;
}
static void TiltedBackboardNormal() {
  auto c=LoadDefaultAlgorithmConfig().goal;
  c.stable_window=c.stable_min_hits=1;c.smooth_alpha=0;
  c.hoop_radius_m=.1;c.throw_landing_x_m=0;c.throw_landing_z_m=.4;
  c.position_tolerance_m=.01;c.shoot_yaw_limit_deg=20;
  c.fine_adjust_start_z_m=.8;
  c.fine_one_step_range_error_m=.15;c.fine_two_step_range_error_m=.30;
  const CameraFeedback camera{CameraMode::kGoal,true};
  for (double degrees : {-30.0,0.0,30.0}) {
    const double phi=degrees*std::acos(-1.0)/180;
    const auto pose=BoardPoseFromPhysicalRim(.4,phi);
    GoalController fine(c,3);fine.StartTestExecute();
    const auto shot=fine.Compute({},Board(),pose,640,480,0,true,camera,{});
    // Tilting the board while holding its real rim fixed must not invent
    // lateral/forward motion or a different waist angle.
    assert(shot.action.action==MissionAction::kShoot && shot.action.target_yaw_deg==0);
    const auto distant=BoardPoseFromPhysicalRim(.695,phi);
    GoalController coarse(c,3);coarse.StartTestNear();
    coarse.Compute({},{},{},640,480,0,true,camera,{});
    auto r=coarse.Compute({},Board(),distant,640,480,.1,true,camera,{},true,distant.z_m);
    assert(r.mode==GoalMode::kCoarseAdjust);
    r=coarse.Compute({},Board(),distant,640,480,.2,true,camera,{});
    // Actual radial excess is .295m: one step. The former +sin correction
    // adds a spurious .10m lateral offset and incorrectly selects two steps.
    assert(r.action.action==MissionAction::kStepForwardOne);
  }
}
static void RotationBeforeTranslation() {
  auto c=LoadDefaultAlgorithmConfig().goal;
  c.stable_window=c.stable_min_hits=1;c.smooth_alpha=0;
  c.hoop_radius_m=.1;c.throw_landing_x_m=-.08;c.throw_landing_z_m=.4;
  c.shoot_yaw_limit_deg=20;c.position_tolerance_m=.06;
  const double pi=std::acos(-1.0);
  const CameraFeedback camera{CameraMode::kGoal,true};
  const auto rim_at=[&](double degrees,double scale=1.0) {
    const double a=degrees*pi/180;
    return Pose(scale*(std::cos(a)*c.throw_landing_x_m-std::sin(a)*c.throw_landing_z_m),
                scale*(std::sin(a)*c.throw_landing_x_m+std::cos(a)*c.throw_landing_z_m)+c.hoop_radius_m);
  };
  for (int sign : {-1,1}) {
    GoalController goal(c,3);goal.StartTestExecute();
    const auto pose=rim_at(sign*22);
    // Preview can recommend a shot without entering SHOOT.
    auto r=goal.Compute({},Board(),pose,640,480,0,true,camera,{},false);
    assert(r.mode==GoalMode::kFineAdjust && r.action.action==MissionAction::kShoot);
    assert(r.action.target_yaw_deg==sign*20);
    r=goal.Compute({},Board(),pose,640,480,.1,true,camera,{});
    assert(r.mode==GoalMode::kShoot && r.action.target_yaw_deg==sign*20);
    assert(std::abs(r.shoot_yaw_rad-sign*20*pi/180)<1e-12);
    // A later pose must not replace the accepted endpoint with the ideal
    // +/-22 degree angle, or recompute a different throw command.
    r=goal.Compute({},Board(),rim_at(-sign*10),640,480,.2,true,camera,{});
    assert(r.action.action==MissionAction::kShoot && r.action.target_yaw_deg==sign*20);
    assert(std::abs(r.shoot_yaw_rad-sign*20*pi/180)<1e-12);

    GoalController too_far_sideways(c,3);too_far_sideways.StartTestExecute();
    r=too_far_sideways.Compute({},Board(),rim_at(sign*40),640,480,0,true,camera,{});
    assert(r.action.action==(sign<0 ? MissionAction::kRightSideStep : MissionAction::kLeftSideStep));
  }
  for (double half_degree : {-.5,.5}) {
    GoalController tied(c,3);tied.StartTestExecute();
    const auto r=tied.Compute({},Board(),rim_at(half_degree),640,480,0,true,camera,{});
    assert(r.action.action==MissionAction::kShoot && r.action.target_yaw_deg==0);
  }
  // A fractional configured limit must not round up to a prohibited integer.
  auto fractional=c;fractional.shoot_yaw_limit_deg=19.6;fractional.position_tolerance_m=.006;
  GoalController within(fractional,3);within.StartTestExecute();
  auto r=within.Compute({},Board(),rim_at(19.6),640,480,0,true,camera,{});
  assert(r.action.action==MissionAction::kShoot && r.action.target_yaw_deg==19);
  fractional.position_tolerance_m=.001;
  GoalController outside(fractional,3);outside.StartTestExecute();
  r=outside.Compute({},Board(),rim_at(19.6),640,480,0,true,camera,{});
  assert(r.action.action!=MissionAction::kShoot);

  // Rotation preserves radius; large range errors still need translation.
  for (double scale : {.7,1.3}) {
    GoalController goal(c,3);goal.StartTestExecute();
    r=goal.Compute({},Board(),rim_at(0,scale),640,480,0,true,camera,{});
    assert(r.action.action==(scale>1 ? MissionAction::kStepForwardHalf : MissionAction::kStepBack));
  }
  // A missing stationary window must not be bypassed by the new selector.
  c.stable_window=c.stable_min_hits=2;
  GoalController stable(c,3);stable.StartTestExecute();
  r=stable.Compute({},Board(),rim_at(22),640,480,0,true,camera,{});
  assert(r.action.action==MissionAction::kNone);
  r=stable.Compute({},Board(),rim_at(22),640,480,.1,true,camera,{});
  assert(r.action.action==MissionAction::kShoot && r.action.target_yaw_deg==20);
}
static void StatelessShotBench() {
  auto c=LoadDefaultAlgorithmConfig().goal;
  c.hoop_radius_m=.2;c.throw_landing_x_m=0;c.throw_landing_z_m=1;
  c.position_tolerance_m=.02;c.shoot_yaw_limit_deg=20;
  const GoalController controller(c,3);
  assert(controller.PlanFineAction({}).action==MissionAction::kNone);
  assert(controller.PlanFineAction(Pose(0,1.2)).action==MissionAction::kShoot);
  assert(controller.PlanFineAction(Pose(0,2)).action==MissionAction::kStepForwardHalf);
  assert(controller.PlanFineAction(Pose(0,.6)).action==MissionAction::kStepBack);
  const double angle=10*3.14159265358979323846/180;
  const auto shot=controller.PlanFineAction(Pose(std::sin(angle),.2+std::cos(angle)));
  assert(shot.action==MissionAction::kShoot && shot.target_yaw_deg==-10);
  assert(!controller.HasBall());
  assert(controller.PlanFineAction(Pose(0,1.2)).action==MissionAction::kShoot);
}
int main() {
  StatelessShotBench();
  EmptyEntryViewRetriesLine();
  TiltedBackboardNormal();
  RotationBeforeTranslation();
  CoarseTurnAlignment(15);
  CoarseTurnAlignment(30);
  Intrinsics k{640,480,50,50};
  const auto p = EstimateGoalPoseFromBackboardDepths(50,1,40,1.1,60,.9,k);
  assert(p.valid && std::abs(p.x_m) < 1e-12);
  GoalConfig c=LoadDefaultAlgorithmConfig().goal; c.stable_window=c.stable_min_hits=1; c.lost_frames=2;
  c.recovery_turn_max=0; // Isolate the pre-existing stationary recovery FSM.
  c.smooth_alpha=0; c.post_pickup_walk_fallback_actions=0;
  c.fine_one_step_range_error_m=c.fine_two_step_range_error_m=1.0; // Isolate near fine geometry.
  c.target_u_px=320; c.approach_u_deadband_px=32; c.fine_adjust_start_z_m=.8;
  c.hoop_radius_m=.1; c.throw_landing_x_m=0; c.throw_landing_z_m=.5; c.position_tolerance_m=.05;
   c.shoot_yaw_limit_deg=30;
  c.recovery_center_tolerance_px=76.8;
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

  // All six approach cells use the same mapping in FORWARD and GOAL views.
  c.approach_short_z_m=1.2;
  for (double depth : {1.8,1.2,1.0}) {
    const bool near=depth<=c.approach_short_z_m;
    for (const auto &scenario : std::vector<std::pair<double,MissionAction>>{
        {.2,near ? MissionAction::kTurnLeftInPlace : MissionAction::kStepForwardLeftOne},
        {.5,MissionAction::kStepForwardThree},
        {.8,near ? MissionAction::kStepForwardRightOneHurdle : MissionAction::kStepForwardRightOne}}) {
      GoalController far(c,3.0);
      far.StartForwardTrial();
      const auto approach=far.Compute({},Board(scenario.first),{},640,480,
                                     0,true,{CameraMode::kForward,true},{},true,depth);
      assert(approach.action.action==scenario.second);
      assert(approach.action.category==ActionCategory::kLocomotion);
      assert(approach.camera_request==CameraRequest::kNone);
      GoalController goal_view(c,3.0);
      goal_view.StartAfterPickup(0);
      goal_view.UpdateBallState(carrying);
      goal_view.Compute({},Board(),{},640,480,0,true,{},{});
      goal_view.Compute({}, {}, {},640,480,.1,true,{CameraMode::kGoal,true},{});
      const auto goal_approach=goal_view.Compute({},Board(scenario.first),{},640,480,
          .2,true,{CameraMode::kGoal,true},{},true,depth);
      assert(goal_approach.action.action==scenario.second);
    }
  }
  GoalController mid(c,3.0);
  mid.StartForwardTrial();
  const auto middle=mid.Compute({},Board(.5),{},640,480,
                                0,true,{CameraMode::kForward,true},{},true,1.0);
  assert(middle.action.action==MissionAction::kStepForwardThree);

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

  // A previously centered target advances exactly one ordinary step when lost.
  GoalController centered(c, 3.0);
  centered.StartAfterPickup(0); centered.UpdateBallState(carrying);
  centered.Compute({}, Board(), Pose(0,.7),640,480,0,true,{},{});
  centered.Compute({}, {}, {},640,480,.1,true,{CameraMode::kGoal,true},{});
  r=centered.Compute({},Board(.52),Pose(0,1.2),640,480,.2,true,
                     {CameraMode::kGoal,true},{});
  assert(r.mode==GoalMode::kApproach);
  centered.Compute({}, {}, {},640,480,.3,true,{CameraMode::kGoal,true},{});
  centered.Compute({}, {}, {},640,480,.4,true,{CameraMode::kGoal,true},{});
  r=centered.Compute({}, {}, {},640,480,.5,true,{CameraMode::kGoal,true},{});
  assert(r.mode==GoalMode::kRecoveryForwardStep && r.action.action==MissionAction::kStepForwardOne);
  r=centered.Compute({}, {}, {},640,480,.6,true,{CameraMode::kGoal,true},{});
  assert(r.mode==GoalMode::kRecoveryForwardStep && r.action.action==MissionAction::kNone);
  r=centered.Compute({}, {}, {},640,480,1.7,true,{CameraMode::kGoal,true},{true,false,true});
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
