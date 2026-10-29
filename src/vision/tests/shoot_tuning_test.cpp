#include "vision/shoot_tuning.hpp"
#include "vision_core/config_loader.hpp"
#include "vision_core/control_command.hpp"
#include <limits>
#include <stdexcept>

namespace {
void Check(bool value,const char* message) {
  if (!value) throw std::runtime_error(message);
}
bool Near(double a,double b) { return std::abs(a-b)<1e-8; }
void ClickDepthAndProjection() {
  vision_core::GoalConfig config;
  config.hoop_radius_m=.2; config.throw_landing_z_m=1;
  const vision_core::Intrinsics intrinsics{100,100,5,5};
  cv::Mat depth(11,11,CV_16UC1,cv::Scalar(2000));
  depth.at<std::uint16_t>(5,5)=0;
  auto click=vision::SampleClickedBackboard(depth,5,5,intrinsics,config,0);
  Check(click.depth_m && Near(*click.depth_m,2),"millimeters must convert to meters");
  Check(click.valid_pixels==24 && click.total_pixels==25,"click must sample exactly 5x5");
  Check(click.landing.rim_valid && Near(click.landing.rim_x_m,0) &&
      Near(click.landing.rim_z_m,1.8),"front-facing board must offset toward robot");
  click=vision::SampleClickedBackboard(depth,0,0,intrinsics,config,90);
  Check(click.total_pixels==9 && click.valid_pixels==9,"edge ROI must clip to image bounds");
  Check(Near(click.landing.rim_x_m,-.3) && Near(click.landing.rim_z_m,2),
      "click yaw and pinhole projection must use the production rim convention");
  cv::Mat floating(5,5,CV_32FC1,cv::Scalar(0));
  floating.at<float>(0,0)=1; floating.at<float>(0,1)=3;
  floating.at<float>(1,1)=std::numeric_limits<float>::quiet_NaN();
  floating.at<float>(1,2)=std::numeric_limits<float>::infinity();
  floating.at<float>(1,3)=-1;
  click=vision::SampleClickedBackboard(floating,2,2,intrinsics,config,0);
  Check(click.depth_m && Near(*click.depth_m,2) && click.valid_pixels==2,
      "even valid counts must average the two middle values; invalid depths excluded");
  floating.setTo(0);
  Check(!vision::SampleClickedBackboard(floating,2,2,intrinsics,config,0).depth_m,
      "empty valid-depth set must fail explicitly");
  Check(!vision::SampleClickedBackboard(depth,-1,5,intrinsics,config,0).depth_m,
      "out-of-image clicks must not project");
  auto invalid=intrinsics; invalid.fx=0;
  click=vision::SampleClickedBackboard(depth,5,5,invalid,config,0);
  Check(click.depth_m && !click.landing.rim_valid,"invalid calibration must not produce coordinates");
}
void StatelessFineDecision() {
  auto config=vision_core::LoadDefaultAlgorithmConfig();
  config.goal.hoop_radius_m=.2;
  config.goal.throw_landing_x_m=0; config.goal.throw_landing_z_m=1;
  config.goal.position_tolerance_m=.02; config.goal.shoot_yaw_limit_deg=20;
  const vision_core::GoalController planner(config.goal,config.camera_motion_timeout_sec);
  using A=vision_core::MissionAction;
  Check(planner.PlanFineAction({}).action==A::kNone,"invalid pose must hold");
  const auto shot=planner.PlanFineAction({true,0,1.2,0,1});
  Check(shot.action==A::kShoot && shot.target_yaw_deg==0,"reachable rim must shoot");
  Check(planner.PlanFineAction({true,0,2,0,1}).action==A::kStepForwardHalf,"far rim needs one fine step");
  Check(planner.PlanFineAction({true,0,.6,0,1}).action==A::kStepBack,"near rim needs one back step");
  const double angle=10*vision::kDebugPi/180;
  const auto angled=planner.PlanFineAction({true,std::sin(angle),.2+std::cos(angle),0,1});
  Check(angled.action==A::kShoot && angled.target_yaw_deg==-10,"right rim needs negative waist yaw");
  Check(planner.PlanFineAction({true,0,1.2,0,1}).action==A::kShoot && !planner.HasBall(),
      "bench planning must neither consume nor require mission ball state");

  // A bench snapshot is independent of the production ten-frame entry vote.
  config.enable_ball=config.enable_goal=config.enable_hurdle=false;
  config.initial_has_ball=config.goal_trial_start_goal=false;
  config.backboard_candidate_tracking.enabled=false;
  config.object_targets.backboard_min_box_width=config.object_targets.backboard_min_box_height=0;
  config.backboard_min_depth_m=.1; config.backboard_max_depth_m=5;
  const vision_core::MissionController observer(config);
  vision_core::PerceptionFrameInput input;
  input.image_width=640; input.image_height=480;
  input.intrinsics={600,600,320,240};
  input.allow_new_commands=input.allow_new_mission_entry=input.advance_line_fsm=false;
  input.camera_feedback={vision_core::CameraMode::kGoal,true};
  vision_core::PerceptionDetection board;
  board.detection={{220,100,200,100},1,config.object_targets.backboard_class_id};
  board.center_depth_m=board.left_depth_m=board.right_depth_m=1.2;
  input.detections={board};
  const auto first=observer.PreviewPerception(input);
  Check(first.perception.targets.backboard.has_value() && first.mission.command.action_id==0,
      "single-frame observation must not dispatch a mission command");
  input.detections.clear();
  Check(!observer.PreviewPerception(input).perception.targets.backboard,
      "snapshot must not reuse a board from a previous frame");
}
void StoredGoalCenters() {
  auto config=vision_core::LoadDefaultAlgorithmConfig();
  config.goal.hoop_radius_m=.2;
  config.goal.throw_landing_x_m=0; config.goal.throw_landing_z_m=1;
  config.goal.position_tolerance_m=.02; config.goal.shoot_yaw_limit_deg=20;
  using A=vision_core::MissionAction;
  const auto plan=[&](const vision::GoalCenterTarget& target) {
    return vision::PlanStoredGoalCenter(target,config.goal,config.camera_motion_timeout_sec);
  };
  const vision::GoalCenterTarget manual{0,1};
  Check(plan(manual).action==A::kShoot,"manual coordinates are rim coordinates, not board coordinates");
  Check(plan({0,1.5}).action==A::kStepForwardHalf,"manual target must use the fine movement decision");
  Check(plan({0,0}).action==A::kNone && plan({0,-1}).action==A::kNone &&
      plan({std::numeric_limits<double>::quiet_NaN(),1}).action==A::kNone,
      "invalid manually entered targets must not dispatch");

  const cv::Mat depth(11,11,CV_16UC1,cv::Scalar(1200));
  const auto click=vision::SampleClickedBackboard(depth,5,5,{100,100,5,5},config.goal,0);
  const auto clicked=vision::GoalCenterFromLanding(click.landing);
  Check(clicked && Near(clicked->y_m,1) && plan(*clicked).action==A::kShoot,
      "adopted click must not subtract the rim offset twice");
  Check(!vision::GoalCenterFromLanding({}),"failed click projection cannot become a stored target");

  const vision_core::GoalController original(config.goal,config.camera_motion_timeout_sec);
  for (double yaw:{-.5,0.,.5}) {
    vision_core::MissionFrameResult result;
    result.active_mission=vision_core::MissionType::kGoal;
    result.goal.fine_decision_pose_used=true;
    result.goal.fine_decision_pose={true,.1,1.2,yaw,1};
    const auto target=vision::GoalCenterFromLanding(vision::ComputeGoalLandingDebug(config.goal,result));
    const auto expected=original.PlanFineAction(result.goal.fine_decision_pose);
    Check(target.has_value(),"valid automatic calculation must provide a stored rim center");
    const auto actual=plan(*target);
    Check(actual.action==expected.action && actual.target_yaw_deg==expected.target_yaw_deg,
        "saved automatic result must retain the production decision and signed waist angle");
  }
  // Editing geometry or receiving an unrelated new image must not move a saved rim.
  config.goal.hoop_radius_m=.7;
  config.goal.throw_landing_z_m=2;
  Check(plan(manual).action==A::kStepBack,"current throw parameters must apply to the saved coordinates");
  config.goal.throw_landing_z_m=1;
  Check(plan(manual).action==A::kShoot && plan(*clicked).action==A::kShoot,
      "radius edits must not reinterpret stored rim coordinates as a new board measurement");
}
void ActualDoneAndExactlyOneAction() {
  using namespace vision_core;
  ControlCommandCoordinator transport({100,2});
  const ActionRequest shot{MissionAction::kShoot,ActionCategory::kMission,-10,false};
  const auto tick=[&](double now,const CommandDeliveryFeedback& feedback={}) {
    return transport.Compute(MissionType::kGoal,0,{},CameraRequest::kNone,feedback,now);
  };
  auto command=transport.Compute(MissionType::kGoal,0,shot,CameraRequest::kNone,{},0);
  Check(command.action_id==100 && command.target_yaw_deg==-10,"shot must preserve signed yaw and ID");
  Check(tick(.1).action_id==100,"pre-ACK retry must reuse the complete command ID");
  command=tick(.2,{100,true,false,false});
  Check(command.command_type==CommandType::kHold && transport.HasPendingAction(),"ACK must wait for DONE");
  tick(.3,{100,false,false,true});
  Check(transport.HasPendingAction() && !transport.HasQueuedAction(),"READY must not queue or complete");
  tick(.4,{99,false,true,false});
  Check(transport.HasPendingAction(),"unrelated DONE must not unlock the buttons");
  tick(.5,{100,false,true,false});
  Check(!transport.HasPendingAction(),"matching physical DONE must finish the single action");
  command=tick(.6);
  Check(command.action_id==0 && command.action==MissionAction::kNone,"DONE must not start another action");
  command=transport.Compute(MissionType::kGoal,0,shot,CameraRequest::kNone,{},.7);
  Check(command.action_id==101,"explicit second click must allocate a new action ID");
  tick(3);
  Check(transport.ActionAckTimedOut() && transport.HasPendingAction(),"timeout must not synthesize DONE");
}
void UnavailableRequestsDoNotLatchTheBench() {
  using namespace vision_core;
  using Reason=vision::ShootRequestUnavailable;
  ControlCommandCoordinator transport({200,1});
  Check(vision::ReleaseUnavailableShootRequest(transport,false)==Reason::kNone,
      "offline idle bench must remain available without creating a transaction");
  const ActionRequest step{MissionAction::kStepForwardHalf,ActionCategory::kLocomotion,0,false};
  const auto begin=[&](double now) {
    return transport.Compute(MissionType::kGoal,0,step,CameraRequest::kNone,{},now);
  };
  const auto tick=[&](double now,const CommandDeliveryFeedback& feedback={}) {
    return transport.Compute(MissionType::kGoal,0,{},CameraRequest::kNone,feedback,now);
  };
  const auto first=begin(0);
  Check(vision::ReleaseUnavailableShootRequest(transport,true)==Reason::kNone && transport.HasPendingAction(),
      "connected request must still wait for real feedback before timeout");
  tick(1.1);
  Check(vision::ReleaseUnavailableShootRequest(transport,true)==Reason::kNoAck && !transport.HasPendingAction(),
      "missing motor/executor ACK must release the bench after a notification");
  const auto idle=tick(1.2,{first.action_id,false,true,false});
  Check(idle.action_id==0 && idle.action==MissionAction::kNone,
      "late feedback must neither execute another action nor recreate the abandoned request");
  const auto second=begin(2);
  Check(second.action_id==first.action_id+1,"a new explicit click must not reuse the abandoned ID");
  Check(vision::ReleaseUnavailableShootRequest(transport,false)==Reason::kDisconnected && !transport.HasPendingAction(),
      "losing the subscriber must end the local wait without a motor-stop action");
  const auto third=begin(3);
  tick(3.1,{third.action_id,true,false,false});
  tick(100);
  Check(vision::ReleaseUnavailableShootRequest(transport,true)==Reason::kNone && transport.HasPendingAction(),
      "an ACKed connected real motion must retain its actual DONE contract");
  tick(101,{third.action_id,false,true,false});
  Check(!transport.HasPendingAction(),"connected real motion still finishes only on matching DONE");
}
} // namespace
int main() {
  ClickDepthAndProjection();
  StatelessFineDecision();
  StoredGoalCenters();
  ActualDoneAndExactlyOneAction();
  UnavailableRequestsDoNotLatchTheBench();
}
