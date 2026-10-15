#include "vision/mission_debug_view.hpp"
#include <limits>
#include <stdexcept>
#include <vector>

static void Check(bool value,const char* message) {
  if(!value)throw std::runtime_error(message);
}
struct AdapterDetection {cv::Rect box;float confidence;int class_id;};
int main() {
  // Fixed rendering scenario, independent of the operator's YAML.
  vision_core::MissionControllerConfig c;
  c.line_features.image_center_u=320;
  c.line_detection.class_id=0;c.line_detection.confidence=.5;
  c.object_targets.ball_class_id=1;c.object_targets.goal_class_id=2;
  c.object_targets.backboard_class_id=3;c.object_targets.hurdle_class_id=4;
  c.ball.pickup_u1_px=310;c.ball.pickup_u2_px=385;
  c.ball.pickup_v1_px=80;c.ball.pickup_v2_px=130;
  c.hurdle.down_one_step_v_px=100;c.hurdle.fine_mode_v_px=180;c.hurdle.contact_v_px=260;
  c.goal.edge_margin_px=50;c.goal.target_u_px=360;c.goal.approach_u_deadband_px=160;
  c.goal.throw_landing_z_m=.4;c.backboard_depth_strip_width_px=5;
  const cv::Mat image=cv::Mat::zeros(480,640,CV_8UC3);
  const std::vector<AdapterDetection> adapter{{{310,190,20,20},1,4}};
  const std::vector<vision_core::Detection> core{{{310,190,20,20},1,4}};
  vision_core::PreparedPerceptionFrame p;
  vision_core::ObjectTarget hurdle;
  hurdle.center_px={300,200};hurdle.box_px={290,190,20,20};
  hurdle.width_px=hurdle.height_px=20;hurdle.confidence=1;
  p.targets.hurdle=hurdle;
  vision_core::MissionFrameResult m;
  m.active_mission=vision_core::MissionType::kHurdle;
  m.hurdle.tracked.visible=true;m.hurdle.tracked.center_px={300,200};
  m.foot_pose=vision_core::FootPose::kFine;
  const vision_core::CameraFeedback camera{vision_core::CameraMode::kDown,true};
  const vision::MissionDebugContext state;
  Check(vision::DebugRepeatLabel(300+static_cast<int>(vision_core::GoalMode::kEdgeTurn),c)=="EDGE TURN x2",
        "edge pair preview must show its repeat scope");
  auto a=vision::RenderMissionDebug(image,adapter,{},c,p,m,camera,{},state);
  auto b=vision::RenderMissionDebug(image,core,{},c,p,m,camera,{},state);
  Check(cv::norm(a.image,b.image,cv::NORM_INF)==0 && cv::norm(a.panel,b.panel,cv::NORM_INF)==0,
        "adapter/core detection representations render differently");
  Check(a.image.at<cv::Vec3b>(260,600)!=cv::Vec3b(0,0,0),"FINE CONTACT line absent");
  Check(a.image.at<cv::Vec3b>(200,300)!=cv::Vec3b(0,0,0),"retained decision marker absent");
  Check(cv::norm(image,cv::NORM_INF)==0,"renderer changed the source RGB frame");
  m.foot_pose=vision_core::FootPose::kWalk;
  auto walk=vision::RenderMissionDebug(image,adapter,{},c,p,m,camera,{},state);
  Check(walk.image.at<cv::Vec3b>(260,600)==cv::Vec3b(0,0,0),"CONTACT drawn in WALK");
  m.active_mission=vision_core::MissionType::kGoal;m.goal.mode=vision_core::GoalMode::kApproach;
  const std::vector<vision_core::Detection> empty;
  c.goal.edge_margin_px=40;
  auto edge=vision::RenderMissionDebug(image,empty,{},c,{},m,camera,{},state);
  Check(edge.image.at<cv::Vec3b>(400,40)!=cv::Vec3b(0,0,0),"configured edge guide absent");
  c.goal.edge_margin_px=90;
  edge=vision::RenderMissionDebug(image,empty,{},c,{},m,camera,{},state);
  Check(edge.image.at<cv::Vec3b>(400,40)==cv::Vec3b(0,0,0) &&
        edge.image.at<cv::Vec3b>(400,90)!=cv::Vec3b(0,0,0),"edge guide ignored new tuning");

  c.ball.far_u_des_px=380;c.ball.approach_u_deadband_px=90;
  c.ball.approach_short_v_px=100;c.ball.approach_two_step_v_px=240;
  c.ball.approach_direct_down_v_px=355;
  m.active_mission=vision_core::MissionType::kBall;
  m.ball.mode=vision_core::BallMode::kApproachBall;
  const vision_core::CameraFeedback forward{vision_core::CameraMode::kForward,true};
  auto direct=vision::RenderMissionDebug(image,empty,{},c,{},m,forward,{},state);
  Check(direct.image.at<cv::Vec3b>(355,380)!=cv::Vec3b(0,0,0) &&
        direct.image.at<cv::Vec3b>(355,200)==cv::Vec3b(0,0,0) &&
        direct.image.at<cv::Vec3b>(355,550)==cv::Vec3b(0,0,0),
        "direct DOWN boundary must split only the center column");
  c.ball.approach_direct_down_v_px=390;
  m.ball.mode=vision_core::BallMode::kWaitCameraDown;
  direct=vision::RenderMissionDebug(image,empty,{},c,{},m,forward,{},state);
  Check(direct.image.at<cv::Vec3b>(355,380)==cv::Vec3b(0,0,0) &&
        direct.image.at<cv::Vec3b>(390,380)!=cv::Vec3b(0,0,0),
        "direct DOWN guide must update while waiting for the camera");
  direct=vision::RenderMissionDebug(image,empty,{},c,{},m,camera,{},state);
  Check(direct.image.at<cv::Vec3b>(390,380)==cv::Vec3b(0,0,0),
        "FORWARD direct DOWN boundary leaked into the DOWN view");
  m.active_mission=vision_core::MissionType::kGoal;

  // Fixed calibration exercises the displayed geometry without pinning YAML.
  c.goal.throw_landing_x_m=-.08;c.goal.throw_landing_z_m=.48;
  c.goal.hoop_radius_m=.1;
  m.goal.mode=vision_core::GoalMode::kFineAdjust;
  m.goal.fine_decision_pose_used=true;
  m.goal.fine_decision_pose={true,-.08,.58,0,1};
  auto landing=vision::ComputeGoalLandingDebug(c.goal,m);
  Check(landing.valid && landing.rim_valid && !landing.commanded &&
        std::abs(landing.x_m+.08)<1e-12 && std::abs(landing.z_m-.48)<1e-12 &&
        landing.position_error_m<1e-12,"zero-yaw landing/rim coordinates mismatch");

  m.goal.mode=vision_core::GoalMode::kShoot;
  m.command.mission=vision_core::MissionType::kGoal;
  m.command.action=vision_core::MissionAction::kShoot;
  m.command.target_yaw_deg=-10;
  m.goal.shoot_yaw_rad=1; // Live tracking yaw must not override the command.
  landing=vision::ComputeGoalLandingDebug(c.goal,m);
  const double angle=-10*vision::kDebugPi/180;
  Check(landing.commanded && landing.yaw_deg==-10 &&
        std::abs(landing.x_m-(-.08*std::cos(angle)-.48*std::sin(angle)))<1e-12 &&
        std::abs(landing.z_m-(-.08*std::sin(angle)+.48*std::cos(angle)))<1e-12,
        "landing must use the actual signed integer SHOOT angle");
  const auto shot=vision::RenderMissionDebug(image,empty,{},c,{},m,camera,{},state);
  m.goal.fine_decision_pose_used=false;
  landing=vision::ComputeGoalLandingDebug(c.goal,m);
  Check(landing.valid && landing.commanded && !landing.rim_valid && landing.yaw_deg==-10,
        "SHOOT landing disappeared when the decision pose expired");
  m.command={};m.goal.action={};
  Check(!vision::ComputeGoalLandingDebug(c.goal,m).valid,
        "missing decision/command produced a fabricated landing");
  const auto missing=vision::RenderMissionDebug(image,empty,{},c,{},m,camera,{},state);
  Check(cv::norm(shot.image(cv::Rect(0,54,640,100)),
                 missing.image(cv::Rect(0,54,640,100)),cv::NORM_INF)>0,
        "camera overlay did not reflect shot coordinates");
  m.goal.fine_decision_pose_used=true;
  m.goal.fine_decision_pose.x_m=std::numeric_limits<double>::quiet_NaN();
  Check(!vision::ComputeGoalLandingDebug(c.goal,m).valid,
        "invalid depth pose produced landing coordinates");
  m.active_mission=vision_core::MissionType::kBall;
  m.command.action=vision_core::MissionAction::kShoot;
  Check(!vision::ComputeGoalLandingDebug(c.goal,m).valid,
        "previous GOAL landing leaked into a different mission");
  Check(cv::norm(image,cv::NORM_INF)==0,"GOAL overlay changed the source RGB frame");
}
