#include "vision/mission_debug_view.hpp"
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
}
