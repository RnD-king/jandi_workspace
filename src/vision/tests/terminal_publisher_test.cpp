#include "vision/terminal_publisher.hpp"
#include "vision/types.hpp"
#include <stdexcept>

static void Check(bool value,const char* text) {
  if (!value) throw std::runtime_error(text);
}
int main() {
  vision_core::MissionControllerConfig config;
  // Non-default model IDs prove the wire roles are independent of class IDs.
  config.line_detection.class_id=10; config.line_detection.confidence=.5;
  config.line_detection.min_box_width=config.line_detection.min_box_height=2;
  config.object_targets.ball_class_id=11; config.object_targets.goal_class_id=12;
  config.object_targets.backboard_class_id=13; config.object_targets.hurdle_class_id=14;
  std::vector<vision::Detection> detections{
      {{0,0,10,10},.9f,10},{{30,40,50,60},.9f,11},{{10,20,20,20},.1f,10},
      {{60,70,20,20},.9f,13},{{100,100,20,20},.9f,99}};
  vision_core::PerceptionMissionFrameResult result;
  result.perception.accepted_object_indices={1};
  vision::MissionDebugContext context;
  context.control_state="RUNNING"; context.waiting_action_id=42;
  const vision_core::CameraFeedback camera{vision_core::CameraMode::kGoal,true};
  std_msgs::msg::Header header; header.frame_id="camera"; header.stamp.sec=123;
  auto frame=vision::MakeTerminalFrame(header,640,480,detections,config,result,camera,context);
  Check(frame.detections.size()==3,"unrecognized classes and rejected line centers must be omitted");
  Check(frame.header.stamp.sec==123 && frame.image_width==640 && frame.waiting_action_id==42,
      "metadata must retain the original frame and real action state");
  Check(frame.filters_current && frame.detections[0].kind==vision::msg::TerminalDetection::LINE &&
      frame.detections[1].kind==vision::msg::TerminalDetection::BALL && frame.detections[1].filtered &&
      !frame.detections[2].filtered,"filter flags must use the production accepted indices");
  // The paused mission result may still contain an older accepted board index.
  context.live_object_detections=false; context.control_state="PAUSED";
  result.perception.accepted_object_indices={3};
  detections.resize(1);
  frame=vision::MakeTerminalFrame(header,640,480,detections,config,result,camera,context);
  Check(!frame.filters_current && frame.detections.size()==1,
      "paused terminal data must use this frame's raw detections, never frozen object boxes");
}
