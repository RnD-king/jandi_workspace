#pragma once

#include "vision/mission_state_log.hpp"
#include "vision/msg/terminal_frame.hpp"
#include <rclcpp/rclcpp.hpp>
#include <chrono>

namespace vision {

template<typename DetectionRange>
msg::TerminalFrame MakeTerminalFrame(const std_msgs::msg::Header& header,
    int width,int height,const DetectionRange& detections,
    const vision_core::MissionControllerConfig& config,
    const vision_core::PerceptionMissionFrameResult& result,
    const vision_core::CameraFeedback& camera,const MissionDebugContext& context) {
  msg::TerminalFrame frame;
  frame.header=header; frame.image_width=width; frame.image_height=height;
  frame.control_state=context.control_state;
  frame.mission=DebugMissionName(result.mission.active_mission);
  frame.phase=StateMissionPhase(result.mission);
  if (result.mission.approach_mission!=vision_core::MissionType::kNone)
    frame.phase+=" "+std::to_string(result.mission.approach_completed_blocks)+"/"+
        std::to_string(config.object_approach.lock_blocks)+(result.mission.approach_locked ? " LOCK" : " OPEN");
  frame.action_name=DebugActionName(result.mission.command.action);
  if (result.mission.goal.depth_recovery_active)
    frame.phase+=" "+std::to_string(result.mission.goal.depth_recovery_steps)+"/"+
        std::to_string(config.goal.depth_recovery_max_steps);
  if (result.mission.continuous_approach_enabled &&
      result.mission.active_mission==vision_core::MissionType::kLine && result.mission.goal.entry_line_retries)
    frame.phase+=" retry "+std::to_string(result.mission.goal.entry_line_retries)+"/"+
        std::to_string(config.object_approach.goal_empty_max_retries);
  frame.action=static_cast<std::uint16_t>(result.mission.command.action);
  frame.waiting_action_id=context.waiting_action_id;
  frame.camera=StateCameraName(camera.actual_mode);
  frame.camera_settled=camera.settled;
  frame.filters_current=context.live_object_detections && camera.settled &&
      camera.actual_mode!=vision_core::CameraMode::kTransition;
  std::size_t index=0;
  for (const auto& detection:detections) {
    const auto current=index++;
    int kind=-1;
    if (detection.class_id==config.line_detection.class_id) kind=msg::TerminalDetection::LINE;
    else if (detection.class_id==config.object_targets.ball_class_id) kind=msg::TerminalDetection::BALL;
    else if (detection.class_id==config.object_targets.goal_class_id) kind=msg::TerminalDetection::GOAL;
    else if (detection.class_id==config.object_targets.backboard_class_id) kind=msg::TerminalDetection::BACKBOARD;
    else if (detection.class_id==config.object_targets.hurdle_class_id) kind=msg::TerminalDetection::HURDLE;
    if (kind<0 || !std::isfinite(detection.confidence) || detection.confidence<0 ||
        detection.confidence>1 || !std::isfinite(static_cast<double>(detection.box.x)) ||
        !std::isfinite(static_cast<double>(detection.box.y)) ||
        !std::isfinite(static_cast<double>(detection.box.width)) ||
        !std::isfinite(static_cast<double>(detection.box.height)) ||
        detection.box.width<=0 || detection.box.height<=0) continue;
    msg::TerminalDetection box;
    box.kind=kind; box.x=detection.box.x; box.y=detection.box.y;
    box.width=detection.box.width; box.height=detection.box.height;
    box.confidence=detection.confidence;
    if (kind==msg::TerminalDetection::LINE) {
      box.filtered=detection.confidence>=config.line_detection.confidence &&
          detection.box.width>config.line_detection.min_box_width &&
          detection.box.height>config.line_detection.min_box_height;
      if (!box.filtered) continue;
    } else if (frame.filters_current) {
      box.filtered=std::find(result.perception.accepted_object_indices.begin(),
          result.perception.accepted_object_indices.end(),current)!=
          result.perception.accepted_object_indices.end();
    }
    frame.detections.push_back(box);
  }
  return frame;
}

class TerminalPublisher {
public:
  explicit TerminalPublisher(rclcpp::Node& node) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only=true;
    descriptor.description="Publish lightweight detections/state for the SSH terminal viewer";
    const bool enabled=node.declare_parameter<bool>("terminal_node",false,descriptor);
    descriptor.description="Maximum terminal metadata publication rate";
    const double hz=node.declare_parameter<double>("terminal_fps",5.0,descriptor);
    descriptor.description="Local metadata topic; no images are published here";
    const auto topic=node.declare_parameter<std::string>("terminal_topic","/jandi_vision/terminal",descriptor);
    if (!std::isfinite(hz) || hz<=0 || hz>60)
      throw std::invalid_argument("terminal_fps must be in (0,60]");
    period_=std::chrono::duration<double>(1.0/hz);
    if (enabled) publisher_=node.create_publisher<msg::TerminalFrame>(topic,
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile());
  }
  template<typename DetectionRange>
  void Publish(const std_msgs::msg::Header& header,int width,int height,
      const DetectionRange& detections,const vision_core::MissionControllerConfig& config,
      const vision_core::PerceptionMissionFrameResult& result,
      const vision_core::CameraFeedback& camera,const MissionDebugContext& context) {
    if (!publisher_ || (publisher_->get_subscription_count()==0 &&
        publisher_->get_intra_process_subscription_count()==0)) return;
    const auto now=std::chrono::steady_clock::now();
    if (last_ && now-*last_<period_) return;
    last_=now;
    publisher_->publish(MakeTerminalFrame(header,width,height,detections,config,result,camera,context));
  }
private:
  rclcpp::Publisher<msg::TerminalFrame>::SharedPtr publisher_;
  std::chrono::duration<double> period_{};
  std::optional<std::chrono::steady_clock::time_point> last_;
};
} // namespace vision
