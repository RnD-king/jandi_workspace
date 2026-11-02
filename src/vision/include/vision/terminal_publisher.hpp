#pragma once

#include "vision/mission_state_log.hpp"
#include "vision/msg/terminal_frame.hpp"
#include <rclcpp/rclcpp.hpp>
#include <chrono>

namespace vision {
inline const char* CurveTerminalName(int phase) {
  switch(phase) {
    case 0:return "NONE";case 1:return "CONFIRM";case 2:return "DELAY";
    case 3:return "DRAIN";case 4:return "SCAN";case 5:return "HOLD";
    case 6:return "RETURN_FOUND";case 7:return "RETURN_MISS";
    case 8:return "PROBE_WALK";case 9:return "BODY_TURN";
    case 10:return "BODY_FORWARD";case 11:return "BODY_OBSERVE";
    case 12:return "LOCKED";default:return "UNKNOWN";
  }
}
inline const char* LineTerminalName(int phase) {
  switch(phase) {
    case 0:return "NORMAL";case 1:return "FAIL_OBSERVE";
    case 2:return "RECOVERY_TURN";case 3:return "RECOVERY_OBSERVE";
    case 4:return "FINAL_HOLD";case 5:return "SCORE_RECOVERY";
    default:return "UNKNOWN";
  }
}
inline const char* TerminalWaitName(vision_core::ControlPhase phase) {
  using vision_core::ControlPhase;
  switch(phase) {
    case ControlPhase::kWaitingActionAck:return "ACK";
    case ControlPhase::kWaitingActionDone:return "DONE";
    case ControlPhase::kWaitingQueuedActionAck:return "QUEUED_ACK";
    case ControlPhase::kWaitingQueuedActionStart:return "QUEUED_START";
    case ControlPhase::kActionAckTimedOut:return "ACK_TIMEOUT";
    default:return "IDLE";
  }
}

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
  if (result.mission.active_mission==vision_core::MissionType::kLine && result.mission.goal.entry_line_retries)
    frame.phase+=" retry "+std::to_string(result.mission.goal.entry_line_retries)+"/"+
        std::to_string(config.object_approach.goal_empty_max_retries);
  frame.action=static_cast<std::uint16_t>(result.mission.command.action);
  frame.waiting_action_id=context.waiting_action_id;
  frame.camera=StateCameraName(camera.actual_mode);
  frame.camera_settled=camera.settled;
  frame.filters_current=context.live_object_detections && camera.settled &&
      camera.actual_mode!=vision_core::CameraMode::kTransition;
  const auto& m=result.mission;
  const auto& perception=result.perception;
  frame.curve_info="CURVE:"+std::string(CurveTerminalName(m.curve_search_phase))+
      " IDX:"+std::to_string(m.course_curve_index)+
      " HEAD:"+std::to_string(m.curve_head_attempts)+
      " BODY:"+std::to_string(m.curve_body_attempts);
  const std::uint64_t tracked=m.approach_track_id ? m.approach_track_id :
      (m.active_mission==vision_core::MissionType::kBall ? perception.ball_target_id :
       m.active_mission==vision_core::MissionType::kHurdle ? perception.hurdle_target_id :
       m.active_mission==vision_core::MissionType::kGoal ? perception.backboard_target_id : 0);
  std::uint64_t alternative=0;
  for(const auto& candidate:perception.ball_tracks)
    if(candidate.detection_index && candidate.id!=tracked) {
      alternative=candidate.id;break;
    }
  frame.object_info="OBJ:"+std::to_string(tracked)+
      " CAND:"+std::to_string(alternative)+
      " MISS:"+std::to_string(m.approach_missing_frames);
  if(perception.targets.ball)
    frame.object_info+=" BALL_V:"+std::to_string(
        static_cast<int>(std::lround(perception.targets.ball->center_px.v)));
  const std::string wait=TerminalWaitName(m.command.control_phase);
  std::string state="NONE";
  if(m.command.control_phase==vision_core::ControlPhase::kActionAckTimedOut)
    state="ACK_TIMEOUT";
  else if(m.terminal_hold) state="FINAL_HOLD";
  else if(m.recovery_failed) state="MISSION_RECOVERY_FAILED";
  else if(m.approach_probe_active) state="PRELOCK_PROBE";
  else if(m.approach_loss_pending) state="PRELOCK_LOSS";
  else if(m.curve_search_phase==3 && m.approach_mission!=vision_core::MissionType::kNone)
    state="CURVE_PREEMPT";
  else if(m.curve_search_phase>=2 && m.curve_search_phase<=11)
    state="CURVE_"+std::string(CurveTerminalName(m.curve_search_phase));
  else if(!camera.settled) state="CAMERA_WAIT";
  else if(m.post_motion_observing) state="POST_DONE_OBSERVE";
  else if(m.line_score_recovery) state="LINE_SCORE_RECOVERY";
  else if(m.line_in_recovery) state="LINE_"+std::string(LineTerminalName(m.line_recovery_phase));
  else if(m.approach_missing_frames>0 && m.approach_mission!=vision_core::MissionType::kNone)
    state="OBJECT_MISSING";
  frame.control_info="WAIT:"+wait+" STATE:"+state;
  const auto& w=m.walking;
  frame.walk_info="WALK DONE:"+std::to_string(w.done)+
      " ISSUED:"+std::to_string(w.issued)+
      " LAST:#"+std::to_string(w.last)+
      " LINE:"+std::to_string(w.line)+" BALL:"+std::to_string(w.ball)+
      " GOAL:"+std::to_string(w.goal)+" HURDLE:"+std::to_string(w.hurdle);
  frame.fine_info="FINE_DONE:"+std::to_string(w.fine)+
      " FINE_ISSUED:"+std::to_string(w.fine_issued)+
      "(F"+std::to_string(w.fine_forward)+
      "/S"+std::to_string(w.fine_side)+
      "/T"+std::to_string(w.fine_turn)+") PROBE:"+
      std::to_string(m.approach_probe_used ? 1 : 0)+"/"+
      std::to_string(config.object_approach.prelock_loss_extra_blocks)+
      " PROBE_DONE:"+std::to_string(w.probe)+
      " CURVE_LOCK_REMAIN:"+std::to_string(m.curve_unlock_blocks)+
      " BALL_REARM:N/A";
  if(m.active_mission==vision_core::MissionType::kBall && m.ball.recovery_step_limit>0)
    frame.fine_info+=" BALL_REC:"+std::to_string(m.ball.recovery_steps_completed)+
        "/"+std::to_string(m.ball.recovery_step_limit);
  if(m.active_mission==vision_core::MissionType::kHurdle && m.hurdle.recovery_step_limit>0)
    frame.fine_info+=" HURDLE_REC:"+std::to_string(m.hurdle.recovery_steps_completed)+
        "/"+std::to_string(m.hurdle.recovery_step_limit);
  if(m.goal.depth_recovery_active)
    frame.fine_info+=" GOAL_DEPTH:"+std::to_string(m.goal.depth_recovery_steps)+
        "/"+std::to_string(config.goal.depth_recovery_max_steps);
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
