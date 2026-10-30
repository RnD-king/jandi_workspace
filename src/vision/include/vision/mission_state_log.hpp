#pragma once

#include "vision/mission_debug_view.hpp"
#include <optional>
#include <sstream>

namespace vision {
inline const char* StateCameraName(vision_core::CameraMode mode) {
  using vision_core::CameraMode;
  switch(mode) {
    case CameraMode::kForward:return "FORWARD";
    case CameraMode::kDown:return "DOWN";
    case CameraMode::kGoal:return "GOAL";
    default:return "TRANSITION";
  }
}
inline const char* StateMissionPhase(const vision_core::MissionFrameResult& m) {
  using vision_core::MissionType;
  if(m.approach_goal_search_failed) return "GOAL_SEARCH_FAILED";
  if(m.approach_goal_depth_failed) return "GOAL_DEPTH_FAILED";
  if(m.goal.depth_recovery_active) return "GOAL_DEPTH_RECOVERY";
  if(m.active_mission==MissionType::kBall) return vision_core::BallController::ModeName(m.ball.mode);
  if(m.active_mission==MissionType::kHurdle) return vision_core::HurdleController::ModeName(m.hurdle.mode);
  if(m.active_mission==MissionType::kGoal) return vision_core::GoalController::ModeName(m.goal.mode);
  if(m.terminal_hold) return "LINE_FINAL_HOLD";
  if(m.approach_mission!=MissionType::kNone) {
    if(m.approach_near) return "APPROACH_NEAR";
    if(m.approach_mission==MissionType::kBall) return "APP_BALL";
    if(m.approach_mission==MissionType::kGoal) return "APP_GOAL";
    return "APP_HURDLE";
  }
  if(m.candidate_phase) return "LINE_OBJECT_CANDIDATE";
  return m.line_in_recovery ? "LINE_RECOVERY" : "LINE_FOLLOW";
}
inline const char* StateCandidatePhase(int phase) {
  switch(phase) {
    case 1:return "DRAIN";case 2:return "OBSERVE";case 3:return "SEARCH";case 4:return "RESTORE";
    default:return "NONE";
  }
}

// Read-only diagnostics. A changed phase/identity/wait is emitted immediately;
// changing frame counters alone cannot turn the log into a per-frame stream.
class MissionStateLog {
public:
  std::optional<std::string> Update(double now,const std::string& run,
      const vision_core::PerceptionMissionFrameResult& result,
      const vision_core::CameraFeedback& camera,
      const vision_core::MissionControllerConfig& config,
      std::uint64_t waiting_action,std::uint64_t waiting_camera) {
    using namespace vision_core;
    const auto& m=result.mission; const auto& p=result.perception;
    const char* wait="IDLE";
    switch(m.command.control_phase) {
      case ControlPhase::kWaitingActionAck:wait="ACK";break;
      case ControlPhase::kWaitingActionDone:wait=waiting_action ? "DONE" : "FRESH_FRAME";break;
      case ControlPhase::kWaitingQueuedActionAck:wait="QUEUED_ACK";break;
      case ControlPhase::kWaitingQueuedActionStart:wait="QUEUED_START";break;
      case ControlPhase::kActionAckTimedOut:wait="ACK_TIMEOUT";break;
      default:if(waiting_action)wait="DONE";break;
    }
    if(waiting_camera || !camera.settled) wait="CAMERA";
    if(m.post_motion_observing) wait="OBSERVE";
    if(m.terminal_hold) wait="TERMINAL_HOLD";
    std::ostringstream key;
    key<<run<<' '<<int(m.active_mission)<<' '<<StateMissionPhase(m)<<' '
       <<int(m.candidate_mission)<<':'<<m.candidate_phase<<' '
       <<int(camera.actual_mode)<<':'<<camera.settled<<' '<<wait<<' '
       <<waiting_action<<' '<<waiting_camera<<' '<<m.command.action_id<<' '
       <<p.ball_target_id<<' '<<p.hurdle_target_id<<' '<<p.backboard_target_id<<' '
       <<p.ball_target_locked<<p.hurdle_target_locked<<p.backboard_target_locked<<' '
       <<m.ball.down_search_found<<m.hurdle.down_search_found<<' '
       <<int(m.approach_mission)<<':'<<m.approach_track_id<<':'<<m.approach_completed_blocks<<':'<<m.approach_locked;
    const bool changed=key.str()!=last_key_;
    if(!changed && now>=last_sec_ && now-last_sec_<1.0) return std::nullopt;
    last_key_=key.str(); last_sec_=now;
    MissionType object=m.active_mission;
    if(object==MissionType::kLine) object=m.approach_mission!=MissionType::kNone ?
        m.approach_mission : m.candidate_mission;
    int hits=0,frames=0,lost=0,window=0; bool found=false;
    if(object==MissionType::kBall) {
      hits=m.ball.detection_hits;frames=m.ball.detection_frames;lost=m.ball.lost_frames;
      window=config.ball.stable_window;found=m.ball.down_search_found;
    } else if(object==MissionType::kHurdle) {
      hits=m.hurdle.detection_hits;frames=m.hurdle.detection_frames;lost=m.hurdle.lost_frames;
      window=config.hurdle.stable_window;found=m.hurdle.down_search_found;
    } else if(object==MissionType::kGoal) {
      hits=m.goal.detection_hits;frames=m.goal.detection_frames;lost=m.goal.lost_frames;
      window=config.goal.stable_window;
    }
    if(m.active_mission==MissionType::kLine && m.approach_mission==MissionType::kNone && object==MissionType::kBall) {
      hits=m.ball.entry_hits;frames=m.ball.entry_frames;
    }
    std::ostringstream text;
    text<<(changed ? "CHANGE " : "STATUS ")<<"run="<<run
        <<" mission="<<DebugMissionName(m.active_mission)<<" phase="<<StateMissionPhase(m)
        <<" candidate="<<DebugMissionName(m.candidate_mission)<<':'<<StateCandidatePhase(m.candidate_phase)
        <<" camera="<<StateCameraName(camera.actual_mode)<<" settled="<<camera.settled
        <<" wait="<<wait<<" action_id="<<(waiting_action ? waiting_action : m.command.action_id)
        <<" action="<<int(m.command.action)<<" cmd_mission="<<DebugMissionName(m.command.mission)
        <<" camera_id="<<waiting_camera
        <<" ball_id="<<p.ball_target_id<<" hurdle_id="<<p.hurdle_target_id<<" board_id="<<p.backboard_target_id
        <<" locks="<<p.ball_target_locked<<','<<p.hurdle_target_locked<<','<<p.backboard_target_locked
        <<" object="<<DebugMissionName(object)<<" hits="<<hits<<'/'<<window<<" frames="<<frames
        <<" lost="<<lost<<" found="<<found<<" filtered="<<p.accepted_object_detections.size()
        <<" observe="<<m.post_motion_valid<<'/'<<m.post_motion_window_frames;
    text<<" goal_empty_retries="<<m.goal.entry_line_retries<<'/'<<config.object_approach.goal_empty_max_retries
        <<" approach="<<DebugMissionName(m.approach_mission)<<" id="<<m.approach_track_id
          <<" blocks="<<m.approach_completed_blocks<<'/'<<config.object_approach.lock_blocks
          <<" locked="<<m.approach_locked<<" missing="<<m.approach_missing_frames<<" near="<<m.approach_near
          <<" seq="<<m.approach_sequence_hits<<'/'<<m.approach_sequence_frames
          <<" qualified="<<m.approach_sequence_qualified;
    if(object==MissionType::kGoal) text<<" pose_lost="<<m.goal.pose_lost_frames;
    const auto candidates=[&](const char* label,const std::vector<BallCandidateTrack>& tracks) {
      for(const auto& track:tracks)
        text<<' '<<label<<'='<<track.id<<':'<<track.HitCount()<<'/'<<track.hits.size()<<":miss"<<track.missing_frames;
    };
    candidates("B",p.ball_tracks);candidates("H",p.hurdle_tracks);candidates("G",p.backboard_tracks);
    return text.str();
  }
private:
  std::string last_key_;
  double last_sec_{0};
};
} // namespace vision
