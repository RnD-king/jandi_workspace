#include "vision_core/control_command.hpp"

#include "vision_core/config_loader.hpp"

#include <algorithm>

namespace vision_core {
namespace {
bool IsLongLineAction(MissionAction action) {
  return action == MissionAction::kStepForwardFive ||
         action == MissionAction::kStepForwardLeft ||
         action == MissionAction::kStepForwardRight;
}
} // namespace

ControlCommandCoordinator::ControlCommandCoordinator()
    : ControlCommandCoordinator(LoadDefaultAlgorithmConfig().command) {}

ControlCommandCoordinator::ControlCommandCoordinator(
    const ControlCommandConfig &config)
    : config_(config), next_action_id_(config.first_action_id) {
  if (next_action_id_ == 0) next_action_id_ = 1;
}

ControlCommand ControlCommandCoordinator::BeginAction(
    ControlCommand command, const ActionRequest &request) {
  pending_action_id_ = next_action_id_++;
  if (next_action_id_ == 0) next_action_id_ = 1;
  pending_mission_ = command.mission;
  pending_request_ = request;
  pending_issued_sec_ = now_sec_;
  pending_acknowledged_ = false;
  pending_ready_ = false;
  command.command_type = CommandType::kAction;
  command.control_phase = ControlPhase::kWaitingActionAck;
  command.action_id = pending_action_id_;
  command.action = request.action;
  command.action_category = request.category;
  command.target_yaw_deg = request.target_yaw_deg;
  return command;
}

ControlCommand ControlCommandCoordinator::BeginQueuedLineAction(
    ControlCommand command, const ActionRequest &request) {
  queued_action_id_ = next_action_id_++;
  if (next_action_id_ == 0) next_action_id_ = 1;
  queued_request_ = request;
  queued_issued_sec_ = now_sec_;
  queued_acknowledged_ = false;
  command.command_type = CommandType::kAction;
  command.control_phase = ControlPhase::kWaitingQueuedActionAck;
  command.action_id = queued_action_id_;
  command.action = request.action;
  command.action_category = request.category;
  command.target_yaw_deg = request.target_yaw_deg;
  return command;
}

ControlCommand ControlCommandCoordinator::PendingCommand(
    ControlCommand command) const {
  if (queued_action_id_ != 0) {
    command.mission = MissionType::kLine;
    command.action_id = queued_action_id_;
    command.action = queued_request_.action;
    command.action_category = queued_request_.category;
    command.target_yaw_deg = queued_request_.target_yaw_deg;
    command.command_type = queued_acknowledged_ ? CommandType::kHold
                                                : CommandType::kAction;
    command.control_phase = queued_acknowledged_
        ? ControlPhase::kWaitingQueuedActionStart
        : ControlPhase::kWaitingQueuedActionAck;
    return command;
  }
  command.mission = pending_mission_;
  command.action_id = pending_action_id_;
  command.action = pending_request_.action;
  command.action_category = pending_request_.category;
  command.target_yaw_deg = pending_request_.target_yaw_deg;
  command.command_type = pending_acknowledged_ ? CommandType::kHold
                                               : CommandType::kAction;
  command.control_phase = pending_acknowledged_
      ? ControlPhase::kWaitingActionDone
      : ControlPhase::kWaitingActionAck;
  return command;
}

ControlCommand ControlCommandCoordinator::Compute(
    MissionType mission, int mission_phase, const ActionRequest &request,
    CameraRequest camera_request, const CommandDeliveryFeedback &feedback,
    double now_sec) {
  now_sec_ = now_sec;
  if (!action_ack_timed_out_ && queued_action_id_ != 0 &&
      feedback.action_id == queued_action_id_) {
    queued_acknowledged_ = queued_acknowledged_ || feedback.acknowledged;
  }
  if (!action_ack_timed_out_ && pending_action_id_ != 0 &&
      feedback.action_id == pending_action_id_) {
    pending_acknowledged_ = pending_acknowledged_ || feedback.acknowledged;
    pending_ready_ = pending_ready_ || feedback.ready;
    if (feedback.done) {
      if (pending_request_.category == ActionCategory::kMission)
        suppressed_mission_action_ = pending_request_.action;
      if (queued_action_id_ != 0) {
        pending_action_id_ = queued_action_id_;
        pending_mission_ = MissionType::kLine;
        pending_request_ = queued_request_;
        pending_issued_sec_ = queued_issued_sec_;
        pending_acknowledged_ = queued_acknowledged_;
        pending_ready_ = false;
        queued_action_id_ = 0;
        queued_request_ = {};
        queued_issued_sec_ = 0.0;
        queued_acknowledged_ = false;
      } else {
        pending_action_id_ = 0;
        pending_request_ = {};
        pending_acknowledged_ = false;
        pending_ready_ = false;
      }
    }
  }

  const double timeout = std::max(0.0, config_.action_ack_timeout_sec);
  if (!action_ack_timed_out_ && timeout > 0.0) {
    action_ack_timed_out_ =
        (pending_action_id_ != 0 && !pending_acknowledged_ &&
         now_sec_ - pending_issued_sec_ >= timeout) ||
        (queued_action_id_ != 0 && !queued_acknowledged_ &&
         now_sec_ - queued_issued_sec_ >= timeout);
  }

  ControlCommand command;
  command.mission = mission;
  command.mission_phase = mission_phase;
  command.camera_request = camera_request;
  if (action_ack_timed_out_) {
    command = PendingCommand(command);
    command.command_type = CommandType::kHold;
    command.control_phase = ControlPhase::kActionAckTimedOut;
    return command;
  }

  if (pending_action_id_ != 0) {
    if (queued_action_id_ == 0 && pending_ready_ &&
        pending_mission_ == MissionType::kLine &&
        pending_request_.category == ActionCategory::kLocomotion &&
        IsLongLineAction(pending_request_.action) &&
        mission == MissionType::kLine && request.line_queue_eligible &&
        request.action != MissionAction::kNone) {
      return BeginQueuedLineAction(command, request);
    }
    return PendingCommand(command);
  }

  if (request.action == MissionAction::kNone) {
    suppressed_mission_action_ = MissionAction::kNone;
    return command;
  }
  if (request.category == ActionCategory::kMission &&
      request.action == suppressed_mission_action_) return command;
  return BeginAction(command, request);
}

void ControlCommandCoordinator::Reset() {
  next_action_id_ = config_.first_action_id == 0 ? 1 : config_.first_action_id;
  pending_action_id_ = 0;
  pending_mission_ = MissionType::kLine;
  pending_request_ = {};
  pending_issued_sec_ = 0.0;
  pending_acknowledged_ = false;
  pending_ready_ = false;
  queued_action_id_ = 0;
  queued_request_ = {};
  queued_issued_sec_ = 0.0;
  queued_acknowledged_ = false;
  action_ack_timed_out_ = false;
  suppressed_mission_action_ = MissionAction::kNone;
}

} // namespace vision_core
