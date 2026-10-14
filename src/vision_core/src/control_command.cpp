#include "vision_core/control_command.hpp"

#include "vision_core/config_loader.hpp"

#include <algorithm>
#include <cmath>

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

ControlCommand ControlCommandCoordinator::BeginQueuedAction(
    ControlCommand command, const ActionRequest &request) {
  queued_action_id_ = next_action_id_++;
  if (next_action_id_ == 0) next_action_id_ = 1;
  queued_request_ = request;
  queued_mission_ = command.mission;
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
    command.mission = queued_mission_;
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

void ControlCommandCoordinator::ObserveExecutionTime(
    const CommandDeliveryFeedback &feedback, double now_sec) {
  if (!std::isfinite(now_sec)) return;
  if (execution_pending_id_ != pending_action_id_) {
    execution_pending_id_ = pending_action_id_;
    execution_observed_sec_ = now_sec;
  }
  if (pending_action_id_ != 0 && pending_action_id_ != execution_finished_id_ &&
      pending_acknowledged_ && pending_mission_ == MissionType::kLine &&
      IsLongLineAction(pending_request_.action))
    line_execution_sec_ += std::max(0.0, now_sec - execution_observed_sec_);
  execution_observed_sec_ = now_sec;
  if (feedback.done && feedback.action_id == pending_action_id_)
    execution_finished_id_ = pending_action_id_;
}

ControlCommand ControlCommandCoordinator::Compute(
    MissionType mission, int mission_phase, const ActionRequest &request,
    CameraRequest camera_request, const CommandDeliveryFeedback &feedback,
    double now_sec) {
  now_sec_ = now_sec;
  ObserveExecutionTime(feedback, now_sec);
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
        pending_mission_ = queued_mission_;
        pending_request_ = queued_request_;
        pending_issued_sec_ = queued_issued_sec_;
        pending_acknowledged_ = queued_acknowledged_;
        pending_ready_ = false;
        queued_action_id_ = 0;
        queued_request_ = {};
        queued_mission_ = MissionType::kLine;
        queued_issued_sec_ = 0.0;
        queued_acknowledged_ = false;
        ObserveExecutionTime({}, now_sec); // Queued execution begins at current DONE.
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
    // READY admission is governed by the currently executing action A.
    // The successor B may be any valid ActionCommand, including a different
    // action number or mission; the executor performs full preparation before ACK.
    if (queued_action_id_ == 0 && pending_ready_ &&
        pending_request_.line_queue_eligible &&
        request.action != MissionAction::kNone &&
        request.category != ActionCategory::kNone &&
        camera_request == CameraRequest::kNone) {
      return BeginQueuedAction(command, request);
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

void ControlCommandCoordinator::Reset() { Reset(false); }

void ControlCommandCoordinator::Reset(bool preserve_action_ids) {
  if (!preserve_action_ids)
    next_action_id_ = config_.first_action_id == 0 ? 1 : config_.first_action_id;
  pending_action_id_ = 0;
  pending_mission_ = MissionType::kLine;
  pending_request_ = {};
  pending_issued_sec_ = 0.0;
  pending_acknowledged_ = false;
  pending_ready_ = false;
  queued_action_id_ = 0;
  queued_request_ = {};
  queued_mission_ = MissionType::kLine;
  queued_issued_sec_ = 0.0;
  queued_acknowledged_ = false;
  action_ack_timed_out_ = false;
  suppressed_mission_action_ = MissionAction::kNone;
  line_execution_sec_ = execution_observed_sec_ = 0.0;
  execution_pending_id_ = execution_finished_id_ = 0;
}

} // namespace vision_core
