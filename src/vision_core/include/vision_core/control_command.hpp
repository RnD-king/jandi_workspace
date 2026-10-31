#pragma once

#include "vision_core/types.hpp"

namespace vision_core {

struct CommandDeliveryFeedback {
  std::uint64_t action_id{0};
  bool acknowledged{false};
  bool done{false};
  bool ready{false};
};

struct ControlCommandConfig {
  std::uint64_t first_action_id{};
  double action_ack_timeout_sec{};
};

class ControlCommandCoordinator {
public:
  ControlCommandCoordinator();
  explicit ControlCommandCoordinator(const ControlCommandConfig &config);

  ControlCommand Compute(MissionType mission, int mission_phase,
                         const ActionRequest &request,
                         CameraRequest camera_request,
                         const CommandDeliveryFeedback &feedback,
                         double now_sec);
  void Reset(); // Original symbol retained for already-built clients.
  void Reset(bool preserve_action_ids);
  // Execution clock includes arm transitions; queued ACK does not start it.
  void ObserveExecutionTime(const CommandDeliveryFeedback &feedback, double now_sec);
  double LineExecutionSeconds() const { return line_execution_sec_; }
  bool HasQueuedAction() const { return queued_action_id_ != 0; }
  bool HasPendingAction() const { return pending_action_id_ != 0 || queued_action_id_ != 0; }
  std::uint64_t PendingActionId() const { return pending_action_id_; }
  bool ActionAckTimedOut() const { return action_ack_timed_out_; }

private:
  ControlCommand BeginAction(ControlCommand command,
                             const ActionRequest &request);
  ControlCommand BeginQueuedAction(ControlCommand command,
                                       const ActionRequest &request);
  ControlCommand PendingCommand(ControlCommand command) const;

  ControlCommandConfig config_;
  std::uint64_t next_action_id_{1};
  std::uint64_t pending_action_id_{0};
  MissionType pending_mission_{MissionType::kLine};
  ActionRequest pending_request_;
  double pending_issued_sec_{0.0};
  bool pending_acknowledged_{false};
  bool pending_ready_{false};
  std::uint64_t queued_action_id_{0};
  ActionRequest queued_request_;
  MissionType queued_mission_{MissionType::kLine};
  double queued_issued_sec_{0.0};
  bool queued_acknowledged_{false};
  bool action_ack_timed_out_{false};
  MissionAction suppressed_mission_action_{MissionAction::kNone};
  double now_sec_{0.0};
  double line_execution_sec_{0.0}, execution_observed_sec_{0.0};
  std::uint64_t execution_pending_id_{0}, execution_finished_id_{0};
};

} // namespace vision_core
