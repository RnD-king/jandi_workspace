#pragma once

#include <deque>
#include <optional>

#include "vision_core/types.hpp"
#include "vision_core/down_search_walk.hpp"
#include "vision_core/object_observation_window.hpp"
#include "vision_core/stationary_object_recovery.hpp"

namespace vision_core {

enum class HurdleMode {
  kLineFollow = 0,
  kApproach,
  kWaitCameraDown,
  kContactWalk,
  kCross,
  kReturnCameraToLine,
  kRecoveryForward,
  kFailed = 8,
  kDownSearch,
  kFineAdjust,
  kRecoveryDown,
  kApproachTwoBeforeDown,
  kCoarseAdjust,
  kWaitFinePose,
  kRecoveryStep,
  kRecoveryStepObserve,
};

struct HurdleConfig {
  double approach_short_v_px{};
  int down_search_max_steps{10};
  double down_search_inter_action_hold_sec{}; // Real DONE to next DOWN search step.
  double recovery_observation_sec{};
  int recovery_max_observations{};
  int entry_recovery_max_steps{}; // First FORWARD observation loss; exhausted -> LINE.
  int recovery_max_steps{}; // DOWN loss: WALK step or FINE step, then stationary fallback.
  double target_u_px{};
  double contact_v_px{};
  int contact_steps{};
  double contact_inter_action_hold_sec{};

  int stable_window{};
  int stable_min_hits{};
  int down_search_min_hits{}; // DOWN search only, within stable_window.
  int lost_frames{};
  double upper_acquire_v_px{};
  double approach_two_step_v_px{};
  double down_one_step_v_px{};
  double fine_mode_v_px{};
  double hurdle_ignore_duration_sec{};
  double recovery_center_tolerance_px{};
};

struct TrackedHurdle {
  bool stable{false};
  bool visible{false};
  Point2 center_px;
  double u_norm{0.0};
  double v_norm{0.0};
  double h_norm{0.0};
  double bottom_norm{0.0};
  double confidence{0.0};
};

struct HurdleResult {
  int recovery_observations_completed{0};
  double recovery_observation_elapsed_sec{0.0};
  int recovery_steps_completed{0}, recovery_step_limit{0};
  bool entry_recovery{false};
  int down_search_steps_remaining{0};
  bool down_search_found{false};
  int detection_hits{0}, detection_frames{0}, lost_frames{0};
  bool active{false};
  CameraRequest camera_request{CameraRequest::kNone};
  HurdleMode mode{HurdleMode::kLineFollow};
  TrackedHurdle tracked;
  ActionRequest action;
};

class HurdleController {
public:
  HurdleController();
  HurdleController(const HurdleConfig &config, double camera_motion_timeout_sec);
  HurdleResult Compute(const std::optional<ObjectTarget> &target,
                       int image_width, int image_height, double now_sec,
                       const CameraFeedback &camera_feedback,
                       const ActionExecutionFeedback &action_feedback,
                       const CruiseDecision &line_decision,
                       bool allow_new_body_actions = true,
                       bool allow_mission_entry = true);
  static const char *ModeName(HurdleMode mode);
  bool EntryReady() const;
  void EnterVerified(const std::optional<ObjectTarget>& mean,
                     const std::deque<bool>& hits, int width, int height, double now);
  // Discard all observations made before the most recent motion DONE.
  // The mission phase and last known recovery direction are retained.
  void ClearObservationHistory();
  // A changed bbox filter invalidates the latched search detection as well.
  void InvalidateTargetIdentity() {
    ClearObservationHistory();
    stationary_recovery_.DiscardDetections();
  }
  void InvalidateDetectionEvidence() {
    InvalidateTargetIdentity();
    down_search_.InvalidateDetection();
  }
  void ObserveOnly(const std::optional<ObjectTarget> &target, int width, int height);
  void UseVerifiedObservation(const std::optional<ObjectTarget>& mean,
      const std::deque<bool>& hits, int width, int height);
  void UseNextObservationAsMean() { has_smoothed_ = false; observation_window_.Clear(); }
  void UpdateTuning(const HurdleConfig &config) {
    if (contact_next_step_sec_>0.0)
      contact_next_step_sec_+=config.contact_inter_action_hold_sec-config_.contact_inter_action_hold_sec;
    if (mode_==HurdleMode::kDownSearch && config_.down_search_min_hits!=config.down_search_min_hits)
      down_search_.InvalidateDetection();
    config_ = config;
  }
  void ShiftTimers(double duration) {
    state_enter_sec_+=duration;
    down_search_.ShiftTimers(duration);
    if (contact_next_step_sec_>0.0) contact_next_step_sec_+=duration;
  }
  // Isolated test startup selects an existing FSM phase; no completion is fabricated.
  void StartTestNear() { Reset(); mode_=HurdleMode::kWaitCameraDown; test_camera_pending_=true; }
  void StartTestExecute() { Reset(); fine_pose_entered_=true; mode_=HurdleMode::kFineAdjust; }
  void Reset();
  bool ExecutionObservationReady() const {
    return tracked_.visible && tracked_.stable &&
        hit_history_.size()>=static_cast<std::size_t>(config_.stable_window);
  }
  int RecoveryObservationsCompleted() const { return stationary_recovery_.Completed(); }
  int RecoveryStepsCompleted() const { return recovery_steps_completed_; }
  int RecoveryStepLimit() const { return recovery_step_limit_; }
  bool EntryRecovery() const { return entry_step_recovery_; }
  bool DownSearchFound() const { return down_search_.Found(); }
  int LostFrames() const { return lost_count_; }
  int DetectionFrames() const { return static_cast<int>(hit_history_.size()); }
  int DetectionHits() const { return static_cast<int>(std::count(hit_history_.begin(),hit_history_.end(),true)); }
  double RecoveryObservationElapsed(double now) const { return stationary_recovery_.Elapsed(now); }
  int DownSearchStepsRemaining() const {
    return down_search_.Remaining(config_.down_search_max_steps);
  }

private:
  static double Clamp(double value, double low, double high);
  void UpdateTracker(const std::optional<ObjectTarget> &target,
                     int image_width, int image_height);
  ActionRequest RecoveryAction() const;
  void BeginStepRecovery(double now_sec, bool entry_loss);
  void ExhaustStepRecovery(double now_sec, HurdleResult &result);
  void ResetToLine(bool clear_ignore);
  void ClearTracking();

  HurdleConfig config_;
  double camera_motion_timeout_sec_{};
  HurdleMode mode_{HurdleMode::kLineFollow};
  std::deque<bool> hit_history_;
  std::deque<bool> acquire_history_;
  bool awaiting_approach_observation_{false};
  bool entry_observation_pending_{false};
  bool entry_step_recovery_{false}, recovery_step_pending_{false};
  int recovery_steps_completed_{0}, recovery_step_limit_{0};
  HurdleMode recovery_resume_mode_{HurdleMode::kApproach};
  MissionAction recovery_step_action_{MissionAction::kStepForwardOne};
  bool last_approach_pending_{false};
  int last_approach_zone_{0};
  int last_approach_steps_{0};
  bool fine_pose_entered_{false};
  bool approach_two_issued_{false};
  bool test_camera_pending_{false};
  int lost_count_{0};
  int recovery_visible_count_{0};
  bool has_smoothed_{false};
  bool verified_observation_pending_{false};
  ObjectObservationWindow observation_window_;
  DownSearchWalk down_search_;
  StationaryObjectRecovery stationary_recovery_;
  int completed_back_steps_{0};
  int completed_contact_steps_{0};
  bool contact_step_pending_{false};
  double contact_next_step_sec_{0.0};
  TrackedHurdle tracked_;
  double last_seen_u_px_{0.0};
  double state_enter_sec_{0.0};
  double ignore_until_sec_{0.0};
};

} // namespace vision_core
