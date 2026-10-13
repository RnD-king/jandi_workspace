#pragma once

#include <deque>
#include <optional>

#include "vision_core/types.hpp"
#include "vision_core/object_observation_window.hpp"
#include "vision_core/stationary_object_recovery.hpp"

namespace vision_core {

enum class BallMode {
  kLineFollow = 0,
  kApproachBall,
  kWaitCameraDown,
  kFineAdjustForPickup,
  kPickupBall,
  kVerifyPickupObservation = 6, // Keep later phase IDs stable after removing RECATCH.
  kStandUpAfterPickup,
  kPostPickupBackAway,
  kReturnCameraToLine,
  kPostPickupLineRecovery,
  kBallRecoveryForward,
  kBallRecoveryDown,
  kFailed,
  kDownSearch,
  kApproachTwoBeforeDown,
  kCoarseAdjust,
  kWaitFinePose,
};

struct BallConfig {
  double approach_short_v_px{};
  double down_search_timeout_sec{};
  double recovery_observation_sec{};
  int recovery_max_observations{};
  int post_pickup_back_steps{};

  int stable_window{};
  int stable_min_hits{};
  int lost_frames{};
  double smooth_alpha{}; // Legacy config key; position uses the bounded 1->3 window.
  double far_u_des_px{};
  double approach_u_deadband_px{};
  double upper_acquire_v_px{};
  double approach_two_step_v_px{};
  double down_one_step_v_px{};
  double fine_mode_v_px{};
  int pickup_max_attempts{};
  double ball_ignore_duration_sec{};
  // Inclusive pickup rectangle edges, in input-image pixels.
  double pickup_u1_px{}, pickup_u2_px{}, pickup_v1_px{}, pickup_v2_px{};
  double PickupCenterU() const { return (pickup_u1_px + pickup_u2_px) * 0.5; }
  double PickupCenterV() const { return (pickup_v1_px + pickup_v2_px) * 0.5; }
  double PickupHalfWidth() const { return (pickup_u2_px - pickup_u1_px) * 0.5; }
  double PickupHalfHeight() const { return (pickup_v2_px - pickup_v1_px) * 0.5; }
  double fine_settle_duration_sec{};
  double recovery_timeout_sec{};
  int recovery_reacquire_min_hits{};
  double recovery_center_tolerance_px{};
  double recovery_settle_duration_sec{};
};

struct TrackedBall {
  bool stable{false};
  bool visible{false};
  Point2 center_px;
  double u_norm{0.0};
  double v_norm{0.0};
  double h_norm{0.0};
  double area_norm{0.0};
  double confidence{0.0};
};

struct BallResult {
  bool active{false};
  bool has_ball{false};
  bool pickup_failed{false};
  int pickup_attempt_count{0};
  int recovery_observations_completed{0};
  double recovery_observation_elapsed_sec{0.0};
  double down_search_remaining_sec{0.0};
  CameraRequest camera_request{CameraRequest::kNone};
  BallMode mode{BallMode::kLineFollow};
  TrackedBall tracked;
  ActionRequest action;
};

class BallController {
public:
  BallController();
  BallController(const BallConfig &config, double camera_motion_timeout_sec);
  BallResult Compute(const std::optional<ObjectTarget> &ball_target,
                     int image_width, int image_height, double now_sec,
                     const CameraFeedback &camera_feedback,
                     const ActionExecutionFeedback &action_feedback,
                     bool allow_new_body_actions = true);
  static const char *ModeName(BallMode mode);
  bool HasBall() const { return has_ball_; }
  void SetHasBall(bool has_ball) { has_ball_ = has_ball; }
  void CompleteLineRecovery(double now_sec);
  void ClearEntryEvidence();
  // Discard all observations made before the most recent motion DONE.
  // The mission phase and last known recovery direction are retained.
  void ClearObservationHistory();
  void ObserveOnly(const std::optional<ObjectTarget> &target, int width, int height);
  void UseNextObservationAsMean() { has_smoothed_ = false; observation_window_.Clear(); }
  void UpdateTuning(const BallConfig &config) { config_ = config; }
  void ShiftTimers(double duration) { state_enter_sec_+=duration; settle_until_sec_+=duration; }
  // Isolated test startup selects an existing FSM phase; no completion is fabricated.
  void StartTestNear() { Reset(); mode_=BallMode::kWaitCameraDown; test_camera_pending_=true; }
  void StartTestExecute() { Reset(); fine_pose_entered_=true; mode_=BallMode::kFineAdjustForPickup; }
  void Reset();
  bool ExecutionObservationReady() const {
    return tracked_.visible && tracked_.stable &&
        hit_history_.size()>=static_cast<std::size_t>(config_.stable_window);
  }
  int RecoveryObservationsCompleted() const { return stationary_recovery_.Completed(); }
  double RecoveryObservationElapsed(double now) const { return stationary_recovery_.Elapsed(now); }
  double DownSearchRemaining(double now) const {
    return std::max(0.0, config_.down_search_timeout_sec -
        (down_search_started_ ? now - state_enter_sec_ : 0.0));
  }

private:
  static double Clamp(double value, double low, double high);
  void UpdateTracker(const std::optional<ObjectTarget> &target,
                     int image_width, int image_height);
  ActionRequest FarAction() const;
  ActionRequest FineAction() const;
  ActionRequest RecoveryAction() const;
  void ClearTracking();
  void ResetToLine(bool clear_ignore);

  BallConfig config_;
  double camera_motion_timeout_sec_{};
  BallMode mode_{BallMode::kLineFollow};
  std::deque<bool> hit_history_;
  std::deque<bool> upper_acquire_history_;
  int last_approach_steps_{0};
  bool fine_pose_entered_{false};
  bool approach_two_issued_{false};
  bool test_camera_pending_{false};
  int lost_count_{0};
  int recovery_visible_count_{0};
  bool has_smoothed_{false};
  ObjectObservationWindow observation_window_;
  bool down_search_started_{false};
  bool down_search_found_{false};
  StationaryObjectRecovery stationary_recovery_;
  int completed_back_steps_{0};
  int completed_contact_steps_{0};
  TrackedBall tracked_;
  double last_seen_u_px_{0.0};
  double state_enter_sec_{0.0};
  double settle_until_sec_{0.0};
  double ignore_ball_until_sec_{0.0};
  bool back_away_issued_{false};
  bool has_ball_{false};
  bool pickup_failed_{false};
  int pickup_attempt_count_{0};
};

} // namespace vision_core
