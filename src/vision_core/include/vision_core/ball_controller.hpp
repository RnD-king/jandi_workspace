#pragma once

#include <deque>
#include <optional>

#include "vision_core/types.hpp"

namespace vision_core {

enum class BallMode {
  kLineFollow = 0,
  kApproachBall,
  kWaitCameraDown,
  kFineAdjustForPickup,
  kPickupBall,
  kVerifyPickup,
  kVerifyPickupObservation,
  kStandUpAfterPickup,
  kPostPickupBackAway,
  kReturnCameraToLine,
  kPostPickupLineRecovery,
  kBallRecoveryForward,
  kBallRecoveryDown,
  kFailed,
};

struct BallConfig {
  int stable_window{};
  int stable_min_hits{};
  int lost_frames{};
  double smooth_alpha{};
  double far_u_des_norm{};
  double approach_u_deadband{};
  double upper_acquire_v_norm{};
  double tilt_down_v_norm{};
  int tilt_down_window{};
  int tilt_down_min_hits{};
  int pickup_max_attempts{};
  double ball_ignore_duration_sec{};
  double fine_target_u_norm{};
  double fine_target_v_norm{};
  double fine_u_deadband{};
  double fine_v_deadband{};
  double fine_settle_duration_sec{};
  double recovery_timeout_sec{};
  int recovery_reacquire_min_hits{};
  double recovery_center_tolerance_norm{};
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
                     const ActionExecutionFeedback &action_feedback);
  static const char *ModeName(BallMode mode);
  bool HasBall() const { return has_ball_; }
  void SetHasBall(bool has_ball) { has_ball_ = has_ball; }
  void CompleteLineRecovery(double now_sec);
  void ClearEntryEvidence();
  // Discard all observations made before the most recent motion DONE.
  // The mission phase and last known recovery direction are retained.
  void ClearObservationHistory();
  void ObserveOnly(const std::optional<ObjectTarget> &target, int width, int height);
  void Reset();

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
  std::deque<bool> tilt_history_;
  int lost_count_{0};
  int recovery_visible_count_{0};
  bool has_smoothed_{false};
  TrackedBall tracked_;
  double last_seen_u_norm_{0.5};
  double state_enter_sec_{0.0};
  double settle_until_sec_{0.0};
  double ignore_ball_until_sec_{0.0};
  bool camera_trigger_latched_{false};
  bool back_away_issued_{false};
  bool has_ball_{false};
  bool pickup_failed_{false};
  int pickup_attempt_count_{0};
};

} // namespace vision_core
