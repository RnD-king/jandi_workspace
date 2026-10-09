#pragma once

#include <deque>
#include <optional>

#include "vision_core/types.hpp"

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
};

struct HurdleConfig {
  int stable_window{};
  int stable_min_hits{};
  int lost_frames{};
  double smooth_alpha{};
  double acquire_min_v_norm{};
  double tilt_trigger_v_norm{};
  int tilt_trigger_window{};
  int tilt_trigger_min_hits{};
  double hurdle_ignore_duration_sec{};
  int recovery_reacquire_min_hits{};
  double recovery_timeout_sec{};
  double recovery_center_tolerance_norm{};
  double recovery_settle_duration_sec{};
};

struct TrackedHurdle {
  bool stable{false};
  bool visible{false};
  double u_norm{0.0};
  double v_norm{0.0};
  double h_norm{0.0};
  double bottom_norm{0.0};
  double confidence{0.0};
};

struct HurdleResult {
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
                       const ActionExecutionFeedback &action_feedback);
  static const char *ModeName(HurdleMode mode);
  void Reset();

private:
  static double Clamp(double value, double low, double high);
  void UpdateTracker(const std::optional<ObjectTarget> &target,
                     int image_width, int image_height);
  ActionRequest RecoveryAction() const;
  void ResetToLine(bool clear_ignore);

  HurdleConfig config_;
  double camera_motion_timeout_sec_{};
  HurdleMode mode_{HurdleMode::kLineFollow};
  std::deque<bool> hit_history_;
  std::deque<bool> acquire_history_;
  std::deque<bool> tilt_history_;
  int lost_count_{0};
  int recovery_visible_count_{0};
  bool has_smoothed_{false};
  TrackedHurdle tracked_;
  double last_seen_u_norm_{0.5};
  double state_enter_sec_{0.0};
  double settle_until_sec_{0.0};
  double ignore_until_sec_{0.0};
  bool close_trigger_latched_{false};
};

} // namespace vision_core
