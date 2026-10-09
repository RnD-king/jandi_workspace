#pragma once

#include <deque>
#include <optional>

#include "vision_core/ball_controller.hpp"
#include "vision_core/types.hpp"

namespace vision_core {

enum class GoalMode {
  kLineFollow = 0,
  kWaitCameraGoal,
  kSearch,
  kApproach,
  kFineAdjust,
  kShoot,
  kReturnCameraToLine,
  kHeadingRecovery,
  kFailed,
};

struct GoalConfig {
  int stable_window{};
  int stable_min_hits{};
  int lost_frames{};
  double smooth_alpha{};
  double post_pickup_wait_sec{};
  double target_u_norm{};
  double approach_u_deadband{};
  double fine_adjust_start_z_m{};
  double hoop_radius_m{};
  double throwing_range_m{};
  double position_tolerance_m{};
  double fine_settle_duration_sec{};
  double shoot_yaw_limit_deg{};
};

struct GoalPoseObservation {
  bool valid{false};
  double x_m{0.0};
  double z_m{0.0};
  double yaw_rad{0.0};
  double confidence{0.0};
};

GoalPoseObservation EstimateGoalPoseFromBackboardDepths(
    double center_u_px, double center_depth_m, double left_u_px,
    double left_depth_m, double right_u_px, double right_depth_m,
    const Intrinsics &intrinsics, double confidence = 1.0);

GoalPoseObservation EstimateGoalPoseFromEdgeDepths(
    double left_u_px, double left_depth_m, double right_u_px,
    double right_depth_m, const Intrinsics &intrinsics,
    double confidence = 1.0);

struct TrackedGoalPose {
  bool stable{false};
  bool visible{false};
  double x_m{0.0};
  double z_m{0.0};
  double yaw_rad{0.0};
  double confidence{0.0};
};

struct TrackedGoal {
  bool stable{false};
  bool visible{false};
  double u_norm{0.0};
  double v_norm{0.0};
  double h_norm{0.0};
  double confidence{0.0};
};

struct GoalResult {
  bool active{false};
  CameraRequest camera_request{CameraRequest::kNone};
  GoalMode mode{GoalMode::kLineFollow};
  TrackedGoal tracked;
  TrackedGoalPose pose;
  double shoot_yaw_rad{0.0};
  ActionRequest action;
};

class GoalController {
public:
  GoalController();
  GoalController(const GoalConfig &config, double camera_motion_timeout_sec);
  void StartAfterPickup(double now_sec);
  void SetHasBall(bool has_ball);
  void UpdateBallState(const BallResult &ball_result);
  bool HasBall() const { return has_ball_; }
  GoalResult Compute(const std::optional<ObjectTarget> &goal_target,
                     const std::optional<ObjectTarget> &backboard_target,
                     const GoalPoseObservation &goal_pose,
                     int image_width, int image_height, double now_sec,
                     bool line_reference_valid,
                     const CameraFeedback &camera_feedback,
                     const ActionExecutionFeedback &action_feedback);
  static const char *ModeName(GoalMode mode);
  void Reset();

private:
  static double Clamp(double value, double low, double high);
  void UpdateGoalTracker(const std::optional<ObjectTarget> &target,
                         int image_width, int image_height);
  void UpdatePoseTracker(const std::optional<ObjectTarget> &backboard_target,
                         const GoalPoseObservation &goal_pose);
  bool PoseReadyForFineAdjust() const;
  ActionRequest FineAction(const GoalPoseObservation &pose) const;
  double ComputeShootYawRad(double x_m, double z_m, double yaw_rad) const;
  void ClearTracking();

  GoalConfig config_;
  double camera_motion_timeout_sec_{};
  GoalMode mode_{GoalMode::kLineFollow};
  std::deque<bool> hit_history_;
  std::deque<bool> pose_hit_history_;
  std::deque<bool> fine_enter_history_;
  int lost_count_{0};
  int pose_lost_count_{0};
  bool has_smoothed_{false};
  bool has_pose_smoothed_{false};
  TrackedGoal tracked_;
  TrackedGoalPose tracked_pose_;
  double state_enter_sec_{0.0};
  double settle_until_sec_{0.0};
  double shoot_yaw_rad_{0.0};
  bool has_ball_{false};
  bool ball_consumed_{false};
  bool goal_entry_armed_{false};
  bool post_pickup_line_wait_active_{false};
  double post_pickup_line_wait_start_sec_{0.0};
  bool camera_trigger_latched_{false};
  bool fine_trigger_latched_{false};
};

} // namespace vision_core
