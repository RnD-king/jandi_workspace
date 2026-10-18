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
  kRecoverySearch, // Preserve existing goal mission phase values.
  kForwardApproach,
  kPostShootBackAway,
  kCoarseAdjust,
  kWaitFinePose,
  kFineRecovery,
  kCoarseRecovery,
  kRecoveryTurn,
  kEdgeTurn,
};

struct GoalConfig {
  double edge_margin_px{};
  int recovery_turn_max{3};
  double recovery_observation_sec{};
  int recovery_max_observations{};
  double approach_short_z_m{};
  double fine_one_step_range_error_m{};
  double fine_two_step_range_error_m{};
  double camera_goal_start_z_m{};
  int post_shoot_back_steps{};
  int stable_window{};
  int stable_min_hits{};
  int lost_frames{};
  double smooth_alpha{};
  int post_pickup_walk_fallback_actions{}; // after three completed LINE walks if no 5-step DONE
  double target_u_px{};
  double approach_u_deadband_px{};
  double fine_adjust_start_z_m{};
  double hoop_radius_m{};
  double throw_landing_x_m{};
  double throw_landing_z_m{};
  double position_tolerance_m{};
  double shoot_yaw_limit_deg{};
  double recovery_timeout_sec{};
  int recovery_reacquire_min_hits{};
  double recovery_center_tolerance_px{};
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
  Point2 center_px;
  double u_norm{0.0};
  double v_norm{0.0};
  double h_norm{0.0};
  double confidence{0.0};
};

struct GoalResult {
  int recovery_observations_completed{0};
  double recovery_observation_elapsed_sec{0.0};
  bool active{false};
  CameraRequest camera_request{CameraRequest::kNone};
  GoalMode mode{GoalMode::kLineFollow};
  TrackedGoal tracked;
  TrackedGoalPose pose;
  // Equal-weight stationary pose supplied to FineAction for this step.
  GoalPoseObservation fine_decision_pose;
  bool fine_decision_pose_used{false};
  double shoot_yaw_rad{0.0};
  ActionRequest action;
};

class GoalController {
public:
  bool CanEnterFromLine() const { return has_ball_ && goal_entry_armed_ && !ball_consumed_; }
  GoalController();
  GoalController(const GoalConfig &config, double camera_motion_timeout_sec);
  void StartAfterPickup(double now_sec);
  void StartForwardTrial();
  void StartGoalTrial() { Reset(); StartAfterPickup(0); mode_=GoalMode::kWaitCameraGoal; }
  void StartTestNear() { StartGoalTrial(); }
  void StartTestExecute() { Reset(); test_execute_requires_stable_=true; SetHasBall(true); mode_=GoalMode::kFineAdjust; }
  void SetHasBall(bool has_ball);
  void UpdateBallState(const BallResult &ball_result);
  bool HasBall() const { return has_ball_; }
  bool ExecutionObservationReady() const {
    return tracked_.visible && tracked_.stable && tracked_pose_.visible && tracked_pose_.stable &&
        hit_history_.size()>=static_cast<std::size_t>(config_.stable_window) &&
        pose_hit_history_.size()>=static_cast<std::size_t>(config_.stable_window);
  }
  int RecoveryTurnsCompleted() const { return recovery_turns_completed_; }
  int RecoveryObservationsCompleted() const { return stationary_recovery_.Completed(); }
  double RecoveryObservationElapsed(double now) const { return stationary_recovery_.Elapsed(now); }
  GoalResult Compute(const std::optional<ObjectTarget> &goal_target,
                     const std::optional<ObjectTarget> &backboard_target,
                     const GoalPoseObservation &goal_pose,
                     int image_width, int image_height, double now_sec,
                     bool line_reference_valid,
                     const CameraFeedback &camera_feedback,
                     const ActionExecutionFeedback &action_feedback,
                     bool allow_new_body_actions = true,
                     std::optional<double> center_depth_m = std::nullopt,
                     bool post_pickup_line_gate_satisfied = false,
                     bool backboard_image_without_depth = false,
                     bool allow_mission_entry = true);
  static const char *ModeName(GoalMode mode);
  // Discard all observations made before the most recent motion DONE.
  // The mission phase and last known recovery direction are retained.
  void ClearObservationHistory();
  // When validated moving READY evidence replaces in-motion samples at
  // DONE, its depth/pose alone must decide the next camera/fine transition.
  void ClearMovingApproachTriggers() {
    camera_trigger_latched_ = false;
    fine_trigger_latched_ = false;
  }
  void ObserveOnly(const std::optional<ObjectTarget> &target,
                   const GoalPoseObservation &pose, int width, int height);
  void UseVerifiedObservation(const std::optional<ObjectTarget>& mean,
      const GoalPoseObservation& pose, const std::deque<bool>& hits,
      const std::deque<bool>& pose_hits, int width, int height,
      const std::deque<bool>& near_hits = {});
  void UseNextObservationAsMean() {
    has_smoothed_ = false; has_pose_smoothed_ = false;
    fine_pose_window_.clear();
    fine_pose_mean_pending_ = true;
    fine_pose_is_preaveraged_ = false;
  }
  void UpdateTuning(const GoalConfig &config) {
    // A previewed boundary correction must be reconsidered with the new margin;
    // never interrupt an accepted two-turn sequence, including between DONEs.
    if (mode_ == GoalMode::kEdgeTurn && !turn_pending_ && edge_turns_completed_ == 0)
      mode_ = turn_resume_mode_;
    config_ = config;
  }
  void ShiftTimers(double duration) { state_enter_sec_+=duration; }
  void Reset();

private:
  static double Clamp(double value, double low, double high);
  void UpdateGoalTracker(const std::optional<ObjectTarget> &target,
                         int image_width, int image_height);
  void UpdatePoseTracker(const std::optional<ObjectTarget> &backboard_target,
                         const GoalPoseObservation &goal_pose);
  bool PoseReadyForFineAdjust() const;
  std::optional<GoalPoseObservation> FinePoseMean() const;
  ActionRequest FineAction(const GoalPoseObservation &pose) const;
  double ComputeShootYawRad(double x_m, double z_m, double yaw_rad) const;
  double LandingErrorM(double rim_x_m, double rim_z_m, double shoot_yaw_rad) const;
  double ThrowRangeM() const;
  void ClearTracking();
  ActionRequest SearchRecoveryAction() const;

  GoalConfig config_;
  double camera_motion_timeout_sec_{};
  bool test_execute_requires_stable_{false};
  GoalMode mode_{GoalMode::kLineFollow};
  std::deque<bool> hit_history_;
  std::deque<bool> pose_hit_history_;
  std::deque<bool> fine_enter_history_;
  std::deque<std::optional<GoalPoseObservation>> fine_pose_window_;
  bool fine_pose_mean_pending_{false};
  bool fine_pose_is_preaveraged_{false};
  int lost_count_{0};
  int pose_lost_count_{0};
  bool has_smoothed_{false};
  bool verified_observation_pending_{false};
  bool has_pose_smoothed_{false};
  TrackedGoal tracked_;
  TrackedGoalPose tracked_pose_;
  double state_enter_sec_{0.0};
  double shoot_yaw_rad_{0.0};
  bool has_ball_{false};
  bool ball_consumed_{false};
  bool goal_entry_armed_{false};
  bool post_pickup_line_wait_active_{false};
  double post_pickup_line_wait_start_sec_{0.0};
  bool camera_trigger_latched_{false};
  bool camera_request_started_{false};
  bool fine_trigger_latched_{false};
  double last_seen_u_px_{0.0};
  std::optional<double> last_stable_u_px_;
  // Direction only: never reused as GOAL-view stability or Depth evidence.
  std::optional<double> entry_recovery_u_;
  bool entry_recovery_pending_{false};
  GoalMode turn_resume_mode_{GoalMode::kApproach};
  bool turn_left_{false};
  bool turn_pending_{false};
  bool turn_done_{false};
  bool recovery_exhausted_{false};
  int recovery_turns_completed_{0};
  int edge_turns_completed_{0};
  void CaptureEntryDirection();
  void BeginLostRecovery(double now_sec, bool fine,
                         std::optional<double> direction_u = std::nullopt);

  bool seen_after_camera_goal_{false};
  int recovery_visible_count_{0};
  int completed_back_steps_{0};
  bool back_away_issued_{false};
  StationaryObjectRecovery stationary_recovery_;
};

} // namespace vision_core
