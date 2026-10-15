#pragma once

#include <deque>
#include <optional>

#include "vision_core/types.hpp"
#include "vision_core/down_search_walk.hpp"
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
  kApproachTwoBeforeDown, // Final one/two-step walk; retain the existing phase ID.
  kCoarseAdjust,
  kWaitFinePose,
  kBallRecoveryWalkTurn, // WALK-stance 27/28 search before stationary fallback
  kNearTurnBeforeDown, // One extra zone-7/9 turn, reobserve, then DOWN if still lost
  kFineSidePair, // Fixed pair of existing 5/6 actions; append to preserve phase IDs.
  kMidTurnBeforeRecovery, // One WALK turn after zone-4/6 loss, then stationary recovery.
};

struct BallConfig {
  double approach_short_v_px{};
  int down_search_max_steps{};
  double recovery_observation_sec{};
  int recovery_max_observations{};
  int recovery_walk_turn_max{3};
  int post_pickup_back_steps{};

  int stable_window{};
  int stable_min_hits{};
  int down_search_min_hits{}; // DOWN search only, within stable_window.
  int entry_min_hits{}; // LINE entry only; independent of FINE stability.
  int lost_frames{};
  double smooth_alpha{}; // Legacy config key; position uses the bounded 1->3 window.
  double far_u_des_px{};
  double approach_u_deadband_px{};
  double upper_acquire_v_px{};
  double approach_two_step_v_px{};
  double approach_direct_down_v_px{}; // Zone 8 only: one step instead of two before DOWN.
  double down_one_step_v_px{};
  double fine_mode_v_px{};
  double fine_side_two_step_margin_px{};
  int pickup_max_attempts{};
  double ball_ignore_duration_sec{};
  // Inclusive pickup rectangle edges, in input-image pixels.
  double pickup_u1_px{}, pickup_u2_px{}, pickup_v1_px{}, pickup_v2_px{};
  double PickupCenterU() const { return (pickup_u1_px + pickup_u2_px) * 0.5; }
  double PickupCenterV() const { return (pickup_v1_px + pickup_v2_px) * 0.5; }
  double PickupHalfWidth() const { return (pickup_u2_px - pickup_u1_px) * 0.5; }
  double PickupHalfHeight() const { return (pickup_v2_px - pickup_v1_px) * 0.5; }
  double recovery_timeout_sec{};
  int recovery_reacquire_min_hits{};
  double recovery_center_tolerance_px{};
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
  int entry_frames{0};
  int entry_hits{0};
  int recovery_observations_completed{0};
  double recovery_observation_elapsed_sec{0.0};
  int down_search_steps_remaining{0};
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
                     bool allow_new_body_actions = true,
                     bool allow_mission_entry = true,
                     std::optional<bool> entry_detected = std::nullopt,
                     std::optional<double> entry_center_u = std::nullopt,
                     bool allow_pickup = true,
                     bool allow_fine_pair = true);
  static const char *ModeName(BallMode mode);
  bool HasBall() const { return has_ball_; }
  void SetHasBall(bool has_ball) { has_ball_ = has_ball; }
  void CompleteLineRecovery(double now_sec);
  void ClearEntryEvidence();
  // Discard all observations made before the most recent motion DONE.
  // The mission phase and last known recovery direction are retained.
  void ClearObservationHistory();
  // A changed bbox filter invalidates the latched search detection as well.
  void InvalidateDetectionEvidence() { ClearObservationHistory(); down_search_.InvalidateDetection(); }
  void ObserveOnly(const std::optional<ObjectTarget> &target, int width, int height);
  void UseVerifiedObservation(const std::optional<ObjectTarget>& mean,
      const std::deque<bool>& hits, int width, int height);
  void UseNextObservationAsMean() { has_smoothed_ = false; observation_window_.Clear(); }
  // Accepted pairs retain their action/count when parameters change.
  void UpdateTuning(const BallConfig &config) {
    if (mode_==BallMode::kDownSearch && config_.down_search_min_hits!=config.down_search_min_hits)
      down_search_.InvalidateDetection();
    config_ = config;
  }
  void ShiftTimers(double duration) { state_enter_sec_+=duration; }
  // Isolated test startup selects an existing FSM phase; no completion is fabricated.
  void StartTestNear() { Reset(); mode_=BallMode::kWaitCameraDown; test_camera_pending_=true; }
  void StartTestExecute() { Reset(); fine_pose_entered_=true; mode_=BallMode::kFineAdjustForPickup; }
  void Reset();
  bool ExecutionObservationReady() const {
    return tracked_.visible && tracked_.stable &&
        hit_history_.size()>=static_cast<std::size_t>(config_.stable_window);
  }
  int RecoveryObservationsCompleted() const { return stationary_recovery_.Completed(); }
  int FineSideActionsRemaining() const { return fine_side_remaining_; }
  int RecoveryWalkTurnsCompleted() const { return recovery_walk_turns_completed_; }
  double RecoveryObservationElapsed(double now) const { return stationary_recovery_.Elapsed(now); }
  int DownSearchStepsRemaining() const {
    return down_search_.Remaining(config_.down_search_max_steps);
  }

private:
  static double Clamp(double value, double low, double high);
  void UpdateTracker(const std::optional<ObjectTarget> &target,
                     int image_width, int image_height);
  ActionRequest FarAction() const;
  ActionRequest FineAction() const;
  ActionRequest RecoveryAction() const;
  void ClearTracking();
  void BeginForwardRecovery(double now, std::optional<double> u, bool immediate_turn);
  void ResetToLine(bool clear_ignore);

  BallConfig config_;
  double camera_motion_timeout_sec_{};
  BallMode mode_{BallMode::kLineFollow};
  std::deque<bool> hit_history_;
  std::deque<bool> entry_hit_history_;
  std::deque<std::optional<double>> entry_u_history_;
  std::optional<double> entry_recovery_u_;
  bool entry_recovery_pending_{false};
  MissionAction fine_side_action_{MissionAction::kNone};
  int fine_side_remaining_{0};
  bool fine_side_pending_{false};
  bool fine_side_done_{false};
  std::deque<std::optional<double>> recent_ball_u_;
  int last_approach_steps_{0};
  int last_approach_zone_{0};
  bool approach_action_pending_{false},awaiting_approach_observation_{false};
  MissionAction mid_recovery_turn_{MissionAction::kNone};
  bool mid_recovery_turn_pending_{false};
  bool fine_pose_entered_{false};
  bool approach_walk_issued_{false};
  MissionAction approach_before_down_action_{MissionAction::kStepForwardTwo};
  MissionAction near_turn_action_{MissionAction::kNone};
  bool near_turn_done_{false};
  bool near_turn_retry_issued_{false};
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
  TrackedBall tracked_;
  double last_seen_u_px_{0.0};
  std::optional<double> last_stable_ball_u_;
  std::optional<bool> recovery_turn_left_;
  int recovery_walk_turns_completed_{0};
  bool recovery_walk_turn_pending_{false};
  bool recovery_walk_turn_done_{false};
  bool recovery_turn_search_exhausted_{false};
  double state_enter_sec_{0.0};
  double ignore_ball_until_sec_{0.0};
  bool back_away_issued_{false};
  bool has_ball_{false};
  bool pickup_failed_{false};
  int pickup_attempt_count_{0};
};

} // namespace vision_core
