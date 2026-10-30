#pragma once

#include <algorithm>
#include <deque>
#include <optional>
#include <utility>

#include "vision_core/types.hpp"
#include "vision_core/ball_fine_motion.hpp"
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
  kBallRecoveryWalkTurn, // WALK-stance 31/28 search before stationary fallback
  kNearTurnBeforeDown, // One extra zone-7/9 turn, reobserve, then DOWN if still lost
  kFineSidePair, // Fixed pair of existing 5/6 actions; append to preserve phase IDs.
  kMidTurnBeforeRecovery, // One WALK turn after zone-4/6 loss, then stationary recovery.
  kFineMotionRepeat, // Calibrated same-direction FINE chain, at most three actions.
  kForwardRecoveryTurn, // Side hint: one WALK turn, then one forward step.
  kForwardRecoveryStep,
  kForwardRecoveryObserve, // Initial central wait or fresh post-step evidence.
};

struct BallConfig {
  double approach_short_v_px{};
  int down_search_max_steps{};
  double down_search_inter_action_hold_sec{}; // Real DONE to next DOWN search step.
  double recovery_observation_sec{};
  int recovery_max_observations{};
  int recovery_walk_turn_max{3}; // Zone-4/6 rotation-only recovery.
  int recovery_max_steps{}; // General FORWARD loss: turn/step or central step search.
  int entry_recovery_max_steps{}; // First post-entry loss; exhausted -> LINE.
  int down_recovery_max_actions{}; // FINE rectangle-directed search, then fail.
  int post_pickup_back_steps{};

  int stable_window{};
  int stable_min_hits{};
  int down_search_min_hits{}; // DOWN search only, within stable_window.
  int entry_min_hits{}; // LINE entry only; independent of FINE stability.
  int lost_frames{};
  double far_u_des_px{};
  double approach_u_deadband_px{};
  double approach_left_top_u_px{};
  double approach_right_top_u_px{};
  // Top endpoints are at v=0; bottom endpoints keep the legacy center/band
  // at v=image_height-1. Use the same interpolation for control and drawing.
  std::pair<double,double> ApproachBounds(double v, int image_height) const {
    const double t=std::clamp(v/std::max(1,image_height-1),0.0,1.0);
    const double left_bottom=far_u_des_px-approach_u_deadband_px;
    const double right_bottom=far_u_des_px+approach_u_deadband_px;
    if (t==1.0) return {left_bottom,right_bottom};
    return {approach_left_top_u_px+t*(left_bottom-approach_left_top_u_px),
            approach_right_top_u_px+t*(right_bottom-approach_right_top_u_px)};
  }
  double upper_acquire_v_px{};
  double approach_two_step_v_px{};
  double approach_direct_down_v_px{}; // Zone 8 only: one step instead of two before DOWN.
  double fine_side_two_step_margin_px{};
  BallFineMotionConfig fine_motion;
  int pickup_max_attempts{};
  int pickup_verify_min_hits{}; // Residual ball votes after PICK DONE; independent of stability.
  // Inclusive pickup rectangle edges, in input-image pixels.
  double pickup_u1_px{}, pickup_u2_px{}, pickup_v1_px{}, pickup_v2_px{};
  double PickupCenterU() const { return (pickup_u1_px + pickup_u2_px) * 0.5; }
  double PickupCenterV() const { return (pickup_v1_px + pickup_v2_px) * 0.5; }
  double PickupHalfWidth() const { return (pickup_u2_px - pickup_u1_px) * 0.5; }
  double PickupHalfHeight() const { return (pickup_v2_px - pickup_v1_px) * 0.5; }
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
  int recovery_steps_completed{0}, recovery_step_limit{0};
  bool entry_recovery{false};
  int down_search_steps_remaining{0};
  bool down_search_found{false};
  int detection_hits{0}, detection_frames{0}, lost_frames{0};
  MissionAction fine_motion_action{MissionAction::kNone};
  int fine_motion_repetitions{0},fine_motion_remaining{0};
  bool fine_motion_prediction_valid{false};
  Point2 fine_motion_predicted_center_px;
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
  std::optional<double> EntryDirection() const;
  void EnterVerified(const std::optional<ObjectTarget>& mean,
                     const std::deque<bool>& hits, int width, int height, double now,
                     bool allow_entry_recovery = true);
  void EnterContinuousRecovery(const ObjectTarget&,const std::deque<bool>&,
                               int width,int height,double now);
  void EnterContinuousNear(double now);
  const std::deque<bool>& ObservationHits() const { return hit_history_; }
  bool HasBall() const { return has_ball_; }
  void SetHasBall(bool has_ball) { has_ball_ = has_ball; }
  void CompleteLineRecovery();
  void ClearEntryEvidence();
  // Discard all observations made before the most recent motion DONE.
  // The mission phase and last known recovery direction are retained.
  void ClearObservationHistory();
  void InvalidateTargetIdentity() {
    // A completed detection vote still stops DOWN search after the real DONE.
    // A new identity must rebuild its own observations, not revoke that stop.
    ClearObservationHistory();
  }
  // A changed bbox filter invalidates the latched search detection as well.
  void InvalidateDetectionEvidence() {
    InvalidateTargetIdentity();
    down_search_.InvalidateDetection();
  }
  void ObserveOnly(const std::optional<ObjectTarget> &target, int width, int height);
  void UseVerifiedObservation(const std::optional<ObjectTarget>& mean,
      const std::deque<bool>& hits, int width, int height);
  void UseNextObservationAsMean() { has_smoothed_ = false; observation_window_.Clear(); }
  // Accepted same-direction chains retain their action/count when parameters change.
  void UpdateTuning(const BallConfig &config) {
    if (mode_==BallMode::kDownSearch && config_.down_search_min_hits!=config.down_search_min_hits)
      down_search_.InvalidateDetection();
    config_ = config;
  }
  void ShiftTimers(double duration) {
    state_enter_sec_+=duration;
    down_search_.ShiftTimers(duration);
  }
  // Isolated test startup selects an existing FSM phase; no completion is fabricated.
  void StartTestNear() { Reset(); mode_=BallMode::kWaitCameraDown; test_camera_pending_=true; }
  void StartTestExecute() { Reset(); fine_pose_entered_=true; mode_=BallMode::kFineAdjustForPickup; }
  void Reset();
  bool ExecutionObservationReady() const {
    return tracked_.visible && tracked_.stable &&
        hit_history_.size()>=static_cast<std::size_t>(config_.stable_window);
  }
  int RecoveryObservationsCompleted() const { return stationary_recovery_.Completed(); }
  bool DownSearchFound() const { return down_search_.Found(); }
  int LostFrames() const { return lost_count_; }
  int DetectionFrames() const { return static_cast<int>(hit_history_.size()); }
  int DetectionHits() const { return static_cast<int>(std::count(hit_history_.begin(),hit_history_.end(),true)); }
  int FineSideActionsRemaining() const { return fine_side_remaining_; }
  int FineMotionActionsRemaining() const { return fine_side_remaining_; }
  int FineMotionRepetitions() const { return fine_repeat_total_; }
  MissionAction FineMotionAction() const { return fine_side_action_; }
  Point2 FineMotionPredictedCenter() const { return fine_repeat_predicted_center_; }
  int RecoveryWalkTurnsCompleted() const { return recovery_walk_turns_completed_; }
  int RecoveryStepsCompleted() const { return down_recovery_active_ ? down_recovery_completed_ : forward_recovery_steps_completed_; }
  int RecoveryStepLimit() const { return down_recovery_active_ ? config_.down_recovery_max_actions : forward_recovery_step_limit_; }
  bool EntryRecovery() const { return !down_recovery_active_ && forward_recovery_entry_; }
  double RecoveryObservationElapsed(double now) const { return stationary_recovery_.Elapsed(now); }
  int DownSearchStepsRemaining() const {
    return down_search_.Remaining(config_.down_search_max_steps);
  }

private:
  static double Clamp(double value, double low, double high);
  void UpdateTracker(const std::optional<ObjectTarget> &target,
                     int image_width, int image_height);
  ActionRequest FarAction(int image_height) const;
  ActionRequest FineAction() const;
  ActionRequest FineActionAt(Point2 center) const;
  ActionRequest RecoveryAction() const;
  void ClearTracking();
  void BeginForwardRecovery(double now, std::optional<double> u, bool entry_loss,
                            bool rotation_only = false);
  void ExhaustForwardRecovery(double now, BallResult &result);
  void BeginDownRecovery(double now);
  void ResetToLine();

  BallConfig config_;
  double camera_motion_timeout_sec_{};
  BallMode mode_{BallMode::kLineFollow};
  std::deque<bool> hit_history_;
  std::deque<bool> entry_hit_history_;
  std::deque<std::optional<double>> entry_u_history_;
  std::optional<double> entry_recovery_u_;
  bool entry_recovery_pending_{false};
  bool entry_recovery_allowed_{true};
  MissionAction fine_side_action_{MissionAction::kNone};
  int fine_side_remaining_{0};
  bool fine_side_pending_{false};
  bool fine_side_done_{false};
  int fine_repeat_total_{0};
  Point2 fine_repeat_predicted_center_;
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
  std::optional<Point2> last_down_center_;
  MissionAction down_recovery_action_{MissionAction::kNone};
  int down_recovery_completed_{0};
  bool down_recovery_active_{false}, down_recovery_pending_{false};
  bool down_recovery_observing_{false}, down_recovery_pose_pending_{false};
  std::optional<bool> recovery_turn_left_;
  int recovery_walk_turns_completed_{0};
  int recovery_walk_turn_limit_{3}; // Fixed per loss episode.
  bool recovery_walk_turn_pending_{false};
  bool recovery_walk_turn_done_{false};
  bool recovery_turn_search_exhausted_{false};
  bool forward_recovery_entry_{false};
  int forward_recovery_steps_completed_{0};
  int forward_recovery_step_limit_{0};
  bool forward_recovery_action_pending_{false};
  double state_enter_sec_{0.0};
  bool back_away_issued_{false};
  bool has_ball_{false};
  bool pickup_failed_{false};
  int pickup_attempt_count_{0};
};

} // namespace vision_core
