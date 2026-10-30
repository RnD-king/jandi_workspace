#pragma once

#include <algorithm>
#include <cstddef>
#include <deque>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "vision_core/ball_controller.hpp"
#include "vision_core/ball_candidate_tracker.hpp"
#include "vision_core/control_command.hpp"
#include "vision_core/goal_controller.hpp"
#include "vision_core/hurdle_controller.hpp"
#include "vision_core/line_detection_extractor.hpp"
#include "vision_core/line_feature_extractor.hpp"
#include "vision_core/line_p2p_controller.hpp"
#include "vision_core/line_plan.hpp"
#include "vision_core/object_approach.hpp"
#include <array>
#include "vision_core/object_target_extractor.hpp"
#include "vision_core/object_association_tracker.hpp"
#include "vision_core/object_target_tracker.hpp"

namespace vision_core {

struct MissionControllerConfig {
  FeatureConfig line_features;
  LineTrackingConfig line;
  LineP2pConfig line_p2p;
  BallConfig ball;
  HurdleConfig hurdle;
  GoalConfig goal;
  ControlCommandConfig command;
  LineDetectionConfig line_detection;
  ObjectTargetConfig object_targets;
  ObjectAssociationConfig object_association;
  BallCandidateTrackingConfig ball_candidate_tracking;
  ObjectCandidateTrackingConfig hurdle_candidate_tracking, backboard_candidate_tracking;
  // 모든 object mission의 camera settled 대기 제한 시간이다.
  double camera_motion_timeout_sec{};
  // Fresh, post-DONE observation interval for non-queued actions and camera turns.
  double post_motion_observation_sec{};
  int candidate_search_max_turns{};
  double transition_settle_sec{};
  double walk_inter_action_hold_sec{};
  double fine_motion_settle_sec{};
  // YAML allowlist of currently executing object actions whose valid READY
  // moving mean permits one successor ActionCommand to be queued in AUTO.
  // MANUAL reuses the mean for Preview/Commit without reserving B.
  // Empty means all object walks use post-DONE stationary observation.
  std::vector<int> object_ready_mean_actions;
  ObjectApproachConfig object_approach;
  bool UsesObjectReadyMean(MissionAction action) const {
    return std::find(object_ready_mean_actions.begin(),
                     object_ready_mean_actions.end(), static_cast<int>(action)) !=
           object_ready_mean_actions.end();
  }
  // YOLO backboard 후보 중심의 aligned depth가 이 범위 안에 있어야 실제
  // backboard 관측으로 인정한다.
  double backboard_min_depth_m{};
  double backboard_max_depth_m{};
  bool enable_ball{};
  bool enable_hurdle{};
  bool enable_goal{};
  // Goal 단독 시험처럼 공 집기 과정을 생략하는 구성에서만 사용한다.
  bool initial_has_ball{};
  bool goal_trial_start_goal{}; // Skip initial LINE walking and start in GOAL view.
  int test_ball_mission_index{};
  int test_goal_mission_index{};
  int mission_recovery_max_turns{};
  MissionType test_mission{MissionType::kNone};
  int test_stage{}; // 0 full, 1 far approach, 2 near alignment, 3 mission execution/recovery
  int backboard_depth_strip_width_px{};
};

enum class FootPose { kWalk, kFine };

struct MissionFrameInput {
  std::vector<Point2> line_centers;
  Intrinsics intrinsics;
  bool rectify_approach{false};
  double approach_roll_rad{}, approach_pitch_rad{};
  std::uint64_t ball_track_id{}, hurdle_track_id{}, backboard_track_id{};
  std::optional<int> backboard_entry_track_hits;
  std::optional<ObjectTarget> ball_target;
  // StepPerception supplies per-ID entry evidence within upper_acquire_v_px.
  // Candidates already passed confidence, bbox, aspect and optional depth gates.
  // Direct callers may omit this and use ball_target detection presence.
  std::optional<int> ball_entry_track_hits, ball_entry_track_frames;
  std::optional<bool> ball_entry_detected;
  std::optional<double> ball_entry_center_u; // Actual candidate used for entry-direction memory.
  std::optional<ObjectTarget> hurdle_target;
  std::optional<int> hurdle_entry_track_hits;
  std::optional<double> hurdle_entry_center_u;
  std::optional<ObjectTarget> goal_target;
  std::optional<ObjectTarget> backboard_target;
  GoalPoseObservation goal_pose;
  // A valid image detection whose center depth is rejected/missing.
  bool backboard_image_without_depth{false};
  std::optional<double> backboard_center_depth_m;
  int image_width{0};
  int image_height{0};
  double now_sec{0.0};
  CameraFeedback camera_feedback;
  // true이면 controller용 ActionExecutionFeedback을 직전 공통 command와
  // delivery_feedback에서 MissionController가 직접 만든다. 실행기 어댑터는
  // controller 상태를 건드리지 않고 ACK/READY/DONE만 반환하면 된다.
  bool command_transport_enabled{false};
  // command_transport_enabled=false인 기존 직접 호출자를 위한 호환 입력이다.
  ActionExecutionFeedback action_feedback;
  CommandDeliveryFeedback delivery_feedback;
  // false이면 인식/FSM과 기존 feedback 처리는 계속하지만 새 LINE action만
  // 시작하지 않는다. 튜닝 도구의 one-shot gate용이다.
  bool allow_new_line_action{true};
  bool allow_new_line_plan{true};
  // Block NEW actions/camera requests across all missions while processing
  // already-issued ACK/READY/DONE normally.
  bool allow_new_commands{true};
  // In manual tuning, camera requests and DONE processing continue while
  // new body actions wait for explicit NEXT authorization.
  bool allow_new_body_actions{true};
  // Only dedicated Stage START may execute setup pose actions without COMMIT.
  bool allow_test_setup_body_actions{false};
  // During PAUSING do not discover a NEW object mission from moving images.
  bool allow_new_mission_entry{true};
  // false이면 특징은 계산하되 LINE 판단/recovery 상태는 동결한다.
  // 기존 action의 coordinator ACK/READY/DONE 처리는 계속 수행한다.
  bool advance_line_fsm{true};
  // Manual PREVIEW triggers observation; AUTO default keeps post-DONE gates.
  bool manual_observation_control{false};
  bool begin_manual_observation{false};
  // Real 10-frame evidence accompanying a manual READY average.
  std::deque<bool> manual_object_hits, manual_pose_hits, manual_near_hits;
  bool manual_ready_mean{false};
  // 단발 튜닝처럼 여러 프레임에서 집계한 guide로 이번 LINE 판단만 수행한다.
  std::optional<LineGuide> line_decision_guide_override;
};

struct MissionFrameResult {
  ControlCommand command;
  MissionType active_mission{MissionType::kLine};
  Features line_features;
  bool line_computed{false};
  std::optional<double> remembered_line_score;
  bool approach_goal_search_failed{false};
  bool approach_goal_depth_failed{false};
  // The guide supplied to LINE selector (raw, READY-cache, or post-DONE mean).
  std::optional<LineGuide> line_decision_guide;
  std::optional<LineWindowStats> line_window_stats;
  bool line_in_recovery{true};
  bool line_score_recovery{false};
  int line_score_recovery_turns_completed{0};
  bool has_ball{false};
  // Diagnostics of post-DONE observations, never counting pre-DONE history.
  bool post_motion_observing{false};
  // Candidate is pre-entry: active_mission remains LINE until fresh 10/7.
  MissionType candidate_mission{MissionType::kNone};
  int candidate_phase{0}, candidate_search_turns{0}, candidate_restore_turns{0};
  bool line_plan_active{false};
  bool object_ready_mean_used{false}; // True only on the real object-action DONE.
  bool approach_locked{false}, approach_near{false};
  MissionType approach_mission{MissionType::kNone};
  std::uint64_t approach_track_id{};
  int approach_completed_blocks{}, approach_missing_frames{};
  int approach_sequence_hits{}, approach_sequence_frames{};
  bool approach_sequence_qualified{};
  std::size_t post_motion_frames{0};
  std::size_t post_motion_window_frames{0};
  int post_motion_valid{0};
  double post_motion_elapsed_sec{0.0};
  bool trial_complete{false};
  int test_stage{0};
  bool test_ready{false};
  bool test_executing{false};
  bool repeat_sequence{false};
  int repeat_scope{0};
  int ball_mission_index{1}, goal_mission_index{1};
  int recovery_turns_completed{0};
  bool recovery_failed{false};
  bool terminal_hold{false};
  bool recovery_verifying{false};
  int recovery_max_turns{0};
  int recovery_observation_frames{0};
  FootPose foot_pose{FootPose::kWalk};
  bool foot_transition{false};
  bool physical_reset_complete{false};
  int goal_walk_actions_completed{0}, goal_walk_actions_remaining{0};
  int goal_straight_blocks_completed{0};
  bool goal_five_step_completed{false}; // Legacy field name: true when the configured straight-block gate is reached.
  bool goal_walk_gate_active{false};
  BallResult ball;
  HurdleResult hurdle;
  GoalResult goal;
};

// 센서 adapter가 detection 하나에 부가할 수 있는 센서 의존 표본이다.
// depth 영상 해석은 ROS에 남기고, 거리/자세 계산은 core가 수행한다.
struct PerceptionDetection {
  Detection detection;
  std::optional<double> center_depth_m;
  std::optional<double> left_depth_m;
  std::optional<double> right_depth_m;
  std::optional<double> left_depth_u_px, right_depth_u_px;
  int depth_valid_pixels{};
};

struct PerceptionFrameInput {
  std::vector<PerceptionDetection> detections;
  Intrinsics intrinsics;
  bool enable_imu_rectification{false};
  bool imu_valid{false};
  double roll_rad{0.0};
  double pitch_rad{0.0};
  int image_width{0};
  int image_height{0};
  double now_sec{0.0};
  CameraFeedback camera_feedback;
  bool command_transport_enabled{false};
  ActionExecutionFeedback action_feedback;
  CommandDeliveryFeedback delivery_feedback;
  bool allow_new_line_action{true};
  bool allow_new_line_plan{true};
  bool allow_new_commands{true};
  bool allow_new_body_actions{true};
  // Only dedicated Stage START may execute setup pose actions without COMMIT.
  bool allow_test_setup_body_actions{false};
  bool allow_new_mission_entry{true};
  bool advance_line_fsm{true};
  // Manual PREVIEW triggers observation; AUTO default keeps post-DONE gates.
  bool manual_observation_control{false};
  bool begin_manual_observation{false};
  // Real 10-frame evidence accompanying a manual READY average.
  std::deque<bool> manual_object_hits, manual_pose_hits, manual_near_hits;
  std::uint64_t ball_target_id{0}; // Identity of a validated MANUAL READY mean.
  std::uint64_t hurdle_target_id{0}, backboard_target_id{0};
  // PerceptionFrameInput only: one synthetic 40%-to-READY object mean.
  bool manual_ready_mean{false};
  std::optional<LineGuide> line_decision_guide_override;
};

struct ObjectFilterDiagnostic {
  int class_id{};
  Point2 center;
  double aspect_ratio{};
  std::optional<double> depth_m;
  bool depth_passed{true}, aspect_passed{true};
  int depth_valid_pixels{};
};
struct PreparedPerceptionFrame {
  std::vector<Point2> raw_line_centers;
  std::vector<Point2> line_centers;
  // Center order is retained by IMU rectification; these map back to YOLO.
  std::vector<std::size_t> line_detection_indices;
  ObjectTargets targets;
  // center depth는 후보 검증과 접근 거리 판정에 항상 사용한다. 좌/우 depth
  // 기반 goal_pose는 미세조정 진입 거리 이후에만 유효해진다.
  GoalPoseObservation goal_pose;
  std::optional<double> ball_center_depth_m, hurdle_center_depth_m;
  std::optional<double> backboard_center_depth_m;
  std::optional<double> backboard_left_depth_m;
  std::optional<double> backboard_right_depth_m;
  std::optional<double> backboard_left_depth_u_px, backboard_right_depth_u_px;
  std::vector<BallCandidateTrack> ball_tracks;
  std::vector<BallCandidateTrack> hurdle_tracks, backboard_tracks;
  std::vector<ObjectFilterDiagnostic> object_filter_diagnostics;
  // Current-frame objects after class/confidence/box/aspect/depth filtering,
  // before identity association or temporal entry/stability votes.
  std::vector<Detection> accepted_object_detections;
  // Original-frame indices join filter-pass boxes to real tracker identities.
  std::vector<std::size_t> accepted_object_indices;
  std::vector<std::uint64_t> detection_track_ids;
  std::uint64_t ball_target_id{};
  std::uint64_t hurdle_target_id{}, backboard_target_id{};
  bool hurdle_target_locked{false}, backboard_target_locked{false};
  bool hurdle_reacquiring{false}, backboard_reacquiring{false};
  bool ball_target_locked{false}, ball_reacquiring{false};
  int ball_missing_frames{};
  std::string ball_tracking_reason;
  std::optional<double> ball_match_distance_px, ball_match_size_log_ratio;
  bool imu_rectification_applied{false};
};

struct PerceptionMissionFrameResult {
  MissionFrameResult mission;
  PreparedPerceptionFrame perception;
};

// 전체 미션의 유일한 진입점이다. LINE에서만 진입 후보를 관찰하고, 물체
// 미션이 잠긴 동안에는 해당 controller 하나만 실행한다.
class MissionController {
public:
  // 설치된 공통 algorithm YAML을 읽는다. 파일이나 필수 키가
  // 없으면 불완전한 0 설정으로 계속하지 않고 예외를 던진다.
  MissionController();
  explicit MissionController(const MissionControllerConfig &config);

  MissionFrameResult Step(const MissionFrameInput &input);
  PerceptionMissionFrameResult StepPerception(
      const PerceptionFrameInput &input);
  // Speculative preview never changes the real FSM or allocates a real ID.
  PerceptionMissionFrameResult PreviewPerception(
      const PerceptionFrameInput &input) const;
  // Update user-tunable decision parameters, retaining mission state.
  bool UpdateTuning(const MissionControllerConfig &candidate);
  MissionType ActiveMission() const { return active_mission_; }
  // Explicit manual PREVIEW honors the LINE FSM's 2s failure / 1s recovery
  // observation instead of adding an unrelated one-second shortcut.
  double ManualPreviewDurationSec() const;
  bool ManualObservationHasEnoughFrames() const {
    return !decision_observation_active_ || decision_observation_mission_==MissionType::kLine ||
        decision_object_hits_.size()>=9; // next processed frame completes the 10-frame window
  }
  bool ManualObservationCanFinish(double now_sec) const {
    return ManualObservationHasEnoughFrames() && (!decision_observation_active_ ||
        now_sec + 1e-9 >= decision_observation_start_sec_ + config_.post_motion_observation_sec);
  }
  int RepeatScope() const;
  bool NeedsStationaryObservation() const;
  bool SetTestMissionIndices(int ball_index, int goal_index);
  // Adapter calls only after physical body/camera DONE while disarmed/paused.
  // A sensory DONE may still be waiting for RESUME; preserve that pending edge.
  void SetGoalEdgeMargin(double margin_px);
  void SetBallFineSideTwoStepMargin(double margin_px);
  void SetLivePerceptionTuning(const MissionControllerConfig& candidate);
  bool SetTestStage(MissionType mission,int stage);
  bool TestSetupPending() const { return test_setup_pending_; }
  int MissionStateKey() const {
    return 10000 * static_cast<int>(active_mission_) +
        (active_mission_ == MissionType::kBall ? static_cast<int>(ball_result_.mode) :
         active_mission_ == MissionType::kGoal ? static_cast<int>(goal_result_.mode) :
         active_mission_ == MissionType::kHurdle ? static_cast<int>(hurdle_result_.mode) :
         candidate_.phase != CandidatePhase::kNone ? 1000 + static_cast<int>(candidate_.phase) :
         static_cast<int>(line_state_));
  }
  // Transport-only advancement: no observations, body requests, or synthetic DONE.
  ControlCommand AcceptDeliveryFeedback(const CommandDeliveryFeedback&,double now_sec);
  bool HasBall() const { return has_ball_; }
  bool IsTerminalHold() const {
    return test_failed_ || mission_recovery_failed_ || line_state_==LineState::kFinalHold ||
        (active_mission_==MissionType::kBall && ball_result_.mode==BallMode::kFailed) ||
        (active_mission_==MissionType::kGoal && goal_result_.mode==GoalMode::kFailed) ||
        (active_mission_==MissionType::kHurdle && hurdle_result_.mode==HurdleMode::kFailed);
  }
  bool FeetReadyToStart() const {
    const bool known_stance=foot_pose_==FootPose::kWalk ||
        (config_.test_mission!=MissionType::kNone && config_.test_stage>=2);
    return known_stance && !pose_change_pending_ && !physical_reset_pending_;
  }
  void BeginPhysicalReset();
  MissionFrameResult StepPhysicalReset(const CommandDeliveryFeedback &feedback,
                                      const CameraFeedback &camera, double now_sec);
  // Observed configured-start-to-READY LINE guide, for manual one-slot preview only.
  std::optional<LineGuide> LastReadyLineGuide() const { return last_ready_line_guide_; }
  // 진행 중 action/짧은 동작 후 관측이 없을 때만 LINE direct selector
  // gain/deadband를 상태 손실 없이 교체한다.
  bool UpdateLineP2pTuning(const LineP2pConfig &line_p2p);
  // Apply on the next START Reset; action IDs must not be recycled.
  void SetGoalTrialStartGoal(bool enabled) { config_.goal_trial_start_goal = enabled; }
  // Discard stale line/object evidence without resetting mission progress.
  void ClearObservationEvidence();
  void Reset(); // Retain the original public ABI for existing clients.
  void Reset(bool preserve_action_ids);

private:
  struct ApproachEvidence {
    std::uint64_t id{};
    std::optional<ObjectTarget> last;
    std::deque<bool> hits;
  };
  struct ContinuousApproach {
    MissionType mission{MissionType::kNone};
    std::uint64_t id{}, epoch{};
    int blocks{}, missing{}, blend{};
    bool locked{false}, near{false}, mission_started{false}, loss_pending{false};
    std::optional<ObjectTarget> last;
    GoalPoseObservation pose;
    std::optional<double> depth;
    LineGuide last_guide;
    bool near_observing{false}, near_qualified{false};
    double near_observation_started_sec{0.0}, near_sum_u{0.0}, near_sum_v{0.0};
    int near_valid_count{0}, near_attempts{0};
    std::deque<bool> near_observation_hits;
    ActionRequest near_motion;
    std::uint64_t near_motion_id{0};
    std::deque<bool> hits, good_hits, pose_hits, near_hits;
  } continuous_;
  std::optional<ContinuousApproach> recovering_approach_;
  struct RouteDecision {
    ActionRequest action;
    bool depth_recovery_step{false};
    std::optional<LineGuide> guide;
    CameraRequest camera{CameraRequest::kNone};
    bool hold{false}, entered{false}, object_visible{false}, near_final_action{false};
  };
  std::array<ApproachEvidence,3> approach_evidence_;
  std::uint64_t next_approach_epoch_{1};
  bool approach_goal_probe_{false}, approach_return_forward_{false};
  int approach_goal_empty_frames_{0}, approach_goal_retry_after_{0};
  int approach_goal_empty_retries_{0};
  bool approach_goal_search_failed_{false};
  bool approach_goal_depth_failed_{false};
  int approach_goal_bad_depth_frames_{0};
  std::uint64_t approach_goal_depth_action_id_{0};
  double approach_camera_started_{-1.0};
  RouteDecision PrepareContinuousApproach(const MissionFrameInput&,const Features&,bool idle);
  void ReleaseContinuousApproach();
  void EnterContinuousMission(const MissionFrameInput&,bool lost);
  bool ResumeContinuousApproach(const MissionFrameInput&);
  void PopulateApproachDiagnostics(MissionFrameResult&) const;
  bool PrepareGoalDepthRecovery(const MissionFrameInput&,bool visible,bool usable,bool idle,RouteDecision&);
  enum class LineState {
    kNormal,
    kFailureObserve,
    kRecoveryTurn,
    kRecoveryObserve,
    kFinalHold,
    kScoreRecovery, // WALK 31/28 recovery for an excessive valid O/H score.
  };

  MissionFrameResult StepWithLineImageCenter(
      const MissionFrameInput &input,
      std::optional<double> line_image_center_u,
      bool evaluating_ready_successor = false);
  ActionRequest ComputeLineAction(const LineGuide &guide,
                                  const MissionFrameInput &input,
                                  bool action_active,
                                  bool action_done,
                                  bool *reference_valid,
                                  bool append_stability = true);
  ActionRequest CruiseAction(const LineGuide &guide, bool queue_eligible);
  void BeginObservation(LineState state, double now_sec);
  void AddObservation(const LineGuide &guide);
  std::optional<LineGuide> FinishObservation() const;
  void BeginLineReacquisition();
  ActionRequest ComputeMissionLineRecovery(const MissionFrameInput& input,
      const ActionExecutionFeedback& feedback, bool* valid, MissionFrameResult* output);
  bool ShouldDeferDone(const CommandDeliveryFeedback &feedback) const;
  // Production object approaches: cache 40%-to-READY evidence without
  // starting an object successor/queue before the current DONE.
  void TrackMovingObjectReadyWindow(const MissionFrameInput &input);
  void BeginDecisionObservation(double now_sec, MissionType mission,
                                bool camera_only,
                                const CommandDeliveryFeedback &done);
  void AddDecisionObservation(const MissionFrameInput &input);
  bool DecisionObservationReady(double now_sec) const;
  std::optional<LineGuide> DecisionLineGuide() const;
  void EnterMission(MissionType mission);
  void FinishMission();
  bool StraightGoalGateReached(int additional_straight_blocks = 0) const;
  void PopulateDiagnostics(MissionFrameResult &output, double now_sec) const;
  void InitializeTest();
  bool TestExecutionReady() const;
  bool TestBoundaryReached(const CameraFeedback&) const;

  MissionFrameResult StepPlannedLine(const MissionFrameInput& input,
                                    std::optional<double> image_center);
  void ObserveLineFeedback(const CommandDeliveryFeedback& feedback, double now);
  void BeginCandidateObservation(double now);
  void ReleaseCandidate();
  enum class CandidatePhase { kNone, kDrain, kObserve, kSearch, kRestore };
  struct Candidate {
    MissionType mission{MissionType::kNone};
    CandidatePhase phase{CandidatePhase::kNone};
    std::optional<double> direction_u;
    std::optional<bool> search_left;
    int searches{0}, restores{0};
    std::uint64_t turn_id{0};
    bool turn_is_restore{false};
    bool observe_after_turn{false};
  } candidate_;
  std::uint64_t line_ready_handled_id_{0};
  std::uint64_t line_auxiliary_id_{0};
  bool line_auxiliary_done_{false};
  MissionAction line_auxiliary_action_{MissionAction::kNone};
  bool line_auxiliary_observing_{false};
  MissionControllerConfig config_;
  LinePlan line_plan_;
  bool goal_trial_complete_{false}; // Shared trial completion, retained name for compatibility.
  bool test_entry_pending_{false}, test_setup_pending_{false}, test_execution_started_{false};
  bool fresh_after_search_{false};
  bool decision_observation_restart_pending_{false};
  int line_guard_turns_completed_{0};
  bool test_setup_camera_started_{false}, test_failed_{false};
  bool test_setup_pose_done_{false};
  bool goal_start_camera_pending_{false};
  double test_setup_camera_start_sec_{0};
  FootPose foot_pose_{FootPose::kWalk};
  bool pose_change_pending_{false};
  bool pose_observation_pending_{false};
  FootPose pose_target_{FootPose::kWalk};
  double pose_started_sec_{0.0};
  bool mission_finish_pending_{false};
  bool physical_reset_pending_{false}, reset_walk_done_{false};
  bool reset_camera_command_sent_{false};
  int ball_mission_index_{0}, goal_mission_index_{0};
  int mission_recovery_turns_{0};
  bool mission_recovery_issued_{false};
  bool mission_recovery_failed_{false};
  bool mission_recovery_found_{false};
  bool mission_recovery_verifying_{false};
  bool mission_recovery_qualified_{false};
  double mission_recovery_verify_start_sec_{0.0};
  bool post_pickup_goal_line_gate_active_{false};
  int post_pickup_line_actions_issued_{0};
  int post_pickup_line_actions_completed_{0};
  int post_pickup_straight_blocks_completed_{0};
  bool post_pickup_five_step_completed_{false};
  std::unordered_map<std::uint64_t, MissionAction> post_pickup_line_actions_;
  std::deque<bool> mission_recovery_hits_;
  double decision_object_weight_{0.0};
  // Width for renormalizing cached LINE offsets after optical reference edits.
  int last_line_image_width_{0};
  double last_line_image_center_u_{0.0};
  LineP2pController line_p2p_controller_;
  LineGuideAccumulator line_guide_accumulator_;
  struct MovingObjectSample {
    double now_sec{0.0};
    std::optional<ObjectTarget> target;
    std::optional<double> center_depth_m;
    GoalPoseObservation pose;
  };
  struct MovingObjectReadyMean {
    std::uint64_t action_id{0};
    MissionType mission{MissionType::kNone};
    CameraMode camera{CameraMode::kForward};
    ObjectTarget target;
    std::optional<double> center_depth_m;
    GoalPoseObservation pose;
    std::deque<bool> hits, pose_hits, near_hits;
  };
  std::uint64_t moving_object_action_id_{0};
  CameraMode moving_object_camera_{CameraMode::kForward};
  std::vector<MovingObjectSample> moving_object_samples_;
  std::optional<MovingObjectReadyMean> moving_object_ready_mean_;
  // Keep queued B's identity even if transport accepts both A DONE and
  // B DONE before the next camera frame. Never dispatch B twice.
  std::uint64_t object_queued_parent_id_{0};
  std::optional<ControlCommand> object_queued_successor_;
  std::optional<LineGuide> last_ready_line_guide_;
  std::uint64_t ready_line_action_id_{0};
  bool line_failure_pending_done_{false};
  LineState line_state_{LineState::kNormal};
  std::optional<double> last_line_score_;
  bool line_recovery_uses_visible_centers_{false};
  bool line_recovery_left_{false};
  double line_observation_start_sec_{0.0};
  bool line_observation_active_{false};
  double observation_offset_sum_{0.0};
  double observation_heading_sum_{0.0};
  double observation_confidence_sum_{0.0};
  double observation_curvature_sum_{0.0};
  int observation_valid_count_{0};
  int observation_curvature_count_{0};
  std::deque<bool> line_observation_hits_;
  bool line_observation_qualified_{false};
  int line_visible_recovery_turns_{0};
  int line_last_score_recovery_turns_{0};
  int line_no_evidence_observations_{0};
  bool recovery_turn_issued_{false};
  std::deque<bool> line_stability_history_;
  bool decision_observation_active_{false};
  bool decision_observation_camera_only_{false};
  bool decision_observation_first_frame_{false};
  MissionType decision_observation_mission_{MissionType::kLine};
  CommandDeliveryFeedback deferred_done_;
  double decision_observation_start_sec_{0.0};
  double settle_until_sec_{0.0};
  std::uint64_t last_settle_done_id_{0};
  std::deque<bool> decision_object_hits_;
  std::deque<bool> decision_qualified_object_hits_;
  bool decision_object_qualified_{false};
  std::deque<bool> decision_goal_pose_hits_;
  std::deque<bool> decision_qualified_goal_pose_hits_;
  bool decision_goal_pose_qualified_{false};
  std::deque<bool> decision_goal_near_hits_;
  std::size_t decision_observation_total_frames_{0};
  // Raw (already associated) object samples for the 1s stationary mean.
  // Ball/hurdle use bounded later-weighted coordinates. Goal keeps its pose
  // estimator; stationary means replace prior movement evidence.
  std::optional<ObjectTarget> decision_last_target_;
  double decision_target_u_sum_{0.0}, decision_target_v_sum_{0.0};
  double decision_rect_u_sum_{0.0}, decision_rect_v_sum_{0.0};
  int decision_target_count_{0};
  double decision_pose_x_sum_{0.0}, decision_pose_z_sum_{0.0};
  double decision_pose_sin_sum_{0.0}, decision_pose_cos_sum_{0.0};
  int decision_pose_count_{0};
  int decision_line_valid_count_{0};
  std::deque<bool> decision_line_hits_;
  bool decision_line_qualified_{false};
  double decision_line_offset_sum_{0.0};
  double decision_line_heading_sum_{0.0};
  double decision_line_confidence_sum_{0.0};
  bool previous_camera_settled_{true};
  CameraMode previous_camera_mode_{CameraMode::kForward};
  BallController ball_controller_;
  HurdleController hurdle_controller_;
  GoalController goal_controller_;
  ControlCommandCoordinator command_coordinator_;
  // 서로 다른 클래스의 identity가 섞이지 않도록 상태를 분리한다.
  ObjectAssociationTracker ball_association_tracker_;
  BallCandidateTracker ball_candidate_tracker_;
  std::uint64_t ball_target_id_{0};
  bool ball_target_locked_{false}, ball_reacquiring_{false};
  bool ball_recovery_was_active_{false};
  ObjectTargetTracker backboard_target_tracker_, hurdle_target_tracker_;
  MissionType active_mission_{MissionType::kLine};
  bool has_ball_{false};
  bool object_tracking_camera_moving_{false};
  CameraMode object_tracking_camera_mode_{CameraMode::kForward};
  BallResult ball_result_;
  HurdleResult hurdle_result_;
  GoalResult goal_result_;
  // delivery feedback을 해당 Mission action에만 라우팅하기 위한 직전 출력이다.
  ControlCommand last_command_;
};

} // namespace vision_core
