#pragma once

#include <cstdint>

namespace vision_core {

struct Point2 {
  // OpenCV pixel convention: u is positive to image-right and v is positive
  // downward.
  double u{0.0};
  double v{0.0};
};

struct Intrinsics {
  double fx{600.0};
  double fy{600.0};
  double cx{320.0};
  double cy{240.0};
};

struct FeatureConfig {
  int max_centers{};
  // O/H guide는 최소 3개의 유한 line center를 요구한다.
  int guide_min_points{};
  double image_center_u{};
  int curve_min_points{};
  double curve_min_v_span_px{};
  int curve_local_fit_points{};
  double guide_fit_rmse_full_scale_px{};
};

// P2P 모션 판단에 필요한 최소 라인 기하 표현이다.
struct LineGuide {
  // 모든 signed 값은 image-right가 양수다.
  // 가까운 점군 fitting의 대표 위치를 영상 중심으로 정규화한 값.
  double offset{0.0};
  // 가까운 점군에서 화면 위(진행 방향)로 향하는 방향각.
  double heading_rad{0.0};
  // 먼 점군 방향각 - 가까운 점군 방향각. 오른쪽 커브가 양수다.
  double curvature_rad{0.0};
  // 가까운 점 개수, 세로 분포, near fit 잔차를 합친 0..1 O/H 신뢰도.
  // far fit 기반 curvature 진단 유효성과는 독립적이다.
  double confidence{0.0};
  bool valid{false};
  // curvature는 먼 점군 fit까지 성공했을 때만 유효하다. 정상 P2P LINE
  // 조향은 이 값과 무관하게 offset/heading validity만 사용한다.
  bool curvature_valid{false};
};

struct Features {
  double n_visible{0.0};
  LineGuide guide;
};

struct LineTrackingConfig {
  int line_stable_window{};
  int line_stable_min_hits{};
};

struct Pose2 {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Box2 {
  double x{0.0};
  double y{0.0};
  double width{0.0};
  double height{0.0};
};

struct Detection {
  Box2 box;
  double confidence{0.0};
  int class_id{0};
};

struct ObjectTarget {
  int class_id{0};
  double confidence{0.0};
  Box2 box_px;
  Point2 center_px;
  Point2 rectified_center_px;
  double width_px{0.0};
  double height_px{0.0};
  double area_px{0.0};
  bool center_rectified{false};
};

enum class CommandType {
  kNone = 0,
  kAction = 2,
  kHold = 3,
};

enum class MissionType {
  kNone = 0,
  kLine = 1,
  kBall = 2,
  kGoal = 3,
  kHurdle = 4,
};

enum class MissionAction {
  kNone = 0,
  // ROS ActionCommand.msg와 공유하는 단일 action 번호 규약.
  kDefaultPosition = 1,
  kDefaultPoseMode = 2,
  kStepForwardHalf = 3,
  kStepBack = 4,
  kLeftSideStep = 5,
  kRightSideStep = 6,
  kTurnLeft = 7,
  kTurnRight = 8,
  kWalkMode = 9,
  kStepForwardOne = 10,
  kStepForwardLeft = 11,
  kStepForwardRight = 12,
  kStepForwardFive = 13,
  kPickBall = 14,
  kRecatch = 15,
  kHurdle = 16,
  kShoot = 17,
  kTurnLeftAndStep = 18,
  kTurnRightAndStep = 19,
  // 실행기 mapping 준비 전까지 protocol에만 예약한다. 현재 HURDLE FSM은
  // STEP_FORWARD_ONE(10)을 contact action으로 사용한다.
  kContactWalk = 20,
};

enum class CruiseDirection {
  kNone = 0,
  kLeft = 1,
  kStraight = 2,
  kRight = 3,
};

// applicable은 현재 mission phase가 direct cruise 대상인지를 나타낸다.
// applicable=true, direction=NONE은 quantizer fallback이 아니라 HOLD다.
struct CruiseDecision {
  bool applicable{false};
  CruiseDirection direction{CruiseDirection::kNone};
  double error{0.0};
  double deadband{0.0};
};

// action 숫자는 하나의 ROS 토픽으로 전달하지만, 생성 원인과 DONE 처리 규칙은
// 분리한다. Mission은 상태 전이용 단발 동작이고 Locomotion은 반복 가능한
// 보행 primitive다.
enum class ActionCategory {
  kNone = 0,
  kMission = 1,
  kLocomotion = 2,
};

enum class ControlPhase {
  kMission = 0,
  kWaitingActionAck = 2,
  kWaitingActionDone = 3,
  kWaitingQueuedActionAck = 4,
  kWaitingQueuedActionStart = 5,
  // ACTION 발행 뒤 설정 시간 안에 ACK를 받지 못해 재실행을 금지한 정지 상태다.
  // 늦은 ACK로 같은 동작이 중복 실행되는 것을 막기 위해 Reset 전까지 유지한다.
  kActionAckTimedOut = 6,
};

enum class CameraMode {
  kForward = 0,
  kDown = 1,
  kTransition = 2,
  kGoal = 3,
};

enum class CameraRequest {
  kNone = 0,
  kDown = 1,
  kForward = 2,
  kGoal = 3,
};

struct CameraFeedback {
  CameraMode actual_mode{CameraMode::kForward};
  bool settled{true};
};

// MissionController가 직전 command와 ACK/DONE으로 만든 실행 상태다.
struct ActionExecutionFeedback {
  bool action_done{false};
  bool action_active{false};
};

struct ActionRequest {
  MissionAction action{MissionAction::kNone};
  ActionCategory category{ActionCategory::kNone};
  // TURN은 action이 방향을 나타내므로 양수 magnitude, SHOOT만 signed다.
  std::int16_t target_yaw_deg{0};
  bool line_queue_eligible{false};
};

struct ControlCommand {
  CommandType command_type{CommandType::kHold};
  MissionType mission{MissionType::kLine};
  int mission_phase{0};
  ControlPhase control_phase{ControlPhase::kMission};
  MissionAction action{MissionAction::kNone};
  ActionCategory action_category{ActionCategory::kNone};
  std::uint64_t action_id{0};
  std::int16_t target_yaw_deg{0};
  CameraRequest camera_request{CameraRequest::kNone};
};

} // namespace vision_core
