#pragma once

#include "vision/mission_debug_view.hpp"
#include "vision_core/control_command.hpp"
#include <optional>

namespace vision {

enum class ShootRequestUnavailable { kNone,kDisconnected,kNoAck };

// Drop only this bench's transport wait. This is neither a motor stop command
// nor fabricated ACK/DONE. Preserve IDs so a later click is a new transaction.
inline ShootRequestUnavailable ReleaseUnavailableShootRequest(
    vision_core::ControlCommandCoordinator& transport,bool executor_connected) {
  if (!transport.HasPendingAction()) return ShootRequestUnavailable::kNone;
  const auto reason=!executor_connected ? ShootRequestUnavailable::kDisconnected :
      transport.ActionAckTimedOut() ? ShootRequestUnavailable::kNoAck : ShootRequestUnavailable::kNone;
  if (reason!=ShootRequestUnavailable::kNone) transport.Reset(true);
  return reason;
}

struct ClickedBackboard {
  std::optional<double> depth_m;
  int valid_pixels{0}, total_pixels{0};
  vision_core::GoalPoseObservation board_pose;
  GoalLandingDebug landing;
};

// A stored target is already the rim center, not the backboard center.
// Keeping this separate prevents subtracting the hoop radius a second time
// when a clicked or manually entered target is passed to the production planner.
struct GoalCenterTarget {
  double x_m{},y_m{};
  bool Valid() const {
    return std::isfinite(x_m) && std::isfinite(y_m) && y_m>0;
  }
};

inline std::optional<GoalCenterTarget> GoalCenterFromLanding(const GoalLandingDebug& landing) {
  const GoalCenterTarget target{landing.rim_x_m,landing.rim_z_m};
  if (!landing.rim_valid || !target.Valid()) return {};
  return target;
}

inline vision_core::ActionRequest PlanStoredGoalCenter(
    const GoalCenterTarget& target,const vision_core::GoalConfig& config,
    double camera_motion_timeout_sec) {
  if (!target.Valid() || !std::isfinite(config.hoop_radius_m) || config.hoop_radius_m<0)
    return {};
  // FineAction accepts a board pose and subtracts the radius. This equivalent
  // front-facing board pose reconstructs the saved rim exactly, even if the
  // radius was edited after saving the target. No image/depth is consulted.
  const vision_core::GoalPoseObservation pose{
      true,target.x_m,target.y_m+config.hoop_radius_m,0,1};
  const vision_core::GoalController planner(config,camera_motion_timeout_sec);
  return planner.PlanFineAction(pose);
}

// An isolated shoot preview never schedules a walking correction. It snapshots
// a rim-center target and the best *integer* waist angle, using the same
// physical waist pivot and tie-breaking contract as GOAL FineAction.
struct GoalShotPreview {
  int yaw_deg{0};
  double landing_x_m{0}, landing_z_m{0}, position_error_m{0};
  bool within_tolerance{false};
};
inline std::optional<GoalShotPreview> PreviewGoalShot(
    const GoalCenterTarget& target,const vision_core::GoalConfig& config) {
  if (!target.Valid() ||
      !std::isfinite(config.throw_landing_x_m) ||
      !std::isfinite(config.throw_landing_z_m) ||
      !std::isfinite(config.waist_axis_in_camera_x_m) ||
      !std::isfinite(config.waist_axis_in_camera_z_m) ||
      !std::isfinite(config.position_tolerance_m) ||
      !std::isfinite(config.shoot_yaw_limit_deg) ||
      config.position_tolerance_m<0 || config.shoot_yaw_limit_deg<0)
    return std::nullopt;
  const int limit=static_cast<int>(
      std::floor(std::min(180.0,config.shoot_yaw_limit_deg)));
  GoalShotPreview best;
  double error=std::numeric_limits<double>::infinity();
  const double wx=config.waist_axis_in_camera_x_m;
  const double wz=config.waist_axis_in_camera_z_m;
  const double dx=config.throw_landing_x_m-wx;
  const double dz=config.throw_landing_z_m-wz;
  for (int deg=-limit;deg<=limit;++deg) {
    const double rad=deg*kDebugPi/180.0;
    const double x=wx+std::cos(rad)*dx-std::sin(rad)*dz;
    const double z=wz+std::sin(rad)*dx+std::cos(rad)*dz;
    const double candidate=std::hypot(target.x_m-x,target.y_m-z);
    if (candidate<error-1e-12 ||
        (std::abs(candidate-error)<=1e-12 && std::abs(deg)<std::abs(best.yaw_deg))) {
      error=candidate;
      best.yaw_deg=deg;
      best.landing_x_m=x;
      best.landing_z_m=z;
      best.position_error_m=candidate;
    }
  }
  if (!std::isfinite(error)) return std::nullopt;
  best.within_tolerance=error<=config.position_tolerance_m;
  return best;
}

// Sampling alone does not replace the stored result. The UI explicitly adopts
// the sampled rim center, with the yaw assumption captured at click time.
inline ClickedBackboard SampleClickedBackboard(
    const cv::Mat& depth, int u, int v,
    const vision_core::Intrinsics& intrinsics,
    const vision_core::GoalConfig& config, double board_yaw_deg) {
  ClickedBackboard result;
  if (depth.empty() || u<0 || v<0 || u>=depth.cols || v>=depth.rows ||
      (depth.type()!=CV_16UC1 && depth.type()!=CV_32FC1)) return result;
  const auto roi=cv::Rect(u-2,v-2,5,5) & cv::Rect(0,0,depth.cols,depth.rows);
  result.total_pixels=roi.area();
  std::vector<double> values;
  for (int y=roi.y;y<roi.y+roi.height;++y)
    for (int x=roi.x;x<roi.x+roi.width;++x) {
      const double z=depth.type()==CV_16UC1
          ? 0.001*depth.at<std::uint16_t>(y,x) : depth.at<float>(y,x);
      if (std::isfinite(z) && z>0) values.push_back(z);
    }
  result.valid_pixels=static_cast<int>(values.size());
  if (values.empty()) return result;
  std::sort(values.begin(),values.end());
  const auto n=values.size();
  result.depth_m=n%2 ? values[n/2] : (values[n/2-1]+values[n/2])*0.5;
  if (!std::isfinite(intrinsics.fx) || intrinsics.fx<=0 ||
      !std::isfinite(intrinsics.cx) || !std::isfinite(board_yaw_deg)) return result;
  vision_core::MissionFrameResult frame;
  frame.active_mission=vision_core::MissionType::kGoal;
  frame.goal.fine_decision_pose_used=true;
  frame.goal.fine_decision_pose={true,(u-intrinsics.cx)*(*result.depth_m)/intrinsics.fx,
      *result.depth_m,board_yaw_deg*kDebugPi/180.0,1.0};
  result.board_pose=frame.goal.fine_decision_pose;
  result.landing=ComputeGoalLandingDebug(config,frame);
  return result;
}

} // namespace vision
