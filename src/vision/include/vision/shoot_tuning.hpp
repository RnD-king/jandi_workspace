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
