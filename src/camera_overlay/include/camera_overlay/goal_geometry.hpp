#pragma once

#include <opencv2/core.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <vector>

namespace camera_overlay {

// Standalone measurement implementation copied from GOAL on 2026-10-22.
// References only: vision_core/src/goal_controller.cpp,
// vision_core/config/vision_algorithm.yaml, vision/backboard_depth.hpp.
// No headers, libraries or configuration files from those packages are used.
struct Intrinsics {
  double fx{0},fy{0},cx{0},cy{0};
};
struct GoalPointXZ {
  bool valid{false};
  double x_m{0},z_m{0};
};
struct GoalPoseObservation {
  bool valid{false};
  double x_m{0},z_m{0},yaw_rad{0};
};
struct GoalMeasurementConfig {
  int backboard_depth_strip_width_px{5};
  double backboard_min_depth_m{.10};
  double backboard_max_depth_m{2.50};
  double hoop_radius_m{.10};
};

inline void ValidateGoalMeasurementConfig(const GoalMeasurementConfig& c) {
  if (c.backboard_depth_strip_width_px<1 || c.backboard_depth_strip_width_px>=60)
    throw std::invalid_argument("backboard_depth_strip_width_px must be in [1,59]");
  if (!std::isfinite(c.hoop_radius_m) || c.hoop_radius_m<0 ||
      !std::isfinite(c.backboard_min_depth_m) || c.backboard_min_depth_m<=0 ||
      !std::isfinite(c.backboard_max_depth_m) || c.backboard_max_depth_m<c.backboard_min_depth_m)
    throw std::invalid_argument("Invalid hoop radius or backboard depth range");
}

inline GoalPointXZ ProjectGoalPointXZ(double u,double depth,const Intrinsics& k) {
  if (!std::isfinite(u) || !std::isfinite(depth) || depth<=0 ||
      !std::isfinite(k.fx) || std::abs(k.fx)<=1e-9 || !std::isfinite(k.cx)) return {};
  const double x=(u-k.cx)*depth/k.fx;
  return std::isfinite(x) ? GoalPointXZ{true,x,depth} : GoalPointXZ{};
}

inline GoalPoseObservation EstimateGoalPoseFromBackboardDepths(
    double center_u,double center_z,double left_u,double left_z,
    double right_u,double right_z,const Intrinsics& k) {
  const auto center=ProjectGoalPointXZ(center_u,center_z,k);
  const auto left=ProjectGoalPointXZ(left_u,left_z,k);
  const auto right=ProjectGoalPointXZ(right_u,right_z,k);
  if (!center.valid || !left.valid || !right.valid) return {};
  const double width=right.x_m-left.x_m;
  if (!std::isfinite(width) || width<=1e-9) return {};
  return {true,center.x_m,center.z_m,std::atan2(left_z-right_z,width)};
}

inline GoalPointXZ EstimateGoalRimCenter(const GoalPoseObservation& b,double radius) {
  if (!b.valid || !std::isfinite(b.x_m) || !std::isfinite(b.z_m) || b.z_m<=0 ||
      !std::isfinite(b.yaw_rad) || !std::isfinite(radius) || radius<0) return {};
  const double x=b.x_m-radius*std::sin(b.yaw_rad);
  const double z=b.z_m-radius*std::cos(b.yaw_rad);
  return std::isfinite(x) && std::isfinite(z) ? GoalPointXZ{true,x,z} : GoalPointXZ{};
}

inline bool ValidGoalDepthSample(double z) {
  return std::isfinite(z) && z>.05 && z<10.0;
}
struct DepthRegionPixelStats {
  int valid{0},total{0};
};
inline DepthRegionPixelStats CountValidDepthPixels(const cv::Mat& depth,const cv::Rect& raw) {
  DepthRegionPixelStats stats;
  if (depth.empty() || (depth.type()!=CV_16UC1 && depth.type()!=CV_32FC1)) return stats;
  const auto box=raw & cv::Rect(0,0,depth.cols,depth.rows);
  stats.total=box.area();
  for (int y=box.y;y<box.y+box.height;++y) for (int x=box.x;x<box.x+box.width;++x) {
    const double z=depth.type()==CV_16UC1 ? depth.at<std::uint16_t>(y,x)*.001 : depth.at<float>(y,x);
    if (ValidGoalDepthSample(z)) ++stats.valid;
  }
  return stats;
}
inline std::optional<double> MedianRegionMeters(const cv::Mat& depth,const cv::Rect& raw) {
  if (depth.empty() || (depth.type()!=CV_16UC1 && depth.type()!=CV_32FC1)) return {};
  const auto box=raw & cv::Rect(0,0,depth.cols,depth.rows);
  std::vector<double> values;
  values.reserve(box.area());
  for (int y=box.y;y<box.y+box.height;++y) for (int x=box.x;x<box.x+box.width;++x) {
    const double z=depth.type()==CV_16UC1 ? depth.at<std::uint16_t>(y,x)*.001 : depth.at<float>(y,x);
    if (ValidGoalDepthSample(z)) values.push_back(z);
  }
  if (values.empty()) return {};
  auto middle=values.begin()+values.size()/2;
  std::nth_element(values.begin(),middle,values.end());
  return *middle;
}

}  // namespace camera_overlay
