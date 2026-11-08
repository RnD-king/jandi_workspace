#pragma once

#include "camera_overlay/depth_measurement.hpp"
#include "camera_overlay/goal_geometry.hpp"

#include <cmath>
#include <optional>
#include <string>

namespace camera_overlay {

struct GoalMeasurement {
  DepthMeasurement point_depth;
  GoalPointXZ point;
  cv::Rect center_roi,left_roi,right_roi;
  DepthRegionPixelStats center_stats,left_stats,right_stats;
  std::optional<double> center_depth_m,left_depth_m,right_depth_m;
  double left_u{0},right_u{0};
  GoalPointXZ left,right,rim;
  GoalPoseObservation board;
  std::string status;
};

inline GoalMeasurement MeasureGoalAtClick(const cv::Mat& depth,cv::Point pixel,
    const std::optional<Intrinsics>& intrinsics,
    const GoalMeasurementConfig& config) {
  GoalMeasurement m;
  m.point_depth=MeasureDepth5x5(depth,pixel);
  if (!m.point_depth.supported) {m.status="NO SUPPORTED DEPTH";return m;}
  if (intrinsics && m.point_depth.median_m)
    m.point=ProjectGoalPointXZ(pixel.x,*m.point_depth.median_m,*intrinsics);

  // The test has no detected bounding box. Treat u-30/u+30 as the
  // nominal band centres and use a five-pixel-high strip on this scanline.
  // GOAL itself samples full detected-box height; its central patch is 3x3.
  constexpr int offset=30,strip_height=5;
  const int width=config.backboard_depth_strip_width_px;
  if (width<1 || width>=2*offset) {m.status="INVALID/OVERLAPPING BAND WIDTH";return m;}
  const cv::Rect bounds(0,0,depth.cols,depth.rows);
  const cv::Rect center(pixel.x-1,pixel.y-1,3,3);
  const cv::Rect left(pixel.x-offset-width/2,pixel.y-strip_height/2,width,strip_height);
  const cv::Rect right(pixel.x+offset-width/2,pixel.y-strip_height/2,width,strip_height);
  m.center_roi=center & bounds;m.left_roi=left & bounds;m.right_roi=right & bounds;
  m.left_u=left.x+.5*(left.width-1);
  m.right_u=right.x+.5*(right.width-1);
  // Local copies of GOAL's upper-median and depth validity rules.
  m.center_depth_m=MedianRegionMeters(depth,m.center_roi);
  m.left_depth_m=MedianRegionMeters(depth,m.left_roi);
  m.right_depth_m=MedianRegionMeters(depth,m.right_roi);
  m.center_stats=CountValidDepthPixels(depth,m.center_roi);
  m.left_stats=CountValidDepthPixels(depth,m.left_roi);
  m.right_stats=CountValidDepthPixels(depth,m.right_roi);
  if (intrinsics) {
    if (m.left_depth_m && m.left_roi==left)
      m.left=ProjectGoalPointXZ(m.left_u,*m.left_depth_m,*intrinsics);
    if (m.right_depth_m && m.right_roi==right)
      m.right=ProjectGoalPointXZ(m.right_u,*m.right_depth_m,*intrinsics);
  }
  if (!intrinsics) {m.status="NO MATCHING CAMERA INFO";return m;}
  if (m.center_roi!=center || m.left_roi!=left || m.right_roi!=right) {
    m.status="BAND/PATCH OUTSIDE IMAGE";return m;
  }
  if (!m.center_depth_m || !m.left_depth_m || !m.right_depth_m) {
    m.status="MISSING GOAL CENTER/LEFT/RIGHT DEPTH";return m;
  }
  // Same center-depth gate as MissionController::StepPerception.
  if (*m.center_depth_m<config.backboard_min_depth_m ||
      *m.center_depth_m>config.backboard_max_depth_m) {
    m.status="CENTER OUTSIDE GOAL DEPTH RANGE";return m;
  }
  m.board=EstimateGoalPoseFromBackboardDepths(pixel.x,*m.center_depth_m,
      m.left_u,*m.left_depth_m,m.right_u,*m.right_depth_m,*intrinsics);
  m.rim=EstimateGoalRimCenter(m.board,config.hoop_radius_m);
  m.status=m.rim.valid ? "MANUAL BOARD ASSUMPTION" : "INVALID GOAL GEOMETRY";
  return m;
}

}  // namespace camera_overlay
