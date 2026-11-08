#include "camera_overlay/goal_measurement.hpp"
#include "camera_overlay/goal_measurement_view.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <limits>

int main() {
  camera_overlay::GoalMeasurementConfig c;
  c.backboard_depth_strip_width_px=5;
  c.backboard_min_depth_m=.1;c.backboard_max_depth_m=2.5;c.hoop_radius_m=.1;
  const camera_overlay::Intrinsics k{600,600,320,240};
  const auto point=camera_overlay::ProjectGoalPointXZ(380,1.5,k);
  assert(point.valid && std::abs(point.x_m-.15)<1e-12 && point.z_m==1.5);
  assert(!camera_overlay::ProjectGoalPointXZ(380,0,k).valid);
  assert(!camera_overlay::ProjectGoalPointXZ(380,1.5,{}).valid);
  assert(!camera_overlay::EstimateGoalRimCenter({},.1).valid);
  const cv::Point pixel(380,240);
  cv::Mat depth(480,640,CV_16UC1,cv::Scalar(1000));
  auto m=camera_overlay::MeasureGoalAtClick(depth,pixel,k,c);
  assert(m.point.valid && std::abs(m.point.x_m-.1)<1e-12 && m.point.z_m==1);
  assert(m.left_u==350 && m.right_u==410);
  assert(m.center_roi==cv::Rect(379,239,3,3));
  assert(m.left_roi==cv::Rect(348,238,5,5) && m.right_roi==cv::Rect(408,238,5,5));
  assert(m.board.valid && m.board.yaw_rad==0 && m.rim.valid);
  assert(std::abs(m.rim.x_m-.1)<1e-12 && std::abs(m.rim.z_m-.9)<1e-12);

  // Derive expected yaw from the independently constructed physical tangent.
  for (int sign : {-1,1}) {
    cv::Mat tilted(480,640,CV_32FC1,cv::Scalar(1));
    tilted(m.left_roi).setTo(1+sign*.05);
    tilted(m.right_roi).setTo(1-sign*.05);
    const auto result=camera_overlay::MeasureGoalAtClick(tilted,pixel,k,c);
    const double lz=static_cast<double>(tilted.at<float>(240,350));
    const double rz=static_cast<double>(tilted.at<float>(240,410));
    const double lx=(350-320)*lz/600,rx=(410-320)*rz/600;
    const double phi=std::atan2(lz-rz,rx-lx);
    assert(result.board.valid && result.rim.valid);
    assert(std::abs(result.board.yaw_rad-phi)<1e-12);
    assert(std::abs(result.rim.x_m-(.1-.1*std::sin(phi)))<1e-12);
    assert(std::abs(result.rim.z_m-(1-.1*std::cos(phi)))<1e-12);
    assert(sign>0 ? result.rim.x_m<.1 : result.rim.x_m>.1);
  }
  // Point 5x5 and production GOAL 3x3 must not reuse each other's depth.
  depth(cv::Rect(378,238,5,5)).setTo(2000);
  depth(m.center_roi).setTo(1000);
  auto separate=camera_overlay::MeasureGoalAtClick(depth,pixel,k,c);
  assert(separate.point_depth.median_m==2 && separate.point.z_m==2);
  assert(separate.board.valid && separate.board.z_m==1);

  auto missing_info=camera_overlay::MeasureGoalAtClick(depth,pixel,std::nullopt,c);
  assert(missing_info.point_depth.median_m && !missing_info.point.valid && !missing_info.rim.valid);
  depth(m.left_roi).setTo(0);
  auto missing_band=camera_overlay::MeasureGoalAtClick(depth,pixel,k,c);
  assert(missing_band.point.valid && !missing_band.board.valid && !missing_band.rim.valid);
  depth.setTo(3000);
  auto out_of_range=camera_overlay::MeasureGoalAtClick(depth,pixel,k,c);
  assert(out_of_range.point.valid && !out_of_range.rim.valid);
  depth.setTo(1000);
  auto edge=camera_overlay::MeasureGoalAtClick(depth,{5,240},k,c);
  assert(edge.point.valid && !edge.rim.valid);

  // Same median rule as GOAL: out-of-range/NaN samples are invalid.
  // Four valid samples use the upper median.
  cv::Mat metres(480,640,CV_32FC1,cv::Scalar(1));
  metres(m.left_roi).setTo(0);
  metres.at<float>(238,348)=.03f;
  metres.at<float>(238,349)=std::numeric_limits<float>::quiet_NaN();
  metres.at<float>(239,348)=.5f;metres.at<float>(239,349)=.6f;
  metres.at<float>(240,348)=.7f;metres.at<float>(240,349)=.8f;
  const auto upper=camera_overlay::MeasureGoalAtClick(metres,pixel,k,c);
  assert(upper.left_stats.valid==4 && upper.left_depth_m);
  assert(*upper.left_depth_m==static_cast<double>(.7f));
  // Rendering must tolerate every diagnostic validity state without a GUI.
  for (const auto& state : {m,separate,missing_info,missing_band,out_of_range,edge}) {
    const auto view=camera_overlay::DrawGoalMeasurementTopView(state,c.hoop_radius_m);
    assert(!view.empty() && view.cols==760 && view.rows==620);
  }
  assert(!camera_overlay::DrawGoalMeasurementTopView(std::nullopt,c.hoop_radius_m).empty());
}
