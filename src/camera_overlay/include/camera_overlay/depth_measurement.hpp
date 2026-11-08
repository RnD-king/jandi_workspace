#pragma once

#include <opencv2/core.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace camera_overlay {

struct DepthMeasurement {
  cv::Point pixel;
  cv::Rect roi;
  bool supported{false};
  bool clipped{false};
  std::size_t valid_pixels{0};
  std::optional<double> median_m;
};

// RealSense aligned 16UC1 is millimetres; ROS 32FC1 depth is metres.
// Return optical-axis depth Z, not Euclidean distance from the camera.
inline DepthMeasurement MeasureDepth5x5(const cv::Mat& depth, cv::Point pixel) {
  DepthMeasurement result;
  result.pixel=pixel;
  if (depth.empty() || pixel.x<0 || pixel.y<0 ||
      pixel.x>=depth.cols || pixel.y>=depth.rows) return result;
  result.supported=depth.type()==CV_16UC1 || depth.type()==CV_32FC1;
  if (!result.supported) return result;
  const cv::Rect requested(pixel.x-2,pixel.y-2,5,5);
  result.roi=requested & cv::Rect(0,0,depth.cols,depth.rows);
  result.clipped=result.roi!=requested;
  std::vector<double> values;
  values.reserve(25);
  for (int y=result.roi.y;y<result.roi.y+result.roi.height;++y) {
    for (int x=result.roi.x;x<result.roi.x+result.roi.width;++x) {
      const double z=depth.type()==CV_16UC1
          ? depth.at<std::uint16_t>(y,x)*.001
          : static_cast<double>(depth.at<float>(y,x));
      if (std::isfinite(z) && z>0) values.push_back(z);
    }
  }
  result.valid_pixels=values.size();
  if (values.empty()) return result;
  std::sort(values.begin(),values.end());
  const auto middle=values.size()/2;
  result.median_m=values.size()%2 ? values[middle]
      : (values[middle-1]+values[middle])*.5;
  return result;
}

}  // namespace camera_overlay
