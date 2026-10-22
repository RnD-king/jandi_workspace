#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>
#include <opencv2/core.hpp>

namespace vision {
struct ObjectDepthSample {
  std::optional<double> median_m;
  int valid_pixels{};
  cv::Rect roi;
};
inline ObjectDepthSample SampleObjectCenterDepth(const cv::Mat& depth,
                                                const cv::Rect& bbox,int size) {
  ObjectDepthSample sample;
  if(depth.empty() || (depth.type()!=CV_16UC1 && depth.type()!=CV_32FC1) || size<3 || size%2==0) return sample;
  const int u=static_cast<int>(std::lround(bbox.x+0.5*bbox.width));
  const int v=static_cast<int>(std::lround(bbox.y+0.5*bbox.height));
  sample.roi=cv::Rect(u-size/2,v-size/2,size,size) & bbox & cv::Rect(0,0,depth.cols,depth.rows);
  std::vector<double> values;
  values.reserve(sample.roi.area());
  for(int y=sample.roi.y;y<sample.roi.y+sample.roi.height;++y)
    for(int x=sample.roi.x;x<sample.roi.x+sample.roi.width;++x) {
      const double z=depth.type()==CV_16UC1 ? depth.at<std::uint16_t>(y,x)*0.001 : depth.at<float>(y,x);
      // Do not turn out-of-range distances into missing data: the core owns
      // camera-specific range rejection and the missing-depth policy.
      if(std::isfinite(z) && z>0) values.push_back(z);
    }
  sample.valid_pixels=static_cast<int>(values.size());
  if(values.empty()) return sample;
  const auto middle=values.begin()+values.size()/2;
  std::nth_element(values.begin(),middle,values.end());
  sample.median_m=values.size()%2 ? *middle : 0.5*(*middle+*std::max_element(values.begin(),middle));
  return sample;
}
}
