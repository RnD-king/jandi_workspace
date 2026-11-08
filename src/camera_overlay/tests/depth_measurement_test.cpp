#include "camera_overlay/depth_measurement.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>

int main() {
  using camera_overlay::MeasureDepth5x5;
  // Outside the requested 5x5 patch must not influence the statistic.
  cv::Mat depth(9,9,CV_16UC1,cv::Scalar(65000));
  for (int y=2;y<=6;++y)
    for (int x=2;x<=6;++x)
      depth.at<std::uint16_t>(y,x)=static_cast<std::uint16_t>(1000+(y-2)*5+x-2);
  auto m=MeasureDepth5x5(depth,{4,4});
  assert(m.supported && !m.clipped && m.roi==cv::Rect(2,2,5,5));
  assert(m.valid_pixels==25 && m.median_m && std::abs(*m.median_m-1.012)<1e-12);

  depth.at<std::uint16_t>(4,4)=65000;
  m=MeasureDepth5x5(depth,{4,4});
  assert(m.median_m && std::abs(*m.median_m-1.013)<1e-12);
  depth.at<std::uint16_t>(4,4)=1012;

  // Zero is missing depth; for even valid counts, average the middle pair.
  depth.at<std::uint16_t>(2,2)=0;
  m=MeasureDepth5x5(depth,{4,4});
  assert(m.valid_pixels==24 && m.median_m && std::abs(*m.median_m-1.0125)<1e-12);

  cv::Mat metres(5,5,CV_32FC1,cv::Scalar(0));
  metres.at<float>(0,0)=std::numeric_limits<float>::quiet_NaN();
  metres.at<float>(0,1)=std::numeric_limits<float>::infinity();
  metres.at<float>(0,2)=-1;
  metres.at<float>(1,1)=.5f;metres.at<float>(2,2)=1.5f;
  m=MeasureDepth5x5(metres,{2,2});
  assert(m.valid_pixels==2 && m.median_m && *m.median_m==1);
  metres.setTo(cv::Scalar(0));
  m=MeasureDepth5x5(metres,{2,2});
  assert(m.supported && m.valid_pixels==0 && !m.median_m);

  // A corner uses only the available 3x3; no recentering or fabricated values.
  depth.setTo(cv::Scalar(750));
  m=MeasureDepth5x5(depth,{0,0});
  assert(m.clipped && m.roi==cv::Rect(0,0,3,3) && m.valid_pixels==9);
  assert(m.median_m && *m.median_m==.75);
  // Non-contiguous image storage must retain its actual row stride.
  const cv::Mat cropped=depth(cv::Rect(1,1,5,5));
  m=MeasureDepth5x5(cropped,{2,2});
  assert(m.valid_pixels==25 && m.median_m && *m.median_m==.75);

  assert(!MeasureDepth5x5({}, {0,0}).median_m);
  assert(!MeasureDepth5x5(depth,{-1,0}).median_m);
  assert(!MeasureDepth5x5(depth,{9,0}).median_m);
  m=MeasureDepth5x5(cv::Mat(5,5,CV_8UC1,cv::Scalar(20)),{2,2});
  assert(!m.supported && !m.median_m);
}
