#include "vision/backboard_depth.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <limits>
int main(){
  using namespace vision;
  const cv::Rect box{100,100,100,100};
  const auto r=DepthRegions(box,{640,480},7,21,.10);
  assert(r.inner==cv::Rect(110,110,80,80));
  assert(r.center==cv::Rect(140,140,21,21));
  assert(r.left==cv::Rect(110,110,7,80));
  assert(r.right==cv::Rect(183,110,7,80));
  assert(r.left_u==113 && r.right_u==186);
  cv::Mat depth(480,640,CV_32FC1,cv::Scalar(0));
  depth(r.center).setTo(1.4);
  depth(r.left).setTo(1.0);
  depth(r.right).setTo(2.0);
  auto sample=SampleBackboardDepth(depth,box,21,7,7,28,.10,.10,.1,2.5);
  assert(sample.center && sample.left && sample.right);
  assert(sample.regions.strip_width==7);
  // A single valid depth is no longer accepted in a 21x21 region.
  depth.setTo(0);
  depth.at<float>(150,150)=1.5;
  assert(!MedianRegionMeters(depth,r.center,.10,.1,2.5));
  // The 10% threshold for 441 pixels requires at least 45 readings.
  for(int i=0;i<44;++i)depth.at<float>(140+i/21,140+i%21)=1.5;
  assert(!MedianRegionMeters(depth,r.center,.10,.1,2.5));
  depth.at<float>(142,142)=1.5;
  assert(MedianRegionMeters(depth,r.center,.10,.1,2.5));
  // Center fallback uses the full 80%-inset area.
  depth.setTo(0);
  depth(cv::Rect(120,120,20,35)).setTo(1.7);
  sample=SampleBackboardDepth(depth,box,21,7,7,28,.10,.10,.1,2.5);
  assert(sample.center && std::abs(*sample.center-1.7)<1e-5);
  assert(sample.regions.center_fallback && sample.regions.center==r.inner);
  // One missing side makes both bands grow together, never different widths.
  depth.setTo(0);
  depth(r.center).setTo(1.5);
  depth(r.left).setTo(1.2);
  depth(cv::Rect(176,110,7,80)).setTo(1.8); // only the second right band
  sample=SampleBackboardDepth(depth,box,21,7,7,28,.10,.10,.1,2.5);
  assert(sample.left && sample.right && sample.regions.strip_width==14);
  assert(std::abs(*sample.left-1.2)<1e-5);
  assert(std::abs(*sample.right-1.8)<1e-5);
  // Valid 16-bit depth uses the same conversion and threshold.
  cv::Mat mm(480,640,CV_16UC1,cv::Scalar(1500));
  assert(MedianRegionMeters(mm,r.center,.1,.1,2.5).value()==1.5);
}
