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
  const auto r=DepthRegions(box,{640,480},.15,21,.10);
  assert(r.inner==cv::Rect(100,110,100,80));
  assert(r.center==cv::Rect(140,140,21,21));
  assert(r.left==cv::Rect(100,110,15,80));
  assert(r.right==cv::Rect(185,110,15,80));
  assert(r.left_u==107 && r.right_u==192);
  const auto large=DepthRegions({100,100,200,100},{640,480},.15,21,.10);
  assert(large.left.width==30 && large.right.width==30);
  const auto clipped=DepthRegions({-40,100,100,100},{640,480},.15,21,.10);
  assert(clipped.inner==cv::Rect(0,110,60,80));
  assert(clipped.left.empty() && clipped.right==cv::Rect(45,110,15,80));
  assert(DepthRegions({100,100,103,100},{640,480},.15).strip_width==16);
  assert(DepthRegions(box,{640,480},std::numeric_limits<double>::quiet_NaN()).left.empty());
  assert(DepthRegions({100,100,0,100},{640,480},.15).inner.empty());

  cv::Mat depth(480,640,CV_32FC1,cv::Scalar(0));
  depth(r.center).setTo(1.4);
  depth(r.left).setTo(1.0);
  depth(r.right).setTo(2.0);
  auto sample=SampleBackboardDepth(depth,box,21,.15,.05,.30,.10,.10,.1,2.5);
  assert(sample.center && sample.left && sample.right);
  assert(sample.regions.strip_width==15);
  assert(std::abs(*sample.left-1.0)<1e-5 && std::abs(*sample.right-2.0)<1e-5);

  // Pixels in the removed TOP/BOTTOM margins must never count.
  depth.setTo(0);
  depth(cv::Rect(100,100,100,10)).setTo(1.5);
  depth(cv::Rect(100,190,100,10)).setTo(1.5);
  sample=SampleBackboardDepth(depth,box,21,.15,.05,.30,.10,.10,.1,2.5);
  assert(!sample.center && !sample.left && !sample.right);
  assert(sample.regions.strip_width==30);

  // The central 21x21 valid-pixel threshold and full-inner fallback remain.
  depth.setTo(0);
  depth.at<float>(150,150)=1.5;
  assert(!MedianRegionMeters(depth,r.center,.10,.1,2.5));
  depth.setTo(0);
  for(int i=0;i<44;++i)depth.at<float>(140+i/21,140+i%21)=1.5;
  assert(!MedianRegionMeters(depth,r.center,.10,.1,2.5));
  depth.at<float>(142,142)=1.5;
  assert(MedianRegionMeters(depth,r.center,.10,.1,2.5));
  depth.setTo(0);
  depth(cv::Rect(120,110,20,40)).setTo(1.7);
  sample=SampleBackboardDepth(depth,box,21,.15,.05,.30,.10,.10,.1,2.5);
  assert(sample.center && std::abs(*sample.center-1.7)<1e-5);
  assert(sample.regions.center_fallback && sample.regions.center==r.inner);

  // Either missing side grows BOTH bands by five percentage points.
  for(int width : {15,20,25,30}) {
    depth.setTo(0);
    depth(r.center).setTo(1.5);
    depth(r.left).setTo(1.2);
    depth(cv::Rect(200-width,110,3,80)).setTo(1.8);
    sample=SampleBackboardDepth(depth,box,21,.15,.05,.30,.10,.10,.1,2.5);
    assert(sample.left && sample.right && sample.regions.strip_width==width);
    assert(sample.regions.left.width==width && sample.regions.right.width==width);
    assert(std::abs(*sample.left-1.2)<1e-5 && std::abs(*sample.right-1.8)<1e-5);
    assert(sample.regions.left_u==100+.5*(width-1));
    assert(sample.regions.right_u==200-width+.5*(width-1));
  }
  // Valid readings outside 30% must NOT extend the search or become a depth.
  depth.setTo(0);
  depth(r.center).setTo(1.5);
  depth(r.left).setTo(1.2);
  depth(cv::Rect(165,110,3,80)).setTo(1.8);
  sample=SampleBackboardDepth(depth,box,21,.15,.05,.30,.10,.10,.1,2.5);
  assert(sample.center && sample.left && !sample.right);
  assert(sample.regions.strip_width==30);
  // Far-view callers still request only center depth.
  sample=SampleBackboardDepth(depth,box,21,.15,.05,.30,.10,.10,.1,2.5,false);
  assert(sample.center && !sample.left && !sample.right);
  assert(sample.regions.strip_width==15);
  cv::Mat mm(480,640,CV_16UC1,cv::Scalar(1500));
  sample=SampleBackboardDepth(mm,box,21,.15,.05,.30,.10,.10,.1,2.5);
  assert(sample.center.value()==1.5 && sample.left.value()==1.5 && sample.right.value()==1.5);
}
