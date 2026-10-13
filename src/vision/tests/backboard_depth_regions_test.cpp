#include "vision/backboard_depth.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <limits>
int main() {
  const cv::Rect box(10,20,30,40);
  const auto region=vision::DepthRegions(box,{640,480},5);
  assert(region.left==cv::Rect(10,20,5,40));
  assert(region.right==cv::Rect(35,20,5,40));
  assert(region.center.width==3 && region.center.height==3);
  assert(region.left_u==12 && region.right_u==37);
  cv::Mat depth(480,640,CV_32FC1,cv::Scalar(4));
  depth(region.left).setTo(1);depth(region.right).setTo(2);
  depth.at<float>(20,10)=0;depth.at<float>(21,10)=std::numeric_limits<float>::quiet_NaN();
  assert(vision::MedianRegionMeters(depth,region.left).value()==1);
  assert(vision::MedianRegionMeters(depth,region.right).value()==2);
  const auto left_stats=vision::CountValidDepthPixels(depth,region.left);
  assert(left_stats.total==200 && left_stats.valid==198);
  assert(std::abs(left_stats.Ratio()-.99)<1e-12);
  // Most valid rows can differ from the old middle 3x3, and define the new median.
  depth(cv::Rect(10,39,5,3)).setTo(3);
  assert(vision::MedianRegionMeters(depth,region.left).value()==1);
  const auto clipped=vision::DepthRegions({-3,470,12,40},{640,480},5);
  assert(clipped.left==cv::Rect(0,470,5,10));
  assert(clipped.right==cv::Rect(4,470,5,10));
  depth(region.left).setTo(0);assert(!vision::MedianRegionMeters(depth,region.left));
  cv::Mat millimeters(480,640,CV_16UC1,cv::Scalar(1500));
  assert(vision::MedianRegionMeters(millimeters,region.right).value()==1.5);
}
