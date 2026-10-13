#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
namespace vision {
struct BackboardDepthRegions { cv::Rect center,left,right; double left_u{},right_u{}; };
inline BackboardDepthRegions DepthRegions(const cv::Rect& raw,const cv::Size& size,int width) {
  BackboardDepthRegions r;
  const auto box=raw & cv::Rect(0,0,size.width,size.height);
  if (box.empty()) return r;
  const int w=std::min(width,box.width);
  if (w<1) return r;
  r.left={box.x,box.y,w,box.height};
  r.right={box.x+box.width-w,box.y,w,box.height};
  r.left_u=r.left.x+0.5*(r.left.width-1);
  r.right_u=r.right.x+0.5*(r.right.width-1);
  const int u=static_cast<int>(std::lround(raw.x+0.5*raw.width));
  const int vv=static_cast<int>(std::lround(raw.y+0.5*raw.height));
  r.center=cv::Rect(u-1,vv-1,3,3) & box;
  return r;
}
inline std::optional<double> MedianRegionMeters(const cv::Mat& depth,const cv::Rect& raw) {
  if (depth.empty() || (depth.type()!=CV_16UC1 && depth.type()!=CV_32FC1)) return {};
  const auto box=raw & cv::Rect(0,0,depth.cols,depth.rows);
  std::vector<double> values; values.reserve(box.area());
  for (int y=box.y;y<box.y+box.height;++y) for (int x=box.x;x<box.x+box.width;++x) {
    const double z=depth.type()==CV_16UC1 ? depth.at<std::uint16_t>(y,x)*0.001 : depth.at<float>(y,x);
    if (std::isfinite(z) && z>0.05 && z<10.0) values.push_back(z);
  }
  if (values.empty()) return {};
  auto middle=values.begin()+values.size()/2;
  std::nth_element(values.begin(),middle,values.end()); return *middle;
}
inline void DrawDepthRegions(cv::Mat& image,const cv::Rect& box,int width) {
  const auto r=DepthRegions(box,image.size(),width);
  if (!r.center.empty()) cv::rectangle(image,r.center,{230,230,230},1);
  if (!r.left.empty()) cv::rectangle(image,r.left,{255,180,0},1);
  if (!r.right.empty()) cv::rectangle(image,r.right,{0,220,255},1);
}
} // namespace vision
