#pragma once
#include <algorithm>
#include <cmath>
#include <cstdio>
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
struct DepthRegionPixelStats {
  int valid{0};
  int total{0};
  double Ratio() const { return total > 0 ? static_cast<double>(valid) / total : 0.0; }
};
inline DepthRegionPixelStats CountValidDepthPixels(const cv::Mat& depth,
                                                   const cv::Rect& raw) {
  DepthRegionPixelStats stats;
  if (depth.empty() || (depth.type()!=CV_16UC1 && depth.type()!=CV_32FC1)) return stats;
  const auto box=raw & cv::Rect(0,0,depth.cols,depth.rows);
  stats.total=box.area();
  for (int y=box.y;y<box.y+box.height;++y)
    for (int x=box.x;x<box.x+box.width;++x) {
      const double z=depth.type()==CV_16UC1 ? depth.at<std::uint16_t>(y,x)*0.001
                                               : depth.at<float>(y,x);
      if (std::isfinite(z) && z>0.05 && z<10.0) ++stats.valid;
    }
  return stats;
}
inline void DrawDepthRegions(cv::Mat& image,const cv::Rect& box,int width,
                             const cv::Mat& aligned_depth=cv::Mat()) {
  const auto r=DepthRegions(box,image.size(),width);
  if (!r.center.empty()) cv::rectangle(image,r.center,{230,230,230},1);
  if (!r.left.empty()) cv::rectangle(image,r.left,{255,180,0},1);
  if (!r.right.empty()) cv::rectangle(image,r.right,{0,220,255},1);
  if (aligned_depth.empty()) return;
  const cv::Rect regions[] = {r.left,r.center,r.right};
  const char* names[] = {"LEFT","CTR","RIGHT"};
  const cv::Scalar colors[] = {{255,180,0},{230,230,230},{0,220,255}};
  const int label_x = std::clamp(box.x, 0, std::max(0,image.cols-1));
  // Diagnostic-only values: never introduce an acceptance/rejection gate.
  for (int i=0;i<3;++i) {
    const auto stats=CountValidDepthPixels(aligned_depth,regions[i]);
    const auto median=MedianRegionMeters(aligned_depth,regions[i]);
    char label[120];
    if (median) {
      std::snprintf(label,sizeof(label),"%s %.3fm %d/%d (%.0f%%)",
                    names[i],*median,stats.valid,stats.total,100.0*stats.Ratio());
    } else {
      std::snprintf(label,sizeof(label),"%s -- %d/%d (%.0f%%)",
                    names[i],stats.valid,stats.total,100.0*stats.Ratio());
    }
    const int text_y = std::clamp(box.y+16+17*i,14,std::max(14,image.rows-1));
    cv::putText(image,label,{label_x,text_y},cv::FONT_HERSHEY_SIMPLEX,
                0.46,colors[i],1,cv::LINE_AA);
  }
}
} // namespace vision
