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
// One source of truth for the actual Depth ROIs and the debug overlay.
struct BackboardDepthRegions {
  cv::Rect inner,center,left,right;
  double left_u{},right_u{};
  bool center_fallback{false};
  int strip_width{0};
};
struct BackboardDepthSample {
  BackboardDepthRegions regions;
  std::optional<double> center,left,right;
};
inline cv::Rect InnerBackboard(const cv::Rect& raw,const cv::Size& image,double margin) {
  if(raw.width<=0 || raw.height<=0 || image.width<=0 || image.height<=0 ||
     !std::isfinite(margin) || margin<0 || margin>=.5) return {};
  const int my=static_cast<int>(std::ceil(raw.height*margin));
  return cv::Rect(raw.x,raw.y+my,raw.width,
                  std::max(0,raw.height-2*my)) & cv::Rect(0,0,image.width,image.height);
}
inline BackboardDepthRegions DepthRegions(const cv::Rect& raw,const cv::Size& image,
                                          double strip_width_ratio,int center_size=21,
                                          double margin=.10) {
  BackboardDepthRegions r;
  r.inner=InnerBackboard(raw,image,margin);
  if(r.inner.empty()) return r;
  const int center_u=static_cast<int>(std::lround(raw.x+.5*raw.width));
  const int center_v=static_cast<int>(std::lround(raw.y+.5*raw.height));
  r.center=cv::Rect(center_u-center_size/2,center_v-center_size/2,
                    center_size,center_size)&r.inner;
  if(!std::isfinite(strip_width_ratio) || strip_width_ratio<=0 || strip_width_ratio>.5)
    return r;
  // Ratios refer to the original detection, including when it is image-clipped.
  const int width=std::min(static_cast<int>(std::ceil(raw.width*strip_width_ratio-1e-9)),raw.width/2);
  if(width<=0)return r;
  r.strip_width=width;
  r.left=cv::Rect(raw.x,r.inner.y,width,r.inner.height)&r.inner;
  r.right=cv::Rect(raw.x+raw.width-width,r.inner.y,width,r.inner.height)&r.inner;
  r.left_u=r.left.x+.5*(r.left.width-1);
  r.right_u=r.right.x+.5*(r.right.width-1);
  return r;
}
struct DepthRegionPixelStats {
  int valid{0},total{0};
  double Ratio() const {return total>0?static_cast<double>(valid)/total:0.0;}
};
inline DepthRegionPixelStats CountValidDepthPixels(const cv::Mat& depth,
    const cv::Rect& raw,double min_depth=.05,double max_depth=10.) {
  DepthRegionPixelStats stats;
  if(depth.empty() || (depth.type()!=CV_16UC1 && depth.type()!=CV_32FC1))return stats;
  const auto box=raw & cv::Rect(0,0,depth.cols,depth.rows);
  stats.total=box.area();
  for(int y=box.y;y<box.y+box.height;++y)for(int x=box.x;x<box.x+box.width;++x){
    const double z=depth.type()==CV_16UC1?depth.at<std::uint16_t>(y,x)*.001:depth.at<float>(y,x);
    if(std::isfinite(z)&&z>=min_depth&&z<=max_depth)++stats.valid;
  }
  return stats;
}
inline std::optional<double> MedianRegionMeters(const cv::Mat& depth,
    const cv::Rect& raw,double min_valid_ratio=0.0,double min_depth=.05,
    double max_depth=10.) {
  if(depth.empty() || (depth.type()!=CV_16UC1 && depth.type()!=CV_32FC1))return {};
  const auto box=raw&cv::Rect(0,0,depth.cols,depth.rows);
  if(box.empty())return {};
  std::vector<double> values;values.reserve(box.area());
  for(int y=box.y;y<box.y+box.height;++y)for(int x=box.x;x<box.x+box.width;++x){
    const double z=depth.type()==CV_16UC1?depth.at<std::uint16_t>(y,x)*.001:depth.at<float>(y,x);
    if(std::isfinite(z)&&z>=min_depth&&z<=max_depth)values.push_back(z);
  }
  if(values.empty()||static_cast<double>(values.size())+1e-9<
     min_valid_ratio*box.area())return {};
  auto middle=values.begin()+values.size()/2;
  std::nth_element(values.begin(),middle,values.end());return *middle;
}
inline BackboardDepthSample SampleBackboardDepth(const cv::Mat& depth,
    const cv::Rect& box,int center_size,double strip_width,double strip_step,
    double strip_max,double margin,double valid_ratio,double min_depth,
    double max_depth,bool need_sides=true){
  BackboardDepthSample out;
  out.regions=DepthRegions(box,depth.size(),strip_width,center_size,margin);
  const auto median=[&](const cv::Rect& r){
    return MedianRegionMeters(depth,r,valid_ratio,min_depth,max_depth);
  };
  out.center=median(out.regions.center);
  if(!out.center){
    out.regions.center=out.regions.inner;
    out.regions.center_fallback=true;
    out.center=median(out.regions.center);
  }
  if(!need_sides)return out;
  if(!std::isfinite(strip_width) || !std::isfinite(strip_step) ||
     !std::isfinite(strip_max) || strip_width<=0 || strip_step<=0 ||
     strip_max<strip_width || strip_max>.5)return out;
  for(int attempt=0;;++attempt){
    const double width=std::min(strip_width+attempt*strip_step,strip_max);
    auto candidate=DepthRegions(box,depth.size(),width,center_size,margin);
    candidate.center=out.regions.center;
    candidate.center_fallback=out.regions.center_fallback;
    out.regions=candidate;
    out.left=median(candidate.left);
    out.right=median(candidate.right);
    if((out.left && out.right) || width>=strip_max-1e-12)break;
  }
  return out;
}
inline void DrawDepthRegions(cv::Mat& image,const cv::Rect& box,double strip_width,
    const cv::Mat& aligned_depth=cv::Mat(),int center_size=21,double strip_step=.05,
    double strip_max=.30,double margin=.10,double valid_ratio=.10,
    double min_depth=.10,double max_depth=2.50,bool near=true){
  const auto sample=SampleBackboardDepth(aligned_depth,box,center_size,
      strip_width,strip_step,strip_max,margin,valid_ratio,min_depth,max_depth,near);
  auto r=sample.regions;
  if(aligned_depth.empty())r=DepthRegions(box,image.size(),strip_width,center_size,margin);
  if(!r.inner.empty())cv::rectangle(image,r.inner,{100,200,100},1);
  if(!r.center.empty())cv::rectangle(image,r.center,{230,230,230},1);
  if(near && !r.left.empty())cv::rectangle(image,r.left,{255,180,0},1);
  if(near && !r.right.empty())cv::rectangle(image,r.right,{0,220,255},1);
  if(aligned_depth.empty())return;
  const cv::Rect regions[]={r.left,r.center,r.right};
  const std::optional<double> depths[]={sample.left,sample.center,sample.right};
  const char* names[]={"LEFT","CTR","RIGHT"};
  const cv::Scalar colors[]={{255,180,0},{230,230,230},{0,220,255}};
  const int x=std::clamp(box.x,0,std::max(0,image.cols-1));
  for(int i=0;i<3;++i){
    if(!near && i!=1)continue;
    const auto stats=CountValidDepthPixels(aligned_depth,regions[i],min_depth,max_depth);
    char label[120];
    if(depths[i])std::snprintf(label,sizeof(label),"%s %.3fm %d/%d (%.0f%%)",
       names[i],*depths[i],stats.valid,stats.total,100.*stats.Ratio());
    else std::snprintf(label,sizeof(label),"%s -- %d/%d (%.0f%%)",
       names[i],stats.valid,stats.total,100.*stats.Ratio());
    const int y=std::clamp(box.y+16+17*i,14,std::max(14,image.rows-1));
    cv::putText(image,label,{x,y},cv::FONT_HERSHEY_SIMPLEX,.46,colors[i],1,cv::LINE_AA);
  }
}
} // namespace vision
