#include "vision/object_depth.hpp"
#include <limits>
#include <stdexcept>
static void Check(bool ok,const char* message) { if(!ok) throw std::runtime_error(message); }
int main() {
  cv::Mat depth(11,11,CV_16UC1,cv::Scalar(4000));
  const cv::Rect box(0,0,10,10);
  for(int y=3;y<=7;++y) for(int x=3;x<=7;++x) depth.at<std::uint16_t>(y,x)=1000;
  depth.at<std::uint16_t>(5,5)=9000;
  auto result=vision::SampleObjectCenterDepth(depth,box,5);
  Check(result.roi==cv::Rect(3,3,5,5),"sampling must use bbox center 5x5");
  Check(result.valid_pixels==25 && result.median_m && *result.median_m==1,"center spike must not override ROI median");
  cv::Mat floating(5,5,CV_32FC1,cv::Scalar(0));
  floating.at<float>(1,1)=1; floating.at<float>(1,2)=3;
  floating.at<float>(2,2)=std::numeric_limits<float>::quiet_NaN();
  floating.at<float>(2,3)=std::numeric_limits<float>::infinity();
  floating.at<float>(3,3)=-1;
  result=vision::SampleObjectCenterDepth(floating,{0,0,4,4},5);
  Check(result.valid_pixels==2 && result.median_m && *result.median_m==2,"float metres and even-count median");
  floating.setTo(0);
  result=vision::SampleObjectCenterDepth(floating,{0,0,4,4},5);
  Check(!result.median_m && result.valid_pixels==0,"empty valid depth must stay missing");
  floating.setTo(20);
  result=vision::SampleObjectCenterDepth(floating,{0,0,4,4},5);
  Check(result.median_m && *result.median_m==20,"far valid depth must not become missing and bypass rejection");
  result=vision::SampleObjectCenterDepth(depth,{-5,-5,10,10},5);
  Check(result.roi==cv::Rect(0,0,3,3) && result.valid_pixels==9,"clip ROI at image edge");
  Check(!vision::SampleObjectCenterDepth({},box,5).median_m,"no image must stay missing");
  cv::Mat wrong_type(5,5,CV_8UC1,cv::Scalar(1));
  Check(!vision::SampleObjectCenterDepth(wrong_type,box,5).median_m,"unsupported encoding must stay missing");
}
