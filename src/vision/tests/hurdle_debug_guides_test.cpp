#include "vision/debug_guides.hpp"
#include <stdexcept>
int main() {
  vision_core::HurdleConfig config;
  config.down_one_step_v_px=100;config.fine_mode_v_px=180;config.contact_v_px=260;
  cv::Mat walk=cv::Mat::zeros(480,640,CV_8UC3),fine=walk.clone();
  vision::DrawHurdleDownGuides(walk,config,false);
  vision::DrawHurdleDownGuides(fine,config,true);
  const auto visible=[](const cv::Mat& image,int y) {
    return image.at<cv::Vec3b>(y,400)!=cv::Vec3b(0,0,0);
  };
  if(!visible(walk,100)||!visible(walk,180)||visible(walk,260)||!visible(fine,260))
    throw std::runtime_error("HURDLE WALK/FINE guide count mismatch");
  if(fine.at<cv::Vec3b>(260,400)==fine.at<cv::Vec3b>(180,400))
    throw std::runtime_error("CONTACT guide must have a different color");
  // Tuning values must drive the exact same helper as live display values.
  config.contact_v_px=300;
  cv::Mat tuned=cv::Mat::zeros(480,640,CV_8UC3);
  vision::DrawHurdleDownGuides(tuned,config,true);
  if(visible(tuned,260)||!visible(tuned,300))throw std::runtime_error("CONTACT guide ignored configuration");
}
