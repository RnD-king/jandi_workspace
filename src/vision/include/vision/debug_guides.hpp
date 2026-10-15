#pragma once
#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>
#include "vision_core/hurdle_controller.hpp"

namespace vision {
// Shared by the live and tuning displays; all thresholds come from the core.
inline void DrawHurdleDownGuides(cv::Mat& image,
                                const vision_core::HurdleConfig& config,
                                bool fine_stance) {
  const auto line = [&](double v, const cv::Scalar& color, const char* label) {
    if (image.empty() || !std::isfinite(v) || v < 0 || v >= image.rows) return;
    const int y=std::clamp(static_cast<int>(std::lround(v)),0,image.rows-1);
    cv::line(image,{0,y},{image.cols-1,y},color,1,cv::LINE_AA);
    if (label) cv::putText(image,label,{8,std::max(14,y-5)},
        cv::FONT_HERSHEY_SIMPLEX,0.55,color,2,cv::LINE_AA);
  };
  line(config.down_one_step_v_px,cv::Scalar(255,120,255),nullptr);
  line(config.fine_mode_v_px,cv::Scalar(200,0,200),nullptr);
  if (fine_stance) line(config.contact_v_px,cv::Scalar(0,255,255),nullptr);
}
}
