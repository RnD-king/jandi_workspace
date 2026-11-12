#pragma once

#include <vision/mission_debug_view.hpp>
#include <vision/types.hpp>

namespace camera_overlay {
// Reuse the production box colors and the exact adaptive depth ROIs, medians,
// and valid-pixel ratios. No mission controller is stepped by this viewer.
inline cv::Mat RenderYoloInspection(const cv::Mat& bgr,
    const std::vector<vision::Detection>& detections, const cv::Mat& depth,
    const vision_core::MissionControllerConfig& config, bool goal_bands) {
  auto image = bgr.clone();
  vision::DrawRawDebugDetections(image, detections, config);
  for (const auto& detection : detections)
    vision::DrawFilteredDebugDetection(image, detection, config);
  if (goal_bands) {
    for (const auto& detection : detections) {
      if (detection.class_id != config.object_targets.backboard_class_id ||
          detection.confidence < config.object_targets.backboard_confidence) continue;
      vision::DrawDepthRegions(image, detection.box,
          config.backboard_depth_strip_width_ratio, depth,
          config.backboard_depth_center_size_px,
          config.backboard_depth_strip_step_ratio,
          config.backboard_depth_strip_max_width_ratio,
          config.backboard_depth_inner_margin_ratio,
          config.backboard_depth_min_valid_ratio,
          config.backboard_min_depth_m, config.backboard_max_depth_m, true);
    }
  }
  return image;
}
}  // namespace camera_overlay
