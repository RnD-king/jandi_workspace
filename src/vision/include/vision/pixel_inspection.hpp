#pragma once

// Small ROS/TensorRT-independent helpers for raw YOLO pixel inspection.
#include "vision/types.hpp"

#include <cmath>
#include <optional>
#include <vector>

namespace vision {

inline cv::Point BoxCenter(const cv::Rect &box) {
  return {box.x + box.width / 2, box.y + box.height / 2};
}

// This deliberately does NOT apply mission depth/size/temporal filters.
inline std::optional<Detection> SelectHighestConfidenceBall(
    const std::vector<Detection> &detections,
    int class_id, float confidence_threshold) {
  std::optional<Detection> best;
  for (const auto &d : detections) {
    if (d.class_id != class_id || !std::isfinite(d.confidence) ||
        d.confidence < confidence_threshold ||
        d.box.width <= 0 || d.box.height <= 0) continue;
    if (!best || d.confidence > best->confidence) best = d;
  }
  return best;
}

} // namespace vision
