#pragma once

#include <vector>

#include "vision_core/types.hpp"

namespace vision_core {

// YOLO detection 목록에서 line class의 bbox 중심만 추출하고 아래쪽 점부터
// 위쪽 점 순서로 정렬한다.
struct LineDetectionConfig {
  int class_id{};
  double confidence{};
  double min_box_width{};
  double min_box_height{};
};

std::vector<Point2>
ExtractLineCenters(const std::vector<Detection> &detections,
                   const LineDetectionConfig &config);

} // namespace vision_core
