#include "vision_core/line_detection_extractor.hpp"

#include <algorithm>

namespace vision_core {

std::vector<Point2>
ExtractLineCenters(const std::vector<Detection> &detections,
                   const LineDetectionConfig &config) {
  std::vector<Point2> centers;
  centers.reserve(detections.size());
  for (const auto &detection : detections) {
    if (detection.class_id != config.class_id ||
        detection.confidence < config.confidence ||
        detection.box.width <= config.min_box_width ||
        detection.box.height <= config.min_box_height) {
      continue;
    }
    centers.push_back({detection.box.x + 0.5 * detection.box.width,
                       detection.box.y + 0.5 * detection.box.height});
  }
  std::sort(centers.begin(), centers.end(),
            [](const Point2 &a, const Point2 &b) {
              return (a.v == b.v) ? (a.u < b.u) : (a.v > b.v);
            });
  return centers;
}

} // namespace vision_core
