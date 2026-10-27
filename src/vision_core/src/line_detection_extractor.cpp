#include "vision_core/line_detection_extractor.hpp"

#include <algorithm>
#include <utility>

namespace vision_core {

std::vector<Point2>
ExtractLineCenters(const std::vector<Detection> &detections,
                   const LineDetectionConfig &config,
                   std::vector<std::size_t> *detection_indices) {
  std::vector<std::pair<Point2,std::size_t>> selected;
  selected.reserve(detections.size());
  for (std::size_t index=0;index<detections.size();++index) {
    const auto &detection=detections[index];
    if (detection.class_id != config.class_id ||
        detection.confidence < config.confidence ||
        detection.box.width <= config.min_box_width ||
        detection.box.height <= config.min_box_height) {
      continue;
    }
    selected.push_back({{detection.box.x + 0.5 * detection.box.width,
                         detection.box.y + 0.5 * detection.box.height},index});
  }
  std::sort(selected.begin(), selected.end(),
            [](const auto &left, const auto &right) {
              const auto &a=left.first,&b=right.first;
              return (a.v == b.v) ? (a.u < b.u) : (a.v > b.v);
            });
  std::vector<Point2> centers;
  centers.reserve(selected.size());
  if (detection_indices) detection_indices->clear();
  for (const auto &item:selected) {
    centers.push_back(item.first);
    if (detection_indices) detection_indices->push_back(item.second);
  }
  return centers;
}

} // namespace vision_core
