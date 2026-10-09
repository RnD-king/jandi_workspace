#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include "vision_core/types.hpp"

namespace vision_core {

struct ObjectTargetConfig {
  int ball_class_id{};
  int goal_class_id{};
  int backboard_class_id{};
  int hurdle_class_id{};
  double ball_confidence{};
  double goal_confidence{};
  double backboard_confidence{};
  double hurdle_confidence{};
  double min_box_width{};
  double min_box_height{};
};

struct ObjectTargets {
  std::optional<ObjectTarget> ball;
  std::optional<ObjectTarget> goal;
  std::optional<ObjectTarget> backboard;
  std::optional<ObjectTarget> hurdle;
};

struct ObjectTargetIndices {
  std::optional<std::size_t> ball;
  std::optional<std::size_t> goal;
  std::optional<std::size_t> backboard;
  std::optional<std::size_t> hurdle;
};

struct ObjectTargetSelection {
  ObjectTargets targets;
  ObjectTargetIndices indices;
};

struct ObjectTargetCandidate {
  ObjectTarget target;
  std::size_t detection_index{};
};

// confidence는 threshold 필터에만 사용한다. 반환 순서는 원본 detection
// 순서이며 association tracker가 이후 중심 이동과 bbox 크기 변화로 고른다.
std::vector<ObjectTargetCandidate> ExtractObjectTargetCandidates(
    const std::vector<Detection> &detections, int class_id,
    double confidence_threshold, const ObjectTargetConfig &config);

// target과 원본 detection index를 함께 반환한다. index는 RGB-D adapter가
// 선택된 bbox에 대응하는 depth 표본을 찾을 때 사용한다.
ObjectTargetSelection
ExtractObjectTargetSelection(const std::vector<Detection> &detections,
                             const ObjectTargetConfig &config);

ObjectTargets ExtractObjectTargets(const std::vector<Detection> &detections,
                                   const ObjectTargetConfig &config);

} // namespace vision_core
