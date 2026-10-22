#pragma once

#include <cstddef>
#include <optional>
#include <vector>
#include <utility>
#include <limits>

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
  double ball_min_box_width{};
  double ball_min_box_height{};
  double goal_min_box_width{};
  double goal_min_box_height{};
  double backboard_min_box_width{};
  double backboard_min_box_height{};
  double hurdle_min_box_width{};
  double hurdle_min_box_height{};

  double ball_aspect_ratio_min{}, ball_aspect_ratio_max{};
  double hurdle_aspect_ratio_min{};
  bool ball_depth_enabled{}, hurdle_depth_enabled{};
  bool ball_allow_missing_depth{}, hurdle_allow_missing_depth{};
  double ball_forward_min_depth_m{}, ball_forward_max_depth_m{};
  double ball_down_min_depth_m{}, ball_down_max_depth_m{};
  double hurdle_min_depth_m{}, hurdle_max_depth_m{};
  int depth_roi_size_px{};
  bool SamePerceptionFilters(const ObjectTargetConfig& c) const {
    return SameBoxFilters(c) && ball_aspect_ratio_min==c.ball_aspect_ratio_min &&
        ball_aspect_ratio_max==c.ball_aspect_ratio_max && hurdle_aspect_ratio_min==c.hurdle_aspect_ratio_min &&
        ball_depth_enabled==c.ball_depth_enabled && hurdle_depth_enabled==c.hurdle_depth_enabled &&
        ball_allow_missing_depth==c.ball_allow_missing_depth && hurdle_allow_missing_depth==c.hurdle_allow_missing_depth &&
        ball_forward_min_depth_m==c.ball_forward_min_depth_m && ball_forward_max_depth_m==c.ball_forward_max_depth_m &&
        ball_down_min_depth_m==c.ball_down_min_depth_m && ball_down_max_depth_m==c.ball_down_max_depth_m &&
        hurdle_min_depth_m==c.hurdle_min_depth_m && hurdle_max_depth_m==c.hurdle_max_depth_m && depth_roi_size_px==c.depth_roi_size_px;
  }
  bool AcceptAspect(const Detection& d) const {
    if (!(d.box.height>0) || !(d.box.width>0)) return false;
    const double ratio=d.box.width/d.box.height;
    if(d.class_id==ball_class_id && ball_aspect_ratio_max>0)
      return ratio>=ball_aspect_ratio_min && ratio<=ball_aspect_ratio_max;
    if(d.class_id==hurdle_class_id) return ratio>=hurdle_aspect_ratio_min;
    return true;
  }

  std::pair<double,double> MinimumBoxSize(int class_id) const {
    if (class_id == ball_class_id) return {ball_min_box_width,ball_min_box_height};
    if (class_id == goal_class_id) return {goal_min_box_width,goal_min_box_height};
    if (class_id == backboard_class_id) return {backboard_min_box_width,backboard_min_box_height};
    if (class_id == hurdle_class_id) return {hurdle_min_box_width,hurdle_min_box_height};
    return {std::numeric_limits<double>::infinity(),std::numeric_limits<double>::infinity()};
  }
  bool SameBoxFilters(const ObjectTargetConfig& other) const {
    return ball_min_box_width == other.ball_min_box_width &&
        ball_min_box_height == other.ball_min_box_height &&
        goal_min_box_width == other.goal_min_box_width &&
        goal_min_box_height == other.goal_min_box_height &&
        backboard_min_box_width == other.backboard_min_box_width &&
        backboard_min_box_height == other.backboard_min_box_height &&
        hurdle_min_box_width == other.hurdle_min_box_width &&
        hurdle_min_box_height == other.hurdle_min_box_height;
  }
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
