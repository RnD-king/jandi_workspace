// 처리 순서: [물체 분기 1단계] YOLO bbox가 공통 Detection으로 변환된 뒤 호출한다.
// 역할: class/confidence/크기 조건을 통과한 후보를 만든다. 공/백보드/허들은
// object_association_tracker가 identity를 연결하고, goal은 기존 최고 confidence 선택을 유지한다.
// 값 전달: Detection 목록을 인자로 받고, 선택된 ObjectTargets를 반환한다.
// 다음 단계: 선택된 현재 프레임 target만 각 controller로 전달한다.

#include "vision_core/object_target_extractor.hpp"

#include <cmath>

namespace vision_core {
namespace {

ObjectTarget MakeTarget(const Detection &detection) {
  ObjectTarget target;
  target.class_id = detection.class_id;
  target.confidence = detection.confidence;
  target.box_px = detection.box;
  target.width_px = detection.box.width;
  target.height_px = detection.box.height;
  target.area_px = target.width_px * target.height_px;
  target.center_px = {detection.box.x + 0.5 * target.width_px,
                      detection.box.y + 0.5 * target.height_px};
  target.rectified_center_px = target.center_px;
  return target;
}

struct IndexedTarget {
  std::optional<ObjectTarget> target;
  std::optional<std::size_t> index;
};

IndexedTarget ExtractBest(
    const std::vector<Detection> &detections, int class_id,
    double confidence_threshold, const ObjectTargetConfig &config) {
  IndexedTarget best;
  for (std::size_t index = 0; index < detections.size(); ++index) {
    const auto &detection = detections[index];
    if (detection.class_id != class_id ||
        detection.confidence < confidence_threshold ||
        detection.box.width < config.min_box_width ||
        detection.box.height < config.min_box_height) {
      continue;
    }
    const ObjectTarget candidate = MakeTarget(detection);
    if (!best.target || candidate.confidence > best.target->confidence ||
        (candidate.confidence == best.target->confidence &&
         candidate.area_px > best.target->area_px)) {
      best.target = candidate;
      best.index = index;
    }
  }
  return best;
}

} // namespace

std::vector<ObjectTargetCandidate> ExtractObjectTargetCandidates(
    const std::vector<Detection> &detections, int class_id,
    double confidence_threshold, const ObjectTargetConfig &config) {
  std::vector<ObjectTargetCandidate> candidates;
  for (std::size_t index = 0; index < detections.size(); ++index) {
    const auto &detection = detections[index];
    if (detection.class_id != class_id ||
        !std::isfinite(detection.confidence) ||
        detection.confidence < confidence_threshold ||
        !std::isfinite(detection.box.x) ||
        !std::isfinite(detection.box.y) ||
        !std::isfinite(detection.box.width) ||
        !std::isfinite(detection.box.height) ||
        detection.box.width < config.min_box_width ||
        detection.box.height < config.min_box_height) {
      continue;
    }
    candidates.push_back({MakeTarget(detection), index});
  }
  return candidates;
}

ObjectTargetSelection
ExtractObjectTargetSelection(const std::vector<Detection> &detections,
                             const ObjectTargetConfig &config) {
  ObjectTargetSelection selection;
  const auto ball = ExtractBest(detections, config.ball_class_id,
                                config.ball_confidence, config);
  const auto goal = ExtractBest(detections, config.goal_class_id,
                                config.goal_confidence, config);
  const auto backboard = ExtractBest(detections, config.backboard_class_id,
                                     config.backboard_confidence, config);
  const auto hurdle = ExtractBest(detections, config.hurdle_class_id,
                                  config.hurdle_confidence, config);
  selection.targets.ball = ball.target;
  selection.targets.goal = goal.target;
  selection.targets.backboard = backboard.target;
  selection.targets.hurdle = hurdle.target;
  selection.indices.ball = ball.index;
  selection.indices.goal = goal.index;
  selection.indices.backboard = backboard.index;
  selection.indices.hurdle = hurdle.index;
  return selection;
}

ObjectTargets ExtractObjectTargets(const std::vector<Detection> &detections,
                                   const ObjectTargetConfig &config) {
  return ExtractObjectTargetSelection(detections, config).targets;
}

} // namespace vision_core
