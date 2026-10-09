#include "vision_core/object_association_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace vision_core {
namespace {

constexpr double kSizeEpsilon = 1e-6;

double SafePositive(double value) {
  return std::isfinite(value) ? std::max(value, kSizeEpsilon)
                              : kSizeEpsilon;
}

double NormalizedCenterDistance(const ObjectTarget &previous,
                                const ObjectTarget &candidate,
                                int image_width, int image_height) {
  const double diagonal = std::max(
      1.0, std::hypot(static_cast<double>(std::max(0, image_width)),
                      static_cast<double>(std::max(0, image_height))));
  return std::hypot(candidate.center_px.u - previous.center_px.u,
                    candidate.center_px.v - previous.center_px.v) /
         diagonal;
}

// width/height 비율을 각각 log 공간에서 비교한다. log ratio는 확대와 축소에
// 대칭이고, epsilon으로 0 또는 비정상 크기의 나눗셈을 피한다.
double SafeSizeLogRatio(const ObjectTarget &previous,
                        const ObjectTarget &candidate) {
  const double width_change = std::abs(std::log(
      SafePositive(candidate.width_px) / SafePositive(previous.width_px)));
  const double height_change = std::abs(std::log(
      SafePositive(candidate.height_px) / SafePositive(previous.height_px)));
  return 0.5 * (width_change + height_change);
}

} // namespace

ObjectAssociationTracker::ObjectAssociationTracker(
    const ObjectAssociationConfig &config)
    : config_(config) {}

ObjectAssociationSelection ObjectAssociationTracker::AcquireInitial(
    const std::vector<ObjectTargetCandidate> &candidates) {
  if (candidates.empty()) return {};

  // 이전 identity가 없을 때는 association 점수를 정의할 수 없다. confidence를
  // 순위에 쓰지 않고 가장 큰 bbox를 시작점으로 삼으며, 동률이면 입력 순서다.
  const auto selected = std::max_element(
      candidates.begin(), candidates.end(),
      [](const ObjectTargetCandidate &lhs, const ObjectTargetCandidate &rhs) {
        return lhs.target.area_px < rhs.target.area_px;
      });
  previous_target_ = selected->target;
  missing_frames_ = 0;
  return {selected->target, selected->detection_index};
}

ObjectAssociationSelection ObjectAssociationTracker::Update(
    const std::vector<ObjectTargetCandidate> &candidates,
    int image_width, int image_height) {
  if (!previous_target_) return AcquireInitial(candidates);

  const double max_center =
      std::max(0.0, config_.max_center_distance_norm);
  const double max_size = std::max(0.0, config_.max_size_log_ratio);
  const double center_weight =
      std::max(0.0, config_.center_distance_weight);
  const double size_weight = std::max(0.0, config_.size_change_weight);

  const ObjectTargetCandidate *best = nullptr;
  double best_score = std::numeric_limits<double>::infinity();
  for (const auto &candidate : candidates) {
    const double center_distance = NormalizedCenterDistance(
        *previous_target_, candidate.target, image_width, image_height);
    const double size_change =
        SafeSizeLogRatio(*previous_target_, candidate.target);
    if (!std::isfinite(center_distance) || !std::isfinite(size_change) ||
        center_distance > max_center || size_change > max_size) {
      continue;
    }
    // confidence는 후보 필터 통과 이후 association에 사용하지 않는다.
    const double score = center_weight * center_distance +
                         size_weight * size_change;
    if (score < best_score) {
      best = &candidate;
      best_score = score;
    }
  }

  if (best != nullptr) {
    previous_target_ = best->target;
    missing_frames_ = 0;
    return {best->target, best->detection_index};
  }

  ++missing_frames_;
  if (missing_frames_ <= std::max(0, config_.missing_frame_limit)) {
    // identity만 유지한다. 이전 bbox를 현재 검출로 반환하지 않는다.
    return {};
  }

  Reset();
  // 유예가 끝난 프레임에 후보가 있었다면 새 identity로 즉시 시작한다.
  return AcquireInitial(candidates);
}

void ObjectAssociationTracker::Reset() {
  previous_target_.reset();
  missing_frames_ = 0;
}

} // namespace vision_core
