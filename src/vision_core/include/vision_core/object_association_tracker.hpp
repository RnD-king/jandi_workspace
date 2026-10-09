#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include "vision_core/object_target_extractor.hpp"

namespace vision_core {

// association에만 사용하는 설정이다. controller의 lost_frames와 달리
// missing_frame_limit은 이전 identity를 후보 비교용으로 보존하는 기간이다.
struct ObjectAssociationConfig {
  int missing_frame_limit{};
  double max_center_distance_norm{};
  double max_size_log_ratio{};
  double center_distance_weight{};
  double size_change_weight{};
};

struct ObjectAssociationSelection {
  std::optional<ObjectTarget> target;
  std::optional<std::size_t> detection_index;
};

// 한 클래스의 identity 하나를 유지한다. 누락 중에는 마지막 bbox를 내부 비교
// 기준으로만 보존하고, 호출자에게 실제 검출처럼 반환하지 않는다.
class ObjectAssociationTracker {
public:
  ObjectAssociationTracker() = default;
  explicit ObjectAssociationTracker(const ObjectAssociationConfig &config);

  ObjectAssociationSelection
  Update(const std::vector<ObjectTargetCandidate> &candidates,
         int image_width, int image_height);
  void Reset();

  bool HasIdentity() const { return previous_target_.has_value(); }
  int MissingFrames() const { return missing_frames_; }

private:
  ObjectAssociationSelection
  AcquireInitial(const std::vector<ObjectTargetCandidate> &candidates);

  ObjectAssociationConfig config_;
  std::optional<ObjectTarget> previous_target_;
  int missing_frames_{0};
};

} // namespace vision_core
