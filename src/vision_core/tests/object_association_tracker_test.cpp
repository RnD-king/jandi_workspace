#include "vision_core/object_association_tracker.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <iostream>

namespace {

using namespace vision_core;

ObjectTargetCandidate Candidate(std::size_t index, double center_u,
                                double center_v, double width, double height,
                                double confidence = 0.7) {
  ObjectTarget target;
  target.class_id = 1;
  target.confidence = confidence;
  target.width_px = width;
  target.height_px = height;
  target.area_px = width * height;
  target.center_px = {center_u, center_v};
  target.rectified_center_px = target.center_px;
  target.box_px = {center_u - 0.5 * width, center_v - 0.5 * height,
                   width, height};
  return {target, index};
}

ObjectAssociationConfig Config() {
  ObjectAssociationConfig config;
  config.missing_frame_limit = 5;
  config.max_center_distance_norm = 0.50;
  config.max_size_log_ratio = 2.0;
  config.center_distance_weight = 1.0;
  config.size_change_weight = 0.25;
  return config;
}

void TestConfidenceFiltersOnlyAndDoesNotRankAssociation() {
  ObjectTargetConfig filter;
  filter.ball_class_id = 1;
  filter.ball_confidence = 0.60;
  filter.min_box_width = 1.0;
  filter.min_box_height = 1.0;
  const std::vector<Detection> detections{
      {{40.0, 40.0, 20.0, 20.0}, 0.59, 1},
      {{42.0, 40.0, 20.0, 20.0}, 0.61, 1},
      {{75.0, 40.0, 20.0, 20.0}, 0.99, 1},
  };
  const auto candidates = ExtractObjectTargetCandidates(
      detections, filter.ball_class_id, filter.ball_confidence, filter);
  assert(candidates.size() == 2);

  ObjectAssociationTracker tracker(Config());
  auto selected = tracker.Update({Candidate(10, 50.0, 50.0, 20.0, 20.0)},
                                 100, 100);
  assert(selected.detection_index == 10);
  selected = tracker.Update(candidates, 100, 100);
  // 더 낮은 confidence라도 이전 중심에 가까운 index 1을 연결한다.
  assert(selected.detection_index == 1);
  assert(selected.target->confidence == 0.61);
}

void TestCenterAndSafeSizeChangeAreTheOnlyScoreTerms() {
  ObjectAssociationTracker tracker(Config());
  tracker.Update({Candidate(0, 50.0, 50.0, 20.0, 20.0)}, 100, 100);

  const auto selected = tracker.Update(
      {Candidate(1, 51.0, 50.0, 40.0, 40.0, 0.99),
       Candidate(2, 53.0, 50.0, 20.0, 20.0, 0.61)},
      100, 100);
  // index 1은 중심은 가깝지만 크기 변화가 커서 index 2보다 점수가 나쁘다.
  assert(selected.detection_index == 2);

  ObjectAssociationTracker zero_size_tracker(Config());
  zero_size_tracker.Update({Candidate(3, 20.0, 20.0, 0.0, 0.0)}, 100, 100);
  const auto safe = zero_size_tracker.Update(
      {Candidate(4, 20.0, 20.0, 0.0, 0.0)}, 100, 100);
  assert(safe.detection_index == 4);
}

void TestMissingKeepsOnlyIdentityForFiveFrames() {
  ObjectAssociationTracker tracker(Config());
  tracker.Update({Candidate(0, 50.0, 50.0, 20.0, 20.0)}, 100, 100);

  for (int missing = 1; missing <= 5; ++missing) {
    const auto selected = tracker.Update({}, 100, 100);
    assert(!selected.target);
    assert(!selected.detection_index);
    assert(tracker.HasIdentity());
    assert(tracker.MissingFrames() == missing);
  }

  const auto reappeared = tracker.Update(
      {Candidate(5, 52.0, 50.0, 20.0, 20.0)}, 100, 100);
  assert(reappeared.detection_index == 5);
  assert(tracker.MissingFrames() == 0);

  for (int missing = 1; missing <= 6; ++missing) {
    const auto selected = tracker.Update({}, 100, 100);
    assert(!selected.target);
  }
  assert(!tracker.HasIdentity());

  const auto new_identity = tracker.Update(
      {Candidate(6, 90.0, 90.0, 10.0, 10.0)}, 100, 100);
  assert(new_identity.detection_index == 6);
}

} // namespace

int main() {
  TestConfidenceFiltersOnlyAndDoesNotRankAssociation();
  TestCenterAndSafeSizeChangeAreTheOnlyScoreTerms();
  TestMissingKeepsOnlyIdentityForFiveFrames();
  std::cout << "object association tracker tests passed\n";
  return 0;
}
