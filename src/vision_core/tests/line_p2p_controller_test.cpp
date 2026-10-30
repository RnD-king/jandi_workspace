#include "vision_core/line_p2p_controller.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace {
constexpr double kEps = 1e-12;

vision_core::LineGuide Guide(double offset, double heading,
                             double curvature, bool curvature_valid = true,
                             double confidence = 1.0) {
  vision_core::LineGuide guide;
  guide.offset = offset;
  guide.heading_rad = heading;
  guide.curvature_rad = curvature;
  guide.confidence = confidence;
  guide.valid = true;
  guide.curvature_valid = curvature_valid;
  return guide;
}
} // namespace

int main() {
  using namespace vision_core;
  LineP2pConfig config;
  config.offset_gain = 2.0;
  config.heading_gain = 1.0;
  config.steering_deadband = 0.10;
  const LineP2pController controller(config);

  assert(controller.Compute(Guide(0.0, 0.0, 0.0)).direction ==
         CruiseDirection::kStraight);
  assert(controller.Compute(Guide(-0.10, 0.0, 0.0)).direction ==
         CruiseDirection::kLeft);
  assert(controller.Compute(Guide(0.10, 0.0, 0.0)).direction ==
         CruiseDirection::kRight);

  // O와 H가 반대일 때도 weighted score만 사용한다.
  assert(controller.Compute(Guide(0.10, -0.15, 0.0)).direction ==
         CruiseDirection::kStraight);
  assert(controller.Compute(Guide(0.20, -0.10, 0.0)).direction ==
         CruiseDirection::kRight);

  // curvature 값과 validity는 정상 LINE action에 영향을 주지 않는다.
  const auto no_curve = controller.Compute(Guide(0.10, 0.0, 0.0, false));
  const auto sharp_curve = controller.Compute(Guide(0.10, 0.0, -2.0, true));
  assert(no_curve.direction == sharp_curve.direction);

  LineGuide invalid = Guide(0.0, 0.0, 0.0);
  invalid.valid = false;
  assert(controller.Compute(invalid).direction == CruiseDirection::kNone);
  LineGuide nonfinite = Guide(0.0, 0.0, 0.0);
  nonfinite.offset = std::numeric_limits<double>::quiet_NaN();
  assert(controller.Compute(nonfinite).direction == CruiseDirection::kNone);

  // deadband 경계는 STRAIGHT다.
  LineP2pConfig boundary_config = config;
  boundary_config.offset_gain = 1.0;
  boundary_config.heading_gain = 0.0;
  const LineP2pController boundary(boundary_config);
  assert(boundary.Compute(Guide(-0.10, 0.0, 0.0)).direction ==
         CruiseDirection::kStraight);
  assert(boundary.Compute(Guide(0.10, 0.0, 0.0)).direction ==
         CruiseDirection::kStraight);

  LineGuideAccumulator accumulator(1, .40, 3, 2); // Isolate weighting from sample-count admission.
  accumulator.Begin(7, 0.0);
  accumulator.Add(7, 0.0, Guide(-1.0, -1.0, -1.0));
  accumulator.Add(7, 1.0, Guide(-0.8, -0.8, -0.8));
  accumulator.Add(7, 2.0, Guide(0.20, 0.20, 0.20, false));
  accumulator.Add(7, 3.0, Guide(0.30, 0.30, 0.30, true));
  accumulator.Add(7, 4.0, Guide(0.40, 0.40, 0.40, false));
  const auto accumulated = accumulator.Finish(7, 4.0);
  assert(accumulated.has_value());
  // At 40% of 4s, discard 0s and 1s. Weight 2s/3s/4s by phase.
  const double w2 = 1.0 + 2.0 * (2.0 - 1.6) / 2.4;
  const double w3 = 1.0 + 2.0 * (3.0 - 1.6) / 2.4;
  const double w4 = 3.0;
  const double expected = (0.20*w2 + 0.30*w3 + 0.40*w4) / (w2+w3+w4);
  const auto counts = accumulator.LastStats();
  assert(counts && counts->total_frames == 5 && counts->valid_frames == 5);
  assert(counts->window_frames == 3 && counts->used_frames == 3);
  assert(std::abs(accumulated->offset - expected) < kEps);
  assert(std::abs(accumulated->heading_rad - expected) < kEps);
  assert(accumulated->curvature_valid);
  assert(std::abs(accumulated->curvature_rad - 0.30) < kEps);

  accumulator.Begin(70, 10.0);
  accumulator.Add(70, 10.0, Guide(-1.0, 0.0, 0.0));
  accumulator.Add(70, 10.7, Guide(-1.0, 0.0, 0.0));
  accumulator.Add(70, 11.0, Guide(1.0, 0.0, 0.0));
  auto invalid_frame = Guide(0.0, 0.0, 0.0); invalid_frame.valid = false;
  accumulator.Add(70, 11.3, invalid_frame);
  accumulator.Add(70, 11.75, Guide(1.0, 0.0, 0.0));
  const auto ready_average = accumulator.Finish(70, 11.75, 0.25);
  const auto ready_stats = accumulator.LastStats();
  assert(ready_average && ready_average->offset > 0.99);
  assert(ready_stats && ready_stats->total_frames == 5);
  assert(ready_stats->valid_frames == 4 && ready_stats->window_frames == 3);
  assert(ready_stats->used_frames == 2);
  assert(std::abs(ready_stats->window_start_sec - 10.8) < kEps);
  accumulator.Begin(8, 5.0);
  accumulator.Add(8, 5.0, Guide(-0.30, -0.20, -0.10, false));
  accumulator.Add(8, 5.5, Guide(0.30, 0.20, 0.10, false));
  accumulator.Add(8, 6.0, Guide(0.60, 0.40, 0.20, false));
  const auto all_samples = accumulator.FinishAll(8);
  assert(all_samples.has_value());
  assert(std::abs(all_samples->offset - 0.20) < kEps);
  assert(std::abs(all_samples->heading_rad - (0.40 / 3.0)) < kEps);
  assert(!all_samples->curvature_valid);
  assert(std::abs(all_samples->curvature_rad) < kEps);

  accumulator.Begin(9, 0.0);
  LineGuide malformed = Guide(0.0, 0.0, 0.0);
  malformed.offset = std::numeric_limits<double>::infinity();
  accumulator.Add(9, 0.5, malformed);
  assert(!accumulator.FinishAll(9).has_value());

  // Stationary admission needs five hits inside one complete ten-frame window.
  LineGuideAccumulator stationary(5, 0.0, 10, 5);
  stationary.Begin(10, 0.0);
  for (int i = 0; i < 17; ++i) {
    auto guide = Guide(double(i), 0.0, 0.0);
    guide.valid = i % 4 == 0;
    stationary.Add(10, 0.1 * i, guide);
  }
  assert(stationary.HasFullWindow(10));
  assert(!stationary.FinishAll(10)); // Five isolated hits never form 10/5.

  stationary.Begin(11, 2.0);
  for (int i = 0; i < 9; ++i)
    stationary.Add(11, 2.0 + 0.1 * i, Guide(double(i), 0.0, 0.0));
  assert(!stationary.HasFullWindow(11));
  stationary.Add(11, 2.9, Guide(9.0, 0.0, 0.0));
  assert(stationary.HasFullWindow(11));
  const auto stationary_mean = stationary.FinishAll(11);
  assert(stationary_mean && std::abs(stationary_mean->offset - 4.5) < kEps);

  stationary.Begin(12, 4.0);
  for (int i = 0; i < 15; ++i) {
    auto guide = Guide(double(i), 0.0, 0.0);
    guide.valid = i < 5 || i == 12;
    stationary.Add(12, 4.0 + 0.1 * i, guide);
  }
  const auto latched_mean = stationary.FinishAll(12);
  assert(latched_mean && std::abs(latched_mean->offset - 22.0 / 6.0) < kEps);

  // A custom 20% start keeps the same 1->3 time weights through READY.
  LineGuideAccumulator earlier(3, .20, 3, 3);
  earlier.Begin(90, 10.0);
  earlier.Add(90, 10.79, Guide(-100.0, 0.0, 0.0));
  earlier.Add(90, 10.8, Guide(1.0, 0.0, 0.0));
  earlier.Add(90, 12.275, Guide(2.0, 0.0, 0.0));
  earlier.Add(90, 13.75, Guide(3.0, 0.0, 0.0));
  const auto earlier_mean = earlier.Finish(90, 13.75, .25);
  assert(earlier_mean && std::abs(earlier_mean->offset - 14.0/6.0) < kEps);
  const auto earlier_stats = earlier.LastStats();
  assert(earlier_stats && earlier_stats->used_frames == 3);
  assert(std::abs(earlier_stats->window_start_sec - 10.8) < kEps);
  assert(std::abs(earlier_stats->remaining_sec - .25) < kEps);

  LineGuideAccumulator from_start(1, 0.0, 2, 1);
  from_start.Begin(91, 0.0);
  from_start.Add(91, 0.0, Guide(1.0, 0.0, 0.0));
  from_start.Add(91, 1.0, Guide(3.0, 0.0, 0.0));
  const auto full_mean = from_start.Finish(91, 1.0);
  assert(full_mean && std::abs(full_mean->offset - 2.5) < kEps);

  // A configured start after READY gives no evidence; never widen it silently.
  LineGuideAccumulator too_late(1, .99, 1, 1);
  too_late.Begin(92, 0.0);
  too_late.Add(92, 3.75, Guide(1.0, 0.0, 0.0));
  assert(!too_late.Finish(92, 3.75, .25));
  for (double ratio : {-0.1, 1.0, std::numeric_limits<double>::infinity(),
                       std::numeric_limits<double>::quiet_NaN()}) {
    bool rejected = false;
    try { LineGuideAccumulator invalid_ratio(1, ratio); }
    catch (const std::invalid_argument&) { rejected = true; }
    assert(rejected);
  }
  return 0;
}
