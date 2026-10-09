#include "vision_core/line_p2p_controller.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <limits>

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

  LineGuideAccumulator accumulator;
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
  return 0;
}
