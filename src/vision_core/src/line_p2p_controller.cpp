#include "vision_core/line_p2p_controller.hpp"

#include <algorithm>
#include <cmath>

namespace vision_core {
namespace {
double Clamp(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

} // namespace

LineP2pController::LineP2pController(const LineP2pConfig &config)
    : config_(config) {}

CruiseDecision LineP2pController::Compute(const LineGuide &guide) const {
  const bool valid = guide.valid && std::isfinite(guide.offset) &&
                     std::isfinite(guide.heading_rad);
  const double score = config_.offset_gain * guide.offset +
                       config_.heading_gain * guide.heading_rad;
  return SelectCruiseDecision(true, valid, score,
                              config_.steering_deadband);
}

void LineGuideAccumulator::Begin(std::uint64_t action_id, double now_sec) {
  action_id_ = action_id;
  begin_sec_ = std::isfinite(now_sec) ? now_sec : 0.0;
  samples_.clear();
  frame_times_.clear();
  last_stats_.reset();
}

void LineGuideAccumulator::Add(std::uint64_t action_id, double now_sec,
                               const LineGuide &guide) {
  if (!ActiveFor(action_id) || !std::isfinite(now_sec)) return;
  frame_times_.push_back(now_sec);
  if (!guide.valid || !std::isfinite(guide.offset) ||
      !std::isfinite(guide.heading_rad)) return;
  samples_.push_back({now_sec, guide});
}

std::optional<LineGuide>
LineGuideAccumulator::Finish(std::uint64_t action_id, double now_sec,
                             double estimated_remaining_sec) {
  if (!ActiveFor(action_id)) { last_stats_.reset(); return std::nullopt; }
  const double end_sec = std::isfinite(now_sec) ? std::max(begin_sec_, now_sec) : begin_sec_;
  const double remaining = std::isfinite(estimated_remaining_sec)
      ? std::max(0.0, estimated_remaining_sec) : 0.0;
  // READY occurs approximately 'remaining' seconds before the executor's DONE.
  // The first observed frame after ACK approximates the action's start time.
  const double estimated_motion_sec = end_sec - begin_sec_ + remaining;
  const double cutoff = begin_sec_ + 0.40 * estimated_motion_sec;
  const double span = std::max(0.0, end_sec - cutoff);
  LineWindowStats stats;
  stats.action_id = action_id;
  stats.total_frames = frame_times_.size();
  stats.valid_frames = samples_.size();
  stats.estimated_motion_sec = estimated_motion_sec;
  stats.window_start_sec = cutoff;
  stats.window_end_sec = end_sec;
  stats.remaining_sec = remaining;
  for (double time : frame_times_)
    if (time >= cutoff && time <= end_sec) ++stats.window_frames;
  double offset_sum=0, heading_sum=0, curvature_sum=0, curvature_weight=0;
  double confidence_sum=0, total_weight=0;
  for (const Sample& sample : samples_) {
    if (sample.now_sec < cutoff || sample.now_sec > end_sec) continue;
    ++stats.used_frames;
    const double phase = span > 1e-9
        ? Clamp((sample.now_sec - cutoff) / span, 0.0, 1.0) : 1.0;
    const double weight = 1.0 + 2.0 * phase;
    offset_sum += weight * sample.guide.offset;
    heading_sum += weight * sample.guide.heading_rad;
    if (sample.guide.curvature_valid && std::isfinite(sample.guide.curvature_rad)) {
      curvature_sum += weight * sample.guide.curvature_rad;
      curvature_weight += weight;
    }
    if (std::isfinite(sample.guide.confidence))
      confidence_sum += weight * sample.guide.confidence;
    total_weight += weight;
  }
  LineGuide result;
  if (total_weight > 0) {
    result.offset = offset_sum / total_weight;
    result.heading_rad = heading_sum / total_weight;
    if (curvature_weight > 0) {
      result.curvature_rad = curvature_sum / curvature_weight;
      result.curvature_valid = true;
    }
    result.confidence = confidence_sum / total_weight;
    result.valid = true;
  }
  last_stats_ = stats;
  Reset();
  return result.valid ? std::optional<LineGuide>(result) : std::nullopt;
}

std::optional<LineGuide>
LineGuideAccumulator::FinishAll(std::uint64_t action_id) {
  if (!ActiveFor(action_id) || samples_.empty()) {
    Reset();
    return std::nullopt;
  }

  LineGuide result;
  double curvature_count = 0.0;
  for (const Sample &sample : samples_) {
    result.offset += sample.guide.offset;
    result.heading_rad += sample.guide.heading_rad;
    if (sample.guide.curvature_valid &&
        std::isfinite(sample.guide.curvature_rad)) {
      result.curvature_rad += sample.guide.curvature_rad;
      curvature_count += 1.0;
    }
    // confidence는 의도 계산에는 쓰지 않고 진단값으로만 평균한다.
    if (std::isfinite(sample.guide.confidence))
      result.confidence += sample.guide.confidence;
  }
  const double count = static_cast<double>(samples_.size());
  result.offset /= count;
  result.heading_rad /= count;
  if (curvature_count > 0.0) {
    result.curvature_rad /= curvature_count;
    result.curvature_valid = true;
  }
  result.confidence /= count;
  result.valid = true;
  Reset();
  return result;
}

void LineGuideAccumulator::Reset() {
  action_id_ = 0;
  begin_sec_ = 0.0;
  samples_.clear();
  frame_times_.clear();
}

bool LineGuideAccumulator::ActiveFor(std::uint64_t action_id) const {
  return action_id != 0 && action_id_ == action_id;
}

} // namespace vision_core
