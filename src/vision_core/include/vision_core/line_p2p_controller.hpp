#pragma once

#include <cstdint>
#include <cstddef>
#include <optional>
#include <vector>

#include "vision_core/cruise_selector.hpp"
#include "vision_core/types.hpp"

namespace vision_core {

// 정상 P2P LINE의 O/H score와 direct 3-action 구간을 정의한다.
struct LineP2pConfig {
  int left_repetitions{}, straight_repetitions{}, right_repetitions{};
  double offset_gain{};
  double heading_gain{};
  double steering_deadband{};
  double in_place_turn_score{}; // Above this absolute O/H score, turn once in WALK.
  double ready_lead_sec{}; // Matches the motion executor's READY remaining time.
  double observation_start_ratio{}; // Fraction of estimated LINE block duration, in [0,1).
  // 판단 실패 또는 recovery 회전 뒤 stationary re-observation 시간이다.
  double failure_observation_sec{};
  // Observation after a LINE loss-recovery turn, separate from 2s loss wait.
  double recovery_observation_sec{};
  // 이 개수 이상의 유효 O/H 표본만 정상 LINE 판단에 사용한다.
  int failure_min_valid_samples{};
  // Loss recovery: current visible centers take priority over the last score.
  // Counts remain cumulative across source switches until a valid guide returns.
  int visible_recovery_max_turns{};
  int visible_recovery_turn_yaw_deg{};
  int last_score_recovery_max_turns{};
  int last_score_recovery_turn_yaw_deg{};
  // Separate limit for WALK corrections when a valid score is too large.
  int recovery_max_turns{};
};

class LineP2pController {
public:
  explicit LineP2pController(const LineP2pConfig &config);

  CruiseDecision Compute(const LineGuide &guide) const;

private:
  LineP2pConfig config_;
};

// Keep the configured start-to-READY window; later valid guides get 1->3 weight.
struct LineWindowStats {
  std::uint64_t action_id{0};
  std::size_t total_frames{0}, valid_frames{0}, window_frames{0}, used_frames{0};
  double estimated_motion_sec{0.0};
  double window_start_sec{0.0}, window_end_sec{0.0}, remaining_sec{0.0};
};
class LineGuideAccumulator {
public:
  LineGuideAccumulator();
  explicit LineGuideAccumulator(int minimum_valid_samples);
  explicit LineGuideAccumulator(const LineP2pConfig &config);
  LineGuideAccumulator(int minimum_valid_samples, double observation_start_ratio);
  void Begin(std::uint64_t action_id, double now_sec);
  void Add(std::uint64_t action_id, double now_sec, const LineGuide &guide);
  std::optional<LineGuide> Finish(std::uint64_t action_id, double now_sec,
                                  double estimated_remaining_sec = 0.0);
  std::optional<LineWindowStats> LastStats() const { return last_stats_; }
  // 짧은 action 뒤의 고정 관측 구간은 전체 유효 표본을 동일 가중 평균한다.
  std::optional<LineGuide> FinishAll(std::uint64_t action_id);
  void Reset();
  bool ActiveFor(std::uint64_t action_id) const;

private:
  struct Sample {
    double now_sec{0.0};
    LineGuide guide;
  };

  std::uint64_t action_id_{0};
  double begin_sec_{0.0};
  std::vector<Sample> samples_;
  std::vector<double> frame_times_;
  std::optional<LineWindowStats> last_stats_;
  int minimum_valid_samples_;
  double observation_start_ratio_;
};

} // namespace vision_core
