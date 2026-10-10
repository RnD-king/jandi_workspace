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
  double offset_gain{};
  double heading_gain{};
  double steering_deadband{};
  double ready_lead_sec{}; // Matches the motion executor's READY remaining time.
  // 판단 실패 또는 recovery 회전 뒤 stationary re-observation 시간이다.
  double failure_observation_sec{};
  // Observation after a LINE loss-recovery turn, separate from 2s loss wait.
  double recovery_observation_sec{};
  // 이 개수 이상의 유효 O/H 표본만 정상 LINE 판단에 사용한다.
  int failure_min_valid_samples{};
  // 방향 기억 없이 정지 재관측하는 추가 횟수와 방향 기억을 따라 회전하는
  // 최대 횟수다. 한도를 넘으면 명시적 Reset 전까지 FINAL HOLD다.
  int no_evidence_max_retries{};
  int recovery_max_turns{};
  int recovery_turn_yaw_deg{};
};

class LineP2pController {
public:
  explicit LineP2pController(const LineP2pConfig &config);

  CruiseDecision Compute(const LineGuide &guide) const;

private:
  LineP2pConfig config_;
};

// Keep the 40%-to-READY window of long LINE motion; later valid guides get 1->3 weight.
struct LineWindowStats {
  std::uint64_t action_id{0};
  std::size_t total_frames{0}, valid_frames{0}, window_frames{0}, used_frames{0};
  double estimated_motion_sec{0.0};
  double window_start_sec{0.0}, window_end_sec{0.0}, remaining_sec{0.0};
};
class LineGuideAccumulator {
public:
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
};

} // namespace vision_core
