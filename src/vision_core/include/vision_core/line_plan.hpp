#pragma once

#include <deque>
#include <optional>
#include <unordered_map>
#include <vector>
#include "vision_core/control_command.hpp"
#include "vision_core/line_p2p_controller.hpp"

namespace vision_core {

// Plans are logical blocks of the configured unit repetitions, not another executor queue. The
// coordinator remains the sole owner of the current and one queued command.
class LinePlan {
public:
  explicit LinePlan(const LineP2pConfig& config) : config_(config) {}
  static bool IsUnit(MissionAction action) {
    return action == MissionAction::kLineLeftUnit ||
           action == MissionAction::kLineStraightUnit ||
           action == MissionAction::kLineRightUnit;
  }
  static bool IsCruise(MissionAction action) {
    return action == MissionAction::kStepForwardLeft ||
           action == MissionAction::kStepForwardFour ||
           action == MissionAction::kStepForwardRight;
  }
  void UpdateConfig(const LineP2pConfig& config) { config_ = config; }
  ActionRequest Begin(MissionAction cruise, std::uint64_t owner = 0);
  ActionRequest Continue(std::uint64_t ready_id = 0) const;
  void Issued(const ControlCommand& command, std::uint64_t parent_id);
  void Feedback(const CommandDeliveryFeedback& feedback, double now, double hold);
  void Observe(double now, const LineGuide& guide);
  bool Contains(std::uint64_t id) const { return units_.count(id) != 0; }
  bool Done(std::uint64_t id) const;
  bool LastUnit(std::uint64_t id) const;
  MissionAction CruiseFor(std::uint64_t id) const;
  bool Active() const;
  bool Outstanding() const;
  bool WindowFinished(std::uint64_t id) const;
  std::optional<LineGuide> Finish(std::uint64_t id, double now, double remaining);
  std::optional<LineWindowStats> Stats(std::uint64_t id) const;
  std::vector<MissionAction> TakeCompleted();
  struct CompletedBlock { MissionAction cruise; std::uint64_t owner; };
  std::vector<CompletedBlock> TakeCompletedBlocks();
  void Abort();
  void ClearEvidence();
  void Reset();
  // Initial ACK is a start estimate. A queued first unit starts at its
  // parent's DONE + configured inter-action hold, never at its storage ACK.
  std::optional<double> StartEstimate(std::uint64_t id) const;

private:
  struct Block {
    std::uint64_t id;
    MissionAction cruise, unit;
    std::uint64_t owner{0};
    int count, issued{0}, completed{0};
    bool aborted{false}, reported{false}, window_finished{false};
    std::optional<double> start;
    LineGuideAccumulator observations;
    std::optional<LineGuide> mean;
    Block(std::uint64_t key, MissionAction old, MissionAction basic, int repeat,
          const LineP2pConfig &config)
        : id(key), cruise(old), unit(basic), count(repeat), observations(config) {}
  };
  struct Unit {
    std::uint64_t block, parent;
    int ordinal;
    bool done{false};
    bool acknowledged{false};
    std::optional<double> parent_start_estimate;
  };
  Block* Find(std::uint64_t id);
  const Block* Find(std::uint64_t id) const;
  LineP2pConfig config_;
  std::uint64_t next_block_{1};
  std::deque<Block> blocks_;
  std::unordered_map<std::uint64_t, Unit> units_;
};
} // namespace vision_core
