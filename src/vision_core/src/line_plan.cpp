#include "vision_core/line_plan.hpp"
#include <algorithm>
#include <cmath>

namespace vision_core {
namespace {
ActionRequest UnitRequest(MissionAction action) {
  return {action, ActionCategory::kLocomotion, 0, true};
}
}
LinePlan::Block* LinePlan::Find(std::uint64_t id) {
  for (auto& block : blocks_) if (block.id == id) return &block;
  return nullptr;
}
const LinePlan::Block* LinePlan::Find(std::uint64_t id) const {
  for (const auto& block : blocks_) if (block.id == id) return &block;
  return nullptr;
}
ActionRequest LinePlan::Begin(MissionAction cruise, std::uint64_t owner) {
  if (!IsCruise(cruise)) return {};
  // Retain recent IDs for idempotent late feedback, without unbounded growth.
  while (blocks_.size() >= 32 && blocks_.front().reported) {
    const auto key = blocks_.front().id;
    for (auto it = units_.begin(); it != units_.end();) {
      if (it->second.block == key) it = units_.erase(it); else ++it;
    }
    blocks_.pop_front();
  }
  const bool left = cruise == MissionAction::kStepForwardLeft;
  const bool right = cruise == MissionAction::kStepForwardRight;
  const auto unit = left ? MissionAction::kLineLeftUnit :
      right ? MissionAction::kLineRightUnit : MissionAction::kLineStraightUnit;
  const int count = left ? config_.left_repetitions :
      right ? config_.right_repetitions : config_.straight_repetitions;
  blocks_.emplace_back(next_block_++, cruise, unit, count, config_);
  blocks_.back().owner=owner;
  return UnitRequest(unit);
}
ActionRequest LinePlan::Continue(std::uint64_t ready_id) const {
  if (ready_id) {
    const auto it = units_.find(ready_id);
    if (it == units_.end() || it->second.done) return {};
    const auto* block = Find(it->second.block);
    if (block && !block->aborted && block->issued == it->second.ordinal &&
        block->issued < block->count) return UnitRequest(block->unit);
    return {};
  }
  if (Outstanding()) return {};
  for (const auto& block : blocks_)
    if (!block.aborted && block.issued < block.count && block.completed == block.issued)
      return UnitRequest(block.unit);
  return {};
}
void LinePlan::Issued(const ControlCommand& command, std::uint64_t parent_id) {
  if (command.command_type != CommandType::kAction || !IsUnit(command.action) ||
      command.action_id == 0 || Contains(command.action_id)) return;
  for (auto& block : blocks_) {
    if (!block.aborted && block.unit == command.action && block.issued < block.count) {
      units_.emplace(command.action_id, Unit{block.id, parent_id, ++block.issued, false});
      return;
    }
  }
}
void LinePlan::Feedback(const CommandDeliveryFeedback& feedback, double now, double hold) {
  const auto it = units_.find(feedback.action_id);
  if (it == units_.end() || !std::isfinite(now)) return;
  auto& unit = it->second;
  auto* block = Find(unit.block);
  if (!block) return;
  if (!unit.done && feedback.acknowledged && !feedback.done && !feedback.ready) {
    unit.acknowledged = true;
    if (unit.ordinal == 1 && !block->start) {
      if (unit.parent == 0) block->start = now;
      else if (unit.parent_start_estimate)
        block->start = std::max(now, *unit.parent_start_estimate);
    }
  }
  if (!feedback.done || unit.done) return;
  unit.done = true;
  ++block->completed;
  for (auto& successor : units_) {
    if (successor.second.parent != feedback.action_id || successor.second.ordinal != 1) continue;
    auto* next = Find(successor.second.block);
    successor.second.parent_start_estimate = now + std::max(0.0, hold);
    if (next && !next->start && successor.second.acknowledged)
      next->start = successor.second.parent_start_estimate;
  }
}
void LinePlan::Observe(double now, const LineGuide& guide) {
  for (auto& block : blocks_) {
    if (block.aborted || block.window_finished || !block.start || now < *block.start ||
        block.completed == block.count) continue;
    if (!block.observations.ActiveFor(block.id)) block.observations.Begin(block.id, *block.start);
    block.observations.Add(block.id, now, guide);
  }
}
bool LinePlan::Done(std::uint64_t id) const {
  const auto it = units_.find(id);
  return it != units_.end() && it->second.done;
}
bool LinePlan::LastUnit(std::uint64_t id) const {
  const auto it = units_.find(id);
  if (it == units_.end()) return false;
  const auto* block = Find(it->second.block);
  return block && !block->aborted && it->second.ordinal == block->count;
}
bool LinePlan::Active() const {
  for (const auto& block : blocks_)
    if (!block.aborted && block.completed < block.count) return true;
  return false;
}
MissionAction LinePlan::CruiseFor(std::uint64_t id) const {
  const auto it = units_.find(id);
  const auto* block = it == units_.end() ? nullptr : Find(it->second.block);
  return block ? block->cruise : MissionAction::kNone;
}
bool LinePlan::Outstanding() const {
  for (const auto& entry : units_) if (!entry.second.done) return true;
  return false;
}
bool LinePlan::WindowFinished(std::uint64_t id) const {
  const auto it = units_.find(id);
  const auto* block = it == units_.end() ? nullptr : Find(it->second.block);
  return block && block->window_finished;
}
std::optional<LineGuide> LinePlan::Finish(std::uint64_t id, double now, double remaining) {
  const auto it = units_.find(id);
  auto* block = it == units_.end() ? nullptr : Find(it->second.block);
  if (!block || block->aborted) return std::nullopt;
  if (!block->window_finished) {
    block->mean = block->observations.Finish(block->id, now, remaining);
    block->window_finished = true;
  }
  return block->mean;
}
std::optional<LineWindowStats> LinePlan::Stats(std::uint64_t id) const {
  const auto it = units_.find(id);
  const auto* block = it == units_.end() ? nullptr : Find(it->second.block);
  return block ? block->observations.LastStats() : std::nullopt;
}
std::vector<MissionAction> LinePlan::TakeCompleted() {
  std::vector<MissionAction> result;
  for (const auto& block:TakeCompletedBlocks()) result.push_back(block.cruise);
  return result;
}
std::vector<LinePlan::CompletedBlock> LinePlan::TakeCompletedBlocks() {
  std::vector<CompletedBlock> result;
  for (auto& block : blocks_) {
    if (!block.reported && block.completed == block.issued &&
        (block.aborted || block.completed == block.count)) {
      if (!block.aborted) result.push_back({block.cruise,block.owner});
      block.reported = true;
    }
  }
  return result;
}
void LinePlan::Abort() {
  for (auto& block : blocks_) {
    if (block.completed < block.count) block.aborted = true;
    block.observations.Reset();
  }
}
void LinePlan::ClearEvidence() {
  for (auto& block : blocks_) {
    block.observations.Reset();
    block.mean.reset();
    // Retain execution origin and DONE counters; new frames must independently
    // satisfy the block window following RESUME.
  }
}
void LinePlan::Reset() { blocks_.clear(); units_.clear(); next_block_ = 1; }
std::optional<double> LinePlan::StartEstimate(std::uint64_t id) const {
  const auto it = units_.find(id);
  const auto* block = it == units_.end() ? nullptr : Find(it->second.block);
  return block ? block->start : std::nullopt;
}
} // namespace vision_core
