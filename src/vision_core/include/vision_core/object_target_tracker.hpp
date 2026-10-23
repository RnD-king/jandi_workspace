#pragma once

#include <algorithm>
#include <cmath>
#include "vision_core/ball_candidate_tracker.hpp"

namespace vision_core {
// Reuse the BALL candidate matcher, with independent identities and thresholds
// for each class. A locked identity cannot silently expire into another object.
class ObjectTargetTracker {
public:
  ObjectAssociationSelection Observe(
      const std::vector<ObjectTargetCandidate>& detections,
      const ObjectCandidateTrackingConfig& config, const ObjectAssociationConfig& weights,
      int width, int height, int window, double max_entry_v,
      bool active, bool recovery, bool manual_mean = false, std::uint64_t mean_id = 0) {
    identity_changed_ = false;
    auto association = weights;
    if (config.enabled) {
      association.max_center_distance_norm = config.max_center_distance_px /
          std::max(1.0, std::hypot(width, height));
      association.max_size_log_ratio = config.max_size_log_ratio;
      association.missing_frame_limit = config.missing_frame_limit;
    }
    association_.UpdateConfig(association);
    if (!config.enabled) return association_.Update(detections, width, height);
    if (manual_mean) {
      if (!mean_id || mean_id != id_ || detections.size() != 1) return {};
      if (locked_) association_.Seed(detections.front().target);
      return {detections.front().target, detections.front().detection_index};
    }
    if (recovery && !was_recovery_) {
      candidates_.Reset(); association_.Reset(); selected_.reset();
      id_ = 0; locked_ = false; reacquiring_ = true;
    }
    was_recovery_ = recovery;
    if (locked_) {
      const auto result = association_.Update(detections, width, height, false);
      if (selected_) {
        selected_->detection_index = result.detection_index;
        selected_->missing_frames = association_.MissingFrames();
        if (result.target) selected_->target = *result.target;
      }
      return result;
    }
    reacquiring_ = active;
    candidates_.Update(detections, config, weights, window, max_entry_v);
    const auto* chosen = active ? candidates_.Find(id_) : nullptr;
    if (!chosen) chosen = candidates_.Best();
    const auto next_id = chosen ? chosen->id : 0;
    identity_changed_ = next_id != id_;
    id_ = next_id;
    selected_ = chosen ? std::optional<BallCandidateTrack>(*chosen) : std::nullopt;
    if (!chosen || !chosen->detection_index) return {};
    return {chosen->target, chosen->detection_index};
  }
  void Lock() {
    if (locked_ || !selected_ || !selected_->detection_index) return;
    association_.Seed(selected_->target);
    locked_ = true; reacquiring_ = false; candidates_.Reset();
  }
  void Reset() {
    candidates_.Reset(); association_.Reset(); selected_.reset();
    id_ = 0; locked_ = reacquiring_ = was_recovery_ = identity_changed_ = false;
  }
  void ClearUnpinned() { if (!locked_ && !reacquiring_) Reset(); }
  std::uint64_t TargetId() const { return id_; }
  bool Locked() const { return locked_; }
  bool Reacquiring() const { return reacquiring_; }
  bool IdentityChanged() const { return identity_changed_; }
  const std::optional<BallCandidateTrack>& Selected() const { return selected_; }
  std::vector<BallCandidateTrack> Tracks() const {
    auto tracks = candidates_.Tracks();
    if (tracks.empty() && selected_) tracks.push_back(*selected_);
    return tracks;
  }
private:
  BallCandidateTracker candidates_;
  ObjectAssociationTracker association_;
  std::optional<BallCandidateTrack> selected_;
  std::uint64_t id_{0};
  bool locked_{false}, reacquiring_{false}, was_recovery_{false}, identity_changed_{false};
};
} // namespace vision_core
