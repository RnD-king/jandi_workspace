#pragma once
#include <cstdint>
#include <deque>
#include "vision_core/object_association_tracker.hpp"

namespace vision_core {
struct BallCandidateTrackingConfig {
  bool enabled{};
  double max_center_distance_px{};
  double max_size_log_ratio{};
  int missing_frame_limit{};
  int max_tracks{};
  bool operator==(const BallCandidateTrackingConfig& c) const {
    return enabled==c.enabled && max_center_distance_px==c.max_center_distance_px &&
        max_size_log_ratio==c.max_size_log_ratio && missing_frame_limit==c.missing_frame_limit && max_tracks==c.max_tracks;
  }
};
using ObjectCandidateTrackingConfig = BallCandidateTrackingConfig;
struct BallCandidateTrack {
  std::uint64_t id{};
  ObjectTarget target;
  std::optional<std::size_t> detection_index;
  std::uint64_t last_frame{};
  int missing_frames{};
  std::deque<bool> hits;
  std::deque<std::optional<Point2>> positions;
  double center_distance_px{}, size_log_ratio{};
  int HitCount() const;
  std::optional<double> MeanU() const;
};
// One detection/track per image. Geometry-sorted greedy edges make matching
// deterministic independently of detector output order. Misses never emit boxes.
class BallCandidateTracker {
public:
  void Update(const std::vector<ObjectTargetCandidate>& candidates,
              const BallCandidateTrackingConfig&, const ObjectAssociationConfig&,
              int window, double max_entry_v);
  const BallCandidateTrack* Find(std::uint64_t id) const;
  const BallCandidateTrack* Best() const;
  const std::vector<BallCandidateTrack>& Tracks() const { return tracks_; }
  void Reset() { tracks_.clear(); } // IDs remain monotonic across observation resets.
private:
  std::vector<BallCandidateTrack> tracks_;
  std::uint64_t next_id_{1}, frame_{0};
};
}
