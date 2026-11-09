#pragma once

#include <algorithm>
#include <cmath>
#include <deque>

namespace vision_core {

// All FAST observation windows, including moving/READY/entry/scan and
// stationary verification, use 8 frames. The FAST YAML already contains
// proportionally scaled hit thresholds; do not scale them a second time.
inline constexpr int kFastStationaryWindow = 8;

inline int FastStationaryMinHits(int configured_min_hits) {
  return std::clamp(configured_min_hits, 1, kFastStationaryWindow);
}

inline bool FastStationaryQualified(const std::deque<bool>& hits,
                                   int original_min_hits) {
  return hits.size() >= static_cast<std::size_t>(kFastStationaryWindow) &&
      std::count(hits.end() - kFastStationaryWindow, hits.end(), true) >=
          FastStationaryMinHits(original_min_hits);
}

}  // namespace vision_core
