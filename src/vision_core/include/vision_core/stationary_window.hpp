#pragma once

#include <algorithm>
#include <cmath>
#include <deque>

namespace vision_core {

// Fast race-round-2 verification ONLY. The normal moving/entry windows
// (10/5, 10/7, LINE READY and dedicated scan evidence) remain unchanged.
inline constexpr int kFastStationaryWindow = 8;

inline int FastStationaryMinHits(int original_min_hits) {
  // Keep the agreed stability ratios for stationary verification.
  switch (original_min_hits) {
  case 3: return 3;
  case 4: return 3;
  case 5: return 4;
  case 7: return 6;
  default:
    return std::clamp(static_cast<int>(std::lround(
        static_cast<double>(original_min_hits) * 0.8)),
        1, kFastStationaryWindow);
  }
}

inline bool FastStationaryQualified(const std::deque<bool>& hits,
                                   int original_min_hits) {
  return hits.size() >= static_cast<std::size_t>(kFastStationaryWindow) &&
      std::count(hits.end() - kFastStationaryWindow, hits.end(), true) >=
          FastStationaryMinHits(original_min_hits);
}

}  // namespace vision_core
