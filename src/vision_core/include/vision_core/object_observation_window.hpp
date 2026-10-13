#pragma once

#include <algorithm>
#include <deque>
#include <optional>
#include "vision_core/types.hpp"

namespace vision_core {
// Detection evidence and coordinates share a bounded camera-view window.
// Missing frames occupy a slot; they never become a zero coordinate sample.
class ObjectObservationWindow {
public:
  void Clear() { samples_.clear(); }
  void Add(const std::optional<ObjectTarget>& sample, int window) {
    samples_.push_back(sample);
    while (samples_.size() > static_cast<std::size_t>(std::max(1, window)))
      samples_.pop_front();
  }
  // Approach keeps the recent-biased mean; fine adjustment treats every
  // stationary, valid observation equally.
  std::optional<ObjectTarget> Mean(bool recent_weighted = true) const {
    std::optional<ObjectTarget> result;
    double total = 0, u=0, v=0, ru=0, rv=0, x=0, y=0, w=0, h=0, pw=0, ph=0, c=0;
    for (std::size_t i=0; i<samples_.size(); ++i) {
      if (!samples_[i]) continue;
      const auto& t = *samples_[i];
      const double weight = recent_weighted
          ? 1.0 + 2.0 * i / std::max<std::size_t>(1, samples_.size()-1)
          : 1.0;
      result=t; total+=weight;
      u+=weight*t.center_px.u; v+=weight*t.center_px.v;
      const auto p=t.center_rectified ? t.rectified_center_px : t.center_px;
      ru+=weight*p.u; rv+=weight*p.v;
      x+=weight*t.box_px.x; y+=weight*t.box_px.y;
      w+=weight*t.box_px.width; h+=weight*t.box_px.height;
      pw+=weight*t.width_px; ph+=weight*t.height_px; c+=weight*t.confidence;
    }
    if (!result) return std::nullopt;
    result->center_px={u/total,v/total};
    result->rectified_center_px={ru/total,rv/total};
    result->box_px={x/total,y/total,w/total,h/total};
    result->width_px=pw/total; result->height_px=ph/total;
    result->area_px=result->width_px*result->height_px; result->confidence=c/total;
    return result;
  }
private:
  std::deque<std::optional<ObjectTarget>> samples_;
};

inline MissionAction ApproachAction(CruiseDirection direction, bool short_walk) {
  if (direction==CruiseDirection::kLeft) return MissionAction::kStepForwardLeftTwo;
  if (direction==CruiseDirection::kRight) return MissionAction::kStepForwardRightTwo;
  if (direction==CruiseDirection::kStraight)
    return short_walk ? MissionAction::kStepForwardThree : MissionAction::kStepForwardFour;
  return MissionAction::kNone;
}
} // namespace vision_core
