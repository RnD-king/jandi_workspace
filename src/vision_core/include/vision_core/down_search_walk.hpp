#pragma once
#include <algorithm>
namespace vision_core {
// BALL and HURDLE share this count-based search lifecycle. Time never consumes
// a step; only a pending action's real DONE, filtered by the mission transport.
class DownSearchWalk {
public:
  enum class Decision { kMoving,kFound,kExhausted,kStep };
  Decision Observe(bool visible,int hits,int min_hits,int max_steps,
                   bool action_active,bool action_done) {
    found_=found_ || (visible && hits>=min_hits);
    if (action_active && !action_done) return Decision::kMoving;
    if (action_done && pending_) {++completed_;pending_=false;}
    // A qualifying detection on the final DONE wins over the limit.
    if (found_) return Decision::kFound;
    if (completed_>=max_steps) return Decision::kExhausted;
    return Decision::kStep;
  }
  void Issued() { pending_=true; }
  void InvalidateDetection() { found_=false; }
  int Remaining(int max_steps) const { return std::max(0,max_steps-completed_); }
  void Reset() { completed_=0;pending_=found_=false; }
private:
  int completed_{0};
  bool pending_{false},found_{false};
};
} // namespace vision_core
