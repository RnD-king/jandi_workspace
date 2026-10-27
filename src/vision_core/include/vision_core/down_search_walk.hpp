#pragma once
#include <algorithm>
#include <optional>
namespace vision_core {
// BALL and HURDLE share this count-based search lifecycle. Time never consumes
// a step; only a pending action's real DONE, filtered by the mission transport.
class DownSearchWalk {
public:
  enum class Decision { kMoving,kFound,kExhausted,kStep,kWaiting };
  Decision Observe(bool visible,int hits,int min_hits,int max_steps,
                   bool action_active,bool action_done,
                   double now_sec,double inter_action_hold_sec) {
    found_=found_ || (visible && hits>=min_hits);
    if (action_active && !action_done) return Decision::kMoving;
    if (action_done && pending_) {
      ++completed_;pending_=false;last_done_sec_=now_sec;
    }
    // A qualifying detection on the final DONE wins over the limit.
    if (found_) return Decision::kFound;
    if (completed_>=max_steps) return Decision::kExhausted;
    // First step has no added wait. Re-evaluate the latest tuning relative to
    // real DONE; duplicate DONE while waiting must not extend the deadline.
    if (last_done_sec_ && now_sec+1e-9<*last_done_sec_+inter_action_hold_sec)
      return Decision::kWaiting;
    return Decision::kStep;
  }
  void Issued() { pending_=true; }
  bool Found() const { return found_; }
  void InvalidateDetection() { found_=false; }
  int Remaining(int max_steps) const { return std::max(0,max_steps-completed_); }
  void ShiftTimers(double duration) {
    if (last_done_sec_) *last_done_sec_+=duration;
  }
  void Reset() { completed_=0;pending_=found_=false;last_done_sec_.reset(); }
private:
  int completed_{0};
  bool pending_{false},found_{false};
  std::optional<double> last_done_sec_;
};
} // namespace vision_core
