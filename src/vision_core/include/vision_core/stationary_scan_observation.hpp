#pragma once
#include <cstdint>
#include <algorithm>

namespace vision_core {
// A bounded, fresh window at ONE stationary head pose. The first candidate
// starts a new N-frame confirmation; earlier misses never consume its votes.
class StationaryScanObservation {
public:
  enum class Result { kWaiting, kEmpty, kRejected, kConfirmed };
  void Reset() { *this = {}; }
  Result Observe(std::uint64_t stop, bool fresh, double now, bool candidate,
                 double dwell, double timeout, int window, int required) {
    if (!stop || !fresh) return Result::kWaiting;
    if (stop_ != stop) { Reset(); stop_=stop; started_=now; }
    if (result_ != Result::kWaiting) return result_;
    if (now <= last_sample_) return Result::kWaiting;
    last_sample_=now;
    if (candidate && !candidate_) candidate_=true;
    if (candidate_ && frames_<window) { ++frames_; if (candidate) ++hits_; }
    if (frames_ >= window && now-started_+1e-9 >= dwell)
      return result_=hits_>=required ? Result::kConfirmed : Result::kRejected;
    if (now-started_ >= timeout) return result_=Result::kRejected;
    if (!candidate_ && now-started_+1e-9 >= dwell) return result_=Result::kEmpty;
    return Result::kWaiting;
  }
  bool Confirmed() const { return result_==Result::kConfirmed; }
private:
  std::uint64_t stop_{0};
  double started_{0}, last_sample_{-1};
  bool candidate_{false};
  int frames_{0},hits_{0};
  Result result_{Result::kWaiting};
};
}
