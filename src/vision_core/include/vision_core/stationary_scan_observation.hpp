#pragma once
#include <cstdint>
#include <cmath>
#include <optional>

namespace vision_core {
// One bounded window after an encoder-confirmed stop. Misses occupy slots;
// old/duplicate captures never vote, but the deadline still advances.
class StationaryScanObservation {
public:
  enum class Result { kWaiting, kEmpty, kRejected, kConfirmed };
  void Reset() { *this = {}; }
  void Begin(std::uint64_t stop,double now) { Reset();stop_=stop;started_=now; }
  Result Observe(std::uint64_t stop, bool fresh, double now, bool candidate,
                 double dwell, double timeout, int window, int required,
                 std::optional<double> u = std::nullopt,
                 std::optional<double> stationary_since = std::nullopt,
                 std::optional<double> capture = std::nullopt) {
    if (!stop || !std::isfinite(now)) return Result::kWaiting;
    if (stop_ != stop) {
      Reset(); stop_=stop;
      started_=stationary_since && std::isfinite(*stationary_since) &&
          *stationary_since<=now ? *stationary_since : now;
    }
    if (result_ != Result::kWaiting) return result_;
    // A late frame must not retroactively succeed after the deadline.
    if (now-started_>timeout+1e-9)
      return result_=hits_==0 ? Result::kEmpty : Result::kRejected;
    if (fresh && now>last_sample_ && frames_<window &&
        (!capture || (std::isfinite(*capture) && *capture>last_capture_))) {
      if(capture) last_capture_=*capture;
      last_sample_=now; ++frames_;
      if (candidate) {
        ++hits_;
        if (u && std::isfinite(*u)) { sum_u_+=*u; ++coordinate_samples_; }
      }
    }
    if (frames_>=window && now-started_+1e-9>=dwell)
      return result_=hits_>=required ? Result::kConfirmed :
          hits_==0 ? Result::kEmpty : Result::kRejected;
    if (now-started_+1e-9>=timeout)
      return result_=hits_==0 ? Result::kEmpty : Result::kRejected;
    return Result::kWaiting;
  }
  bool Confirmed() const { return result_==Result::kConfirmed; }
  std::optional<double> MeanU() const {
    return coordinate_samples_ ? std::optional<double>(sum_u_/coordinate_samples_) : std::nullopt;
  }
private:
  std::uint64_t stop_{0};
  double started_{0}, last_sample_{-1}, last_capture_{-1}, sum_u_{0};
  int frames_{0},hits_{0},coordinate_samples_{0};
  Result result_{Result::kWaiting};
};
}
