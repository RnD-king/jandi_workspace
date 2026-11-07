#pragma once
#include <cmath>
#include <cstdint>
namespace vision {
class ScanFrameGuard {
public:
  bool Accept(std::uint64_t command, std::uint32_t stop, double captured,
              double stationary_since, double timeout) {
    if(!command || !stop || !std::isfinite(captured) ||
       !std::isfinite(stationary_since) || !std::isfinite(timeout) || timeout<=0 ||
       stationary_since<=0 || captured<=stationary_since || captured>=stationary_since+timeout)
      return false;
    if(command!=command_ || stop!=stop_) {
      command_=command;stop_=stop;last_capture_=stationary_since;
    }
    if(captured<=last_capture_)return false;
    last_capture_=captured;return true;
  }
  bool AcceptDepth(double captured) {
    if(!std::isfinite(captured) || captured<=0 || captured<=last_depth_)return false;
    last_depth_=captured;return true;
  }
private:
  std::uint64_t command_{0};
  std::uint32_t stop_{0};
  double last_capture_{0};
  double last_depth_{0};
};
}
