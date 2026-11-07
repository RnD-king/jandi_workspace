#include "vision/scan_frame_guard.hpp"
#include <stdexcept>
#include <limits>
void Check(bool value) { if(!value)throw std::runtime_error("stale/duplicate stationary scan frame accepted"); }
int main() {
  vision::ScanFrameGuard guard;
  auto accept=[&](std::uint64_t command,std::uint32_t stop,double captured,double since) {
    return guard.Accept(command,stop,captured,since,2);
  };
  Check(!accept(1,1,9,10));
  Check(!accept(1,1,10,10));
  Check(accept(1,1,10.1,10));
  Check(!accept(1,1,10.1,10));
  Check(!accept(1,1,10.05,10));
  Check(accept(1,1,10.2,10));
  Check(!accept(1,2,10.2,11));
  Check(accept(1,2,11.1,11));
  Check(!accept(2,0,12,11));
  Check(!accept(2,1,12,0));
  Check(!accept(2,1,std::numeric_limits<double>::quiet_NaN(),11));
  Check(accept(2,1,12.1,12));
  Check(!accept(3,1,14,12)); // An expired stop cannot qualify delayed movement images.
  Check(!accept(3,1,14.1,12));
  Check(guard.AcceptDepth(12.1));
  Check(!guard.AcceptDepth(12.1));
  Check(!guard.AcceptDepth(12.0));
  Check(guard.AcceptDepth(12.2));
}
