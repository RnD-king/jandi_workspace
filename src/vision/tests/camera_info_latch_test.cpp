#include "vision/camera_info_latch.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <limits>

int main() {
  vision::CameraInfoLatch latch;
  const vision::CameraInfoCalibration valid{640,480,615.0,614.0,318.0,241.0};
  assert(!latch.Ready());
  // CameraInfo can arrive ahead of the first RGB frame.
  latch.ObserveInfo(valid);
  assert(!latch.Ready());
  latch.ObserveImage(640,480);
  assert(latch.Ready());
  assert(latch.MatchesImage(640,480));
  assert(!latch.MatchesImage(1280,720));
  assert(latch.Calibration().fx == 615.0);
  // Later valid CameraInfo, even after a resolution change, has no effect.
  latch.ObserveInfo({640,480,900.0,900.0,100.0,100.0});
  latch.ObserveImage(1280,720);
  assert(latch.Calibration().fx == 615.0 && latch.Calibration().width == 640);

  vision::CameraInfoLatch delayed;
  delayed.ObserveImage(640,480);
  delayed.ObserveInfo({640,480,0.0,614.0,318.0,241.0});
  delayed.ObserveInfo({640,480,std::numeric_limits<double>::quiet_NaN(),614.0,318.0,241.0});
  delayed.ObserveInfo({640,480,615.0,614.0,std::numeric_limits<double>::infinity(),241.0});
  delayed.ObserveInfo({1280,720,615.0,614.0,318.0,241.0});
  assert(!delayed.Ready());
  delayed.ObserveInfo(valid);
  assert(delayed.Ready());
  assert(delayed.Calibration().fy == 614.0);

  vision::CameraInfoLatch frame_after_invalid;
  frame_after_invalid.ObserveInfo({640,480,-1.0,614.0,318.0,241.0});
  frame_after_invalid.ObserveImage(640,480);
  assert(!frame_after_invalid.Ready());
  frame_after_invalid.ObserveInfo(valid);
  assert(frame_after_invalid.Ready());
  return 0;
}
