#include "motion_executor_state.hpp"
#include <cassert>
#include <chrono>
#include <cmath>
using namespace motion_executor;
int main() {
  CameraConfig cfg;
  CameraMotion camera(cfg);
  RawPositions positions{{21,cfg.forward.yaw},{22,cfg.forward.pitch}};
  const auto start=Clock::now();
  camera.StartScan(true,45,0.5,0.5,0.5,positions,start);
  assert(camera.ScanActive());
  assert(camera.ScanState()==CameraMotion::ScanPhase::Outbound);
  for(int i=0;i<10;++i) camera.Update(start+std::chrono::milliseconds(21*i));
  const double paused=camera.ScanCommandedYawDeg();
  assert(paused<0.0 && paused>-45.0);
  assert(camera.HoldScan());
  assert(camera.ScanState()==CameraMotion::ScanPhase::Paused);
  for(int i=10;i<30;++i) camera.Update(start+std::chrono::milliseconds(21*i));
  assert(std::abs(camera.ScanCommandedYawDeg()-paused)<0.01);
  assert(camera.ResumeScan(start+std::chrono::milliseconds(630)));
  assert(camera.ReturnScan(start+std::chrono::milliseconds(640)));
  bool verifying=false;
  for(int i=0;i<100;++i) {
    const auto status=camera.Update(start+std::chrono::milliseconds(640+21*i));
    if(status==CameraMotion::Result::Verify) {verifying=true;break;}
  }
  assert(verifying);
  assert(camera.ConfirmArrival(cfg.forward));
  assert(!camera.ScanActive());
  // The normal forward/down/goal interpolation remains available.
  camera.Start(1,positions,start);
  assert(!camera.ScanActive());
  return 0;
}
