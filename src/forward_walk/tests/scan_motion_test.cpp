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
  assert(IsReadyWalkAction(34) && IsReadyWalkAction(36) && !IsReadyWalkAction(35));
  WalkInterActionHold walk_hold;walk_hold.SetDuration(.2);walk_hold.Complete(36,start);
  assert(walk_hold.Waiting(start+std::chrono::milliseconds(100)));
  assert(!walk_hold.Waiting(start+std::chrono::milliseconds(250)));
  camera.StartScan(true,45,0.5,0.5,0.5,positions,start);
  assert(camera.ScanActive());
  assert(camera.ScanState()==CameraMotion::ScanPhase::Outbound);
  for(int i=0;i<10;++i) camera.Update(start+std::chrono::milliseconds(21*i));
  const double paused=camera.ScanCommandedYawDeg();
  assert(paused>0.0 && paused<45.0);
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
  // A ball can become centered on the return sweep rather than outbound.
  // HOLD must stop the current return target, and RESUME must not reverse
  // the camera back toward the far (45-degree) scan limit.
  CameraMotion return_scan(cfg);
  const auto second_start=start+std::chrono::seconds(10);
  return_scan.StartScan(true,45,0.5,0.5,0.5,positions,second_start);
  bool return_sweep=false;
  auto return_at=second_start;
  for(int i=0;i<95;++i) {
    return_at=second_start+std::chrono::milliseconds(21*i);
    return_scan.Update(return_at);
    const double angle=return_scan.ScanCommandedYawDeg();
    if(return_scan.ScanState()==CameraMotion::ScanPhase::Returning &&
        angle < 40.0 && angle > 5.0) {
      return_sweep=true;
      break;
    }
  }
  assert(return_sweep);
  const auto held_angle=return_scan.ScanCommandedYawDeg();
  assert(return_scan.HoldScan());
  assert(return_scan.ScanState()==CameraMotion::ScanPhase::Paused);
  for(int i=1;i<=10;++i)
    return_scan.Update(return_at+std::chrono::milliseconds(21*i));
  assert(std::abs(return_scan.ScanCommandedYawDeg()-held_angle)<0.01);
  const auto resumed_at=return_at+std::chrono::milliseconds(230);
  assert(return_scan.ResumeScan(resumed_at));
  assert(return_scan.ScanState()==CameraMotion::ScanPhase::Returning);
  bool return_finished=false;
  for(int i=0;i<90;++i) {
    const auto result=return_scan.Update(resumed_at+std::chrono::milliseconds(21*i));
    assert(return_scan.ScanCommandedYawDeg()<=held_angle+0.5);
    if(result==CameraMotion::Result::Verify) {
      return_finished=true;
      break;
    }
  }
  assert(return_finished);
  assert(return_scan.ConfirmArrival(cfg.forward));
  assert(!return_scan.ScanActive());
  // Right scan must use the opposite (negative) yaw delta.
  CameraMotion right_scan(cfg);
  right_scan.StartScan(false,45,0.5,0.5,0.5,positions,start);
  for(int i=0;i<10;++i) right_scan.Update(start+std::chrono::milliseconds(21*i));
  assert(right_scan.ScanCommandedYawDeg()<0.0);
  // The normal forward/down/goal interpolation remains available.
  camera.Start(1,positions,start);
  assert(!camera.ScanActive());
  return 0;
}
