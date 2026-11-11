#include "vision_core/scan_body_correction.hpp"
#include <limits>
#include <stdexcept>
using namespace vision_core;
void Check(bool ok) { if(!ok) throw std::runtime_error("scan correction contract failed"); }
int main() {
  const ScanBodyCorrectionConfig c;
  const Intrinsics camera{600,600,320,240};
  auto p=ObjectScanCorrection(25,320,camera,c);
  Check(p && p->left && p->turns==2);
  p=ObjectScanCorrection(-45,320,camera,c);Check(p && !p->left && p->turns==3);
  p=ObjectScanCorrection(30,320,camera,c);Check(p && p->turns==2);
  p=ObjectScanCorrection(30.01,320,camera,c);Check(p && p->turns==3);
  p=ObjectScanCorrection(0,320,camera,c);Check(p && p->turns==0);
  // The image coordinate may change the count or reverse the correction side.
  p=ObjectScanCorrection(45,620,camera,c);Check(p && p->left && p->turns==2);
  p=ObjectScanCorrection(10,620,camera,c);Check(p && !p->left && p->turns==2);
  Check(!ObjectScanCorrection(45,std::nullopt,camera,c));
  auto invalid=camera;invalid.fx=0;
  Check(!ObjectScanCorrection(45,320,invalid,c));
  Check(!ObjectScanCorrection(45,std::numeric_limits<double>::quiet_NaN(),camera,c));
  Check(ScanFirstLeft(100,320,50,false));Check(!ScanFirstLeft(500,320,50,true));
  Check(!ScanFirstLeft(320,320,50,false));Check(ScanFirstLeft(std::nullopt,320,50,true));
}
