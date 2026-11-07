#include "vision_core/stationary_scan_observation.hpp"
#include <stdexcept>
using Window=vision_core::StationaryScanObservation;
using Result=Window::Result;
void Check(bool value) { if(!value)throw std::runtime_error("stationary scan window contract failed"); }
int main() {
  Window w;
  auto sample=[&](unsigned stop,double now,bool candidate,bool fresh=true) {
    return w.Observe(stop,fresh,now,candidate,.5,2,10,5);
  };
  Check(sample(1,0,false)==Result::kWaiting);
  Check(sample(1,.49,false)==Result::kWaiting);
  Check(sample(1,.5,false)==Result::kEmpty);
  w.Reset();
  Check(sample(1,0,false)==Result::kWaiting);
  Check(sample(1,.49,true)==Result::kWaiting);
  // Movement/old frames and repeated frame times cannot create votes.
  for(int i=0;i<20;++i)Check(sample(1,.49,true)==Result::kWaiting);
  Check(sample(1,.50,true,false)==Result::kWaiting);
  for(int i=1;i<9;++i)
    Check(sample(1,.49+.067*i,i<5)==Result::kWaiting);
  Check(sample(1,1.10,false)==Result::kConfirmed); // 5/10, final frame missing.
  Check(sample(1,1.2,false)==Result::kConfirmed);
  Check(sample(2,1.3,false)==Result::kWaiting); // New pose starts a fresh window.
  w.Reset();
  for(int i=0;i<9;++i)Check(sample(1,.067*i,i<4)==Result::kWaiting);
  Check(sample(1,.603,false)==Result::kRejected);
  w.Reset();
  Check(sample(1,0,true)==Result::kWaiting);
  Check(sample(1,2.01,false)==Result::kRejected); // Low FPS is bounded.
}
