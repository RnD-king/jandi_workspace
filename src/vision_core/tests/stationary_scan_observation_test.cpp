#include "vision_core/stationary_scan_observation.hpp"
#include <cmath>
#include <stdexcept>
using Window=vision_core::StationaryScanObservation;
using Result=Window::Result;
void Check(bool value) { if(!value)throw std::runtime_error("stationary scan window contract failed"); }
int main() {
  for(int size:{10,8}) {
    Window w;const int required=size/2;
    auto sample=[&](unsigned stop,double now,bool candidate,bool fresh=true,
                    std::optional<double> capture=std::nullopt) {
      return w.Observe(stop,fresh,now,candidate,.5,2,size,required,
                       candidate ? std::optional<double>(100+now) : std::nullopt,
                       std::nullopt,capture);
    };
    // Misses count from arrival; an empty partial window cannot end at .5s.
    Check(sample(1,0,false)==Result::kWaiting);
    Check(sample(1,.5,false)==Result::kWaiting);
    Check(sample(1,2,false,false)==Result::kEmpty); // No fresh frames still times out.
    w.Reset();
    for(int i=0;i<size;++i)
      Check(sample(1,.02*i,i<required)==Result::kWaiting);
    Check(sample(1,.49,true,false)==Result::kWaiting);
    Check(sample(1,.5,true,false)==Result::kConfirmed);
    Check(w.MeanU() && std::abs(*w.MeanU()-(100+.01*(required-1)))<1e-8);
    Check(sample(2,.6,false)==Result::kWaiting); // A new stop cannot inherit success.
    w.Reset();
    Check(sample(1,0,true,true,100)==Result::kWaiting);
    for(int i=1;i<20;++i)Check(sample(1,.05*i,true,true,100)==Result::kWaiting);
    Check(sample(1,2.01,false,false)==Result::kRejected); // Duplicate captures never fill the window.
    w.Reset();
    for(int i=0;i<size;++i) {
      const auto result=sample(1,.08*i,i<required-1);
      Check(i==size-1 ? result==Result::kRejected : result==Result::kWaiting);
    }
    w.Reset();w.Begin(1,10);
    Check(sample(1,12.01,true)==Result::kEmpty); // Late captures cannot rescue an expired stop.
  }
}
