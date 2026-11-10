#include "vision_core/config_loader.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <limits>
#include <stdexcept>

using namespace vision_core;
namespace {
BallConfig Config() {
  auto c=LoadDefaultAlgorithmConfig().ball;
  c.approach_top_center_u_px=c.approach_bottom_center_u_px=380;
  c.approach_top_half_width_px=70;
  c.approach_bottom_half_width_px=90;
  c.approach_near_outer_margin_px=60;
  c.stable_window=c.stable_min_hits=c.entry_min_hits=1;
  c.pickup_verify_min_hits=1;
  c.upper_acquire_v_px=1000;
  c.approach_short_v_px=100;c.approach_two_step_v_px=240;
  c.approach_direct_down_v_px=355;
  return c;
}
ObjectTarget Target(double u,double v) {
  ObjectTarget t;t.center_px={u,v};t.box_px={u-5,v-5,10,10};
  t.width_px=t.height_px=10;t.area_px=100;t.confidence=1;return t;
}
MissionAction ActionAt(const BallConfig& c,double u,double v) {
  BallController controller(c,3);
  return controller.Compute(Target(u,v),640,480,0,{},{}).action.action;
}
void Bounds() {
  auto c=Config();
  auto top=c.ApproachBounds(0,480),middle=c.ApproachBounds(239.5,480),bottom=c.ApproachBounds(479,480);
  assert(top.first==310 && top.second==450);
  assert(middle.first==300 && middle.second==460);
  assert(bottom.first==290 && bottom.second==470);
  for (double v : {0.,17.3,100.2,239.5,355.9,479.}) {
    auto inner=c.ApproachBounds(v,480);
    auto outer=c.ExtremeApproachBounds(v,480);
    assert(std::abs(inner.first+inner.second-760.)<1e-9);
    assert(std::abs(outer.first-(inner.first-60.))<1e-9);
    assert(std::abs(outer.second-(inner.second+60.))<1e-9);
  }
  c.approach_top_half_width_px=100;
  middle=c.ApproachBounds(239.5,480);bottom=c.ApproachBounds(479,480);
  assert(middle.first==285 && middle.second==475);
  assert(bottom.first==290 && bottom.second==470);
  bottom=c.ApproachBounds(959,960);middle=c.ApproachBounds(479.5,960);
  assert(bottom.first==290 && bottom.second==470);
  assert(middle.first==285 && middle.second==475);
  c.approach_top_center_u_px=c.approach_bottom_center_u_px=360;
  assert(c.ApproachBounds(0,480).first==260);
  assert(c.ExtremeApproachBounds(0,480).first==200);
}
void Actions() {
  auto c=Config();
  assert(ActionAt(c,300,0)==MissionAction::kStepForwardLeftTwo);
  assert(ActionAt(c,380,0)==MissionAction::kStepForwardThree);
  assert(ActionAt(c,470,0)==MissionAction::kStepForwardRightTwo);
  assert(ActionAt(c,290,239.5)==MissionAction::kStepForwardLeftOne);
  assert(ActionAt(c,350,239.5)==MissionAction::kStepForwardThree);
  assert(ActionAt(c,480,239.5)==MissionAction::kStepForwardRightOne);
  assert(ActionAt(c,300,239.5)==MissionAction::kStepForwardThree);
  assert(ActionAt(c,460,239.5)==MissionAction::kStepForwardThree);
  assert(ActionAt(c,400,300)==MissionAction::kStepForwardTwo);
  assert(ActionAt(c,400,355)==MissionAction::kStepForwardOne);
  assert(ActionAt(c,280,355)==MissionAction::kTurnLeftInPlace);
  assert(ActionAt(c,550,355)==MissionAction::kStepForwardRightOneHurdle);
  assert(ActionAt(c,300,479)==MissionAction::kStepForwardOne);
  c.approach_top_half_width_px=100;
  assert(ActionAt(c,300,0)==MissionAction::kStepForwardThree);
  assert(ActionAt(c,300,479)==MissionAction::kStepForwardOne);
  c.approach_top_half_width_px=90;
  for(double v : {0.,17.3,100.2,239.5,355.9,479.}) {
    auto bounds=c.ApproachBounds(v,480);
    assert(bounds.first==290 && bounds.second==470);
  }
  assert(ActionAt(c,350,0)==MissionAction::kStepForwardThree);
  assert(ActionAt(c,280,0)==MissionAction::kStepForwardLeftTwo);
  assert(ActionAt(c,480,0)==MissionAction::kStepForwardRightTwo);
}
void RetuningAndValidation() {
  auto c=Config();BallController controller(c,3);
  auto result=controller.Compute(Target(300,0),640,480,0,{},{},false);
  assert(result.action.action==MissionAction::kStepForwardLeftTwo);
  c.approach_top_half_width_px=100;
  controller.UpdateTuning(c);
  result=controller.Compute(Target(300,0),640,480,.1,{},{},false);
  assert(result.action.action==MissionAction::kStepForwardThree);
  for(int which:{0,1,2}) for(double bad:{-1.0,
      std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()}) {
    auto cfg=LoadDefaultAlgorithmConfig();
    double* field=which==0 ? &cfg.ball.approach_top_half_width_px :
        which==1 ? &cfg.ball.approach_bottom_half_width_px :
                   &cfg.ball.approach_near_outer_margin_px;
    *field=bad;
    bool rejected=false;
    try {ValidateAlgorithmConfig(cfg);}catch(const std::runtime_error&){rejected=true;}
    assert(rejected);
  }
  auto cfg=LoadDefaultAlgorithmConfig();
  cfg.ball.approach_top_half_width_px=0.0;
  bool rejected=false;
  try {ValidateAlgorithmConfig(cfg);}catch(const std::runtime_error&){rejected=true;}
  assert(rejected);
}
}
int main(){Bounds();Actions();RetuningAndValidation();}
