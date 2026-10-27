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
  c.far_u_des_px=380;c.approach_u_deadband_px=90;
  c.approach_left_top_u_px=390;c.approach_right_top_u_px=530;
  c.stable_window=c.stable_min_hits=c.entry_min_hits=1;
  c.pickup_verify_min_hits=1; // This fixture uses a one-frame window.
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
  assert(top.first==390 && top.second==530);
  assert(middle.first==340 && middle.second==500);
  assert(bottom.first==290 && bottom.second==470);
  c.approach_left_top_u_px=450;
  middle=c.ApproachBounds(239.5,480);bottom=c.ApproachBounds(479,480);
  assert(middle.first==370 && middle.second==500); // Independent endpoints.
  assert(bottom.first==290 && bottom.second==470);
  bottom=c.ApproachBounds(959,960);middle=c.ApproachBounds(479.5,960);
  assert(bottom.first==290 && bottom.second==470);
  assert(middle.first==370 && middle.second==500); // Actual image height.
}
void Actions() {
  auto c=Config();
  assert(ActionAt(c,350,0)==MissionAction::kStepForwardLeftTwo);
  assert(ActionAt(c,450,0)==MissionAction::kStepForwardThree);
  assert(ActionAt(c,560,0)==MissionAction::kStepForwardRightTwo);
  assert(ActionAt(c,320,239.5)==MissionAction::kStepForwardLeftOne);
  assert(ActionAt(c,350,239.5)==MissionAction::kStepForwardThree);
  assert(ActionAt(c,520,239.5)==MissionAction::kStepForwardRightOne);
  // Center-column boundary points remain inclusive.
  assert(ActionAt(c,340,239.5)==MissionAction::kStepForwardThree);
  assert(ActionAt(c,500,239.5)==MissionAction::kStepForwardThree);
  assert(ActionAt(c,400,300)==MissionAction::kStepForwardTwo);
  assert(ActionAt(c,400,355)==MissionAction::kStepForwardOne);
  assert(ActionAt(c,300,355)==MissionAction::kTurnLeftInPlace);
  assert(ActionAt(c,550,355)==MissionAction::kStepForwardRightOneHurdle);
  assert(ActionAt(c,300,479)==MissionAction::kStepForwardOne);
  c.approach_left_top_u_px=450;
  assert(ActionAt(c,350,239.5)==MissionAction::kStepForwardLeftOne);
  assert(ActionAt(c,300,479)==MissionAction::kStepForwardOne);
  // Vertical endpoints reproduce the existing straight grid.
  c.approach_left_top_u_px=290;c.approach_right_top_u_px=470;
  for (double v : {0.0,17.3,100.2,239.5,355.9,479.0}) {
    const auto vertical=c.ApproachBounds(v,480);
    assert(vertical.first==290 && vertical.second==470);
  }
  assert(ActionAt(c,350,0)==MissionAction::kStepForwardThree);
  assert(ActionAt(c,280,0)==MissionAction::kStepForwardLeftTwo);
  assert(ActionAt(c,480,0)==MissionAction::kStepForwardRightTwo);
}
void RetuningAndValidation() {
  auto c=Config();BallController controller(c,3);
  auto r=controller.Compute(Target(350,0),640,480,0,{},{},false);
  assert(r.action.action==MissionAction::kStepForwardLeftTwo);
  c.approach_left_top_u_px=290;c.approach_right_top_u_px=470;controller.UpdateTuning(c);
  r=controller.Compute(Target(350,0),640,480,.1,{},{},false);
  assert(r.action.action==MissionAction::kStepForwardThree);
  for (int which : {0,1}) for (double bad : {-1.0,
      std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()}) {
    auto cfg=LoadDefaultAlgorithmConfig();
    (which==0 ? cfg.ball.approach_left_top_u_px : cfg.ball.approach_right_top_u_px)=bad;
    bool rejected=false;
    try {ValidateAlgorithmConfig(cfg);} catch (const std::runtime_error&) {rejected=true;}
    assert(rejected);
  }
  auto cfg=LoadDefaultAlgorithmConfig();
  cfg.ball.approach_left_top_u_px=cfg.ball.approach_right_top_u_px;
  bool rejected=false;
  try {ValidateAlgorithmConfig(cfg);} catch (const std::runtime_error&) {rejected=true;}
  assert(rejected);
}
}
int main() {Bounds();Actions();RetuningAndValidation();}
