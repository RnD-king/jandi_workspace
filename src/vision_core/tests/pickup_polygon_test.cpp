#include "vision_core/ball_controller.hpp"
#include "vision_core/ball_fine_motion.hpp"
#include "vision_core/config_loader.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <deque>
#include <stdexcept>
#include <string>

using namespace vision_core;
namespace {
PickupPolygon Six() {
  return {{384,338},{370,282},{394,256},{438,254},{450,298},{433,338}};
}
bool Rejects(const PickupPolygon& polygon) {
  try { ValidatePickupPolygon(polygon); }
  catch(const std::invalid_argument&) {return true;}
  return false;
}
ObjectTarget Target(double u,double v) {
  ObjectTarget target;
  target.center_px={u,v};
  target.box_px={u-10,v-10,20,20};
  target.width_px=20;target.height_px=20;
  target.area_px=400;target.confidence=1;
  return target;
}
BallConfig Config() {
  auto c=LoadDefaultAlgorithmConfig().ball;
  c.pickup_polygon=Six();
  c.pickup_shrink_enabled=true;
  c.pickup_shrink_percent=7;
  c.pickup_shrink_max_failures=2;
  c.fine_motion={};
  c.stable_window=10;
  c.stable_min_hits=7;
  return c;
}
MissionAction FineAt(const BallConfig& config,Point2 point) {
  BallController controller(config,3.0);
  controller.StartTestExecute();
  controller.UseVerifiedObservation(Target(point.u,point.v),
                                     std::deque<bool>(10,true),640,480);
  const auto result=controller.Compute({},640,480,0,
      {CameraMode::kBallView,true},{},false);
  assert(result.mode==BallMode::kFineAdjustForPickup);
  assert(controller.FineMotionRepetitions()==0);
  return result.action.action;
}
}
int main() {
  auto hex=Six();
  ValidatePickupPolygon(hex);
  assert(PickupPolygonContains(hex,{415,300}));
  assert(!PickupPolygonContains(hex,{350,300}));
  assert(!PickupPolygonContains(hex,{470,300}));
  assert(PickupPolygonDistance(hex,{415,300})==0.0);
  assert(PickupPolygonDistance(hex,{350,300})>0);
  const auto serialized=FormatPickupPolygonPoints(hex);
  const auto parsed=ParsePickupPolygonPoints(serialized);
  assert(parsed.size()==6);
  for(std::size_t i=0;i<hex.size();++i)
    assert(parsed[i].u==hex[i].u && parsed[i].v==hex[i].v);
  auto seven=ParsePickupPolygonPoints(
      "384,338;370,282;380,265;394,256;438,254;450,298;433,338");
  assert(seven.size()==7);
  assert(Rejects({{1,1},{1,1},{100,100}}));
  assert(Rejects({{100,100},{500,500},{100,400}}));
  assert(Rejects({{100,100},{200,200},{100,200},{200,100}}));
  for(const auto& bad:{"","10,10;20,20","-1,10;20,20;30,30",
                       "10,10;20,20;30,nan","10,10;20,20;30,30;",
                       "10,10;20,20;30,30;10,10"}) {
    bool rejected=false;
    try { (void)ParsePickupPolygonPoints(bad); }
    catch(const std::invalid_argument&) {rejected=true;}
    assert(rejected);
  }

  auto c=Config();
  ValidateAlgorithmConfig([&]{
    auto all=LoadDefaultAlgorithmConfig();
    all.ball=c;
    return all;
  }());
  const auto center=PickupPolygonCentroid(hex);
  const auto initial=c.PickupRegionAfterFailures(0).PickupVertices();
  const auto once=c.PickupRegionAfterFailures(1).PickupVertices();
  const auto twice=c.PickupRegionAfterFailures(2).PickupVertices();
  const auto clamped=c.PickupRegionAfterFailures(15).PickupVertices();
  const double area=std::abs(PickupSignedArea2(hex));
  assert(std::abs(std::abs(PickupSignedArea2(initial))/area - .9*.9)<1e-9);
  assert(std::abs(std::abs(PickupSignedArea2(once))/area - std::pow(.9*.93,2))<1e-9);
  assert(std::abs(std::abs(PickupSignedArea2(twice))/area - std::pow(.9*.93*.93,2))<1e-9);
  for(std::size_t i=0;i<hex.size();++i) {
    assert(std::abs((initial[i].u-center.u)-.9*(hex[i].u-center.u))<1e-7);
    assert(std::abs((once[i].u-center.u)-.9*.93*(hex[i].u-center.u))<1e-7);
    assert(std::abs((twice[i].u-center.u)-.9*.93*.93*(hex[i].u-center.u))<1e-7);
    assert(std::abs(twice[i].u-clamped[i].u)<1e-9);
    assert(std::abs(twice[i].v-clamped[i].v)<1e-9);
  }
  c.pickup_shrink_enabled=false;
  const auto no_shrink=c.PickupRegionAfterFailures(2).PickupVertices();
  assert(std::abs(no_shrink[0].u-initial[0].u)<1e-9 &&
         std::abs(no_shrink[0].v-initial[0].v)<1e-9);
  c.pickup_shrink_enabled=true;

  // All four uncalibrated directions choose exactly one existing FINE action.
  assert(FineAt(c,{415,300})==MissionAction::kPickBall);
  assert(FineAt(c,{350,300})==MissionAction::kLeftSideStep);
  assert(FineAt(c,{470,300})==MissionAction::kRightSideStep);
  assert(FineAt(c,{415,230})==MissionAction::kStepForwardHalf);
  assert(FineAt(c,{415,370})==MissionAction::kStepBack);

  BallFineMotionConfig fine;
  assert(PlanBallFineMotion(fine,{330,300},hex,640,480).repetitions==0);
  fine.left_delta_px={20,0};
  auto plan=PlanBallFineMotion(fine,{330,300},hex,640,480);
  assert(plan.action==MissionAction::kLeftSideStep && plan.repetitions==3);
  assert(PickupPolygonContains(hex,plan.predicted_center_px));
  fine.left_delta_px={50,0};
  plan=PlanBallFineMotion(fine,{330,300},hex,640,480);
  assert(plan.action==MissionAction::kLeftSideStep && plan.repetitions==1);
  fine.left_delta_px={150,0}; // Jumps over both boundaries: never choose.
  assert(PlanBallFineMotion(fine,{330,300},hex,640,480).repetitions==0);
  fine.left_delta_px={-20,0}; // Wrong sign: cannot improve.
  assert(PlanBallFineMotion(fine,{330,300},hex,640,480).repetitions==0);
  fine.left_delta_px={20,0};
  fine.max_repetitions=1;
  plan=PlanBallFineMotion(fine,{330,300},hex,640,480);
  assert(plan.repetitions==1);

  auto invalid=LoadDefaultAlgorithmConfig();
  invalid.ball=c;
  invalid.ball.pickup_shrink_percent=100;
  bool rejected=false;
  try { ValidateAlgorithmConfig(invalid); }
  catch(const std::runtime_error&) {rejected=true;}
  assert(rejected);
  invalid.ball=c;
  invalid.ball.pickup_initial_shrink_percent=100.0;
  rejected=false;
  try { ValidateAlgorithmConfig(invalid); }
  catch(const std::runtime_error&) {rejected=true;}
  assert(rejected);
  invalid.ball=c;
  invalid.ball.pickup_polygon={{100,100},{200,200},{100,200},{200,100}};
  rejected=false;
  try { ValidateAlgorithmConfig(invalid); }
  catch(const std::runtime_error&) {rejected=true;}
  assert(rejected);
  return 0;
}
