#include "vision_core/config_loader.hpp"
#include <cassert>
#include <cmath>
#include <limits>
#include <filesystem>
#include "vision_core/line_plan.hpp"
using namespace vision_core;
int main() {
  for(double w:{1.0,2.0,3.0}) {
    LineGuideAccumulator a(1,0.0,2,1,w), b(1,0.0,2,1,w);
    a.Begin(7,0);b.Begin(8,0);
    LineGuide g;g.valid=true;g.offset=0;g.heading_rad=0;
    a.Add(7,0,g);b.AddCenter(8,0,Point2{0,10});
    g.offset=10;g.heading_rad=2;a.Add(7,1,g);b.AddCenter(8,1,Point2{10,30});
    const auto x=a.Finish(7,1);const auto y=b.FinishCenter(8,1,0,false);
    assert(x && y && std::abs(x->offset-10*w/(1+w))<1e-12);
    assert(std::abs(x->heading_rad-2*w/(1+w))<1e-12);
    assert(std::abs(y->u-x->offset)<1e-12 && std::abs(y->v-(10+30*w)/(1+w))<1e-12);
  }
  for(double bad:{0.0,-1.0,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}) {
    bool threw=false;try {LineGuideAccumulator a(1,0,2,1,bad);}catch(const std::invalid_argument&){threw=true;}
    assert(threw);
  }
  const auto production=LoadAlgorithmConfig((std::filesystem::path(__FILE__).parent_path().parent_path()/"config/vision_algorithm.yaml").string());
  assert(production.line_p2p.ready_observation_end_weight==1 &&
         production.ball.ready_observation_end_weight==1 &&
         production.goal.ready_observation_end_weight==1 &&
         production.hurdle.ready_observation_end_weight==3);
  auto c=LoadDefaultAlgorithmConfig();
  auto lp=c.line_p2p;lp.failure_min_valid_samples=1;lp.ready_observation_end_weight=1;lp.straight_repetitions=2;
  LinePlan plan(lp,2,1);plan.Begin(MissionAction::kStepForwardFour,7,false,3);
  ControlCommand command;command.command_type=CommandType::kAction;
  command.action=MissionAction::kLineStraightUnit;command.action_id=21;
  plan.Issued(command,0);plan.Feedback({21,true,false,false},0,0);
  plan.Observe(0,{}, {},7,Point2{0,10},true);
  plan.Observe(1,{}, {},7,Point2{10,30},true);
  command.action_id=22;plan.Issued(command,21);plan.Feedback({22,true,false,false},.1,.4);
  assert(!plan.StartEstimate(22));
  plan.Feedback({21,true,true,false},1,.4);
  assert(plan.StartEstimate(22) && std::abs(*plan.StartEstimate(22)-1.4)<1e-12);
  const auto center=plan.FinishObjectCenter(21,1,0,false,true);
  assert(center && std::abs(center->u-7.5)<1e-12);
  auto b=c.ball;
  b.approach_top_center_u_px=350;b.approach_bottom_center_u_px=450;
  b.approach_top_half_width_px=40;b.approach_bottom_half_width_px=100;
  const auto top=b.ApproachBounds(0,480),mid=b.ApproachBounds(239.5,480),bottom=b.ApproachBounds(479,480);
  assert(top.first==310 && top.second==390 && mid.first==330 && mid.second==470);
  assert(bottom.first==350 && bottom.second==550);
  const auto outer=b.ExtremeApproachBounds(239.5,480);
  assert(outer.first==mid.first-b.approach_near_outer_margin_px && outer.second==mid.second+b.approach_near_outer_margin_px);
}
