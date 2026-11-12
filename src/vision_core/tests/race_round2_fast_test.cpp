#include "vision_core/ball_controller.hpp"
#include "vision_core/config_loader.hpp"
#include "vision_core/down_search_walk.hpp"
#include "vision_core/hurdle_controller.hpp"
#include "vision_core/stationary_object_recovery.hpp"
#include "vision_core/stationary_window.hpp"

#include <cassert>
#include <deque>
#include <optional>

using namespace vision_core;
namespace {
ObjectTarget Seen(double v = 200.0) {
  ObjectTarget target;
  target.center_px = {320.0,v};
  target.box_px = {300.0,v-10.0,40.0,20.0};
  target.width_px = 40.0;
  target.height_px = 20.0;
  target.area_px = 800.0;
  target.confidence = .95;
  return target;
}
ActionExecutionFeedback Moving() {
  ActionExecutionFeedback x;
  x.action_active = true;
  return x;
}
ActionExecutionFeedback Done() {
  ActionExecutionFeedback x;
  x.action_done = true;
  return x;
}
ActionExecutionFeedback Observed() {
  ActionExecutionFeedback x;
  x.observation_complete = true;
  return x;
}

void StationaryPolicy() {
  const auto production = LoadDefaultAlgorithmConfig();
  assert(production.race_round2_fast_enabled);
  assert(production.ball.race_round2_fast_enabled);
  assert(production.hurdle.race_round2_fast_enabled);
  assert(production.goal.race_round2_fast_enabled);
  assert(kFastStationaryWindow == 8);
  assert(FastStationaryMinHits(3) == 3);
  assert(FastStationaryMinHits(4) == 4);
  assert(FastStationaryMinHits(5) == 5);
  assert(FastStationaryMinHits(7) == 7);
  assert(production.line.line_stable_window == 8 && production.line.line_stable_min_hits == 4);
  assert(production.ball.stable_window == 8 && production.ball.stable_min_hits == 4);
  assert(production.ball.near_v_min_hits == 3);
  assert(production.ball.pickup_verify_min_hits == 3);
  assert(production.hurdle.stable_window == 8 && production.hurdle.stable_min_hits == 4);
  assert(production.hurdle.near_v_min_hits == 3);
  assert(production.goal.stable_window == 8 && production.goal.stable_min_hits == 6);
  assert(production.goal.near_distance_min_hits == 3);
  assert(production.goal.edge_stable_window == 8 && production.goal.edge_stable_min_hits == 5);
  const std::deque<bool> hits{true,true,true,false,false,false,false,false};
  assert(FastStationaryQualified(hits,3)); // Last frame is a miss.
  assert(!FastStationaryQualified(hits,5));

  StationaryObjectRecovery recovery;
  recovery.Begin(0.0);
  for (int i=0; i<7; ++i)
    assert(recovery.Observe(i<6,0.05*(i+1),8,6,1.0,1,true) ==
           StationaryObjectRecovery::Result::kWaiting);
  assert(recovery.Observe(false,.40,8,6,1.0,1,true) ==
         StationaryObjectRecovery::Result::kFound);
  StationaryObjectRecovery missing;
  missing.Begin(0.0);
  for (int i=0;i<8;++i)
    assert(missing.Observe(false,0.05*(i+1),8,6,1.0,1,true) ==
           StationaryObjectRecovery::Result::kWaiting);
  assert(missing.Observe(false,1.01,8,6,1.0,1,true) ==
         StationaryObjectRecovery::Result::kFailed);
}

void BallSearchLastMissAndSkipHold() {
  auto cfg=LoadDefaultAlgorithmConfig().ball;
  cfg.down_search_min_hits=3;
  cfg.down_search_inter_action_hold_sec=.3;
  BallController ball(cfg,3.0);
  const CameraFeedback down{CameraMode::kBallView,true};
  ball.StartTestNear();
  auto r=ball.Compute({},640,480,0.0,down,{});
  assert(r.mode==BallMode::kDownSearch);
  r=ball.Compute({},640,480,0.1,down,{});
  assert(r.action.action==MissionAction::kStepForwardOne);
  for(int i=0;i<8;++i) {
    const std::optional<ObjectTarget> t=i<3 ? std::optional<ObjectTarget>(Seen()) : std::nullopt;
    r=ball.Compute(t,640,480,.15+.05*i,down,Moving());
    assert(r.mode==BallMode::kDownSearch);
  }
  r=ball.Compute({},640,480,.65,down,Done());
  assert(r.mode==BallMode::kWaitFinePose);
  // The production config holds the actual post-discovery FINE transition
  // for 0.5 seconds; the original fixture incorrectly expected it on DONE.
  assert(cfg.fine_pose_pre_hold_sec > 0);
  assert(r.action.action==MissionAction::kNone);
  r=ball.Compute({},640,480,.65+cfg.fine_pose_pre_hold_sec-.01,down,{});
  assert(r.mode==BallMode::kWaitFinePose && r.action.action==MissionAction::kNone);
  r=ball.Compute({},640,480,.65+cfg.fine_pose_pre_hold_sec+.01,down,{});
  assert(r.mode==BallMode::kWaitFinePose && r.action.action==MissionAction::kDefaultPoseMode);
}

void HurdleMovingDiscoveryNeedsStationaryVerification() {
  auto cfg=LoadDefaultAlgorithmConfig().hurdle;
  cfg.down_search_min_hits=3;
  cfg.down_search_inter_action_hold_sec=.3;
  HurdleController hurdle(cfg,3.0);
  const CameraFeedback down{CameraMode::kDown,true};
  hurdle.StartTestNear(true);
  auto r=hurdle.Compute({},640,480,0.0,down,{},{});
  assert(r.mode==HurdleMode::kDownSearch);
  r=hurdle.Compute({},640,480,0.1,down,{},{});
  assert(r.action.action==MissionAction::kLineStraightUnit);
  for(int i=0;i<6;++i) {
    const std::optional<ObjectTarget> t=i<3 ? std::optional<ObjectTarget>(Seen(300)) : std::nullopt;
    r=hurdle.Compute(t,640,480,.2+.05*i,down,Moving(),{});
    assert(r.mode==HurdleMode::kDownSearch);
    assert(r.action.action==MissionAction::kNone);
  }
  r=hurdle.Compute({},640,480,.55,down,Done(),{});
  assert(r.mode==HurdleMode::kCoarseAdjust);
  assert(r.action.action==MissionAction::kNone);
  // A moving v>=220 cannot enter FINE. Only fresh stationary raw votes can.
  const std::deque<std::optional<double>> raw_v(8,300.0);
  hurdle.UseVerifiedObservation(Seen(300),std::deque<bool>(8,true),640,480,&raw_v);
  r=hurdle.Compute({},640,480,.65,down,Observed(),{});
  assert(r.mode==HurdleMode::kWaitFinePose);
  assert(r.action.action==MissionAction::kDefaultPoseMode);
}

void HurdleMissingSearchAndHold() {
  auto cfg=LoadDefaultAlgorithmConfig().hurdle;
  cfg.down_search_min_hits=3;
  cfg.down_search_inter_action_hold_sec=.3;
  HurdleController hurdle(cfg,3.0);
  const CameraFeedback down{CameraMode::kDown,true};
  hurdle.StartTestNear(true);
  hurdle.Compute({},640,480,0.0,down,{},{});
  auto r=hurdle.Compute({},640,480,.1,down,{},{});
  assert(r.action.action==MissionAction::kLineStraightUnit);
  for(int i=0;i<6;++i)
    r=hurdle.Compute({},640,480,.2+.05*i,down,Moving(),{});
  r=hurdle.Compute({},640,480,.55,down,Done(),{});
  assert(r.action.action==MissionAction::kStepForwardOne);
  for(int i=0;i<8;++i) {
    const std::optional<ObjectTarget> t=i<3 ? std::optional<ObjectTarget>(Seen(50)) : std::nullopt;
    r=hurdle.Compute(t,640,480,.60+.05*i,down,Moving(),{});
  }
  r=hurdle.Compute({},640,480,1.02,down,Done(),{});
  assert(r.mode==HurdleMode::kCoarseAdjust);
  assert(r.action.action==MissionAction::kNone); // No 0.3s after found.
}
}  // namespace

int main() {
  StationaryPolicy();
  BallSearchLastMissAndSkipHold();
  HurdleMovingDiscoveryNeedsStationaryVerification();
  HurdleMissingSearchAndHold();
}
