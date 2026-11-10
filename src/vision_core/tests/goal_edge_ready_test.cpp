// Seed completed course stages without running unrelated BALL/GOAL missions.
// Dependencies are included first; only MissionController access is exposed.
#include <algorithm>
#include <cstddef>
#include <deque>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>
#include "vision_core/ball_controller.hpp"
#include "vision_core/ball_candidate_tracker.hpp"
#include "vision_core/control_command.hpp"
#include "vision_core/goal_controller.hpp"
#include "vision_core/hurdle_controller.hpp"
#include "vision_core/line_detection_extractor.hpp"
#include "vision_core/line_feature_extractor.hpp"
#include "vision_core/line_p2p_controller.hpp"
#include "vision_core/line_plan.hpp"
#include "vision_core/object_approach.hpp"
#include <array>
#include "vision_core/object_target_extractor.hpp"
#include "vision_core/object_association_tracker.hpp"
#include "vision_core/object_target_tracker.hpp"
#define private public
#include "vision_core/mission_controller.hpp"
#undef private
#include "vision_core/config_loader.hpp"
#include <stdexcept>
#include <limits>

using namespace vision_core;
namespace { void Check(bool value,const char* m){if(!value)throw std::runtime_error(m);} }

namespace {
ObjectTarget Board(double u) {
  ObjectTarget b;b.class_id=3;b.confidence=1;b.center_px={u,150};
  b.box_px={u-20,130,40,40};b.width_px=b.height_px=40;b.area_px=1600;return b;
}
struct Run {
  MissionController core;MissionFrameInput in;MissionFrameResult out;
  static MissionControllerConfig Config() {
    auto c=LoadDefaultAlgorithmConfig();c.enable_ball=c.enable_hurdle=false;c.enable_goal=true;
    c.race_round2_fast_enabled=c.goal.race_round2_fast_enabled=true;
    c.initial_has_ball=c.goal_trial_start_goal=true;c.goal.stable_window=8;c.goal.stable_min_hits=5;
    c.goal.edge_stable_window=8;c.goal.edge_stable_min_hits=5;
    c.line_p2p.left_repetitions=c.line_p2p.right_repetitions=c.line_p2p.straight_repetitions=1;
    c.post_motion_observation_sec=1;c.goal.lost_frames=3;return c;
  }
  Run():core(Config()) {
    in.image_width=640;in.image_height=480;in.command_transport_enabled=true;
    in.camera_feedback={CameraMode::kGoal,true};in.backboard_target=Board(320);
    in.backboard_track_id=9;in.backboard_entry_track_hits=8;in.backboard_entry_track_frames=8;
    in.backboard_center_depth_m=2;in.goal_pose={true,0,2,0,1};
    in.raw_backboard_observation_available=true;in.raw_backboard_target=Board(320);
  }
  void Tick(CommandDeliveryFeedback f={},double dt=.02) {
    in.now_sec+=dt;in.delivery_feedback=f;out=core.Step(in);
  }
  std::uint64_t Warm() {
    for(int i=0;i<30;++i) {Tick();if(LinePlan::IsUnit(out.command.action))break;}
    Check(LinePlan::IsUnit(out.command.action) && in.now_sec<1,
          "centered far GOAL waited for a one-second edge observation");
    auto id=out.command.action_id;Tick({id,true,false,false});return id;
  }
};
void MovingEdgeBeforeReady() {
  Run d;const auto walk=d.Warm();d.in.raw_backboard_target=Board(20);
  for(int i=0;i<8;++i)d.Tick();
  Check(!d.core.command_coordinator_.HasQueuedAction(),"edge caused an early READY successor");
  d.Tick({walk,true,false,true});
  Check(!d.core.command_coordinator_.HasQueuedAction() &&
        d.core.goal_edge_guard_.phase==MissionController::GoalEdgeGuard::Phase::kPending,
        "edge was not checked before normal READY reservation");
  d.Tick({walk,true,true,false});
  const auto first=d.out.command.action_id;
  Check(first && first!=walk && d.out.command.action==MissionAction::kTurnLeftInPlace,
        "drained edge did not start first WALK turn");
  d.Tick({first,true,false,false});d.Tick({first,true,true,false});
  const auto second=d.out.command.action_id;
  Check(second && second!=first && d.out.command.action==MissionAction::kTurnLeftInPlace,
        "first real DONE did not start exactly one second turn");
  d.Tick({second,true,false,false});d.Tick({second,true,true,false});
  d.in.raw_backboard_target=Board(320);
  for(int i=0;i<7;++i) {d.Tick();Check(d.out.command.action_id==0,"post-turn observation skipped its fresh window");}
  for(int i=0;i<60 && !LinePlan::IsUnit(d.out.command.action);++i)d.Tick();
  Check(LinePlan::IsUnit(d.out.command.action),"normal approach did not resume after fresh unclipped window");
}
void ReadyTailAndQueuedStart() {
  Run d;const auto a=d.Warm();for(int i=0;i<10;++i)d.Tick();
  d.Tick({a,true,false,true});const auto b=d.out.command.action_id;
  Check(b && b!=a && d.core.command_coordinator_.HasQueuedAction(),"normal READY did not reserve B");
  d.Tick({b,true,false,false});d.in.raw_backboard_target=Board(20);
  for(int i=0;i<10;++i)d.Tick();
  Check(!d.core.goal_edge_guard_.left_qualified,"post-READY/pre-start tail polluted edge evidence");
  d.Tick({a,true,true,false});
  for(int i=0;i<8;++i)d.Tick();
  Check(d.core.goal_edge_guard_.left_qualified,"queued B actual start did not open fresh edge window");
  d.Tick({b,true,false,true});
  Check(!d.core.command_coordinator_.HasQueuedAction(),"edge READY reserved an unwanted C");
  d.Tick({b,true,true,false});
  Check(d.out.command.action==MissionAction::kTurnLeftInPlace,"B DONE did not reach edge correction");
}
void MissingAfterTurnsUsesExistingRecovery() {
  Run d;const auto walk=d.Warm();d.in.raw_backboard_target=Board(20);
  for(int i=0;i<8;++i)d.Tick();
  d.Tick({walk,true,false,true});d.Tick({walk,true,true,false});
  const auto a=d.out.command.action_id;d.Tick({a,true,false,false});d.Tick({a,true,true,false});
  const auto b=d.out.command.action_id;d.Tick({b,true,false,false});d.Tick({b,true,true,false});
  d.in.raw_backboard_target.reset();d.in.backboard_target.reset();
  d.in.backboard_center_depth_m.reset();d.in.goal_pose={};
  bool recovery=false;
  for(int i=0;i<100 && !recovery;++i) {
    d.Tick({},.04);
    recovery=d.out.command.camera_request==CameraRequest::kGoalScanLeft;
    Check(!LinePlan::IsUnit(d.out.command.action),"missing post-turn board was treated as unclipped");
  }
  Check(recovery,"post-turn RGB loss did not reach existing GOAL head search");
}
void CapturedBeforeWalkStartIsExcluded() {
  Run d;const auto walk=d.Warm();d.in.raw_backboard_target=Board(20);
  const auto start=d.core.line_plan_.StartEstimate(walk);
  Check(start.has_value(),"physical walk start missing");
  for(int i=0;i<8;++i) {
    d.in.frame_capture_sec=*start-.1+.001*i;d.Tick();
  }
  Check(!d.core.goal_edge_guard_.left_qualified,"pre-start captured frames counted as walk evidence");
}
void StaleCaptureIsNotEightFrames() {
  Run d;d.Warm();d.in.raw_backboard_target=Board(20);d.in.frame_capture_sec=100.0;
  for(int i=0;i<12;++i)d.Tick();
  Check(std::count(d.core.goal_edge_guard_.left_hits.begin(),d.core.goal_edge_guard_.left_hits.end(),true)==1 && !d.core.goal_edge_guard_.left_qualified,
        "repeated captured image counted as multiple edge frames");
}
}
int main(){MovingEdgeBeforeReady();ReadyTailAndQueuedStart();CapturedBeforeWalkStartIsExcluded();StaleCaptureIsNotEightFrames();MissingAfterTurnsUsesExistingRecovery();}
