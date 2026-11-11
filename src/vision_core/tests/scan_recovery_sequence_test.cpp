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
namespace {
void Check(bool ok,const char* message) { if(!ok) throw std::runtime_error(message); }
MissionControllerConfig Config() {
  auto c=LoadDefaultAlgorithmConfig();c.enable_ball=c.enable_hurdle=c.enable_goal=false;
  c.curve_search.enabled=false;c.test_mission=MissionType::kNone;
  c.initial_has_ball=c.goal_trial_start_goal=false;
  c.line_features.image_center_u=320;c.line.line_stable_window=10;c.line.line_stable_min_hits=5;
  c.line_recovery.scan_verify_window=10;c.line_recovery.scan_verify_hits=5;
  c.line_p2p.left_repetitions=c.line_p2p.right_repetitions=1;
  c.transition_settle_sec=c.walk_inter_action_hold_sec=0;
  return c;
}
LineGuide Guide(double offset) { LineGuide g;g.valid=true;g.offset=offset;return g; }
struct Trial {
  MissionController core{Config()};MissionFrameInput in;MissionFrameResult out;
  Trial(int points,double u=150,bool memory=true,bool degenerate=false) {
    in.image_width=640;in.image_height=480;in.intrinsics={600,600,320,240};
    in.command_transport_enabled=true;in.camera_feedback={CameraMode::kForward,true};
    if(memory) { core.line_recovery_memory_.normal=Guide(-.4);core.line_recovery_memory_.last_left=true; }
    Points(points,u,degenerate);
  }
  void Points(int count,double u,bool degenerate=false) {
    in.line_centers.clear();for(int i=0;i<count;++i)
      in.line_centers.push_back({u+i*(degenerate?10:0),degenerate?400.0:450.0-i*50});
  }
  void Tick(CommandDeliveryFeedback feedback={},double dt=.1) {
    in.now_sec+=dt;in.delivery_feedback=feedback;out=core.Step(in);
  }
  void Observe() { for(int i=0;i<15;++i) { Tick();if(out.command.action_id || out.command.camera_request!=CameraRequest::kNone) break; } }
  void FinishUnit() {
    const auto id=out.command.action_id;Check(id!=0,"recovery unit missing");
    Tick({id,true,false,false});
    for(int i=0;i<15;++i) { Tick();Check(out.command.action_id==0 || out.command.action_id==id,"new recovery before DONE"); }
    Tick({id,true,false,true});Check(!core.command_coordinator_.HasQueuedAction(),"READY queued another recovery unit");
    Tick({id,true,true,false});
  }
};
void LossBudgetsAndFallback() {
  Trial gated(2);gated.in.allow_new_body_actions=false;gated.Observe();
  Check(gated.core.line_forward_recovery_issued_==0,"blocked recovery consumed its count");
  gated.in.allow_new_body_actions=true;gated.Tick();
  Check(gated.core.line_forward_recovery_issued_==1 && gated.out.command.action_id,
        "blocked recovery lost its pending intent");
  for(int n:{0,1}) { Trial t(n);t.Observe();
    Check(t.core.line_forward_recovery_issued_==0,"0/1 points walked");
    Check(t.out.command.camera_request==CameraRequest::kLineScanLeft,"0/1 points did not scan"); }
  for(const auto pair:std::array<std::pair<int,int>,2>{{{3,2},{2,3}}}) {
    Trial t(pair.first);t.Observe();Check(t.core.line_forward_recovery_issued_==1,"first recovery not counted");
    t.FinishUnit();t.Points(pair.second,150);t.Observe();
    Check(t.core.line_forward_recovery_issued_==1,"changing point count increased budget");
    Check(t.core.line_state_==LineState::kRecoveryHeadScan,"exhausted episode did not scan");
  }
  Trial twice(3);twice.Observe();twice.FinishUnit();
  twice.core.line_recovery_memory_.normal=Guide(.8); // Must not replace the frozen loss direction.
  twice.Points(3,400,true);twice.Observe();
  Check(twice.core.line_forward_recovery_issued_==2,"three-point budget did not allow its second unit");
  Check(twice.out.command.action==MissionAction::kLineLeftUnit,"degenerate fit ignored frozen direction");
  twice.FinishUnit();twice.Observe();
  Check(twice.core.line_state_==LineState::kRecoveryHeadScan,"third recovery was allowed");
  for(int n:{2,3}) { Trial missing(n,320,false,true);missing.Observe();
    Check(missing.core.line_forward_recovery_issued_==0,"invalid fit selected an arbitrary side"); }
  Trial center(2);center.core.line_recovery_memory_.normal=Guide(0);center.Observe();
  Check(center.out.command.action==MissionAction::kLineLeftUnit,"zero score lost the last valid side");
}
void OffCenterLineAndBoundedReobserve() {
  using Phase=MissionController::LineHeadSearch::Phase;
  Trial found(2,550);found.core.line_state_=LineState::kRecoveryHeadScan;
  found.core.line_head_search_.phase=Phase::kScanningRight;
  found.core.line_head_search_.tried=true;
  found.in.camera_feedback={CameraMode::kTransition,false};
  found.in.camera_scan={1,2,-45,false,1,true};
  bool returned=false;
  for(int i=0;i<12;++i) { found.Tick();returned|=found.out.command.camera_request==CameraRequest::kScanReturn; }
  Check(returned && found.core.line_head_search_.align_remaining==3,"off-center stable LINE was rejected");
  for(int used:{0,2}) {
    Trial t(0);t.core.line_state_=LineState::kRecoveryHeadScan;
    auto& scan=t.core.line_head_search_;scan.phase=Phase::kReobserve;scan.tried=true;
    scan.completed_pairs=used;scan.last_found_left=false;scan.observation.Begin(1,0);
    for(int i=0;i<10;++i)t.Tick();
    Check(used==2 ? t.core.line_state_==LineState::kFinalHold :
          scan.phase==Phase::kPendingRight,"failed reacquisition reset attempts or forgot the last side");
  }
  Trial restored(4,320);restored.core.line_state_=LineState::kRecoveryHeadScan;
  restored.core.line_head_search_.phase=Phase::kReobserve;
  restored.core.line_head_search_.tried=true;restored.core.line_forward_recovery_issued_=2;
  restored.core.line_recovery_memory_.budget=2;
  for(int i=0;i<10;++i)restored.Tick();
  Check(restored.core.line_state_==LineState::kNormal &&
        restored.core.line_forward_recovery_issued_==0 &&
        restored.core.line_recovery_memory_.budget==-1 && !restored.core.line_head_search_.tried,
        "normal reacquisition did not reset the loss episode");
}
ObjectTarget Board(double u=550) {
  ObjectTarget t;t.center_px={u,100};t.box_px={u-20,80,40,40};t.width_px=t.height_px=40;
  t.area_px=1600;t.confidence=1;return t;
}
void GoalNormalVotesDepthAndDone() {
  using Phase=MissionController::GoalHeadSearch::Phase;
  for(int hits:{5,7}) {
    auto c=Config();c.goal.stable_window=10;c.goal.stable_min_hits=7;
    MissionController core(c);auto& scan=core.goal_head_search_;
    scan.phase=Phase::kReobserve;scan.attempts=1;scan.used=true;scan.last_found_left=false;
    MissionFrameInput in;in.image_width=640;in.image_height=480;in.camera_feedback={CameraMode::kGoal,true};
    for(int i=0;i<10;++i) {
      in.now_sec=.1*(i+1);
      if(i<hits) { in.backboard_target=Board();in.backboard_center_depth_m=2; }
      else { in.backboard_target.reset();in.backboard_center_depth_m.reset(); }
      core.PrepareContinuousApproach(in,{},true);
    }
    Check(hits==7 ? scan.phase==Phase::kIdle && !scan.used :
          scan.phase==Phase::kPendingRight && scan.attempts==1,
          "GOAL recheck used scan 10/5 instead of normal 10/7, or lost its retry side");
  }
  MissionController core(Config());auto& scan=core.goal_head_search_;
  scan.phase=Phase::kBodyTurn;scan.turn_left=true;scan.turns_remaining=2;scan.attempts=2;scan.body_action_id=42;
  MissionFrameInput in;in.now_sec=1;in.camera_feedback={CameraMode::kGoal,true};
  auto r=core.PrepareContinuousApproach(in,{},false);
  Check(scan.turns_remaining==2 && r.action.action==MissionAction::kNone,"body correction advanced without DONE");
  in.delivery_feedback={41,true,true,false};core.PrepareContinuousApproach(in,{},true);
  Check(scan.turns_remaining==2,"another action's DONE consumed a correction");
  in.delivery_feedback={42,true,true,false};r=core.PrepareContinuousApproach(in,{},true);
  Check(scan.turns_remaining==1 && r.action.action==MissionAction::kTurnLeftInPlace,"matching DONE did not permit the next turn");
  scan.body_action_id=43;core.PrepareContinuousApproach(in,{},true);
  Check(scan.turns_remaining==1,"duplicate DONE consumed a second correction");
  in.delivery_feedback={43,true,true,false};core.PrepareContinuousApproach(in,{},true);
  Check(scan.phase==Phase::kReobserve && scan.attempts==2,"body completion reset the scan budget");
  in.delivery_feedback={};in.backboard_target=Board(); // RGB without depth must fail recheck.
  for(int i=0;i<10;++i) { in.now_sec+=.1;core.PrepareContinuousApproach(in,{},true); }
  Check(scan.phase==Phase::kPendingLeft && scan.attempts==2,"invalid depth passed GOAL recheck");
  scan.phase=Phase::kReobserve;scan.attempts=3;scan.observation.Reset();
  for(int i=0;i<10;++i) { in.now_sec+=.1;core.PrepareContinuousApproach(in,{},true); }
  Check(scan.exhausted && scan.phase==Phase::kIdle,"three failed GOAL rounds did not terminate");
}
void CurveThreeTurnsAndSharedRetryBudget() {
  using Phase=MissionController::CurveSearchState::Phase;
  Trial t(4,320);auto& curve=t.core.curve_;curve.phase=Phase::kBodyTurn;
  curve.turns_remaining=3;curve.turn_left=true;curve.direction=1;curve.head_attempts=2;
  for(int turn=0;turn<3;++turn) {
    t.Tick();const auto id=t.out.command.action_id;
    Check(id && t.out.command.action==MissionAction::kTurnLeftInPlace,"curve correction did not request one rotation");
    t.Tick({id,true,false,false});t.Tick();
    Check(curve.turns_remaining==3-turn,"curve rotation advanced before DONE");
    t.Tick({id,true,true,false});
  }
  Check(curve.phase==Phase::kBodyObserve && curve.head_attempts==2,"three DONEs did not start a fresh recheck");
  for(int i=0;i<10 && curve.phase==Phase::kBodyObserve;++i)t.Tick();
  Check(curve.phase==Phase::kDrain && curve.head_attempts==2,"failed recheck reset the scan budget");
  t.Tick();Check(curve.head_attempts==3 && t.out.command.camera_request==CameraRequest::kScanLeft,
                 "curve did not consume its third scan round");
  t.in.camera_feedback={CameraMode::kTransition,false};t.in.camera_scan={1,2,45,false,1,true};
  for(int i=0;i<10;++i)t.Tick();
  Check(curve.phase==Phase::kReturnMiss,"empty curve scan did not finish");
  t.in.camera_feedback={CameraMode::kForward,true};t.in.camera_scan={1,0,0,true};t.Tick();
  Check(curve.phase==Phase::kLocked && curve.head_attempts==3 && curve.skip_pending,
        "third failed curve round failed to preserve the course lock policy");
}

void NoRgbDeadlineAndDuplicateDone() {
  using Phase=MissionController::LineHeadSearch::Phase;
  Trial t(0);auto& scan=t.core.line_head_search_;t.core.line_state_=LineState::kRecoveryHeadScan;
  scan.phase=Phase::kAlignBody;scan.align_remaining=1;scan.align_action_id=77;
  t.core.ObserveLineFeedback({77,true,true,false},10);
  Check(scan.phase==Phase::kReobserve && scan.align_remaining==0,"transport DONE did not begin stationary time without RGB");
  t.core.ObserveLineFeedback({77,true,true,false},11);
  Check(scan.phase==Phase::kReobserve,"duplicate transport DONE changed scan phase");
  t.core.AdvanceScanDeadlines(11.99);Check(scan.phase==Phase::kReobserve,"stationary deadline expired early");
  t.core.AdvanceScanDeadlines(12);
  Check(scan.phase==Phase::kPendingLeft && scan.completed_pairs==1,"missing RGB prevented a two-second failure");
  t.core.AdvanceScanDeadlines(20);
  Check(scan.completed_pairs==1,"timer consumed a scan round that never started");
}

}
int main() { LossBudgetsAndFallback();OffCenterLineAndBoundedReobserve();GoalNormalVotesDepthAndDone();CurveThreeTurnsAndSharedRetryBudget();NoRgbDeadlineAndDuplicateDone(); }
