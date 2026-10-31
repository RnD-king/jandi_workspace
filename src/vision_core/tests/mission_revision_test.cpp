#include "vision_core/config_loader.hpp"
#include "vision_core/cruise_selector.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <limits>
using namespace vision_core;
namespace {
ObjectTarget Target(double u, double v=100, double width=20) {
  ObjectTarget t; t.center_px={u,v}; t.box_px={u-width/2,v-10,width,20};
  t.width_px=width;t.height_px=20;t.area_px=width*20;t.confidence=1;return t;
}
void VerifiedFineWindows() {
  const auto c=LoadDefaultAlgorithmConfig();
  const std::deque<bool> hits{true,false,true,true,false,true,true,false,true,true};
  const ActionExecutionFeedback done{true,false,true};
  BallController ball(c.ball,3); ball.StartTestExecute();
  ball.UseVerifiedObservation(Target(345,100),hits,640,480);
  auto b=ball.Compute(Target(0,0),640,480,1,{CameraMode::kDown,true},done);
  assert(b.action.action==MissionAction::kPickBall && b.tracked.stable);
  BallController single(c.ball,3);single.StartTestExecute();
  single.UseVerifiedObservation(Target(345,100),{true},640,480);
  b=single.Compute(Target(345,100),640,480,1,{CameraMode::kDown,true},done);
  assert(b.action.action==MissionAction::kNone && !single.ExecutionObservationReady());
  HurdleController hurdle(c.hurdle,3);hurdle.StartTestExecute();
  hurdle.UseVerifiedObservation(Target(320,270),hits,640,480);
  auto h=hurdle.Compute({},640,480,1,{CameraMode::kDown,true},done,{});
  assert(h.mode==HurdleMode::kContactWalk && h.action.action==MissionAction::kStepForwardHalf);
  GoalController goal(c.goal,3);goal.StartTestExecute();
  GoalPoseObservation pose{true,-.08,.5,0,1};
  goal.UseVerifiedObservation(Target(320),pose,hits,hits,640,480);
  auto g=goal.Compute({},{},{},640,480,1,true,{CameraMode::kGoal,true},done);
  assert(g.action.action==MissionAction::kShoot && g.fine_decision_pose_used);
}
void InvalidAndNearWindows() {
  auto c=LoadDefaultAlgorithmConfig();
  const std::deque<bool> hits(10,true), missing(10,false);
  const ActionExecutionFeedback done{true,false,true};
  BallController ball(c.ball,3);ball.StartTestExecute();
  ball.UseVerifiedObservation({},missing,640,480);
  auto b=ball.Compute({},640,480,1,{CameraMode::kDown,true},done);
  assert(b.mode==BallMode::kBallRecoveryDown);
  HurdleController hurdle(c.hurdle,3);hurdle.StartTestExecute();
  hurdle.UseVerifiedObservation({},missing,640,480);
  auto h=hurdle.Compute({},640,480,1,{CameraMode::kDown,true},done,{});
  assert(h.mode==HurdleMode::kRecoveryStep);
  GoalController goal(c.goal,3);goal.StartTestNear();
  const CameraFeedback camera{CameraMode::kGoal,true};
  goal.Compute({},{},{},640,480,0,true,camera,{});
  goal.UseVerifiedObservation(Target(320),{true,0,.7,0,1},hits,hits,640,480,hits);
  auto g=goal.Compute({},{},{},640,480,1,true,camera,{});
  assert(g.mode==GoalMode::kWaitFinePose); // no second near-detection window
}
void GoalRecovery(bool fine, bool left) {
  auto c=LoadDefaultAlgorithmConfig().goal;
  c.stable_window=c.stable_min_hits=c.lost_frames=1;c.smooth_alpha=1;
  GoalController goal(c,3);
  if(fine)goal.StartTestExecute();else goal.StartTestNear();
  const CameraFeedback camera{CameraMode::kGoal,true};
  const auto target=Target(left?150:550);
  if(!fine)goal.Compute({},{},{},640,480,0,true,camera,{},false);
  goal.Compute({},target,{true,0,1.5,0,1},640,480,.1,true,camera,{},false);
  auto g=goal.Compute({},{},{},640,480,.2,true,camera,{});
  assert(g.mode==GoalMode::kRecoveryTurn);
  g=goal.Compute({},{},{},640,480,.3,true,camera,{});
  const auto turn=fine ? (left?MissionAction::kTurnLeft:MissionAction::kTurnRight)
      : (left?MissionAction::kTurnLeftInPlace:MissionAction::kStepForwardRightOneHurdle);
  assert(g.action.action==turn && goal.RecoveryTurnsCompleted()==0);
  g=goal.Compute({},{},{},640,480,.4,true,camera,{false,true,false});
  assert(g.action.action==MissionAction::kNone && goal.RecoveryTurnsCompleted()==0);
  // Moving evidence cannot interrupt recovery or schedule the extra turn.
  g=goal.Compute({},target,{true,0,1.5,0,1},640,480,.5,true,camera,{false,true,false});
  assert(g.mode==GoalMode::kRecoveryTurn && goal.RecoveryTurnsCompleted()==0);
  g=goal.Compute({},target,{true,0,1.5,0,1},640,480,.6,true,camera,{true,false,false});
  assert(g.mode==GoalMode::kRecoveryTurn);
  g=goal.Compute({},target,{true,0,1.5,0,1},640,480,.7,true,camera,{false,false,true});
  assert(g.mode==GoalMode::kRecoveryExtraTurn && goal.RecoveryTurnsCompleted()==1);
  g=goal.Compute({},target,{true,0,1.5,0,1},640,480,.8,true,camera,{});
  assert(g.action.action==turn);
  g=goal.Compute({},target,{true,0,1.5,0,1},640,480,.9,true,camera,{true,false,true});
  assert(g.mode==(fine?GoalMode::kFineAdjust:GoalMode::kApproach));
  assert(goal.RecoveryTurnsCompleted()==0);
  // A fresh loss has exactly three completed turns before stationary fallback.
  goal.Reset();
  if(fine)goal.StartTestExecute();else goal.StartTestNear();
  if(!fine)goal.Compute({},{},{},640,480,1,true,camera,{},false);
  goal.Compute({},target,{true,0,1.5,0,1},640,480,1.1,true,camera,{},false);
  goal.Compute({},{},{},640,480,1.2,true,camera,{});
  g=goal.Compute({},{},{},640,480,1.3,true,camera,{});
  for(int i=1;i<=3;++i) {
    assert(g.action.action==turn);
    g=goal.Compute({},{},{},640,480,1.3+.2*i,true,camera,{true,false,true});
    assert(goal.RecoveryTurnsCompleted()==i);
  }
  assert(g.mode==(fine?GoalMode::kFineRecovery:GoalMode::kRecoverySearch));
  // Visible board + failed pose uses stationary pose recovery, never a turn.
  if(fine) {
    goal.StartTestExecute();
    goal.Compute({},target,{true,0,1,0,1},640,480,3,true,camera,{},false);
    g=goal.Compute({},target,{},640,480,3.1,true,camera,{});
    assert(g.mode==GoalMode::kFineRecovery && g.action.action==MissionAction::kStepForwardHalf);
  }
}
void GoalEdges() {
  auto c=LoadDefaultAlgorithmConfig().goal;c.stable_window=c.stable_min_hits=c.near_distance_min_hits=1;
  c.lost_frames=100; // Isolate edge-turn behavior from Depth-loss recovery.
  const CameraFeedback camera{CameraMode::kGoal,true};
  for(double margin:{0.0,35.0,50.0,80.0}) for(bool left:{false,true}) {
    c.edge_margin_px=margin;
    GoalController goal(c,3);goal.StartTestNear();
    goal.Compute({},{},{},640,480,0,true,camera,{});
    const auto edge=Target(left?margin+10:640-margin-10,100,20);
    auto g=goal.Compute({},edge,{},640,480,.1,true,camera,{});
    const auto turn=left?MissionAction::kTurnLeftInPlace:MissionAction::kStepForwardRightOneHurdle;
    assert(g.mode==GoalMode::kEdgeTurn && g.action.action==turn);
    // Stable reacquisition during either cycle cannot stop a fixed pair.
    g=goal.Compute({},Target(320),{},640,480,.2,true,camera,{false,true,false});
    assert(g.mode==GoalMode::kEdgeTurn && g.action.action==MissionAction::kNone);
    g=goal.Compute({},Target(320),{},640,480,.4,true,camera,{false,true,false});
    assert(g.mode==GoalMode::kEdgeTurn && g.action.action==MissionAction::kNone);
    // First actual DONE immediately requests the second single turn, even
    // without observation_complete and despite stable centered reacquisition.
    g=goal.Compute({},Target(320),{},640,480,.5,true,camera,{true,false,false});
    assert(g.mode==GoalMode::kEdgeTurn && g.action.action==turn);
    g=goal.Compute({},Target(320),{},640,480,.6,true,camera,{false,true,false});
    assert(g.mode==GoalMode::kEdgeTurn && g.action.action==MissionAction::kNone);
    g=goal.Compute({},Target(320),{},640,480,.7,true,camera,{true,false,false});
    assert(g.mode==GoalMode::kSearch && g.action.action==MissionAction::kNone);
    assert(goal.RecoveryTurnsCompleted()==0);
    // A later edge event starts a fresh two-turn budget.
    g=goal.Compute({},edge,{},640,480,.8,true,camera,{});
    assert(g.mode==GoalMode::kEdgeTurn && g.action.action==turn);
    g=goal.Compute({},Target(320),{},640,480,.9,true,camera,{true,false,false});
    assert(g.mode==GoalMode::kEdgeTurn && g.action.action==turn);
    g=goal.Compute({},Target(320),{},640,480,1.0,true,camera,{true,false,false});
    assert(g.mode==GoalMode::kSearch && g.action.action==MissionAction::kNone);
    // A box one pixel outside the margin does not trigger a pair.
    GoalController outside(c,3);outside.StartTestNear();
    outside.Compute({},{},{},640,480,0,true,camera,{});
    g=outside.Compute({},Target(left?margin+11:640-margin-11),{},640,480,.1,true,camera,{});
    assert(g.mode!=GoalMode::kEdgeTurn);
  }
  c.edge_margin_px=50;
  GoalController both(c,3);both.StartTestNear();
  both.Compute({},{},{},640,480,0,true,camera,{});
  auto g=both.Compute({},Target(320,100,600),{},640,480,.1,true,camera,{});
  assert(g.action.action==MissionAction::kNone && g.mode==GoalMode::kSearch);
  // Retuning a not-yet-committed pair re-evaluates the new boundary.
  GoalController preview(c,3);preview.StartTestNear();
  preview.Compute({},{},{},640,480,0,true,camera,{});
  g=preview.Compute({},Target(60),{},640,480,.1,true,camera,{},false);
  assert(g.action.action==MissionAction::kTurnLeftInPlace);
  c.edge_margin_px=20;preview.UpdateTuning(c);
  g=preview.Compute({},Target(60),{},640,480,.2,true,camera,{},false);
  assert(g.mode!=GoalMode::kEdgeTurn);
  // PAUSE can consume A DONE without authorizing B yet. Retuning in that
  // idle gap must preserve the accepted pair and its one remaining turn.
  c.edge_margin_px=50;
  GoalController paused(c,3);paused.StartTestNear();
  paused.Compute({},{},{},640,480,0,true,camera,{});
  g=paused.Compute({},Target(60),{},640,480,.1,true,camera,{});
  assert(g.action.action==MissionAction::kTurnLeftInPlace);
  g=paused.Compute({},Target(320),{},640,480,.2,true,camera,{true,false,false},false);
  assert(g.mode==GoalMode::kEdgeTurn);
  c.edge_margin_px=0;paused.UpdateTuning(c);
  g=paused.Compute({},Target(320),{},640,480,.3,true,camera,{},false);
  assert(g.mode==GoalMode::kEdgeTurn);
  g=paused.Compute({},Target(320),{},640,480,.4,true,camera,{});
  assert(g.action.action==MissionAction::kTurnLeftInPlace);
  g=paused.Compute({},Target(320),{},640,480,.5,true,camera,{true,false,false});
  assert(g.mode==GoalMode::kSearch && g.action.action==MissionAction::kNone);
}
MissionFrameInput Frame(double t) {
  MissionFrameInput f;f.now_sec=t;f.image_width=640;f.image_height=480;
  f.command_transport_enabled=true;f.advance_line_fsm=false;return f;
}
void BallEntryCountsOnlyDetections() {
  auto c=LoadDefaultAlgorithmConfig().ball;
  c.stable_window=10;c.entry_min_hits=6;c.stable_min_hits=7;
  c.upper_acquire_v_px=470;
  const CameraFeedback camera{CameraMode::kForward,true};
  const ActionExecutionFeedback moving{false,true,false};
  BallController ball(c,3);
  BallResult b;
  // Six real hits, then misses; permission blocks entry but retains evidence.
  for(int i=0;i<9;++i) {
    b=ball.Compute(i<6 ? std::optional<ObjectTarget>(Target(380,470)) : std::nullopt,
        640,480,.1*i,camera,moving,true,false);
    assert(b.mode==BallMode::kLineFollow);
  }
  b=ball.Compute({},640,480,.9,camera,moving);
  assert(b.entry_frames==10 && b.entry_hits==6);
  assert(b.mode==BallMode::kApproachBall && !b.tracked.visible && !b.tracked.stable);

  BallController five(c,3);
  for(int i=0;i<10;++i)
    b=five.Compute(i<5 ? std::optional<ObjectTarget>(Target(380,470)) : std::nullopt,
        640,480,.1*i,camera,moving);
  assert(b.entry_frames==10 && b.entry_hits==5 && b.mode==BallMode::kLineFollow);
  // Old hits falling out of the sliding window cannot be counted again.
  b=five.Compute(Target(380,470),640,480,1,camera,moving);
  assert(b.entry_hits==5 && b.mode==BallMode::kLineFollow);
  five.ClearEntryEvidence();
  b=five.Compute(Target(380,470),640,480,1.1,camera,moving);
  assert(b.entry_frames==1 && b.entry_hits==1 && b.mode==BallMode::kLineFollow);

  // The row threshold is inclusive; tracked stability alone cannot admit
  // six valid-row hits when fewer than six frames meet that threshold.
  BallController mixed(c,3);
  for(int i=0;i<10;++i)
    b=mixed.Compute(Target(380,i<5 ? 470 : 471),640,480,.1*i,camera,moving);
  assert(b.entry_hits==5 && b.tracked.stable && b.mode==BallMode::kLineFollow);
  BallController outside(c,3);
  for(int i=0;i<10;++i)
    b=outside.Compute(Target(380,471),640,480,.1*i,camera,moving);
  assert(b.entry_hits==0 && b.mode==BallMode::kLineFollow);

  // The relaxed entry threshold cannot relax verified FINE observation.
  BallController fine(c,3);fine.StartTestExecute();
  fine.UseVerifiedObservation(Target(345,100),
      {true,true,true,true,true,true,false,false,false,false},640,480);
  b=fine.Compute({},640,480,1,{CameraMode::kDown,true},{true,false,true});
  assert(!b.tracked.stable && b.action.action==MissionAction::kNone);
}

void BallEntryAndReadyQueue(bool already_queued) {
  auto c=LoadDefaultAlgorithmConfig();
  c.enable_ball=true;c.enable_goal=c.enable_hurdle=false;
  c.initial_has_ball=false;c.goal_trial_start_goal=false;
  c.ball.stable_window=10;c.ball.entry_min_hits=6;c.ball.stable_min_hits=7;
  c.ball.upper_acquire_v_px=470;
  MissionController core(c);
  auto frame=[&](double at) {
    auto f=Frame(at);f.advance_line_fsm=true;f.manual_observation_control=true;
    for(double v:{450.,400.,350.,300.,250.,200.,150.,100.})
      f.line_centers.push_back({c.line_features.image_center_u,v});
    return f;
  };
  auto f=frame(0);auto r=core.Step(f);
  assert(r.command.action==MissionAction::kLineStraightUnit);
  const auto a=r.command.action_id;
  for(int i=1;i<=10;++i) {
    f=frame(i*.1);
    if(!already_queued && (i<=5 || i==10)) f.ball_target=Target(380,470);
    f.delivery_feedback={a,true,false,i==10};r=core.Step(f);
    assert(r.active_mission==MissionType::kLine);
  }
  std::uint64_t last=a;
  double done_time=1.1;
  if(already_queued) {
    const auto b=r.command.action_id;
    assert(b && b!=a);
    for(int i=11;i<=16;++i) {
      f=frame(i*.1);f.ball_target=Target(380,470);
      f.delivery_feedback={b,true,false,false};r=core.Step(f);
      assert(r.active_mission==MissionType::kLine && r.command.action_id==b);
    }
    assert(r.candidate_mission==MissionType::kBall && r.ball.entry_hits==6);
    f=frame(1.7);f.delivery_feedback={a,true,true,false};r=core.Step(f);
    assert(!r.post_motion_observing && r.command.action_id==b);
    f=frame(1.8);f.delivery_feedback={b,true,false,true};r=core.Step(f);
    assert(r.command.action_id==b); // No C, even on B READY.
    last=b;done_time=1.9;
  } else {
    assert(r.candidate_mission==MissionType::kBall && r.ball.entry_hits==6);
    assert(r.command.action_id==a && r.command.control_phase!=ControlPhase::kWaitingQueuedActionAck);
  }
  f=frame(done_time);f.delivery_feedback={last,true,true,false};r=core.Step(f);
  assert(r.active_mission==MissionType::kLine && r.post_motion_observing);
  assert(r.command.action_id==0); // Physical DONE is consumed immediately; observer owns the hold.
  for(int i=1;i<=9;++i) {
    f=frame(done_time+.1*i);f.ball_target=Target(380,470);r=core.Step(f);
    assert(r.active_mission==MissionType::kLine && r.post_motion_observing);
    assert(r.command.command_type!=CommandType::kAction);
  }
  f=frame(done_time+1.01);f.ball_target=Target(380,470);r=core.Step(f);
  assert(r.active_mission==MissionType::kBall && !r.post_motion_observing);
  f=frame(done_time+1.02);f.ball_target=Target(380,470);r=core.Step(f);
  assert(r.command.command_type==CommandType::kAction && r.command.mission==MissionType::kBall);
  assert(r.command.action==MissionAction::kStepForwardTwo && r.command.action_id!=last);
}

void BallEntryUsesCandidateIdentity() {
  auto c=LoadDefaultAlgorithmConfig();
  c.enable_ball=true;c.enable_goal=c.enable_hurdle=false;
  c.initial_has_ball=false;c.goal_trial_start_goal=false;
  c.ball.entry_min_hits=6;c.ball.stable_window=10;c.ball.stable_min_hits=7;
  c.ball.upper_acquire_v_px=470;
  c.object_association.max_center_distance_norm=.001;
  c.object_association.missing_frame_limit=100;
  MissionController core(c);
  PerceptionFrameInput f;f.image_width=640;f.image_height=480;
  f.advance_line_fsm=false;f.allow_new_body_actions=false;f.manual_observation_control=true;
  PerceptionDetection d;d.detection.class_id=c.object_targets.ball_class_id;
  d.detection.confidence=1;d.detection.box={90,450,20,20};
  // A appears once, then B appears five times. Raw detections total six,
  // but neither identity satisfies 6/10. The approach association's much
  // tighter distance gate must not be applied to candidate tracklets.
  for(int i=0;i<6;++i) {
    f.now_sec=.1*i;d.detection.box.x=i==0 ? 90 : 490;f.detections={d};
    const auto out=core.StepPerception(f);
    if(i==5) {
      assert(out.perception.targets.ball && out.perception.targets.ball->center_px.u==500);
      assert(out.mission.ball.entry_hits==5);
      assert(out.perception.ball_tracks.size()==2);
    }
    assert(out.mission.active_mission==MissionType::kLine);
    assert(out.mission.candidate_mission==MissionType::kNone);
  }
  f.now_sec=.6;f.detections={d};
  const auto selected=core.StepPerception(f); // B's sixth actual detection.
  assert(selected.mission.active_mission==MissionType::kLine);
  assert(selected.mission.candidate_mission==MissionType::kBall);
  assert(selected.mission.ball.entry_hits==6);
  assert(selected.perception.ball_target_locked && selected.perception.targets.ball->center_px.u==500);
  MissionController filtered(c);
  for(int i=0;i<12;++i) {
    f.now_sec=.1*i;d.detection.confidence=c.object_targets.ball_confidence*.5;
    f.detections={d};const auto out=filtered.StepPerception(f);
    assert(out.mission.active_mission==MissionType::kLine && out.mission.ball.entry_hits==0);
  }
  MissionController outside(c);
  d.detection.confidence=1;d.detection.box.y=461; // Center v=471 > 470.
  for(int i=0;i<12;++i) {
    f.now_sec=.1*i;f.detections={d};const auto out=outside.StepPerception(f);
    assert(out.mission.active_mission==MissionType::kLine && out.mission.ball.entry_hits==0);
  }
}

void BallEntryRawFallbackIsExplicit() {
  auto c=LoadDefaultAlgorithmConfig();
  c.enable_ball=true;c.enable_goal=c.enable_hurdle=false;
  c.ball.entry_min_hits=6;c.ball.stable_window=10;
  c.ball_candidate_tracking.enabled=false;
  c.object_association.max_center_distance_norm=.001;
  c.object_association.missing_frame_limit=100;
  MissionController core(c);
  PerceptionFrameInput f;f.image_width=640;f.image_height=480;
  f.advance_line_fsm=false;f.allow_new_body_actions=false;f.manual_observation_control=true;
  PerceptionDetection d;d.detection.class_id=c.object_targets.ball_class_id;
  d.detection.confidence=1;d.detection.box={90,450,20,20};
  PerceptionMissionFrameResult out;
  for(int i=0;i<6;++i) {
    f.now_sec=.1*i;d.detection.box.x=i==0 ? 90 : 490;f.detections={d};
    out=core.StepPerception(f);
  }
  // Only explicitly disabling candidate tracking retains the old raw-count
  // admission path, independently of failed approach association.
  assert(!out.perception.targets.ball);
  assert(out.mission.active_mission==MissionType::kLine);
  assert(out.mission.candidate_mission==MissionType::kBall && out.mission.ball.entry_hits==6);
}

void DirectGoalStartAndReset() {
  auto c=LoadDefaultAlgorithmConfig();
  c.enable_ball=c.enable_hurdle=false;c.enable_goal=true;
  c.initial_has_ball=true;c.goal_trial_start_goal=true;
  c.transition_settle_sec=.6;c.post_motion_observation_sec=1;
  c.goal.stable_window=10;c.goal.stable_min_hits=7;
  for(auto initial:{CameraMode::kForward,CameraMode::kGoal}) {
    MissionController core(c);
    auto f=Frame(0);f.camera_feedback={initial,true};
    f.allow_new_commands=false;
    auto r=core.Step(f);
    assert(r.command.camera_request==CameraRequest::kNone && r.command.action_id==0);
    f.now_sec=.1;f.allow_new_commands=true;r=core.Step(f);
    assert(r.active_mission==MissionType::kLine && r.command.action_id==0);
    assert(r.command.camera_request==(initial==CameraMode::kForward ? CameraRequest::kGoal : CameraRequest::kNone));
    f.now_sec=.2;f.camera_feedback={CameraMode::kTransition,false};r=core.Step(f);
    assert(r.command.action_id==0);
    f.camera_feedback={CameraMode::kGoal,true};f.advance_line_fsm=true;
    f.backboard_target=Target(320);f.backboard_center_depth_m=1.6;
    for(int i=0;i<12;++i){f.now_sec=.5+.1*i;r=core.Step(f);}
    assert(r.active_mission==MissionType::kLine && r.approach_mission==MissionType::kGoal);
    assert(LinePlan::IsUnit(r.command.action) && r.command.action_id);
  }
  MissionController reset(c);reset.BeginPhysicalReset();
  auto r=reset.StepPhysicalReset({}, {CameraMode::kGoal,true},0);
  assert(r.command.action==MissionAction::kWalkMode && r.command.camera_request==CameraRequest::kGoal);
  assert(!r.physical_reset_complete);
  const auto id=r.command.action_id;
  r=reset.StepPhysicalReset({id,true,true,false},{CameraMode::kTransition,false},.1);
  assert(!r.physical_reset_complete);
  r=reset.StepPhysicalReset({},{CameraMode::kForward,true},.2);
  assert(!r.physical_reset_complete && r.command.camera_request==CameraRequest::kGoal);
  r=reset.StepPhysicalReset({},{CameraMode::kGoal,true},.3);
  assert(r.physical_reset_complete);
  auto f=Frame(.4);f.camera_feedback={CameraMode::kGoal,true};
  r=reset.Step(f);
  assert(r.active_mission==MissionType::kLine && r.command.action_id==0);
  // Default false retains LINE entry and FORWARD reset.
  c.goal_trial_start_goal=false;
  MissionController normal(c);f=Frame(0);r=normal.Step(f);
  assert(r.active_mission==MissionType::kLine);
  normal.BeginPhysicalReset();
  r=normal.StepPhysicalReset({}, {CameraMode::kGoal,true},1);
  assert(r.command.camera_request==CameraRequest::kForward);
}
void GoalPairHasOneObservation(bool left) {
  auto c=LoadDefaultAlgorithmConfig();
  c.test_mission=MissionType::kGoal;c.test_stage=1;
  c.enable_ball=c.enable_hurdle=false;c.initial_has_ball=true;
  c.transition_settle_sec=0;
  c.goal.stable_window=c.goal.stable_min_hits=c.goal.near_distance_min_hits=1;
  MissionController core(c);
  auto f=Frame(0);f.camera_feedback={CameraMode::kGoal,true};
  // The CTest fixture's near threshold is 1.20 m; keep this edge-turn
  // scenario in the far approach instead of entering FINE on equality.
  f.backboard_target=Target(left?60:580);f.goal_pose={true,0,1.5,0,1};
  f.backboard_center_depth_m=1.5;
  const auto turn=left?MissionAction::kTurnLeftInPlace:MissionAction::kStepForwardRightOneHurdle;
  MissionFrameResult r;
  for(int i=0;i<100;++i) {
    f.now_sec=.1*i;f.delivery_feedback={};
    if(r.command.action_id)f.delivery_feedback={r.command.action_id,true,true,false};
    r=core.Step(f);
    if(r.command.action==turn)break;
  }
  assert(r.command.action==turn && r.repeat_sequence);
  const auto first_id=r.command.action_id;
  const int scope=r.repeat_scope;
  f.backboard_target=Target(320);
  for(int i=0;i<5;++i) {
    f.now_sec+=.1;f.delivery_feedback={first_id,true,false,false};r=core.Step(f);
    assert(!r.post_motion_observing && r.repeat_scope==scope);
  }
  // A's actual DONE dispatches B with a new ID in this same frame.
  f.now_sec+=.1;f.delivery_feedback={first_id,true,true,false};r=core.Step(f);
  assert(!r.post_motion_observing && r.repeat_scope==scope);
  assert(r.command.command_type==CommandType::kAction && r.command.action==turn);
  const auto second_id=r.command.action_id;
  assert(second_id!=0 && second_id!=first_id);
  // A repeated/late A DONE must not finish B or issue a third turn.
  for(int i=0;i<5;++i) {
    f.now_sec+=.1;
    f.delivery_feedback=i==0 ? CommandDeliveryFeedback{first_id,true,true,false}
        : CommandDeliveryFeedback{second_id,true,false,false};
    r=core.Step(f);
    assert(!r.post_motion_observing && r.repeat_scope==scope);
    assert(r.command.action_id==second_id);
  }
  f.now_sec+=.1;const double done_at=f.now_sec;
  // PAUSE may receive physical DONE before its next sensory frame. Changing
  // only the margin must keep that edge available for the normal observation.
  core.AcceptDeliveryFeedback({second_id,true,true,false},f.now_sec);
  core.SetGoalEdgeMargin(40);
  f.delivery_feedback={second_id,true,true,false};r=core.Step(f);
  assert(r.post_motion_observing && !r.repeat_sequence && r.command.action_id==0);
  for(int i=1;i<10;++i) {
    f.now_sec=done_at+.1*i;f.delivery_feedback={};r=core.Step(f);
    assert(r.post_motion_observing && !r.repeat_sequence && r.command.action_id==0);
  }
  f.now_sec=done_at+1;f.delivery_feedback={};r=core.Step(f);
  assert(!r.post_motion_observing && r.goal.mode!=GoalMode::kEdgeTurn);
}
void PoseSettleAndConcurrentObservation() {
  auto c=LoadDefaultAlgorithmConfig();c.test_mission=MissionType::kBall;c.test_stage=3;
  c.enable_goal=c.enable_hurdle=false;
  c.transition_settle_sec=.6;c.post_motion_observation_sec=1;
  MissionController core(c);
  auto f=Frame(0);f.camera_feedback={CameraMode::kDown,true};
  auto r=core.Step(f);assert(r.command.action==MissionAction::kDefaultPoseMode);
  const auto id=r.command.action_id;
  const double pose_done_at=.1;
  f.now_sec=pose_done_at;f.delivery_feedback={id,true,true,false};r=core.Step(f);
  // The camera was already settled during setup: there is no new camera
  // completion edge. Pose DONE starts .6s of settle before observation.
  const double observe_at=pose_done_at+c.transition_settle_sec;
  const double decision_at=observe_at+c.post_motion_observation_sec;
  for(int i=2;.1*i+1e-9<observe_at;++i) {
    f.now_sec=.1*i;f.delivery_feedback={};f.ball_target=Target(345,100);r=core.Step(f);
    assert(r.command.action_id==0 && !r.post_motion_observing);
    assert(r.post_motion_frames==0 && r.post_motion_window_frames==0);
  }
  // Fresh evidence and the one-second timer accumulate together after settle.
  for(int i=7;.1*i+1e-9<decision_at;++i) {
    f.now_sec=.1*i;f.delivery_feedback={};f.ball_target=Target(345,100);r=core.Step(f);
    assert(r.command.action_id==0 && r.post_motion_observing);
    assert(r.post_motion_window_frames>0);
  }
  f.now_sec=decision_at;f.delivery_feedback={};f.ball_target=Target(345,100);r=core.Step(f);
  assert(!r.post_motion_observing && r.command.action==MissionAction::kPickBall);
}
void FineObservationTiming(double extra_settle, bool manual=false,
                            double observation_sec=1.0, bool trailing_misses=false) {
  auto c=LoadDefaultAlgorithmConfig();c.test_mission=MissionType::kBall;c.test_stage=2;
  c.enable_goal=c.enable_hurdle=false;c.fine_motion_settle_sec=extra_settle;
  c.post_motion_observation_sec=observation_sec;
  MissionController core(c);
  auto f=Frame(0);f.camera_feedback={CameraMode::kDown,true};
  MissionFrameResult r;
  std::uint64_t setup_done=0;
  for(int i=0;i<=240;++i) {
    f.now_sec=.05*i;f.ball_target=Target(345,50);f.delivery_feedback={};
    if(r.command.action_id!=0 && r.command.action_id!=setup_done) {
      setup_done=r.command.action_id;f.delivery_feedback={setup_done,true,true,false};
    }
    r=core.Step(f);
    if(r.command.action==MissionAction::kStepForwardHalf)break;
  }
  assert(r.command.action==MissionAction::kStepForwardHalf);
  const auto motion_id=r.command.action_id;
  f.now_sec+=.1;const double done_at=f.now_sec;
  f.manual_observation_control=manual;
  f.delivery_feedback={motion_id,true,true,false};f.ball_target=Target(345,150);
  r=core.Step(f);if(!manual)assert(r.post_motion_observing);
  const double decision_at=done_at+std::max(extra_settle,manual?.05:0.0)+observation_sec;
  bool decided=false;
  for(int i=1;i<=80;++i) {
    f.now_sec=done_at+(trailing_misses ? .1*i+(i>=7?.1:0.0) : .05*i);
    if(trailing_misses && i>=7)f.ball_target.reset();
    f.begin_manual_observation=manual && i==1;
    f.delivery_feedback=i==2 ? CommandDeliveryFeedback{motion_id,true,true,false} : CommandDeliveryFeedback{};
    r=core.Step(f);
    if(f.now_sec+1e-9<decision_at) {
      assert(r.command.action!=MissionAction::kStepBack);
    } else {
      assert(r.command.action==MissionAction::kStepBack);
      assert(f.now_sec<=decision_at+.051);
      decided=true;break;
    }
  }
  assert(decided); // No extra ten-frame/FINE wait after the completed window.
}
void RecoveryWithoutLastDetectionDeadline() {
  for(int hits:{6,7}) {
    StationaryObjectRecovery recovery;recovery.Begin(0);
    StationaryObjectRecovery::Result result{};
    // Seven hits followed by three misses: the final detection is 0.4 s old.
    for(int i=0;i<10;++i) {
      const double now=i<7?.1*i:.1*(i+1);
      result=recovery.Observe(i<hits,now,10,7,1,2);
      if(i<9)assert(result==StationaryObjectRecovery::Result::kWaiting);
    }
    assert(result==(hits==7 ? StationaryObjectRecovery::Result::kFound :
                             StationaryObjectRecovery::Result::kRetry));
  }
}
void FineTurnObservationTiming(bool left) {
  auto c=LoadDefaultAlgorithmConfig();c.test_mission=MissionType::kGoal;c.test_stage=2;
  c.enable_ball=c.enable_hurdle=false;c.transition_settle_sec=0;
  c.fine_motion_settle_sec=.25;c.post_motion_observation_sec=1;
  c.goal.lost_frames=1;c.goal.smooth_alpha=1;
  MissionController core(c);
  auto f=Frame(0);f.camera_feedback={CameraMode::kGoal,true};
  f.allow_new_body_actions=true;f.allow_test_setup_body_actions=true;
  f.backboard_target=Target(left?150:550);f.goal_pose={true,0,.7,0,1};
  f.backboard_center_depth_m=.7;
  MissionFrameResult r;
  std::uint64_t setup_done=0;
  for(int i=0;i<=100;++i) {
    f.now_sec=.05*i;f.delivery_feedback={};
    if(r.command.action_id!=0 && r.command.action_id!=setup_done) {
      setup_done=r.command.action_id;f.delivery_feedback={setup_done,true,true,false};
    }
    r=core.Step(f);
    if(r.goal.mode==GoalMode::kFineAdjust && r.goal.tracked.stable &&
       !r.post_motion_observing)break;
  }
  assert(r.goal.mode==GoalMode::kFineAdjust && r.goal.tracked.stable);
  if(r.command.action_id) {
    const auto pending=r.command.action_id;
    f.now_sec+=.05;
    f.allow_new_body_actions=false;
    f.delivery_feedback={pending,true,true,false};
    r=core.Step(f);
  }
  f.allow_new_body_actions=true;f.backboard_target.reset();
  f.goal_pose={};f.backboard_center_depth_m.reset();
  const auto turn=left?MissionAction::kTurnLeft:MissionAction::kTurnRight;
  for(int i=0;i<100;++i) {
    f.now_sec+=.05;f.delivery_feedback={};r=core.Step(f);
    if(r.command.action==turn && r.command.action_id!=0)break;
  }
  assert(r.command.action==turn && r.command.action_id!=0);
  const auto motion_id=r.command.action_id;
  f.now_sec+=.1;const double done_at=f.now_sec;
  f.delivery_feedback={motion_id,true,true,false};r=core.Step(f);
  assert(r.post_motion_observing && r.post_motion_frames==0);
  const double decision_at=done_at+c.fine_motion_settle_sec+c.post_motion_observation_sec;
  bool decided=false;
  for(int i=1;i<=40;++i) {
    f.now_sec=done_at+.05*i;f.delivery_feedback={};r=core.Step(f);
    if(f.now_sec+1e-9<decision_at) {
      assert(r.post_motion_observing);
      assert(r.command.action_id==0 || r.command.action_id==motion_id);
      if(f.now_sec+1e-9<done_at+c.fine_motion_settle_sec)
        assert(r.post_motion_frames==0);
    } else {
      assert(!r.post_motion_observing);
      assert(r.command.action==turn && r.command.action_id!=0 &&
             r.command.action_id!=motion_id);
      decided=true;break;
    }
  }
  assert(decided);
}
void ImageDetectionWithMissingDepth() {
  auto c=LoadDefaultAlgorithmConfig();c.test_mission=MissionType::kGoal;c.test_stage=3;
  c.enable_ball=c.enable_hurdle=false;c.goal.lost_frames=1;
  MissionController core(c);
  PerceptionFrameInput f;f.image_width=640;f.image_height=480;
  f.intrinsics={600,600,320,240};f.camera_feedback={CameraMode::kGoal,true};
  f.command_transport_enabled=true;f.allow_new_body_actions=false;f.allow_test_setup_body_actions=true;
  PerceptionDetection d;d.detection.class_id=c.object_targets.backboard_class_id;
  d.detection.confidence=1;d.detection.box={140,90,20,20};
  d.center_depth_m=d.left_depth_m=d.right_depth_m=.5;f.detections={d};
  auto out=core.StepPerception(f);const auto pose=out.mission.command.action_id;
  f.now_sec=.1;f.delivery_feedback={pose,true,true,false};core.StepPerception(f);
  for(int i=2;i<=40;++i) {f.now_sec=.1*i;f.delivery_feedback={};out=core.StepPerception(f);}
  assert(out.mission.goal.mode==GoalMode::kFineAdjust && out.mission.goal.tracked.stable);
  f.detections[0].center_depth_m.reset();
  f.detections[0].left_depth_m.reset();f.detections[0].right_depth_m.reset();
  f.now_sec+=.1;out=core.StepPerception(f);
  assert(!out.perception.targets.backboard);
  assert(out.mission.goal.mode==GoalMode::kFineRecovery);
  assert(out.mission.goal.action.action==MissionAction::kStepForwardHalf);
}
void FineSidePairDirect(bool left) {
  auto c=LoadDefaultAlgorithmConfig().ball;
  c.stable_window=10;c.stable_min_hits=7;c.fine_side_two_step_margin_px=100;
  const auto side=left?MissionAction::kLeftSideStep:MissionAction::kRightSideStep;
  const auto target=Target(left?c.pickup_u1_px-100:c.pickup_u2_px+100,c.PickupCenterV());
  const std::deque<bool> hits(10,true);
  const CameraFeedback down{CameraMode::kDown,true};
  BallController ball(c,3);ball.StartTestExecute();
  ball.UseVerifiedObservation(target,hits,640,480);
  auto b=ball.Compute({},640,480,0,down,{},false);
  assert(b.mode==BallMode::kFineAdjustForPickup && b.action.action==side);
  // Preview is reversible: a changed threshold re-evaluates the same evidence.
  auto preview=ball;
  b=preview.Compute(target,640,480,.01,down,{});
  assert(b.mode==BallMode::kFineSidePair && b.action.action==side);
  auto changed=c;changed.fine_side_two_step_margin_px=101;ball.UpdateTuning(changed);
  b=ball.Compute(target,640,480,.02,down,{});
  assert(b.mode==BallMode::kFineAdjustForPickup && b.action.action==side);
  ball.UpdateTuning(c);
  b=ball.Compute(target,640,480,.03,down,{});
  assert(b.mode==BallMode::kFineSidePair && b.action.action==side);
  // Neither opposite detections nor loss can change an accepted fixed pair.
  b=ball.Compute(Target(left?600:50),640,480,.1,down,{false,true,false});
  assert(b.mode==BallMode::kFineSidePair && b.action.action==MissionAction::kNone);
  // PAUSE consumes A DONE but does not authorize B. Repeated DONE while idle
  // cannot decrement the remaining count again.
  b=ball.Compute({},640,480,.2,down,{true,false,false},false);
  assert(b.mode==BallMode::kFineSidePair);
  b=ball.Compute({},640,480,.21,down,{true,false,false},false);
  assert(b.mode==BallMode::kFineSidePair);
  b=ball.Compute({},640,480,.3,down,{});
  assert(b.mode==BallMode::kFineSidePair && b.action.action==side);
  b=ball.Compute({},640,480,.4,down,{true,false,false});
  assert(b.mode==BallMode::kFineAdjustForPickup && b.action.action==MissionAction::kNone);
  assert(!ball.ExecutionObservationReady());
  ball.Reset();
  assert(ball.Compute({},640,480,.5,{},{}).mode==BallMode::kLineFollow);
}

void FineSidePairTransport(bool left, bool manual) {
  auto c=LoadDefaultAlgorithmConfig();c.test_mission=MissionType::kBall;c.test_stage=2;
  c.enable_goal=c.enable_hurdle=false;c.ball.fine_side_two_step_margin_px=100;
  c.fine_motion_settle_sec=.2;
  MissionController core(c);
  auto f=Frame(0);f.camera_feedback={CameraMode::kDown,true};
  const auto side=left?MissionAction::kLeftSideStep:MissionAction::kRightSideStep;
  const auto target=Target(left?c.ball.pickup_u1_px-100:c.ball.pickup_u2_px+100,c.ball.PickupCenterV());
  MissionFrameResult r;std::uint64_t setup_done=0;
  for(int i=0;i<=180;++i) {
    f.now_sec=.05*i;f.ball_target=target;f.delivery_feedback={};
    if(r.command.action_id && r.command.action_id!=setup_done) {
      setup_done=r.command.action_id;f.delivery_feedback={setup_done,true,true,false};
    }
    r=core.Step(f);
    if(r.command.action==side)break;
  }
  assert(r.command.action==side && r.ball.mode==BallMode::kFineSidePair && r.repeat_sequence);
  const auto a=r.command.action_id;f.manual_observation_control=manual;
  f.now_sec+=.1;f.ball_target=Target(left?600:50,100);
  f.delivery_feedback={a,true,true,false};r=core.Step(f);
  const auto b=r.command.action_id;
  assert(b && b!=a && r.command.action==side && !r.post_motion_observing);
  // The fixed pair must also bypass fine_motion_settle_sec between its actions.
  f.now_sec+=.01;f.delivery_feedback={a,true,true,false};r=core.Step(f);
  assert(r.command.action_id==b && r.command.action==side);
  f.now_sec+=.3;const auto done_at=f.now_sec;
  f.ball_target=Target(c.ball.pickup_u1_px-10,c.ball.PickupCenterV());
  f.delivery_feedback={b,true,true,false};r=core.Step(f);
  assert(!r.repeat_sequence && r.command.action_id==0);
  const double begin=done_at+std::max(manual?.05:0,c.fine_motion_settle_sec);
  bool decided=false;
  for(int i=1;i<=40;++i) {
    f.now_sec=done_at+.05*i;f.delivery_feedback={};
    f.begin_manual_observation=manual && i==1;r=core.Step(f);
    if(f.now_sec+1e-9<begin+1) assert(r.command.action_id==0);
    else {
      assert(r.command.action==MissionAction::kLeftSideStep);
      assert(r.ball.mode==BallMode::kFineAdjustForPickup && !r.repeat_sequence);
      decided=true;break;
    }
  }
  assert(decided); // Exactly one common observation, with concurrent 10/7.
}

void EntryDirectionRecovery(bool left, bool reacquire) {
  auto c=LoadDefaultAlgorithmConfig().ball;
  c.entry_min_hits=6;c.stable_min_hits=7;c.stable_window=10;
  c.recovery_max_steps=1;c.entry_recovery_max_steps=3;
  c.recovery_center_tolerance_px=50;
  c.recovery_observation_sec=1;c.recovery_max_observations=5;
  BallController ball(c,3);
  const auto target=Target(c.far_u_des_px+(left?-120:120),100);
  const CameraFeedback forward{CameraMode::kForward,true};
  BallResult b;
  for(int i=0;i<6;++i)
    b=ball.Compute(target,640,480,.1*i,forward,{false,true,false});
  assert(b.mode==BallMode::kApproachBall && b.entry_hits==6 && !b.tracked.stable);
  ball.UseVerifiedObservation({},std::deque<bool>(10,false),640,480);
  b=ball.Compute({},640,480,2,forward,{true,false,true});
  assert(b.mode==BallMode::kForwardRecoveryTurn && ball.RecoveryStepLimit()==3);
  const auto turn=left?MissionAction::kTurnLeftInPlace:MissionAction::kStepForwardRightOneHurdle;
  double now=2;
  for(int n=1;n<=3;++n) {
    b=ball.Compute({},640,480,now+=.1,forward,{});
    assert(b.action.action==turn);
    b=ball.Compute(target,640,480,now+=.1,forward,{false,true,false});
    assert(b.mode==BallMode::kForwardRecoveryTurn && ball.RecoveryStepsCompleted()==n-1);
    b=ball.Compute({},640,480,now+=.1,forward,{true,false,false});
    assert(b.mode==BallMode::kForwardRecoveryStep && ball.RecoveryWalkTurnsCompleted()==n);
    b=ball.Compute({},640,480,now+=.1,forward,{});
    assert(b.action.action==MissionAction::kStepForwardOne);
    b=ball.Compute({},640,480,now+=.1,forward,{true,false,false});
    assert(b.mode==BallMode::kForwardRecoveryObserve && ball.RecoveryStepsCompleted()==n);
    for(int i=0;i<10;++i)
      b=ball.Compute(reacquire?std::optional<ObjectTarget>(target):std::nullopt,
                     640,480,now+=.1,forward,{});
    if(reacquire) {
      assert(b.mode==BallMode::kApproachBall && ball.RecoveryWalkTurnsCompleted()==0);
      return;
    }
    assert(b.mode==(n<3?BallMode::kForwardRecoveryTurn:BallMode::kLineFollow));
  }
  assert(!b.active && !b.has_ball && !b.pickup_failed);
  ball.Reset();assert(ball.RecoveryStepsCompleted()==0);
}

void EntryDirectionTransport(bool left) {
  auto c=LoadDefaultAlgorithmConfig();c.enable_ball=true;c.enable_goal=c.enable_hurdle=false;
  c.ball.entry_min_hits=6;c.ball.stable_min_hits=7;c.ball.stable_window=10;
  c.initial_has_ball=false;c.goal_trial_start_goal=false;
  MissionController core(c);
  auto f=Frame(0);f.advance_line_fsm=true;f.manual_observation_control=true;
  for(double v:{450.,400.,350.,300.})f.line_centers.push_back({c.line_features.image_center_u,v});
  auto r=core.Step(f);const auto a=r.command.action_id;
  assert(r.command.action==MissionAction::kLineStraightUnit);
  for(int i=1;i<=6;++i) {
    f.now_sec=.1*i;f.ball_target=Target(c.ball.far_u_des_px+(left?-120:120),100);
    f.delivery_feedback={a,true,false,false};r=core.Step(f);
  }
  assert(r.active_mission==MissionType::kLine && r.candidate_mission==MissionType::kBall);
  assert(!r.ball.tracked.stable);
  f.now_sec=.7;f.ball_target.reset();f.delivery_feedback={a,true,true,false};r=core.Step(f);
  assert(r.post_motion_observing);
  for(int i=1;i<=20;++i) {
    f.now_sec=.7+.05*i;f.delivery_feedback={};r=core.Step(f);
  }
  assert(r.active_mission==MissionType::kLine && r.candidate_mission==MissionType::kBall);
  assert(r.candidate_search_turns==0);
  f.now_sec=1.75;r=core.Step(f);
  assert(r.command.action==(left?MissionAction::kTurnLeftInPlace:MissionAction::kStepForwardRightOneHurdle));
}

void EntryDirectionCentralFallback() {
  auto c=LoadDefaultAlgorithmConfig().ball;c.entry_min_hits=6;
  c.stable_window=10;c.stable_min_hits=7;
  BallController ball(c,3);BallResult b;
  const CameraFeedback forward{CameraMode::kForward,true};
  for(int i=0;i<6;++i)
    b=ball.Compute(Target(c.far_u_des_px,100),640,480,.1*i,forward,{false,true,false});
  ball.UseVerifiedObservation({},std::deque<bool>(10,false),640,480);
  b=ball.Compute({},640,480,2,forward,{true,false,true});
  assert(b.mode==BallMode::kForwardRecoveryObserve);
  b=ball.Compute({},640,480,2.1,forward,{});
  assert(b.action.action==MissionAction::kNone);
  ball.Reset();
  b=ball.Compute({},640,480,3,forward,{});
  assert(b.mode==BallMode::kLineFollow && b.entry_hits==0 && ball.RecoveryWalkTurnsCompleted()==0);
}

void GoalEntryFirstObservation(bool left, bool reacquire, int prior_hits) {
  auto c=LoadDefaultAlgorithmConfig().goal;
  c.entry_line_retry_max_attempts=0; // Explicitly exercise the legacy directional recovery option.
  c.stable_window=10;c.stable_min_hits=7;c.post_pickup_walk_fallback_actions=1;
  c.recovery_turn_max=3;c.recovery_max_observations=5;c.recovery_observation_sec=1;
  c.edge_margin_px=50;
  const auto board=Target(c.target_u_px+(left?-140:140));
  GoalController goal(c,3);goal.StartAfterPickup(0);
  BallResult carrying;carrying.has_ball=true;goal.UpdateBallState(carrying);
  const CameraFeedback forward{CameraMode::kForward,true}, camera{CameraMode::kGoal,true};
  GoalResult g;
  for(int i=0;i<prior_hits;++i) {
    g=goal.Compute({},board,{true,0,1.5,0,1},640,480,.05*i,true,forward,{},false);
    assert(g.mode==GoalMode::kLineFollow);
  }
  g=goal.Compute({},{},{},640,480,1,true,forward,{},false,{},true);
  assert(g.mode==GoalMode::kWaitCameraGoal);
  // A transitional image in the opposite direction cannot replace memory.
  goal.Compute({},Target(c.target_u_px+(left?140:-140)),{},640,480,1.1,true,
      {CameraMode::kTransition,false},{},false);
  g=goal.Compute({},{},{},640,480,1.2,true,camera,{},false);
  assert(g.mode==GoalMode::kSearch && !g.tracked.visible && !g.tracked.stable);
  // No turn before the first complete observation, even with total loss.
  g=goal.Compute({},{},{},640,480,1.3,true,camera,{});
  assert(g.mode==GoalMode::kSearch && g.action.action==MissionAction::kNone);
  goal.ClearObservationHistory();
  goal.UseVerifiedObservation({}, {}, std::deque<bool>(10,false),
      std::deque<bool>(10,false),640,480);
  g=goal.Compute({},{},{},640,480,2.8,true,camera,{false,false,true});
  assert(g.mode==GoalMode::kRecoveryTurn && !g.tracked.stable);
  g=goal.Compute({},{},{},640,480,2.85,true,camera,{});
  const auto turn=left?MissionAction::kTurnLeftInPlace:MissionAction::kStepForwardRightOneHurdle;
  assert(g.action.action==turn && goal.RecoveryTurnsCompleted()==0);
  if(reacquire) {
    for(int i=1;i<=10;++i)
      g=goal.Compute({},board,{true,0,1.5,0,1},640,480,2.85+.05*i,true,camera,{false,true,false});
    assert(g.mode==GoalMode::kRecoveryTurn && goal.RecoveryTurnsCompleted()==0);
    goal.UseVerifiedObservation(board,{true,0,1.5,0,1},
        std::deque<bool>(10,true),std::deque<bool>(10,true),640,480);
    g=goal.Compute({},board,{true,0,1.5,0,1},640,480,4.5,true,camera,{true,false,true});
    assert(g.mode==GoalMode::kRecoveryExtraTurn && goal.RecoveryTurnsCompleted()==1);
    g=goal.Compute({},{},{},640,480,4.6,true,camera,{});
    assert(g.action.action==turn);
    goal.UseVerifiedObservation(board,{true,0,1.5,0,1},
        std::deque<bool>(10,true),std::deque<bool>(10,true),640,480);
    g=goal.Compute({},board,{true,0,1.5,0,1},640,480,5.8,true,camera,{true,false,true});
    assert(g.mode==GoalMode::kSearch && goal.RecoveryTurnsCompleted()==0);
    assert(g.tracked.visible && g.tracked.stable);
    return;
  }
  for(int i=1;i<=3;++i) {
    g=goal.Compute({},{},{},640,480,2.85+.2*i,true,camera,{false,true,false});
    assert(goal.RecoveryTurnsCompleted()==i-1);
    g=goal.Compute({},{},{},640,480,2.9+.2*i,true,camera,{true,false,true});
    assert(goal.RecoveryTurnsCompleted()==i);
    if(i<3) assert(g.action.action==turn);
  }
  assert(g.mode==GoalMode::kRecoverySearch && g.action.action==MissionAction::kNone);
  for(int i=1;i<=100 && g.mode!=GoalMode::kFailed;++i)
    g=goal.Compute({},{},{},640,480,3.5+.1*i,true,camera,{});
  assert(g.mode==GoalMode::kFailed && goal.RecoveryObservationsCompleted()==5);
}

void GoalEntryFallbackAndDepth() {
  auto c=LoadDefaultAlgorithmConfig().goal;c.stable_window=10;c.stable_min_hits=7;
  c.entry_line_retry_max_attempts=0; // Legacy fallback remains available when LINE retries are disabled.
  c.recovery_turn_max=3;c.lost_frames=1;
  const CameraFeedback forward{CameraMode::kForward,true}, camera{CameraMode::kGoal,true};
  const std::deque<bool> misses(10,false), hits(10,true);
  // Known center advances once; missing direction, disabled turns and
  // image-only Depth failure remain stationary.
  for(int scenario=0;scenario<4;++scenario) {
    auto cfg=c;if(scenario==2)cfg.recovery_turn_max=0;
    GoalController goal(cfg,3);goal.StartGoalTrial();
    const auto board=scenario==0 ? std::optional<ObjectTarget>{} :
        std::optional<ObjectTarget>(Target(cfg.target_u_px+(scenario==1?0:-140)));
    goal.Compute({},board,{},640,480,0,true,forward,{});
    auto g=goal.Compute({},{},{},640,480,.1,true,camera,{});
    assert(g.mode==GoalMode::kSearch);
    goal.UseVerifiedObservation({}, {},misses,misses,640,480);
    g=goal.Compute({},{},{},640,480,2,true,camera,{false,false,true},true,{},false,scenario==3);
    assert(g.mode==(scenario==1?GoalMode::kRecoveryForwardStep:GoalMode::kRecoverySearch));
    assert(g.action.action==(scenario==3 ? MissionAction::kStepForwardOne : MissionAction::kNone));
    // RESET must also discard an earlier directional sighting.
    goal.StartGoalTrial();goal.Compute({},{},{},640,480,3,true,camera,{});
    goal.UseVerifiedObservation({}, {},misses,misses,640,480);
    g=goal.Compute({},{},{},640,480,4.5,true,camera,{false,false,true});
    assert(g.mode==GoalMode::kRecoverySearch);
  }
  // A single real GOAL-view sighting can preserve direction independently
  // of stable detection; the first window itself remains invalid.
  GoalController brief(c,3);brief.StartGoalTrial();
  brief.Compute({},{},{},640,480,0,true,camera,{});
  brief.ObserveOnly(Target(c.target_u_px+140),{},640,480);
  brief.UseVerifiedObservation({}, {},misses,misses,640,480);
  auto g=brief.Compute({},{},{},640,480,1.5,true,camera,{false,false,true});
  assert(g.mode==GoalMode::kRecoveryTurn && !g.tracked.stable);
  g=brief.Compute({},{},{},640,480,1.55,true,camera,{});
  assert(g.action.action==MissionAction::kStepForwardRightOneHurdle);

  // Three verified near center-Depth samples enter FINE even when pose
  // estimation fails; an entry search turn must not preempt that decision.
  GoalController pose(c,3);pose.StartGoalTrial();
  pose.Compute({},{},{},640,480,0,true,camera,{});
  auto near_hits=misses;
  for(int i=0;i<c.near_distance_min_hits;++i) near_hits[i]=true;
  pose.UseVerifiedObservation(Target(c.target_u_px),{},hits,misses,640,480,near_hits);
  g=pose.Compute({},Target(c.target_u_px),{},640,480,1.5,true,camera,
      {false,false,true},true,.5);
  assert(g.mode==GoalMode::kWaitFinePose);
  g=pose.Compute({},Target(c.target_u_px),{},640,480,1.6,true,camera,{});
  assert(g.mode==GoalMode::kWaitFinePose && g.action.action==MissionAction::kDefaultPoseMode);
}

void GoalEntryCameraTransport(bool left, bool manual) {
  auto c=LoadDefaultAlgorithmConfig();c.initial_has_ball=true;c.goal_trial_start_goal=false;
  c.goal.entry_line_retry_max_attempts=0; // This case tests the existing directional turn transport.
  c.enable_ball=c.enable_hurdle=false;c.enable_goal=true;
  c.goal.post_pickup_walk_fallback_actions=1;c.transition_settle_sec=.5;
  c.goal.stable_window=10;c.goal.stable_min_hits=7;c.goal.edge_margin_px=50;
  MissionController core(c);
  auto f=Frame(0);f.advance_line_fsm=true;f.manual_observation_control=true;
  for(double v:{450.,400.,350.,300.})f.line_centers.push_back({c.line_features.image_center_u,v});
  auto r=core.Step(f);const auto id=r.command.action_id;
  assert(r.command.action==MissionAction::kLineStraightUnit);
  for(int i=1;i<=10;++i) {
    f.now_sec=.1*i;f.backboard_target=Target(c.goal.target_u_px+(left?-140:140));
    f.goal_pose={true,0,1.5,0,1};f.backboard_center_depth_m=1.5;
    f.delivery_feedback={id,true,false,false};r=core.Step(f);
    assert(r.active_mission==MissionType::kLine);
  }
  f.now_sec=1.1;f.backboard_target.reset();f.goal_pose={};f.backboard_center_depth_m.reset();
  f.delivery_feedback={id,true,true,false};r=core.Step(f);
  assert(r.active_mission==MissionType::kGoal && r.command.camera_request==CameraRequest::kGoal);
  f.now_sec=1.2;f.delivery_feedback={};f.camera_feedback={CameraMode::kTransition,false};core.Step(f);
  f.now_sec=1.3;f.camera_feedback={CameraMode::kGoal,true};
  f.manual_observation_control=manual;f.allow_new_body_actions=!manual;r=core.Step(f);
  assert(r.goal.mode==GoalMode::kSearch);
  for(int i=1;i<=30;++i) {
    f.now_sec=1.3+.05*i;f.begin_manual_observation=manual && i==1;
    r=core.Step(f);
    if(i<30) assert(r.command.action_id==0 && r.goal.mode==GoalMode::kSearch);
  }
  assert(r.goal.mode==GoalMode::kRecoveryTurn && r.command.action_id==0);
  f.now_sec=2.85;f.begin_manual_observation=false;f.allow_new_body_actions=true;r=core.Step(f);
  assert(r.command.action==(left?MissionAction::kTurnLeftInPlace:MissionAction::kStepForwardRightOneHurdle));
  const auto turn=r.command.action_id;
  f.now_sec=2.9;f.delivery_feedback={id,true,true,false};r=core.Step(f);
  assert(r.command.action_id==turn); // Late LINE DONE cannot complete a turn.
}

void GoalEntryForwardCameraTrigger() {
  auto c=LoadDefaultAlgorithmConfig().goal;c.stable_window=10;c.stable_min_hits=7;
  c.entry_line_retry_max_attempts=0; // Isolate the FORWARD-depth trigger and legacy recovery.
  c.camera_goal_start_z_m=.8;c.edge_margin_px=50;c.recovery_turn_max=3;
  GoalController goal(c,3);goal.StartForwardTrial();
  const CameraFeedback forward{CameraMode::kForward,true}, camera{CameraMode::kGoal,true};
  const auto board=Target(c.target_u_px-140);GoalResult g;
  for(int i=0;i<10;++i)
    g=goal.Compute({},board,{true,0,1.5,0,1},640,480,.05*i,true,forward,{},false,1.5);
  assert(g.mode==GoalMode::kForwardApproach);
  g=goal.Compute({},board,{true,0,.7,0,1},640,480,.6,true,forward,{false,true,false},false,.7);
  assert(g.mode==GoalMode::kForwardApproach && g.camera_request==CameraRequest::kNone);
  g=goal.Compute({},{},{},640,480,.7,true,forward,{true,false,true},false);
  assert(g.mode==GoalMode::kWaitCameraGoal && g.camera_request==CameraRequest::kGoal);
  goal.Compute({},{},{},640,480,.8,true,camera,{});
  goal.UseVerifiedObservation({}, {},std::deque<bool>(10,false),std::deque<bool>(10,false),640,480);
  g=goal.Compute({},{},{},640,480,2.3,true,camera,{false,false,true});
  assert(g.mode==GoalMode::kRecoveryTurn);
  g=goal.Compute({},{},{},640,480,2.35,true,camera,{});
  assert(g.action.action==MissionAction::kTurnLeftInPlace);
}

void Defaults() {
  auto c=LoadDefaultAlgorithmConfig();
  assert(CruiseAction(CruiseDirection::kStraight)==MissionAction::kStepForwardFour);
  assert(c.transition_settle_sec==.6 && c.fine_motion_settle_sec==0 && c.walk_inter_action_hold_sec==0);

  assert(c.ball.recovery_walk_turn_max==3 && c.ball.entry_recovery_max_steps==3 && c.goal.recovery_turn_max==3);
  for(int field=0;field<4;++field) {
    auto bad=c;
    if(field==0)bad.transition_settle_sec=-1;
    if(field==1)bad.fine_motion_settle_sec=std::numeric_limits<double>::quiet_NaN();
    if(field==2)bad.walk_inter_action_hold_sec=std::numeric_limits<double>::infinity();
    if(field==3)bad.goal.recovery_turn_max=-1;
    bool rejected=false;try{ValidateAlgorithmConfig(bad);}catch(...){rejected=true;}
    assert(rejected);
  }
}
}
int main() {
  GoalEntryForwardCameraTrigger();
  GoalEntryFallbackAndDepth();
  for(bool left:{false,true}) {
    for(int prior_hits:{1,10})GoalEntryFirstObservation(left,false,prior_hits);
    GoalEntryFirstObservation(left,true,10);
    for(bool manual:{false,true})GoalEntryCameraTransport(left,manual);
  }
  for(bool left:{false,true}) {
    FineSidePairDirect(left);EntryDirectionRecovery(left,false);EntryDirectionRecovery(left,true);
    EntryDirectionTransport(left);
    for(bool manual:{false,true})FineSidePairTransport(left,manual);
  }
  EntryDirectionCentralFallback();
  BallEntryCountsOnlyDetections();BallEntryAndReadyQueue(false);BallEntryAndReadyQueue(true);
  BallEntryUsesCandidateIdentity();
  BallEntryRawFallbackIsExplicit();
  Defaults();VerifiedFineWindows();InvalidAndNearWindows();GoalEdges();ImageDetectionWithMissingDepth();
  DirectGoalStartAndReset();
  for(bool left:{false,true})GoalPairHasOneObservation(left);
  for(bool fine:{false,true})for(bool left:{false,true})GoalRecovery(fine,left);
  PoseSettleAndConcurrentObservation();
  FineObservationTiming(0);FineObservationTiming(.25);FineObservationTiming(.25,true);
  FineObservationTiming(.25,false,2.0);
  FineObservationTiming(0,false,1.0,true);
  RecoveryWithoutLastDetectionDeadline();
  for(bool left:{false,true})FineTurnObservationTiming(left);
}
