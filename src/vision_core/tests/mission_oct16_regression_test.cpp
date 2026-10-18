#include "vision_core/config_loader.hpp"
#include "verified_entry_fixture.hpp"
#include "vision_core/object_target_extractor.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <limits>
using namespace vision_core;
namespace {
ObjectTarget Target(double u,double v) {
  ObjectTarget t;t.center_px={u,v};t.box_px={u-10,v-10,20,20};
  t.width_px=t.height_px=20;t.area_px=400;t.confidence=1;return t;
}
void BallMiddleLoss(int zone) {
  auto c=LoadDefaultAlgorithmConfig().ball;
  c.stable_window=c.stable_min_hits=c.entry_min_hits=c.down_search_min_hits=1;
  c.far_u_des_px=380;c.approach_u_deadband_px=90;
  c.approach_short_v_px=100;c.approach_two_step_v_px=240;
  const double u=zone==4?200:zone==6?550:380;
  BallController ball(c,3);
  auto r=ball.Compute(Target(u,150),640,480,0,{},{ });
  const auto original=zone==4?MissionAction::kStepForwardLeftOne:
      zone==6?MissionAction::kStepForwardRightOne:MissionAction::kStepForwardThree;
  assert(r.action.action==original);
  ActionExecutionFeedback active;active.action_active=true;
  r=ball.Compute({},640,480,.1,{},active);
  assert(r.action.action==MissionAction::kNone && r.camera_request==CameraRequest::kNone);
  ActionExecutionFeedback done;done.action_done=true;
  r=ball.Compute({},640,480,.2,{},done);
  assert(r.mode==BallMode::kApproachBall && r.camera_request==CameraRequest::kNone);
  ActionExecutionFeedback observed;observed.observation_complete=true;
  r=ball.Compute({},640,480,1.2,{},observed,false);
  if(zone==5) {
    assert(r.mode==BallMode::kWaitCameraDown && r.camera_request==CameraRequest::kDown);
    assert(r.action.action==MissionAction::kNone);
  } else {
    const auto turn=zone==4?MissionAction::kTurnLeftInPlace:MissionAction::kStepForwardRightOneHurdle;
    assert(r.mode==BallMode::kMidTurnBeforeRecovery && r.action.action==turn);
    // An uncommitted preview cannot consume a DONE or execute its turn.
    r=ball.Compute({},640,480,1.3,{},done,false);
    assert(r.action.action==turn);
    r=ball.Compute({},640,480,1.4,{},{});
    assert(r.action.action==turn);
    r=ball.Compute({},640,480,1.5,{},active);
    assert(r.action.action==MissionAction::kNone);
    r=ball.Compute({},640,480,1.6,{},done);
    assert(r.mode==BallMode::kBallRecoveryForward && r.camera_request==CameraRequest::kNone);
    assert(ball.RecoveryWalkTurnsCompleted()==1);
    r=ball.Compute({},640,480,1.7,{},{});
    assert(r.mode==BallMode::kBallRecoveryForward && r.action.action==MissionAction::kNone);
    // Only two further DONE turns remain in the total budget of three.
    r=ball.Compute({},640,480,2.7,{},{});
    assert(r.mode==BallMode::kBallRecoveryWalkTurn);
    r=ball.Compute({},640,480,2.8,{},{});
    assert(r.action.action==turn);
    done.observation_complete=true;
    r=ball.Compute({},640,480,3.9,{},done);
    assert(ball.RecoveryWalkTurnsCompleted()==2 && r.action.action==MissionAction::kNone);
    // Completion updates the count; the next decision issues the third turn.
    r=ball.Compute({},640,480,4.0,{},{});
    assert(ball.RecoveryWalkTurnsCompleted()==2 && r.action.action==turn);
    r=ball.Compute({},640,480,5.0,{},done);
    assert(ball.RecoveryWalkTurnsCompleted()==3);
    assert(r.mode==BallMode::kBallRecoveryForward && r.action.action==MissionAction::kNone);
    r=ball.Compute(Target(380,50),640,480,6.1,{},{});
    assert(r.mode==BallMode::kApproachBall && ball.RecoveryWalkTurnsCompleted()==0);
  }
  ball.Reset();
  r=ball.Compute(Target(380,50),640,480,10,{},{});
  assert(r.mode==BallMode::kApproachBall && r.action.action==MissionAction::kStepForwardThree);
}
void BallZoneEightOneStepThenDown() {
  auto c=LoadDefaultAlgorithmConfig().ball;
  c.stable_window=c.stable_min_hits=c.entry_min_hits=c.down_search_min_hits=1;
  c.far_u_des_px=380;c.approach_u_deadband_px=90;
  c.approach_short_v_px=100;c.approach_two_step_v_px=240;
  c.approach_direct_down_v_px=355;
  // Inclusive column edges remain zone 8; its lower part uses one final step.
  for(double u:{290.,380.,470.}) for(double v:{240.,354.99,355.,420.}) {
    BallController ball(c,3);
    const auto r=ball.Compute(Target(u,v),640,480,0,{},{});
    const auto action=v>=355?MissionAction::kStepForwardOne:MissionAction::kStepForwardTwo;
    assert(r.mode==BallMode::kApproachTwoBeforeDown);
    assert(r.camera_request==CameraRequest::kNone && r.action.action==action);
    ActionExecutionFeedback busy;busy.action_active=true;
    auto moving=ball.Compute({},640,480,.1,{},busy);
    assert(moving.camera_request==CameraRequest::kNone && moving.action.action==action);
    ActionExecutionFeedback done;done.action_done=true;
    auto completed=ball.Compute({},640,480,.2,{},done);
    assert(completed.mode==BallMode::kWaitCameraDown);
    assert(completed.camera_request==CameraRequest::kDown && completed.action.action==MissionAction::kNone);
  }
  for(double u:{289.,471.}) {
    BallController ball(c,3);
    const auto r=ball.Compute(Target(u,420),640,480,0,{},{});
    assert(r.camera_request==CameraRequest::kNone);
    assert(r.action.action==(u<380?MissionAction::kTurnLeftInPlace:
        MissionAction::kStepForwardRightOneHurdle));
  }
  BallController preview(c,3);
  auto r=preview.Compute(Target(380,355),640,480,0,{}, {},false);
  assert(r.mode==BallMode::kApproachTwoBeforeDown);
  assert(r.action.action==MissionAction::kStepForwardOne && r.camera_request==CameraRequest::kNone);
  ActionExecutionFeedback unissued_done;unissued_done.action_done=true;
  r=preview.Compute({},640,480,.1,{},unissued_done,false);
  assert(r.mode==BallMode::kApproachTwoBeforeDown && r.camera_request==CameraRequest::kNone);
  r=preview.Compute({},640,480,.2,{},{}); // Commit the one-step action.
  assert(r.action.action==MissionAction::kStepForwardOne && r.camera_request==CameraRequest::kNone);
  auto changed=c;changed.approach_direct_down_v_px=450;preview.UpdateTuning(changed);
  ActionExecutionFeedback active;active.action_active=true;
  r=preview.Compute(Target(200,250),640,480,.3,{},active);
  assert(r.action.action==MissionAction::kStepForwardOne && r.camera_request==CameraRequest::kNone);
  r=preview.Compute({},640,480,.4,{},unissued_done);
  assert(r.mode==BallMode::kWaitCameraDown && r.camera_request==CameraRequest::kDown);
  const CameraFeedback down{CameraMode::kDown,true};
  r=preview.Compute({},640,480,.5,down,{});
  assert(r.mode==BallMode::kDownSearch && r.action.action==MissionAction::kNone);
  r=preview.Compute({},640,480,.6,down,{});
  assert(r.action.action==MissionAction::kStepForwardOne);

  // Retuning cannot interrupt an already issued final two-step motion.
  c.approach_direct_down_v_px=410;
  BallController pending(c,3);
  r=pending.Compute(Target(380,400),640,480,0,{},{});
  assert(r.action.action==MissionAction::kStepForwardTwo);
  c.approach_direct_down_v_px=355;pending.UpdateTuning(c);
  ActionExecutionFeedback busy;busy.action_active=true;
  r=pending.Compute(Target(380,420),640,480,.1,{},busy);
  assert(r.camera_request==CameraRequest::kNone);
  ActionExecutionFeedback done;done.action_done=true;
  r=pending.Compute({},640,480,.2,{},done);
  assert(r.camera_request==CameraRequest::kDown);
  pending.Reset();
  r=pending.Compute(Target(380,400),640,480,1,{},{});
  assert(r.camera_request==CameraRequest::kNone && r.action.action==MissionAction::kStepForwardOne);

  // Live and tuning setters must both update the actual mission controller.
  for(bool live:{false,true}) {
    auto config=LoadDefaultAlgorithmConfig();config.enable_goal=config.enable_hurdle=false;
    config.transition_settle_sec=0;config.ball=c;config.ball.approach_direct_down_v_px=410;
    MissionController core(config);
    auto changed=config;changed.ball.approach_direct_down_v_px=355;
    if(live) core.SetLivePerceptionTuning(changed);
    else assert(core.UpdateTuning(changed));
    MissionFrameInput f;f.image_width=640;f.image_height=480;f.command_transport_enabled=true;
    f.ball_target=Target(380,400);
    AdmitObjectAtRest(core, *f.ball_target);
    f.now_sec=3;
    auto output=core.Step(f);
    assert(output.command.camera_request==CameraRequest::kNone);
    assert(output.command.action==MissionAction::kStepForwardOne);
    const auto id=output.command.action_id;assert(id!=0);
    f.now_sec=3.1;f.delivery_feedback={id,true,false,false};f.ball_target.reset();
    output=core.Step(f);assert(output.command.camera_request==CameraRequest::kNone);
    f.now_sec=3.2;f.delivery_feedback={id,true,true,false};
    output=core.Step(f);
    assert(!output.post_motion_observing && output.command.camera_request==CameraRequest::kDown);
    assert(output.command.action==MissionAction::kNone);
  }
}
void BallTurnDisabled() {
  auto c=LoadDefaultAlgorithmConfig().ball;
  c.stable_window=c.stable_min_hits=c.entry_min_hits=c.down_search_min_hits=1;
  c.far_u_des_px=380;c.approach_u_deadband_px=90;
  c.approach_short_v_px=100;c.approach_two_step_v_px=240;
  c.recovery_walk_turn_max=0;c.entry_recovery_walk_turn_max=0;
  BallController ball(c,3);
  ball.Compute(Target(200,150),640,480,0,{},{});
  ActionExecutionFeedback done;done.action_done=done.observation_complete=true;
  const auto r=ball.Compute({},640,480,1.2,{},done);
  assert(r.mode==BallMode::kBallRecoveryForward && r.action.action==MissionAction::kNone);
  assert(ball.RecoveryWalkTurnsCompleted()==0);
}
void SearchOnlyFiveHits() {
  auto c=LoadDefaultAlgorithmConfig();
  c.ball.stable_window=c.hurdle.stable_window=10;
  c.ball.stable_min_hits=c.hurdle.stable_min_hits=7;
  c.ball.down_search_min_hits=c.hurdle.down_search_min_hits=5;
  BallController ball(c.ball,3);HurdleController hurdle(c.hurdle,3);
  ball.StartTestNear();hurdle.StartTestNear();
  const CameraFeedback down{CameraMode::kDown,true};
  ball.Compute({},640,480,.8,down,{});
  hurdle.Compute({},640,480,.8,down,{},{});
  auto b=ball.Compute({},640,480,.81,down,{});
  auto h=hurdle.Compute({},640,480,.81,down,{},{});
  assert(b.action.action==MissionAction::kStepForwardOne && h.action.action==MissionAction::kStepForwardOne);
  ActionExecutionFeedback active;active.action_active=true;
  const bool hits[]={true,true,false,true,false,true,true};
  for(int i=0;i<7;++i) {
    const auto t=hits[i]?std::optional<ObjectTarget>(Target(320,200)):std::nullopt;
    // The rolling window crosses 1.0 s; no periodic reset is permitted.
    b=ball.Compute(t,640,480,.9+.05*i,down,active);
    h=hurdle.Compute(t,640,480,.9+.05*i,down,active,{});
    assert(b.mode==BallMode::kDownSearch && h.mode==HurdleMode::kDownSearch);
    assert(b.action.action==MissionAction::kNone && h.action.action==MissionAction::kNone);
  }
  assert(!b.tracked.stable && !h.tracked.stable); // still only five, not seven
  ActionExecutionFeedback done;done.action_done=true;
  b=ball.Compute({},640,480,1.3,down,done);
  h=hurdle.Compute({},640,480,1.3,down,done,{});
  assert(b.mode==BallMode::kCoarseAdjust && h.mode==HurdleMode::kCoarseAdjust);
  assert(b.action.action==MissionAction::kNone && h.action.action==MissionAction::kNone);
  // The relaxed search threshold must never authorize FINE with five hits.
  ball.StartTestExecute();hurdle.StartTestExecute();
  const std::deque<bool> five{true,true,true,true,true,false,false,false,false,false};
  ball.UseVerifiedObservation(Target(345,100),five,640,480);
  hurdle.UseVerifiedObservation(Target(320,270),five,640,480);
  b=ball.Compute({},640,480,2,down,done);
  h=hurdle.Compute({},640,480,2,down,done,{});
  assert(b.action.action==MissionAction::kNone && h.action.action==MissionAction::kNone);
}
void CountedDownSearch(bool found_on_last) {
  auto c=LoadDefaultAlgorithmConfig();
  c.ball.down_search_max_steps=c.hurdle.down_search_max_steps=3;
  c.ball.down_search_min_hits=c.ball.stable_min_hits=c.ball.stable_window=1;
  c.hurdle.down_search_min_hits=c.hurdle.stable_min_hits=c.hurdle.stable_window=1;
  BallController ball(c.ball,3);HurdleController hurdle(c.hurdle,3);
  ball.StartTestNear();hurdle.StartTestNear();
  const CameraFeedback down{CameraMode::kDown,true};
  ball.Compute({},640,480,0,down,{});
  hurdle.Compute({},640,480,0,down,{},{});
  auto b=ball.Compute({},640,480,1,down,{});
  auto h=hurdle.Compute({},640,480,1,down,{},{});
  assert(b.action.action==MissionAction::kStepForwardOne && h.action.action==b.action.action);
  for(int i=1;i<=3;++i) {
    ActionExecutionFeedback active;active.action_active=true;
    b=ball.Compute({},640,480,i*100,down,active);
    h=hurdle.Compute({},640,480,i*100,down,active,{});
    assert(b.mode==BallMode::kDownSearch && h.mode==HurdleMode::kDownSearch);
    assert(ball.DownSearchStepsRemaining()==4-i && hurdle.DownSearchStepsRemaining()==4-i);
    ActionExecutionFeedback done;done.action_done=true;
    const auto target=found_on_last && i==3?std::optional<ObjectTarget>(Target(320,200)):std::nullopt;
    b=ball.Compute(target,640,480,i*100+1,down,done);
    h=hurdle.Compute(target,640,480,i*100+1,down,done,{});
    assert(ball.DownSearchStepsRemaining()==3-i && hurdle.DownSearchStepsRemaining()==3-i);
    if(i<3) {
      assert(b.action.action==MissionAction::kStepForwardOne && h.action.action==b.action.action);
    } else {
      assert(b.mode==(found_on_last?BallMode::kCoarseAdjust:BallMode::kFailed));
      assert(h.mode==(found_on_last?HurdleMode::kCoarseAdjust:HurdleMode::kFailed));
      assert(b.action.action==MissionAction::kNone && h.action.action==MissionAction::kNone);
    }
  }
  ball.Reset();hurdle.Reset();
  assert(ball.DownSearchStepsRemaining()==3 && hurdle.DownSearchStepsRemaining()==3);
}
void SharedSearchCounter() {
  DownSearchWalk search;
  assert(search.Observe(false,0,5,1,false,false,0,0)==DownSearchWalk::Decision::kStep);
  search.Issued();
  assert(search.Observe(true,5,5,1,true,false,1,0)==DownSearchWalk::Decision::kMoving);
  assert(search.Remaining(1)==1);
  assert(search.Observe(false,0,5,1,false,true,2,0)==DownSearchWalk::Decision::kFound);
  assert(search.Remaining(1)==0);
  search.Observe(false,0,5,1,false,true,3,0); // no pending action: duplicate cannot count
  assert(search.Remaining(1)==0 && search.Remaining(2)==1);
}
void ObjectFilters() {
  auto c=LoadDefaultAlgorithmConfig();
  auto& o=c.object_targets;
  o.ball_min_box_width=30;o.ball_min_box_height=10;
  o.goal_min_box_width=50;o.goal_min_box_height=12;
  o.backboard_min_box_width=70;o.backboard_min_box_height=14;
  o.hurdle_min_box_width=90;o.hurdle_min_box_height=16;
  ValidateAlgorithmConfig(c);
  for(const auto id:{o.ball_class_id,o.goal_class_id,o.backboard_class_id,o.hurdle_class_id}) {
    const auto size=o.MinimumBoxSize(id);
    std::vector<Detection> detections={{{10,10,size.first-1,size.second},1,id},
      {{10,10,size.first,size.second-1},1,id},{{10,10,size.first,size.second},1,id}};
    const auto candidates=ExtractObjectTargetCandidates(detections,id,.6,o);
    assert(candidates.size()==1 && candidates.front().detection_index==2);
    const auto selected=ExtractObjectTargetSelection(detections,o);
    const auto index=id==o.ball_class_id?selected.indices.ball:
        id==o.goal_class_id?selected.indices.goal:
        id==o.backboard_class_id?selected.indices.backboard:selected.indices.hurdle;
    assert(index && *index==2);
  }
  // Retuning the core must change the real perception pipeline, not just ROS values.
  c.enable_ball=c.enable_goal=c.enable_hurdle=false;
  MissionController live(c);
  PerceptionFrameInput f;f.image_width=640;f.image_height=480;
  f.allow_new_body_actions=f.allow_new_commands=false;
  f.detections.push_back({{{10,10,40,20},1,o.ball_class_id}});
  assert(live.StepPerception(f).perception.targets.ball);
  auto edited=c;edited.object_targets.ball_min_box_width=41;
  live.SetLivePerceptionTuning(edited);
  f.now_sec=.1;
  assert(!live.StepPerception(f).perception.targets.ball);
  edited.object_targets.ball_min_box_width=30;
  assert(live.UpdateTuning(edited));
  f.now_sec=.2;
  assert(live.StepPerception(f).perception.targets.ball);
  edited.object_targets.ball_min_box_height=-1;
  bool rejected=false;
  try {live.SetLivePerceptionTuning(edited);}catch(...){rejected=true;}
  assert(rejected);
}
void LineBoxFilters() {
  auto c=LoadDefaultAlgorithmConfig();
  c.enable_ball=c.enable_goal=c.enable_hurdle=false;
  c.line_detection.min_box_width=c.line_detection.min_box_height=1;
  MissionController core(c);
  PerceptionFrameInput f;f.image_width=640;f.image_height=480;
  f.allow_new_commands=f.allow_new_body_actions=false;
  for(double y:{100.,200.,300.})
    f.detections.push_back({{{310,y,2,10},1,c.line_detection.class_id}});
  assert(core.StepPerception(f).perception.raw_line_centers.size()==3);
  auto edited=c;edited.line_detection.min_box_width=3;
  core.SetLivePerceptionTuning(edited);
  f.now_sec=.1;
  assert(core.StepPerception(f).perception.raw_line_centers.empty());
  edited.line_detection.min_box_width=2;
  assert(core.UpdateTuning(edited));
  f.now_sec=.2;
  // LINE rejects sizes equal to either threshold as well as smaller boxes.
  assert(core.StepPerception(f).perception.raw_line_centers.empty());
  edited.line_detection.min_box_width=1;
  assert(core.UpdateTuning(edited));
  f.now_sec=.3;
  assert(core.StepPerception(f).perception.raw_line_centers.size()==3);
  edited.line_detection.min_box_height=10;
  core.SetLivePerceptionTuning(edited);
  f.now_sec=.4;
  assert(core.StepPerception(f).perception.raw_line_centers.empty());
  edited.line_detection.min_box_height=9;
  assert(core.UpdateTuning(edited));
  f.now_sec=.5;
  assert(core.StepPerception(f).perception.raw_line_centers.size()==3);
}
MissionControllerConfig LineConfig() {
  auto c=LoadDefaultAlgorithmConfig();
  c.enable_ball=c.enable_hurdle=c.enable_goal=false;
  c.line_features.image_center_u=320;c.line_p2p.offset_gain=1;c.line_p2p.heading_gain=0;
  c.line_p2p.steering_deadband=.1;c.line_p2p.in_place_turn_score=.5;
  c.line_p2p.failure_min_valid_samples=1;c.line_p2p.recovery_max_turns=2;
  return c;
}
MissionFrameInput LineFrame(double now,double score) {
  MissionFrameInput f;f.image_width=640;f.image_height=480;f.now_sec=now;
  f.command_transport_enabled=true;
  const double u=320+320*score;
  f.line_centers={{u,100},{u,200},{u,400}};
  LineGuide guide;guide.valid=true;guide.offset=score;guide.confidence=1;
  f.line_decision_guide_override=guide;
  return f;
}
void InvalidNewSettings() {
  for(int i=0;i<7;++i) {
    auto c=LineConfig();
    if(i==0)c.ball.down_search_min_hits=0;
    if(i==1)c.hurdle.down_search_min_hits=c.hurdle.stable_window+1;
    if(i==2)c.line_p2p.in_place_turn_score=c.line_p2p.steering_deadband;
    if(i==3)c.line_p2p.in_place_turn_score=std::numeric_limits<double>::infinity();
    if(i==4)c.ball.down_search_max_steps=0;
    if(i==5)c.hurdle.down_search_max_steps=-1;
    if(i==6)c.ball.entry_recovery_walk_turn_max=-1;
    bool rejected=false;
    try {ValidateAlgorithmConfig(c);}catch(...){rejected=true;}
    assert(rejected);
  }
}
void LargeLineScore() {
  for(double score:{-.6,-.5,-.1,0.,.1,.5,.6}) {
    auto c=LineConfig();MissionController line(c);
    auto r=line.Step(LineFrame(0,score));
    const auto expected=score<-.5?MissionAction::kTurnLeftInPlace:
        score>.5?MissionAction::kStepForwardRightOneHurdle:
        score<-.1?MissionAction::kLineLeftUnit:
        score>.1?MissionAction::kLineRightUnit:MissionAction::kLineStraightUnit;
    assert(r.command.action==expected);
    assert(r.line_score_recovery==(std::abs(score)>.5));
  }
  auto c=LineConfig();MissionController line(c);
  auto r=line.Step(LineFrame(0,.6));
  auto id=r.command.action_id;
  double now=0;
  for(int turn=0;turn<2;++turn) {
    now+=.2;
    auto f=LineFrame(now,.6);f.delivery_feedback={id,true,true,false};
    r=line.Step(f);
    assert(r.post_motion_observing && r.line_in_recovery && r.line_score_recovery);
    assert(r.line_score_recovery_turns_completed==turn+1);
    for(int i=1;i<=11;++i)r=line.Step(LineFrame(now+.1*i,.6));
    if(turn==0) {
      assert(r.command.action==MissionAction::kStepForwardRightOneHurdle && r.command.action_id!=id);
      id=r.command.action_id;
    }
    now+=1.1;
  }
  assert(r.terminal_hold && r.line_score_recovery && r.line_score_recovery_turns_completed==2);
  line.Reset();
  assert(line.Step(LineFrame(0,.6)).command.action==MissionAction::kStepForwardRightOneHurdle);
}
void ReadyKeepsOneSlot() {
  auto c=LineConfig();MissionController line(c);
  auto r=line.Step(LineFrame(0,0));const auto a=r.command.action_id;
  for(int i=1;i<20;++i) {
    auto f=LineFrame(i*.1,.6);f.delivery_feedback={a,true,false,false};r=line.Step(f);
  }
  auto f=LineFrame(2,.6);f.delivery_feedback={a,true,false,true};
  r=line.Step(f);const auto b=r.command.action_id;
  assert(b!=a && r.command.action==MissionAction::kStepForwardRightOneHurdle);
  assert(r.command.control_phase==ControlPhase::kWaitingQueuedActionAck);
  f=LineFrame(2.05,-.6);f.delivery_feedback={b,true,false,false};r=line.Step(f);
  assert(r.command.action_id==b && r.command.control_phase==ControlPhase::kWaitingQueuedActionStart);
  f=LineFrame(2.25,-.6);f.delivery_feedback={a,true,true,false};r=line.Step(f);
  assert(r.command.action_id==b && r.command.action==MissionAction::kStepForwardRightOneHurdle);
  f=LineFrame(2.5,0);f.delivery_feedback={b,true,true,false};r=line.Step(f);
  assert(r.post_motion_observing);
  for(int i=1;i<=11;++i)r=line.Step(LineFrame(2.5+.1*i,0));
  assert(r.command.action==MissionAction::kLineStraightUnit && !r.post_motion_observing);
}
}
int main() {
  for(int zone:{4,5,6})BallMiddleLoss(zone);
  BallZoneEightOneStepThenDown();
  CountedDownSearch(false);CountedDownSearch(true);SharedSearchCounter();
  BallTurnDisabled();SearchOnlyFiveHits();ObjectFilters();LineBoxFilters();InvalidNewSettings();LargeLineScore();ReadyKeepsOneSlot();
}
