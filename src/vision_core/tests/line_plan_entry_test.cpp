#include "vision_core/config_loader.hpp"
#include "vision_core/mission_controller.hpp"
#include <algorithm>
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <limits>
#include <set>
using namespace vision_core;

namespace {
ObjectTarget Target(double u = 150) {
  ObjectTarget t; t.center_px = {u, 100}; t.box_px = {u-10,90,20,20};
  t.width_px = t.height_px = 20; t.area_px = 400; t.confidence = 1;
  return t;
}
MissionControllerConfig Configuration() {
  auto c = LoadDefaultAlgorithmConfig();
  c.enable_ball = c.enable_hurdle = c.enable_goal = false;
  c.initial_has_ball = c.goal_trial_start_goal = false;
  c.line_features.image_center_u = 320;
  c.line_p2p.offset_gain = 1; c.line_p2p.heading_gain = 0;
  c.line_p2p.in_place_turn_score = 1;
  c.line_p2p.left_repetitions = c.line_p2p.right_repetitions = 3;
  c.line_p2p.straight_repetitions = 4;
  c.ball.entry_min_hits = 6; c.ball.stable_window = 10; c.ball.stable_min_hits = 7;
  c.hurdle.stable_window = 10; c.hurdle.stable_min_hits = 7;
  c.post_motion_observation_sec = 1;
  c.candidate_search_max_turns = 2;
  c.transition_settle_sec = 0;
  return c;
}
struct Driver {
  MissionController core;
  MissionFrameInput frame;
  MissionFrameResult out;
  explicit Driver(MissionControllerConfig c) : core(c) {
    frame.image_width = 640; frame.image_height = 480;
    frame.camera_feedback = {CameraMode::kForward, true};
    frame.command_transport_enabled = true;
    // Stationary/manual admission contracts; full AUTO is covered by continuous_object_approach_test.
    frame.manual_observation_control = true;
    Line(320);
  }
  void Line(double u) { frame.line_centers = {{u,450},{u,400},{u,350},{u,300},{u,250}}; }
  void Tick(double dt = .05, CommandDeliveryFeedback feedback = {}) {
    frame.now_sec += dt; frame.delivery_feedback = feedback;
    out = core.Step(frame);
  }
  void Window(bool found, bool hurdle = false) {
    for (int i = 0; i < 12; ++i) {
      const auto t = found ? std::optional<ObjectTarget>(Target()) : std::nullopt;
      if (hurdle) frame.hurdle_target = t; else frame.ball_target = t;
      Tick(.10);
    }
  }
};

void NormalBlocks(bool reduced, double start_ratio) {
  for (const auto direction : {-1,0,1}) {
    auto c = Configuration();
    c.line_p2p.observation_start_ratio = start_ratio;
    if (reduced) {
      c.line_p2p.left_repetitions = c.line_p2p.right_repetitions = 2;
      c.line_p2p.straight_repetitions = 3;
    }
    Driver d(c);
    d.Line(320 + direction*64);
    d.Tick();
    const auto unit = direction < 0 ? MissionAction::kLineLeftUnit :
        direction > 0 ? MissionAction::kLineRightUnit : MissionAction::kLineStraightUnit;
    const int repeats = direction == 0 ? (reduced ? 3 : 4) : (reduced ? 2 : 3);
    std::set<std::uint64_t> issued;
    auto current = d.out.command.action_id;
    assert(d.out.command.action == unit && current);
    issued.insert(current);
    d.Tick(.01, {current,true,false,false});
    const double started = d.frame.now_sec;
    for (int n = 0; n < repeats; ++n) {
      // Later guides ask for the opposite direction; intermediate READY must
      // still reserve the ORIGINAL unit. Direction is reconsidered at block end.
      if (n) d.Line(320 - direction*64);
      for (int i=0; i<12; ++i) d.Tick(.04);
      d.Tick(.01, {current,true,false,true});
      const auto next = d.out.command.action_id;
      assert(next && next != current);
      assert(d.out.active_mission == MissionType::kLine &&
             d.out.command.mission == MissionType::kLine);
      assert(!d.out.post_motion_observing);
      if (n + 1 < repeats) {
        assert(d.out.command.action == unit);
        assert(!d.out.line_window_stats); // No per-unit five-sample gate.
        issued.insert(next);
      } else {
        assert(d.out.line_window_stats && d.out.line_window_stats->used_frames >= 5);
        const auto& stats = *d.out.line_window_stats;
        assert(std::abs(stats.remaining_sec - .25) < 1e-9);
        assert(std::abs(stats.window_end_sec - d.frame.now_sec) < 1e-9);
        assert(std::abs(stats.window_start_sec -
            (started + start_ratio * (d.frame.now_sec - started + .25))) < 1e-9);
      }
      d.Tick(.01, {next,true,false,false});
      d.Tick(.02, {current,true,true,false});
      current = next;
    }
    assert(issued.size() == static_cast<std::size_t>(repeats));
  }
}

void CandidateWinsReady(bool queued, bool early_transport) {
  auto c = Configuration(); c.enable_ball = true;
  Driver d(c); d.Tick();
  const auto a = d.out.command.action_id;
  d.Tick(.01, {a,true,false,false});
  std::uint64_t b = 0;
  if (queued) {
    d.Tick(.01, {a,true,false,true});
    b = d.out.command.action_id;
    assert(b != a && b);
    // No queued ACK yet: a candidate may not cancel or replace this ID.
  }
  d.frame.ball_target = Target();
  for (int i=0; i<6; ++i)
    d.Tick(.05, i==5 && !queued ? CommandDeliveryFeedback{a,true,false,true} : CommandDeliveryFeedback{});
  assert(d.out.active_mission == MissionType::kLine);
  assert(d.out.candidate_mission == MissionType::kBall);
  assert(d.out.command.action_id == (queued ? b : a));
  assert(!d.out.post_motion_observing);
  d.frame.ball_target.reset();
  if (early_transport) {
    if (b) d.core.AcceptDeliveryFeedback({b,true,false,false}, d.frame.now_sec);
    d.core.AcceptDeliveryFeedback({a,true,true,false}, d.frame.now_sec+.01);
    if (b) d.core.AcceptDeliveryFeedback({b,true,true,false}, d.frame.now_sec+.02);
    d.Tick(.03, {a,true,true,false});
    assert(d.out.command.command_type != CommandType::kAction);
    if (b) d.Tick(.01, {b,true,true,false});
  } else {
    d.Tick(.05, {a,true,true,false});
    if (b) {
      assert(!d.out.post_motion_observing);
      assert(d.out.command.action_id == b);
      d.Tick(.01, {b,true,false,false});
      d.Tick(.01, {b,true,false,true});
      assert(d.out.command.action_id == b); // C is forbidden, even at B READY.
      d.Tick(.01, {b,true,true,false});
    }
  }
  assert(d.out.active_mission == MissionType::kLine && d.out.post_motion_observing);
  // Moving six-hit history does not verify the stationary object.
  for (int i=0; i<2; ++i) { d.Tick(.10); assert(d.out.active_mission == MissionType::kLine); }
  d.frame.ball_target = Target();
  d.Window(true);
  assert(d.out.active_mission == MissionType::kBall);
  assert(d.out.candidate_mission == MissionType::kNone);
  d.Tick();
  assert(d.out.command.mission == MissionType::kBall);
}

void SearchAndRestore(bool left, int found_after, bool hurdle = false,
                      double drain_delay = 0) {
  auto c = Configuration(); c.enable_ball = !hurdle; c.enable_hurdle = hurdle;
  Driver d(c); d.Tick(); auto id = d.out.command.action_id;
  d.Tick(.01, {id,true,false,false});
  const auto target = Target(left ? 150 : 550);
  if (hurdle) d.frame.hurdle_target = target; else d.frame.ball_target = target;
  for (int i=0;i<(hurdle?7:6);++i) d.Tick();
  assert(d.out.candidate_mission == (hurdle ? MissionType::kHurdle : MissionType::kBall));
  d.frame.ball_target.reset(); d.frame.hurdle_target.reset();
  if (drain_delay > 0) {
    d.Tick(drain_delay);
    assert(d.out.candidate_phase == 1); // Still draining the issued LINE action.
    assert(d.out.command.action_id == id);
  }
  d.Tick(.01, {id,true,true,false});
  const auto search = left ? MissionAction::kTurnLeftInPlace : MissionAction::kStepForwardRightOneHurdle;
  const auto restore = left ? MissionAction::kStepForwardRightOneHurdle : MissionAction::kTurnLeftInPlace;
  d.Window(false, hurdle);
  for (int n=1;n<=2;++n) {
    assert(d.out.command.action == search);
    id = d.out.command.action_id;
    d.Tick(.01, {id,true,false,false});
    d.Tick(.01, {id,true,true,false});
    // Replayed completion is not another completed rotation.
    d.Tick(.01, {id,true,true,false});
    assert(d.out.candidate_search_turns == n);
    d.Window(found_after == n, hurdle);
    if (found_after == n) {
      assert(d.out.active_mission == (hurdle ? MissionType::kHurdle : MissionType::kBall));
      assert(d.out.candidate_mission == MissionType::kNone);
      return;
    }
  }
  for (int n=1;n<=2;++n) {
    assert(d.out.command.action == restore);
    id = d.out.command.action_id;
    d.Tick(.01, {id,true,false,false});
    d.Tick(.01, {id,true,true,false});
  }
  assert(d.out.active_mission == MissionType::kLine && !d.out.terminal_hold);
  assert(d.out.candidate_mission == MissionType::kNone);
  for (int i=0;i<24;++i) d.Tick(.10);
  assert(LinePlan::IsUnit(d.out.command.action));
}

void GoalBlocks(bool straight, int fallback, int straight_required = 1) {
  auto c = Configuration(); c.enable_goal = true; c.initial_has_ball = true;
  c.goal.post_pickup_straight_blocks_required = straight_required;
  c.goal.post_pickup_walk_fallback_actions = fallback;
  Driver d(c); if (!straight) d.Line(384);
  d.Tick();
  if (fallback == 0) {
    assert(d.out.active_mission == MissionType::kGoal && d.out.command.camera_request == CameraRequest::kGoal);
    return;
  }
  auto current = d.out.command.action_id;
  d.Tick(.01, {current,true,false,false});
  const int plans = straight && straight_required > 0 ? std::min(straight_required, fallback) : fallback;
  const int repeats = straight ? 4 : 3;
  for (int block=0;block<plans;++block) for (int step=0;step<repeats;++step) {
    for (int frame=0;frame<12;++frame) d.Tick(.04);
    d.Tick(.01, {current,true,false,true});
    const bool last = block+1 == plans && step+1 == repeats;
    auto next = d.out.command.action_id;
    if (last) assert(next == current); else {
      assert(next && next != current);
      d.Tick(.01, {next,true,false,false});
    }
    d.Tick(.01, {current,true,true,false});
    assert(d.out.goal_walk_actions_completed == block + (step+1==repeats ? 1 : 0));
    assert(d.out.goal_straight_blocks_completed ==
           (straight ? block + (step+1==repeats ? 1 : 0) : 0));
    if (last) {
      assert(d.out.active_mission == MissionType::kGoal);
      assert(d.out.command.camera_request == CameraRequest::kGoal);
      assert(d.out.goal_five_step_completed ==
             (straight && straight_required > 0 && plans >= straight_required));
    } else assert(d.out.active_mission == MissionType::kLine);
    current = next;
  }
}

void GoalEmptyViewLineRetry(bool straight) {
  auto c=Configuration();c.enable_goal=true;c.initial_has_ball=true;
  c.goal.post_pickup_walk_fallback_actions=0;
  c.goal.entry_line_retry_max_attempts=1;
  Driver d(c);d.Tick();
  assert(d.out.active_mission==MissionType::kGoal);
  assert(d.out.command.camera_request==CameraRequest::kGoal);
  const auto mission_index=d.out.goal_mission_index;
  d.frame.camera_feedback={CameraMode::kTransition,false};d.Tick();
  d.frame.camera_feedback={CameraMode::kGoal,true};d.Tick();
  d.frame.begin_manual_observation=true;d.Tick();
  d.frame.begin_manual_observation=false;
  for(int i=0;i<40 && d.out.goal.mode!=GoalMode::kEntryRetryCameraForward;++i)
    d.Tick(.05);
  assert(d.out.goal.mode==GoalMode::kEntryRetryCameraForward && d.out.has_ball);
  assert(d.out.command.camera_request==CameraRequest::kForward);
  assert(d.out.goal.entry_line_retries==1 && d.out.command.action_id==0);
  d.frame.camera_feedback={CameraMode::kTransition,false};
  d.frame.allow_new_commands=d.frame.allow_new_mission_entry=d.frame.advance_line_fsm=false;
  for(int i=0;i<5;++i) {
    d.Tick();assert(d.out.command.action_id==0);
    assert(d.out.command.camera_request==CameraRequest::kNone);
    assert(d.out.goal.mode==GoalMode::kEntryRetryCameraForward);
  }
  d.frame.allow_new_commands=d.frame.allow_new_mission_entry=d.frame.advance_line_fsm=true;
  d.frame.camera_feedback={CameraMode::kForward,true};d.Tick();
  assert(d.out.active_mission==MissionType::kLine && d.out.has_ball);
  assert(d.out.goal_walk_actions_completed==0 && d.out.goal_mission_index==mission_index);
  // Camera feedback alone cannot re-enter GOAL, and an invalid LINE cannot
  // originate a blind forward walk.
  d.frame.line_centers.clear();
  for(int i=0;i<5;++i) {
    d.Tick();assert(d.out.command.action_id==0);
    assert(d.out.active_mission==MissionType::kLine);
  }
  d.Line(straight?320:384);
  for(int i=0;i<80 && !d.out.command.action_id;++i)d.Tick(.05);
  const auto unit=straight?MissionAction::kLineStraightUnit:MissionAction::kLineRightUnit;
  const int repeats=straight?c.line_p2p.straight_repetitions:c.line_p2p.right_repetitions;
  auto current=d.out.command.action_id;
  assert(current && d.out.command.action==unit);
  d.Tick(.01,{current,true,false,false});
  for(int step=0;step<repeats;++step) {
    for(int i=0;i<12;++i)d.Tick(.04);
    d.Tick(.01,{current,true,false,true});
    const bool last=step+1==repeats;
    const auto next=d.out.command.action_id;
    assert(d.out.active_mission==MissionType::kLine);
    if(last)assert(next==current);else {
      assert(next && next!=current && d.out.command.action==unit);
      d.Tick(.01,{next,true,false,false});
    }
    d.Tick(.01,{999999,true,true,false}); // A stale/unrelated DONE is not a completed block.
    assert(d.out.active_mission==MissionType::kLine && d.out.goal_walk_actions_completed==0);
    d.Tick(.01,{current,true,true,false});
    if(!last)assert(d.out.active_mission==MissionType::kLine);
    current=next;
  }
  assert(d.out.active_mission==MissionType::kGoal);
  assert(d.out.goal.mode==GoalMode::kWaitCameraGoal);
  assert(d.out.command.camera_request==CameraRequest::kGoal);
  assert(d.out.goal_walk_actions_completed==1 && d.out.goal_mission_index==mission_index);
  // A second empty look consumes no extra walk after the configured budget.
  d.frame.camera_feedback={CameraMode::kTransition,false};d.Tick();
  d.frame.camera_feedback={CameraMode::kGoal,true};d.Tick();
  d.frame.begin_manual_observation=true;d.Tick();
  d.frame.begin_manual_observation=false;
  for(int i=0;i<40 && d.out.goal.mode!=GoalMode::kFailed;++i)d.Tick(.05);
  assert(d.out.goal.mode==GoalMode::kFailed && d.out.has_ball);
  assert(d.out.goal.entry_line_retries==1 && d.out.command.action_id==0);
  d.core.Reset();d.Tick();
  assert(d.out.goal.entry_line_retries==0);
}

void StartEstimatesAndReset() {
  auto c = Configuration().line_p2p; c.straight_repetitions = 1;
  LinePlan p(c);
  p.Begin(MissionAction::kStepForwardFour);
  ControlCommand a; a.command_type=CommandType::kAction; a.action_id=10; a.action=MissionAction::kLineStraightUnit;
  p.Issued(a,0); assert(!p.StartEstimate(10));
  p.Feedback({10,true,false,false},.1,0); assert(p.StartEstimate(10)==.1);
  p.Begin(MissionAction::kStepForwardFour);
  auto b=a; b.action_id=11;
  p.Issued(b,10);
  p.Feedback({11,true,false,false},.2,.4);
  assert(!p.StartEstimate(11)); // Storage ACK is never a queued start event.
  p.Feedback({10,true,true,false},1,.4);
  assert(std::abs(*p.StartEstimate(11)-1.4)<1e-9);
  p.Feedback({10,true,true,false},2,.4);
  assert(std::abs(*p.StartEstimate(11)-1.4)<1e-9);
  p.ClearEvidence(); assert(p.StartEstimate(11));
  p.Reset(); assert(!p.Contains(10) && !p.Active());
  p.Begin(MissionAction::kStepForwardFour); p.Issued(a,0);
  p.Feedback({10,true,false,false},.1,.4);
  p.Begin(MissionAction::kStepForwardFour); p.Issued(b,10);
  p.Feedback({10,true,true,false},1,.4);
  assert(!p.StartEstimate(11)); // No queued acceptance confirmation yet.
  p.Feedback({11,true,false,false},1.7,.4);
  assert(p.StartEstimate(11)==1.7);

  // RESUME discards a previously finished window, not the action/DONE state.
  LinePlan resumed(c);resumed.Begin(MissionAction::kStepForwardFour);resumed.Issued(a,0);
  resumed.Feedback({10,true,false,false},0,0);
  LineGuide old;old.valid=true;old.offset=-.4;
  for(int i=0;i<10;++i)resumed.Observe(.5+.04*i,old,{},0);
  assert(resumed.Finish(10,1,0) && resumed.WindowFinished(10));
  resumed.ClearEvidence();
  assert(!resumed.WindowFinished(10) && resumed.Contains(10) && !resumed.Done(10));
  auto fresh=old;fresh.offset=.4;
  for(int i=0;i<10;++i)resumed.Observe(1.1+.04*i,fresh,{},0);
  const auto rebuilt=resumed.Finish(10,1.5,0);
  assert(rebuilt && rebuilt->offset>.39);

  // Switching to an object during an existing LINE block must not change the
  // block's source; missing virtual frames must not reuse a held guide.
  LinePlan line_source(c,10,5);
  line_source.Begin(MissionAction::kStepForwardFour);line_source.Issued(a,0);
  line_source.Feedback({10,true,false,false},0,0);
  auto virtual_guide=old;virtual_guide.offset=.6;
  for(int i=0;i<10;++i)
    line_source.Observe(.1*(i+1),old,virtual_guide,77);
  const auto real_mean=line_source.Finish(10,1.1,0);
  assert(real_mean && real_mean->offset<-.39);

  LinePlan object_source(c,10,5);
  object_source.Begin(MissionAction::kStepForwardFour,77);
  object_source.Issued(a,0);
  object_source.Feedback({10,true,false,false},0,0);
  for(int i=0;i<10;++i)
    object_source.Observe(.1*(i+1),old,i<5 ? virtual_guide : LineGuide{},77);
  const auto object_mean=object_source.Finish(10,1.1,0);
  assert(object_mean && object_mean->offset>.59);
  const auto object_stats=object_source.Stats(10);
  assert(object_stats && object_stats->valid_frames==5 && object_stats->used_frames==5);

  LinePlan object(c,10,5);
  object.Begin(MissionAction::kStepForwardFour,77);object.Issued(a,0);
  object.Feedback({10,true,false,false},0,0);
  for(int i=0;i<10;++i)object.ObserveObject(.1*i,77,true,10,7);
  assert(object.ObjectQualified(10));
  object.ClearEvidence();
  assert(!object.ObjectQualified(10));
  const auto evidence=object.CurrentObjectEvidence(77);
  assert(evidence && evidence->frames==0 && !evidence->qualified);
}

void SparseUnitsAndFreshVerification() {
  {
    Driver d(Configuration()); d.Tick(); auto id=d.out.command.action_id;
    d.Tick(.01,{id,true,false,false});
    d.frame.line_centers.clear();
    for(int n=0;n<4;++n) {
      d.Tick(.01,{id,true,false,true});
      const auto next=d.out.command.action_id;
      if(n<3) {
        assert(next!=id && d.out.command.action==MissionAction::kLineStraightUnit);
        d.Tick(.01,{next,true,false,false});
      } else {
        assert(next==id && !d.core.LastReadyLineGuide());
      }
      d.Tick(.01,{id,true,true,false}); id=next;
    }
    assert(d.out.line_in_recovery && !d.out.terminal_hold);
  }
  for(int hits:{6,7}) {
    auto c=Configuration();c.enable_ball=true;
    Driver d(c);d.Tick();const auto id=d.out.command.action_id;
    d.Tick(.01,{id,true,false,false});d.frame.ball_target=Target();
    for(int i=0;i<6;++i)d.Tick();
    d.frame.ball_target.reset();d.Tick(.01,{id,true,true,false});
    for(int i=1;i<=9;++i) {
      d.frame.ball_target = i >= 10-hits ? std::optional<ObjectTarget>(Target()) : std::nullopt;
      d.Tick(.12);
      if(i<9) assert(d.out.active_mission==MissionType::kLine);
    }
    assert((d.out.active_mission==MissionType::kBall)==(hits==7));
    if(hits==6) assert(d.out.command.action==MissionAction::kTurnLeftInPlace);
  }
  {
    auto c=Configuration();c.enable_ball=true;
    Driver d(c);d.Tick();const auto id=d.out.command.action_id;
    d.Tick(.01,{id,true,false,false});d.frame.ball_target=Target(320);
    for(int i=0;i<6;++i)d.Tick();
    d.frame.ball_target.reset();d.Tick(.01,{id,true,true,false});
    // More than one second but fewer than ten fresh frames cannot finish.
    for(int i=0;i<5;++i)d.Tick(.25);
    assert(d.out.post_motion_observing && d.out.active_mission==MissionType::kLine);
    for(int i=0;i<4;++i)d.Tick(.01);
    assert(d.out.candidate_mission==MissionType::kNone);
    assert(d.out.command.action!=MissionAction::kTurnLeftInPlace &&
           d.out.command.action!=MissionAction::kStepForwardRightOneHurdle);
  }
}

void CandidateWithoutLastDetectionDeadline(bool hurdle, int hits) {
  auto c=Configuration();c.enable_ball=!hurdle;c.enable_hurdle=hurdle;
  Driver d(c);d.Tick();const auto id=d.out.command.action_id;
  d.Tick(.01,{id,true,false,false});
  auto& target=hurdle?d.frame.hurdle_target:d.frame.ball_target;
  target=Target();
  for(int i=0;i<(hurdle?7:6);++i)d.Tick();
  target.reset();d.Tick(.01,{id,true,true,false});
  // The initial missing frame plus nine new frames form the complete window.
  // The last two frames miss, making the final detection 0.4 s old.
  for(int i=1;i<=9;++i) {
    target=i<=hits ? std::optional<ObjectTarget>(Target()) : std::nullopt;
    d.Tick(.2);
    if(i<9)assert(d.out.active_mission==MissionType::kLine);
  }
  const auto mission=hurdle?MissionType::kHurdle:MissionType::kBall;
  assert((d.out.active_mission==mission)==(hits==7));
}

void FineRecoveryTurnSettle(bool left) {
  auto c=Configuration();c.fine_motion_settle_sec=.25;
  c.line_p2p.failure_observation_sec=1;c.line_p2p.recovery_observation_sec=1;
  Driver d(c);d.Line(left?256:384);d.Tick();
  assert(d.out.command.action_id!=0);
  d.frame.line_centers.clear();
  const auto turn=left?MissionAction::kTurnLeftInPlace:MissionAction::kStepForwardRightOneHurdle;
  std::uint64_t completed=0;
  for(int i=0;i<200;++i) {
    CommandDeliveryFeedback feedback;
    if(d.out.command.action_id!=0 && d.out.command.action_id!=completed) {
      completed=d.out.command.action_id;feedback={completed,true,true,false};
    }
    d.Tick(.05,feedback);
    if(d.out.command.action==turn && d.out.command.action_id!=0)break;
  }
  assert(d.out.command.action==turn && d.out.command.action_id!=0);
  const auto id=d.out.command.action_id;
  d.Tick(.05,{id,true,true,false});
  const double decision_at=d.frame.now_sec+c.line_p2p.recovery_observation_sec;
  bool decided=false;
  for(int i=0;i<40;++i) {
    d.Tick(.05);
    if(d.frame.now_sec+1e-9<decision_at) {
      assert(d.out.command.action_id==0);
    } else {
      assert(d.out.command.action==turn && d.out.command.action_id!=0 &&
             d.out.command.action_id!=id);
      decided=true;break;
    }
  }
  assert(decided);
}

// Drive only protocol feedback; never bypass real stance changes/observation.
MissionAction AwaitLossTurn(Driver &d) {
  std::uint64_t completed=0;
  for (int i=0;i<400;++i) {
    const auto &command=d.out.command;
    if (command.action_id && (command.action==MissionAction::kTurnLeftInPlace ||
                              command.action==MissionAction::kStepForwardRightOneHurdle))
      return command.action;
    if (d.out.terminal_hold) return MissionAction::kNone;
    CommandDeliveryFeedback feedback;
    if (command.action_id && command.action_id!=completed) {
      completed=command.action_id;
      feedback={completed,true,true,false};
    }
    d.Tick(.05,feedback);
  }
  assert(false && "loss recovery neither turned nor reached FINAL HOLD");
  return MissionAction::kNone;
}

void LossRecoveryPriority() {
  for (int points : {0,1,2,3}) for (bool visible_left : {false,true}) {
    auto c=Configuration();
    c.line_p2p.left_repetitions=c.line_p2p.straight_repetitions=c.line_p2p.right_repetitions=1;
    Driver d(c);
    // Deliberately opposite history, within STRAIGHT deadband: its sign counts.
    d.Line(320+(visible_left?16:-16));d.Tick();
    assert(d.out.command.action==MissionAction::kLineStraightUnit);
    d.frame.line_centers.clear();
    for (int n=0;n<points;++n)
      d.frame.line_centers.push_back({visible_left?200.:440.,450.-n*50});
    const bool expected_left=points>=2?visible_left:!visible_left;
    assert(AwaitLossTurn(d)==(expected_left?MissionAction::kTurnLeftInPlace:MissionAction::kStepForwardRightOneHurdle));
  }
  // Exact center falls through to the signed history, not an arbitrary side.
  auto c=Configuration();c.line_p2p.straight_repetitions=1;
  Driver centered(c);centered.Line(304);centered.Tick();
  centered.frame.line_centers={{300,450},{340,400}};
  assert(AwaitLossTurn(centered)==MissionAction::kTurnLeftInPlace);

  // Exact zero replaces an older negative score; Reset also removes history.
  Driver zero(c);zero.Line(304);zero.Tick();
  auto old=zero.out.command.action_id;
  zero.Line(320);
  zero.Tick(.01,{old,true,false,false});
  for (int i=0;i<12;++i)zero.Tick(.05);
  zero.Tick(.01,{old,true,false,true});
  const auto straight=zero.out.command.action_id;
  assert(straight && straight!=old);
  zero.Tick(.01,{straight,true,false,false});zero.Tick(.01,{old,true,true,false});
  zero.frame.line_centers.clear();
  assert(AwaitLossTurn(zero)==MissionAction::kNone && zero.out.terminal_hold);
  zero.core.Reset();zero.frame.line_centers.clear();zero.out={};zero.Tick();
  assert(AwaitLossTurn(zero)==MissionAction::kNone && zero.out.terminal_hold);

  // Nonfinite coordinates do not qualify as two current centers.
  Driver malformed(c);malformed.Line(304);malformed.Tick();
  malformed.frame.line_centers={{440,450},{std::numeric_limits<double>::quiet_NaN(),400}};
  assert(AwaitLossTurn(malformed)==MissionAction::kTurnLeftInPlace);

  // Fresh start with no direction evidence observes five windows, then holds.
  Driver fresh(c);fresh.frame.line_centers.clear();fresh.Tick();
  fresh.Tick(1.9);assert(fresh.out.command.action_id==0);
  assert(AwaitLossTurn(fresh)==MissionAction::kNone && fresh.out.terminal_hold);
}

void LossRecoveryBudgets() {
  auto c=Configuration();
  c.line_p2p.visible_recovery_max_turns=2;
  c.line_p2p.last_score_recovery_max_turns=1;
  Driver d(c);d.frame.line_centers={{200,450},{200,400}};d.Tick();
  assert(AwaitLossTurn(d)==MissionAction::kTurnLeftInPlace);
  auto id=d.out.command.action_id;
  d.frame.line_centers={{440,450},{440,400}};d.Tick(.05,{id,true,true,false});
  assert(AwaitLossTurn(d)==MissionAction::kStepForwardRightOneHurdle);
  id=d.out.command.action_id;
  d.frame.line_centers={{440,450},{440,400}};d.Tick(.05,{id,true,true,false});
  assert(AwaitLossTurn(d)==MissionAction::kNone && d.out.terminal_hold);
  d.Line(320);d.Tick(3);assert(d.out.terminal_hold && !d.out.command.action_id);

  // Zero disables the selected recovery source explicitly.
  for (bool visible : {false,true}) {
    auto disabled=c;
    disabled.line_p2p.visible_recovery_max_turns=0;
    disabled.line_p2p.last_score_recovery_max_turns=0;
    Driver held(disabled);held.frame.line_centers.clear();
    if (visible)held.frame.line_centers={{200,450},{200,400}};
    held.Tick();
    assert(AwaitLossTurn(held)==MissionAction::kNone && held.out.terminal_hold);
  }
}

void PauseDuringSearch() {
  auto c=Configuration(); c.enable_ball=true;
  Driver d(c); d.Tick(); auto walk=d.out.command.action_id;
  d.Tick(.01,{walk,true,false,false}); d.frame.ball_target=Target();
  for(int n=0;n<6;++n)d.Tick();
  d.frame.ball_target.reset(); d.Tick(.01,{walk,true,true,false}); d.Window(false);
  const auto turn=d.out.command.action_id; assert(d.out.command.action==MissionAction::kTurnLeftInPlace);
  d.frame.allow_new_commands=false; d.frame.allow_new_mission_entry=false; d.frame.advance_line_fsm=false;
  d.Tick(.01,{turn,true,true,false}); assert(d.out.candidate_search_turns==1);
  d.core.ClearObservationEvidence();
  d.frame.allow_new_commands=d.frame.allow_new_mission_entry=d.frame.advance_line_fsm=true;
  d.Tick(); assert(d.out.post_motion_observing && d.out.candidate_search_turns==1);
  d.core.Reset(true); d.Tick();
  assert(d.out.candidate_mission==MissionType::kNone && d.out.command.action_id>turn);
}
void CurveSearchOneShot() {
  auto c=Configuration();
  c.enable_ball=true;
  c.curve_search.enabled=true;
  c.curve_search.stable_window=10;
  c.curve_search.stable_min_hits=7;
  c.line_p2p.left_repetitions=c.line_p2p.right_repetitions=1;
  c.line_p2p.straight_repetitions=1;
  Driver d(c);
  d.frame.manual_observation_control=false;
  d.frame.line_centers.clear();
  // Right bend: near straight, far shifted to image right.
  for(int i=0;i<8;++i)
    d.frame.line_centers.push_back({i<4 ? 320.0 : 320.0+40.0*(i-3.5),
                                    450.0-50.0*i});
  d.Tick();
  const auto walking=d.out.command.action_id;
  assert(walking && LinePlan::IsUnit(d.out.command.action));
  // Enough valid frames must NOT issue the turn while A is still outstanding.
  for(int i=0;i<14;++i) d.Tick(.09);
  assert(d.out.curve_search_spent);
  assert(d.out.candidate_mission==MissionType::kBall && d.out.candidate_phase==1);
  assert(d.out.command.action!=MissionAction::kTurnLeftInPlace);
  // The existing issued LINE action must finish before a curve-search turn.
  d.Tick(.01,{walking,true,true,false});
  assert(d.out.command.action==MissionAction::kTurnLeftInPlace);
  const auto turn=d.out.command.action_id;
  assert(turn && turn!=walking);
  d.Tick(.02,{turn,true,true,false});
  assert(d.out.candidate_search_turns==1);
  // No BALL in the post-turn 1s observation -> exactly one inverse turn.
  for(int i=0;i<13;++i) d.Tick(.10);
  assert(d.out.candidate_phase==4);
  assert(d.out.command.action==MissionAction::kStepForwardRightOneHurdle);
  const auto restore=d.out.command.action_id;
  assert(restore && restore!=turn);
  d.Tick(.01,{restore,true,true,false});
  d.Tick(.02);
  assert(d.out.candidate_phase==0);
  assert(d.out.curve_search_spent); // No N-block rearm, including GOAL-side curve.
  for(int i=0;i<30;++i) d.Tick(.1);
  assert(d.out.candidate_mission==MissionType::kNone);
  assert(d.out.curve_search_spent);
}

}

int main() {
  CurveSearchOneShot();
  NormalBlocks(false, .40); NormalBlocks(true, .40); NormalBlocks(true, .20);
  StartEstimatesAndReset(); SparseUnitsAndFreshVerification();
  CandidateWinsReady(false,false); CandidateWinsReady(true,false); CandidateWinsReady(true,true);
  SearchAndRestore(true,1); SearchAndRestore(false,2); SearchAndRestore(true,0);
  SearchAndRestore(false,1,true); SearchAndRestore(false,0,true);
  // An old direction is retained even after a long real-DONE wait. Both
  // classes still obey the same search budget and opposite restoration.
  SearchAndRestore(true,0,false,6.0); SearchAndRestore(false,0,true,6.0);
  GoalBlocks(true,3); GoalBlocks(false,1); GoalBlocks(false,3); GoalBlocks(true,0);
  GoalBlocks(true,3,2); GoalBlocks(true,2,0);
  GoalEmptyViewLineRetry(true); GoalEmptyViewLineRetry(false);
  PauseDuringSearch();
  LossRecoveryPriority();LossRecoveryBudgets();
  for(bool left:{false,true})FineRecoveryTurnSettle(left);
  for(bool hurdle:{false,true})for(int hits:{6,7})
    CandidateWithoutLastDetectionDeadline(hurdle,hits);
}
