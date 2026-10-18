#include "vision_core/config_loader.hpp"
#include "vision_core/mission_controller.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
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
  c.candidate_search_max_turns = 2; c.candidate_direction_max_age_sec = 5;
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

void NormalBlocks() {
  for (const auto direction : {-1,0,1}) {
    Driver d(Configuration());
    d.Line(320 + direction*64);
    d.Tick();
    const auto unit = direction < 0 ? MissionAction::kLineLeftUnit :
        direction > 0 ? MissionAction::kLineRightUnit : MissionAction::kLineStraightUnit;
    const int repeats = direction == 0 ? 4 : 3;
    std::set<std::uint64_t> issued;
    auto current = d.out.command.action_id;
    assert(d.out.command.action == unit && current);
    issued.insert(current);
    d.Tick(.01, {current,true,false,false});
    for (int n = 0; n < repeats; ++n) {
      // Later guides ask for the opposite direction; intermediate READY must
      // still reserve the ORIGINAL unit. Direction is reconsidered at block end.
      if (n) d.Line(320 - direction*64);
      for (int i=0; i<12; ++i) d.Tick(.04);
      d.Tick(.01, {current,true,false,true});
      const auto next = d.out.command.action_id;
      assert(next && next != current);
      assert(!d.out.post_motion_observing);
      if (n + 1 < repeats) {
        assert(d.out.command.action == unit);
        assert(!d.out.line_window_stats); // No per-unit five-sample gate.
        issued.insert(next);
      } else {
        assert(d.out.line_window_stats && d.out.line_window_stats->used_frames >= 5);
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

void SearchAndRestore(bool left, int found_after, bool hurdle = false) {
  auto c = Configuration(); c.enable_ball = !hurdle; c.enable_hurdle = hurdle;
  Driver d(c); d.Tick(); auto id = d.out.command.action_id;
  d.Tick(.01, {id,true,false,false});
  const auto target = Target(left ? 150 : 550);
  if (hurdle) d.frame.hurdle_target = target; else d.frame.ball_target = target;
  for (int i=0;i<(hurdle?7:6);++i) d.Tick();
  assert(d.out.candidate_mission == (hurdle ? MissionType::kHurdle : MissionType::kBall));
  d.frame.ball_target.reset(); d.frame.hurdle_target.reset();
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

void GoalBlocks(bool straight, int fallback) {
  auto c = Configuration(); c.enable_goal = true; c.initial_has_ball = true;
  c.goal.post_pickup_walk_fallback_actions = fallback;
  Driver d(c); if (!straight) d.Line(384);
  d.Tick();
  if (fallback == 0) {
    assert(d.out.active_mission == MissionType::kGoal && d.out.command.camera_request == CameraRequest::kGoal);
    return;
  }
  auto current = d.out.command.action_id;
  d.Tick(.01, {current,true,false,false});
  const int plans = straight ? 1 : fallback;
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
    if (last) {
      assert(d.out.active_mission == MissionType::kGoal);
      assert(d.out.command.camera_request == CameraRequest::kGoal);
      assert(d.out.goal_five_step_completed == straight);
    } else assert(d.out.active_mission == MissionType::kLine);
    current = next;
  }
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
}

void SparseUnitsAndFreshVerification() {
  {
    Driver d(Configuration()); d.Tick(); auto id=d.out.command.action_id;
    d.Tick(.01,{id,true,false,false});
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
}

int main() {
  NormalBlocks(); StartEstimatesAndReset(); SparseUnitsAndFreshVerification();
  CandidateWinsReady(false,false); CandidateWinsReady(true,false); CandidateWinsReady(true,true);
  SearchAndRestore(true,1); SearchAndRestore(false,2); SearchAndRestore(true,0);
  SearchAndRestore(false,1,true); SearchAndRestore(false,0,true);
  GoalBlocks(true,3); GoalBlocks(false,1); GoalBlocks(false,3); GoalBlocks(true,0);
  PauseDuringSearch();
}
