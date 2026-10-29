#include "vision_core/config_loader.hpp"
#include "verified_entry_fixture.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>

using namespace vision_core;
namespace {
ObjectTarget Target(double u) {
  ObjectTarget t;t.center_px={u,80};t.box_px={u-20,60,40,40};
  t.width_px=t.height_px=40;t.area_px=1600;t.confidence=1;return t;
}
BallConfig Configuration() {
  auto c=LoadDefaultAlgorithmConfig().ball;
  c.far_u_des_px=320;c.recovery_center_tolerance_px=50;
  c.entry_min_hits=6;c.stable_window=10;c.stable_min_hits=7;c.lost_frames=3;
  c.recovery_max_steps=2;c.entry_recovery_max_steps=3;
  c.recovery_observation_sec=1;c.recovery_max_observations=2;
  return c;
}
struct Driver {
  BallController core;
  BallResult out;
  double now=0;
  CameraFeedback camera{CameraMode::kForward,true};
  explicit Driver(const BallConfig &c):core(c,30) {}
  void Tick(std::optional<ObjectTarget> target={},ActionExecutionFeedback feedback={},
            bool allow=true,double dt=.1) {
    now+=dt;out=core.Compute(target,640,480,now,camera,feedback,allow);
  }
  void Begin(bool entry,double u) {
    for(int i=0;i<7;++i)Tick(Target(u),{},false);
    if(entry) {
      core.UseVerifiedObservation({},std::deque<bool>(10,false),640,480);
      Tick({}, {true,false,true});
    } else {
      Tick(Target(u),{false,false,true},false); // Successful first observation.
      for(int i=0;i<3;++i)Tick({}, {},false);
    }
  }
  void Observe(bool found=false,double u=320) {
    const auto mode=out.mode;
    for(int i=0;i<12 && out.mode==mode;++i)
      Tick(found?std::optional<ObjectTarget>(Target(u)):std::nullopt);
  }
};

void Cycles(bool entry,int side) {
  const auto c=Configuration();Driver d(c);d.Begin(entry,320+side*120);
  const int limit=entry?c.entry_recovery_max_steps:c.recovery_max_steps;
  assert(d.core.RecoveryStepLimit()==limit && d.core.EntryRecovery()==entry);
  if(side==0) {
    assert(d.out.mode==BallMode::kForwardRecoveryObserve);
    d.Tick({}, {},true,.5);assert(d.out.action.action==MissionAction::kNone);
    d.Observe();
    assert(d.out.mode==BallMode::kForwardRecoveryStep);
  }
  for(int n=1;n<=limit;++n) {
    if(side) {
      assert(d.out.mode==BallMode::kForwardRecoveryTurn);
      d.Tick({}, {},false);assert(d.core.RecoveryStepsCompleted()==n-1);
      d.Tick();
      assert(d.out.action.action==(side<0?MissionAction::kTurnLeftInPlace:
                                         MissionAction::kStepForwardRightOneHurdle));
      // Even enough moving detections cannot bypass the committed step.
      for(int i=0;i<10;++i)d.Tick(Target(320),{false,true,false});
      assert(d.out.mode==BallMode::kForwardRecoveryTurn);
      d.Tick({}, {true,false,false});
      assert(d.out.mode==BallMode::kForwardRecoveryStep);
      assert(d.core.RecoveryWalkTurnsCompleted()==n);
    }
    d.Tick();assert(d.out.action.action==MissionAction::kStepForwardOne);
    d.Tick({}, {false,true,false});assert(d.core.RecoveryStepsCompleted()==n-1);
    d.Tick({}, {true,false,false});
    assert(d.out.mode==BallMode::kForwardRecoveryObserve && d.core.RecoveryStepsCompleted()==n);
    // Late/duplicate DONE while observing cannot count a second step.
    d.Tick({}, {true,false,false});assert(d.core.RecoveryStepsCompleted()==n);
    d.Observe();
    if(n<limit)
      assert(d.out.mode==(side?BallMode::kForwardRecoveryTurn:BallMode::kForwardRecoveryStep));
  }
  if(entry) {
    assert(d.out.mode==BallMode::kLineFollow && !d.out.active && !d.out.has_ball);
    assert(!d.out.pickup_failed); // Abandoned search is not a failed pickup.
  } else {
    assert(d.out.mode==BallMode::kBallRecoveryForward);
    for(int i=0;i<25 && d.out.mode!=BallMode::kFailed;++i)d.Tick();
    assert(d.out.mode==BallMode::kFailed && d.core.RecoveryObservationsCompleted()==2);
  }
}

void FoundAfterStep(bool entry) {
  Driver d(Configuration());d.Begin(entry,200);
  d.Tick();assert(d.out.action.action==MissionAction::kTurnLeftInPlace);
  d.Tick({}, {true,false,false});d.Tick();
  assert(d.out.action.action==MissionAction::kStepForwardOne);
  d.Tick({}, {true,false,false});d.Observe(true,320);
  assert(d.out.mode==BallMode::kApproachBall && d.core.RecoveryStepsCompleted()==0);
}

void DisabledSearch(bool entry) {
  auto c=Configuration();c.recovery_max_steps=c.entry_recovery_max_steps=0;
  Driver d(c);d.Begin(entry,200);
  for(int i=0;i<40 && d.out.mode!=BallMode::kLineFollow && d.out.mode!=BallMode::kFailed;++i) {
    assert(d.out.action.action==MissionAction::kNone);
    d.Tick();
  }
  assert(d.out.mode==(entry?BallMode::kLineFollow:BallMode::kFailed));
  assert(!d.out.has_ball && d.core.RecoveryStepsCompleted()==0);
}

void EntryExhaustionResumesLine() {
  auto c=LoadDefaultAlgorithmConfig();c.enable_hurdle=c.enable_goal=false;
  c.enable_ball=true;c.initial_has_ball=false;c.goal_trial_start_goal=false;
  c.test_mission=MissionType::kNone;c.transition_settle_sec=0;
  c.ball.entry_recovery_max_steps=1;
  MissionController core(c);AdmitObjectAtRest(core,Target(c.ball.far_u_des_px-120));
  MissionFrameInput f;f.image_width=640;f.image_height=480;
  f.camera_feedback={CameraMode::kForward,true};f.command_transport_enabled=true;
  f.manual_observation_control=true;f.begin_manual_observation=true;
  f.now_sec=3;auto out=core.Step(f);f.begin_manual_observation=false;
  // Fail the first post-entry observation without issuing an approach action.
  for(int i=0;i<12;++i){f.now_sec+=.1;out=core.Step(f);}
  f.manual_observation_control=false;
  int turns=0,steps=0;std::uint64_t completed=0;
  for(int i=0;i<200 && out.active_mission!=MissionType::kLine;++i) {
    f.delivery_feedback={};
    if(out.command.action_id && out.command.action_id!=completed) {
      completed=out.command.action_id;
      if(out.command.action==MissionAction::kTurnLeftInPlace)++turns;
      else if(out.command.action==MissionAction::kStepForwardOne)++steps;
      else assert(false && "unexpected entry recovery action");
      f.delivery_feedback={completed,true,true,false};
    }
    f.now_sec+=.1;out=core.Step(f);
  }
  assert(turns==1 && steps==1 && out.active_mission==MissionType::kLine && !out.has_ball);
  // Normal LINE reacquisition/planning can issue a fresh walking block.
  f.line_centers={{c.line_features.image_center_u,450},{c.line_features.image_center_u,400},
                  {c.line_features.image_center_u,350},{c.line_features.image_center_u,300}};
  f.delivery_feedback={};
  bool walking=false;
  for(int i=0;i<40;++i) {
    f.now_sec+=.1;out=core.Step(f);
    if(out.command.action==MissionAction::kLineStraightUnit && out.command.action_id) {
      walking=true;break;
    }
  }
  assert(walking && !out.has_ball);
}

void LockedHandoffUsesGeneralBudget(bool lost_at_handoff) {
  const auto c=Configuration();Driver d(c);
  if(lost_at_handoff)
    d.core.EnterContinuousRecovery(Target(200),std::deque<bool>(10,true),640,480,d.now);
  else {
    d.core.EnterVerified(Target(200),std::deque<bool>(10,true),640,480,d.now,false);
    d.core.UseVerifiedObservation({},std::deque<bool>(10,false),640,480);
    d.Tick({}, {false,false,true},false);
  }
  assert(!d.core.EntryRecovery() && d.core.RecoveryStepLimit()==c.recovery_max_steps);
  for(int n=1;n<=c.recovery_max_steps;++n) {
    d.Tick();assert(d.out.action.action==MissionAction::kTurnLeftInPlace);
    d.Tick({}, {true,false,false});d.Tick();
    assert(d.out.action.action==MissionAction::kStepForwardOne);
    d.Tick({}, {true,false,false});d.Observe();
  }
  assert(d.out.mode==BallMode::kBallRecoveryForward);
  for(int i=0;i<30 && d.out.mode!=BallMode::kFailed;++i)d.Tick();
  assert(d.out.mode==BallMode::kFailed && !d.core.EntryRecovery());
}
}
int main() {
  for(bool entry:{false,true}) {
    for(int side:{-1,0,1})Cycles(entry,side);
    FoundAfterStep(entry);
    DisabledSearch(entry);
  }
  EntryExhaustionResumesLine();
  LockedHandoffUsesGeneralBudget(false);LockedHandoffUsesGeneralBudget(true);
}
