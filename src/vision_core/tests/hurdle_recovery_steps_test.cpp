#include "vision_core/config_loader.hpp"
#include "verified_entry_fixture.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>

using namespace vision_core;
namespace {
ObjectTarget Target(double v=100) {
  ObjectTarget t;t.center_px={320,v};t.box_px={220,v-10,200,20};
  t.width_px=200;t.height_px=20;t.area_px=4000;t.confidence=1;return t;
}
struct Driver {
  HurdleController core;
  HurdleResult out;
  double now=2;
  CameraFeedback camera{CameraMode::kForward,true};
  explicit Driver(const HurdleConfig &c):core(c,30) {}
  void Tick(std::optional<ObjectTarget> target={},ActionExecutionFeedback feedback={},
            bool allow=true,double dt=.1) {
    now+=dt;out=core.Compute(target,640,480,now,camera,feedback,{},allow);
  }
  void Begin(int phase,const HurdleConfig &c) {
    if(phase==0) {
      core.EnterVerified(Target(),std::deque<bool>(10,true),640,480,now);
      core.UseVerifiedObservation({},std::deque<bool>(10,false),640,480);
      Tick({}, {false,false,true});
    } else {
      camera={CameraMode::kDown,true};
      if(phase==2)core.StartTestExecute();
      else {core.StartTestNear();Tick({}, {},false);}
      for(int i=0;i<20;++i)Tick(Target(phase==2?200:40),{},false);
      assert(out.mode==(phase==2?HurdleMode::kFineAdjust:HurdleMode::kCoarseAdjust));
      for(int i=0;i<c.lost_frames;++i)Tick({}, {},false);
    }
    assert(out.mode==HurdleMode::kRecoveryStep);
  }
  void Observe(bool found,int phase) {
    const auto before=out.mode;
    for(int i=0;i<12 && out.mode==before;++i)
      Tick(found?std::optional<ObjectTarget>(Target(phase==2?200:40)):std::nullopt);
  }
};
HurdleConfig Configuration() {
  auto c=LoadDefaultAlgorithmConfig().hurdle;
  c.stable_window=10;c.stable_min_hits=7;c.down_search_min_hits=3;c.lost_frames=15;
  c.down_one_step_v_px=80;c.fine_mode_v_px=180;c.contact_v_px=240;
  c.entry_recovery_max_steps=3;c.recovery_max_steps=2;
  c.recovery_observation_sec=1;c.recovery_max_observations=2;
  return c;
}
void Recovery(int phase,bool found) {
  const auto c=Configuration();Driver d(c);d.Begin(phase,c);
  const int limit=phase==0?c.entry_recovery_max_steps:c.recovery_max_steps;
  const auto action=phase==2?MissionAction::kStepForwardHalf:MissionAction::kStepForwardOne;
  assert(d.core.RecoveryStepLimit()==limit && d.core.EntryRecovery()==(phase==0));
  for(int n=1;n<=limit;++n) {
    d.Tick({}, {},false);assert(d.core.RecoveryStepsCompleted()==n-1);
    d.Tick();assert(d.out.action.action==action);
    // Arbitrarily long execution and moving detections cannot consume a step.
    for(int i=0;i<10;++i)d.Tick(Target(),{false,true,false},true,.2);
    assert(d.core.RecoveryStepsCompleted()==n-1 && d.out.mode==HurdleMode::kRecoveryStep);
    d.Tick({}, {true,false,false});
    assert(d.out.mode==HurdleMode::kRecoveryStepObserve && d.core.RecoveryStepsCompleted()==n);
    assert(d.out.action.action==MissionAction::kNone);
    d.Tick({}, {true,false,false});assert(d.core.RecoveryStepsCompleted()==n);
    d.Observe(found,phase);
    if(found) {
      assert(d.out.mode==(phase==0?HurdleMode::kApproach:
          phase==1?HurdleMode::kCoarseAdjust:HurdleMode::kFineAdjust));
      assert(d.core.RecoveryStepsCompleted()==0);
      return;
    }
    if(n<limit)assert(d.out.mode==HurdleMode::kRecoveryStep);
  }
  if(phase==0)assert(d.out.mode==HurdleMode::kLineFollow && !d.out.active);
  else {
    assert(d.out.mode==HurdleMode::kRecoveryDown);
    for(int i=0;i<30 && d.out.mode!=HurdleMode::kFailed;++i) {
      d.Tick();assert(d.out.action.action==MissionAction::kNone);
    }
    assert(d.out.mode==HurdleMode::kFailed && d.core.RecoveryObservationsCompleted()==2);
  }
}
void EntryExhaustionResumesLine() {
  auto c=LoadDefaultAlgorithmConfig();c.enable_hurdle=true;c.enable_ball=c.enable_goal=false;
  c.initial_has_ball=false;c.goal_trial_start_goal=false;c.test_mission=MissionType::kNone;
  c.transition_settle_sec=0;c.hurdle.entry_recovery_max_steps=1;
  MissionController core(c);AdmitObjectAtRest(core,Target(),MissionType::kHurdle);
  MissionFrameInput f;f.image_width=640;f.image_height=480;
  f.camera_feedback={CameraMode::kForward,true};f.command_transport_enabled=true;
  f.manual_observation_control=true;f.begin_manual_observation=true;f.now_sec=3;
  auto out=core.Step(f);f.begin_manual_observation=false;
  for(int i=0;i<12;++i){f.now_sec+=.1;out=core.Step(f);}
  f.manual_observation_control=false;
  int steps=0;std::uint64_t completed=0;
  for(int i=0;i<100 && out.active_mission!=MissionType::kLine;++i) {
    f.delivery_feedback={};
    if(out.command.action_id && out.command.action_id!=completed) {
      completed=out.command.action_id;++steps;
      assert(out.command.action==MissionAction::kStepForwardOne);
      f.delivery_feedback={completed,true,true,false};
    }
    f.now_sec+=.1;out=core.Step(f);
  }
  assert(steps==1 && out.active_mission==MissionType::kLine && !out.terminal_hold);
  f.delivery_feedback={};
  f.line_centers={{c.line_features.image_center_u,450},{c.line_features.image_center_u,400},
                  {c.line_features.image_center_u,350},{c.line_features.image_center_u,300}};
  bool walking=false;
  for(int i=0;i<40;++i) {
    f.now_sec+=.1;out=core.Step(f);
    if(out.command.action==MissionAction::kLineStraightUnit && out.command.action_id) {
      walking=true;break;
    }
  }
  assert(walking);
}
}
static void LockedHandoffUsesGeneralBudget(bool lost_at_handoff) {
  const auto c=Configuration();Driver d(c);
  if(lost_at_handoff)
    d.core.EnterContinuousRecovery(Target(),std::deque<bool>(10,true),640,480,d.now);
  else {
    d.core.EnterVerified(Target(),std::deque<bool>(10,true),640,480,d.now,false);
    d.core.UseVerifiedObservation({},std::deque<bool>(10,false),640,480);
    d.Tick({}, {false,false,true},false);
  }
  assert(!d.core.EntryRecovery() && d.core.RecoveryStepLimit()==c.recovery_max_steps);
  for(int n=1;n<=c.recovery_max_steps;++n) {
    d.Tick();assert(d.out.action.action==MissionAction::kStepForwardOne);
    d.Tick({}, {true,false,false});d.Observe(false,0);
  }
  assert(d.out.mode==HurdleMode::kRecoveryForward);
  for(int i=0;i<30 && d.out.mode!=HurdleMode::kFailed;++i)d.Tick();
  assert(d.out.mode==HurdleMode::kFailed);
}
int main() {
  LockedHandoffUsesGeneralBudget(false);LockedHandoffUsesGeneralBudget(true);
  for(int phase:{0,1,2})for(bool found:{false,true})Recovery(phase,found);
  EntryExhaustionResumesLine();
}
