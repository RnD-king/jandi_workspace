#include "vision_core/config_loader.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>

using namespace vision_core;
namespace {
ObjectTarget Board(double u) {
  ObjectTarget t;t.center_px={u,150};t.box_px={u-10,140,20,20};
  t.width_px=t.height_px=20;t.area_px=400;t.confidence=1;return t;
}
GoalConfig Configuration() {
  auto c=LoadDefaultAlgorithmConfig().goal;
  c.stable_window=10;c.stable_min_hits=7;c.lost_frames=15;c.smooth_alpha=1;
  c.target_u_px=320;c.recovery_center_tolerance_px=10;
  c.recovery_turn_max=1;c.recovery_observation_sec=1;c.recovery_max_observations=2;
  return c;
}
struct Driver {
  GoalConfig config;
  GoalController core;
  GoalResult out;
  CameraFeedback camera{CameraMode::kGoal,true};
  double now=0;
  explicit Driver(GoalConfig c=Configuration()):config(c),core(c,30) {}
  void Tick(std::optional<ObjectTarget> board={},GoalPoseObservation pose={},
            ActionExecutionFeedback feedback={},bool allow=true,bool image_only=false) {
    now+=.1;
    out=core.Compute({},board,pose,640,480,now,true,camera,feedback,allow,
        pose.valid?std::optional<double>(pose.z_m):std::nullopt,false,image_only);
  }
  void Window(bool found,ActionExecutionFeedback feedback,double u=320) {
    const auto board=found?std::optional<ObjectTarget>(Board(u)):std::nullopt;
    const GoalPoseObservation pose=found?GoalPoseObservation{true,0,1.5,0,1}:GoalPoseObservation{};
    const std::deque<bool> hits(10,found);
    core.UseVerifiedObservation(board,pose,hits,hits,640,480);
    Tick(board,pose,feedback);
  }
  static GoalMode Resume(int phase) {
    return phase==0?GoalMode::kApproach:GoalMode::kFineAdjust;
  }
  static GoalMode Stationary(int phase) {
    return phase==0?GoalMode::kRecoverySearch:GoalMode::kFineRecovery;
  }
  void BeginLoss(int phase,double u) {
    if(phase==2)core.StartTestExecute();
    else {core.StartGoalTrial();Tick({}, {},{},false);}
    const GoalPoseObservation pose{true,0,phase==1?.6:1.5,0,1};
    core.UseVerifiedObservation(Board(u),pose,std::deque<bool>(10,true),
        std::deque<bool>(10,true),640,480);
    Tick(Board(u),pose,{false,false,true},false);
    if(phase==1) {
      assert(out.mode==GoalMode::kWaitFinePose);
      Tick(Board(u),pose); // issue the requested feet transition
      Tick(Board(u),pose,{true,false,false});
      core.UseVerifiedObservation(Board(u),pose,std::deque<bool>(10,true),std::deque<bool>(10,true),640,480);
      Tick(Board(u),pose,{},false);
    }
    assert(out.mode==Resume(phase));
    for(int i=1;i<=config.lost_frames;++i) {
      Tick({}, {},{},false);
      if(i<config.lost_frames)assert(out.mode==Resume(phase));
    }
  }
};
void Center(int phase,bool found,double offset) {
  Driver d;d.BeginLoss(phase,320+offset);
  assert(d.out.mode==GoalMode::kRecoveryForwardStep);
  const auto action=phase!=0?MissionAction::kStepForwardHalf:MissionAction::kStepForwardOne;
  d.Tick({}, {},{},false);assert(d.out.action.action==action);
  // An uncommitted preview cannot consume a stray DONE.
  d.Tick({}, {},{true,false,true},false);
  assert(d.out.mode==GoalMode::kRecoveryForwardStep && d.out.action.action==action);
  d.Tick();assert(d.out.action.action==action);
  for(int i=0;i<12;++i)d.Tick(Board(180),{true,0,1.5,0,1},{false,true,false});
  assert(d.out.mode==GoalMode::kRecoveryForwardStep && d.out.action.action==MissionAction::kNone);
  // A completed observation before DONE is not a completed motion.
  d.Window(true,{false,false,true});
  assert(d.out.mode==GoalMode::kRecoveryForwardStep);
  d.Tick({}, {},{true,false,false});
  assert(d.out.mode==GoalMode::kRecoveryForwardStep);
  d.Window(found,{false,false,true});
  assert(d.out.mode==(found?Driver::Resume(phase):Driver::Stationary(phase)));
  assert(d.core.RecoveryTurnsCompleted()==0 && d.out.action.action==MissionAction::kNone);
  if(!found) {
    // The centered policy takes one step, then the existing bounded observer.
    for(int i=0;i<40 && d.out.mode!=GoalMode::kFailed;++i) {
      d.Tick();assert(d.out.action.action==MissionAction::kNone);
    }
    assert(d.out.mode==GoalMode::kFailed && d.core.RecoveryObservationsCompleted()==2);
  }
}
void ExtraTurn(int phase,bool left,bool found_after_extra) {
  Driver d;d.BeginLoss(phase,left?180:460);
  assert(d.out.mode==GoalMode::kRecoveryTurn);
  const auto action=phase!=0 ? (left?MissionAction::kTurnLeft:MissionAction::kTurnRight)
      : (left?MissionAction::kTurnLeftInPlace:MissionAction::kStepForwardRightOneHurdle);
  d.Tick();assert(d.out.action.action==action);
  assert(d.out.action.target_yaw_deg==(phase!=0?15:0));
  const double opposite_u=left?460:180;
  for(int i=0;i<12;++i)d.Tick(Board(opposite_u),{true,0,1.5,0,1},{false,true,false});
  assert(d.out.mode==GoalMode::kRecoveryTurn && d.core.RecoveryTurnsCompleted()==0);
  // Reappearance with invalid Depth while moving cannot interrupt this turn.
  d.Tick({}, {},{false,true,false},true,true);
  assert(d.out.mode==GoalMode::kRecoveryTurn);
  d.Tick({}, {},{true,false,false});
  assert(d.out.mode==GoalMode::kRecoveryTurn);
  // Reacquired on the final allowed search turn: extra turn is still required.
  d.Window(true,{false,false,true},opposite_u);
  assert(d.out.mode==GoalMode::kRecoveryExtraTurn && d.core.RecoveryTurnsCompleted()==1);
  assert(d.out.action.action==MissionAction::kNone);
  d.Tick({}, {},{},false);assert(d.out.action.action==action);
  d.Tick();assert(d.out.action.action==action); // original direction, not opposite_u
  for(int i=0;i<12;++i)d.Tick(Board(320),{true,0,1.5,0,1},{false,true,false});
  assert(d.out.mode==GoalMode::kRecoveryExtraTurn);
  d.Tick({}, {},{true,false,false});
  assert(d.out.mode==GoalMode::kRecoveryExtraTurn);
  d.Window(found_after_extra,{false,false,true});
  assert(d.out.mode==(found_after_extra?Driver::Resume(phase):Driver::Stationary(phase)));
  assert(d.out.action.action==MissionAction::kNone);
  assert(d.core.RecoveryTurnsCompleted()==(found_after_extra?0:1));
}
void UnknownAndImageOnly() {
  auto c=Configuration();c.entry_line_retry_max_attempts=0;
  Driver d(c);d.core.StartGoalTrial();d.Tick();d.Window(false,{false,false,true});
  assert(d.out.mode==GoalMode::kRecoverySearch);
  for(int phase:{0,1,2}) {
    for(double u:{180.,320.}) {
      Driver before_motion;before_motion.BeginLoss(phase,u);
      before_motion.Tick({}, {},{},true,true);
      assert(before_motion.out.mode==Driver::Stationary(phase));
      assert(before_motion.out.action.action==MissionAction::kNone);
    }
    Driver visible;visible.BeginLoss(phase,320);
    // Restart with a valid board, then keep the board visible but lose pose.
    if(phase==2)visible.core.StartTestExecute();
    else {visible.core.StartGoalTrial();visible.Tick();}
    const GoalPoseObservation pose{true,0,phase==0?1.5:.6,0,1};
    visible.core.UseVerifiedObservation(Board(320),pose,std::deque<bool>(10,true),
        std::deque<bool>(10,true),640,480);
    visible.Tick(Board(320),pose,{false,false,true},false);
    if(phase==1) {
      visible.Tick(Board(320),pose);
      visible.Tick(Board(320),pose,{true,false,false});
    }
    for(int i=0;i<c.lost_frames;++i)visible.Tick({}, {},{},false,true);
    assert(visible.out.mode==Driver::Stationary(phase));
    assert(visible.out.depth_recovery_active);
    assert(visible.out.action.action==(phase!=0 ? MissionAction::kStepForwardHalf : MissionAction::kStepForwardOne));
  }
  // Reset removes direction and any pending motion/extra-turn state.
  Driver reset(c);reset.BeginLoss(0,180);reset.Tick();
  reset.Window(true,{true,false,true});
  assert(reset.out.mode==GoalMode::kRecoveryExtraTurn);
  reset.core.StartGoalTrial();reset.Tick();reset.Window(false,{false,false,true});
  assert(reset.out.mode==GoalMode::kRecoverySearch && reset.core.RecoveryTurnsCompleted()==0);
}
void StationaryReacquisitionResetsExhaustedTurns() {
  Driver d;d.BeginLoss(0,180);d.Tick();
  d.Tick({}, {},{true,false,false});d.Window(false,{false,false,true});
  assert(d.out.mode==GoalMode::kRecoverySearch && d.core.RecoveryTurnsCompleted()==1);
  d.Tick(Board(180),{true,0,1.5,0,1});
  assert(d.core.RecoveryTurnsCompleted()==1); // A single sighting is not recovery.
  for(int i=0;i<12 && d.out.mode==GoalMode::kRecoverySearch;++i)
    d.Tick(Board(180),{true,0,1.5,0,1},{},false);
  assert(d.out.mode==GoalMode::kSearch && d.core.RecoveryTurnsCompleted()==0);
  d.Tick(Board(180),{true,0,1.5,0,1},{},false);
  assert(d.out.mode==GoalMode::kApproach);
  for(int i=0;i<d.config.lost_frames;++i)d.Tick({}, {},{},false);
  // A fresh loss has the full turn allowance, not exhausted stationary-only search.
  assert(d.out.mode==GoalMode::kRecoveryTurn && d.core.RecoveryTurnsCompleted()==0);
}
}
int main() {
  for(int phase:{0,1,2}) {
    for(bool found:{false,true})for(double offset:{-10.,0.,10.})Center(phase,found,offset);
    for(bool left:{false,true})for(bool found:{false,true})ExtraTurn(phase,left,found);
  }
  UnknownAndImageOnly();
  StationaryReacquisitionResetsExhaustedTurns();
}
