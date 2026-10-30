#include "vision_core/config_loader.hpp"
#include <stdexcept>

using namespace vision_core;
namespace {
void Check(bool ok,const char* text) { if(!ok) throw std::runtime_error(text); }
ObjectTarget Board() {
  ObjectTarget t;t.center_px={320,150};t.box_px={300,130,40,40};
  t.width_px=t.height_px=40;t.area_px=1600;t.confidence=1;return t;
}
GoalConfig Config() {
  auto c=LoadDefaultAlgorithmConfig().goal;
  c.stable_window=10;c.stable_min_hits=7;c.lost_frames=3;
  c.depth_recovery_max_steps=3;c.recovery_observation_sec=1;
  return c;
}
struct Driver {
  GoalController core{Config(),30};
  GoalResult out;
  double now{};
  bool image_only{};
  int phase{};
  explicit Driver(int p,bool image):image_only(image),phase(p) {
    const GoalPoseObservation pose{true,0,p==1?.6:1.5,0,1};
    const std::deque<bool> hits(10,true);
    if(p==2) {
      core.StartTestExecute();core.UseVerifiedObservation(Board(),pose,hits,hits,640,480);
    } else core.EnterContinuousApproach(Board(),pose,hits,hits,
        std::deque<bool>(10,p==1),640,480,0,false);
    Tick(true,{},false);
    if(p==1) {
      Check(out.mode==GoalMode::kWaitFinePose,"near GOAL did not request immediate FINE feet");
      Tick(true);Tick(true,{true,false,false});
      for(int i=0;i<10;++i)Tick(true,{},false);
    }
    Check(out.mode==Resume(),"Depth recovery fixture did not reach approach/COARSE/FINE");
    for(int i=0;i<3;++i)Tick(false,{},false);
    Check(out.depth_recovery_active,"visible board coordinate loss did not start bounded forward recovery");
  }
  GoalMode Resume() const { return phase==0 ? GoalMode::kApproach : GoalMode::kFineAdjust; }
  MissionAction Action() const { return phase!=0 ? MissionAction::kStepForwardHalf : MissionAction::kStepForwardOne; }
  void Tick(bool usable,ActionExecutionFeedback fb={},bool allow=true,bool visible=true) {
    now+=.1;
    const auto board=visible && (usable || !image_only) ? std::optional<ObjectTarget>(Board()) : std::nullopt;
    const GoalPoseObservation pose=usable ? GoalPoseObservation{true,0,phase==1?.6:1.5,0,1} : GoalPoseObservation{};
    out=core.Compute({},board,pose,640,480,now,true,{CameraMode::kGoal,true},fb,allow,
        usable ? std::optional<double>(pose.z_m) : std::nullopt,false,visible && image_only && !usable);
  }
};
void Bounded(int phase,bool image_only,bool found) {
  Driver d(phase,image_only);
  for(int n=1;n<=3;++n) {
    d.Tick(false,{true,false,true},false);
    Check(d.core.DepthRecoverySteps()==n-1 && d.out.action.action==d.Action(),
          "unissued preview consumed Depth recovery budget or selected wrong stance");
    d.Tick(false);
    Check(d.out.action.action==d.Action(),"Depth recovery did not issue the next forward action");
    for(int i=0;i<12;++i)d.Tick(true,{false,true,false});
    Check(d.core.DepthRecoveryActive() && d.core.DepthRecoverySteps()==n-1,
          "moving valid coordinates bypassed actual DONE/stationary verification");
    d.Tick(false,{true,false,false});
    Check(d.core.DepthRecoverySteps()==n,"forward DONE was not counted exactly once");
    d.Tick(false,{true,false,false});
    Check(d.core.DepthRecoverySteps()==n,"duplicate DONE consumed Depth recovery budget");
    for(int i=0;i<12 && d.core.DepthRecoveryActive();++i)d.Tick(found,{},false);
    if(found) {
      Check(!d.core.DepthRecoveryActive() && d.out.mode==d.Resume(),
            "fresh stationary coordinates did not resume the previous approach phase");
      return;
    }
  }
  Check(d.out.mode==GoalMode::kFailed && d.core.DepthRecoverySteps()==3,
        "Depth loss did not fail after the third completed forward action and observation");
  for(int i=0;i<20;++i) {
    d.Tick(true);Check(d.out.mode==GoalMode::kFailed && d.out.action.action==MissionAction::kNone,
                      "failed Depth recovery resumed without Reset");
  }
  d.core.Reset();Check(!d.core.DepthRecoveryActive() && !d.core.DepthRecoverySteps(),
                       "Reset retained Depth recovery state");
}
void ImageDisappears() {
  Driver d(2,true);d.Tick(false);
  d.Tick(false,{true,false,false},true,false);
  for(int i=0;i<12 && d.core.DepthRecoveryActive();++i)d.Tick(false,{},false,false);
  Check(!d.core.DepthRecoveryActive() && d.out.mode!=GoalMode::kFailed &&
      d.out.action.action==MissionAction::kNone,"missing board continued Depth-only forward search");
}
void NearDuringFarDepthRecovery() {
  Driver d(0,false);
  d.Tick(false);
  Check(d.out.action.action==MissionAction::kStepForwardOne,
        "far Depth recovery fixture did not issue its WALK");
  const auto tick=[&](ActionExecutionFeedback feedback) {
    d.now+=.1;
    d.out=d.core.Compute({},Board(),{},640,480,d.now,true,
        {CameraMode::kGoal,true},feedback,true,.6);
  };
  for(int i=0;i<12;++i) {
    tick({false,true,false});
    Check(d.out.action.action==MissionAction::kNone && d.core.DepthRecoveryActive(),
          "near sighting interrupted an active far recovery WALK");
  }
  tick({true,false,false});
  Check(d.out.mode==GoalMode::kWaitFinePose &&
        d.out.action.action==MissionAction::kDefaultPoseMode && !d.core.DepthRecoveryActive(),
        "near board with invalid pose did not replace further WALK recovery with FINE feet after DONE");
  tick({false,true,false});
  Check(d.out.mode==GoalMode::kWaitFinePose,"active feet transition completed without DONE");
  tick({true,false,false});
  Check(d.out.mode==GoalMode::kFineAdjust,"actual feet DONE did not enter FINE");
}
}
int main() {
  for(int phase:{0,1,2})for(bool image:{false,true})for(bool found:{false,true})Bounded(phase,image,found);
  ImageDisappears();
  NearDuringFarDepthRecovery();
}
