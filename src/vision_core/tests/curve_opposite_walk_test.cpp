#include "vision_core/config_loader.hpp"
#include "vision_core/mission_controller.hpp"
#include <cmath>
#include <filesystem>
#include <stdexcept>

using namespace vision_core;
namespace {
void Check(bool value,const char* message) {
  if(!value) throw std::runtime_error(message);
}
MissionControllerConfig Config(bool enabled=true,int repeats=1) {
  auto c=LoadDefaultAlgorithmConfig();
  c.enable_ball=true;c.enable_hurdle=c.enable_goal=false;
  c.curve_search.enabled=true;c.curve_search.opposite_walk_before_scan=enabled;
  c.curve_search.confirm_extra_sequences=1;c.curve_search.scan_delay_walk_sequences=0;
  c.line_p2p.left_repetitions=c.line_p2p.straight_repetitions=c.line_p2p.right_repetitions=repeats;
  return c;
}
struct Driver {
  MissionController core;
  MissionFrameInput in;
  MissionFrameResult out;
  int repeats;
  Driver(MissionControllerConfig c,int direction):core(c),repeats(c.line_p2p.straight_repetitions) {
    Check(core.SetTestMissionIndices(direction>0 ? 1 : 2,1),"course seed failed");
    in.image_width=640;in.image_height=480;in.intrinsics={600,600,320,240};
    in.command_transport_enabled=true;in.camera_feedback={CameraMode::kForward,true};
    in.line_centers={{380,450},{380,400},{380,350},{380,300},{380+direction*50.0,250}};
    Tick();
  }
  void Tick(CommandDeliveryFeedback fb={}) {
    in.now_sec+=.07;in.delivery_feedback=fb;out=core.Step(in);
  }
  void Block(bool confirming=false,bool defer_issue=false) {
    for(int unit=0;unit<repeats;++unit) {
      for(int i=0;i<30 && !out.command.action_id;++i) Tick();
      const auto id=out.command.action_id;
      Check(id && out.command.action==MissionAction::kLineStraightUnit,"normal LINE unit missing");
      Tick({id,true,false,false});
      for(int i=0;i<12;++i) Tick();
      if(confirming && unit==repeats-1) {
        Tick({id,true,false,true});
        Check(out.command.action_id==0 || out.command.action_id==id,"CONFIRM READY queued another unit");
        Check(out.command.camera_request==CameraRequest::kNone,"scan started before confirmation DONE");
        if(defer_issue) in.allow_new_body_actions=false;
      }
      Tick({id,true,true,false});
    }
  }
  void Confirm(bool defer_issue=false) {
    Block();Check(out.curve_search_phase==1,"first curve did not enter CONFIRM");
    Block(true,defer_issue);
  }
  void FailedScan() {
    Check(out.curve_search_phase==4,"head scan missing");
    in.camera_feedback={CameraMode::kTransition,false};in.camera_scan.phase=1;Tick();
    in.camera_scan.phase=0;in.camera_scan.finished=true;Tick();
    Check(out.curve_search_phase==7,"scan miss did not request return");
    in.camera_feedback={CameraMode::kForward,true};in.camera_scan.finished=false;Tick();
  }
};
void OneUnitAndRealDone(int direction,int right_action,int repeats) {
  auto c=Config(true,repeats);c.line_p2p.right_unit_action=right_action;
  Driver d(c,direction);d.Confirm();
  const auto expected=direction>0 ? MissionAction::kLineLeftUnit : static_cast<MissionAction>(right_action);
  const auto scan=direction>0 ? CameraRequest::kScanLeft : CameraRequest::kScanRight;
  Check(d.out.curve_search_phase==13 && d.out.curve_head_attempts==0,"curve did not schedule opposite walk first");
  Check(d.out.command.action==expected && d.out.command.action_id,"opposite direction incorrect");
  const auto id=d.out.command.action_id;
  d.Tick({id,true,false,false});
  for(int i=0;i<12;++i) {
    d.Tick();Check(d.out.curve_search_phase==13,"ACK/frame consumed opposite walk");
    Check(d.out.command.camera_request==CameraRequest::kNone,"scan overlapped opposite walk");
  }
  d.Tick({id+100,true,true,false});
  Check(d.out.curve_search_phase==13,"stale DONE consumed opposite walk");
  d.Tick({id,true,false,true});
  Check(d.out.curve_search_phase==13,"READY consumed opposite walk");
  Check(d.out.command.action_id==0 || d.out.command.action_id==id,"READY issued another walk");
  d.Tick({id,true,true,false});
  Check(d.out.curve_search_phase==4 && d.out.curve_head_attempts==1,"real DONE did not resume head scan");
  Check(d.out.command.camera_request==scan,"scan side changed");
  d.Tick({id,true,true,false});
  Check(d.out.curve_search_phase==4 && d.out.curve_head_attempts==1,"duplicate DONE restarted curve sequence");
  // A missed head scan retains the existing normal LINE retry probe.
  d.FailedScan();Check(d.out.curve_search_phase==8,"scan retry did not request normal LINE probe");
  for(int i=0;i<60 && !d.out.command.action_id;++i) d.Tick();
  Check(d.out.command.action==MissionAction::kLineStraightUnit,"scan retry repeated opposite walk");
  const auto probe=d.out.command.action_id;
  Check(probe && probe!=id,"retry probe ID missing");
  d.Tick({probe,true,false,false});
  for(int i=0;i<12;++i) d.Tick();
  d.Tick({probe,true,true,false});
  for(int i=0;i<5 && d.out.curve_search_phase!=4;++i) d.Tick();
  Check(d.out.curve_search_phase==4 && d.out.curve_head_attempts==2,"retry probe DONE inserted another opposite walk");
}
ObjectTarget Ball() {
  ObjectTarget t;t.class_id=1;t.confidence=.95;t.center_px={320,100};
  t.width_px=t.height_px=40;t.area_px=1600;return t;
}
void CameraAndBodyGate() {
  Driver d(Config(),1);d.Confirm(true);
  Check(!d.out.command.action_id && d.out.command.camera_request==CameraRequest::kNone,"body gate ignored");
  d.in.allow_new_body_actions=true;d.in.camera_feedback={CameraMode::kTransition,false};
  d.Tick();Check(!d.out.command.action_id && d.out.command.camera_request==CameraRequest::kNone,"unsettled camera allowed walk/scan");
  d.in.camera_feedback={CameraMode::kDown,true};
  d.Tick();Check(!d.out.command.action_id && d.out.command.camera_request==CameraRequest::kNone,"wrong camera view allowed walk/scan");
  d.in.camera_feedback={CameraMode::kForward,true};d.Tick();
  Check(d.out.command.action==MissionAction::kLineLeftUnit && d.out.command.action_id,"camera settle did not release opposite walk");
  d.core.Reset();d.Tick();
  Check(d.out.curve_search_phase==0 && d.out.command.action!=MissionAction::kLineLeftUnit,"Reset retained opposite walk state");
}
void BallPriority(bool after_walk) {
  auto c=Config();Driver d(c,1);d.Confirm(!after_walk);
  const auto id=d.out.command.action_id;
  if(after_walk) d.Tick({id,true,false,false});
  d.in.ball_target=Ball();d.in.ball_track_id=41;d.in.ball_entry_track_hits=c.ball.entry_min_hits;
  if(after_walk) {
    d.Tick();Check(d.out.curve_search_phase==13,"BALL cancelled issued opposite walk");
    d.Tick({id,true,true,false});
  } else {d.in.allow_new_body_actions=true;d.Tick();}
  Check(d.out.approach_mission==MissionType::kBall,"qualified BALL did not win before scan");
  Check(d.out.curve_search_phase==0 && d.out.command.camera_request==CameraRequest::kNone,"BALL admission started curve scan");
}
void ConfigCompatibility() {
  Check(!LoadDefaultAlgorithmConfig().curve_search.opposite_walk_before_scan,"legacy fixture enabled opposite walk");
  const auto path=std::filesystem::path(__FILE__).parent_path().parent_path()/"config/vision_algorithm.yaml";
  const auto p=LoadAlgorithmConfig(path.string());
  Check(p.curve_search.opposite_walk_before_scan,"production ULTRAFAST option missing");
  Check(std::abs(p.line_p2p.offset_gain-.2)<1e-12 && std::abs(p.line_p2p.heading_gain-.4)<1e-12,"ULTRAFAST normal gains incorrect");
  Driver legacy(Config(false),1);legacy.Confirm();
  Check(legacy.out.curve_search_phase==4 && legacy.out.command.camera_request==CameraRequest::kScanLeft,"disabled option changed existing scan behavior");
}
}
int main() {
  ConfigCompatibility();
  OneUnitAndRealDone(1,36,1);
  OneUnitAndRealDone(-1,36,1);
  OneUnitAndRealDone(-1,34,1);
  OneUnitAndRealDone(1,36,3);
  CameraAndBodyGate();BallPriority(false);BallPriority(true);
}
