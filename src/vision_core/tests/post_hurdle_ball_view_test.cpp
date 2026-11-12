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


#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <unistd.h>
using namespace vision_core;
namespace {
void Check(bool value,const char* message) { if(!value) throw std::runtime_error(message); }
ObjectTarget Target(double u=320,double v=100,int cls=1) {
  ObjectTarget t;t.class_id=cls;t.confidence=.95;t.center_px={u,v};
  t.box_px={u-20,v-20,40,40};t.width_px=t.height_px=40;t.area_px=1600;return t;
}
MissionControllerConfig Config() {
  auto c=LoadDefaultAlgorithmConfig();
  const auto production=LoadAlgorithmConfig((std::filesystem::path(__FILE__).parent_path().parent_path()/"config/vision_algorithm.yaml").string());
  Check(std::abs(production.goal.approach_short_z_m-0.87)<1e-9,
        "production GOAL near threshold must be 0.87m");
  Check(std::abs(production.goal.camera_goal_start_z_m-0.87)<1e-9 &&
        std::abs(production.goal.camera_goal_start_z_m-production.goal.approach_short_z_m)<1e-9,
        "GOAL camera transition and near thresholds must both be 0.87m");
  c.enable_ball=true;c.enable_goal=c.enable_hurdle=false;
  c.initial_has_ball=c.goal_trial_start_goal=false;
  c.test_mission=MissionType::kNone;
  c.line_features.image_center_u=320;
  c.goal.auto_approach_center_u_px=280; // Dedicated GOAL vertical zone center.
  c.line_p2p.left_repetitions=c.line_p2p.straight_repetitions=c.line_p2p.right_repetitions=2;
  c.ball.entry_min_hits=4;c.ball.stable_window=10;c.ball.stable_min_hits=7;
  c.ball.approach_short_v_px=100;c.ball.approach_two_step_v_px=240;c.ball.approach_direct_down_v_px=355;
  c.hurdle.approach_short_v_px=100;c.hurdle.approach_two_step_v_px=240;
  c.object_association.max_center_distance_norm=.5;
  // This fixture changes the BALL center from v=100 to v=260 in one update.
  // Its test YAML intentionally has an 80px candidate gate; give this
  // direct-controller fixture the same wider gate the old association used.
  c.ball_candidate_tracking.max_center_distance_px=400.0;
  c.ball_candidate_tracking.max_size_log_ratio=c.object_association.max_size_log_ratio;
  c.object_approach.lock_blocks=3;
  c.race_round2_fast_enabled=true;
  c.post_hurdle_ball_view={true,1452,300,430};
  c.transition_settle_sec=0;c.curve_search.enabled=false;
  return c;
}
struct Driver {
  MissionController core;
  MissionFrameInput in;
  MissionFrameResult out;
  std::uint64_t current{}, last_done{};
  explicit Driver(const MissionControllerConfig& c):core(c) {
    core.post_hurdle_ball_view_active_=core.post_hurdle_forward_seen_=true;
    in.image_width=640;in.image_height=480;in.intrinsics={600,600,320,240};
    in.camera_feedback={CameraMode::kForward,true};in.command_transport_enabled=true;
    in.line_centers={{320,450},{320,400},{320,350},{320,300},{320,250}};
    in.ball_target=Target();in.ball_track_id=41;
  }
  void Tick(CommandDeliveryFeedback fb={},double dt=.05) {
    in.now_sec+=dt;in.delivery_feedback=fb;out=core.Step(in);
  }
  void Warm() {
    Tick();current=out.command.action_id;
    Check(current!=0,"initial LINE unit missing");
    Tick({current,true,false,false});
    for(int i=0;i<12;++i) Tick();
  }
  void Block() {
    for(int n=0;n<2;++n) {
      Tick({current,true,false,false});
      for(int i=0;i<12;++i) Tick();
      Tick({current,true,false,true});
      const auto next=out.command.action_id;
      Check(next && next!=current && LinePlan::IsUnit(out.command.action),
            "continuous READY did not reserve the next shared unit");
      Tick({next,true,false,false});
      last_done=current;
      Tick({current,true,true,false});current=next;
    }
  }
  void MissingBlock() {
    in.ball_target.reset();
    for(int i=0;i<12;++i) Tick();
    Check(out.approach_mission==MissionType::kBall,
          "one missing frame released an object before block READY");
    Tick({current,true,false,true});
    const auto second=out.command.action_id;
    Check(second && second!=current && LinePlan::IsUnit(out.command.action),
          "missing first unit incorrectly cancelled the rest of its 2-unit block");
    Tick({second,true,false,false});
    Tick({current,true,true,false});current=second;
    for(int i=0;i<12;++i) Tick();
    Tick({current,true,false,true});
    // An unqualified BALL block may now reserve exactly one remembered-direction
    // pre-lock probe at final READY. Qualification must remain false.
    Check(!out.approach_sequence_qualified,
          "unqualified object block was incorrectly marked qualified");
    Tick({current,true,true,false});
  }
};
void RaisedNearFinalActionSequence(double u,bool left,bool extreme,double near_v=260) {
  auto c=Config();c.ball.approach_near_outer_margin_px=90.0;
  Driver d(c);
  // Keep center-distance association stable while entering the lower zone:
  // the test changes v by <=200px, never requiring an identity reset.
  d.in.ball_target=Target(u,near_v>=c.ball.approach_direct_down_v_px ? 200 : 100);
  d.Warm();d.Block();d.Block();
  d.in.ball_target=Target(u,near_v);
  for(int n=0;n<3;++n)d.Tick();
  Check(d.out.approach_near,"near sequence failed to latch three v votes");
  d.Tick({d.current,true,false,true});
  d.Tick({d.current,true,true,false});
  for(int n=0;n<35 && d.out.command.action_id==0;++n)d.Tick();
  const MissionAction turn=left ? MissionAction::kTurnLeftInPlace :
      MissionAction::kStepForwardRightOneHurdle;
  if (d.out.command.action!=turn || !d.out.approach_near_final_motion)
    throw std::runtime_error("near final first: action="+
        std::to_string(static_cast<int>(d.out.command.action))+
        " expected="+std::to_string(static_cast<int>(turn))+
        " near_motion="+std::to_string(d.out.approach_near_final_motion)+
        " near_observing="+std::to_string(d.out.approach_near_observing)+
        " mission="+std::to_string(static_cast<int>(d.out.active_mission))+
        " id="+std::to_string(d.out.command.action_id));
  std::uint64_t id=d.out.command.action_id;
  d.Tick({id,true,false,false});
  d.Tick({id,true,true,false});
  if (extreme) {
    Check(d.out.command.action==turn && d.out.command.action_id!=id,
          "extreme zone did not request second turn after first DONE");
    id=d.out.command.action_id;
    d.Tick({id,true,false,false});
    d.Tick({id,true,true,false});
  }
  // A committed near chain must now move forward without an intermediate
  // observation after the 1/2 rotation actions.
  const auto expected_step=near_v<c.post_hurdle_ball_view.one_step_v_px ?
      MissionAction::kStepForwardTwo : MissionAction::kStepForwardOne;
  Check(d.out.command.action==expected_step &&
        d.out.command.action_id!=id && d.out.approach_near_final_motion,
        "near side grid did not select 1/2 forward steps from the shared v boundary");
  id=d.out.command.action_id;
  d.Tick({id,true,false,false});
  Check(d.out.active_mission==MissionType::kLine,
        "BALL near switched camera before committed forward DONE");
  d.Tick({id,true,true,false});
  Check(d.out.active_mission==MissionType::kBall &&
        d.out.command.camera_request==CameraRequest::kBallView,
        "BALL near committed turn + forward chain did not hand off to DOWN");

  d.in.camera_feedback={CameraMode::kBallView,true};d.Tick();
  Check(!d.core.post_hurdle_ball_view_active_,"BALL_VIEW did not end raised profile");
}

void LatchOnlyAfterActualHurdleDone() {
  for(int course:{1,3,5}) for(int feedback:{0,1,2,3}) {
    auto c=Config();c.enable_hurdle=true;
    MissionController core(c);core.EnterMission(MissionType::kHurdle);
    core.course_curve_index_=course;core.foot_pose_=FootPose::kWalk;
    core.last_command_=core.command_coordinator_.Compute(MissionType::kHurdle,0,
        {MissionAction::kHurdle,ActionCategory::kMission,0,false},CameraRequest::kNone,{},0);
    const auto id=core.last_command_.action_id;
    MissionFrameInput in;in.image_width=640;in.image_height=480;in.now_sec=.1;
    in.command_transport_enabled=true;in.allow_new_commands=in.allow_new_body_actions=false;
    in.advance_line_fsm=false;in.camera_feedback={CameraMode::kDown,true};
    in.delivery_feedback={feedback==3 ? id+1 : id,true,feedback>=2,feedback==1};
    core.Step(in);
    Check(core.post_hurdle_ball_view_active_==(feedback==2 && course!=5),
          "raised profile accepted ACK/READY/stale DONE or wrong course");
  }
  auto c=Config();MissionController core(c);
  core.post_hurdle_ball_view_active_=true;
  MissionFrameInput in;in.image_width=640;in.image_height=480;in.now_sec=.1;
  in.allow_new_commands=in.allow_new_body_actions=false;in.advance_line_fsm=false;
  in.camera_feedback={CameraMode::kDown,true};core.Step(in);
  Check(core.post_hurdle_ball_view_active_,"old DOWN feedback cancelled pending raised FORWARD");
  in.camera_feedback={CameraMode::kForward,true};core.Step(in);
  in.camera_feedback={CameraMode::kTransition,false};core.Step(in);
  Check(core.post_hurdle_ball_view_active_,"scan transition cancelled raised profile");
  MissionFrameResult output;output.command.camera_request=CameraRequest::kForward;
  core.PopulateDiagnostics(output,.2);core.ApplyPostHurdleCameraProfile(output);
  Check(output.command.camera_request==CameraRequest::kPostHurdleForward &&
        output.ball_near_v_px==300 && output.ball_one_step_v_px==430,
        "scan restore or debug thresholds ignored raised profile");
  core.Reset();
  Check(!core.post_hurdle_ball_view_active_ && core.BallNearV()==240 && core.BallOneStepV()==355,
        "Reset retained raised camera/thresholds");
}
void RaisedNearVotes() {
  Driver d(Config());d.Warm();d.Block();
  d.in.ball_target=Target(400,250);
  for(int i=0;i<12;++i)d.Tick();
  Check(!d.out.approach_near,"raised view used old 240px near threshold");
  d.in.ball_target=Target(400,300);
  for(int i=0;i<12;++i)d.Tick();
  Check(d.out.approach_near,"raised near threshold was not inclusive at 300px");
}
void ConfigContract() {
  const auto path=std::filesystem::path(__FILE__).parent_path().parent_path()/"config/vision_algorithm.yaml";
  auto production=LoadAlgorithmConfig(path.string());
  Check(production.post_hurdle_ball_view.enabled && production.post_hurdle_ball_view.pitch_tick==1452 &&
        production.post_hurdle_ball_view.near_v_px==240 &&
        production.post_hurdle_ball_view.one_step_v_px==410 && production.ball.pickup_polygon.size()==7,
        "ultrafast YAML defaults/pickup polygon differ from the agreed values");
  for(int field:{0,1,2,3,4}) {
    auto bad=production;
    if(field==0)bad.post_hurdle_ball_view.pitch_tick=-1;
    if(field==1)bad.post_hurdle_ball_view.pitch_tick=4096;
    if(field==2)bad.post_hurdle_ball_view.near_v_px=std::numeric_limits<double>::quiet_NaN();
    if(field==3)bad.post_hurdle_ball_view.one_step_v_px=bad.post_hurdle_ball_view.near_v_px;
    if(field==4)bad.race_round2_fast_enabled=false;
    bool rejected=false;try {ValidateAlgorithmConfig(bad);}catch(const std::runtime_error&){rejected=true;}
    Check(rejected,"invalid raised-view configuration was accepted");
  }
  std::ifstream input(path);const std::string yaml((std::istreambuf_iterator<char>(input)),{});
  const auto temporary=std::filesystem::temp_directory_path()/
      ("jandi-post-hurdle-config-"+std::to_string(getpid())+".yaml");
  for(const auto* key:{"enabled: true # 초FAST", "pitch_tick: 1452", "near_v_px: 240.0", "one_step_v_px: 410.0"}) {
    auto missing=yaml;const auto at=missing.find(key);Check(at!=std::string::npos,"missing source key in test");
    const auto begin=missing.rfind('\n',at);const auto end=missing.find('\n',at);
    missing.erase(begin+1,end-begin);
    {std::ofstream output(temporary);output<<missing;}
    bool rejected=false;try{(void)LoadAlgorithmConfig(temporary.string());}catch(const std::runtime_error&){rejected=true;}
    std::filesystem::remove(temporary);Check(rejected,"missing raised-view field silently defaulted");
  }
  Check(!LoadDefaultAlgorithmConfig().post_hurdle_ball_view.enabled,"legacy fixture unexpectedly raised camera");
}
}
int main() {
  ConfigContract();LatchOnlyAfterActualHurdleDone();RaisedNearVotes();
  // 429/430 split differs from the fixture's old 355 split, for both sides.
  RaisedNearFinalActionSequence(150,true,true,429);
  RaisedNearFinalActionSequence(500,false,false,430);
}
