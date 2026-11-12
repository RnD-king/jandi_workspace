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
using namespace vision_core;
namespace {
constexpr double kPi = 3.14159265358979323846;
void Check(bool value,const char* message) { if(!value) throw std::runtime_error(message); }
std::vector<Point2> Floor(double heading_deg,double center=320) {
  const double slope=-std::tan(heading_deg*kPi/180.0);
  std::vector<Point2> points;
  for(double v : {450.,400.,350.,300.}) points.push_back({center+slope*(v-375),v});
  return points;
}
MissionControllerConfig Config() {
  auto c=LoadDefaultAlgorithmConfig();
  c.enable_ball=c.enable_goal=c.enable_hurdle=false;
  c.initial_has_ball=c.goal_trial_start_goal=false;
  c.curve_search.enabled=false;c.test_mission=MissionType::kNone;
  c.line_features.image_center_u=320;c.line_features.ransac_enabled=false;
  c.line_p2p.offset_gain=1;c.line_p2p.heading_gain=0;
  c.line_p2p.steering_deadband=.01;c.line_p2p.in_place_turn_score=2;
  c.line_p2p.left_repetitions=c.line_p2p.right_repetitions=c.line_p2p.straight_repetitions=1;
  c.transition_settle_sec=c.walk_inter_action_hold_sec=0;
  return c;
}
MissionControllerConfig Production() {
  return LoadAlgorithmConfig((std::filesystem::path(__FILE__).parent_path().parent_path()/
                              "config/vision_algorithm.yaml").string());
}
void References() {
  const auto production=Production();
  Check(production.hurdle.course_line_reference_enabled,"production course references disabled");
  const auto first=production.hurdle.line_reference_u_px;
  const auto second=production.hurdle.second_line_reference_u_px;
  Check(first==370 && second==270,"FAST must use +/-50px course references");
  auto c=Config();c.enable_hurdle=true;c.hurdle.course_line_reference_enabled=true;
  c.hurdle.line_reference_u_px=first;c.hurdle.second_line_reference_u_px=second;
  for(int stage : {1,3}) {
    MissionController core(c);core.course_curve_index_=stage;
    core.ball_mission_index_=stage==1 ? 0 : 1;
    MissionFrameInput in;in.image_width=640;in.image_height=480;in.now_sec=.1;
    in.command_transport_enabled=true;in.camera_feedback={CameraMode::kForward,true};
    in.line_centers=Floor(0);
    const auto expected=stage==1 ? first : second;
    auto out=core.Step(in);
    Check(out.line_reference_u_px && *out.line_reference_u_px==expected,"forward LINE ignored course reference");
    Check(std::abs(out.line_features.guide.offset-(320-expected)/320)<1e-9,"course offset sign is wrong");
    // The physical crossing count may advance before FORWARD camera return.
    // The upcoming BALL stage, not that count alone, owns the reference.
    core.EnterMission(MissionType::kHurdle);
    Check(core.hurdle_controller_.FineLineReferenceU()==expected,"DOWN and far references differ");
    ++core.hurdle_crosses_completed_;
    Check(core.HurdleLineReferenceU()==expected,"crossing DONE changed the current reference");
    core.has_ball_=true;core.active_mission_=MissionType::kLine;
    Check(core.EffectiveLineReferenceU(in,380)==380,"hurdle reference leaked into carrying LINE");
    core.has_ball_=false;core.hurdle_crosses_completed_=2;core.course_curve_index_=5;
    Check(core.EffectiveLineReferenceU(in,380)==380,"hurdle reference leaked into FINISH");
  }
}
void LegacyVotes(int window,int required,int hits,bool pass,double heading=20) {
  auto c=Config();c.line.line_stable_window=window;c.line.line_stable_min_hits=required;
  MissionController core(c);core.active_mission_=MissionType::kBall;
  core.mission_recovery_verifying_=true;core.mission_recovery_verify_start_sec_=0;
  MissionFrameInput in;in.image_width=640;in.image_height=480;
  in.camera_feedback={CameraMode::kForward,true};
  bool valid=false;MissionFrameResult out;
  for(int i=0;i<window;++i) {
    in.now_sec=(i+1.0)/window;
    in.line_centers=i<hits ? Floor(heading,300) : std::vector<Point2>{};
    core.ComputeMissionLineRecovery(in,{},&valid,&out);
    if(i+1<window) Check(!valid,"legacy verification finished before observation interval");
  }
  Check(valid==pass,"legacy LINE guide.valid votes were not enforced");
  if(pass) {
    Check(core.line_state_==MissionController::LineState::kNormal,
          "legacy LINE return did not resume normal walking");
    Check(!core.mission_recovery_line_guide_,
          "legacy LINE return unexpectedly synthesized a next-walk O/H mean");
  }
}
void LegacyTurnDoneAndReset() {
  auto c=Config();MissionController core(c);core.active_mission_=MissionType::kGoal;
  MissionFrameInput in;in.image_width=640;in.image_height=480;in.now_sec=.2;
  in.camera_feedback={CameraMode::kForward,true};in.line_centers=Floor(45);
  bool valid=false;MissionFrameResult out;
  core.mission_recovery_issued_=true;core.last_command_.action=MissionAction::kTurnLeft;
  ActionExecutionFeedback done;done.action_done=true;
  core.ComputeMissionLineRecovery(in,done,&valid,&out);
  Check(core.mission_recovery_turns_==1,"legacy turn DONE did not count exactly once");
  core.ComputeMissionLineRecovery(in,done,&valid,&out);
  Check(core.mission_recovery_turns_==1,"duplicate DONE counted a legacy turn twice");
  core.Reset(true);
  Check(core.mission_recovery_turns_==0 && core.mission_recovery_hits_.empty() &&
        !core.mission_recovery_line_guide_,"RESET retained old recovery state");
}

}
int main() {
  References();
  LegacyVotes(10,5,5,true);LegacyVotes(10,5,4,false);
  LegacyVotes(8,4,4,true);LegacyVotes(8,4,3,false);
  LegacyVotes(8,4,4,true,45); // Old FAST deliberately has no +/-30-degree gate.
  LegacyVotes(8,4,0,false);
  LegacyTurnDoneAndReset();
}
