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

using namespace vision_core;
namespace {
void Check(bool value,const char* message) { if(!value) throw std::runtime_error(message); }
MissionControllerConfig Config() {
  auto c=LoadDefaultAlgorithmConfig();
  c.enable_ball=c.enable_goal=false;c.enable_hurdle=true;
  c.initial_has_ball=c.goal_trial_start_goal=false;c.test_mission=MissionType::kNone;
  c.curve_search.enabled=false;
  c.line_features.image_center_u=c.hurdle.target_u_px=320;
  c.line_p2p.offset_gain=1;c.line_p2p.heading_gain=0;
  c.line_p2p.steering_deadband=.1;c.line_p2p.in_place_turn_score=1;
  c.line_p2p.left_repetitions=c.line_p2p.right_repetitions=c.line_p2p.straight_repetitions=2;
  c.transition_settle_sec=0;c.hurdle.contact_steps=1;
  c.hurdle.contact_inter_action_hold_sec=0;
  return c;
}
ObjectTarget Finish(double u=450) {
  ObjectTarget t;t.center_px={u,400};t.box_px={u-50,390,100,20};
  t.width_px=100;t.height_px=20;t.area_px=2000;t.confidence=.95;return t;
}
struct Driver {
  MissionController core;
  MissionFrameInput in;
  MissionFrameResult out;
  explicit Driver(int crosses=2,int index=5):core(Config()) {
    core.hurdle_crosses_completed_=crosses;core.course_curve_index_=index;
    in.image_width=640;in.image_height=480;in.command_transport_enabled=true;
    in.camera_feedback={CameraMode::kForward,true};
  }
  void Tick(CommandDeliveryFeedback feedback={},double dt=.1) {
    in.now_sec+=dt;in.delivery_feedback=feedback;out=core.Step(in);
    Check(out.command.action!=MissionAction::kHurdle,"finish emitted a crossing action");
    if(core.FinishCourseEnabled(in) && in.camera_feedback.settled &&
        in.camera_feedback.actual_mode==CameraMode::kForward)
      Check(out.command.camera_request==CameraRequest::kNone,"finish started a recovery head scan");
  }
  void Floor(double u,int count=5) {
    in.line_centers.clear();
    for(int i=0;i<count;++i)in.line_centers.push_back({u,450.0-40*i});
  }
};
void PriorityAndGates() {
  Driver floor;floor.Floor(200);floor.in.hurdle_target=Finish(450);floor.Tick();
  Check(floor.out.command.action==MissionAction::kLineLeftUnit,"valid LINE did not win over finish bbox");
  for(int points:{0,1,2,3}) {
    Driver bbox;bbox.Floor(200,points);bbox.in.hurdle_target=Finish(450);bbox.Tick();
    Check(bbox.out.command.action==static_cast<MissionAction>(Config().line_p2p.right_unit_action),
          "sparse LINE did not use finish center");
    Check(!bbox.out.line_features.guide.valid && bbox.out.line_decision_guide &&
          bbox.out.line_decision_guide->valid,
          "virtual finish guide was reported as a real floor fit");
    Check(bbox.out.active_mission==MissionType::kLine && bbox.out.approach_mission==MissionType::kNone,
          "finish entered HURDLE/FINE instead of retaining LINE transport");
  }
  Driver missing;missing.Tick();
  Check(missing.out.command.action==MissionAction::kLineStraightUnit,"missing LINE and bbox did not go straight");
  Driver outlier;outlier.Floor(200,4);outlier.in.line_centers.back().u=600;
  outlier.core.config_.line_features.ransac_enabled=true;
  outlier.core.config_.line_features.ransac_min_input_points=4;
  outlier.core.config_.line_features.ransac_residual_threshold_px=20;
  outlier.core.config_.line_features.ransac_min_sample_v_gap_px=20;
  outlier.in.hurdle_target=Finish(450);outlier.Tick();
  Check(outlier.out.line_features.n_visible==3 &&
        outlier.out.command.action==static_cast<MissionAction>(Config().line_p2p.right_unit_action),
        "three RANSAC inliers incorrectly kept floor steering priority");
  Driver bad;bad.in.hurdle_target=Finish(std::numeric_limits<double>::quiet_NaN());bad.Tick();
  Check(bad.out.command.action==MissionAction::kLineStraightUnit,"invalid finish center was used");
  Driver score;score.core.config_.line_p2p.in_place_turn_score=.01;
  // Recreate the steering controller as a real matching configuration would.
  score.core.line_p2p_controller_=LineP2pController(score.core.config_.line_p2p);
  score.Floor(600);score.Tick();
  Check(score.out.command.action==MissionAction::kLineStraightUnit,"finish started score recovery instead of straight");
  for(const auto stage:std::vector<std::pair<int,int>>{{0,5},{1,5},{2,1},{2,3},{2,4}}) {
    Driver normal(stage.first,stage.second);normal.Floor(200,3);normal.Tick();
    Check(!normal.core.FinishCourseEnabled(normal.in),"partial finish gate was accepted");
    Check(normal.out.command.action_id==0 && normal.out.line_in_recovery,
          "ordinary LINE loss became blind forward walking");
  }
  Driver manual;manual.in.manual_observation_control=true;manual.Tick();
  Check(!manual.core.FinishCourseEnabled(manual.in) && manual.out.command.action_id==0,
        "finish exception leaked into manual mode");
  Driver isolated;isolated.core.config_.test_mission=MissionType::kHurdle;
  Check(!isolated.core.FinishCourseEnabled(isolated.in),"finish exception leaked into standalone HURDLE");
  Driver camera;camera.in.camera_feedback={CameraMode::kTransition,false};camera.Tick();
  Check(camera.out.command.action_id==0,"finish moved during camera transition");
  Driver settled_down;settled_down.in.camera_feedback={CameraMode::kDown,true};settled_down.Tick();
  Check(settled_down.out.command.action_id==0,"finish used a DOWN camera observation");
}
void ActivateRealCooldown(Driver& d) {
  auto& h=d.core.hurdle_controller_;
  h.StartTestExecute();
  const auto target=Finish(320);
  h.UseVerifiedObservation(target,std::deque<bool>(10,true),640,480);
  const CameraFeedback down{CameraMode::kDown,true};
  auto r=h.Compute(target,640,480,.1,down,{},{});
  Check(r.mode==HurdleMode::kContactWalk,"cooldown setup did not enter contact walk");
  ActionExecutionFeedback done;done.action_done=true;
  r=h.Compute(target,640,480,.2,down,done,{});
  Check(r.action.action==MissionAction::kHurdle,"cooldown setup did not cross");
  h.Compute(target,640,480,.3,down,done,{});
  h.Compute({},640,480,.4,{CameraMode::kForward,true},{},{});
  Check(h.IgnoringUntil(.5),"cooldown fixture did not start real post-return ignore timer");
  d.in.now_sec=.4;
}
void CooldownScope() {
  Driver second;ActivateRealCooldown(second);second.in.hurdle_target=Finish(450);second.Tick();
  Check(second.out.command.action==static_cast<MissionAction>(Config().line_p2p.right_unit_action),
        "second crossing cooldown hid finish center");
  Driver first(1,5);ActivateRealCooldown(first);first.Floor(320);first.in.hurdle_target=Finish(450);
  for(int i=0;i<15;++i)first.Tick();
  Check(first.core.hurdle_controller_.IgnoringUntil(first.in.now_sec),"first cooldown ended too soon");
  Check(first.out.approach_mission==MissionType::kNone,"first crossing lost its cooldown protection");
}
void ReadyDonePauseReset() {
  Driver d;d.Tick();const auto a=d.out.command.action_id;
  Check(a!=0,"finish did not issue initial walk");
  d.Tick({a,true,false,false});
  for(int i=0;i<12;++i)d.Tick();
  Check(d.out.command.action_id==a && !d.core.command_coordinator_.HasQueuedAction(),
        "finish issued successor before READY");
  d.Tick({a,true,false,true});const auto b=d.out.command.action_id;
  Check(b!=0 && b!=a && d.out.command.action==MissionAction::kLineStraightUnit,
        "finish READY failed to reserve one successor");
  d.Tick({b,true,false,false});
  Check(!d.core.line_plan_.Done(a),"READY/queued ACK fabricated predecessor DONE");
  d.Tick({a,true,true,false});
  Check(d.core.line_plan_.Done(a) && !d.core.line_plan_.Done(b),"physical DONE ownership was lost");
  for(int i=0;i<12;++i)d.Tick();
  d.Tick({b,true,false,true});const auto c=d.out.command.action_id;
  Check(c && c!=b && d.out.command.action==MissionAction::kLineStraightUnit,
        "empty finish block started loss recovery at final READY");
  d.Tick({c,true,false,false});d.Tick({b,true,true,false});
  d.Tick({b,true,true,false});
  Check(!d.core.line_plan_.Done(c),"duplicate DONE completed a new finish walk");
  d.in.allow_new_commands=d.in.allow_new_body_actions=d.in.allow_new_line_action=false;
  d.in.allow_new_line_plan=d.in.allow_new_mission_entry=d.in.advance_line_fsm=false;
  d.Tick({c,true,false,true});
  Check(!d.core.command_coordinator_.HasQueuedAction(),"PAUSE allowed a finish successor");
  d.Tick({c,true,true,false});
  Check(d.core.line_plan_.Done(c),"PAUSE discarded actual DONE");
  d.in.allow_new_commands=d.in.allow_new_body_actions=d.in.allow_new_line_action=true;
  d.in.allow_new_line_plan=d.in.allow_new_mission_entry=d.in.advance_line_fsm=true;
  d.Tick();
  Check(d.out.command.action==MissionAction::kLineStraightUnit && d.out.command.action_id>c,
        "RESUME did not continue the existing finish walk plan");
  d.core.Reset(false);d.in.delivery_feedback={};d.Tick();
  Check(d.core.hurdle_crosses_completed_==0 && d.out.course_curve_index==1 &&
        d.out.command.action_id==0,"RESET retained finish-only forward policy");
}
}
int main() { PriorityAndGates();CooldownScope();ReadyDonePauseReset(); }
