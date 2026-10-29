#include "vision_core/config_loader.hpp"
#include <stdexcept>

using namespace vision_core;
namespace {
void Check(bool ok,const char* message) { if(!ok) throw std::runtime_error(message); }
ObjectTarget Target(double v) {
  ObjectTarget t;t.center_px={320,v};t.box_px={300,v-10,40,40};
  t.width_px=t.height_px=40;t.area_px=1600;t.confidence=1;return t;
}
bool FineTranslation(MissionAction a) {
  return a==MissionAction::kStepForwardHalf || a==MissionAction::kStepBack ||
      a==MissionAction::kLeftSideStep || a==MissionAction::kRightSideStep;
}
void Run(bool continuous,bool goal) {
  auto c=LoadDefaultAlgorithmConfig();c.object_approach_continous=continuous;
  c.enable_ball=!goal;c.enable_hurdle=false;c.enable_goal=goal;
  c.initial_has_ball=c.goal_trial_start_goal=goal;c.test_mission=MissionType::kNone;
  c.transition_settle_sec=c.fine_motion_settle_sec=0;c.post_motion_observation_sec=1;
  c.line_features.image_center_u=320;
  c.ball.far_u_des_px=320;c.ball.approach_left_top_u_px=230;c.ball.approach_right_top_u_px=410;
  c.ball.approach_direct_down_v_px=480;c.ball.down_search_min_hits=1;
  c.ball.fine_motion={};c.ball.fine_side_two_step_margin_px=1e6;
  c.ball.pickup_u1_px=300;c.ball.pickup_u2_px=340;c.ball.pickup_v1_px=100;c.ball.pickup_v2_px=150;
  c.goal.fine_adjust_start_z_m=.8;c.goal.throw_landing_x_m=0;c.goal.throw_landing_z_m=.4;
  MissionController core(c);MissionFrameInput in;MissionFrameResult out;
  in.image_width=640;in.image_height=480;in.command_transport_enabled=true;
  in.allow_new_line_action=false;in.line_centers={{320,450},{320,350},{320,250},{320,150}};
  in.camera_feedback={goal ? CameraMode::kGoal : CameraMode::kForward,true};
  if(goal) {
    in.backboard_target=Target(150);in.backboard_track_id=9;
    in.backboard_center_depth_m=.8;in.goal_pose={true,0,.8,0,1};
  } else {in.ball_target=Target(300);in.ball_track_id=41;}
  std::uint64_t last_done=0;bool fine_requested=false,fine_done=false,down=false,finished=false;
  for(int frame=0;frame<300;++frame) {
    in.delivery_feedback={};
    if(out.command.camera_request==CameraRequest::kDown) {
      down=true;in.camera_feedback={CameraMode::kDown,true};in.ball_target=Target(5);
    }
    if(out.command.command_type==CommandType::kAction && out.command.action_id && out.command.action_id!=last_done) {
      const auto a=out.command.action;
      if(a==MissionAction::kDefaultPoseMode) {fine_requested=true;fine_done=true;}
      else if(FineTranslation(a)) {
        Check(fine_requested && fine_done,"FINE translation preceded the real feet transition DONE");
        Check(goal || down,"BALL used FINE translation before the DOWN view");
        finished=true;break;
      } else if(goal || down) {
        Check(false,"near GOAL/acquired DOWN BALL issued a WALK or turn instead of FINE translation");
      }
      last_done=out.command.action_id;in.delivery_feedback={last_done,true,true,false};
    }
    in.now_sec+=.1;out=core.Step(in);
  }
  Check(finished,"T/F mode did not complete immediate FINE entry and first translation");
  Check(out.foot_pose==FootPose::kFine,"first FINE translation did not retain FINE foot state");
}
}
int main() {for(bool continuous:{false,true}) {Run(continuous,false);Run(continuous,true);}}
