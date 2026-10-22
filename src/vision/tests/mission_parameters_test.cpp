#include "vision/mission_parameters.hpp"
#include <limits>
#include <stdexcept>

static void Check(bool value,const char* message) {
  if(!value)throw std::runtime_error(message);
}
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try {
    vision_core::MissionControllerConfig base;
    base.transition_settle_sec=.6;base.fine_motion_settle_sec=0;
    base.ball.recovery_walk_turn_max=3;base.goal.recovery_turn_max=3;
    base.goal.edge_margin_px=50;
    base.ball.approach_direct_down_v_px=355;
    base.ball.approach_left_top_u_px=290;base.ball.approach_right_top_u_px=470;
    base.hurdle.contact_inter_action_hold_sec=.5;
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("ball_recovery_walk_turn_max",5),
        rclcpp::Parameter("goal_recovery_turn_max",2),
        rclcpp::Parameter("goal_edge_margin_px",75.0),
        rclcpp::Parameter("ball_approach_direct_down_v_px",370.0),
        rclcpp::Parameter("hurdle_contact_inter_action_hold_sec",.7),
        rclcpp::Parameter("ball_down_search_inter_action_hold_sec",.6),
        rclcpp::Parameter("hurdle_down_search_inter_action_hold_sec",1.2),
        rclcpp::Parameter("ball_approach_left_top_u_px",340.0),
        rclcpp::Parameter("ball_approach_right_top_u_px",520.0),
        rclcpp::Parameter("ball_fine_max_repetitions",2),
        rclcpp::Parameter("ball_fine_right_delta_u_px",-50.0),
        rclcpp::Parameter("ball_fine_right_delta_v_px",3.0)});
    auto node=std::make_shared<rclcpp::Node>("mission_parameter_test",options);
    vision::DeclareMissionParameters(*node,base);
    auto applied=base;
    // Mirror the adapter's final construction order: subconfigs first, then
    // ROS overrides. Source wiring coverage checks both constructors too.
    applied.ball=base.ball;applied.goal=base.goal;
    vision::ReadMissionParameters(*node,applied);
    Check(applied.ball.fine_motion.max_repetitions==2 &&
          applied.ball.fine_motion.right_delta_px.u==-50 && applied.ball.fine_motion.right_delta_px.v==3,
          "BALL fine calibration startup override was lost");
    for (const auto& setting : vision::BallFineMotionParameters(applied.ball.fine_motion)) {
      Check(vision::ApplyLivePerceptionParameter(rclcpp::Parameter(setting.first,-7.5),applied) &&
            *setting.second==-7.5,"signed BALL fine calibration live value rejected");
      for (double bad : {std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()}) {
        bool rejected=false;
        try {vision::ApplyLivePerceptionParameter(rclcpp::Parameter(setting.first,bad),applied);}
        catch (const std::invalid_argument&) {rejected=true;}
        Check(rejected && *setting.second==-7.5,"nonfinite fine delta accepted/changed candidate");
        node->set_parameter(rclcpp::Parameter(setting.first,bad));
        rejected=false;
        try {vision::ReadMissionParameters(*node,applied);} catch (const std::invalid_argument&) {rejected=true;}
        Check(rejected,"nonfinite fine delta startup value accepted");
      }
      node->set_parameter(rclcpp::Parameter(setting.first,0.0));
      bool rejected=false;
      try {vision::ApplyLivePerceptionParameter(rclcpp::Parameter(setting.first,5),applied);}
      catch (const std::invalid_argument&) {rejected=true;}
      Check(rejected,"integer fine delta accepted");
    }
    for (int bad : {0,4,-1}) {
      bool rejected=false;
      try {vision::ApplyLivePerceptionParameter(rclcpp::Parameter("ball_fine_max_repetitions",bad),applied);}
      catch (const std::invalid_argument&) {rejected=true;}
      Check(rejected,"invalid fine repeat count accepted");
      node->set_parameter(rclcpp::Parameter("ball_fine_max_repetitions",bad));
      rejected=false;
      try {vision::ReadMissionParameters(*node,applied);} catch (const std::invalid_argument&) {rejected=true;}
      Check(rejected,"invalid fine repeat startup count accepted");
    }
    Check(vision::ApplyLivePerceptionParameter(rclcpp::Parameter("ball_fine_max_repetitions",3),applied) &&
          applied.ball.fine_motion.max_repetitions==3,"fine maximum 3 not applied");
    node->set_parameter(rclcpp::Parameter("ball_fine_max_repetitions",3));
    vision::ReadMissionParameters(*node,applied);
    // The same field table drives startup overrides and live rqt edits.
    for(const auto& field:vision::kObjectFilterDoubles) {
      Check(vision::ApplyLivePerceptionParameter(rclcpp::Parameter(field.name,1.25),applied),"new object double was not recognized");
      Check(applied.object_targets.*(field.field)==1.25,"new object double was not applied");
      node->set_parameter(rclcpp::Parameter(field.name,1.5));
      vision::ReadMissionParameters(*node,applied);
      Check(applied.object_targets.*(field.field)==1.5,"new object startup override was lost");
      bool rejected=false;
      try {vision::ApplyLivePerceptionParameter(rclcpp::Parameter(field.name,1),applied);}
      catch(const std::invalid_argument&) {rejected=true;}
      Check(rejected,"new object double accepted integer type");
      rejected=false;
      try {vision::ApplyLivePerceptionParameter(rclcpp::Parameter(field.name,-1.0),applied);}
      catch(const std::invalid_argument&) {rejected=true;}
      Check(rejected,"new object double accepted negative value");
    }
    for(const auto& field:vision::kObjectFilterBools) {
      Check(vision::ApplyLivePerceptionParameter(rclcpp::Parameter(field.name,true),applied),"new object bool was not recognized");
      Check(applied.object_targets.*(field.field),"new object bool was not applied");
    }
    for(const auto* name:{"object_depth_roi_size_px","ball_candidate_missing_frame_limit","ball_candidate_max_tracks"})
      Check(vision::ApplyLivePerceptionParameter(rclcpp::Parameter(name,5),applied),"new tracking integer was not recognized");
    Check(applied.object_targets.depth_roi_size_px==5 && applied.ball_candidate_tracking.missing_frame_limit==5 &&
        applied.ball_candidate_tracking.max_tracks==5,"new tracking integer values lost");
    Check(vision::ApplyLivePerceptionParameter(rclcpp::Parameter("ball_candidate_max_center_distance_px",90.0),applied) &&
        applied.ball_candidate_tracking.max_center_distance_px==90,"candidate pixel gate live setting lost");
    Check(vision::ApplyLivePerceptionParameter(rclcpp::Parameter("ball_candidate_max_size_log_ratio",.6),applied) &&
        applied.ball_candidate_tracking.max_size_log_ratio==.6,"candidate size gate live setting lost");
    Check(vision::ApplyLivePerceptionParameter(rclcpp::Parameter("ball_candidate_tracking_enabled",true),applied) &&
        applied.ball_candidate_tracking.enabled,"candidate enable live setting lost");

    Check(applied.ball.recovery_walk_turn_max==5,"BALL startup override was lost");
    Check(applied.goal.recovery_turn_max==2,"GOAL startup override was lost");
    Check(applied.goal.edge_margin_px==75,"edge margin startup override was lost");
    Check(applied.ball.approach_direct_down_v_px==370,"BALL direct DOWN startup override was lost");
    Check(applied.ball.approach_left_top_u_px==340 && applied.ball.approach_right_top_u_px==520,
          "BALL top endpoint startup overrides were lost");
    Check(vision::ApplyLivePerceptionParameter(
        rclcpp::Parameter("ball_approach_left_top_u_px",360.0),applied) &&
        applied.ball.approach_left_top_u_px==360 && applied.ball.approach_right_top_u_px==520,
        "left top endpoint update changed the other endpoint");
    Check(vision::ApplyLivePerceptionParameter(
        rclcpp::Parameter("ball_approach_right_top_u_px",550.0),applied) &&
        applied.ball.approach_right_top_u_px==550 && applied.ball.approach_left_top_u_px==360,
        "right top endpoint update changed the other endpoint");
    for (const auto* name : {"ball_approach_left_top_u_px", "ball_approach_right_top_u_px"}) {
      for (double bad : {-1.0, std::numeric_limits<double>::quiet_NaN(),
                          std::numeric_limits<double>::infinity()}) {
        bool rejected=false;
        try {vision::ApplyLivePerceptionParameter(rclcpp::Parameter(name,bad),applied);}
        catch (const std::invalid_argument&) {rejected=true;}
        Check(rejected,"invalid top endpoint accepted");
      }
      bool rejected=false;
      try {vision::ApplyLivePerceptionParameter(rclcpp::Parameter(name,350),applied);}
      catch (const std::invalid_argument&) {rejected=true;}
      Check(rejected,"integer top endpoint accepted");
    }
    node->set_parameter(rclcpp::Parameter("ball_approach_left_top_u_px",520.0));
    bool crossed=false;
    try {vision::ReadMissionParameters(*node,applied);} catch (const std::invalid_argument&) {crossed=true;}
    Check(crossed,"equal/crossed top endpoints accepted at startup");
    node->set_parameter(rclcpp::Parameter("ball_approach_left_top_u_px",340.0));
    vision::ReadMissionParameters(*node,applied);
    Check(applied.hurdle.contact_inter_action_hold_sec==.7,"HURDLE contact hold startup override was lost");
    Check(applied.ball.down_search_inter_action_hold_sec==.6 &&
          applied.hurdle.down_search_inter_action_hold_sec==1.2,
          "independent DOWN search hold startup overrides were lost");
    for (const auto* name : {"ball_down_search_inter_action_hold_sec",
                             "hurdle_down_search_inter_action_hold_sec"}) {
      for (double valid : {0.0, .8}) {
        Check(vision::ApplyLivePerceptionParameter(rclcpp::Parameter(name,valid),applied),
              "DOWN search hold live setting was not recognized");
        const double value=std::string(name)=="ball_down_search_inter_action_hold_sec"
            ? applied.ball.down_search_inter_action_hold_sec
            : applied.hurdle.down_search_inter_action_hold_sec;
        Check(value==valid,"DOWN search hold live value was lost");
      }
      for (double bad : {-1.0, std::numeric_limits<double>::quiet_NaN(),
                          std::numeric_limits<double>::infinity()}) {
        bool rejected=false;
        try { vision::ApplyLivePerceptionParameter(rclcpp::Parameter(name,bad),applied); }
        catch (const std::invalid_argument&) { rejected=true; }
        Check(rejected,"invalid DOWN search hold live value accepted");
        node->set_parameter(rclcpp::Parameter(name,bad));
        rejected=false;
        try { vision::ReadMissionParameters(*node,applied); }
        catch (const std::invalid_argument&) { rejected=true; }
        Check(rejected,"invalid DOWN search hold startup value accepted");
      }
      node->set_parameter(rclcpp::Parameter(name,0.0));
      bool wrong_type=false;
      try { vision::ApplyLivePerceptionParameter(rclcpp::Parameter(name,1),applied); }
      catch (const std::invalid_argument&) { wrong_type=true; }
      Check(wrong_type,"integer DOWN search hold value accepted");
    }
    Check(vision::ApplyLivePerceptionParameter(
        rclcpp::Parameter("hurdle_contact_inter_action_hold_sec",.9),applied) &&
        applied.hurdle.contact_inter_action_hold_sec==.9,"HURDLE contact hold live setting was not applied");
    Check(vision::ApplyLivePerceptionParameter(
        rclcpp::Parameter("hurdle_contact_inter_action_hold_sec",0.0),applied) &&
        applied.hurdle.contact_inter_action_hold_sec==0,"zero contact hold rejected");
    for(double bad:{-1.0,std::numeric_limits<double>::quiet_NaN(),
                    std::numeric_limits<double>::infinity()}) {
      bool rejected=false;
      try {vision::ApplyLivePerceptionParameter(
          rclcpp::Parameter("hurdle_contact_inter_action_hold_sec",bad),applied);}
      catch(const std::invalid_argument&) {rejected=true;}
      Check(rejected,"invalid HURDLE contact hold accepted");
      Check(applied.hurdle.contact_inter_action_hold_sec==0,"rejected hold changed the candidate");
      node->set_parameter(rclcpp::Parameter("hurdle_contact_inter_action_hold_sec",bad));
      rejected=false;
      try {vision::ReadMissionParameters(*node,applied);}
      catch(const std::invalid_argument&) {rejected=true;}
      Check(rejected,"invalid HURDLE contact hold startup parameter accepted");
    }
    node->set_parameter(rclcpp::Parameter("hurdle_contact_inter_action_hold_sec",.5));
    bool wrong_hold_type=false;
    try {vision::ApplyLivePerceptionParameter(
        rclcpp::Parameter("hurdle_contact_inter_action_hold_sec",1),applied);}
    catch(const std::invalid_argument&) {wrong_hold_type=true;}
    Check(wrong_hold_type,"integer HURDLE contact hold parameter accepted");
    Check(vision::ApplyLivePerceptionParameter(
        rclcpp::Parameter("ball_approach_direct_down_v_px",390.0),applied) &&
        applied.ball.approach_direct_down_v_px==390,"BALL direct DOWN live setting was not applied");
    bool bad_type=false;
    try {vision::ApplyLivePerceptionParameter(
        rclcpp::Parameter("ball_approach_direct_down_v_px",390),applied);}
    catch(const std::invalid_argument&) {bad_type=true;}
    Check(bad_type,"BALL direct DOWN must reject an integer parameter");
    Check(node->set_parameter(rclcpp::Parameter("goal_edge_margin_px",20.0)).successful,
          "double edge margin update rejected");
    vision::ReadMissionParameters(*node,applied);
    Check(applied.goal.edge_margin_px==20,"new edge margin was not read");
    for(double bad:{-1.0,std::numeric_limits<double>::quiet_NaN(),
                    std::numeric_limits<double>::infinity()}) {
      node->set_parameter(rclcpp::Parameter("goal_edge_margin_px",bad));
      bool rejected=false;
      try { vision::ReadMissionParameters(*node,applied); }
      catch(const std::invalid_argument&) { rejected=true; }
      Check(rejected,"invalid edge margin accepted");
    }
    rclcpp::shutdown();return 0;
  } catch(...) {rclcpp::shutdown();throw;}
}
