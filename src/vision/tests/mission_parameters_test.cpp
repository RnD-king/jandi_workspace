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
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("ball_recovery_walk_turn_max",5),
        rclcpp::Parameter("goal_recovery_turn_max",2),
        rclcpp::Parameter("goal_edge_margin_px",75.0)});
    auto node=std::make_shared<rclcpp::Node>("mission_parameter_test",options);
    vision::DeclareMissionParameters(*node,base);
    auto applied=base;
    // Mirror the adapter's final construction order: subconfigs first, then
    // ROS overrides. Source wiring coverage checks both constructors too.
    applied.ball=base.ball;applied.goal=base.goal;
    vision::ReadMissionParameters(*node,applied);
    Check(applied.ball.recovery_walk_turn_max==5,"BALL startup override was lost");
    Check(applied.goal.recovery_turn_max==2,"GOAL startup override was lost");
    Check(applied.goal.edge_margin_px==75,"edge margin startup override was lost");
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
