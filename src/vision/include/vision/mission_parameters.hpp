#pragma once
#include <rclcpp/rclcpp.hpp>
#include <limits>
#include <stdexcept>
#include "vision_core/mission_controller.hpp"

namespace vision {
inline void DeclareMissionParameters(rclcpp::Node& node,
                                    const vision_core::MissionControllerConfig& c) {
  node.declare_parameter<double>("transition_settle_sec", c.transition_settle_sec);
  node.declare_parameter<double>("fine_motion_settle_sec", c.fine_motion_settle_sec);
  node.declare_parameter<int>("ball_recovery_walk_turn_max", c.ball.recovery_walk_turn_max);
  node.declare_parameter<int>("goal_recovery_turn_max", c.goal.recovery_turn_max);
}
inline void ReadMissionParameters(const rclcpp::Node& node,
                                 vision_core::MissionControllerConfig& c) {
  c.transition_settle_sec = node.get_parameter("transition_settle_sec").as_double();
  c.fine_motion_settle_sec = node.get_parameter("fine_motion_settle_sec").as_double();
  const auto turns = [&](const char* name) {
    const auto value=node.get_parameter(name).as_int();
    if(value<0 || value>std::numeric_limits<int>::max())
      throw std::invalid_argument(std::string(name)+" must be a nonnegative int");
    return static_cast<int>(value);
  };
  c.ball.recovery_walk_turn_max = turns("ball_recovery_walk_turn_max");
  c.goal.recovery_turn_max = turns("goal_recovery_turn_max");
}
}
