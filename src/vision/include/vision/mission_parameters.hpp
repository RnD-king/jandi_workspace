#pragma once
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <limits>
#include <stdexcept>
#include "vision_core/mission_controller.hpp"

namespace vision {
inline void DeclareMissionParameters(rclcpp::Node& node,
                                    const vision_core::MissionControllerConfig& c) {
  node.declare_parameter<int>("ball_down_search_max_steps",c.ball.down_search_max_steps);
  node.declare_parameter<double>("ball_down_search_inter_action_hold_sec",c.ball.down_search_inter_action_hold_sec);
  node.declare_parameter<double>("ball_approach_direct_down_v_px",c.ball.approach_direct_down_v_px);
  node.declare_parameter<int>("hurdle_down_search_max_steps",c.hurdle.down_search_max_steps);
  node.declare_parameter<double>("hurdle_down_search_inter_action_hold_sec",c.hurdle.down_search_inter_action_hold_sec);
  node.declare_parameter<double>("hurdle_contact_inter_action_hold_sec",c.hurdle.contact_inter_action_hold_sec);
  node.declare_parameter<double>("line_min_box_width",c.line_detection.min_box_width);
  node.declare_parameter<double>("line_min_box_height",c.line_detection.min_box_height);
  node.declare_parameter<int>("ball_down_search_min_hits",c.ball.down_search_min_hits);
  node.declare_parameter<int>("hurdle_down_search_min_hits",c.hurdle.down_search_min_hits);
  node.declare_parameter<double>("line_p2p_in_place_turn_score",c.line_p2p.in_place_turn_score);
  node.declare_parameter<double>("ball_min_box_width",c.object_targets.ball_min_box_width);
  node.declare_parameter<double>("ball_min_box_height",c.object_targets.ball_min_box_height);
  node.declare_parameter<double>("goal_min_box_width",c.object_targets.goal_min_box_width);
  node.declare_parameter<double>("goal_min_box_height",c.object_targets.goal_min_box_height);
  node.declare_parameter<double>("backboard_min_box_width",c.object_targets.backboard_min_box_width);
  node.declare_parameter<double>("backboard_min_box_height",c.object_targets.backboard_min_box_height);
  node.declare_parameter<double>("hurdle_min_box_width",c.object_targets.hurdle_min_box_width);
  node.declare_parameter<double>("hurdle_min_box_height",c.object_targets.hurdle_min_box_height);
  node.declare_parameter<double>("transition_settle_sec", c.transition_settle_sec);
  node.declare_parameter<double>("fine_motion_settle_sec", c.fine_motion_settle_sec);
  node.declare_parameter<int>("ball_recovery_walk_turn_max", c.ball.recovery_walk_turn_max);
  node.declare_parameter<int>("goal_recovery_turn_max", c.goal.recovery_turn_max);
  node.declare_parameter<double>("goal_edge_margin_px", c.goal.edge_margin_px);
  node.declare_parameter<double>("ball_fine_side_two_step_margin_px", c.ball.fine_side_two_step_margin_px);
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
  c.ball.down_search_max_steps=turns("ball_down_search_max_steps");
  c.ball.approach_direct_down_v_px=node.get_parameter("ball_approach_direct_down_v_px").as_double();
  c.hurdle.down_search_max_steps=turns("hurdle_down_search_max_steps");
  const auto search_hold=[&](const char* name) {
    const double value=node.get_parameter(name).as_double();
    if (!std::isfinite(value) || value<0.0)
      throw std::invalid_argument(std::string(name)+" must be finite and nonnegative");
    return value;
  };
  c.ball.down_search_inter_action_hold_sec=search_hold("ball_down_search_inter_action_hold_sec");
  c.hurdle.down_search_inter_action_hold_sec=search_hold("hurdle_down_search_inter_action_hold_sec");
  const double contact_hold=node.get_parameter("hurdle_contact_inter_action_hold_sec").as_double();
  if (!std::isfinite(contact_hold) || contact_hold<0.0)
    throw std::invalid_argument("hurdle_contact_inter_action_hold_sec must be finite and nonnegative");
  c.hurdle.contact_inter_action_hold_sec=contact_hold;
  c.line_detection.min_box_width=node.get_parameter("line_min_box_width").as_double();
  c.line_detection.min_box_height=node.get_parameter("line_min_box_height").as_double();
  c.ball.down_search_min_hits=turns("ball_down_search_min_hits");
  c.hurdle.down_search_min_hits=turns("hurdle_down_search_min_hits");
  c.line_p2p.in_place_turn_score=node.get_parameter("line_p2p_in_place_turn_score").as_double();
  c.object_targets.ball_min_box_width=node.get_parameter("ball_min_box_width").as_double();
  c.object_targets.ball_min_box_height=node.get_parameter("ball_min_box_height").as_double();
  c.object_targets.goal_min_box_width=node.get_parameter("goal_min_box_width").as_double();
  c.object_targets.goal_min_box_height=node.get_parameter("goal_min_box_height").as_double();
  c.object_targets.backboard_min_box_width=node.get_parameter("backboard_min_box_width").as_double();
  c.object_targets.backboard_min_box_height=node.get_parameter("backboard_min_box_height").as_double();
  c.object_targets.hurdle_min_box_width=node.get_parameter("hurdle_min_box_width").as_double();
  c.object_targets.hurdle_min_box_height=node.get_parameter("hurdle_min_box_height").as_double();
  c.ball.recovery_walk_turn_max = turns("ball_recovery_walk_turn_max");
  c.goal.recovery_turn_max = turns("goal_recovery_turn_max");
  const double margin=node.get_parameter("goal_edge_margin_px").as_double();
  if (!std::isfinite(margin) || margin<0.0)
    throw std::invalid_argument("goal_edge_margin_px must be finite and nonnegative");
  c.goal.edge_margin_px = margin;
  c.ball.fine_side_two_step_margin_px = node.get_parameter("ball_fine_side_two_step_margin_px").as_double();
}
// Return true only for the new live settings. Validation is atomic after the
// entire parameter batch, so unrelated active mission settings stay intact.
inline bool ApplyLivePerceptionParameter(const rclcpp::Parameter& p,
                                        vision_core::MissionControllerConfig& c) {
  const auto& name=p.get_name();
  if (name=="ball_down_search_inter_action_hold_sec" ||
      name=="hurdle_down_search_inter_action_hold_sec") {
    if (p.get_type()!=rclcpp::ParameterType::PARAMETER_DOUBLE ||
        !std::isfinite(p.as_double()) || p.as_double()<0.0)
      throw std::invalid_argument(name+" must be a finite nonnegative double");
    if (name=="ball_down_search_inter_action_hold_sec")
      c.ball.down_search_inter_action_hold_sec=p.as_double();
    else c.hurdle.down_search_inter_action_hold_sec=p.as_double();
    return true;
  }
  if (name=="hurdle_contact_inter_action_hold_sec") {
    if (p.get_type()!=rclcpp::ParameterType::PARAMETER_DOUBLE ||
        !std::isfinite(p.as_double()) || p.as_double()<0.0)
      throw std::invalid_argument(name+" must be a finite nonnegative double");
    c.hurdle.contact_inter_action_hold_sec=p.as_double();
    return true;
  }
  if (name=="ball_approach_direct_down_v_px") {
    if (p.get_type()!=rclcpp::ParameterType::PARAMETER_DOUBLE)
      throw std::invalid_argument(name+" must be double");
    c.ball.approach_direct_down_v_px=p.as_double();
    return true;
  }
  if (name=="ball_down_search_max_steps" || name=="hurdle_down_search_max_steps") {
    if (p.get_type()!=rclcpp::ParameterType::PARAMETER_INTEGER ||
        p.as_int()<1 || p.as_int()>std::numeric_limits<int>::max())
      throw std::invalid_argument(name+" must be a positive integer");
    if (name=="ball_down_search_max_steps") c.ball.down_search_max_steps=static_cast<int>(p.as_int());
    else c.hurdle.down_search_max_steps=static_cast<int>(p.as_int());
    return true;
  }
  if (name=="line_min_box_width" || name=="line_min_box_height") {
    if (p.get_type()!=rclcpp::ParameterType::PARAMETER_DOUBLE)
      throw std::invalid_argument(name+" must be double");
    if (name=="line_min_box_width") c.line_detection.min_box_width=p.as_double();
    else c.line_detection.min_box_height=p.as_double();
    return true;
  }
  if (name=="ball_down_search_min_hits" || name=="hurdle_down_search_min_hits") {
    if (p.get_type()!=rclcpp::ParameterType::PARAMETER_INTEGER ||
        p.as_int()<1 || p.as_int()>std::numeric_limits<int>::max())
      throw std::invalid_argument(name+" must be a positive integer within stable_window");
    if (name=="ball_down_search_min_hits") c.ball.down_search_min_hits=static_cast<int>(p.as_int());
    else c.hurdle.down_search_min_hits=static_cast<int>(p.as_int());
    return true;
  }
  if (name=="line_p2p_in_place_turn_score") {
    if (p.get_type()!=rclcpp::ParameterType::PARAMETER_DOUBLE)
      throw std::invalid_argument(name+" must be double");
    c.line_p2p.in_place_turn_score=p.as_double();
    return true;
  }
  if (name=="ball_min_box_width") {
    if (p.get_type()!=rclcpp::ParameterType::PARAMETER_DOUBLE)
      throw std::invalid_argument(name+" must be double");
    c.object_targets.ball_min_box_width=p.as_double();
    return true;
  }
  if (name=="ball_min_box_height") {
    if (p.get_type()!=rclcpp::ParameterType::PARAMETER_DOUBLE)
      throw std::invalid_argument(name+" must be double");
    c.object_targets.ball_min_box_height=p.as_double();
    return true;
  }
  if (name=="goal_min_box_width") {
    if (p.get_type()!=rclcpp::ParameterType::PARAMETER_DOUBLE)
      throw std::invalid_argument(name+" must be double");
    c.object_targets.goal_min_box_width=p.as_double();
    return true;
  }
  if (name=="goal_min_box_height") {
    if (p.get_type()!=rclcpp::ParameterType::PARAMETER_DOUBLE)
      throw std::invalid_argument(name+" must be double");
    c.object_targets.goal_min_box_height=p.as_double();
    return true;
  }
  if (name=="backboard_min_box_width") {
    if (p.get_type()!=rclcpp::ParameterType::PARAMETER_DOUBLE)
      throw std::invalid_argument(name+" must be double");
    c.object_targets.backboard_min_box_width=p.as_double();
    return true;
  }
  if (name=="backboard_min_box_height") {
    if (p.get_type()!=rclcpp::ParameterType::PARAMETER_DOUBLE)
      throw std::invalid_argument(name+" must be double");
    c.object_targets.backboard_min_box_height=p.as_double();
    return true;
  }
  if (name=="hurdle_min_box_width") {
    if (p.get_type()!=rclcpp::ParameterType::PARAMETER_DOUBLE)
      throw std::invalid_argument(name+" must be double");
    c.object_targets.hurdle_min_box_width=p.as_double();
    return true;
  }
  if (name=="hurdle_min_box_height") {
    if (p.get_type()!=rclcpp::ParameterType::PARAMETER_DOUBLE)
      throw std::invalid_argument(name+" must be double");
    c.object_targets.hurdle_min_box_height=p.as_double();
    return true;
  }
  return false;
}
}
