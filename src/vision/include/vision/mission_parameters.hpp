#pragma once
#include <rclcpp/rclcpp.hpp>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include "vision_core/mission_controller.hpp"

namespace vision {
inline std::array<std::pair<const char*,double*>,8> BallFineMotionParameters(
    vision_core::BallFineMotionConfig& c) {
  return {{{"ball_fine_forward_delta_u_px",&c.forward_delta_px.u},
      {"ball_fine_forward_delta_v_px",&c.forward_delta_px.v},
      {"ball_fine_backward_delta_u_px",&c.backward_delta_px.u},
      {"ball_fine_backward_delta_v_px",&c.backward_delta_px.v},
      {"ball_fine_left_delta_u_px",&c.left_delta_px.u},
      {"ball_fine_left_delta_v_px",&c.left_delta_px.v},
      {"ball_fine_right_delta_u_px",&c.right_delta_px.u},
      {"ball_fine_right_delta_v_px",&c.right_delta_px.v}}};
}
struct ObjectFilterDoubleParameter {
  const char* name;
  double vision_core::ObjectTargetConfig::*field;
};
inline const ObjectFilterDoubleParameter kObjectFilterDoubles[] = {
  {"ball_aspect_ratio_min",&vision_core::ObjectTargetConfig::ball_aspect_ratio_min},
  {"ball_aspect_ratio_max",&vision_core::ObjectTargetConfig::ball_aspect_ratio_max},
  {"hurdle_aspect_ratio_min",&vision_core::ObjectTargetConfig::hurdle_aspect_ratio_min},
  {"ball_forward_min_depth_m",&vision_core::ObjectTargetConfig::ball_forward_min_depth_m},
  {"ball_forward_max_depth_m",&vision_core::ObjectTargetConfig::ball_forward_max_depth_m},
  {"ball_down_min_depth_m",&vision_core::ObjectTargetConfig::ball_down_min_depth_m},
  {"ball_down_max_depth_m",&vision_core::ObjectTargetConfig::ball_down_max_depth_m},
  {"hurdle_min_depth_m",&vision_core::ObjectTargetConfig::hurdle_min_depth_m},
  {"hurdle_max_depth_m",&vision_core::ObjectTargetConfig::hurdle_max_depth_m},
};
struct ObjectFilterBoolParameter {
  const char* name;
  bool vision_core::ObjectTargetConfig::*field;
};
inline const ObjectFilterBoolParameter kObjectFilterBools[] = {
  {"ball_depth_enabled",&vision_core::ObjectTargetConfig::ball_depth_enabled},
  {"hurdle_depth_enabled",&vision_core::ObjectTargetConfig::hurdle_depth_enabled},
  {"ball_allow_missing_depth",&vision_core::ObjectTargetConfig::ball_allow_missing_depth},
  {"hurdle_allow_missing_depth",&vision_core::ObjectTargetConfig::hurdle_allow_missing_depth},
};
struct CandidateTrackingParameterGroup {
  const char* prefix;
  vision_core::ObjectCandidateTrackingConfig vision_core::MissionControllerConfig::*field;
};
inline const CandidateTrackingParameterGroup kAdditionalCandidateTracking[] = {
  {"hurdle_candidate_", &vision_core::MissionControllerConfig::hurdle_candidate_tracking},
  {"backboard_candidate_", &vision_core::MissionControllerConfig::backboard_candidate_tracking},
};
inline void DeclareMissionParameters(rclcpp::Node& node,
                                    const vision_core::MissionControllerConfig& c) {
  node.declare_parameter<int>("ball_fine_max_repetitions",c.ball.fine_motion.max_repetitions);
  auto fine_motion=c.ball.fine_motion;
  for (const auto& setting : BallFineMotionParameters(fine_motion))
    node.declare_parameter<double>(setting.first,*setting.second);

  for(const auto& p:kObjectFilterDoubles) node.declare_parameter<double>(p.name,c.object_targets.*(p.field));
  for(const auto& p:kObjectFilterBools) node.declare_parameter<bool>(p.name,c.object_targets.*(p.field));
  node.declare_parameter<int>("object_depth_roi_size_px",c.object_targets.depth_roi_size_px);
  node.declare_parameter<bool>("ball_candidate_tracking_enabled",c.ball_candidate_tracking.enabled);
  node.declare_parameter<double>("ball_candidate_max_center_distance_px",c.ball_candidate_tracking.max_center_distance_px);
  node.declare_parameter<double>("ball_candidate_max_size_log_ratio",c.ball_candidate_tracking.max_size_log_ratio);
  node.declare_parameter<int>("ball_candidate_missing_frame_limit",c.ball_candidate_tracking.missing_frame_limit);
  node.declare_parameter<int>("ball_candidate_max_tracks",c.ball_candidate_tracking.max_tracks);
  for (const auto& group : kAdditionalCandidateTracking) {
    const std::string prefix = group.prefix;
    const auto& tracking = c.*(group.field);
    node.declare_parameter<bool>(prefix+"tracking_enabled", tracking.enabled);
    node.declare_parameter<double>(prefix+"max_center_distance_px", tracking.max_center_distance_px);
    node.declare_parameter<double>(prefix+"max_size_log_ratio", tracking.max_size_log_ratio);
    node.declare_parameter<int>(prefix+"missing_frame_limit", tracking.missing_frame_limit);
    node.declare_parameter<int>(prefix+"max_tracks", tracking.max_tracks);
  }
  node.declare_parameter<int>("ball_down_search_max_steps",c.ball.down_search_max_steps);
  node.declare_parameter<double>("ball_down_search_inter_action_hold_sec",c.ball.down_search_inter_action_hold_sec);
  node.declare_parameter<double>("ball_approach_direct_down_v_px",c.ball.approach_direct_down_v_px);
  node.declare_parameter<double>("ball_approach_left_top_u_px",c.ball.approach_left_top_u_px);
  node.declare_parameter<double>("ball_approach_right_top_u_px",c.ball.approach_right_top_u_px);
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
  node.declare_parameter<int>("ball_recovery_max_steps", c.ball.recovery_max_steps);
  node.declare_parameter<int>("ball_entry_recovery_max_steps", c.ball.entry_recovery_max_steps);
  node.declare_parameter<int>("hurdle_entry_recovery_max_steps", c.hurdle.entry_recovery_max_steps);
  node.declare_parameter<int>("hurdle_recovery_max_steps", c.hurdle.recovery_max_steps);
  node.declare_parameter<int>("goal_recovery_turn_max", c.goal.recovery_turn_max);
  node.declare_parameter<double>("goal_edge_margin_px", c.goal.edge_margin_px);
  node.declare_parameter<double>("ball_fine_side_two_step_margin_px", c.ball.fine_side_two_step_margin_px);
}
inline void ReadMissionParameters(const rclcpp::Node& node,
                                 vision_core::MissionControllerConfig& c) {
  const auto fine_repetitions=node.get_parameter("ball_fine_max_repetitions").as_int();
  if (fine_repetitions<1 || fine_repetitions>3)
    throw std::invalid_argument("ball_fine_max_repetitions must be an integer in [1,3]");
  c.ball.fine_motion.max_repetitions=static_cast<int>(fine_repetitions);
  for (const auto& setting : BallFineMotionParameters(c.ball.fine_motion)) {
    const double value=node.get_parameter(setting.first).as_double();
    if (!std::isfinite(value)) throw std::invalid_argument(std::string(setting.first)+" must be finite");
    *setting.second=value;
  }
  const auto read_int=[&](const char* name) {
    const auto value=node.get_parameter(name).as_int();
    if(value<0 || value>std::numeric_limits<int>::max()) throw std::invalid_argument(std::string(name)+" out of integer range");
    return static_cast<int>(value);
  };
  for(const auto& p:kObjectFilterDoubles) c.object_targets.*(p.field)=node.get_parameter(p.name).as_double();
  for(const auto& p:kObjectFilterBools) c.object_targets.*(p.field)=node.get_parameter(p.name).as_bool();
  c.object_targets.depth_roi_size_px=read_int("object_depth_roi_size_px");
  c.ball_candidate_tracking.enabled=node.get_parameter("ball_candidate_tracking_enabled").as_bool();
  c.ball_candidate_tracking.max_center_distance_px=node.get_parameter("ball_candidate_max_center_distance_px").as_double();
  c.ball_candidate_tracking.max_size_log_ratio=node.get_parameter("ball_candidate_max_size_log_ratio").as_double();
  c.ball_candidate_tracking.missing_frame_limit=read_int("ball_candidate_missing_frame_limit");
  c.ball_candidate_tracking.max_tracks=read_int("ball_candidate_max_tracks");
  for (const auto& group : kAdditionalCandidateTracking) {
    const std::string prefix = group.prefix;
    auto& tracking = c.*(group.field);
    tracking.enabled = node.get_parameter(prefix+"tracking_enabled").as_bool();
    tracking.max_center_distance_px = node.get_parameter(prefix+"max_center_distance_px").as_double();
    tracking.max_size_log_ratio = node.get_parameter(prefix+"max_size_log_ratio").as_double();
    tracking.missing_frame_limit = read_int((prefix+"missing_frame_limit").c_str());
    tracking.max_tracks = read_int((prefix+"max_tracks").c_str());
  }
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
  c.ball.approach_left_top_u_px=node.get_parameter("ball_approach_left_top_u_px").as_double();
  c.ball.approach_right_top_u_px=node.get_parameter("ball_approach_right_top_u_px").as_double();
  if (!std::isfinite(c.ball.approach_left_top_u_px) ||
      !std::isfinite(c.ball.approach_right_top_u_px) ||
      c.ball.approach_left_top_u_px<0.0 ||
      c.ball.approach_left_top_u_px>=c.ball.approach_right_top_u_px)
    throw std::invalid_argument("BALL approach top endpoints must be finite and 0 <= left < right");
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
  c.ball.recovery_max_steps = turns("ball_recovery_max_steps");
  c.ball.entry_recovery_max_steps = turns("ball_entry_recovery_max_steps");
  c.hurdle.entry_recovery_max_steps = turns("hurdle_entry_recovery_max_steps");
  c.hurdle.recovery_max_steps = turns("hurdle_recovery_max_steps");
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
  if (name=="ball_fine_max_repetitions") {
    if (p.get_type()!=rclcpp::ParameterType::PARAMETER_INTEGER || p.as_int()<1 || p.as_int()>3)
      throw std::invalid_argument(name+" must be an integer in [1,3]");
    c.ball.fine_motion.max_repetitions=static_cast<int>(p.as_int());
    return true;
  }
  for (const auto& setting : BallFineMotionParameters(c.ball.fine_motion)) {
    if (name!=setting.first) continue;
    if (p.get_type()!=rclcpp::ParameterType::PARAMETER_DOUBLE || !std::isfinite(p.as_double()))
      throw std::invalid_argument(name+" must be a finite double");
    *setting.second=p.as_double();return true;
  }
  if (name=="ball_approach_left_top_u_px" || name=="ball_approach_right_top_u_px") {
    if (p.get_type()!=rclcpp::ParameterType::PARAMETER_DOUBLE ||
        !std::isfinite(p.as_double()) || p.as_double()<0.0)
      throw std::invalid_argument(name+" must be a finite nonnegative double");
    if (name=="ball_approach_left_top_u_px") c.ball.approach_left_top_u_px=p.as_double();
    else c.ball.approach_right_top_u_px=p.as_double();
    // Validate left/right order after the full parameter batch, so a paired
    // endpoint move is atomic in the existing adapter callbacks.
    return true;
  }
  for(const auto& entry:kObjectFilterDoubles) if(name==entry.name) {
    if(p.get_type()!=rclcpp::ParameterType::PARAMETER_DOUBLE || !std::isfinite(p.as_double()) || p.as_double()<0)
      throw std::invalid_argument(name+" must be a finite nonnegative double");
    c.object_targets.*(entry.field)=p.as_double(); return true;
  }
  for(const auto& entry:kObjectFilterBools) if(name==entry.name) {
    if(p.get_type()!=rclcpp::ParameterType::PARAMETER_BOOL) throw std::invalid_argument(name+" must be bool");
    c.object_targets.*(entry.field)=p.as_bool(); return true;
  }
  if(name=="ball_candidate_tracking_enabled") {
    if(p.get_type()!=rclcpp::ParameterType::PARAMETER_BOOL) throw std::invalid_argument(name+" must be bool");
    c.ball_candidate_tracking.enabled=p.as_bool(); return true;
  }
  for (const auto& group : kAdditionalCandidateTracking) {
    const std::string prefix = group.prefix;
    auto& tracking = c.*(group.field);
    if (name == prefix+"tracking_enabled") {
      if (p.get_type()!=rclcpp::ParameterType::PARAMETER_BOOL)
        throw std::invalid_argument(name+" must be bool");
      tracking.enabled=p.as_bool(); return true;
    }
    if (name == prefix+"max_center_distance_px" || name == prefix+"max_size_log_ratio") {
      if (p.get_type()!=rclcpp::ParameterType::PARAMETER_DOUBLE ||
          !std::isfinite(p.as_double()) || p.as_double()<=0)
        throw std::invalid_argument(name+" must be a finite positive double");
      if (name == prefix+"max_center_distance_px") tracking.max_center_distance_px=p.as_double();
      else tracking.max_size_log_ratio=p.as_double();
      return true;
    }
    if (name == prefix+"missing_frame_limit" || name == prefix+"max_tracks") {
      if (p.get_type()!=rclcpp::ParameterType::PARAMETER_INTEGER || p.as_int()<0 ||
          p.as_int()>std::numeric_limits<int>::max())
        throw std::invalid_argument(name+" must be a bounded nonnegative integer");
      if (name == prefix+"missing_frame_limit") tracking.missing_frame_limit=static_cast<int>(p.as_int());
      else tracking.max_tracks=static_cast<int>(p.as_int());
      return true;
    }
  }
  if(name=="ball_candidate_max_center_distance_px" || name=="ball_candidate_max_size_log_ratio") {
    if(p.get_type()!=rclcpp::ParameterType::PARAMETER_DOUBLE || !std::isfinite(p.as_double()) || p.as_double()<=0)
      throw std::invalid_argument(name+" must be a finite positive double");
    if(name=="ball_candidate_max_center_distance_px") c.ball_candidate_tracking.max_center_distance_px=p.as_double();
    else c.ball_candidate_tracking.max_size_log_ratio=p.as_double();
    return true;
  }
  if(name=="object_depth_roi_size_px" || name=="ball_candidate_missing_frame_limit" || name=="ball_candidate_max_tracks") {
    if(p.get_type()!=rclcpp::ParameterType::PARAMETER_INTEGER || p.as_int()<0 || p.as_int()>std::numeric_limits<int>::max())
      throw std::invalid_argument(name+" must be a bounded nonnegative integer");
    const int value=static_cast<int>(p.as_int());
    if(name=="object_depth_roi_size_px") c.object_targets.depth_roi_size_px=value;
    else if(name=="ball_candidate_missing_frame_limit") c.ball_candidate_tracking.missing_frame_limit=value;
    else c.ball_candidate_tracking.max_tracks=value;
    return true;
  }

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
