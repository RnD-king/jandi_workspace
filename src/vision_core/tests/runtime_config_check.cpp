#include "vision_core/config_loader.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <utility>

namespace {
template <class T>
void CheckField(const YAML::Node& node, const char* key, const T& actual,
                const std::string& section) {
  if (node[key].as<T>() != actual)
    throw std::runtime_error("YAML value not applied: " + section + "." + key);
}

// Independently locate a raw YAML value in either supported file layout.
// Compare every loaded field with that value, including object-specific IDs.
YAML::Node AlgorithmField(const YAML::Node& a,const std::string& section,
                         const std::string& key) {
  if (!a["common"]) return a[section][key];
  if (section=="line_features") return a["line"]["features"][key];
  if (section=="line_detection") return a["line"]["detection"][key];
  if (section=="line") return a["line"]["stability"][key];
  if (section=="line_p2p") return a["line"]["walk"][key];
  if (section=="object_targets") {
    if (key=="depth_roi_size_px") return a["common"][key];
    const auto split=key.find('_');
    const auto object=key.substr(0,split), field=key.substr(split+1);
    const auto node=object=="backboard" ? a["goal"]["backboard"] : a[object];
    return node["detection"][field];
  }
  const std::string suffix="_candidate_tracking";
  if (section.size()>suffix.size() &&
      section.compare(section.size()-suffix.size(),suffix.size(),suffix)==0) {
    const auto object=section.substr(0,section.size()-suffix.size());
    const auto node=object=="backboard" ? a["goal"]["backboard"] : a[object];
    return node["candidate_tracking"][key];
  }
  if (section=="ball" || section=="hurdle" || section=="goal") {
    if (section=="goal" && key.find("backboard_")==0)
      return a["goal"]["backboard"]["depth"][key.substr(10)];
    return a[section]["control"][key];
  }
  if (section=="mission" && key.find("enable_")==0)
    return a[key.substr(7)]["enabled"];
  return a["common"][section][key];
}
template<class T>
void CheckAlgorithmField(const YAML::Node& a,const char* section,const char* key,const T& actual) {
  if (AlgorithmField(a,section,key).as<T>()!=actual)
    throw std::runtime_error(std::string("YAML value not applied: ")+section+"."+key);
}

void CheckConfiguration(const std::string& path) {
  // Use production loading/validation, then compare against the file's values,
  // not today's tuning constants. Do not implement a second mission decision.
  const auto c = vision_core::LoadAlgorithmConfig(path);
  const auto a = YAML::LoadFile(path)["vision_algorithm"];
#define FIELD(section, key) CheckAlgorithmField(a, #section, #key, c.section.key)
#define MISSION(key) CheckAlgorithmField(a, "mission", #key, c.key)
  const auto approach=AlgorithmField(a,"mission","object_approach");
  CheckField(approach,"lock_blocks",c.object_approach.lock_blocks,"object_approach");
  CheckField(approach,"blend_frames",c.object_approach.blend_frames,"object_approach");
  CheckField(approach,"goal_empty_max_retries",c.object_approach.goal_empty_max_retries,"object_approach");
  CheckField(approach,"reference_v_ratio",c.object_approach.reference_v_ratio,"object_approach");
  for (const auto& camera : {std::make_pair("forward_camera",c.object_approach.forward_camera),
                            std::make_pair("goal_camera",c.object_approach.goal_camera)}) {
    const auto node=approach[camera.first];const auto& pose=camera.second;
    CheckField(node,"right_m",pose.right_m,camera.first);CheckField(node,"height_m",pose.height_m,camera.first);
    CheckField(node,"forward_m",pose.forward_m,camera.first);CheckField(node,"pitch_deg",pose.pitch_deg,camera.first);
    CheckField(node,"yaw_deg",pose.yaw_deg,camera.first);
  }
  FIELD(line_features, max_centers);
  FIELD(line_features, guide_min_points);
  FIELD(line_features, image_center_u);
  FIELD(line_features, curve_min_points);
  FIELD(line_features, curve_min_v_span_px);
  FIELD(line_features, curve_local_fit_points);
  FIELD(line_features, guide_fit_rmse_full_scale_px);
  FIELD(line_features, ransac_enabled);
  FIELD(line_features, ransac_min_input_points);
  FIELD(line_features, ransac_residual_threshold_px);
  FIELD(line_features, ransac_min_sample_v_gap_px);
  FIELD(line_features, ransac_max_outliers);
  FIELD(line, line_stable_window);
  FIELD(line, line_stable_min_hits);
  FIELD(line_p2p, left_repetitions);
  FIELD(line_p2p, straight_repetitions);
  FIELD(line_p2p, right_repetitions);
  FIELD(line_p2p, offset_gain);
  FIELD(line_p2p, heading_gain);
  FIELD(line_p2p, steering_deadband);
  FIELD(line_p2p, in_place_turn_score);
  FIELD(line_p2p, ready_lead_sec);
  FIELD(line_p2p, observation_start_ratio);
  FIELD(line_p2p, failure_observation_sec);
  FIELD(line_p2p, recovery_observation_sec);
  FIELD(line_p2p, failure_min_valid_samples);
  FIELD(line_p2p, visible_recovery_max_turns);
  FIELD(line_p2p, last_score_recovery_max_turns);
  FIELD(line_p2p, recovery_max_turns);
  FIELD(ball, down_search_max_steps);
  FIELD(ball, down_search_inter_action_hold_sec);
  FIELD(ball, recovery_observation_sec);
  FIELD(ball, recovery_max_observations);
  FIELD(ball, recovery_walk_turn_max);
  FIELD(ball, recovery_max_steps);
  FIELD(ball, entry_recovery_max_steps);
  FIELD(ball, down_recovery_max_actions);
  FIELD(ball, post_pickup_back_steps);
  FIELD(ball, stable_window);
  FIELD(ball, stable_min_hits);
  FIELD(ball, down_search_min_hits);
  FIELD(ball, entry_min_hits);
  FIELD(ball, fine_side_two_step_margin_px);
  const auto fine_motion=(a["common"] ? a["ball"]["control"] : a["ball"])["fine_motion"];
  CheckField(fine_motion,"max_repetitions",c.ball.fine_motion.max_repetitions,"ball.fine_motion");
  CheckField(fine_motion,"forward_delta_u_px",c.ball.fine_motion.forward_delta_px.u,"ball.fine_motion");
  CheckField(fine_motion,"forward_delta_v_px",c.ball.fine_motion.forward_delta_px.v,"ball.fine_motion");
  CheckField(fine_motion,"backward_delta_u_px",c.ball.fine_motion.backward_delta_px.u,"ball.fine_motion");
  CheckField(fine_motion,"backward_delta_v_px",c.ball.fine_motion.backward_delta_px.v,"ball.fine_motion");
  CheckField(fine_motion,"left_delta_u_px",c.ball.fine_motion.left_delta_px.u,"ball.fine_motion");
  CheckField(fine_motion,"left_delta_v_px",c.ball.fine_motion.left_delta_px.v,"ball.fine_motion");
  CheckField(fine_motion,"right_delta_u_px",c.ball.fine_motion.right_delta_px.u,"ball.fine_motion");
  CheckField(fine_motion,"right_delta_v_px",c.ball.fine_motion.right_delta_px.v,"ball.fine_motion");
  FIELD(ball, lost_frames);
  FIELD(ball, upper_acquire_v_px);
  FIELD(ball, far_u_des_px);
  FIELD(ball, approach_u_deadband_px);
  FIELD(ball, approach_left_top_u_px);
  FIELD(ball, approach_right_top_u_px);
  FIELD(ball, approach_short_v_px);
  FIELD(ball, approach_two_step_v_px);
  FIELD(ball, approach_direct_down_v_px);
  FIELD(ball, pickup_u1_px);
  FIELD(ball, pickup_u2_px);
  FIELD(ball, pickup_v1_px);
  FIELD(ball, pickup_v2_px);
  FIELD(ball, pickup_max_attempts);
  FIELD(ball, pickup_verify_min_hits);
  FIELD(ball, recovery_center_tolerance_px);
  FIELD(hurdle, down_search_max_steps);
  FIELD(hurdle, down_search_inter_action_hold_sec);
  FIELD(hurdle, recovery_observation_sec);
  FIELD(hurdle, recovery_max_observations);
  FIELD(hurdle, entry_recovery_max_steps);
  FIELD(hurdle, recovery_max_steps);
  FIELD(hurdle, stable_window);
  FIELD(hurdle, stable_min_hits);
  FIELD(hurdle, down_search_min_hits);
  FIELD(hurdle, lost_frames);
  FIELD(hurdle, upper_acquire_v_px);
  FIELD(hurdle, approach_short_v_px);
  FIELD(hurdle, approach_two_step_v_px);
  FIELD(hurdle, target_u_px);
  FIELD(hurdle, down_one_step_v_px);
  FIELD(hurdle, fine_mode_v_px);
  FIELD(hurdle, contact_v_px);
  FIELD(hurdle, contact_steps);
  FIELD(hurdle, contact_inter_action_hold_sec);
  FIELD(hurdle, hurdle_ignore_duration_sec);
  FIELD(hurdle, recovery_center_tolerance_px);
  FIELD(goal, recovery_turn_max);
  FIELD(goal, depth_recovery_max_steps);
  FIELD(goal, edge_margin_px);
  FIELD(goal, recovery_observation_sec);
  FIELD(goal, recovery_max_observations);
  FIELD(goal, stable_window);
  FIELD(goal, stable_min_hits);
  FIELD(goal, lost_frames);
  FIELD(goal, smooth_alpha);
  FIELD(goal, post_pickup_walk_fallback_actions);
  FIELD(goal, entry_line_retry_max_attempts);
  FIELD(goal, target_u_px);
  FIELD(goal, approach_u_deadband_px);
  FIELD(goal, approach_short_z_m);
  FIELD(goal, fine_adjust_start_z_m);
  FIELD(goal, camera_goal_start_z_m);
  FIELD(goal, hoop_radius_m);
  FIELD(goal, throw_landing_x_m);
  FIELD(goal, throw_landing_z_m);
  FIELD(goal, position_tolerance_m);
  FIELD(goal, shoot_yaw_limit_deg);
  FIELD(goal, post_shoot_back_steps);
  FIELD(goal, recovery_center_tolerance_px);
  FIELD(line_detection, class_id);
  FIELD(line_detection, confidence);
  FIELD(line_detection, min_box_width);
  FIELD(line_detection, min_box_height);
  FIELD(object_targets, ball_class_id);
  FIELD(object_targets, goal_class_id);
  FIELD(object_targets, backboard_class_id);
  FIELD(object_targets, hurdle_class_id);
  FIELD(object_targets, ball_confidence);
  FIELD(object_targets, goal_confidence);
  FIELD(object_targets, backboard_confidence);
  FIELD(object_targets, hurdle_confidence);
  FIELD(object_targets, ball_min_box_width);
  FIELD(object_targets, ball_min_box_height);
  FIELD(object_targets, goal_min_box_width);
  FIELD(object_targets, goal_min_box_height);
  FIELD(object_targets, backboard_min_box_width);
  FIELD(object_targets, backboard_min_box_height);
  FIELD(object_targets, hurdle_min_box_width);
  FIELD(object_targets, hurdle_min_box_height);

  FIELD(object_targets, ball_aspect_ratio_min);
  FIELD(object_targets, ball_aspect_ratio_max);
  FIELD(object_targets, hurdle_aspect_ratio_min);
  FIELD(object_targets, ball_depth_enabled);
  FIELD(object_targets, hurdle_depth_enabled);
  FIELD(object_targets, ball_allow_missing_depth);
  FIELD(object_targets, hurdle_allow_missing_depth);
  FIELD(object_targets, ball_forward_min_depth_m);
  FIELD(object_targets, ball_forward_max_depth_m);
  FIELD(object_targets, ball_down_min_depth_m);
  FIELD(object_targets, ball_down_max_depth_m);
  FIELD(object_targets, hurdle_min_depth_m);
  FIELD(object_targets, hurdle_max_depth_m);
  FIELD(object_targets, depth_roi_size_px);
  FIELD(ball_candidate_tracking, enabled);
  FIELD(ball_candidate_tracking, max_center_distance_px);
  FIELD(ball_candidate_tracking, max_size_log_ratio);
  FIELD(ball_candidate_tracking, missing_frame_limit);
  FIELD(ball_candidate_tracking, max_tracks);
  FIELD(hurdle_candidate_tracking, enabled);
  FIELD(hurdle_candidate_tracking, max_center_distance_px);
  FIELD(hurdle_candidate_tracking, max_size_log_ratio);
  FIELD(hurdle_candidate_tracking, missing_frame_limit);
  FIELD(hurdle_candidate_tracking, max_tracks);
  FIELD(backboard_candidate_tracking, enabled);
  FIELD(backboard_candidate_tracking, max_center_distance_px);
  FIELD(backboard_candidate_tracking, max_size_log_ratio);
  FIELD(backboard_candidate_tracking, missing_frame_limit);
  FIELD(backboard_candidate_tracking, max_tracks);
  FIELD(object_association, missing_frame_limit);
  FIELD(object_association, max_center_distance_norm);
  FIELD(object_association, max_size_log_ratio);
  FIELD(object_association, center_distance_weight);
  FIELD(object_association, size_change_weight);
  FIELD(command, first_action_id);
  FIELD(command, action_ack_timeout_sec);
  CheckAlgorithmField(a, "mission", "test_mission", static_cast<int>(c.test_mission));
  MISSION(test_stage);
  CheckAlgorithmField(a, "goal", "backboard_depth_strip_width_px", c.backboard_depth_strip_width_px);
  CheckAlgorithmField(a, "goal", "trial_start_goal", c.goal_trial_start_goal);
  CheckAlgorithmField(a, "ball", "test_mission_index", c.test_ball_mission_index);
  CheckAlgorithmField(a, "goal", "test_mission_index", c.test_goal_mission_index);
  MISSION(candidate_search_max_turns);
  MISSION(mission_recovery_max_turns);
  MISSION(camera_motion_timeout_sec);
  MISSION(transition_settle_sec);
  MISSION(walk_inter_action_hold_sec);
  MISSION(fine_motion_settle_sec);
  MISSION(post_motion_observation_sec);
  MISSION(object_ready_mean_actions);
  CheckAlgorithmField(a, "goal", "backboard_min_depth_m", c.backboard_min_depth_m);
  CheckAlgorithmField(a, "goal", "backboard_max_depth_m", c.backboard_max_depth_m);
  MISSION(enable_ball);
  MISSION(enable_hurdle);
  MISSION(enable_goal);
  MISSION(initial_has_ball);
#undef MISSION
#undef FIELD
  std::cout << "PASS YAML validation and loaded values: " << path << '\n';
}

std::string ReadFile(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) throw std::runtime_error("cannot read: " + path);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}
} // namespace

int main(int argc, char** argv) {
  try {
    if (argc != 2 && !(argc == 4 && std::string(argv[2]) == "--installed"))
      throw std::runtime_error("usage: runtime_config_check SOURCE_YAML [--installed INSTALLED_YAML]");
    CheckConfiguration(argv[1]);
    if (argc == 4) {
      CheckConfiguration(argv[3]);
      if (ReadFile(argv[1]) != ReadFile(argv[3]))
        throw std::runtime_error("installed YAML differs from source; install the current config: " +
                                 std::string(argv[3]));
      std::cout << "PASS source/installed YAML contents match\n";
    }
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FAIL " << error.what() << '\n';
    return 1;
  }
}
