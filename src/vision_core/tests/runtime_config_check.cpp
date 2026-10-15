#include "vision_core/config_loader.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>

namespace {
template <class T>
void CheckField(const YAML::Node& node, const char* key, const T& actual,
                const std::string& section) {
  if (node[key].as<T>() != actual)
    throw std::runtime_error("YAML value not applied: " + section + "." + key);
}

void CheckConfiguration(const std::string& path) {
  // Use production loading/validation, then compare against the file's values,
  // not today's tuning constants. Do not implement a second mission decision.
  const auto c = vision_core::LoadAlgorithmConfig(path);
  const auto a = YAML::LoadFile(path)["vision_algorithm"];
#define FIELD(section, key) CheckField(a[#section], #key, c.section.key, #section)
#define MISSION(key) CheckField(a["mission"], #key, c.key, "mission")
  FIELD(line_features, max_centers);
  FIELD(line_features, guide_min_points);
  FIELD(line_features, image_center_u);
  FIELD(line_features, curve_min_points);
  FIELD(line_features, curve_min_v_span_px);
  FIELD(line_features, curve_local_fit_points);
  FIELD(line_features, guide_fit_rmse_full_scale_px);
  FIELD(line, line_stable_window);
  FIELD(line, line_stable_min_hits);
  FIELD(line_p2p, offset_gain);
  FIELD(line_p2p, heading_gain);
  FIELD(line_p2p, steering_deadband);
  FIELD(line_p2p, in_place_turn_score);
  FIELD(line_p2p, ready_lead_sec);
  FIELD(line_p2p, failure_observation_sec);
  FIELD(line_p2p, recovery_observation_sec);
  FIELD(line_p2p, failure_min_valid_samples);
  FIELD(line_p2p, no_evidence_max_retries);
  FIELD(line_p2p, recovery_max_turns);
  FIELD(line_p2p, recovery_turn_yaw_deg);
  FIELD(ball, down_search_max_steps);
  FIELD(ball, recovery_observation_sec);
  FIELD(ball, recovery_max_observations);
  FIELD(ball, recovery_walk_turn_max);
  FIELD(ball, post_pickup_back_steps);
  FIELD(ball, stable_window);
  FIELD(ball, stable_min_hits);
  FIELD(ball, down_search_min_hits);
  FIELD(ball, entry_min_hits);
  FIELD(ball, fine_side_two_step_margin_px);
  FIELD(ball, lost_frames);
  FIELD(ball, smooth_alpha);
  FIELD(ball, upper_acquire_v_px);
  FIELD(ball, far_u_des_px);
  FIELD(ball, approach_u_deadband_px);
  FIELD(ball, approach_short_v_px);
  FIELD(ball, approach_two_step_v_px);
  FIELD(ball, approach_direct_down_v_px);
  FIELD(ball, down_one_step_v_px);
  FIELD(ball, fine_mode_v_px);
  FIELD(ball, pickup_u1_px);
  FIELD(ball, pickup_u2_px);
  FIELD(ball, pickup_v1_px);
  FIELD(ball, pickup_v2_px);
  FIELD(ball, pickup_max_attempts);
  FIELD(ball, ball_ignore_duration_sec);
  FIELD(ball, recovery_timeout_sec);
  FIELD(ball, recovery_reacquire_min_hits);
  FIELD(ball, recovery_center_tolerance_px);
  FIELD(hurdle, down_search_max_steps);
  FIELD(hurdle, recovery_observation_sec);
  FIELD(hurdle, recovery_max_observations);
  FIELD(hurdle, stable_window);
  FIELD(hurdle, stable_min_hits);
  FIELD(hurdle, down_search_min_hits);
  FIELD(hurdle, lost_frames);
  FIELD(hurdle, smooth_alpha);
  FIELD(hurdle, upper_acquire_v_px);
  FIELD(hurdle, approach_short_v_px);
  FIELD(hurdle, approach_two_step_v_px);
  FIELD(hurdle, target_u_px);
  FIELD(hurdle, u_deadband_px);
  FIELD(hurdle, down_one_step_v_px);
  FIELD(hurdle, fine_mode_v_px);
  FIELD(hurdle, contact_v_px);
  FIELD(hurdle, contact_steps);
  FIELD(hurdle, hurdle_ignore_duration_sec);
  FIELD(hurdle, recovery_reacquire_min_hits);
  FIELD(hurdle, recovery_timeout_sec);
  FIELD(hurdle, recovery_center_tolerance_px);
  FIELD(goal, recovery_turn_max);
  FIELD(goal, edge_margin_px);
  FIELD(goal, recovery_observation_sec);
  FIELD(goal, recovery_max_observations);
  FIELD(goal, stable_window);
  FIELD(goal, stable_min_hits);
  FIELD(goal, lost_frames);
  FIELD(goal, smooth_alpha);
  FIELD(goal, post_pickup_walk_fallback_actions);
  FIELD(goal, target_u_px);
  FIELD(goal, approach_u_deadband_px);
  FIELD(goal, approach_short_z_m);
  FIELD(goal, fine_adjust_start_z_m);
  FIELD(goal, camera_goal_start_z_m);
  FIELD(goal, fine_two_step_range_error_m);
  FIELD(goal, fine_one_step_range_error_m);
  FIELD(goal, hoop_radius_m);
  FIELD(goal, throw_landing_x_m);
  FIELD(goal, throw_landing_z_m);
  FIELD(goal, position_tolerance_m);
  FIELD(goal, shoot_yaw_limit_deg);
  FIELD(goal, post_shoot_back_steps);
  FIELD(goal, recovery_timeout_sec);
  FIELD(goal, recovery_reacquire_min_hits);
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

  FIELD(object_association, missing_frame_limit);
  FIELD(object_association, max_center_distance_norm);
  FIELD(object_association, max_size_log_ratio);
  FIELD(object_association, center_distance_weight);
  FIELD(object_association, size_change_weight);
  FIELD(command, first_action_id);
  FIELD(command, action_ack_timeout_sec);
  CheckField(a["mission"], "test_mission", static_cast<int>(c.test_mission), "mission");
  MISSION(test_stage);
  CheckField(a["goal"], "backboard_depth_strip_width_px", c.backboard_depth_strip_width_px, "goal");
  CheckField(a["goal"], "trial_start_goal", c.goal_trial_start_goal, "goal");
  CheckField(a["ball"], "test_mission_index", c.test_ball_mission_index, "ball");
  CheckField(a["goal"], "test_mission_index", c.test_goal_mission_index, "goal");
  MISSION(mission_recovery_max_turns);
  MISSION(mission_recovery_yaw_deg);
  MISSION(camera_motion_timeout_sec);
  MISSION(transition_settle_sec);
  MISSION(walk_inter_action_hold_sec);
  MISSION(fine_motion_settle_sec);
  MISSION(post_motion_observation_sec);
  MISSION(object_ready_mean_actions);
  CheckField(a["goal"], "backboard_min_depth_m", c.backboard_min_depth_m, "goal");
  CheckField(a["goal"], "backboard_max_depth_m", c.backboard_max_depth_m, "goal");
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
