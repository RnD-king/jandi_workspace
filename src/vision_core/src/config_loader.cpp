#include "vision_core/config_loader.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <utility>

#include <yaml-cpp/yaml.h>

#ifndef VISION_CORE_DEFAULT_ALGORITHM_CONFIG
#define VISION_CORE_DEFAULT_ALGORITHM_CONFIG "vision_algorithm.yaml"
#endif

namespace vision_core {
namespace {
template <typename T>
void Assign(const YAML::Node &node, const char *key, T &value) {
  if (!node || !node.IsMap() || !node[key])
    throw std::runtime_error(std::string("missing required vision algorithm key: ") + key);
  try { value = node[key].as<T>(); }
  catch (const YAML::Exception &error) {
    throw std::runtime_error(std::string("invalid vision algorithm key '") +
                             key + "': " + error.what());
  }
}

// Grouping is a file-layout concern. Keep the internal configuration and
// controller contracts unchanged, and continue accepting existing flat YAMLs.
YAML::Node AlgorithmSections(const YAML::Node &a) {
  if (!a["common"]) return a;
  const auto map = [](const YAML::Node &parent, const char *key) {
    const auto node = parent[key];
    if (!node || !node.IsMap())
      throw std::runtime_error(std::string("missing required vision algorithm map: ") + key);
    return node;
  };
  YAML::Node sections;
  const auto line = map(a, "line");
  sections["line_detection"] = map(line, "detection");
  sections["line_features"] = map(line, "features");
  sections["line"] = map(line, "stability");
  sections["line_p2p"] = map(line, "walk");
  const auto common = map(a, "common");
  sections["command"] = map(common, "command");
  sections["object_association"] = map(common, "object_association");
  sections["mission"] = YAML::Clone(map(common, "mission"));
  for (const auto *name : {"ball", "hurdle", "goal"}) {
    const auto object = map(a, name);
    sections[name] = YAML::Clone(map(object, "control"));
    bool enabled=false;
    Assign(object, "enabled", enabled);
    sections["mission"][std::string("enable_") + name] = enabled;
  }
  for (const auto *name : {"ball", "hurdle", "goal", "backboard"}) {
    const auto object = std::string(name) == "backboard"
        ? map(map(a, "goal"), "backboard") : map(a, name);
    const auto detection = map(object, "detection");
    for (const auto &field : detection)
      sections["object_targets"][std::string(name) + "_" + field.first.as<std::string>()] = field.second;
    if (std::string(name) != "goal")
      sections[std::string(name) + "_candidate_tracking"] = map(object, "candidate_tracking");
  }
  int depth_roi_size_px=0;
  Assign(common, "depth_roi_size_px", depth_roi_size_px);
  sections["object_targets"]["depth_roi_size_px"] = depth_roi_size_px;
  const auto board_depth = map(map(map(a, "goal"), "backboard"), "depth");
  for (const auto *key : {"depth_strip_width_px", "min_depth_m", "max_depth_m"})
    sections["goal"][std::string("backboard_") + key] = board_depth[key];
  return sections;
}
void Finite(double value, const char *key) {
  if (!std::isfinite(value))
    throw std::runtime_error(std::string("vision algorithm key '") + key + "' must be finite");
}
void Nonnegative(double value, const char *key) {
  if (!std::isfinite(value) || value < 0.0)
    throw std::runtime_error(std::string("vision algorithm key '") + key +
                             "' must be finite and >= 0");
}
void Unit(double value, const char *key) {
  if (!std::isfinite(value) || value < 0.0 || value > 1.0)
    throw std::runtime_error(std::string("vision algorithm key '") + key +
                             "' must be finite and in [0, 1]");
}
void NonnegativeInt(int value, const char *key) {
  if (value < 0)
    throw std::runtime_error(std::string("vision algorithm key '") + key +
                             "' must be >= 0");
}
void Positive(double value, const char *key) {
  if (!std::isfinite(value) || value <= 0.0)
    throw std::runtime_error(std::string("vision algorithm key '") + key +
                             "' must be finite and > 0");
}
void PositiveInt(int value, const char *key) {
  if (value <= 0)
    throw std::runtime_error(std::string("vision algorithm key '") + key +
                             "' must be > 0");
}
void HitsWithinWindow(int hits, int window, const char *hits_key,
                      const char *window_key) {
  PositiveInt(window, window_key);
  PositiveInt(hits, hits_key);
  if (hits > window)
    throw std::runtime_error(std::string("vision algorithm key '") + hits_key +
                             "' must be <= '" + window_key + "'");
}
} // namespace

void ValidateAlgorithmConfig(const MissionControllerConfig &c) {
  PositiveInt(c.line_p2p.left_repetitions, "line_p2p.left_repetitions");
  PositiveInt(c.line_p2p.straight_repetitions, "line_p2p.straight_repetitions");
  PositiveInt(c.line_p2p.right_repetitions, "line_p2p.right_repetitions");
  NonnegativeInt(c.candidate_search_max_turns, "mission.candidate_search_max_turns");
  Nonnegative(c.ball.down_one_step_v_px, "ball.down_one_step_v_px");
  Nonnegative(c.ball.fine_mode_v_px, "ball.fine_mode_v_px");
  if (c.ball.down_one_step_v_px >= c.ball.fine_mode_v_px)
    throw std::runtime_error("ball DOWN thresholds must be ordered");
  Nonnegative(c.ball.approach_short_v_px, "ball.approach_short_v_px");
  PositiveInt(c.ball.down_search_max_steps, "ball.down_search_max_steps");
  Nonnegative(c.ball.down_search_inter_action_hold_sec, "ball.down_search_inter_action_hold_sec");
  Positive(c.ball.recovery_observation_sec, "ball.recovery_observation_sec");
  PositiveInt(c.ball.recovery_max_observations, "ball.recovery_max_observations");
  NonnegativeInt(c.ball.recovery_walk_turn_max, "ball.recovery_walk_turn_max");
  NonnegativeInt(c.ball.entry_recovery_walk_turn_max, "ball.entry_recovery_walk_turn_max");
  NonnegativeInt(c.goal.recovery_turn_max, "goal.recovery_turn_max");
  Nonnegative(c.goal.edge_margin_px, "goal.edge_margin_px");
  Nonnegative(c.transition_settle_sec, "mission.transition_settle_sec");
  Nonnegative(c.walk_inter_action_hold_sec, "mission.walk_inter_action_hold_sec");
  Nonnegative(c.fine_motion_settle_sec, "mission.fine_motion_settle_sec");
  NonnegativeInt(c.ball.post_pickup_back_steps, "ball.post_pickup_back_steps");
  Nonnegative(c.hurdle.down_one_step_v_px, "hurdle.down_one_step_v_px");
  Nonnegative(c.hurdle.fine_mode_v_px, "hurdle.fine_mode_v_px");
  if (c.hurdle.down_one_step_v_px >= c.hurdle.fine_mode_v_px)
    throw std::runtime_error("hurdle DOWN thresholds must be ordered");
  Nonnegative(c.hurdle.approach_short_v_px, "hurdle.approach_short_v_px");
  PositiveInt(c.hurdle.down_search_max_steps, "hurdle.down_search_max_steps");
  Nonnegative(c.hurdle.down_search_inter_action_hold_sec, "hurdle.down_search_inter_action_hold_sec");
  Positive(c.hurdle.recovery_observation_sec, "hurdle.recovery_observation_sec");
  PositiveInt(c.hurdle.recovery_max_observations, "hurdle.recovery_max_observations");
  Nonnegative(c.hurdle.target_u_px, "hurdle.target_u_px");
  Nonnegative(c.hurdle.contact_v_px, "hurdle.contact_v_px");
  PositiveInt(c.hurdle.contact_steps, "hurdle.contact_steps");
  Nonnegative(c.hurdle.contact_inter_action_hold_sec, "hurdle.contact_inter_action_hold_sec");
  Nonnegative(c.goal.fine_one_step_range_error_m,"goal.fine_one_step_range_error_m");
  Nonnegative(c.goal.fine_two_step_range_error_m,"goal.fine_two_step_range_error_m");
  Positive(c.goal.recovery_observation_sec, "goal.recovery_observation_sec");
  PositiveInt(c.goal.recovery_max_observations, "goal.recovery_max_observations");
  Positive(c.goal.approach_short_z_m, "goal.approach_short_z_m");
  Positive(c.goal.camera_goal_start_z_m, "goal.camera_goal_start_z_m");
  NonnegativeInt(c.goal.post_shoot_back_steps, "goal.post_shoot_back_steps");
  if (c.ball.approach_short_v_px >= c.ball.approach_two_step_v_px ||
      c.hurdle.approach_short_v_px >= c.hurdle.approach_two_step_v_px ||
      c.goal.camera_goal_start_z_m > c.goal.approach_short_z_m ||
      c.goal.fine_one_step_range_error_m > c.goal.fine_two_step_range_error_m)
    throw std::runtime_error("approach/fine transition thresholds must be ordered");
  const int tm=static_cast<int>(c.test_mission);
  if ((tm!=0 && tm!=2 && tm!=3 && tm!=4) || c.test_stage<0 || c.test_stage>3 ||
      (tm==0 && c.test_stage!=0)) throw std::runtime_error("invalid test mission/stage");
  PositiveInt(c.backboard_depth_strip_width_px,"goal.backboard_depth_strip_width_px");
  PositiveInt(c.test_ball_mission_index,"ball.test_mission_index");
  PositiveInt(c.test_goal_mission_index,"goal.test_mission_index");
  PositiveInt(c.mission_recovery_max_turns,"mission.mission_recovery_max_turns");
  PositiveInt(c.mission_recovery_yaw_deg,"mission.mission_recovery_yaw_deg");
  if (c.mission_recovery_yaw_deg>180) throw std::runtime_error("recovery yaw exceeds 180 degrees");
  // An invalid ID would never receive READY from forward_walk. Reject such
  // YAML errors at startup rather than silently doing a post-DONE fallback.
  const std::array<int, 8> ready_capable_actions = {
      static_cast<int>(MissionAction::kStepForwardLeft),
      static_cast<int>(MissionAction::kStepForwardRight),
      static_cast<int>(MissionAction::kStepForwardFive),
      static_cast<int>(MissionAction::kStepForwardTwo),
      static_cast<int>(MissionAction::kStepForwardThree),
      static_cast<int>(MissionAction::kStepForwardFour),
      static_cast<int>(MissionAction::kStepForwardLeftTwo),
      static_cast<int>(MissionAction::kStepForwardRightTwo)};
  for (std::size_t i = 0; i < c.object_ready_mean_actions.size(); ++i) {
    const int action = c.object_ready_mean_actions[i];
    if (std::find(ready_capable_actions.begin(), ready_capable_actions.end(),
                  action) == ready_capable_actions.end())
      throw std::runtime_error(
          "mission.object_ready_mean_actions contains an action without READY support");
    if (std::find(c.object_ready_mean_actions.begin(),
                  c.object_ready_mean_actions.begin() + i, action) !=
        c.object_ready_mean_actions.begin() + i)
      throw std::runtime_error("mission.object_ready_mean_actions has duplicate action IDs");
  }

  PositiveInt(c.line_features.max_centers, "line_features.max_centers");
  if (c.line_features.guide_min_points < 3 ||
      c.line_features.guide_min_points > c.line_features.max_centers)
    throw std::runtime_error(
        "line_features.guide_min_points must be >= 3 and <= max_centers");
  if (!std::isfinite(c.line_features.image_center_u))
    throw std::runtime_error(
        "vision algorithm key 'line_features.image_center_u' must be finite");
  PositiveInt(c.line_features.curve_min_points,
              "line_features.curve_min_points");
  Nonnegative(c.line_features.curve_min_v_span_px,
              "line_features.curve_min_v_span_px");
  PositiveInt(c.line_features.curve_local_fit_points,
              "line_features.curve_local_fit_points");
  Positive(c.line_features.guide_fit_rmse_full_scale_px,
           "line_features.guide_fit_rmse_full_scale_px");
  if (c.line_features.ransac_min_input_points < 4 ||
      c.line_features.ransac_min_input_points > 8)
    throw std::runtime_error("line_features.ransac_min_input_points must be in [4,8]");
  Positive(c.line_features.ransac_residual_threshold_px,
           "line_features.ransac_residual_threshold_px");
  Positive(c.line_features.ransac_min_sample_v_gap_px,
           "line_features.ransac_min_sample_v_gap_px");
  if (c.line_features.ransac_max_outliers < 0 ||
      c.line_features.ransac_max_outliers > 2)
    throw std::runtime_error("line_features.ransac_max_outliers must be in [0,2]");

  HitsWithinWindow(c.line.line_stable_min_hits, c.line.line_stable_window,
                   "line.line_stable_min_hits", "line.line_stable_window");

  Nonnegative(c.line_p2p.offset_gain, "line_p2p.offset_gain");
  Nonnegative(c.line_p2p.heading_gain, "line_p2p.heading_gain");
  Nonnegative(c.line_p2p.steering_deadband, "line_p2p.steering_deadband");
  Positive(c.line_p2p.in_place_turn_score,"line_p2p.in_place_turn_score");
  if (c.line_p2p.in_place_turn_score <= c.line_p2p.steering_deadband)
    throw std::runtime_error("line_p2p.in_place_turn_score must exceed steering_deadband");
  Nonnegative(c.line_p2p.ready_lead_sec, "line_p2p.ready_lead_sec");
  Nonnegative(c.line_p2p.failure_observation_sec,
              "line_p2p.failure_observation_sec");
  Nonnegative(c.line_p2p.recovery_observation_sec,
              "line_p2p.recovery_observation_sec");
  PositiveInt(c.line_p2p.failure_min_valid_samples,
              "line_p2p.failure_min_valid_samples");
  NonnegativeInt(c.line_p2p.no_evidence_max_retries,
                 "line_p2p.no_evidence_max_retries");
  NonnegativeInt(c.line_p2p.recovery_max_turns,
                 "line_p2p.recovery_max_turns");
  if (c.line_p2p.recovery_turn_yaw_deg < 1 ||
      c.line_p2p.recovery_turn_yaw_deg > 180)
    throw std::runtime_error(
        "vision algorithm key 'line_p2p.recovery_turn_yaw_deg' must be in "
        "[1, 180]");

  HitsWithinWindow(c.ball.entry_min_hits, c.ball.stable_window,
                   "ball.entry_min_hits", "ball.stable_window");
  HitsWithinWindow(c.ball.down_search_min_hits,c.ball.stable_window,
                   "ball.down_search_min_hits","ball.stable_window");
  HitsWithinWindow(c.ball.stable_min_hits, c.ball.stable_window,
                   "ball.stable_min_hits", "ball.stable_window");
  PositiveInt(c.ball.lost_frames, "ball.lost_frames");
  Nonnegative(c.ball.far_u_des_px, "ball.far_u_des_px");
  Nonnegative(c.ball.approach_u_deadband_px, "ball.approach_u_deadband_px");
  Nonnegative(c.ball.approach_left_top_u_px, "ball.approach_left_top_u_px");
  Nonnegative(c.ball.approach_right_top_u_px, "ball.approach_right_top_u_px");
  if (c.ball.approach_left_top_u_px>=c.ball.approach_right_top_u_px)
    throw std::runtime_error("ball approach top endpoints must be ordered left < right");
  Nonnegative(c.ball.upper_acquire_v_px, "ball.upper_acquire_v_px");
  Nonnegative(c.ball.fine_side_two_step_margin_px, "ball.fine_side_two_step_margin_px");
  if (c.ball.fine_motion.max_repetitions<1 || c.ball.fine_motion.max_repetitions>3)
    throw std::runtime_error("ball.fine_motion.max_repetitions must be in [1,3]");
  const std::array<std::pair<const char*,Point2>,4> fine_effects{{
      {"forward",c.ball.fine_motion.forward_delta_px},
      {"backward",c.ball.fine_motion.backward_delta_px},
      {"left",c.ball.fine_motion.left_delta_px},{"right",c.ball.fine_motion.right_delta_px}}};
  for (const auto& effect : fine_effects) {
    Finite(effect.second.u,(std::string("ball.fine_motion.")+effect.first+"_delta_u_px").c_str());
    Finite(effect.second.v,(std::string("ball.fine_motion.")+effect.first+"_delta_v_px").c_str());
  }
  Nonnegative(c.ball.approach_two_step_v_px, "ball.approach_two_step_v_px");
  Nonnegative(c.ball.approach_direct_down_v_px, "ball.approach_direct_down_v_px");
  if (c.ball.approach_direct_down_v_px <= c.ball.approach_two_step_v_px)
    throw std::runtime_error("ball.approach_direct_down_v_px must exceed ball.approach_two_step_v_px");
  PositiveInt(c.ball.pickup_max_attempts, "ball.pickup_max_attempts");
  HitsWithinWindow(c.ball.pickup_verify_min_hits, c.ball.stable_window,
                   "ball.pickup_verify_min_hits", "ball.stable_window");
  Nonnegative(c.ball.pickup_u1_px, "ball.pickup_u1_px");
  Nonnegative(c.ball.pickup_u2_px, "ball.pickup_u2_px");
  Nonnegative(c.ball.pickup_v1_px, "ball.pickup_v1_px");
  Nonnegative(c.ball.pickup_v2_px, "ball.pickup_v2_px");
  if (c.ball.pickup_u1_px >= c.ball.pickup_u2_px ||
      c.ball.pickup_v1_px >= c.ball.pickup_v2_px)
    throw std::runtime_error("ball pickup edges require u1 < u2 and v1 < v2");
  Nonnegative(c.ball.recovery_center_tolerance_px,
       "ball.recovery_center_tolerance_px");

  HitsWithinWindow(c.hurdle.down_search_min_hits,c.hurdle.stable_window,
                   "hurdle.down_search_min_hits","hurdle.stable_window");
  HitsWithinWindow(c.hurdle.stable_min_hits, c.hurdle.stable_window,
                   "hurdle.stable_min_hits", "hurdle.stable_window");
  PositiveInt(c.hurdle.lost_frames, "hurdle.lost_frames");
  Nonnegative(c.hurdle.upper_acquire_v_px, "hurdle.upper_acquire_v_px");
  Nonnegative(c.hurdle.approach_two_step_v_px, "hurdle.approach_two_step_v_px");
  Nonnegative(c.hurdle.hurdle_ignore_duration_sec,
              "hurdle.hurdle_ignore_duration_sec");
  Nonnegative(c.hurdle.recovery_center_tolerance_px,
       "hurdle.recovery_center_tolerance_px");

  HitsWithinWindow(c.goal.stable_min_hits, c.goal.stable_window,
                   "goal.stable_min_hits", "goal.stable_window");
  PositiveInt(c.goal.lost_frames, "goal.lost_frames");
  Unit(c.goal.smooth_alpha, "goal.smooth_alpha");
  NonnegativeInt(c.goal.post_pickup_walk_fallback_actions, "goal.post_pickup_walk_fallback_actions");
  NonnegativeInt(c.goal.entry_line_retry_max_attempts, "goal.entry_line_retry_max_attempts");
  Nonnegative(c.goal.target_u_px, "goal.target_u_px");
  Nonnegative(c.goal.approach_u_deadband_px, "goal.approach_u_deadband_px");
  Nonnegative(c.goal.fine_adjust_start_z_m, "goal.fine_adjust_start_z_m");
  Nonnegative(c.goal.hoop_radius_m, "goal.hoop_radius_m");
  Finite(c.goal.throw_landing_x_m, "goal.throw_landing_x_m");
  Positive(c.goal.throw_landing_z_m, "goal.throw_landing_z_m");
  Nonnegative(c.goal.position_tolerance_m, "goal.position_tolerance_m");
  Nonnegative(c.goal.shoot_yaw_limit_deg, "goal.shoot_yaw_limit_deg");
  Nonnegative(c.goal.recovery_center_tolerance_px,
       "goal.recovery_center_tolerance_px");

  NonnegativeInt(c.line_detection.class_id, "line_detection.class_id");
  Unit(c.line_detection.confidence, "line_detection.confidence");
  Nonnegative(c.line_detection.min_box_width,
              "line_detection.min_box_width");
  Nonnegative(c.line_detection.min_box_height,
              "line_detection.min_box_height");

  const std::array<int, 5> class_ids = {
      c.line_detection.class_id, c.object_targets.ball_class_id,
      c.object_targets.goal_class_id, c.object_targets.backboard_class_id,
      c.object_targets.hurdle_class_id};
  for (std::size_t i = 0; i < class_ids.size(); ++i) {
    if (class_ids[i] < 0)
      throw std::runtime_error(
          "vision algorithm class IDs must be >= 0");
    for (std::size_t j = i + 1; j < class_ids.size(); ++j) {
      if (class_ids[i] == class_ids[j])
        throw std::runtime_error(
            "vision algorithm class IDs must be unique");
    }
  }
  Unit(c.object_targets.ball_confidence,
       "object_targets.ball_confidence");
  Unit(c.object_targets.goal_confidence,
       "object_targets.goal_confidence");
  Unit(c.object_targets.backboard_confidence,
       "object_targets.backboard_confidence");
  Unit(c.object_targets.hurdle_confidence,
       "object_targets.hurdle_confidence");
  Nonnegative(c.object_targets.ball_min_box_width,"object_targets.ball_min_box_width");
  Nonnegative(c.object_targets.ball_min_box_height,"object_targets.ball_min_box_height");
  Nonnegative(c.object_targets.goal_min_box_width,"object_targets.goal_min_box_width");
  Nonnegative(c.object_targets.goal_min_box_height,"object_targets.goal_min_box_height");
  Nonnegative(c.object_targets.backboard_min_box_width,"object_targets.backboard_min_box_width");
  Nonnegative(c.object_targets.backboard_min_box_height,"object_targets.backboard_min_box_height");
  Nonnegative(c.object_targets.hurdle_min_box_width,"object_targets.hurdle_min_box_width");
  Nonnegative(c.object_targets.hurdle_min_box_height,"object_targets.hurdle_min_box_height");


  Nonnegative(c.object_targets.ball_aspect_ratio_min,"object_targets.ball_aspect_ratio_min");
  Nonnegative(c.object_targets.ball_aspect_ratio_max,"object_targets.ball_aspect_ratio_max");
  Nonnegative(c.object_targets.hurdle_aspect_ratio_min,"object_targets.hurdle_aspect_ratio_min");
  Nonnegative(c.object_targets.ball_forward_min_depth_m,"object_targets.ball_forward_min_depth_m");
  Nonnegative(c.object_targets.ball_forward_max_depth_m,"object_targets.ball_forward_max_depth_m");
  Nonnegative(c.object_targets.ball_down_min_depth_m,"object_targets.ball_down_min_depth_m");
  Nonnegative(c.object_targets.ball_down_max_depth_m,"object_targets.ball_down_max_depth_m");
  Nonnegative(c.object_targets.hurdle_min_depth_m,"object_targets.hurdle_min_depth_m");
  Nonnegative(c.object_targets.hurdle_max_depth_m,"object_targets.hurdle_max_depth_m");
  if(c.object_targets.ball_aspect_ratio_min<=0 || c.object_targets.ball_aspect_ratio_min>c.object_targets.ball_aspect_ratio_max ||
      c.object_targets.hurdle_aspect_ratio_min<=0)
    throw std::runtime_error("invalid BALL/HURDLE aspect ratio range");
  if(c.object_targets.ball_forward_min_depth_m>c.object_targets.ball_forward_max_depth_m ||
      c.object_targets.ball_down_min_depth_m>c.object_targets.ball_down_max_depth_m ||
      c.object_targets.hurdle_min_depth_m>c.object_targets.hurdle_max_depth_m ||
      c.object_targets.ball_forward_max_depth_m<=0 || c.object_targets.ball_down_max_depth_m<=0 || c.object_targets.hurdle_max_depth_m<=0)
    throw std::runtime_error("invalid BALL/HURDLE depth range");
  if(c.object_targets.depth_roi_size_px<3 || c.object_targets.depth_roi_size_px>31 || c.object_targets.depth_roi_size_px%2==0)
    throw std::runtime_error("depth_roi_size_px must be odd in [3,31]");
  Positive(c.ball_candidate_tracking.max_center_distance_px,"ball_candidate_tracking.max_center_distance_px");
  Positive(c.ball_candidate_tracking.max_size_log_ratio,"ball_candidate_tracking.max_size_log_ratio");
  NonnegativeInt(c.ball_candidate_tracking.missing_frame_limit,"ball_candidate_tracking.missing_frame_limit");
  if(c.ball_candidate_tracking.max_tracks<1 || c.ball_candidate_tracking.max_tracks>128)
    throw std::runtime_error("ball_candidate_tracking.max_tracks must be in [1,128]");
  for (const auto& item : {std::make_pair("hurdle_candidate_tracking", &c.hurdle_candidate_tracking),
                           std::make_pair("backboard_candidate_tracking", &c.backboard_candidate_tracking)}) {
    const std::string prefix = item.first;
    const auto& tracking = *item.second;
    Positive(tracking.max_center_distance_px, (prefix + ".max_center_distance_px").c_str());
    Positive(tracking.max_size_log_ratio, (prefix + ".max_size_log_ratio").c_str());
    NonnegativeInt(tracking.missing_frame_limit, (prefix + ".missing_frame_limit").c_str());
    if (tracking.max_tracks < 1 || tracking.max_tracks > 128)
      throw std::runtime_error(prefix + ".max_tracks must be in [1,128]");
  }

  NonnegativeInt(c.object_association.missing_frame_limit,
                 "object_association.missing_frame_limit");
  // pixel center distance / image diagonal이므로 화면 내부에서는 일반적으로
  // 0..1이지만, 1보다 큰 값도 gate 완화 용도로 허용한다.
  Nonnegative(c.object_association.max_center_distance_norm,
              "object_association.max_center_distance_norm");
  Nonnegative(c.object_association.max_size_log_ratio,
              "object_association.max_size_log_ratio");
  Nonnegative(c.object_association.center_distance_weight,
              "object_association.center_distance_weight");
  Nonnegative(c.object_association.size_change_weight,
              "object_association.size_change_weight");

  if (c.command.first_action_id == 0)
    throw std::runtime_error(
        "vision algorithm key 'command.first_action_id' must be > 0");
  Nonnegative(c.command.action_ack_timeout_sec,
              "command.action_ack_timeout_sec");

  Positive(c.backboard_min_depth_m, "goal.backboard_min_depth_m");
  Nonnegative(c.camera_motion_timeout_sec, "mission.camera_motion_timeout_sec");
  Positive(c.post_motion_observation_sec,
              "mission.post_motion_observation_sec");
  Positive(c.backboard_max_depth_m, "goal.backboard_max_depth_m");
  if (c.backboard_min_depth_m > c.backboard_max_depth_m)
    throw std::runtime_error(
        "vision algorithm key 'goal.backboard_min_depth_m' must be <= "
        "'goal.backboard_max_depth_m'");
}

std::string DefaultAlgorithmConfigPath() {
  if (const char *path = std::getenv("VISION_CORE_ALGORITHM_CONFIG");
      path && path[0] != '\0') return path;
  return VISION_CORE_DEFAULT_ALGORITHM_CONFIG;
}

MissionControllerConfig LoadAlgorithmConfig(const std::string &path) {
  MissionControllerConfig c{};
  YAML::Node root;
  try { root = YAML::LoadFile(path); }
  catch (const YAML::Exception &error) {
    throw std::runtime_error("failed to load vision algorithm config '" +
                             path + "': " + error.what());
  }
  const YAML::Node raw = root["vision_algorithm"];
  if (!raw || !raw.IsMap())
    throw std::runtime_error("vision algorithm config must contain a vision_algorithm map: " + path);
  const YAML::Node a = AlgorithmSections(raw);

  const auto f = a["line_features"];
  Assign(f, "max_centers", c.line_features.max_centers);
  Assign(f, "guide_min_points", c.line_features.guide_min_points);
  Assign(f, "image_center_u", c.line_features.image_center_u);
  Assign(f, "curve_min_points", c.line_features.curve_min_points);
  Assign(f, "curve_min_v_span_px", c.line_features.curve_min_v_span_px);
  Assign(f, "curve_local_fit_points", c.line_features.curve_local_fit_points);
  Assign(f, "guide_fit_rmse_full_scale_px", c.line_features.guide_fit_rmse_full_scale_px);
  Assign(f, "ransac_enabled", c.line_features.ransac_enabled);
  Assign(f, "ransac_min_input_points", c.line_features.ransac_min_input_points);
  Assign(f, "ransac_residual_threshold_px", c.line_features.ransac_residual_threshold_px);
  Assign(f, "ransac_min_sample_v_gap_px", c.line_features.ransac_min_sample_v_gap_px);
  Assign(f, "ransac_max_outliers", c.line_features.ransac_max_outliers);

  const auto line = a["line"];
  Assign(line, "line_stable_window", c.line.line_stable_window);
  Assign(line, "line_stable_min_hits", c.line.line_stable_min_hits);

  const auto lp = a["line_p2p"];
  Assign(lp, "left_repetitions", c.line_p2p.left_repetitions);
  Assign(lp, "straight_repetitions", c.line_p2p.straight_repetitions);
  Assign(lp, "right_repetitions", c.line_p2p.right_repetitions);
  Assign(lp, "offset_gain", c.line_p2p.offset_gain);
  Assign(lp, "heading_gain", c.line_p2p.heading_gain);
  Assign(lp, "steering_deadband", c.line_p2p.steering_deadband);
  Assign(lp,"in_place_turn_score",c.line_p2p.in_place_turn_score);
  Assign(lp, "ready_lead_sec", c.line_p2p.ready_lead_sec);
  Assign(lp, "failure_observation_sec", c.line_p2p.failure_observation_sec);
  Assign(lp, "recovery_observation_sec", c.line_p2p.recovery_observation_sec);
  Assign(lp, "failure_min_valid_samples", c.line_p2p.failure_min_valid_samples);
  Assign(lp, "no_evidence_max_retries", c.line_p2p.no_evidence_max_retries);
  Assign(lp, "recovery_max_turns", c.line_p2p.recovery_max_turns);
  Assign(lp, "recovery_turn_yaw_deg", c.line_p2p.recovery_turn_yaw_deg);

  const auto b = a["ball"];
  Assign(b, "test_mission_index", c.test_ball_mission_index);
  Assign(b, "down_one_step_v_px", c.ball.down_one_step_v_px);
  Assign(b, "fine_mode_v_px", c.ball.fine_mode_v_px);
  Assign(b, "approach_short_v_px", c.ball.approach_short_v_px);
  Assign(b, "down_search_max_steps", c.ball.down_search_max_steps);
  Assign(b, "down_search_inter_action_hold_sec", c.ball.down_search_inter_action_hold_sec);
  Assign(b, "recovery_observation_sec", c.ball.recovery_observation_sec);
  Assign(b, "recovery_max_observations", c.ball.recovery_max_observations);
  Assign(b, "recovery_walk_turn_max", c.ball.recovery_walk_turn_max);
  Assign(b, "entry_recovery_walk_turn_max", c.ball.entry_recovery_walk_turn_max);
  Assign(b, "post_pickup_back_steps", c.ball.post_pickup_back_steps);

  Assign(b, "stable_window", c.ball.stable_window);
  Assign(b, "stable_min_hits", c.ball.stable_min_hits);
  Assign(b,"down_search_min_hits",c.ball.down_search_min_hits);
  Assign(b, "entry_min_hits", c.ball.entry_min_hits);
  Assign(b, "lost_frames", c.ball.lost_frames);
  Assign(b, "far_u_des_px", c.ball.far_u_des_px);
  Assign(b, "approach_u_deadband_px", c.ball.approach_u_deadband_px);
  Assign(b, "approach_left_top_u_px", c.ball.approach_left_top_u_px);
  Assign(b, "approach_right_top_u_px", c.ball.approach_right_top_u_px);
  Assign(b, "upper_acquire_v_px", c.ball.upper_acquire_v_px);
  Assign(b, "fine_side_two_step_margin_px", c.ball.fine_side_two_step_margin_px);
  const auto fine_motion=b["fine_motion"];
  Assign(fine_motion,"max_repetitions",c.ball.fine_motion.max_repetitions);
  Assign(fine_motion,"forward_delta_u_px",c.ball.fine_motion.forward_delta_px.u);
  Assign(fine_motion,"forward_delta_v_px",c.ball.fine_motion.forward_delta_px.v);
  Assign(fine_motion,"backward_delta_u_px",c.ball.fine_motion.backward_delta_px.u);
  Assign(fine_motion,"backward_delta_v_px",c.ball.fine_motion.backward_delta_px.v);
  Assign(fine_motion,"left_delta_u_px",c.ball.fine_motion.left_delta_px.u);
  Assign(fine_motion,"left_delta_v_px",c.ball.fine_motion.left_delta_px.v);
  Assign(fine_motion,"right_delta_u_px",c.ball.fine_motion.right_delta_px.u);
  Assign(fine_motion,"right_delta_v_px",c.ball.fine_motion.right_delta_px.v);
  Assign(b, "approach_two_step_v_px", c.ball.approach_two_step_v_px);
  Assign(b, "approach_direct_down_v_px", c.ball.approach_direct_down_v_px);
  Assign(b, "pickup_max_attempts", c.ball.pickup_max_attempts);
  Assign(b, "pickup_verify_min_hits", c.ball.pickup_verify_min_hits);
  Assign(b, "pickup_u1_px", c.ball.pickup_u1_px);
  Assign(b, "pickup_u2_px", c.ball.pickup_u2_px);
  Assign(b, "pickup_v1_px", c.ball.pickup_v1_px);
  Assign(b, "pickup_v2_px", c.ball.pickup_v2_px);
  Assign(b, "recovery_center_tolerance_px", c.ball.recovery_center_tolerance_px);

  const auto h = a["hurdle"];
  Assign(h, "down_one_step_v_px", c.hurdle.down_one_step_v_px);
  Assign(h, "fine_mode_v_px", c.hurdle.fine_mode_v_px);
  Assign(h, "approach_short_v_px", c.hurdle.approach_short_v_px);
  Assign(h, "down_search_max_steps", c.hurdle.down_search_max_steps);
  Assign(h, "down_search_inter_action_hold_sec", c.hurdle.down_search_inter_action_hold_sec);
  Assign(h, "recovery_observation_sec", c.hurdle.recovery_observation_sec);
  Assign(h, "recovery_max_observations", c.hurdle.recovery_max_observations);
  Assign(h, "target_u_px", c.hurdle.target_u_px);
  Assign(h, "contact_v_px", c.hurdle.contact_v_px);
  Assign(h, "contact_steps", c.hurdle.contact_steps);
  Assign(h, "contact_inter_action_hold_sec", c.hurdle.contact_inter_action_hold_sec);

  Assign(h, "stable_window", c.hurdle.stable_window);
  Assign(h, "stable_min_hits", c.hurdle.stable_min_hits);
  Assign(h,"down_search_min_hits",c.hurdle.down_search_min_hits);
  Assign(h, "lost_frames", c.hurdle.lost_frames);
  Assign(h, "upper_acquire_v_px", c.hurdle.upper_acquire_v_px);
  Assign(h, "approach_two_step_v_px", c.hurdle.approach_two_step_v_px);
  Assign(h, "hurdle_ignore_duration_sec", c.hurdle.hurdle_ignore_duration_sec);
  Assign(h, "recovery_center_tolerance_px", c.hurdle.recovery_center_tolerance_px);

  const auto g = a["goal"];
  Assign(g, "edge_margin_px", c.goal.edge_margin_px);
  Assign(g, "trial_start_goal", c.goal_trial_start_goal);
  Assign(g, "test_mission_index", c.test_goal_mission_index);
  Assign(g, "backboard_depth_strip_width_px", c.backboard_depth_strip_width_px);
  Assign(g, "backboard_min_depth_m", c.backboard_min_depth_m);
  Assign(g, "backboard_max_depth_m", c.backboard_max_depth_m);
  Assign(g,"fine_one_step_range_error_m",c.goal.fine_one_step_range_error_m);
  Assign(g,"fine_two_step_range_error_m",c.goal.fine_two_step_range_error_m);
  Assign(g, "recovery_turn_max", c.goal.recovery_turn_max);
  Assign(g, "recovery_observation_sec", c.goal.recovery_observation_sec);
  Assign(g, "recovery_max_observations", c.goal.recovery_max_observations);
  Assign(g, "approach_short_z_m", c.goal.approach_short_z_m);
  Assign(g, "camera_goal_start_z_m", c.goal.camera_goal_start_z_m);
  Assign(g, "post_shoot_back_steps", c.goal.post_shoot_back_steps);

  Assign(g, "stable_window", c.goal.stable_window);
  Assign(g, "stable_min_hits", c.goal.stable_min_hits);
  Assign(g, "lost_frames", c.goal.lost_frames);
  Assign(g, "smooth_alpha", c.goal.smooth_alpha);
  Assign(g, "post_pickup_walk_fallback_actions", c.goal.post_pickup_walk_fallback_actions);
  Assign(g, "entry_line_retry_max_attempts", c.goal.entry_line_retry_max_attempts);
  Assign(g, "target_u_px", c.goal.target_u_px);
  Assign(g, "approach_u_deadband_px", c.goal.approach_u_deadband_px);
  Assign(g, "fine_adjust_start_z_m", c.goal.fine_adjust_start_z_m);
  Assign(g, "hoop_radius_m", c.goal.hoop_radius_m);
  Assign(g, "throw_landing_x_m", c.goal.throw_landing_x_m);
  Assign(g, "throw_landing_z_m", c.goal.throw_landing_z_m);
  Assign(g, "position_tolerance_m", c.goal.position_tolerance_m);
  Assign(g, "shoot_yaw_limit_deg", c.goal.shoot_yaw_limit_deg);
  Assign(g, "recovery_center_tolerance_px", c.goal.recovery_center_tolerance_px);

  const auto ld = a["line_detection"];
  Assign(ld, "class_id", c.line_detection.class_id);
  Assign(ld, "confidence", c.line_detection.confidence);
  Assign(ld, "min_box_width", c.line_detection.min_box_width);
  Assign(ld, "min_box_height", c.line_detection.min_box_height);
  const auto t = a["object_targets"];
  Assign(t, "ball_class_id", c.object_targets.ball_class_id);
  Assign(t, "goal_class_id", c.object_targets.goal_class_id);
  Assign(t, "backboard_class_id", c.object_targets.backboard_class_id);
  Assign(t, "hurdle_class_id", c.object_targets.hurdle_class_id);
  Assign(t, "ball_confidence", c.object_targets.ball_confidence);
  Assign(t, "goal_confidence", c.object_targets.goal_confidence);
  Assign(t, "backboard_confidence", c.object_targets.backboard_confidence);
  Assign(t, "hurdle_confidence", c.object_targets.hurdle_confidence);
  Assign(t,"ball_min_box_width",c.object_targets.ball_min_box_width);
  Assign(t,"ball_min_box_height",c.object_targets.ball_min_box_height);
  Assign(t,"goal_min_box_width",c.object_targets.goal_min_box_width);
  Assign(t,"goal_min_box_height",c.object_targets.goal_min_box_height);
  Assign(t,"backboard_min_box_width",c.object_targets.backboard_min_box_width);
  Assign(t,"backboard_min_box_height",c.object_targets.backboard_min_box_height);
  Assign(t,"hurdle_min_box_width",c.object_targets.hurdle_min_box_width);
  Assign(t,"hurdle_min_box_height",c.object_targets.hurdle_min_box_height);

  Assign(t,"ball_aspect_ratio_min",c.object_targets.ball_aspect_ratio_min);
  Assign(t,"ball_aspect_ratio_max",c.object_targets.ball_aspect_ratio_max);
  Assign(t,"hurdle_aspect_ratio_min",c.object_targets.hurdle_aspect_ratio_min);
  Assign(t,"ball_forward_min_depth_m",c.object_targets.ball_forward_min_depth_m);
  Assign(t,"ball_forward_max_depth_m",c.object_targets.ball_forward_max_depth_m);
  Assign(t,"ball_down_min_depth_m",c.object_targets.ball_down_min_depth_m);
  Assign(t,"ball_down_max_depth_m",c.object_targets.ball_down_max_depth_m);
  Assign(t,"hurdle_min_depth_m",c.object_targets.hurdle_min_depth_m);
  Assign(t,"hurdle_max_depth_m",c.object_targets.hurdle_max_depth_m);
  Assign(t,"ball_depth_enabled",c.object_targets.ball_depth_enabled);
  Assign(t,"hurdle_depth_enabled",c.object_targets.hurdle_depth_enabled);
  Assign(t,"ball_allow_missing_depth",c.object_targets.ball_allow_missing_depth);
  Assign(t,"hurdle_allow_missing_depth",c.object_targets.hurdle_allow_missing_depth);
  Assign(t,"depth_roi_size_px",c.object_targets.depth_roi_size_px);
  const auto bt=a["ball_candidate_tracking"];
  Assign(bt,"enabled",c.ball_candidate_tracking.enabled);
  Assign(bt,"max_center_distance_px",c.ball_candidate_tracking.max_center_distance_px);
  Assign(bt,"max_size_log_ratio",c.ball_candidate_tracking.max_size_log_ratio);
  Assign(bt,"missing_frame_limit",c.ball_candidate_tracking.missing_frame_limit);
  Assign(bt,"max_tracks",c.ball_candidate_tracking.max_tracks);
  for (const auto& item : {std::make_pair("hurdle_candidate_tracking", &c.hurdle_candidate_tracking),
                           std::make_pair("backboard_candidate_tracking", &c.backboard_candidate_tracking)}) {
    const auto node = a[item.first];
    auto& tracking = *item.second;
    Assign(node, "enabled", tracking.enabled);
    Assign(node, "max_center_distance_px", tracking.max_center_distance_px);
    Assign(node, "max_size_log_ratio", tracking.max_size_log_ratio);
    Assign(node, "missing_frame_limit", tracking.missing_frame_limit);
    Assign(node, "max_tracks", tracking.max_tracks);
  }

  const auto oa = a["object_association"];
  Assign(oa, "missing_frame_limit", c.object_association.missing_frame_limit);
  Assign(oa, "max_center_distance_norm", c.object_association.max_center_distance_norm);
  Assign(oa, "max_size_log_ratio", c.object_association.max_size_log_ratio);
  Assign(oa, "center_distance_weight", c.object_association.center_distance_weight);
  Assign(oa, "size_change_weight", c.object_association.size_change_weight);
  const auto cmd = a["command"];
  Assign(cmd, "first_action_id", c.command.first_action_id);
  Assign(cmd, "action_ack_timeout_sec", c.command.action_ack_timeout_sec);
  const auto m = a["mission"];
  Assign(m, "candidate_search_max_turns", c.candidate_search_max_turns);
  int test_mission=0;
  Assign(m,"test_mission",test_mission); c.test_mission=static_cast<MissionType>(test_mission);
  Assign(m,"test_stage",c.test_stage);
  Assign(m, "mission_recovery_max_turns", c.mission_recovery_max_turns);
  Assign(m, "mission_recovery_yaw_deg", c.mission_recovery_yaw_deg);

  Assign(m, "camera_motion_timeout_sec", c.camera_motion_timeout_sec);
  Assign(m, "transition_settle_sec", c.transition_settle_sec);
  Assign(m, "walk_inter_action_hold_sec", c.walk_inter_action_hold_sec);
  Assign(m, "fine_motion_settle_sec", c.fine_motion_settle_sec);
  Assign(m, "post_motion_observation_sec", c.post_motion_observation_sec);
  Assign(m, "object_ready_mean_actions", c.object_ready_mean_actions);
  Assign(m, "enable_ball", c.enable_ball);
  Assign(m, "enable_hurdle", c.enable_hurdle);
  Assign(m, "enable_goal", c.enable_goal);
  Assign(m, "initial_has_ball", c.initial_has_ball);
  ValidateAlgorithmConfig(c);
  return c;
}

MissionControllerConfig LoadDefaultAlgorithmConfig() {
  return LoadAlgorithmConfig(DefaultAlgorithmConfigPath());
}
} // namespace vision_core
