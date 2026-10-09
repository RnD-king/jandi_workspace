#include "vision_core/config_loader.hpp"

#include <array>
#include <cmath>
#include <cstdlib>
#include <stdexcept>
#include <string>

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

  HitsWithinWindow(c.line.line_stable_min_hits, c.line.line_stable_window,
                   "line.line_stable_min_hits", "line.line_stable_window");

  Nonnegative(c.line_p2p.offset_gain, "line_p2p.offset_gain");
  Nonnegative(c.line_p2p.heading_gain, "line_p2p.heading_gain");
  Nonnegative(c.line_p2p.steering_deadband, "line_p2p.steering_deadband");
  Nonnegative(c.line_p2p.ready_lead_sec, "line_p2p.ready_lead_sec");
  Nonnegative(c.line_p2p.failure_observation_sec,
              "line_p2p.failure_observation_sec");
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

  HitsWithinWindow(c.ball.stable_min_hits, c.ball.stable_window,
                   "ball.stable_min_hits", "ball.stable_window");
  PositiveInt(c.ball.lost_frames, "ball.lost_frames");
  Unit(c.ball.smooth_alpha, "ball.smooth_alpha");
  Unit(c.ball.far_u_des_norm, "ball.far_u_des_norm");
  Unit(c.ball.approach_u_deadband, "ball.approach_u_deadband");
  Unit(c.ball.upper_acquire_v_norm, "ball.upper_acquire_v_norm");
  Unit(c.ball.tilt_down_v_norm, "ball.tilt_down_v_norm");
  HitsWithinWindow(c.ball.tilt_down_min_hits, c.ball.tilt_down_window,
                   "ball.tilt_down_min_hits", "ball.tilt_down_window");
  PositiveInt(c.ball.pickup_max_attempts, "ball.pickup_max_attempts");
  Nonnegative(c.ball.ball_ignore_duration_sec,
              "ball.ball_ignore_duration_sec");
  Unit(c.ball.fine_target_u_norm, "ball.fine_target_u_norm");
  Unit(c.ball.fine_target_v_norm, "ball.fine_target_v_norm");
  Unit(c.ball.fine_u_deadband, "ball.fine_u_deadband");
  Unit(c.ball.fine_v_deadband, "ball.fine_v_deadband");
  Nonnegative(c.ball.fine_settle_duration_sec,
              "ball.fine_settle_duration_sec");
  Nonnegative(c.ball.recovery_timeout_sec, "ball.recovery_timeout_sec");
  PositiveInt(c.ball.recovery_reacquire_min_hits,
              "ball.recovery_reacquire_min_hits");
  Unit(c.ball.recovery_center_tolerance_norm,
       "ball.recovery_center_tolerance_norm");
  Nonnegative(c.ball.recovery_settle_duration_sec,
              "ball.recovery_settle_duration_sec");

  HitsWithinWindow(c.hurdle.stable_min_hits, c.hurdle.stable_window,
                   "hurdle.stable_min_hits", "hurdle.stable_window");
  PositiveInt(c.hurdle.lost_frames, "hurdle.lost_frames");
  Unit(c.hurdle.smooth_alpha, "hurdle.smooth_alpha");
  Unit(c.hurdle.acquire_min_v_norm, "hurdle.acquire_min_v_norm");
  Unit(c.hurdle.tilt_trigger_v_norm, "hurdle.tilt_trigger_v_norm");
  HitsWithinWindow(c.hurdle.tilt_trigger_min_hits,
                   c.hurdle.tilt_trigger_window,
                   "hurdle.tilt_trigger_min_hits",
                   "hurdle.tilt_trigger_window");
  Nonnegative(c.hurdle.hurdle_ignore_duration_sec,
              "hurdle.hurdle_ignore_duration_sec");
  PositiveInt(c.hurdle.recovery_reacquire_min_hits,
              "hurdle.recovery_reacquire_min_hits");
  Nonnegative(c.hurdle.recovery_timeout_sec,
              "hurdle.recovery_timeout_sec");
  Unit(c.hurdle.recovery_center_tolerance_norm,
       "hurdle.recovery_center_tolerance_norm");
  Nonnegative(c.hurdle.recovery_settle_duration_sec,
              "hurdle.recovery_settle_duration_sec");

  HitsWithinWindow(c.goal.stable_min_hits, c.goal.stable_window,
                   "goal.stable_min_hits", "goal.stable_window");
  PositiveInt(c.goal.lost_frames, "goal.lost_frames");
  Unit(c.goal.smooth_alpha, "goal.smooth_alpha");
  Nonnegative(c.goal.post_pickup_wait_sec, "goal.post_pickup_wait_sec");
  Unit(c.goal.target_u_norm, "goal.target_u_norm");
  Unit(c.goal.approach_u_deadband, "goal.approach_u_deadband");
  Nonnegative(c.goal.fine_adjust_start_z_m, "goal.fine_adjust_start_z_m");
  Nonnegative(c.goal.hoop_radius_m, "goal.hoop_radius_m");
  Positive(c.goal.throwing_range_m, "goal.throwing_range_m");
  Nonnegative(c.goal.position_tolerance_m, "goal.position_tolerance_m");
  Nonnegative(c.goal.fine_settle_duration_sec,
              "goal.fine_settle_duration_sec");
  Nonnegative(c.goal.shoot_yaw_limit_deg, "goal.shoot_yaw_limit_deg");

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
  Nonnegative(c.object_targets.min_box_width,
              "object_targets.min_box_width");
  Nonnegative(c.object_targets.min_box_height,
              "object_targets.min_box_height");

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

  Positive(c.backboard_min_depth_m, "mission.backboard_min_depth_m");
  Nonnegative(c.camera_motion_timeout_sec, "mission.camera_motion_timeout_sec");
  Positive(c.backboard_max_depth_m, "mission.backboard_max_depth_m");
  if (c.backboard_min_depth_m > c.backboard_max_depth_m)
    throw std::runtime_error(
        "vision algorithm key 'mission.backboard_min_depth_m' must be <= "
        "'mission.backboard_max_depth_m'");
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
  const YAML::Node a = root["vision_algorithm"];
  if (!a || !a.IsMap())
    throw std::runtime_error("vision algorithm config must contain a vision_algorithm map: " + path);

  const auto f = a["line_features"];
  Assign(f, "max_centers", c.line_features.max_centers);
  Assign(f, "guide_min_points", c.line_features.guide_min_points);
  Assign(f, "image_center_u", c.line_features.image_center_u);
  Assign(f, "curve_min_points", c.line_features.curve_min_points);
  Assign(f, "curve_min_v_span_px", c.line_features.curve_min_v_span_px);
  Assign(f, "curve_local_fit_points", c.line_features.curve_local_fit_points);
  Assign(f, "guide_fit_rmse_full_scale_px", c.line_features.guide_fit_rmse_full_scale_px);

  const auto line = a["line"];
  Assign(line, "line_stable_window", c.line.line_stable_window);
  Assign(line, "line_stable_min_hits", c.line.line_stable_min_hits);

  const auto lp = a["line_p2p"];
  Assign(lp, "offset_gain", c.line_p2p.offset_gain);
  Assign(lp, "heading_gain", c.line_p2p.heading_gain);
  Assign(lp, "steering_deadband", c.line_p2p.steering_deadband);
  Assign(lp, "ready_lead_sec", c.line_p2p.ready_lead_sec);
  Assign(lp, "failure_observation_sec", c.line_p2p.failure_observation_sec);
  Assign(lp, "failure_min_valid_samples", c.line_p2p.failure_min_valid_samples);
  Assign(lp, "no_evidence_max_retries", c.line_p2p.no_evidence_max_retries);
  Assign(lp, "recovery_max_turns", c.line_p2p.recovery_max_turns);
  Assign(lp, "recovery_turn_yaw_deg", c.line_p2p.recovery_turn_yaw_deg);

  const auto b = a["ball"];
  Assign(b, "stable_window", c.ball.stable_window);
  Assign(b, "stable_min_hits", c.ball.stable_min_hits);
  Assign(b, "lost_frames", c.ball.lost_frames);
  Assign(b, "smooth_alpha", c.ball.smooth_alpha);
  Assign(b, "far_u_des_norm", c.ball.far_u_des_norm);
  Assign(b, "approach_u_deadband", c.ball.approach_u_deadband);
  Assign(b, "upper_acquire_v_norm", c.ball.upper_acquire_v_norm);
  Assign(b, "tilt_down_v_norm", c.ball.tilt_down_v_norm);
  Assign(b, "tilt_down_window", c.ball.tilt_down_window);
  Assign(b, "tilt_down_min_hits", c.ball.tilt_down_min_hits);
  Assign(b, "pickup_max_attempts", c.ball.pickup_max_attempts);
  Assign(b, "ball_ignore_duration_sec", c.ball.ball_ignore_duration_sec);
  Assign(b, "fine_target_u_norm", c.ball.fine_target_u_norm);
  Assign(b, "fine_target_v_norm", c.ball.fine_target_v_norm);
  Assign(b, "fine_u_deadband", c.ball.fine_u_deadband);
  Assign(b, "fine_v_deadband", c.ball.fine_v_deadband);
  Assign(b, "fine_settle_duration_sec", c.ball.fine_settle_duration_sec);
  Assign(b, "recovery_timeout_sec", c.ball.recovery_timeout_sec);
  Assign(b, "recovery_reacquire_min_hits", c.ball.recovery_reacquire_min_hits);
  Assign(b, "recovery_center_tolerance_norm", c.ball.recovery_center_tolerance_norm);
  Assign(b, "recovery_settle_duration_sec", c.ball.recovery_settle_duration_sec);

  const auto h = a["hurdle"];
  Assign(h, "stable_window", c.hurdle.stable_window);
  Assign(h, "stable_min_hits", c.hurdle.stable_min_hits);
  Assign(h, "lost_frames", c.hurdle.lost_frames);
  Assign(h, "smooth_alpha", c.hurdle.smooth_alpha);
  Assign(h, "acquire_min_v_norm", c.hurdle.acquire_min_v_norm);
  Assign(h, "tilt_trigger_v_norm", c.hurdle.tilt_trigger_v_norm);
  Assign(h, "tilt_trigger_window", c.hurdle.tilt_trigger_window);
  Assign(h, "tilt_trigger_min_hits", c.hurdle.tilt_trigger_min_hits);
  Assign(h, "hurdle_ignore_duration_sec", c.hurdle.hurdle_ignore_duration_sec);
  Assign(h, "recovery_reacquire_min_hits", c.hurdle.recovery_reacquire_min_hits);
  Assign(h, "recovery_timeout_sec", c.hurdle.recovery_timeout_sec);
  Assign(h, "recovery_center_tolerance_norm", c.hurdle.recovery_center_tolerance_norm);
  Assign(h, "recovery_settle_duration_sec", c.hurdle.recovery_settle_duration_sec);

  const auto g = a["goal"];
  Assign(g, "stable_window", c.goal.stable_window);
  Assign(g, "stable_min_hits", c.goal.stable_min_hits);
  Assign(g, "lost_frames", c.goal.lost_frames);
  Assign(g, "smooth_alpha", c.goal.smooth_alpha);
  Assign(g, "post_pickup_wait_sec", c.goal.post_pickup_wait_sec);
  Assign(g, "target_u_norm", c.goal.target_u_norm);
  Assign(g, "approach_u_deadband", c.goal.approach_u_deadband);
  Assign(g, "fine_adjust_start_z_m", c.goal.fine_adjust_start_z_m);
  Assign(g, "hoop_radius_m", c.goal.hoop_radius_m);
  Assign(g, "throwing_range_m", c.goal.throwing_range_m);
  Assign(g, "position_tolerance_m", c.goal.position_tolerance_m);
  Assign(g, "fine_settle_duration_sec", c.goal.fine_settle_duration_sec);
  Assign(g, "shoot_yaw_limit_deg", c.goal.shoot_yaw_limit_deg);

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
  Assign(t, "min_box_width", c.object_targets.min_box_width);
  Assign(t, "min_box_height", c.object_targets.min_box_height);
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
  Assign(m, "camera_motion_timeout_sec", c.camera_motion_timeout_sec);
  Assign(m, "backboard_min_depth_m", c.backboard_min_depth_m);
  Assign(m, "backboard_max_depth_m", c.backboard_max_depth_m);
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
