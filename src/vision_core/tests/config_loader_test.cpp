#include "vision_core/config_loader.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>

static void Reject(const std::string &yaml, const std::string &from,
                   const std::string &to) {
  std::string bad = yaml;
  const auto pos = bad.find(from); assert(pos != std::string::npos);
  bad.replace(pos, from.size(), to);
  const char *path = "invalid_algorithm.yaml";
  { std::ofstream out(path); out << bad; }
  bool rejected = false;
  try { (void)vision_core::LoadAlgorithmConfig(path); }
  catch (const std::runtime_error &) { rejected = true; }
  std::remove(path); assert(rejected);
}

static vision_core::MissionControllerConfig LoadText(const std::string &yaml) {
  const char *path = "modified_algorithm.yaml";
  { std::ofstream out(path); out << yaml; }
  try {
    auto result = vision_core::LoadAlgorithmConfig(path);
    std::remove(path);
    return result;
  } catch (...) {
    std::remove(path);
    throw;
  }
}

int main(int argc, char **argv) {
  assert(argc == 2);
  const auto c = vision_core::LoadAlgorithmConfig(argv[1]);
  const auto defaults = vision_core::LoadDefaultAlgorithmConfig();
  assert(defaults.command.first_action_id != 0);
  assert(c.line_p2p.failure_min_valid_samples == 5);
  assert(c.line_p2p.recovery_max_turns == 5);
  assert(c.line_p2p.recovery_turn_yaw_deg == 15);
  assert(c.goal.shoot_yaw_limit_deg == 30.0);
  assert(c.hurdle.stable_window == 10);
  assert(c.hurdle.stable_min_hits == 7);
  assert(c.post_motion_observation_sec == 1.0);
  assert(c.object_ready_mean_actions == std::vector<int>({22, 23}));
  assert(defaults.object_ready_mean_actions == c.object_ready_mean_actions);
  assert(c.UsesObjectReadyMean(vision_core::MissionAction::kStepForwardThree));
  assert(c.UsesObjectReadyMean(vision_core::MissionAction::kStepForwardFour));
  assert(!c.UsesObjectReadyMean(vision_core::MissionAction::kStepForwardLeftTwo));
  assert(c.test_stage==0 && c.test_mission==vision_core::MissionType::kNone);
  assert(c.backboard_depth_strip_width_px==5);
  assert(c.line_p2p.recovery_observation_sec == 1.0);
  assert(c.goal.recovery_timeout_sec == 5.0);
  assert(c.goal.recovery_reacquire_min_hits == 3);
  assert(c.goal.recovery_center_tolerance_px == 76.8);
  assert(c.ball.pickup_u1_px == 310);
  assert(c.ball.pickup_u2_px == 385);
  assert(c.ball.pickup_v1_px == 80);
  assert(c.ball.pickup_v2_px == 130);
  assert(c.ball.down_one_step_v_px == 10);
  assert(c.ball.fine_mode_v_px == 80);
  assert(c.ball.pickup_max_attempts == 100);
  assert(c.hurdle.contact_steps == 5);
  assert(c.hurdle.down_one_step_v_px == 120);
  assert(c.hurdle.fine_mode_v_px == 210);
  assert(c.goal.fine_one_step_range_error_m == 0.15);
  assert(c.goal.fine_two_step_range_error_m == 0.30);
  assert(c.backboard_max_depth_m == 2.50);
  assert(c.line_features.image_center_u == 410.0);
  assert(c.camera_motion_timeout_sec == 3.0);
  assert(c.line_features.guide_min_points == 3);
  std::ifstream in(argv[1]); std::ostringstream text; text << in.rdbuf();
  // Future opt-in only edits YAML; AUTO and tuning use the same parser.
  const auto replaced = [&text](const std::string &value) {
    std::string modified = text.str();
    const std::string original = "object_ready_mean_actions: [22, 23]";
    const auto pos = modified.find(original);
    assert(pos != std::string::npos);
    modified.replace(pos, original.size(), value);
    return modified;
  };
  auto future = LoadText(replaced("object_ready_mean_actions: [22, 23, 24, 25]"));
  assert(future.UsesObjectReadyMean(vision_core::MissionAction::kStepForwardLeftTwo));
  assert(future.UsesObjectReadyMean(vision_core::MissionAction::kStepForwardRightTwo));
  auto disabled = LoadText(replaced("object_ready_mean_actions: []"));
  assert(disabled.object_ready_mean_actions.empty());
  assert(!disabled.UsesObjectReadyMean(vision_core::MissionAction::kStepForwardFour));
  Reject(text.str(), "object_ready_mean_actions: [22, 23]",
         "object_ready_mean_actions: [22, 23, 22]");
  Reject(text.str(), "object_ready_mean_actions: [22, 23]",
         "object_ready_mean_actions: [10, 22]");
  Reject(text.str(), "object_ready_mean_actions: [22, 23]",
         "object_ready_mean_actions: [7]");
  Reject(text.str(), "object_ready_mean_actions: [22, 23]",
         "object_ready_mean_actions: [22, not_an_action]");
  Reject(text.str(), "object_ready_mean_actions: [22, 23]",
         "object_ready_mean_actions: 22");
  Reject(text.str(), "object_ready_mean_actions: [22, 23]", "");
  Reject(text.str(), "post_motion_observation_sec: 1.0", "post_motion_observation_sec: 0.0");
  Reject(text.str(), "test_stage: 0", "test_stage: 4");
  Reject(text.str(), "backboard_depth_strip_width_px: 5", "backboard_depth_strip_width_px: 0");
  Reject(text.str(), "offset_gain: 0.3", "offset_gain: .nan");
  Reject(text.str(), "guide_min_points: 3", "guide_min_points: 2");
  Reject(text.str(), "guide_min_points: 3", "guide_min_points: 9");
  Reject(text.str(), "guide_min_points: 3", "guide_min_points: .nan");
  Reject(text.str(), "guide_min_points: 3", "");
  Reject(text.str(), "camera_motion_timeout_sec: 3.0",
         "camera_motion_timeout_sec: -0.1");
  Reject(text.str(), "camera_motion_timeout_sec: 3.0",
         "camera_motion_timeout_sec: .nan");
  Reject(text.str(), "camera_motion_timeout_sec: 3.0",
         "camera_motion_timeout_sec: .inf");
  Reject(text.str(), "heading_gain: 0.3", "heading_gain: -1.0");
  Reject(text.str(), "steering_deadband: 0.10", "steering_deadband: -0.1");
  Reject(text.str(), "line_stable_min_hits: 7", "line_stable_min_hits: 11");
  Reject(text.str(), "failure_min_valid_samples: 5",
         "failure_min_valid_samples: 0");
  Reject(text.str(), "pickup_u1_px: 310", "pickup_u1_px: -1.0");
  Reject(text.str(), "pickup_u1_px: 310", "pickup_u1_px: .nan");
  Reject(text.str(), "pickup_u1_px: 310", "pickup_u1_px: 386");
  Reject(text.str(), "pickup_u2_px: 385", "pickup_u2_px: 309.0");
  Reject(text.str(), "pickup_v1_px: 80", "pickup_v1_px: 131.0");
  Reject(text.str(), "pickup_v2_px: 130", "pickup_v2_px: .inf");
  Reject(text.str(), "smooth_alpha: 0.45", "smooth_alpha: .nan");
  Reject(text.str(), "approach_two_step_v_px: 360.0", "approach_two_step_v_px: .nan");
  Reject(text.str(), "pickup_max_attempts: 100", "pickup_max_attempts: 0");
  Reject(text.str(), "upper_acquire_v_px: 470.0", "upper_acquire_v_px: -0.1");
  Reject(text.str(), "target_u_px: 380.0", "target_u_px: .inf");
  Reject(text.str(), "throw_landing_z_m: 0.48", "throw_landing_z_m: 0.0");
  Reject(text.str(), "throw_landing_x_m: -0.08", "throw_landing_x_m: .nan");
  Reject(text.str(), "fine_settle_duration_sec: 0.60",
         "fine_settle_duration_sec: -0.1");
  Reject(text.str(), "action_ack_timeout_sec: 10.0",
         "action_ack_timeout_sec: .nan");
  Reject(text.str(), "backboard_min_depth_m: 0.20",
         "backboard_min_depth_m: 6.00");
  Reject(text.str(), "recovery_turn_yaw_deg: 15",
         "recovery_turn_yaw_deg: 0");
  Reject(text.str(), "recovery_turn_yaw_deg: 15",
         "recovery_turn_yaw_deg: 181");
  Reject(text.str(), "\n    confidence: 0.60\n    min_box_width: 1.0",
         "\n    confidence: .nan\n    min_box_width: 1.0");
  Reject(text.str(), "min_box_width: 1.0", "min_box_width: -1.0");
  Reject(text.str(), "ball_confidence: 0.60", "ball_confidence: 1.1");
  Reject(text.str(), "ball_class_id: 1", "ball_class_id: -1");
  Reject(text.str(), "ball_class_id: 1", "ball_class_id: 0");
  Reject(text.str(), "hurdle_class_id: 4", "hurdle_class_id: 0");
  Reject(text.str(), "hurdle_class_id: 4", "hurdle_class_id: 3");
  Reject(text.str(), "missing_frame_limit: 5", "missing_frame_limit: -1");
  Reject(text.str(), "center_distance_weight: 1.0",
         "center_distance_weight: -1.0");

  auto direct = c;
  direct.line_detection.class_id = -1;
  bool direct_rejected = false;
  try { vision_core::ValidateAlgorithmConfig(direct); }
  catch (const std::runtime_error &) { direct_rejected = true; }
  assert(direct_rejected);

  auto invalid_goal = c;
  invalid_goal.goal.recovery_reacquire_min_hits = 0;
  bool rejected_goal = false;
  try { vision_core::ValidateAlgorithmConfig(invalid_goal); }
  catch (const std::runtime_error &) { rejected_goal = true; }
  assert(rejected_goal);

  auto relaxed_association = c;
  relaxed_association.object_association.max_center_distance_norm = 1.2;
  vision_core::ValidateAlgorithmConfig(relaxed_association);

  bool constructor_rejected = false;
  try { vision_core::MissionController invalid(direct); }
  catch (const std::runtime_error &) { constructor_rejected = true; }
  assert(constructor_rejected);
  return 0;
}
