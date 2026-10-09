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

int main(int argc, char **argv) {
  assert(argc == 2);
  const auto c = vision_core::LoadAlgorithmConfig(argv[1]);
  const auto defaults = vision_core::LoadDefaultAlgorithmConfig();
  assert(defaults.command.first_action_id != 0);
  assert(c.line_p2p.failure_min_valid_samples == 5);
  assert(c.line_p2p.recovery_max_turns == 5);
  assert(c.line_p2p.recovery_turn_yaw_deg == 15);
  assert(c.goal.shoot_yaw_limit_deg == 30.0);
  assert(c.ball.fine_target_u_norm == 0.50);
  assert(c.camera_motion_timeout_sec == 3.0);
  assert(c.line_features.guide_min_points == 3);
  std::ifstream in(argv[1]); std::ostringstream text; text << in.rdbuf();
  Reject(text.str(), "offset_gain: 1.0", "offset_gain: .nan");
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
  Reject(text.str(), "heading_gain: 1.0", "heading_gain: -1.0");
  Reject(text.str(), "steering_deadband: 0.10", "steering_deadband: -0.1");
  Reject(text.str(), "line_stable_min_hits: 7", "line_stable_min_hits: 11");
  Reject(text.str(), "failure_min_valid_samples: 5",
         "failure_min_valid_samples: 0");
  Reject(text.str(), "fine_target_u_norm: 0.50", "fine_target_u_norm: 1.1");
  Reject(text.str(), "smooth_alpha: 0.45", "smooth_alpha: .nan");
  Reject(text.str(), "tilt_down_min_hits: 7", "tilt_down_min_hits: 11");
  Reject(text.str(), "pickup_max_attempts: 3", "pickup_max_attempts: 0");
  Reject(text.str(), "acquire_min_v_norm: 0.60", "acquire_min_v_norm: -0.1");
  Reject(text.str(), "target_u_norm: 0.50", "target_u_norm: .inf");
  Reject(text.str(), "throwing_range_m: 0.40", "throwing_range_m: 0.0");
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

  auto relaxed_association = c;
  relaxed_association.object_association.max_center_distance_norm = 1.2;
  vision_core::ValidateAlgorithmConfig(relaxed_association);

  bool constructor_rejected = false;
  try { vision_core::MissionController invalid(direct); }
  catch (const std::runtime_error &) { constructor_rejected = true; }
  assert(constructor_rejected);
  return 0;
}
