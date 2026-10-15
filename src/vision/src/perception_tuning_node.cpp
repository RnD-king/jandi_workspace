#include "vision/mission_debug_view.hpp"
#include "vision/debug_guides.hpp"
#include "vision/mission_parameters.hpp"
#include "vision/backboard_depth.hpp"
// vision/src/perception_tuning_node.cpp
//
// 1. perception_tuning_node.cpp
//    전체 ROS 노드 시작점. 이미지/IMU를 받고 전체 파이프라인을 호출함.

// 2. yolo_trt_engine.cpp
//    line_perception_node가 YOLO 추론을 요청하면 실행됨.
//    엔진 로드, 입력/출력 메모리 관리, TensorRT enqueue, 후처리 담당.

// 3. yolo_preprocess.cu
//    yolo_trt_engine.cpp 내부 Preprocess()에서 호출됨.
//    YOLO 입력용으로 BGR -> RGB, HWC -> CHW, float [0,1], letterbox padding
//    수행.

// 4. yolo_trt_engine.cpp
//    TensorRT 추론 후 output [x1,y1,x2,y2,conf,class]를 원본 이미지 좌표 bbox로
//    복원.

// 5. shared_vision_core/MissionController::StepPerception
//    detection 전처리, 선택적 IMU 좌표 보정, line 특징·진입 판정·활성
//    미션 잠금·내부 단계·최종 ControlCommand를 계산한다.

// 7. perception_tuning_node.cpp
//    core가 반환한 최종 액션/카메라 명령을 ROS 토픽으로 전달한다.

// 터미널1: ros2 run my_cv val_image_publisher
// 터미널2: ros2 run vision line_perception_node

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <rclcpp/parameter_map.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <sys/utsname.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "vision_core/config_loader.hpp"
#include "vision_core/mission_controller.hpp"

#include "vision/yolo_trt_engine.hpp"
#include "vision/msg/action_command.hpp"
#include "vision/msg/camera_command.hpp"
#include "vision/msg/command_status.hpp"

namespace vision {

namespace {

bool IsJetsonTarget() {
  struct utsname info {};
  if (uname(&info) != 0) {
    return false;
  }
  const std::string machine(info.machine);
  return machine == "aarch64" || machine == "arm64";
}

std::string RuntimeConfigPath() {
  if (const char *env_path = std::getenv("YOLO26_RUNTIME_CONFIG"); env_path != nullptr && env_path[0] != '\0') {
    return std::string(env_path);
  }
  return ament_index_cpp::get_package_share_directory("vision") + "/config/yolo26_runtime.yaml";
}

std::string ReadRuntimeString(const std::string &param_name) {
  try {
    const auto map = rclcpp::parameter_map_from_yaml_file(RuntimeConfigPath());
    for (const auto &node_name : {"/yolo26_runtime", "yolo26_runtime"}) {
      const auto it = map.find(node_name);
      if (it == map.end()) {
        continue;
      }
      for (const auto &param : it->second) {
        if (param.get_name() == param_name) {
          return param.as_string();
        }
      }
    }
  } catch (const std::exception &e) {
    std::cerr << "[WARN] Failed to load YOLO26 runtime config: " << e.what() << std::endl;
  }
  return {};
}

std::string DefaultYolo26EnginePath() {
  if (const char *env_path = std::getenv("YOLO26_ENGINE_PATH"); env_path != nullptr && env_path[0] != '\0') {
    return std::string(env_path);
  }

  return ReadRuntimeString(IsJetsonTarget() ? "jetson_engine_path" : "pc_engine_path");
}

vision_core::MissionControllerConfig LoadSharedAlgorithmDefaults() {
  const std::string path = vision_core::DefaultAlgorithmConfigPath();
  auto config = vision_core::LoadAlgorithmConfig(path);
  std::cerr << "[INFO] Loaded shared vision algorithm config: " << path
            << std::endl;
  return config;
}

} // namespace

class PerceptionTuningNode final : public rclcpp::Node {
public:
  PerceptionTuningNode() : rclcpp::Node(
#if defined(VISION_BALL_TUNING_NODE)
      "ball_tuning_node"
#elif defined(VISION_HURDLE_TUNING_NODE)
      "hurdle_tuning_node"
#elif defined(VISION_GOAL_TUNING_NODE)
      "goal_tuning_node"
#else
      "perception_tuning_node"
#endif
  ) {
    const vision_core::MissionControllerConfig algorithm_defaults =
        LoadSharedAlgorithmDefaults();
    vision::DeclareMissionParameters(*this,algorithm_defaults);
    declare_parameter<bool>("goal_trial_start_goal",algorithm_defaults.goal_trial_start_goal);
    declare_parameter<std::string>("image_topic", "/camera/camera/color/image_raw"); // 입력 이미지 토픽
    declare_parameter<std::string>("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw");
    declare_parameter<std::string>("camera_info_topic", "/camera/camera/color/camera_info");
    declare_parameter<std::string>("imu_topic", "/camera/imu_tilt");          // IMU roll/pitch 토픽
    declare_parameter<std::string>("action_cmd_topic", "/jandi_vision/action_cmd");
    declare_parameter<std::string>("action_status_topic", "/jandi_vision/action_status");
    declare_parameter<std::string>("camera_cmd_topic", "/jandi_vision/camera_cmd");
    declare_parameter<std::string>("camera_status_topic", "/jandi_vision/camera_status");
    declare_parameter<std::string>("engine_path", DefaultYolo26EnginePath()); // TensorRT engine 경로
    declare_parameter<int>("line_class_id", algorithm_defaults.line_detection.class_id); // line YOLO class id
    declare_parameter<int>("ball_class_id", algorithm_defaults.object_targets.ball_class_id); // ball YOLO class id
    declare_parameter<int>("goal_class_id", algorithm_defaults.object_targets.goal_class_id); // goal YOLO class id
    declare_parameter<int>("backboard_class_id", algorithm_defaults.object_targets.backboard_class_id); // backboard YOLO class id
    declare_parameter<int>("hurdle_class_id", algorithm_defaults.object_targets.hurdle_class_id); // hurdle YOLO class id
    declare_parameter<double>("conf_thres", algorithm_defaults.line_detection.confidence); // YOLO 1차 confidence threshold
    declare_parameter<double>("ball_conf_thres", algorithm_defaults.object_targets.ball_confidence); // ball target 2차 confidence threshold
    declare_parameter<double>("goal_conf_thres", algorithm_defaults.object_targets.goal_confidence); // goal target 2차 confidence threshold
    declare_parameter<double>("backboard_conf_thres", algorithm_defaults.object_targets.backboard_confidence); // backboard target 2차 confidence threshold
    declare_parameter<double>("hurdle_conf_thres", algorithm_defaults.object_targets.hurdle_confidence); // hurdle target 2차 confidence threshold
    declare_parameter<int>("object_min_box_width", static_cast<int>(algorithm_defaults.object_targets.min_box_width));
    declare_parameter<int>("object_min_box_height", static_cast<int>(algorithm_defaults.object_targets.min_box_height));
    declare_parameter<double>("fx", 600.0);                      // camera intrinsic fx
    declare_parameter<double>("fy", 600.0);                      // camera intrinsic fy
    declare_parameter<double>("cx", 320.0);                      // camera intrinsic cx
    declare_parameter<double>("cy", 240.0);                      // camera intrinsic cy
    declare_parameter<bool>("use_imu_rectification", false);     // IMU 기반 픽셀 보정 사용 여부
    declare_parameter<bool>("assume_zero_imu", false);           // roll/pitch를 0으로 가정
    declare_parameter<double>("imu_abs_limit_deg", 45.0);        // IMU roll/pitch clamp 각도
    // core가 확정한 객체만 표시하는 기본 디버그 화면/상태 패널이다.
    declare_parameter<bool>("show_debug_view", true);
    // YOLO의 필터 전 raw detection bbox를 별도 창에서 확인할 때만 켠다.
    declare_parameter<bool>("show_yolo_debug_view", false);
    declare_parameter<double>("inference_hz", 15.0);
    declare_parameter<int>("max_centers", algorithm_defaults.line_features.max_centers);
    // Runtime-only tuning: values are intentionally not written to YAML.
    declare_parameter<double>("line_reference_u_px", algorithm_defaults.line_features.image_center_u);
    declare_parameter<double>("line_p2p_offset_gain", algorithm_defaults.line_p2p.offset_gain);
    declare_parameter<double>("line_p2p_heading_gain", algorithm_defaults.line_p2p.heading_gain);
    declare_parameter<double>("line_p2p_steering_deadband", algorithm_defaults.line_p2p.steering_deadband);
    declare_parameter<double>("ball_down_one_step_v_px", algorithm_defaults.ball.down_one_step_v_px);
    declare_parameter<double>("ball_fine_mode_v_px", algorithm_defaults.ball.fine_mode_v_px);
    declare_parameter<double>("ball_approach_short_v_px", algorithm_defaults.ball.approach_short_v_px);
    declare_parameter<double>("hurdle_down_one_step_v_px", algorithm_defaults.hurdle.down_one_step_v_px);
    declare_parameter<double>("hurdle_fine_mode_v_px", algorithm_defaults.hurdle.fine_mode_v_px);
    declare_parameter<double>("hurdle_approach_short_v_px", algorithm_defaults.hurdle.approach_short_v_px);
    declare_parameter<double>("hurdle_target_u_px", algorithm_defaults.hurdle.target_u_px);
    declare_parameter<double>("hurdle_u_deadband_px", algorithm_defaults.hurdle.u_deadband_px);
    declare_parameter<double>("hurdle_contact_v_px", algorithm_defaults.hurdle.contact_v_px);
    declare_parameter<double>("goal_fine_one_step_range_error_m",algorithm_defaults.goal.fine_one_step_range_error_m);
    declare_parameter<double>("goal_fine_two_step_range_error_m",algorithm_defaults.goal.fine_two_step_range_error_m);
    declare_parameter<double>("goal_approach_short_z_m", algorithm_defaults.goal.approach_short_z_m);
    declare_parameter<double>("goal_camera_goal_start_z_m", algorithm_defaults.goal.camera_goal_start_z_m);
    declare_parameter<int>("mission_recovery_max_turns",algorithm_defaults.mission_recovery_max_turns);
    declare_parameter<int>("hurdle_contact_steps",algorithm_defaults.hurdle.contact_steps);
    declare_parameter<int>("test_stage",algorithm_defaults.test_stage);
    declare_parameter<int>("test_ball_mission_index",algorithm_defaults.test_ball_mission_index);
    declare_parameter<int>("test_goal_mission_index",algorithm_defaults.test_goal_mission_index);
    declare_parameter<double>("ball_far_u_des_px", algorithm_defaults.ball.far_u_des_px);
    declare_parameter<double>("ball_approach_u_deadband_px", algorithm_defaults.ball.approach_u_deadband_px);
    declare_parameter<double>("ball_upper_acquire_v_px", algorithm_defaults.ball.upper_acquire_v_px);
    declare_parameter<double>("ball_approach_two_step_v_px", algorithm_defaults.ball.approach_two_step_v_px);
    declare_parameter<double>("ball_pickup_u1_px", algorithm_defaults.ball.pickup_u1_px);
    declare_parameter<double>("ball_pickup_u2_px", algorithm_defaults.ball.pickup_u2_px);
    declare_parameter<double>("ball_pickup_v1_px", algorithm_defaults.ball.pickup_v1_px);
    declare_parameter<double>("ball_pickup_v2_px", algorithm_defaults.ball.pickup_v2_px);
    declare_parameter<double>("goal_target_u_px", algorithm_defaults.goal.target_u_px);
    declare_parameter<double>("goal_approach_u_deadband_px", algorithm_defaults.goal.approach_u_deadband_px);
    declare_parameter<double>("hurdle_upper_acquire_v_px", algorithm_defaults.hurdle.upper_acquire_v_px);
    declare_parameter<double>("hurdle_approach_two_step_v_px", algorithm_defaults.hurdle.approach_two_step_v_px);
    // 각 object mission controller의 사용 여부다.
    declare_parameter<bool>("enable_ball_controller", algorithm_defaults.enable_ball);
    declare_parameter<bool>("enable_hurdle_controller", algorithm_defaults.enable_hurdle);
    declare_parameter<bool>("enable_goal_controller", algorithm_defaults.enable_goal);
    // all은 위 enable_* 값을 따르고, 나머지는 지정한 알고리즘 하나만 실행한다.
    declare_parameter<std::string>("algorithm_mode",
#if defined(VISION_BALL_TUNING_NODE)
        "ball"
#elif defined(VISION_HURDLE_TUNING_NODE)
        "hurdle"
#elif defined(VISION_GOAL_TUNING_NODE)
        "goal"
#else
        "all"
#endif
    );
    // 실제 카메라 대신 endpoint 도달 feedback을 모사하는 시험 옵션이다.
    declare_parameter<bool>("simulate_camera_feedback", false);
    // 실제 executor 없이 ACK/READY/DONE lifecycle을 시험하는 명시적
    // simulation/test mode다.
    declare_parameter<bool>("simulate_decision", false);
    // 미션/기타 action의 fallback 시간이다. 보행 action은 아래 종류별 시간을
    // 우선 사용한다.
    declare_parameter<double>("simulate_action_duration_sec", 0.50);
    declare_parameter<double>("simulate_step_forward_five_duration_sec", 3.0);
    declare_parameter<double>("simulate_curve_step_duration_sec", 2.5);
    declare_parameter<double>("simulate_step_forward_one_duration_sec", 2.0);
    declare_parameter<double>("simulate_turn_duration_sec", 1.0);
    declare_parameter<double>("simulate_ready_lead_sec", 0.25);
    declare_parameter<bool>("enable_command_transport", true);
    declare_parameter<int>("backboard_depth_strip_width_px",algorithm_defaults.backboard_depth_strip_width_px);
    declare_parameter<double>("max_depth_age_sec", 0.20);
    declare_parameter<double>("backboard_min_depth_m", algorithm_defaults.backboard_min_depth_m);
    declare_parameter<double>("backboard_max_depth_m", algorithm_defaults.backboard_max_depth_m);
    declare_parameter<double>("goal_fine_adjust_start_z_m", algorithm_defaults.goal.fine_adjust_start_z_m);
    declare_parameter<double>("goal_hoop_radius_m", algorithm_defaults.goal.hoop_radius_m);
    declare_parameter<double>("goal_throw_landing_x_m", algorithm_defaults.goal.throw_landing_x_m);
    declare_parameter<double>("goal_throw_landing_z_m", algorithm_defaults.goal.throw_landing_z_m);
    declare_parameter<double>("goal_position_tolerance_m", algorithm_defaults.goal.position_tolerance_m);
    declare_parameter<double>("goal_shoot_yaw_limit_deg", algorithm_defaults.goal.shoot_yaw_limit_deg);
    declare_parameter<double>("camera_motion_timeout_sec", algorithm_defaults.camera_motion_timeout_sec);

    image_topic_ = get_parameter("image_topic").as_string();
    depth_topic_ = get_parameter("depth_topic").as_string();
    camera_info_topic_ = get_parameter("camera_info_topic").as_string();
    imu_topic_ = get_parameter("imu_topic").as_string();
    action_cmd_topic_ = get_parameter("action_cmd_topic").as_string();
    action_status_topic_ = get_parameter("action_status_topic").as_string();
    camera_cmd_topic_ = get_parameter("camera_cmd_topic").as_string();
    camera_status_topic_ = get_parameter("camera_status_topic").as_string();
    engine_path_ = get_parameter("engine_path").as_string();
    line_class_id_ = static_cast<int>(get_parameter("line_class_id").as_int());
    ball_class_id_ = static_cast<int>(get_parameter("ball_class_id").as_int());
    goal_class_id_ = static_cast<int>(get_parameter("goal_class_id").as_int());
    backboard_class_id_ = static_cast<int>(get_parameter("backboard_class_id").as_int());
    hurdle_class_id_ = static_cast<int>(get_parameter("hurdle_class_id").as_int());
    conf_thres_ = static_cast<float>(get_parameter("conf_thres").as_double());
    ball_conf_thres_ = static_cast<float>(get_parameter("ball_conf_thres").as_double());
    goal_conf_thres_ = static_cast<float>(get_parameter("goal_conf_thres").as_double());
    backboard_conf_thres_ = static_cast<float>(get_parameter("backboard_conf_thres").as_double());
    hurdle_conf_thres_ = static_cast<float>(get_parameter("hurdle_conf_thres").as_double());
    use_imu_rectification_ = get_parameter("use_imu_rectification").as_bool();
    assume_zero_imu_ = get_parameter("assume_zero_imu").as_bool();
    imu_abs_limit_rad_ = Deg2Rad(get_parameter("imu_abs_limit_deg").as_double());
    show_debug_view_ = get_parameter("show_debug_view").as_bool();
    show_yolo_debug_view_ =
        get_parameter("show_yolo_debug_view").as_bool();
    inference_hz_ = get_parameter("inference_hz").as_double();
    enable_ball_controller_ = get_parameter("enable_ball_controller").as_bool();
    enable_hurdle_controller_ = get_parameter("enable_hurdle_controller").as_bool();
    enable_goal_controller_ = get_parameter("enable_goal_controller").as_bool();
    algorithm_mode_ = get_parameter("algorithm_mode").as_string();
    simulate_camera_feedback_ = get_parameter("simulate_camera_feedback").as_bool();
    simulate_decision_ = get_parameter("simulate_decision").as_bool();
    simulate_action_duration_sec_ =
        get_parameter("simulate_action_duration_sec").as_double();
    simulate_step_forward_five_duration_sec_ =
        get_parameter("simulate_step_forward_five_duration_sec").as_double();
    simulate_curve_step_duration_sec_ =
        get_parameter("simulate_curve_step_duration_sec").as_double();
    simulate_step_forward_one_duration_sec_ =
        get_parameter("simulate_step_forward_one_duration_sec").as_double();
    simulate_turn_duration_sec_ =
        get_parameter("simulate_turn_duration_sec").as_double();
    simulate_ready_lead_sec_ =
        get_parameter("simulate_ready_lead_sec").as_double();
    enable_command_transport_ = get_parameter("enable_command_transport").as_bool();
    max_depth_age_sec_ = get_parameter("max_depth_age_sec").as_double();
    backboard_depth_strip_width_px_=static_cast<int>(get_parameter("backboard_depth_strip_width_px").as_int());
    if (algorithm_mode_ != "all" && algorithm_mode_ != "line" && algorithm_mode_ != "ball" &&
        algorithm_mode_ != "hurdle" && algorithm_mode_ != "goal") {
      throw std::invalid_argument("algorithm_mode must be one of: all, line, ball, hurdle, goal");
    }
#if defined(VISION_GOAL_TUNING_NODE) || defined(VISION_BALL_TUNING_NODE) || defined(VISION_HURDLE_TUNING_NODE)
#if defined(VISION_GOAL_TUNING_NODE)
    const std::string mission="goal";
#elif defined(VISION_BALL_TUNING_NODE)
    const std::string mission="ball";
#else
    const std::string mission="hurdle";
#endif
    if (algorithm_mode_!=mission) throw std::invalid_argument("Dedicated tuning node requires its matching algorithm_mode");
    dedicated_test_=true;
#endif
    vision_core::BallConfig ball_cmd_cfg = algorithm_defaults.ball;

    const double fx = get_parameter("fx").as_double();
    const double fy = get_parameter("fy").as_double();
    const double cx = get_parameter("cx").as_double();
    const double cy = get_parameter("cy").as_double();

    vision_core::FeatureConfig line_feature_cfg = algorithm_defaults.line_features;
    line_feature_cfg.max_centers = static_cast<int>(get_parameter("max_centers").as_int());
    line_feature_cfg.image_center_u = get_parameter("line_reference_u_px").as_double();

    if (engine_path_.empty()) {
      throw std::runtime_error("YOLO TensorRT engine 경로(engine_path)가 비어 있습니다.");
    }

    vision_core::HurdleConfig hurdle_cmd_cfg = algorithm_defaults.hurdle;
    vision_core::GoalConfig goal_cmd_cfg = algorithm_defaults.goal;
    goal_cmd_cfg.fine_adjust_start_z_m =
        get_parameter("goal_fine_adjust_start_z_m").as_double();
    goal_cmd_cfg.hoop_radius_m =
        get_parameter("goal_hoop_radius_m").as_double();
    goal_cmd_cfg.throw_landing_x_m =
        get_parameter("goal_throw_landing_x_m").as_double();
    goal_cmd_cfg.throw_landing_z_m =
        get_parameter("goal_throw_landing_z_m").as_double();
    goal_cmd_cfg.position_tolerance_m =
        get_parameter("goal_position_tolerance_m").as_double();
    goal_cmd_cfg.shoot_yaw_limit_deg =
        get_parameter("goal_shoot_yaw_limit_deg").as_double();
    ball_cmd_cfg.far_u_des_px = get_parameter("ball_far_u_des_px").as_double();
    ball_cmd_cfg.approach_u_deadband_px = get_parameter("ball_approach_u_deadband_px").as_double();
    ball_cmd_cfg.upper_acquire_v_px = get_parameter("ball_upper_acquire_v_px").as_double();
    ball_cmd_cfg.approach_two_step_v_px = get_parameter("ball_approach_two_step_v_px").as_double();
    ball_cmd_cfg.pickup_u1_px = get_parameter("ball_pickup_u1_px").as_double();
    ball_cmd_cfg.pickup_u2_px = get_parameter("ball_pickup_u2_px").as_double();
    ball_cmd_cfg.pickup_v1_px = get_parameter("ball_pickup_v1_px").as_double();
    ball_cmd_cfg.pickup_v2_px = get_parameter("ball_pickup_v2_px").as_double();
    goal_cmd_cfg.target_u_px = get_parameter("goal_target_u_px").as_double();
    goal_cmd_cfg.approach_u_deadband_px = get_parameter("goal_approach_u_deadband_px").as_double();
    hurdle_cmd_cfg.upper_acquire_v_px = get_parameter("hurdle_upper_acquire_v_px").as_double();
    hurdle_cmd_cfg.approach_two_step_v_px = get_parameter("hurdle_approach_two_step_v_px").as_double();
    vision_core::MissionControllerConfig mission_config = algorithm_defaults;
    mission_config.line_p2p.offset_gain = get_parameter("line_p2p_offset_gain").as_double();
    mission_config.line_p2p.heading_gain = get_parameter("line_p2p_heading_gain").as_double();
    mission_config.line_p2p.steering_deadband =
        get_parameter("line_p2p_steering_deadband").as_double();
    mission_config.backboard_depth_strip_width_px=backboard_depth_strip_width_px_;
    mission_config.camera_motion_timeout_sec =
        get_parameter("camera_motion_timeout_sec").as_double();
    mission_config.line_features = line_feature_cfg;
    mission_config.line = algorithm_defaults.line;
    mission_config.ball = ball_cmd_cfg;
    mission_config.hurdle = hurdle_cmd_cfg;
    mission_config.goal = goal_cmd_cfg;
    mission_config.line_detection.class_id = line_class_id_;
    mission_config.line_detection.confidence = conf_thres_;
    mission_config.object_targets.ball_class_id = ball_class_id_;
    mission_config.object_targets.goal_class_id = goal_class_id_;
    mission_config.object_targets.backboard_class_id = backboard_class_id_;
    mission_config.object_targets.hurdle_class_id = hurdle_class_id_;
    mission_config.object_targets.ball_confidence = ball_conf_thres_;
    mission_config.object_targets.goal_confidence = goal_conf_thres_;
    mission_config.object_targets.backboard_confidence =
        backboard_conf_thres_;
    mission_config.object_targets.hurdle_confidence = hurdle_conf_thres_;
    mission_config.object_targets.min_box_width =
        get_parameter("object_min_box_width").as_int();
    mission_config.object_targets.min_box_height =
        get_parameter("object_min_box_height").as_int();
    mission_config.backboard_min_depth_m =
        get_parameter("backboard_min_depth_m").as_double();
    mission_config.backboard_max_depth_m =
        get_parameter("backboard_max_depth_m").as_double();
    mission_config.enable_ball =
        algorithm_mode_ == "ball" ||
        (algorithm_mode_ == "all" && enable_ball_controller_);
    mission_config.enable_hurdle =
        algorithm_mode_ == "hurdle" ||
        (algorithm_mode_ == "all" && enable_hurdle_controller_);
    mission_config.enable_goal =
        algorithm_mode_ == "goal" ||
        (algorithm_mode_ == "all" && enable_goal_controller_);
    // Goal 단독 실행만 공 집기 과정을 생략한다. 통합 모드는 반드시 Ball
    // 성공 결과를 통해서만 has_ball이 켜진다.
    mission_config.initial_has_ball = algorithm_mode_ == "goal";
    mission_config.goal_trial_start_goal=algorithm_mode_=="goal" &&
        get_parameter("goal_trial_start_goal").as_bool();
    mission_config.test_stage=static_cast<int>(get_parameter("test_stage").as_int());
    if (dedicated_test_) mission_config.test_mission=algorithm_mode_=="ball" ? vision_core::MissionType::kBall :
        algorithm_mode_=="hurdle" ? vision_core::MissionType::kHurdle : vision_core::MissionType::kGoal;
    else { mission_config.test_mission=vision_core::MissionType::kNone; }
    if (!dedicated_test_ && mission_config.test_stage!=0) throw std::invalid_argument("test_stage requires a dedicated mission tuning node");
    mission_config.hurdle.contact_steps=static_cast<int>(get_parameter("hurdle_contact_steps").as_int());
    mission_config.mission_recovery_max_turns=static_cast<int>(get_parameter("mission_recovery_max_turns").as_int());
    mission_config.test_ball_mission_index=static_cast<int>(get_parameter("test_ball_mission_index").as_int());
    mission_config.test_goal_mission_index=static_cast<int>(get_parameter("test_goal_mission_index").as_int());
    mission_config.ball.down_one_step_v_px=get_parameter("ball_down_one_step_v_px").as_double();
    mission_config.ball.fine_mode_v_px=get_parameter("ball_fine_mode_v_px").as_double();
    mission_config.ball.approach_short_v_px=get_parameter("ball_approach_short_v_px").as_double();
    mission_config.hurdle.down_one_step_v_px=get_parameter("hurdle_down_one_step_v_px").as_double();
    mission_config.hurdle.fine_mode_v_px=get_parameter("hurdle_fine_mode_v_px").as_double();
    mission_config.hurdle.approach_short_v_px=get_parameter("hurdle_approach_short_v_px").as_double();
    mission_config.hurdle.target_u_px=get_parameter("hurdle_target_u_px").as_double();
    mission_config.hurdle.u_deadband_px=get_parameter("hurdle_u_deadband_px").as_double();
    mission_config.hurdle.contact_v_px=get_parameter("hurdle_contact_v_px").as_double();
    mission_config.goal.fine_one_step_range_error_m=get_parameter("goal_fine_one_step_range_error_m").as_double();
    mission_config.goal.fine_two_step_range_error_m=get_parameter("goal_fine_two_step_range_error_m").as_double();
    mission_config.goal.approach_short_z_m=get_parameter("goal_approach_short_z_m").as_double();
    mission_config.goal.camera_goal_start_z_m=get_parameter("goal_camera_goal_start_z_m").as_double();

    // ROS override가 shared YAML validation을 우회하지 못하게 하고, 무거운
    // TensorRT engine을 만들기 전에 fail-fast한다.
    vision::ReadMissionParameters(*this,mission_config);
    vision_core::ValidateAlgorithmConfig(mission_config);
    yolo_ = std::make_unique<YoloTrtEngine>(engine_path_, 640, 640,
                                            conf_thres_);
    tuning_config_ = mission_config;
    mission_controller_ =
        std::make_unique<vision_core::MissionController>(mission_config);

    camera_intrinsics_ = {fx, fy, cx, cy};

    // ----------------------------
    // ROS Pub/Sub
    // ----------------------------
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic_, rclcpp::SensorDataQoS(), std::bind(&PerceptionTuningNode::OnImage, this, std::placeholders::_1));
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        depth_topic_, rclcpp::SensorDataQoS(), std::bind(&PerceptionTuningNode::OnDepth, this, std::placeholders::_1));
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_, rclcpp::SensorDataQoS(),
        std::bind(&PerceptionTuningNode::OnCameraInfo, this, std::placeholders::_1));

    // 보정을 끄거나 zero-IMU 시험을 선택하면 입력 구독 자체를 만들지 않는다.
    if (use_imu_rectification_ && !assume_zero_imu_) {
      auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(10));
      imu_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
      imu_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
      imu_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
          imu_topic_, imu_qos,
          std::bind(&PerceptionTuningNode::OnImu, this,
                    std::placeholders::_1));
    }

    auto command_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    action_cmd_pub_ = create_publisher<vision::msg::ActionCommand>(action_cmd_topic_, command_qos);
    camera_cmd_pub_ = create_publisher<vision::msg::CameraCommand>(camera_cmd_topic_, command_qos);
    action_status_sub_ = create_subscription<vision::msg::CommandStatus>(
        action_status_topic_, command_qos,
        std::bind(&PerceptionTuningNode::OnActionStatus, this, std::placeholders::_1));
    camera_status_sub_ = create_subscription<vision::msg::CommandStatus>(
        camera_status_topic_, command_qos,
        std::bind(&PerceptionTuningNode::OnCameraStatus, this, std::placeholders::_1));


    // Like AUTO, manual tuning starts disarmed. Neither launch nor RESET may
    // start the 1-second observation; only an explicit START can do that.
    start_service_ = create_service<std_srvs::srv::Trigger>(
        ServiceName("start"), [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          const bool goal_stage_one = dedicated_test_ && algorithm_mode_ == "goal" &&
                                      tuning_config_.test_stage == 1;
          const bool direct_goal_start = !dedicated_test_ && tuning_config_.goal_trial_start_goal;
          if (run_state_ != RunState::kWaitStart || !mission_controller_->FeetReadyToStart() || pending_camera_id_ != 0 ||
              !outstanding_actions_.empty() || !outstanding_cameras_.empty() ||
              manual_action_active_ || preview_ready_ ||
              !camera_feedback_.settled ||
              (!direct_goal_start && !(dedicated_test_ && (tuning_config_.test_stage>=2 || goal_stage_one)) &&
               camera_feedback_.actual_mode != vision_core::CameraMode::kForward)) {
            response->success = false;
            response->message = "START requires WAIT_START, known foot stance and settled camera (FORWARD for full/non-GOAL far test)";
            return;
          }
          authorized_repeat_scope_=0; trial_finished_=false;
          mission_controller_->Reset(true); // Preserve issued action IDs.
          last_debug_result_={};
          observation_accumulator_.Reset();
          cached_ready_guide_.reset();
          preview_input_ = {};
          preview_result_ = {};
          preview_command_ = {};
          preview_image_.release();
          manual_wait_preview_ = false;
          manual_wait_commit_ = false;
          manual_observation_request_pending_ = false;
          manual_observation_session_ = false;
          observation_required_sec_ = 1.0;
          manual_wait_accumulated_sec_ = 0.0;
          if (direct_goal_start) run_state_=RunState::kRunning;
          else BeginFreshObservation(get_clock()->now().seconds());
          response->success = true;
          response->message = direct_goal_start ? "START accepted; preparing GOAL camera" : "START accepted; observing for 1.0 s";
          RCLCPP_INFO(get_logger(), "[CONTROL] %s", response->message.c_str());
        });
    preview_service_ = create_service<std_srvs::srv::Trigger>(
        ServiceName("preview"), [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          if (run_state_ != RunState::kRunning ||
              (!manual_wait_preview_ && !preview_ready_) ||
              manual_action_active_ || !outstanding_actions_.empty() ||
              !outstanding_cameras_.empty() || pending_camera_id_ != 0) {
            response->success = false;
            response->message = "PREVIEW requires WAIT_PREVIEW/WAIT_COMMIT and idle camera/body";
            return;
          }
          if (mission_controller_->IsTerminalHold()) {
            response->success=false; response->message="HOLD: recovery failed; RESET required"; return;
          }
          // Discarding a frozen candidate must also close WAIT_COMMIT
          // without letting its wall-clock duration advance mission timers.
          LeaveWaitCommit(get_clock()->now().seconds());
          LeaveWaitPreview(get_clock()->now().seconds());
          preview_ready_ = false;
          preview_image_.release();
          preview_command_ = {};
          cached_ready_guide_.reset();  // fresh PREVIEW supersedes old READY
          manual_observation_request_pending_ = true;
          manual_observation_session_ = true;
          observation_required_sec_ =
              std::max(1.0, mission_controller_->ManualPreviewDurationSec());
          BeginFreshObservation(get_clock()->now().seconds());
          response->success = true;
          response->message = "PREVIEW: collecting fresh " +
              std::to_string(observation_required_sec_) + "s observation";
        });

    commit_service_ = create_service<std_srvs::srv::Trigger>(
        ServiceName("commit"), [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          if (!preview_ready_ || manual_action_active_ ||
              !outstanding_actions_.empty() || !outstanding_cameras_.empty() ||
              pending_camera_id_ != 0 || run_state_ != RunState::kRunning) {
            response->success = false;
            response->message = "COMMIT requires WAIT_COMMIT and idle camera/body";
            return;
          }
          if (!enable_command_transport_ && !simulate_decision_) {
            response->success = false;
            response->message = "Command transport disabled: fake DONE prohibited";
            return;
          }
          if (preview_command_.command_type != vision_core::CommandType::kAction) {
            response->success = false;
            response->message = "No executable action; adjust RQT or press PREVIEW";
            return;
          }
          auto input = preview_input_;
          input.allow_new_commands = true;
          input.allow_new_body_actions = true;
          input.delivery_feedback = {};
          auto trial = *mission_controller_;
          const auto result = trial.StepPerception(input);
          const auto &cmd = result.mission.command;
          if (cmd.command_type != vision_core::CommandType::kAction ||
              cmd.action_id == 0 ||
              cmd.action_id != preview_command_.action_id ||
              cmd.action_category != preview_command_.action_category ||
              cmd.action != preview_command_.action ||
              cmd.target_yaw_deg != preview_command_.target_yaw_deg ||
              cmd.mission != preview_command_.mission ||
              cmd.mission_phase != preview_command_.mission_phase ||
              result.mission.repeat_scope != preview_result_.mission.repeat_scope) {
            response->success = false;
            response->message = "Candidate changed; press PREVIEW to refresh";
            LeaveWaitCommit(get_clock()->now().seconds());
            preview_ready_ = false;
            preview_image_.release();
            EnterWaitPreview(get_clock()->now().seconds());
            return;
          }
          *mission_controller_ = std::move(trial);
          authorized_repeat_scope_=result.mission.repeat_scope;
          LeaveWaitCommit(get_clock()->now().seconds());
          preview_ready_ = false;
          preview_image_.release();
          cached_ready_guide_.reset();
          manual_action_active_ = true;
          manual_committed_command_ = cmd;
          LeaveWaitPreview(get_clock()->now().seconds());
          ready_window_input_.reset();
          ready_motion_samples_.clear();
          manual_action_id_ = cmd.action_id;
          PublishActionCommand(cmd);
          response->success = true;
          response->message = "COMMIT executed id=" + std::to_string(cmd.action_id)+" / "+RepeatLabel(authorized_repeat_scope_);
          RCLCPP_INFO(get_logger(), "[TUNING] COMMIT id=%llu action=%u",
                      static_cast<unsigned long long>(cmd.action_id),
                      static_cast<unsigned>(cmd.action));
        });
    stop_repeat_service_=create_service<std_srvs::srv::Trigger>(
        ServiceName("stop_repeat"),[this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          authorized_repeat_scope_=0;
          response->success=true;
          response->message="Repeat permission revoked; accepted motion finishes normally";
          if (!manual_action_active_ && outstanding_actions_.empty() && pending_camera_id_==0)
            EnterWaitPreview(get_clock()->now().seconds());
        });
    reset_service_ = create_service<std_srvs::srv::Trigger>(
        ServiceName("reset"), [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          DrainExecutionFeedback();
          bool feedback_empty = false;
          {
            std::lock_guard<std::mutex> lock(action_feedback_mutex_);
            feedback_empty = action_feedback_queue_.empty();
          }
          if (manual_action_active_ || !outstanding_actions_.empty() ||
              !outstanding_cameras_.empty() || pending_camera_id_ != 0 ||
              !feedback_empty ||
              (simulate_decision_ &&
                  (simulated_current_action_.id != 0 ||
                   simulated_queued_action_.id != 0 ||
                   !simulated_action_feedback_queue_.empty()))) {
            response->success = false;
            response->message = "RESET rejected: action/camera/feedback active";
            return;
          }
          if (run_state_ != RunState::kRunning &&
              run_state_ != RunState::kObserving &&
              run_state_ != RunState::kWaitStart && run_state_ != RunState::kPaused) {
            response->success = false;
            response->message = "RESET requires RUNNING, OBSERVING, or WAIT_START";
            return;
          }
          if (!enable_command_transport_ && !simulate_decision_) {
            response->success = false;
            response->message = "RESET requires body transport or simulated executor";
            return;
          }
          authorized_repeat_scope_=0;
          applied_action_feedback_queue_.clear(); action_delivery_feedback_={};
          mission_controller_->BeginPhysicalReset();
          observation_accumulator_.Reset();
          preview_ready_ = false;
          manual_wait_preview_ = false;
          manual_wait_commit_ = false;
          manual_observation_request_pending_ = false;
          manual_observation_session_ = false;
          manual_camera_done_pending_ = false;
          manual_wait_accumulated_sec_ = 0.0;
          ready_window_input_.reset();
          ready_motion_samples_.clear();
          ready_window_image_.release();
          preview_image_.release();
          preview_input_ = {};
          preview_result_ = {};
          preview_command_ = {};
          cached_ready_guide_.reset();
          manual_action_id_ = 0;
          action_delivery_feedback_ = {};
          {
            std::lock_guard<std::mutex> lock(image_mutex_);
            latest_image_.reset();
          }
          // RESET is an explicit disarm. Camera goes FORWARD automatically,
          // but observing/previewing requires a NEW, separate START request.
          run_state_ = RunState::kResettingCamera;
          DrivePhysicalReset();
          response->success = true;
          response->message = "RESET: waiting for WALK feet and FORWARD camera DONE";
        });
    tuning_callback_ = add_on_set_parameters_callback(
        std::bind(&PerceptionTuningNode::OnTuningParameters, this,
                  std::placeholders::_1));
    // The constructor must not start perception decision timers.
    run_state_ = RunState::kWaitStart;
    RCLCPP_INFO(get_logger(),
        "[TUNING] WAIT_START; use /start, /preview, /commit, /reset");
    RCLCPP_INFO(get_logger(), "  image_topic    : %s", image_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  depth_topic    : %s", depth_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  camera_info    : %s", camera_info_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  imu_topic      : %s", imu_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  action_cmd     : %s", action_cmd_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  action_status  : %s", action_status_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  camera_cmd     : %s", camera_cmd_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  camera_status  : %s", camera_status_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  engine_path    : %s", engine_path_.c_str());
    RCLCPP_INFO(get_logger(), "  line_class_id  : %d", line_class_id_);
    RCLCPP_INFO(get_logger(), "  ball_class_id  : %d", ball_class_id_);
    RCLCPP_INFO(get_logger(), "  goal_class_id  : %d", goal_class_id_);
    RCLCPP_INFO(get_logger(), "  backboard_cls  : %d", backboard_class_id_);
    RCLCPP_INFO(get_logger(), "  hurdle_class_id: %d", hurdle_class_id_);
    RCLCPP_INFO(get_logger(), "  ball_conf_thres: %.2f", ball_conf_thres_);
    RCLCPP_INFO(get_logger(), "  goal_conf_thres: %.2f", goal_conf_thres_);
    RCLCPP_INFO(get_logger(), "  backboard_conf : %.2f", backboard_conf_thres_);
    RCLCPP_INFO(get_logger(), "  hurdle_conf_thr: %.2f", hurdle_conf_thres_);
    RCLCPP_INFO(get_logger(), "  inference_hz   : %.2f", inference_hz_);
    RCLCPP_INFO(get_logger(), "  imu_rectify    : %s", use_imu_rectification_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "  zero_imu       : %s", assume_zero_imu_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "  ball_ctrl      : %s", enable_ball_controller_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "  hurdle_ctrl    : %s", enable_hurdle_controller_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "  goal_ctrl      : %s", enable_goal_controller_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "  algorithm_mode : %s", algorithm_mode_.c_str());
    if (ball_conf_thres_ < conf_thres_) {
      RCLCPP_WARN(get_logger(),
                  "ball_conf_thres(%.2f) < conf_thres(%.2f), but YOLO "
                  "engine output is already filtered by conf_thres.",
                  ball_conf_thres_, conf_thres_);
    }
    if (goal_conf_thres_ < conf_thres_) {
      RCLCPP_WARN(get_logger(),
                  "goal_conf_thres(%.2f) < conf_thres(%.2f), but YOLO "
                  "engine output is already filtered by conf_thres.",
                  goal_conf_thres_, conf_thres_);
    }
    if (backboard_conf_thres_ < conf_thres_) {
      RCLCPP_WARN(get_logger(),
                  "backboard_conf_thres(%.2f) < conf_thres(%.2f), but YOLO "
                  "engine output is already filtered by conf_thres.",
                  backboard_conf_thres_, conf_thres_);
    }
    if (hurdle_conf_thres_ < conf_thres_) {
      RCLCPP_WARN(get_logger(),
                  "hurdle_conf_thres(%.2f) < conf_thres(%.2f), but YOLO "
                  "engine output is already filtered by conf_thres.",
                  hurdle_conf_thres_, conf_thres_);
    }

    RCLCPP_INFO(get_logger(), "[SETUP] image subscription created (SensorDataQoS)");
    if (!use_imu_rectification_) {
      RCLCPP_INFO(get_logger(), "[SETUP] IMU rectification disabled; no IMU subscription");
    } else if (assume_zero_imu_) {
      RCLCPP_INFO(get_logger(), "[SETUP] zero IMU assumption; no IMU subscription");
    } else if (imu_sub_) {
      RCLCPP_INFO(get_logger(), "[SETUP] imu subscription created");
    }

    if (show_debug_view_ || show_yolo_debug_view_) {
      try {
        if (show_debug_view_) {
          cv::namedWindow(kDebugWindowName, cv::WINDOW_NORMAL);
          cv::namedWindow(kStateWindowName, cv::WINDOW_NORMAL);
        }
        if (show_yolo_debug_view_) {
          cv::namedWindow(kYoloDebugWindowName, cv::WINDOW_NORMAL);
        }
        RCLCPP_INFO(get_logger(),
                    "[SETUP] confirmed_debug=%d yolo_debug=%d",
                    show_debug_view_ ? 1 : 0,
                    show_yolo_debug_view_ ? 1 : 0);
      } catch (const cv::Exception &e) {
        show_debug_view_ = false;
        show_yolo_debug_view_ = false;
        RCLCPP_WARN(get_logger(), "[SETUP] debug window disabled: %s", e.what());
      }
    }

    reset_timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() {
      if (run_state_==RunState::kResettingCamera) DrivePhysicalReset();
      else DrainExecutionFeedback();
    });

    // ---- heartbeat(1Hz) ----
    hb_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
      RCLCPP_INFO(get_logger(), "[HB] frames=%zu imu=%zu action_pub=%zu (last_img_stamp=%.9f)", frames_count_, imu_count_,
                  action_pub_count_, last_img_stamp_sec_);
    });

    if (inference_hz_ > 0.0) {
      const auto period =
          std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / inference_hz_));
      inference_timer_ = create_wall_timer(period, [this]() { ProcessLatestImage(); });
    }

    // ---- PERF(1초 갱신) 초기화: "오버레이만" ----
    last_report_time_ = this->get_clock()->now();
    perf_frame_count_ = 0;
    perf_infer_time_sec_ = 0.0;
    perf_loop_time_sec_ = 0.0;
    perf_text_ = "INFER: -- ms | LOOP: -- ms | FPS: --";
  }

  ~PerceptionTuningNode() override {
    if (show_debug_view_ || show_yolo_debug_view_) {
      try {
        if (show_debug_view_) {
          cv::destroyWindow(kDebugWindowName);
          cv::destroyWindow(kStateWindowName);
        }
        if (show_yolo_debug_view_) {
          cv::destroyWindow(kYoloDebugWindowName);
        }
      } catch (...) {
      }
    }
  }

private:
  rcl_interfaces::msg::SetParametersResult OnTuningParameters(
      const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult response;
    response.successful = true;
    const auto start_goal=std::find_if(parameters.begin(),parameters.end(),[](const auto& p) {
      return p.get_name()=="goal_trial_start_goal";
    });
    if (start_goal!=parameters.end()) {
      if (parameters.size()!=1 || dedicated_test_ || algorithm_mode_!="goal" ||
          start_goal->get_type()!=rclcpp::ParameterType::PARAMETER_BOOL ||
          run_state_!=RunState::kWaitStart || !outstanding_actions_.empty() ||
          !outstanding_cameras_.empty() || pending_camera_id_!=0) {
        response.successful=false;
        response.reason="goal_trial_start_goal: non-staged GOAL node, WAIT_START, bool; change separately";
        return response;
      }
      tuning_config_.goal_trial_start_goal=start_goal->as_bool();
      mission_controller_->SetGoalTrialStartGoal(start_goal->as_bool());
      return response;
    }
    const auto stage_parameter=std::find_if(parameters.begin(),parameters.end(),[](const auto& p) { return p.get_name()=="test_stage"; });
    if (stage_parameter!=parameters.end()) {
      if (parameters.size()!=1 || !dedicated_test_ ||
          stage_parameter->get_type()!=rclcpp::ParameterType::PARAMETER_INTEGER ||
          stage_parameter->as_int()<0 || stage_parameter->as_int()>3 ||
          run_state_!=RunState::kWaitStart || !outstanding_actions_.empty() || pending_camera_id_!=0 ||
          !mission_controller_->SetTestStage(tuning_config_.test_mission,static_cast<int>(stage_parameter->as_int()))) {
        response.successful=false; response.reason="test_stage: dedicated node, WAIT_START, integer 0..3; change separately"; return response;
      }
      tuning_config_.test_stage=static_cast<int>(stage_parameter->as_int()); trial_finished_=false; return response;
    }
    const bool edits_index=std::any_of(parameters.begin(),parameters.end(),[](const auto& p) {
      return p.get_name()=="test_ball_mission_index" || p.get_name()=="test_goal_mission_index";
    });
    if (edits_index) {
      int ball=tuning_config_.test_ball_mission_index, goal=tuning_config_.test_goal_mission_index;
      for (const auto& p:parameters) {
        if ((p.get_name()!="test_ball_mission_index" && p.get_name()!="test_goal_mission_index") ||
            p.get_type()!=rclcpp::ParameterType::PARAMETER_INTEGER || p.as_int()<1 || p.as_int()>1000000) {
          response.successful=false; response.reason="Set test indices separately, as integers in [1,1000000]"; return response;
        }
        if (p.get_name()=="test_ball_mission_index") ball=static_cast<int>(p.as_int());
        else goal=static_cast<int>(p.as_int());
      }
      if (run_state_!=RunState::kWaitStart || !outstanding_actions_.empty() || pending_camera_id_!=0 ||
          !mission_controller_->SetTestMissionIndices(ball,goal)) {
        response.successful=false; response.reason="Test indices may change in WAIT_START only"; return response;
      }
      tuning_config_.test_ball_mission_index=ball; tuning_config_.test_goal_mission_index=goal;
      return response;
    }
    auto candidate = tuning_config_;
    bool touched = false;
    for (const auto &parameter : parameters) {
      double *target = nullptr;
      const std::string &name = parameter.get_name();
      if (name=="ball_recovery_walk_turn_max" || name=="goal_recovery_turn_max") {
        if (parameter.get_type()!=rclcpp::ParameterType::PARAMETER_INTEGER || parameter.as_int()<0 || parameter.as_int()>1000000) {
          response.successful=false; response.reason="recovery turn maximum must be a nonnegative integer"; return response;
        }
        if (name=="ball_recovery_walk_turn_max") candidate.ball.recovery_walk_turn_max=parameter.as_int();
        else candidate.goal.recovery_turn_max=parameter.as_int();
        touched=true; continue;
      }
      if (name=="backboard_depth_strip_width_px") {
        if (parameter.get_type()!=rclcpp::ParameterType::PARAMETER_INTEGER || parameter.as_int()<1 || parameter.as_int()>640) {
          response.successful=false; response.reason="depth strip width must be an integer in [1,640]"; return response;
        }
        candidate.backboard_depth_strip_width_px=static_cast<int>(parameter.as_int()); touched=true; continue;
      }
      if (name=="mission_recovery_max_turns") {
        if (parameter.get_type()!=rclcpp::ParameterType::PARAMETER_INTEGER || parameter.as_int()<1 || parameter.as_int()>100) {
          response.successful=false; response.reason="mission_recovery_max_turns must be an integer in [1,100]"; return response;
        }
        candidate.mission_recovery_max_turns=static_cast<int>(parameter.as_int()); touched=true; continue;
      }
      if (name=="hurdle_contact_steps") {
        if (parameter.get_type()!=rclcpp::ParameterType::PARAMETER_INTEGER || parameter.as_int()<1 || parameter.as_int()>100) {
          response.successful=false; response.reason="hurdle_contact_steps must be an integer in [1,100]"; return response;
        }
        candidate.hurdle.contact_steps=static_cast<int>(parameter.as_int()); touched=true; continue;
      }
      if (name == "transition_settle_sec") target=&candidate.transition_settle_sec;
      else if (name == "fine_motion_settle_sec") target=&candidate.fine_motion_settle_sec;
      else if (name == "goal_edge_margin_px") target=&candidate.goal.edge_margin_px;
      else if (name == "goal_fine_one_step_range_error_m") target=&candidate.goal.fine_one_step_range_error_m;
      else if (name == "goal_fine_two_step_range_error_m") target=&candidate.goal.fine_two_step_range_error_m;
      else if (name == "ball_down_one_step_v_px") target=&candidate.ball.down_one_step_v_px;
      else if (name == "ball_fine_mode_v_px") target=&candidate.ball.fine_mode_v_px;
      else if (name == "ball_approach_short_v_px") target = &candidate.ball.approach_short_v_px;
      else if (name == "hurdle_down_one_step_v_px") target=&candidate.hurdle.down_one_step_v_px;
      else if (name == "hurdle_fine_mode_v_px") target=&candidate.hurdle.fine_mode_v_px;
      else if (name == "hurdle_approach_short_v_px") target = &candidate.hurdle.approach_short_v_px;
      else if (name == "hurdle_target_u_px") target = &candidate.hurdle.target_u_px;
      else if (name == "hurdle_u_deadband_px") target = &candidate.hurdle.u_deadband_px;
      else if (name == "hurdle_contact_v_px") target = &candidate.hurdle.contact_v_px;
      else if (name == "goal_approach_short_z_m") target = &candidate.goal.approach_short_z_m;
      else if (name == "goal_camera_goal_start_z_m") target = &candidate.goal.camera_goal_start_z_m;
      else if (name == "line_reference_u_px") target = &candidate.line_features.image_center_u;
      else if (name == "line_p2p_offset_gain") target = &candidate.line_p2p.offset_gain;
      else if (name == "line_p2p_heading_gain") target = &candidate.line_p2p.heading_gain;
      else if (name == "line_p2p_steering_deadband") target = &candidate.line_p2p.steering_deadband;
      else if (name == "ball_far_u_des_px") target = &candidate.ball.far_u_des_px;
      else if (name == "ball_approach_u_deadband_px") target = &candidate.ball.approach_u_deadband_px;
      else if (name == "ball_upper_acquire_v_px") target = &candidate.ball.upper_acquire_v_px;
      else if (name == "ball_approach_two_step_v_px") target = &candidate.ball.approach_two_step_v_px;
      else if (name == "ball_pickup_u1_px") target = &candidate.ball.pickup_u1_px;
      else if (name == "ball_pickup_u2_px") target = &candidate.ball.pickup_u2_px;
      else if (name == "ball_pickup_v1_px") target = &candidate.ball.pickup_v1_px;
      else if (name == "ball_pickup_v2_px") target = &candidate.ball.pickup_v2_px;
      else if (name == "goal_target_u_px") target = &candidate.goal.target_u_px;
      else if (name == "goal_approach_u_deadband_px") target = &candidate.goal.approach_u_deadband_px;
      else if (name == "goal_fine_adjust_start_z_m") target = &candidate.goal.fine_adjust_start_z_m;
      else if (name == "goal_throw_landing_x_m") target = &candidate.goal.throw_landing_x_m;
      else if (name == "goal_throw_landing_z_m") target = &candidate.goal.throw_landing_z_m;
      else if (name == "goal_position_tolerance_m") target = &candidate.goal.position_tolerance_m;
      else if (name == "goal_hoop_radius_m") target = &candidate.goal.hoop_radius_m;
      else if (name == "goal_shoot_yaw_limit_deg") target = &candidate.goal.shoot_yaw_limit_deg;
      else if (name == "backboard_min_depth_m") target = &candidate.backboard_min_depth_m;
      else if (name == "backboard_max_depth_m") target = &candidate.backboard_max_depth_m;
      else if (name == "hurdle_upper_acquire_v_px") target = &candidate.hurdle.upper_acquire_v_px;
      else if (name == "hurdle_approach_two_step_v_px") target = &candidate.hurdle.approach_two_step_v_px;
      if (!target) continue;
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        response.successful = false;
        response.reason = name + " must be double";
        return response;
      }
      *target = parameter.as_double();
      touched = true;
    }
    if (!touched) return response;
    if (candidate.line_features.image_center_u < 0.0 ||
        candidate.line_features.image_center_u >= 640.0 ||
        !std::isfinite(candidate.line_features.image_center_u)) {
      response.successful = false;
      response.reason = "line_reference_u_px must be finite and in [0,640)";
      return response;
    }
    if (run_state_ != RunState::kRunning ||
        (!preview_ready_ && !manual_wait_preview_) ||
        authorized_repeat_scope_!=0 || manual_action_active_ || !outstanding_actions_.empty() ||
        !outstanding_cameras_.empty() || pending_camera_id_ != 0) {
      response.successful = false;
      response.reason = "Tuning parameters may change in WAIT_PREVIEW/WAIT_COMMIT only";
      return response;
    }
    try {
      vision_core::ValidateAlgorithmConfig(candidate);
      // Update the trial FIRST. A preview failure must not leave the live
      // controller partially retuned after ROS rejects a parameter change.
      auto trial_controller = *mission_controller_;
      if (!trial_controller.UpdateTuning(candidate)) {
        response.successful = false;
        response.reason = "Core still has an active command";
        return response;
      }
      auto trial_input = preview_input_;
      auto trial_ready_guide = cached_ready_guide_;
      const double old_reference = tuning_config_.line_features.image_center_u;
      const double new_reference = candidate.line_features.image_center_u;
      if (trial_input.image_width > 1 && old_reference != new_reference) {
        const double delta = (old_reference - new_reference) /
            (0.5 * static_cast<double>(trial_input.image_width));
        if (trial_input.line_decision_guide_override &&
            trial_input.line_decision_guide_override->valid)
          trial_input.line_decision_guide_override->offset += delta;
        if (trial_ready_guide && trial_ready_guide->valid)
          trial_ready_guide->offset += delta;
      }
      // WAIT_PREVIEW has no frozen evidence; never accidentally advance a
      // speculative FSM on a zero-dimension/default PerceptionFrameInput.
      std::optional<vision_core::PerceptionMissionFrameResult> updated;
      const bool depth_observation_changed =
          candidate.backboard_depth_strip_width_px != backboard_depth_strip_width_px_ ||
          candidate.backboard_min_depth_m != tuning_config_.backboard_min_depth_m ||
          candidate.backboard_max_depth_m != tuning_config_.backboard_max_depth_m;
      if (preview_ready_ && !depth_observation_changed)
        updated = trial_controller.PreviewPerception(trial_input);
      *mission_controller_ = std::move(trial_controller);
      tuning_config_ = candidate;
      backboard_depth_strip_width_px_=candidate.backboard_depth_strip_width_px;
      if (preview_ready_ && depth_observation_changed) {
        LeaveWaitCommit(get_clock()->now().seconds());
        preview_ready_=false; preview_command_={}; preview_image_.release();
        EnterWaitPreview(get_clock()->now().seconds());
        RCLCPP_INFO(get_logger(),"Depth sampling/filter changed: fresh PREVIEW required");
      } else if (preview_ready_) {
        preview_input_ = std::move(trial_input);
        preview_result_ = *updated;
        preview_command_ = updated->mission.command;
      }
      cached_ready_guide_ = std::move(trial_ready_guide);
    } catch (const std::exception &e) {
      response.successful = false;
      response.reason = e.what();
    }
    return response;
  }

  enum class RunState { kWaitStart, kObserving, kRunning, kResettingCamera, kPausing, kPaused };
  static const char *RunStateName(RunState s) {
    switch (s) {
    case RunState::kWaitStart: return "WAIT_START";
    case RunState::kObserving: return "OBSERVING";
    case RunState::kRunning: return "RUNNING";
    case RunState::kResettingCamera: return "RESET_CAMERA";
    case RunState::kPausing: return "PAUSING";
    case RunState::kPaused: return "PAUSED";
    }
    return "UNKNOWN";
  }

  std::string ServiceName(const char* name) const {
    return dedicated_test_ ? std::string("~/")+name : std::string(name);
  }

  std::string RepeatLabel(int scope) const {
    return vision::DebugRepeatLabel(scope,tuning_config_);
  }
  bool IsReadyWalk(vision_core::MissionAction action) const {
    using vision_core::MissionAction;
    // LINE READY uses its original 11/12/13 protocol, regardless of YAML.
    // Object mean eligibility is shared with AUTO via the loaded core config.
    const auto mission = mission_controller_->ActiveMission();
    if (manual_committed_command_.mission != mission) return false;
    if (mission == vision_core::MissionType::kLine)
      return action == MissionAction::kStepForwardLeft ||
             action == MissionAction::kStepForwardFive ||
             action == MissionAction::kStepForwardRight;
    return tuning_config_.UsesObjectReadyMean(action);
  }
  static bool IsFixedSuccessor(vision_core::MissionType mission,
                              vision_core::MissionAction action) {
    using vision_core::MissionAction;
    using vision_core::MissionType;
    return action == MissionAction::kPickBall ||
        action == MissionAction::kDefaultPosition;
  }
  void EnterWaitPreview(double now) {
    if (manual_wait_preview_) return;
    manual_wait_preview_ = true;
    manual_wait_preview_started_sec_ = now;
  }
  void LeaveWaitPreview(double now) {
    if (!manual_wait_preview_) return;
    manual_wait_accumulated_sec_ += std::max(0.0, now -
        manual_wait_preview_started_sec_);
    manual_wait_preview_ = false;
  }
  void EnterWaitCommit(double now) {
    if (manual_wait_commit_) return;
    manual_wait_commit_ = true;
    manual_wait_commit_started_sec_ = now;
  }
  void LeaveWaitCommit(double now) {
    if (!manual_wait_commit_) return;
    manual_wait_accumulated_sec_ += std::max(0.0, now -
        manual_wait_commit_started_sec_);
    manual_wait_commit_ = false;
  }

  // Samples come from the current committed motion only. The motion's
  // first processed ACK-era image approximates motion start; READY is
  // approximately ready_lead_sec before actual completion.
  void AddReadyMotionSample(const vision_core::PerceptionFrameInput &input,
                            const vision_core::PreparedPerceptionFrame &prepared) {
    if (!manual_action_active_ || !IsReadyWalk(manual_committed_command_.action) ||
        !input.camera_feedback.settled) return;
    ready_motion_samples_.push_back({input.now_sec, input, prepared});
    constexpr std::size_t kMaxMotionSamples = 450;
    if (ready_motion_samples_.size() > kMaxMotionSamples)
      ready_motion_samples_.pop_front();
  }

  std::optional<vision_core::PerceptionFrameInput>
  BuildReadyWindowInput(vision_core::MissionType mission, double ready_sec) const {
    if (ready_motion_samples_.empty()) return std::nullopt;
    const double started = ready_motion_samples_.front().now_sec;
    const double total = std::max(0.0, ready_sec - started) +
        std::max(0.0, tuning_config_.line_p2p.ready_lead_sec);
    const double cutoff = started + 0.40 * total;
    std::vector<const ReadyMotionSample *> window;
    const auto final_camera_mode =
        ready_motion_samples_.back().input.camera_feedback.actual_mode;
    for (const auto &sample : ready_motion_samples_) {
      // Never mix FORWARD/DOWN/GOAL pixel coordinates in one mean.
      if (sample.now_sec >= cutoff && sample.now_sec <= ready_sec &&
          sample.input.camera_feedback.settled &&
          sample.input.camera_feedback.actual_mode == final_camera_mode)
        window.push_back(&sample);
    }
    if (window.empty()) return std::nullopt;
    auto result = window.back()->input;
    result.delivery_feedback = {};
    result.begin_manual_observation = false;
    result.manual_ready_mean = mission != vision_core::MissionType::kLine;
    result.line_decision_guide_override.reset();
    if (mission == vision_core::MissionType::kLine) {
      if (!cached_ready_guide_ || !cached_ready_guide_->valid)
        return std::nullopt;
      result.line_decision_guide_override = cached_ready_guide_;
      return result;
    }
    if (window.size() < 10) return std::nullopt;
    const auto target_of = [mission](const ReadyMotionSample &sample)
        -> const std::optional<vision_core::ObjectTarget> & {
      if (mission == vision_core::MissionType::kBall)
        return sample.prepared.targets.ball;
      if (mission == vision_core::MissionType::kGoal)
        return sample.prepared.targets.backboard;
      return sample.prepared.targets.hurdle;
    };
    int last_ten_hits = 0;
    for (std::size_t j = window.size() - 10; j < window.size(); ++j)
      if (target_of(*window[j])) ++last_ten_hits;
    if (last_ten_hits < 7) return std::nullopt;
    result.manual_object_hits.clear();
    result.manual_pose_hits.clear();
    result.manual_near_hits.clear();
    for (std::size_t j = window.size()-10; j < window.size(); ++j) {
      result.manual_object_hits.push_back(target_of(*window[j]).has_value());
      const auto& pose=window[j]->prepared.goal_pose;
      result.manual_pose_hits.push_back(target_of(*window[j]) && pose.valid &&
          std::isfinite(pose.x_m) && std::isfinite(pose.z_m) && pose.z_m>0 &&
          std::isfinite(pose.yaw_rad));
      result.manual_near_hits.push_back(result.manual_pose_hits.back() && pose.z_m<=tuning_config_.goal.fine_adjust_start_z_m);
    }

    double sx=0, sy=0, sw=0, sh=0, confidence=0;
    double center_depth=0, left_depth=0, right_depth=0, left_u=0, right_u=0;
    int hits=0, center_hits=0, left_hits=0, right_hits=0;
    double object_weight=0.0;
    double last_seen_sec = -1.0;
    int class_id=0;
    for (const auto *sample : window) {
      const auto &obj = target_of(*sample);
      if (!obj) continue;
      ++hits;
      last_seen_sec = sample->now_sec;
      class_id = obj->class_id;
      const double weight=mission==vision_core::MissionType::kGoal ? 1.0 :
          1.0+2.0*std::clamp((sample->now_sec-cutoff)/std::max(1e-9,ready_sec-cutoff),0.0,1.0);
      object_weight+=weight;
      sx += weight*obj->box_px.x; sy += weight*obj->box_px.y;
      sw += weight*obj->box_px.width; sh += weight*obj->box_px.height;
      confidence += weight*obj->confidence;
      if (mission == vision_core::MissionType::kGoal) {
        const auto &prepared = sample->prepared;
        if (prepared.backboard_center_depth_m &&
            std::isfinite(*prepared.backboard_center_depth_m)) {
          center_depth += *prepared.backboard_center_depth_m;
          ++center_hits;
        }
        if (prepared.backboard_left_depth_m &&
            std::isfinite(*prepared.backboard_left_depth_m)) {
          left_depth += *prepared.backboard_left_depth_m; left_u+=prepared.backboard_left_depth_u_px.value_or(obj->box_px.x); ++left_hits;
        }
        if (prepared.backboard_right_depth_m &&
            std::isfinite(*prepared.backboard_right_depth_m)) {
          right_depth += *prepared.backboard_right_depth_m; right_u+=prepared.backboard_right_depth_u_px.value_or(obj->box_px.x+obj->box_px.width-1); ++right_hits;
        }
      }
    }
    if (hits < 7 || last_seen_sec < 0.0 ||
        ready_sec - last_seen_sec > 0.30) return std::nullopt;
    if (mission == vision_core::MissionType::kGoal && center_hits < 7)
      return std::nullopt;
    vision_core::PerceptionDetection mean;
    mean.detection.class_id = class_id;
    mean.detection.confidence = confidence / object_weight;
    mean.detection.box = {sx / object_weight, sy / object_weight, sw / object_weight, sh / object_weight};
    if (center_hits >= 7) mean.center_depth_m = center_depth / center_hits;
    if (left_hits >= 7) { mean.left_depth_m = left_depth / left_hits; mean.left_depth_u_px=left_u/left_hits; }
    if (right_hits >= 7) { mean.right_depth_m = right_depth / right_hits; mean.right_depth_u_px=right_u/right_hits; }
    // Only the tracked object corresponding to the committed mission may
    // generate the next object Action; no competing old RGB detections.
    result.detections.clear();
    result.detections.push_back(mean);
    return result;
  }

  void BeginFreshObservation(double now) {
    ++observation_cycle_id_;
    if (observation_cycle_id_ == 0) ++observation_cycle_id_;
    observation_start_sec_ = now;
    observation_accumulator_.Begin(observation_cycle_id_, now);
    {
      std::lock_guard<std::mutex> lock(image_mutex_);
      latest_image_.reset();
    }
    run_state_ = RunState::kObserving;
  }

  void CompletePauseAfterFeedback() {
    if (run_state_ != RunState::kPausing ||
        !outstanding_actions_.empty() || !outstanding_cameras_.empty() ||
        pending_camera_id_ != 0 || (simulate_decision_ &&
            (simulated_current_action_.id != 0 || simulated_queued_action_.id != 0)))
      return;
    {
      std::lock_guard<std::mutex> lock(action_feedback_mutex_);
      if (!action_feedback_queue_.empty()) return;
    }
    run_state_ = RunState::kPaused;
    RCLCPP_INFO(get_logger(),
                "[CONTROL] PAUSED: all published motions and camera commands completed");
  }

  static constexpr const char *kDebugWindowName = "Vision Confirmed Debug";
  static constexpr const char *kStateWindowName = "Vision State Debug";
  static constexpr const char *kYoloDebugWindowName = "YOLO Detection Debug";
  static double Deg2Rad(const double deg) {
    return deg * M_PI / 180.0;
  }



  static const char *MissionName(vision_core::MissionType mission) {
    switch (mission) {
      case vision_core::MissionType::kLine: return "LINE";
      case vision_core::MissionType::kBall: return "BALL";
      case vision_core::MissionType::kGoal: return "GOAL";
      case vision_core::MissionType::kHurdle: return "HURDLE";
      default: return "NONE";
    }
  }

  static const char *ActionName(vision_core::MissionAction action) {
    return vision::DebugActionName(action);
  }

  vision::MissionDebugContext DebugContext(bool imu_ready,double roll,double pitch) const {
    vision::MissionDebugContext view;
    view.control_state=RunStateName(run_state_);
    view.performance=perf_text_;
    view.waiting_action_id=outstanding_actions_.empty() ? 0 : *outstanding_actions_.begin();
    view.last_received_done_id=last_received_done_id_;
    view.last_processed_done_id=last_processed_done_id_;
    view.imu_enabled=use_imu_rectification_;
    view.imu_ready=imu_ready;
    view.roll_rad=roll;view.pitch_rad=pitch;
    view.control_state=preview_ready_ ? "WAIT_COMMIT" : manual_action_active_ ? "WAIT_DONE" : manual_wait_preview_ ? "WAIT_PREVIEW" : RunStateName(run_state_);
    view.preview_ready=preview_ready_;
    view.approved_repeat_scope=authorized_repeat_scope_;
    view.control_hint="PREVIEW / COMMIT / PAUSE / RESUME / RESET";
    return view;
  }

  bool ShouldDrawDebugView() const {
    return show_debug_view_ || show_yolo_debug_view_;
  }
  static double Clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
  }

  static double StampSeconds(const std_msgs::msg::Header &header) {
    return static_cast<double>(header.stamp.sec) + static_cast<double>(header.stamp.nanosec) * 1e-9;
  }

  std::vector<vision_core::PerceptionDetection> BuildPerceptionDetections(
      const std::vector<Detection> &detections, const cv::Mat &depth) const {
    std::vector<vision_core::PerceptionDetection> output;
    output.reserve(detections.size());
    for (const auto &detection : detections) {
      vision_core::PerceptionDetection observation;
      observation.detection = {
          {static_cast<double>(detection.box.x),
           static_cast<double>(detection.box.y),
           static_cast<double>(detection.box.width),
           static_cast<double>(detection.box.height)},
          detection.confidence, detection.class_id};
      // 모든 backboard 후보에 표본을 붙인다. 최종 후보 선택과 pose 계산은
      // core에서 한 번만 수행한다.
      if (detection.class_id == backboard_class_id_ && !depth.empty()) {
        const auto region=vision::DepthRegions(detection.box,depth.size(),backboard_depth_strip_width_px_);
        observation.center_depth_m=vision::MedianRegionMeters(depth,region.center);
        observation.left_depth_m=vision::MedianRegionMeters(depth,region.left);
        observation.right_depth_m=vision::MedianRegionMeters(depth,region.right);
        if (!region.left.empty()) observation.left_depth_u_px=region.left_u;
        if (!region.right.empty()) observation.right_depth_u_px=region.right_u;
      }
      output.push_back(observation);
    }
    return output;
  }

  struct SimulatedActionState {
    std::uint64_t id{0};
    vision_core::MissionAction action{vision_core::MissionAction::kNone};
    vision_core::ActionCategory category{vision_core::ActionCategory::kNone};
    double start_sec{0.0};
    double duration_sec{0.0};
    bool acknowledged{false};
    bool ready_sent{false};
  };

  static bool IsSimulatedLongWalk(vision_core::MissionAction action) {
    using vision_core::MissionAction;
    // Mirror forward_walk's READY protocol independently of the YAML
    // object-average allowlist, which controls perception only.
    return action == MissionAction::kStepForwardLeft ||
           action == MissionAction::kStepForwardRight ||
           action == MissionAction::kStepForwardFive ||
           action == MissionAction::kStepForwardTwo ||
           action == MissionAction::kStepForwardThree ||
           action == MissionAction::kStepForwardFour ||
           action == MissionAction::kStepForwardLeftTwo ||
           action == MissionAction::kStepForwardRightTwo ||
           action == MissionAction::kStepForwardRightOne;
  }

  void QueueSimulatedActionFeedback(std::uint64_t action_id,
                                    bool acknowledged, bool ready,
                                    bool done) {
    if (action_id == 0) return;
    for (const auto &pending : simulated_action_feedback_queue_) {
      if (pending.action_id == action_id &&
          pending.acknowledged == acknowledged &&
          pending.ready == ready && pending.done == done) {
        return;
      }
    }
    simulated_action_feedback_queue_.push_back(
        {action_id, acknowledged, done, ready});
    if (done) {
      outstanding_actions_.erase(action_id);
      if (manual_action_id_ == action_id) manual_action_active_ = false;
    }
  }

  void RememberCompletedSimulatedAction(
      const SimulatedActionState &completed) {
    simulated_completed_actions_.push_back(completed);
    constexpr std::size_t kCompletedHistoryLimit = 64;
    while (simulated_completed_actions_.size() > kCompletedHistoryLimit) {
      simulated_completed_actions_.pop_front();
    }
  }

  const SimulatedActionState *FindCompletedSimulatedAction(
      std::uint64_t action_id) const {
    for (auto it = simulated_completed_actions_.rbegin();
         it != simulated_completed_actions_.rend(); ++it) {
      if (it->id == action_id) return &*it;
    }
    return nullptr;
  }

  void UpdateSimulatedDecisionFeedback(double now_sec) {
    if (!simulate_decision_) return;
    if (simulated_current_action_.id != 0) {
      auto &current = simulated_current_action_;
      const double elapsed = std::max(0.0, now_sec - current.start_sec);
      const double ready_at = std::max(
          0.0,
          current.duration_sec - std::max(0.0, simulate_ready_lead_sec_));
      if (IsSimulatedLongWalk(current.action) && !current.ready_sent &&
          elapsed >= ready_at) {
        current.ready_sent = true;
        QueueSimulatedActionFeedback(current.id, true, true, false);
      } else if (elapsed >= current.duration_sec) {
        const SimulatedActionState completed = current;
        QueueSimulatedActionFeedback(completed.id, true, false, true);
        RememberCompletedSimulatedAction(completed);
        if (simulated_queued_action_.id != 0) {
          simulated_current_action_ = simulated_queued_action_;
          simulated_current_action_.start_sec = now_sec;
          simulated_current_action_.ready_sent = false;
          simulated_queued_action_ = {};
        } else {
          simulated_current_action_ = {};
        }
      }
    }
    if (action_delivery_feedback_.action_id == 0 &&
        !simulated_action_feedback_queue_.empty()) {
      action_delivery_feedback_ = simulated_action_feedback_queue_.front();
      simulated_action_feedback_queue_.pop_front();
    }
  }

  double SimulatedActionDurationSec(
      vision_core::MissionAction action) const {
    switch (action) {
    case vision_core::MissionAction::kStepForwardTwo: return 2.0*simulate_step_forward_one_duration_sec_;
    case vision_core::MissionAction::kStepForwardThree: return 3.0*simulate_step_forward_one_duration_sec_;
    case vision_core::MissionAction::kStepForwardFour: return 4.0*simulate_step_forward_one_duration_sec_;
    case vision_core::MissionAction::kStepForwardLeftTwo:
    case vision_core::MissionAction::kStepForwardRightTwo: return 2.0*simulate_curve_step_duration_sec_/3.0;
    case vision_core::MissionAction::kStepForwardRightOne: return simulate_curve_step_duration_sec_/3.0;
    case vision_core::MissionAction::kStepForwardFive:
    case vision_core::MissionAction::kTurnLeftAndStep:
    case vision_core::MissionAction::kTurnRightAndStep:
      return std::max(0.0, simulate_step_forward_five_duration_sec_);
    case vision_core::MissionAction::kStepForwardLeft:
    case vision_core::MissionAction::kStepForwardRight:
      return std::max(0.0, simulate_curve_step_duration_sec_);
    case vision_core::MissionAction::kStepForwardOne:
    case vision_core::MissionAction::kStepBack:
    case vision_core::MissionAction::kLeftSideStep:
    case vision_core::MissionAction::kRightSideStep:
      return std::max(0.0, simulate_step_forward_one_duration_sec_);
    case vision_core::MissionAction::kTurnLeft:
    case vision_core::MissionAction::kTurnRight:
      return std::max(0.0, simulate_turn_duration_sec_);
    default:
      return std::max(0.0, simulate_action_duration_sec_);
    }
  }

  void OnActionStatus(const vision::msg::CommandStatus::SharedPtr msg) {
    if (msg->command_id == 0 || simulate_decision_ ||
        outstanding_actions_.count(msg->command_id) == 0) return;
    vision_core::CommandDeliveryFeedback feedback;
    feedback.action_id = msg->command_id;
    if (msg->status == vision::msg::CommandStatus::ACK) {
      feedback.acknowledged = true;
    } else if (msg->status == vision::msg::CommandStatus::READY) {
      // READY는 해당 action이 이미 수락되어 실행 중이라는 뜻도 포함한다.
      feedback.acknowledged = true;
      feedback.ready = true;
    } else if (msg->status == vision::msg::CommandStatus::DONE) {
      // Stamp the event in the same ROS clock domain as incoming images.
      // Queued frames captured BEFORE DONE must never count as new evidence.
      last_action_done_event_sec_ = this->get_clock()->now().seconds();
      feedback.acknowledged = true;
      feedback.done = true;
      outstanding_actions_.erase(msg->command_id);
      if (manual_action_id_ == msg->command_id) manual_action_active_ = false;
    } else {
      return;
    }
    std::lock_guard<std::mutex> lock(action_feedback_mutex_);
    action_feedback_queue_.push_back(feedback);
  }

  void OnCameraStatus(const vision::msg::CommandStatus::SharedPtr msg) {
    if (msg->command_id == 0 || msg->command_id != pending_camera_id_) return;
    if (msg->status == vision::msg::CommandStatus::ACK) {
      camera_acknowledged_ = true;
      return;
    }
    if (msg->status != vision::msg::CommandStatus::DONE) return;
    last_camera_done_event_sec_ = this->get_clock()->now().seconds();
    RCLCPP_INFO(get_logger(),"[FEEDBACK] CAMERA DONE id=%llu received; awaiting fresh image",
        static_cast<unsigned long long>(msg->command_id));
    switch (pending_camera_request_) {
    case vision_core::CameraRequest::kDown:
      camera_feedback_ = {vision_core::CameraMode::kDown, true};
      break;
    case vision_core::CameraRequest::kForward:
      camera_feedback_ = {vision_core::CameraMode::kForward, true};
      break;
    case vision_core::CameraRequest::kGoal:
      camera_feedback_ = {vision_core::CameraMode::kGoal, true};
      break;
    case vision_core::CameraRequest::kNone:
      break;
    }
    outstanding_cameras_.erase(msg->command_id);
    pending_camera_id_ = 0;
    pending_camera_request_ = vision_core::CameraRequest::kNone;
    camera_acknowledged_ = false;
    if (run_state_ == RunState::kRunning)
      manual_camera_done_pending_ = true;

  }

  double ExecutionClock(double now) const {
    const double wait=manual_wait_preview_ ? std::max(0.0,now-manual_wait_preview_started_sec_) :
        manual_wait_commit_ ? std::max(0.0,now-manual_wait_commit_started_sec_) : 0.0;
    return (run_state_==RunState::kPausing ? pause_requested_sec_ : now)-
        paused_accumulated_sec_-manual_wait_accumulated_sec_-wait;
  }
  void DrainExecutionFeedback() {
    const double now=get_clock()->now().seconds();
    std::deque<vision_core::CommandDeliveryFeedback> arrived;
    if (simulate_decision_) {
      UpdateSimulatedDecisionFeedback(now);
      if (action_delivery_feedback_.action_id!=0) {
        arrived.push_back(action_delivery_feedback_); action_delivery_feedback_={};
      }
    }
    {
      std::lock_guard<std::mutex> lock(action_feedback_mutex_);
      while (!action_feedback_queue_.empty()) {
        arrived.push_back(action_feedback_queue_.front()); action_feedback_queue_.pop_front();
      }
    }
    for (const auto& feedback:arrived) {
      last_transport_command_=mission_controller_->AcceptDeliveryFeedback(feedback,ExecutionClock(now));
      applied_action_feedback_queue_.push_back(feedback);
      if (feedback.done) {
        last_received_done_id_=feedback.action_id;
        RCLCPP_INFO(get_logger(),"[FEEDBACK] DONE id=%llu received and accepted by core; awaiting fresh observation",
            static_cast<unsigned long long>(feedback.action_id));
      }
    }
    last_transport_command_=mission_controller_->AcceptDeliveryFeedback({},ExecutionClock(now));
    // Only retransmit IDs already dispatched. Never select a new body/camera action here.
    if (last_transport_command_.command_type==vision_core::CommandType::kAction &&
        outstanding_actions_.count(last_transport_command_.action_id)!=0)
      PublishActionCommand(last_transport_command_);
    if (pending_camera_id_!=0) PublishCameraCommand(pending_camera_request_);
    // Physical PAUSE completion depends on received DONE, not availability of RGB.
    // Keep the ordered sensory edges for RESUME, or discard them on explicit RESET.
    CompletePauseAfterFeedback();
  }

  void DrivePhysicalReset() {
    if (run_state_!=RunState::kResettingCamera) return;
    const double now=get_clock()->now().seconds();
    if (simulate_decision_) UpdateSimulatedDecisionFeedback(now);
    vision_core::CommandDeliveryFeedback feedback;
    if (simulate_decision_ && action_delivery_feedback_.action_id!=0) {
      feedback=action_delivery_feedback_; action_delivery_feedback_={};
    }
    {
      std::lock_guard<std::mutex> lock(action_feedback_mutex_);
      if (!action_feedback_queue_.empty()) { feedback=action_feedback_queue_.front(); action_feedback_queue_.pop_front(); }
    }
    const auto result=mission_controller_->StepPhysicalReset(feedback,camera_feedback_,now);
    last_debug_result_={};
    last_debug_result_.mission=result;
    PublishActionCommand(result.command);
    PublishCameraCommand(result.command.camera_request);
    if (result.physical_reset_complete && outstanding_actions_.empty() && outstanding_cameras_.empty()) {
      run_state_=RunState::kWaitStart;
      manual_action_active_=false; manual_action_id_=0;
      RCLCPP_INFO(get_logger(),"[RESET] WALK feet and %s camera DONE -> WAIT_START",
          camera_feedback_.actual_mode==vision_core::CameraMode::kGoal ? "GOAL" : "FORWARD");
    }
  }

  void PublishActionCommand(const vision_core::ControlCommand &command) {
    if (command.command_type != vision_core::CommandType::kAction) return;
    // During PAUSING, only retransmit IDs published before PAUSE. Never
    // create or dispatch another motion (including queued LINE actions).
    if (run_state_ == RunState::kPausing &&
        outstanding_actions_.count(command.action_id) == 0) return;
    if (run_state_ != RunState::kRunning &&
        run_state_ != RunState::kPausing && run_state_ != RunState::kResettingCamera) return;
    outstanding_actions_.insert(command.action_id);
    if (simulate_decision_) {
      if (simulated_current_action_.id == command.action_id) {
        // 같은 ID 재전송은 동작 시작 시각을 건드리지 않고 ACK만 재응답한다.
        QueueSimulatedActionFeedback(command.action_id, true, false, false);
      } else if (simulated_queued_action_.id == command.action_id) {
        QueueSimulatedActionFeedback(command.action_id, true, false, false);
      } else if (FindCompletedSimulatedAction(command.action_id) != nullptr) {
        // DONE 유실 뒤 늦게 온 동일 ID도 절대 다시 실행하지 않는다.
        QueueSimulatedActionFeedback(command.action_id, true, false, true);
      } else {
        SimulatedActionState next;
        next.id = command.action_id;
        next.action = command.action;
        next.category = command.action_category;
        next.start_sec = this->get_clock()->now().seconds();
        next.duration_sec = SimulatedActionDurationSec(command.action);
        next.acknowledged = true;
        if (simulated_current_action_.id != 0) {
          if (simulated_current_action_.ready_sent &&
              simulated_queued_action_.id == 0 &&
              command.control_phase ==
                  vision_core::ControlPhase::kWaitingQueuedActionAck) {
            // 실행 중에는 잠근다. READY 뒤에 발급된 서로 다른 ID 하나만
            // 예약하고 현재 동작의 DONE 전에는 시작하지 않는다.
            simulated_queued_action_ = next;
            QueueSimulatedActionFeedback(next.id, true, false, false);
          }
        } else {
          simulated_current_action_ = next;
          QueueSimulatedActionFeedback(next.id, true, false, false);
        }
      }
      return;
    }
    if (!enable_command_transport_) {
      outstanding_actions_.erase(command.action_id);
      return;
    }

    vision::msg::ActionCommand message;
    message.action_id = command.action_id;
    message.mission = static_cast<std::uint8_t>(command.mission);
    message.action = static_cast<std::uint16_t>(command.action);
    message.target_yaw_deg = command.target_yaw_deg;
    action_cmd_pub_->publish(message);
    ++action_pub_count_;
  }

  void PublishCameraCommand(vision_core::CameraRequest request) {
    if (request == vision_core::CameraRequest::kNone) return;
    // The node may only originate new camera requests in RUNNING.
    if (run_state_ != RunState::kRunning &&
        !(run_state_ == RunState::kResettingCamera &&
          (request == vision_core::CameraRequest::kForward || request == vision_core::CameraRequest::kGoal))) return;
    if (simulate_camera_feedback_) {
      switch (request) {
      case vision_core::CameraRequest::kDown:
        camera_feedback_ = {vision_core::CameraMode::kDown, true};
        break;
      case vision_core::CameraRequest::kForward:
        camera_feedback_ = {vision_core::CameraMode::kForward, true};
        break;
      case vision_core::CameraRequest::kGoal:
        camera_feedback_ = {vision_core::CameraMode::kGoal, true};
        break;
      case vision_core::CameraRequest::kNone:
        break;
      }
      manual_camera_done_pending_ = true;
      return;
    }
    if (!enable_command_transport_) return;
    // Camera commands are serialized: a new request must not overwrite an
    // in-flight command's ID, even if the desired camera mode changes.
    // Keep retransmitting the original request until ACK, then wait for DONE.
    if (pending_camera_id_ == 0) {
      pending_camera_id_ = next_camera_id_++;
      outstanding_cameras_.insert(pending_camera_id_);
      pending_camera_request_ = request;
      camera_acknowledged_ = false;
      camera_feedback_.settled = false;
      camera_feedback_.actual_mode = vision_core::CameraMode::kTransition;
    } else if (pending_camera_request_ != request) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Camera request %u deferred until pending command id=%llu request=%u completes",
          static_cast<unsigned int>(request),
          static_cast<unsigned long long>(pending_camera_id_),
          static_cast<unsigned int>(pending_camera_request_));
    }
    if (camera_acknowledged_) return;
    vision::msg::CameraCommand message;
    message.command_id = pending_camera_id_;
    message.request = static_cast<std::uint8_t>(pending_camera_request_);
    camera_cmd_pub_->publish(message);
  }

  void ResendPendingCameraDuringPause() {
    if (run_state_ != RunState::kPausing || pending_camera_id_ == 0 ||
        camera_acknowledged_ || simulate_camera_feedback_ ||
        !enable_command_transport_) return;
    vision::msg::CameraCommand message;
    message.command_id = pending_camera_id_;
    message.request = static_cast<std::uint8_t>(pending_camera_request_);
    camera_cmd_pub_->publish(message);
  }

  void OnImu(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
    ++imu_count_;

    std::lock_guard<std::mutex> lock(imu_mutex_);

    double r = static_cast<double>(msg->vector.x);
    double p = static_cast<double>(msg->vector.y);

    r = std::max(-imu_abs_limit_rad_, std::min(imu_abs_limit_rad_, r));
    p = std::max(-imu_abs_limit_rad_, std::min(imu_abs_limit_rad_, p));

    last_roll_rad_ = r;
    last_pitch_rad_ = p;

    imu_ready_ = true;

    RCLCPP_DEBUG(get_logger(), "[IMU] recv roll=%.5f pitch=%.5f (clamped) | imu_ready=%d", last_roll_rad_,
                 last_pitch_rad_, imu_ready_ ? 1 : 0);
  }

  void DrawYoloDetections(cv::Mat& image,const std::vector<Detection>& detections) {
    vision::DrawRawDebugDetections(image,detections,tuning_config_);
  }




  // PERF: 1초에 1번만 문자열 갱신 (로그로는 절대 안 찍음)
  void UpdatePerfOverlayOncePerSecond(double infer_time_sec, double loop_time_sec) {
    const rclcpp::Time now = this->get_clock()->now();
    const rclcpp::Duration elapsed = now - last_report_time_;

    perf_frame_count_ += 1;
    perf_infer_time_sec_ += infer_time_sec;
    perf_loop_time_sec_ += loop_time_sec;

    if (elapsed.seconds() >= 1.0) {
      const int denom = std::max(1, perf_frame_count_);
      const double avg_infer_ms = (perf_infer_time_sec_ / static_cast<double>(denom)) * 1000.0;
      const double avg_loop_ms = (perf_loop_time_sec_ / static_cast<double>(denom)) * 1000.0;
      const double fps = static_cast<double>(perf_frame_count_) / elapsed.seconds();

      last_report_time_ = now;
      perf_frame_count_ = 0;
      perf_infer_time_sec_ = 0.0;
      perf_loop_time_sec_ = 0.0;

      char buf[128];
      std::snprintf(buf, sizeof(buf), "INFER: %.2fms | LOOP: %.2fms | FPS: %.2f", avg_infer_ms, avg_loop_ms, fps);
      perf_text_ = buf;
    }
  }

  void OnImage(const sensor_msgs::msg::Image::SharedPtr msg) {
    ++frames_count_;
    last_img_stamp_sec_ =
        static_cast<double>(msg->header.stamp.sec) + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;

    if (inference_hz_ > 0.0) {
      std::lock_guard<std::mutex> lock(image_mutex_);
      latest_image_ = msg;
      return;
    }

    ProcessImage(msg);
  }

  void OnDepth(const sensor_msgs::msg::Image::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(image_mutex_);
    latest_depth_ = msg;
  }

  void OnCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (msg->k[0] <= 0.0 || msg->k[4] <= 0.0)
      return;
    camera_intrinsics_ = {msg->k[0], msg->k[4], msg->k[2], msg->k[5]};
  }

  void ProcessLatestImage() {
    sensor_msgs::msg::Image::SharedPtr msg;
    {
      std::lock_guard<std::mutex> lock(image_mutex_);
      msg = latest_image_;
      latest_image_.reset();
    }

    if (!msg) {
      return;
    }

    ProcessImage(msg);
  }

  void ProcessImage(const sensor_msgs::msg::Image::SharedPtr &msg) {
    const auto t0 = std::chrono::steady_clock::now();
    // A queued image from before START/RESUME must never qualify as fresh
    // stationary evidence. Other states continue to accept processed images.
    if (run_state_ == RunState::kObserving &&
        StampSeconds(msg->header) <= observation_start_sec_) return;
    // Transport may deliver frames taken during a P2P/camera movement only
    // after DONE is received. Those frames are not post-DONE observations.
    const double completion_sec =
        std::max(last_action_done_event_sec_, last_camera_done_event_sec_);
    if (completion_sec > 0.0) {
      const double capture_sec = StampSeconds(msg->header);
      if (!std::isfinite(capture_sec) || capture_sec <= 0.0 ||
          capture_sec <= completion_sec) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "[OBS] Dropping image predating a motion/camera DONE or lacking a valid ROS timestamp");
        return;
      }
    }

    RCLCPP_DEBUG(get_logger(), "[STEP %zu] Process image", frames_count_);

    // 1) ROS Image -> cv::Mat(BGR8)
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "[IMG] cv_bridge 예외: %s", e.what());
      return;
    }

    const cv::Mat &bgr = cv_ptr->image;
    if (bgr.empty()) {
      RCLCPP_WARN(get_logger(), "[IMG] 빈 이미지가 들어왔습니다.");
      return;
    }

    // 2) IMU 최신값 스냅샷
    double roll = 0.0;
    double pitch = 0.0;
    bool imu_ready_snapshot = false;
    if (use_imu_rectification_ && !assume_zero_imu_) {
      std::lock_guard<std::mutex> lock(imu_mutex_);
      roll = last_roll_rad_;
      pitch = last_pitch_rad_;
      imu_ready_snapshot = imu_ready_;
    } else if (use_imu_rectification_ && assume_zero_imu_) {
      imu_ready_snapshot = true;
    }
    RCLCPP_DEBUG(get_logger(), "[IMU] snapshot roll=%.5f pitch=%.5f | imu_ready=%d", roll, pitch,
                 imu_ready_snapshot ? 1 : 0);

    // 3) YOLO 추론
    std::vector<Detection> dets;

    const auto t1 = std::chrono::steady_clock::now();
    const bool ok = yolo_->Infer(bgr, dets);
    const auto t2 = std::chrono::steady_clock::now();

    const auto dt_yolo_us = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();

    if (!ok) {
      RCLCPP_ERROR(get_logger(), "[YOLO] Infer failed -> skip remaining stages");
      return;
    }
    // YOLO first: subscribe to depth continuously but convert/read it only
    // when an actual BACKBOARD candidate exists. Keep RGB/Depth timestamp
    // and image-size gates identical in AUTO and MANUAL executables.
    cv::Mat aligned_depth;
    const bool need_board_depth = std::any_of(
        dets.begin(), dets.end(), [this](const Detection &d) {
          return d.class_id == backboard_class_id_;
        });
    if (need_board_depth && run_state_ != RunState::kWaitStart &&
        run_state_ != RunState::kPaused) {
      sensor_msgs::msg::Image::SharedPtr depth_msg;
      {
        std::lock_guard<std::mutex> lock(image_mutex_);
        depth_msg = latest_depth_;
      }
      if (depth_msg &&
          std::abs(StampSeconds(msg->header) - StampSeconds(depth_msg->header)) <=
              std::max(0.0, max_depth_age_sec_)) {
        try {
          const auto depth_cv_ptr = cv_bridge::toCvShare(depth_msg);
          if (depth_cv_ptr->image.cols == bgr.cols &&
              depth_cv_ptr->image.rows == bgr.rows)
            aligned_depth = depth_cv_ptr->image;
        } catch (const cv_bridge::Exception &e) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                              "[DEPTH] cv_bridge exception: %s", e.what());
        }
      }
    }

    if (run_state_ == RunState::kWaitStart ||
        run_state_ == RunState::kPaused ||
        run_state_ == RunState::kResettingCamera) {
      // Disarmed/paused/reset-camera: freeze MissionController. RESET may
      // retransmit only its already assigned FORWARD CameraCommand ID.
      if (show_debug_view_ || show_yolo_debug_view_) {
        try {
          if (show_yolo_debug_view_) {
            cv::Mat raw_view = bgr.clone();
            DrawYoloDetections(raw_view, dets);
            cv::imshow(kYoloDebugWindowName, raw_view);
          }
          if (show_debug_view_) {
            const auto views=vision::RenderMissionDebug(bgr,dets,aligned_depth,tuning_config_,
              last_debug_result_.perception,last_debug_result_.mission,camera_feedback_,camera_intrinsics_,
              DebugContext(imu_ready_snapshot,roll,pitch));
          cv::imshow(kDebugWindowName,views.image);
          cv::imshow(kStateWindowName,views.panel);
          }
          cv::waitKey(1);
        } catch (const cv::Exception &error) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                               "[VIEW] disarmed debug error: %s", error.what());
        }
      }
      UpdatePerfOverlayOncePerSecond(
          static_cast<double>(dt_yolo_us) * 1e-6,
          std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
      return;
    }

    // 4) ROS/sensor 형식을 core의 raw perception 입력으로만 변환한다.
    const auto t3 = std::chrono::steady_clock::now();
    const double frame_now_sec = this->get_clock()->now().seconds();
    DrainExecutionFeedback();
    action_delivery_feedback_={};
    if (!applied_action_feedback_queue_.empty()) {
      action_delivery_feedback_=applied_action_feedback_queue_.front();
      applied_action_feedback_queue_.pop_front();
      if (action_delivery_feedback_.done) last_processed_done_id_=action_delivery_feedback_.action_id;
    }
    vision_core::PerceptionFrameInput perception_input;
    perception_input.detections =
        BuildPerceptionDetections(dets, aligned_depth);
    perception_input.intrinsics = camera_intrinsics_;
    perception_input.enable_imu_rectification = use_imu_rectification_;
    perception_input.imu_valid = imu_ready_snapshot;
    perception_input.roll_rad = roll;
    perception_input.pitch_rad = pitch;
    perception_input.image_width = bgr.cols;
    perception_input.image_height = bgr.rows;
    // Freeze mission timers during PAUSED, including camera/object
    // observation deadlines, without changing ROS camera timestamps.
    // Manual WAIT_PREVIEW and WAIT_COMMIT are human-controlled pauses:
    // exclude BOTH from mission/recovery/ACK timeouts. Actual body motion,
    // camera movement and commanded observation time still advance.
    const double manual_wait_in_progress_sec =
        manual_wait_preview_ ? std::max(0.0, frame_now_sec -
            manual_wait_preview_started_sec_) :
        manual_wait_commit_ ? std::max(0.0, frame_now_sec -
            manual_wait_commit_started_sec_) : 0.0;
    perception_input.now_sec =
        (run_state_ == RunState::kPausing ? pause_requested_sec_ : frame_now_sec)
        - paused_accumulated_sec_ - manual_wait_accumulated_sec_ -
        manual_wait_in_progress_sec;
    // While draining already-dispatched actions, freeze LINE recovery and
    // cruise decisions. Feedback still reaches the command coordinator.
    if (run_state_ == RunState::kPausing) {
      perception_input.advance_line_fsm = false;
      perception_input.allow_new_mission_entry = false;
    }
    // The manual core's observation gate starts with its FIRST processed
    // RGB frame, not at the ROS service arrival. Align the outer window to
    // that same frame or it could freeze the core at 0.93s of a 1s gate.
    if (run_state_ == RunState::kObserving &&
        manual_observation_request_pending_)
      observation_start_sec_ = frame_now_sec;
    const bool finishing_observation =
        run_state_ == RunState::kObserving &&
        frame_now_sec - observation_start_sec_ >= observation_required_sec_ &&
        (!manual_observation_session_ || mission_controller_->ManualObservationCanFinish(perception_input.now_sec));
    perception_input.allow_new_commands =
        run_state_ == RunState::kRunning || finishing_observation;
    const bool continuing_repeat=authorized_repeat_scope_!=0 &&
        mission_controller_->RepeatScope()==authorized_repeat_scope_ && run_state_==RunState::kRunning;
    perception_input.allow_new_body_actions = continuing_repeat;
    const bool stage_setup_was_pending =
        dedicated_test_ && mission_controller_->TestSetupPending();
    // Stage START may automatically run its initial WALK/FINE pose only.
    perception_input.allow_test_setup_body_actions =
        stage_setup_was_pending &&
        (run_state_ == RunState::kRunning || finishing_observation);
    if (finishing_observation && !manual_observation_session_) {
      // START uses this uniform LINE mean. Explicit manual PREVIEW
      // uses MissionController's own 1s object/LINE observation.
      // NONE (invalid average) falls into normal LINE failure observation.
      const auto averaged =
          observation_accumulator_.FinishAll(observation_cycle_id_);
      perception_input.line_decision_guide_override =
          averaged ? *averaged : vision_core::LineGuide{};
    }
    if (!applied_action_feedback_queue_.empty()) perception_input.allow_new_commands=false;
    perception_input.camera_feedback = camera_feedback_;
    perception_input.command_transport_enabled =
        enable_command_transport_ || simulate_decision_;
    perception_input.manual_observation_control = true;
    perception_input.begin_manual_observation =
        manual_observation_request_pending_;
    perception_input.delivery_feedback = action_delivery_feedback_;
    // The explicit observation trigger is cleared after the first real core step.
    // Manual LINE uses the same cached 40%-to-READY measurement that the
    // existing line_p2p_tuning_node saves; never queue B in forward_walk.
    if (cached_ready_guide_ && !manual_action_active_ &&
        outstanding_actions_.empty() &&
        mission_controller_->ActiveMission() == vision_core::MissionType::kLine)
      perception_input.line_decision_guide_override = cached_ready_guide_;
    const auto t4 = std::chrono::steady_clock::now();
    const auto dt_pts_us = std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3).count();

    // 5) PREVIEW is speculative, COMMIT is the sole body-motion admission.
    // DONE updates the real coordinator immediately, but NEVER starts its
    // next stationary observation without a new explicit PREVIEW.
    const auto t5 = std::chrono::steady_clock::now();
    const bool committed_done =
        perception_input.delivery_feedback.done &&
        manual_committed_command_.action_id != 0 &&
        perception_input.delivery_feedback.action_id ==
            manual_committed_command_.action_id;
    const bool committed_ready =
        perception_input.delivery_feedback.ready &&
        manual_committed_command_.action_id != 0 &&
        perception_input.delivery_feedback.action_id ==
            manual_committed_command_.action_id &&
        IsReadyWalk(manual_committed_command_.action);
    // If camera DONE overlaps a still-running 11/12/13, keep its READY
    // samples intact and process the final camera phase after body DONE.
    const bool camera_done_edge = manual_camera_done_pending_ &&
        !manual_action_active_ && outstanding_actions_.empty();
    const bool completed_with_ready_window =
        committed_done && IsReadyWalk(manual_committed_command_.action) &&
        ready_window_input_.has_value();
    if (completed_with_ready_window && !continuing_repeat) {
      // Apply the already acquired moving-window average at the DONE edge.
      // New detections seen while waiting for the user are not substituted.
      auto average = *ready_window_input_;
      average.now_sec = perception_input.now_sec;
      average.delivery_feedback = perception_input.delivery_feedback;
      average.camera_feedback = perception_input.camera_feedback;
      average.allow_new_body_actions = false;
      average.allow_new_commands = perception_input.allow_new_commands;
      average.manual_observation_control = true;
      average.begin_manual_observation = false;
      perception_input = std::move(average);
    }

    const auto capture_preview = [&](const vision_core::PerceptionFrameInput &input,
                                     const cv::Mat &image) {
      const auto proposed = mission_controller_->PreviewPerception(input);
      const auto &candidate = proposed.mission.command;
      if (candidate.command_type != vision_core::CommandType::kAction ||
          candidate.action_id == 0 ||
          candidate.camera_request != vision_core::CameraRequest::kNone)
        return false;
      preview_input_ = input;
      preview_input_.delivery_feedback = {};
      preview_input_.begin_manual_observation = false;
      preview_result_ = proposed;
      preview_command_ = candidate;
      preview_image_ = image.clone();
      preview_ready_ = true;
      LeaveWaitPreview(frame_now_sec);
      EnterWaitCommit(frame_now_sec);
      RCLCPP_INFO(get_logger(),
          "[TUNING] WAIT_COMMIT mission=%u action=%u id=%llu",
          static_cast<unsigned>(candidate.mission),
          static_cast<unsigned>(candidate.action),
          static_cast<unsigned long long>(candidate.action_id));
      return true;
    };

    const int mission_state_before = mission_controller_->MissionStateKey();
    vision_core::PerceptionMissionFrameResult perception_result;
    const bool was_stationary_observing = mission_controller_->NeedsStationaryObservation();
    bool stepped_real_core = false;
    if (preview_ready_) {
      perception_result = preview_result_;
    } else if (manual_wait_preview_ &&
               !stage_setup_was_pending &&
               !mission_controller_->NeedsStationaryObservation() &&
               !committed_done && !camera_done_edge &&
               !manual_action_active_ && pending_camera_id_ == 0 &&
               run_state_ == RunState::kRunning) {
      // Keep YOLO/debug alive while freezing the *real* FSM, tracking
      // history, command coordinator and mission deadlines.
      auto view_only = perception_input;
      view_only.allow_new_commands = false;
      view_only.allow_new_body_actions = false;
      perception_result = mission_controller_->PreviewPerception(view_only);
    } else {
      const bool can_capture_fresh =
          finishing_observation &&
          !manual_action_active_ && outstanding_actions_.empty() &&
          pending_camera_id_ == 0 &&
          perception_input.delivery_feedback.action_id == 0 &&
           !stage_setup_was_pending;
      if (can_capture_fresh &&
          capture_preview(perception_input, bgr)) {
        // Real controller remains poised before its final observation frame;
        // COMMIT will replay precisely this input on a copied controller.
        perception_result = preview_result_;
      } else {
        perception_result = mission_controller_->StepPerception(perception_input);
        stepped_real_core = true;
        if (manual_observation_request_pending_)
          manual_observation_request_pending_ = false;
        if (perception_result.mission.line_window_stats)
          cached_ready_guide_ = mission_controller_->LastReadyLineGuide();

        if (manual_action_active_ &&
            IsReadyWalk(manual_committed_command_.action))
          AddReadyMotionSample(perception_input, perception_result.perception);
        if (committed_ready) {
          ready_window_input_ = BuildReadyWindowInput(
              mission_controller_->ActiveMission(), perception_input.now_sec);
          // An otherwise valid LINE guide with only one/two READY samples
          // is not strong enough to bypass explicit fresh PREVIEW.
          if (mission_controller_->ActiveMission() ==
                  vision_core::MissionType::kLine &&
              (!perception_result.mission.line_window_stats ||
               perception_result.mission.line_window_stats->used_frames <
                   static_cast<std::size_t>(std::max(
                       1, tuning_config_.line_p2p.failure_min_valid_samples))))
            ready_window_input_.reset();
          if (ready_window_input_) {
            ready_window_image_ = bgr.clone();
            RCLCPP_INFO(get_logger(),
                "[TUNING] READY: moving-window evidence stored for mission=%u",
                static_cast<unsigned>(manual_committed_command_.mission));
          } else {
            RCLCPP_WARN(get_logger(),
                "[TUNING] READY: insufficient moving-window evidence; use PREVIEW after DONE");
          }
        }

        const bool repeat_finished = authorized_repeat_scope_!=0 &&
            perception_result.mission.repeat_scope!=authorized_repeat_scope_;
        if (repeat_finished)
          authorized_repeat_scope_=0;
        if ((committed_done || camera_done_edge || repeat_finished) && authorized_repeat_scope_==0) {
          manual_camera_done_pending_ = false;
          const bool fixed_successor =
              committed_done &&
              IsFixedSuccessor(manual_committed_command_.mission,
                               manual_committed_command_.action);
          const bool ready_successor = completed_with_ready_window;
          const bool may_auto_preview = fixed_successor || ready_successor;
          bool captured = false;
          if (may_auto_preview &&
              pending_camera_id_ == 0 &&
              perception_result.mission.command.camera_request ==
                  vision_core::CameraRequest::kNone) {
            auto next_input = perception_input;
            next_input.delivery_feedback = {};
            next_input.begin_manual_observation = false;
            next_input.allow_new_commands = true;
            next_input.allow_new_body_actions = false;
            captured = capture_preview(next_input,
                ready_successor && !ready_window_image_.empty()
                    ? ready_window_image_ : bgr);
          }
          if (!captured &&
              !mission_controller_->TestSetupPending() &&
              pending_camera_id_ == 0 &&
              perception_result.mission.command.camera_request ==
                  vision_core::CameraRequest::kNone)
            EnterWaitPreview(frame_now_sec);
          ready_window_input_.reset();
          ready_motion_samples_.clear();
          ready_window_image_.release();
        }
      }
    }
    if (run_state_ == RunState::kObserving) {
      if (finishing_observation) {
        run_state_ = RunState::kRunning;
        manual_observation_session_ = false;
        if (!preview_ready_ && !stage_setup_was_pending &&
            pending_camera_id_ == 0 &&
            perception_result.mission.command.camera_request ==
                vision_core::CameraRequest::kNone)
          EnterWaitPreview(frame_now_sec);
        RCLCPP_INFO(get_logger(),
                    "[CONTROL] fresh 1s observation finished: %s",
                    preview_ready_ ? "WAIT_COMMIT" : "WAIT_PREVIEW or camera");
      } else {
        observation_accumulator_.Add(
            observation_cycle_id_, frame_now_sec,
            perception_result.mission.line_features.guide);
      }
    }
    // An internal FSM-only transition must not consume a second PREVIEW click.
    // Do not bypass COMMIT for any real action: this only starts a new
    // stationary observation if a preceding window/camera completion changed
    // the FSM without issuing a body or camera command.
    if (!stage_setup_was_pending && (finishing_observation || camera_done_edge) &&
        mission_controller_->MissionStateKey() != mission_state_before &&
        !preview_ready_ && !manual_action_active_ &&
        outstanding_actions_.empty() && pending_camera_id_ == 0 &&
        perception_result.mission.command.action_id == 0 &&
        perception_result.mission.command.camera_request ==
            vision_core::CameraRequest::kNone &&
        !mission_controller_->IsTerminalHold() &&
        run_state_ == RunState::kRunning) {
      LeaveWaitPreview(frame_now_sec);
      manual_observation_request_pending_ = true;
      manual_observation_session_ = true;
      observation_required_sec_ = 1.0;
      BeginFreshObservation(frame_now_sec);
    }
    if (mission_controller_->NeedsStationaryObservation()) {
      // Stationary recovery is automatic observation, not a human wait.
      LeaveWaitPreview(frame_now_sec);
    } else if (was_stationary_observing && !preview_ready_ && !manual_action_active_ &&
               outstanding_actions_.empty() && pending_camera_id_ == 0 &&
               perception_result.mission.command.camera_request == vision_core::CameraRequest::kNone &&
               !perception_result.mission.trial_complete) {
      EnterWaitPreview(frame_now_sec);
    }
    (void)stepped_real_core;
    const auto t6 = std::chrono::steady_clock::now();
    const auto dt_rec_us = std::chrono::duration_cast<std::chrono::microseconds>(t6 - t5).count();

    const auto &prepared = perception_result.perception;
    const auto &object_targets = prepared.targets;
    const auto &ball_target = object_targets.ball;
    const auto &goal_target = object_targets.goal;
    const auto &backboard_target = object_targets.backboard;
    const auto &hurdle_target = object_targets.hurdle;
    const auto &goal_pose = prepared.goal_pose;

    // 6) core가 반환한 최종 명령을 ROS 토픽 형식으로 전달한다.
    const auto t7 = std::chrono::steady_clock::now();
    const auto &mission_result = perception_result.mission;
    const auto &control_command = mission_result.command;
    if (mission_result.post_motion_observing) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
          "[OBS] HOLD after DONE: mission=%u elapsed=%.2fs frames=%zu valid=%d/%zu (object window has 1s deadline)",
          static_cast<unsigned>(mission_result.active_mission),
          mission_result.post_motion_elapsed_sec,
          mission_result.post_motion_frames,
          mission_result.post_motion_valid,
          mission_result.post_motion_window_frames);
    }
    if (mission_result.line_window_stats) {
      const auto &stats = *mission_result.line_window_stats;
      RCLCPP_INFO(
          get_logger(),
          "[LINE OBS] id=%llu frames_total=%zu valid_line=%zu "
          "window_frames=%zu used_valid=%zu est_motion=%.3fs "
          "window=[%.3f,%.3f] remaining=%.3fs next_action=%u",
          static_cast<unsigned long long>(stats.action_id),
          stats.total_frames, stats.valid_frames,
          stats.window_frames, stats.used_frames,
          stats.estimated_motion_sec, stats.window_start_sec,
          stats.window_end_sec, stats.remaining_sec,
          static_cast<unsigned int>(control_command.action));
    }
    const auto &ball_cmd = mission_result.ball;
    const auto &hurdle_cmd = mission_result.hurdle;
    const auto &goal_cmd = mission_result.goal;
    const auto &feats = mission_result.line_features;
    const bool selected_ball =
        control_command.mission == vision_core::MissionType::kBall;
    const bool selected_hurdle =
        control_command.mission == vision_core::MissionType::kHurdle;
    const bool selected_goal =
        control_command.mission == vision_core::MissionType::kGoal;
    const char *selected_mode =
        selected_ball
            ? vision_core::BallController::ModeName(ball_cmd.mode)
            : (selected_hurdle
                   ? vision_core::HurdleController::ModeName(hurdle_cmd.mode)
                   : (selected_goal
                          ? vision_core::GoalController::ModeName(goal_cmd.mode)
                          : "LINE_P2P"));
    if (authorized_repeat_scope_!=0 &&
        control_command.command_type==vision_core::CommandType::kAction &&
        control_command.action_id!=manual_action_id_) {
      manual_action_id_=control_command.action_id;
      manual_committed_command_=control_command;
      manual_action_active_=true;
      ready_motion_samples_.clear(); ready_window_input_.reset();
    }
    if (mission_result.trial_complete && !manual_action_active_ && outstanding_actions_.empty()) {
      RCLCPP_INFO(get_logger(),"[TEST] stage=%d completed has_ball=%d pickup_attempts=%d pickup_failed=%d turns=%d",
          tuning_config_.test_stage,mission_result.has_ball,mission_result.ball.pickup_attempt_count,
          mission_result.ball.pickup_failed,mission_result.recovery_turns_completed);
      authorized_repeat_scope_=0; preview_ready_=false; trial_finished_=true;
      LeaveWaitPreview(frame_now_sec); LeaveWaitCommit(frame_now_sec);
      run_state_=RunState::kWaitStart;
      if ((tuning_config_.test_stage==0 || tuning_config_.test_stage==3)) {
        if (algorithm_mode_=="goal" && mission_result.goal_mission_index<1000000)
          set_parameter(rclcpp::Parameter("test_goal_mission_index",mission_result.goal_mission_index+1));
        if (dedicated_test_ && algorithm_mode_=="ball" && mission_result.ball_mission_index<1000000)
          set_parameter(rclcpp::Parameter("test_ball_mission_index",mission_result.ball_mission_index+1));
      }
    }
    const bool stage_pose_command = dedicated_test_ &&
        control_command.mission_phase == 903 &&
        (control_command.action == vision_core::MissionAction::kWalkMode ||
         control_command.action == vision_core::MissionAction::kDefaultPoseMode);
    if (!preview_ready_ && (manual_action_active_ || stage_pose_command))
      PublishActionCommand(control_command);
    if (!preview_ready_)
      PublishCameraCommand(control_command.camera_request);
    if (stage_setup_was_pending && !mission_controller_->TestSetupPending() &&
        !manual_action_active_ && outstanding_actions_.empty() &&
        pending_camera_id_ == 0 && run_state_ == RunState::kRunning) {
      // Stage prep is complete. Clear setup-era evidence and begin a fresh
      // one-second window automatically; mission actions still need COMMIT.
      LeaveWaitPreview(frame_now_sec);
      LeaveWaitCommit(frame_now_sec);
      manual_observation_request_pending_ = true;
      manual_observation_session_ = true;
      observation_required_sec_ = 1.0;
      BeginFreshObservation(frame_now_sec);
    }
    ResendPendingCameraDuringPause();
    action_delivery_feedback_ = {};
    // PAUSED is entered only after every sent command has a matching DONE
    // and the ROS feedback queue has been drained through MissionController.
    CompletePauseAfterFeedback();

    const auto t8 = std::chrono::steady_clock::now();
    const auto dt_feat_us = std::chrono::duration_cast<std::chrono::microseconds>(t8 - t7).count();

    last_debug_result_=perception_result;
    // ---- 시각화 ----
    if (ShouldDrawDebugView()) {
      try {
        if (show_yolo_debug_view_) {
          cv::Mat yolo_vis = bgr.clone();
          DrawYoloDetections(yolo_vis, dets);
          cv::imshow(kYoloDebugWindowName, yolo_vis);
        }

        if (show_debug_view_) {
          const auto views=vision::RenderMissionDebug(bgr,dets,aligned_depth,tuning_config_,
              perception_result.perception,perception_result.mission,camera_feedback_,camera_intrinsics_,
              DebugContext(imu_ready_snapshot,roll,pitch));
          cv::imshow(kDebugWindowName,views.image);
          cv::imshow(kStateWindowName,views.panel);
        }
        cv::waitKey(1);
      } catch (const cv::Exception &e) {
        show_debug_view_ = false;
        show_yolo_debug_view_ = false;
        RCLCPP_WARN(get_logger(), "[VIEW] debug window disabled: %s", e.what());
      }
    }

    const auto t_end = std::chrono::steady_clock::now();
    const auto dt_loop_us = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t0).count();

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "[TIME] yolo=%.2fms pts=%.2fms rect=%.2fms "
                         "feat=%.2fms infer=%.2fms loop=%.2fms n=%.0f recovery=%d "
                         "ball=%d goal=%d backboard=%d hurdle=%d mode=%s "
                         "pose=%d x=%+.3f z=%.3f yaw=%+.1fdeg action=%u",
                         static_cast<double>(dt_yolo_us) / 1000.0, static_cast<double>(dt_pts_us) / 1000.0,
                         static_cast<double>(dt_rec_us) / 1000.0, static_cast<double>(dt_feat_us) / 1000.0,
                         static_cast<double>(dt_yolo_us) / 1000.0, static_cast<double>(dt_loop_us) / 1000.0,
                         feats.n_visible, mission_result.line_in_recovery ? 1 : 0,
                         ball_target ? 1 : 0, goal_target ? 1 : 0,
                         backboard_target ? 1 : 0, hurdle_target ? 1 : 0, selected_mode, goal_pose.valid ? 1 : 0,
                         goal_pose.x_m, goal_pose.z_m,
                         goal_pose.yaw_rad * 180.0 / M_PI,
                         static_cast<unsigned int>(control_command.action));

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "[ACTION] %s -> %s | id=%lu action=%u yaw=%d | line_n=%.0f ball_stable=%d request_cam_down=%d",
                         selected_goal ? "GOAL" : (selected_hurdle ? "HURDLE" : (selected_ball ? "BALL" : "LINE")),
                         action_cmd_topic_.c_str(), control_command.action_id,
                         static_cast<unsigned int>(control_command.action),
                         static_cast<int>(control_command.target_yaw_deg),
                         feats.n_visible, ball_cmd.tracked.stable ? 1 : 0,
                         ball_cmd.camera_request == vision_core::CameraRequest::kDown ? 1 : 0);

    // ===== PERF overlay 업데이트(1초에 1번만 문자열 갱신) =====
    UpdatePerfOverlayOncePerSecond(static_cast<double>(dt_yolo_us) * 1e-6, static_cast<double>(dt_loop_us) * 1e-6);
  }

private:
  // ROS Sub/Pub
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr imu_sub_;
  rclcpp::Publisher<vision::msg::ActionCommand>::SharedPtr action_cmd_pub_;
  rclcpp::Publisher<vision::msg::CameraCommand>::SharedPtr camera_cmd_pub_;
  rclcpp::Subscription<vision::msg::CommandStatus>::SharedPtr action_status_sub_;
  rclcpp::Subscription<vision::msg::CommandStatus>::SharedPtr camera_status_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr preview_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr commit_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  bool preview_ready_{false};
  bool dedicated_test_{false}, trial_finished_{false};
  bool manual_wait_preview_{false};
  bool manual_wait_commit_{false};
  bool manual_observation_request_pending_{false};
  bool manual_observation_session_{false};
  bool manual_camera_done_pending_{false};
  double manual_wait_preview_started_sec_{0.0};
  double manual_wait_commit_started_sec_{0.0};
  // Completed manual waits (PREVIEW + COMMIT), both excluded from FSM time.
  double manual_wait_accumulated_sec_{0.0};
  double observation_required_sec_{1.0};
  vision_core::ControlCommand manual_committed_command_;
  struct ReadyMotionSample {
    double now_sec{};
    vision_core::PerceptionFrameInput input;
    vision_core::PreparedPerceptionFrame prepared;
  };
  std::deque<ReadyMotionSample> ready_motion_samples_;
  std::optional<vision_core::PerceptionFrameInput> ready_window_input_;
  cv::Mat ready_window_image_;
  cv::Mat preview_image_;  // frozen RGB decision evidence for WAIT_COMMIT
  int authorized_repeat_scope_{0};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_repeat_service_;
  bool manual_action_active_{false};
  std::uint64_t manual_action_id_{0};
  std::optional<vision_core::LineGuide> cached_ready_guide_;
  vision_core::PerceptionFrameInput preview_input_;
  vision_core::PerceptionMissionFrameResult preview_result_;
  vision_core::ControlCommand preview_command_;
  RunState run_state_{RunState::kWaitStart};
  std::unordered_set<std::uint64_t> outstanding_actions_;
  std::unordered_set<std::uint64_t> outstanding_cameras_;
  std::uint64_t observation_cycle_id_{0};
  vision_core::LineGuideAccumulator observation_accumulator_;
  double observation_start_sec_{0.0};
  double pause_requested_sec_{0.0};
  double paused_accumulated_sec_{0.0};
  rclcpp::TimerBase::SharedPtr hb_timer_;
  rclcpp::TimerBase::SharedPtr inference_timer_;
  rclcpp::TimerBase::SharedPtr reset_timer_;

  // 모듈
  std::unique_ptr<YoloTrtEngine> yolo_;
  std::unique_ptr<vision_core::MissionController> mission_controller_;
  vision_core::Intrinsics camera_intrinsics_{};
  vision_core::PerceptionMissionFrameResult last_debug_result_;
  vision_core::MissionControllerConfig tuning_config_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr tuning_callback_;

  // IMU 최신 상태(latest)
  std::mutex imu_mutex_;
  bool imu_ready_{false};
  bool use_imu_rectification_{false};
  bool assume_zero_imu_{false};
  double last_roll_rad_{0.0};
  double last_pitch_rad_{0.0};
  double imu_abs_limit_rad_{Deg2Rad(45.0)};

  std::mutex image_mutex_;
  sensor_msgs::msg::Image::SharedPtr latest_image_;
  sensor_msgs::msg::Image::SharedPtr latest_depth_;
  bool enable_ball_controller_{false};
  bool enable_hurdle_controller_{false};
  bool enable_goal_controller_{false};
  bool simulate_camera_feedback_{true};
  bool simulate_decision_{false};
  double simulate_action_duration_sec_{0.50};
  double simulate_step_forward_five_duration_sec_{3.0};
  double simulate_curve_step_duration_sec_{2.5};
  double simulate_step_forward_one_duration_sec_{2.0};
  double simulate_turn_duration_sec_{1.0};
  double simulate_ready_lead_sec_{0.25};
  bool enable_command_transport_{true};
  vision_core::CommandDeliveryFeedback action_delivery_feedback_{};
  std::mutex action_feedback_mutex_;
  std::deque<vision_core::CommandDeliveryFeedback> action_feedback_queue_;
  std::deque<vision_core::CommandDeliveryFeedback> applied_action_feedback_queue_;
  vision_core::ControlCommand last_transport_command_;
  std::uint64_t last_received_done_id_{0}, last_processed_done_id_{0};
  SimulatedActionState simulated_current_action_;
  SimulatedActionState simulated_queued_action_;
  std::deque<SimulatedActionState> simulated_completed_actions_;
  std::deque<vision_core::CommandDeliveryFeedback>
      simulated_action_feedback_queue_;
  vision_core::CameraFeedback camera_feedback_{};
  // ROS-clock timestamps of the latest completion events. Image header
  // stamps are compared before YOLO/FSM processing, so stale frames cannot
  // satisfy the post-DONE observation window.
  double last_action_done_event_sec_{0.0};
  double last_camera_done_event_sec_{0.0};
  std::uint64_t next_camera_id_{1};
  std::uint64_t pending_camera_id_{0};
  vision_core::CameraRequest pending_camera_request_{
      vision_core::CameraRequest::kNone};
  bool camera_acknowledged_{false};
  int backboard_depth_strip_width_px_{};
  double max_depth_age_sec_{0.20};
  std::string algorithm_mode_{"all"};

  // 디버그 카운터
  size_t frames_count_{0};
  size_t imu_count_{0};
  size_t action_pub_count_{0};
  double last_img_stamp_sec_{0.0};

  // ---- PERF 오버레이(1초 갱신) ----
  rclcpp::Time last_report_time_{0, 0, RCL_ROS_TIME};
  int perf_frame_count_{0};
  double perf_infer_time_sec_{0.0};
  double perf_loop_time_sec_{0.0};
  std::string perf_text_;

  // ---- 시각화/필터 설정(멤버) ----
  int line_class_id_{0};
  int ball_class_id_{1};
  int goal_class_id_{2};
  int backboard_class_id_{3};
  int hurdle_class_id_{4};
  float conf_thres_{0.60f};
  float ball_conf_thres_{0.60f};
  float goal_conf_thres_{0.60f};
  float backboard_conf_thres_{0.60f};
  float hurdle_conf_thres_{0.60f};
  bool show_debug_view_{false};
  bool show_yolo_debug_view_{false};
  double inference_hz_{5.0};

  std::string image_topic_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string imu_topic_;
  std::string action_cmd_topic_;
  std::string action_status_topic_;
  std::string camera_cmd_topic_;
  std::string camera_status_topic_;
  std::string engine_path_;
};

} // namespace vision

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<vision::PerceptionTuningNode>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("vision"), "치명적 오류: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
