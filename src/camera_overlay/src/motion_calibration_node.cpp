#include "camera_overlay/debug_view.hpp"
// camera_overlay/src/motion_calibration_node.cpp
// ROS 2 Humble, OpenCV HighGUI clickable menu; NO keyboard interaction.
// Standalone measurement: do not run line_perception_node simultaneously.
// Visual thresholds are hardcoded from current JANDI settings.
// No YAML is loaded at runtime.

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision/msg/action_command.hpp>
#include <vision/msg/camera_command.hpp>
#include <vision/msg/command_status.hpp>
#include <vision/backboard_depth.hpp>
#include <vision/yolo_trt_engine.hpp>
#include <vision_core/goal_controller.hpp>
#include <vision_core/line_feature_extractor.hpp>
#include <vision_core/pickup_polygon.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>
#include <deque>
#include <sys/utsname.h>
#include <unistd.h>

namespace {

using Image = sensor_msgs::msg::Image;

using Policy = message_filters::sync_policies::ApproximateTime<Image, Image>;

using Synchronizer = message_filters::Synchronizer<Policy>;

using Clock = std::chrono::steady_clock;

constexpr double kPi = 3.14159265358979323846;

constexpr const char *kWindow = "JANDI Motion Calibration - Mouse Buttons";

constexpr int kCanvasW = 1040;
constexpr int kCanvasH = 670;

const cv::Rect kVideo(20, 98, 640, 480);

const cv::Rect kMenuBall(68, 200, 276, 96);
const cv::Rect kMenuHurdle(380, 200, 276, 96);
const cv::Rect kMenuGoal(692, 200, 276, 96);

const cv::Rect kMenuOK(380, 370, 276, 62);
const cv::Rect kMenuQuit(380, 457, 276, 56);

const cv::Rect kBefore(694, 138, 320, 60);
const cv::Rect kAfter(694, 211, 320, 60);
const cv::Rect kClear(694, 284, 320, 60);
const cv::Rect kSetup(694, 367, 320, 50);
const cv::Rect kBack(694, 432, 320, 50);
const cv::Rect kQuit(694, 497, 320, 50);

struct Reading {
  double a{};
  double b{};
};

struct Frame {
  cv::Mat rgb;
  cv::Mat depth;

  std::optional<vision_core::Intrinsics> K;
};

enum class Mode { kNone, kBall, kHurdle, kGoal };

enum class Stage { kMenu, kPose, kCamera, kReady };

const char *ModeName(Mode m) {
  switch (m) {
  case Mode::kBall:
    return "BALL";

  case Mode::kHurdle:
    return "HURDLE";

  case Mode::kGoal:
    return "GOAL";

  default:
    return "NONE";
  }
}

std::string EnginePath() {
  // Hardcoded paths matching the current PC / Jetson runtime.

  struct utsname u {};

  if (uname(&u) == 0 && std::string(u.machine) == "aarch64") {
    return "/home/rnd/jandi/src/dataset/best_1029.engine";
  }

  return "/home/noh/dataset/runs/yolo26n_goal_1029/weights/best.engine";
}

void PutText(cv::Mat &img, const std::string &s, cv::Point p, double scale = .55, cv::Scalar color = {235, 235, 235}) {
  cv::putText(img, s, p, cv::FONT_HERSHEY_SIMPLEX, scale, {5, 5, 5}, 3, cv::LINE_AA);

  cv::putText(img, s, p, cv::FONT_HERSHEY_SIMPLEX, scale, color, 1, cv::LINE_AA);
}

void Button(cv::Mat &img, const cv::Rect &r, const std::string &label, bool enabled = true, bool chosen = false) {
  const cv::Scalar fill = !enabled ? cv::Scalar(52, 52, 52) : chosen ? cv::Scalar(58, 111, 50) : cv::Scalar(75, 70, 63);

  cv::rectangle(img, r, fill, cv::FILLED, cv::LINE_AA);

  cv::rectangle(img, r, chosen ? cv::Scalar(100, 240, 120) : cv::Scalar(140, 140, 140), 2, cv::LINE_AA);

  int baseline = 0;

  const double font_scale = .67;

  const auto size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, font_scale, 2, &baseline);

  const cv::Point text_pos{r.x + (r.width - size.width) / 2, r.y + (r.height + size.height) / 2};

  cv::putText(img, label, text_pos, cv::FONT_HERSHEY_SIMPLEX, font_scale,
              enabled ? cv::Scalar(250, 250, 250) : cv::Scalar(130, 130, 130), 2, cv::LINE_AA);
}

std::string Format(const std::string &label, double a, double b) {
  char buf[140];

  std::snprintf(buf, sizeof(buf), "%s: %+.4f, %+.4f", label.c_str(), a, b);

  return buf;
}

} // namespace

class MotionCalibrationNode final : public rclcpp::Node {
public:
  MotionCalibrationNode() : Node("motion_calibration_node") {
    // Hardcoded ROS topics.

    const std::string color_topic = "/camera/camera/color/image_raw";

    const std::string depth_topic = "/camera/camera/aligned_depth_to_color/image_raw";

    const std::string info_topic = "/camera/camera/color/camera_info";

    action_pub_ = create_publisher<vision::msg::ActionCommand>("/jandi_vision/action_cmd", 10);

    camera_pub_ = create_publisher<vision::msg::CameraCommand>("/jandi_vision/camera_cmd", 10);

    action_status_sub_ = create_subscription<vision::msg::CommandStatus>(
        "/jandi_vision/action_status", 10,
        [this](vision::msg::CommandStatus::ConstSharedPtr msg) { OnActionStatus(*msg); });

    camera_status_sub_ = create_subscription<vision::msg::CommandStatus>(
        "/jandi_vision/camera_status", 10,
        [this](vision::msg::CommandStatus::ConstSharedPtr msg) { OnCameraStatus(*msg); });

    // RGB-only path for BALL / HURDLE.

    rgb_sub_ = create_subscription<Image>(color_topic, rclcpp::SensorDataQoS(), [this](Image::ConstSharedPtr msg) {
      if (mode_ != Mode::kGoal && stage_ != Stage::kMenu) {
        Receive(msg, {});
      }
    });

    // RGB + aligned depth for GOAL.

    rgb_sync_.subscribe(this, color_topic, rmw_qos_profile_sensor_data);

    depth_sync_.subscribe(this, depth_topic, rmw_qos_profile_sensor_data);

    sync_ = std::make_shared<Synchronizer>(Policy(10), rgb_sync_, depth_sync_);

    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(.03));

    // ROS2 Humble compatible callback registration.

    sync_->registerCallback(
        std::bind(&MotionCalibrationNode::ReceiveGoal, this, std::placeholders::_1, std::placeholders::_2));

    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        info_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr info) { camera_info_ = info; });

    debug_view_ = std::make_unique<camera_overlay::DebugView>(*this);
    if (debug_view_->Local()) {
      cv::namedWindow(kWindow, cv::WINDOW_AUTOSIZE | cv::WINDOW_GUI_NORMAL);

      cv::setMouseCallback(kWindow, &MotionCalibrationNode::MouseCallback, this);

    }
    timer_ = create_wall_timer(std::chrono::milliseconds(30), [this]() { Tick(); });

    RCLCPP_INFO(get_logger(), "Click BALL / HURDLE / GOAL, then OK. No keys.");

    RCLCPP_WARN(get_logger(), "Do not run line_perception_node together with this GUI.");
  }

  ~MotionCalibrationNode() override {
    if (!debug_view_->Local()) return;
    try {
      cv::destroyWindow(kWindow);
    } catch (const cv::Exception &) {
    }
  }

private:
  using Status = vision::msg::CommandStatus;
  struct WebContext {
    std::uint64_t revision, generation;
    Stage stage;
    Mode mode, selected;
    std::optional<Reading> mean;
  };

  static void MouseCallback(int event, int x, int y, int, void *userdata) {
    if (event != cv::EVENT_LBUTTONDOWN || !userdata) {
      return;
    }

    static_cast<MotionCalibrationNode *>(userdata)->OnClick({x, y});
  }

  void OnClick(cv::Point p, const std::optional<Reading>* snapshot=nullptr) {
    ++web_revision_;
    // ============================================
    // Mode selection screen
    // ============================================

    if (stage_ == Stage::kMenu) {
      if (kMenuBall.contains(p)) {
        selected_ = Mode::kBall;
      } else if (kMenuHurdle.contains(p)) {
        selected_ = Mode::kHurdle;
      } else if (kMenuGoal.contains(p)) {
        selected_ = Mode::kGoal;
      } else if (kMenuOK.contains(p) && selected_ != Mode::kNone) {
        mode_ = selected_;

        BeginSetup();
      } else if (kMenuQuit.contains(p)) {
        rclcpp::shutdown();
      }

      return;
    }

    // ============================================
    // Measurement screen
    // ============================================

    if (kQuit.contains(p)) {
      rclcpp::shutdown();
      return;
    }

    if (stage_ != Stage::kReady) {
      return;
    }

    if (kBefore.contains(p)) {
      Capture(true,snapshot);
    } else if (kAfter.contains(p)) {
      Capture(false,snapshot);
    } else if (kClear.contains(p)) {
      before_.reset();
      after_.reset();

      ResetMeasurements();

      message_ = "BEFORE and AFTER cleared";
    } else if (kSetup.contains(p) && active_motions_.empty()) {
      BeginSetup();
    } else if (kBack.contains(p) && active_motions_.empty()) {
      stage_ = Stage::kMenu;

      mode_ = Mode::kNone;

      ResetMeasurements();

      before_.reset();
      after_.reset();

      message_ = "Select a mode, then click OK";
    }
  }

  std::uint8_t MissionCode() const {
    if (mode_ == Mode::kBall) {
      return vision::msg::ActionCommand::MISSION_BALL;
    }

    if (mode_ == Mode::kHurdle) {
      return vision::msg::ActionCommand::MISSION_HURDLE;
    }

    return vision::msg::ActionCommand::MISSION_GOAL;
  }

  std::uint8_t CameraCode() const {
    if (mode_ == Mode::kBall) {
      return vision::msg::CameraCommand::BALL_VIEW;
    }

    if (mode_ == Mode::kHurdle) {
      return vision::msg::CameraCommand::DOWN;
    }

    return vision::msg::CameraCommand::GOAL;
  }

  // ================================================
  // Setup: FINE pose -> camera
  // ================================================

  void BeginSetup() {
    if (stage_ == Stage::kPose || stage_ == Stage::kCamera) {
      return;
    }

    if (!active_motions_.empty()) {
      message_ = "Wait for RQT action DONE before SETUP";

      return;
    }

    if (!yolo_) {
      try {
        yolo_ = std::make_unique<vision::YoloTrtEngine>(EnginePath(), 640, 640, .60f);
      } catch (const std::exception &e) {
        message_ = std::string("YOLO init failed: ") + e.what();

        stage_ = Stage::kMenu;

        return;
      }
    }

    const auto ticks = std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();

    pose_id_ = static_cast<std::uint64_t>(ticks) ^ (static_cast<std::uint64_t>(getpid()) << 32);

    if (pose_id_ == 0) {
      pose_id_ = 1;
    }

    camera_id_ = pose_id_ + 1;

    pose_ack_ = false;
    camera_ack_ = false;

    last_send_ = Clock::time_point{};

    active_motions_.clear();

    before_.reset();
    after_.reset();

    ResetMeasurements();

    stage_ = Stage::kPose;

    message_ = "OK: Request FINE pose (Action 2), then camera";

    SendPose();
  }

  void SendPose() {
    if (action_pub_->get_subscription_count() == 0) {
      message_ = "No executor subscriber on /jandi_vision/action_cmd";

      return;
    }

    vision::msg::ActionCommand cmd;

    cmd.action_id = pose_id_;

    cmd.mission = MissionCode();

    cmd.action = vision::msg::ActionCommand::DEFAULT_POSE_MODE;

    cmd.target_yaw_deg = 0;

    action_pub_->publish(cmd);

    last_send_ = Clock::now();
  }

  void SendCamera() {
    if (camera_pub_->get_subscription_count() == 0) {
      message_ = "No executor subscriber on /jandi_vision/camera_cmd";

      return;
    }

    vision::msg::CameraCommand cmd;

    cmd.command_id = camera_id_;

    cmd.request = CameraCode();

    camera_pub_->publish(cmd);

    last_send_ = Clock::now();
  }

  // ================================================
  // Action and camera feedback
  // ================================================

  void OnActionStatus(const Status &s) {
    if (stage_ == Stage::kPose && s.command_id == pose_id_) {
      if (s.status == Status::ACK) {
        pose_ack_ = true;
      }

      if (s.status == Status::DONE) {
        stage_ = Stage::kCamera;

        camera_ack_ = false;

        message_ = "FINE pose DONE; requesting camera";

        SendCamera();
      }

      return;
    }

    if (stage_ != Stage::kReady || s.command_id == 0 || s.command_id == pose_id_) {
      return;
    }

    // RQT motion monitoring.
    // Ignore frames collected while any action is active.

    if (s.status == Status::ACK || s.status == Status::READY) {
      active_motions_.insert(s.command_id);

      ResetMeasurements();

      message_ = "RQT motion active - wait for real DONE";
    } else if (s.status == Status::DONE) {
      active_motions_.erase(s.command_id);

      ResetMeasurements();

      message_ = active_motions_.empty() ? "RQT motion DONE; collecting fresh frames"
                                         : "Queued RQT motion active; wait for all DONE";
    }
  }

  void OnCameraStatus(const Status &s) {
    if (stage_ != Stage::kCamera || s.command_id != camera_id_) {
      return;
    }

    if (s.status == Status::ACK) {
      camera_ack_ = true;
    }

    if (s.status == Status::DONE) {
      stage_ = Stage::kReady;

      active_motions_.clear();

      ResetMeasurements();

      message_ = std::string("CAMERA DONE: ") + ModeName(mode_) + " | click BEFORE, execute RQT motion, click AFTER";

      RCLCPP_INFO(get_logger(), "FINE pose + %s camera DONE", ModeName(mode_));
    }
  }

  // ================================================
  // Image subscriptions
  // ================================================

  void ReceiveGoal(const Image::ConstSharedPtr &rgb, const Image::ConstSharedPtr &depth) {
    if (mode_ == Mode::kGoal && stage_ != Stage::kMenu) {
      Receive(rgb, depth);
    }
  }

  void Receive(const Image::ConstSharedPtr &rgb, const Image::ConstSharedPtr &depth) {
    if (!rgb || rgb->width != 640 || rgb->height != 480) {
      return;
    }

    if (depth && (depth->width != rgb->width || depth->height != rgb->height ||
                  (depth->encoding != sensor_msgs::image_encodings::TYPE_16UC1 &&
                   depth->encoding != sensor_msgs::image_encodings::TYPE_32FC1))) {
      return;
    }

    try {
      auto f = std::make_shared<Frame>();

      f->rgb = cv_bridge::toCvShare(rgb, "bgr8")->image.clone();

      if (depth) {
        f->depth = cv_bridge::toCvShare(depth)->image.clone();
      }

      if (camera_info_ && camera_info_->width == rgb->width && camera_info_->height == rgb->height &&
          camera_info_->k[0] > 0 && camera_info_->k[4] > 0 &&
          (camera_info_->header.frame_id.empty() || rgb->header.frame_id.empty() ||
           camera_info_->header.frame_id == rgb->header.frame_id)) {
        f->K = vision_core::Intrinsics{camera_info_->k[0], camera_info_->k[4], camera_info_->k[2], camera_info_->k[5]};
      }

      latest_ = std::move(f);
    } catch (const std::exception &e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Camera conversion failed: %s", e.what());
    }
  }

  void ResetMeasurements() {
    ++measurement_generation_;
    recent_.clear();

    last_point_.reset();
    last_box_.reset();

    latest_.reset();
    processed_.reset();
  }

  // ================================================
  // YOLO inference and measurements
  // ================================================

  void ProcessFrame() {
    if (stage_ != Stage::kReady || !active_motions_.empty() || !latest_ || latest_ == processed_ || !yolo_) {
      return;
    }

    const auto now = Clock::now();

    if (processed_ && std::chrono::duration<double>(now - last_infer_).count() < .066) {
      return;
    }

    processed_ = latest_;

    last_infer_ = now;

    std::vector<vision::Detection> detections;

    if (!yolo_->Infer(processed_->rgb, detections)) {
      return;
    }

    last_point_.reset();
    last_box_.reset();

    // ============================================
    // BALL
    // ============================================

    if (mode_ == Mode::kBall) {
      const vision::Detection *best = nullptr;

      for (const auto &d : detections) {
        if (d.class_id == 1 && d.confidence >= .60f && d.box.width >= 20 && d.box.height >= 20 &&
            (!best || d.confidence > best->confidence)) {
          best = &d;
        }
      }

      if (best) {
        last_box_ = best->box;

        last_point_ = Reading{best->box.x + best->box.width * .5,

                              best->box.y + best->box.height * .5};
      }
    }

    // ============================================
    // HURDLE: LINE Offset / Heading
    // ============================================

    else if (mode_ == Mode::kHurdle) {
      std::vector<vision_core::Point2> pts;

      for (const auto &d : detections) {
        if (d.class_id == 0 && d.confidence >= .60f && d.box.width >= 1 && d.box.height >= 1) {
          pts.push_back({d.box.x + d.box.width * .5, d.box.y + d.box.height * .5});
        }
      }

      vision_core::FeatureConfig c{};

      c.max_centers = 8;
      c.guide_min_points = 2;

      // HURDLE DOWN reference.
      c.image_center_u = 380.0;

      // Same as HURDLE FINE O/H.
      c.ransac_enabled = false;

      c.curve_local_fit_points = 4;
      c.curve_min_points = 5;
      c.curve_min_v_span_px = 80.0;

      c.guide_fit_rmse_full_scale_px = 20.0;

      const auto guide = vision_core::ComputeLineFeatures(pts, 640, 480, c, 2).guide;

      if (guide.valid) {
        last_point_ = Reading{guide.offset * 320.0, guide.heading_rad * 180.0 / kPi};
      }
    }

    // ============================================
    // GOAL: Rim X / Z
    // ============================================

    else if (mode_ == Mode::kGoal && processed_->K && !processed_->depth.empty()) {
      const vision::Detection *best = nullptr;

      for (const auto &d : detections) {
        if (d.class_id == 3 && d.confidence >= .60f && d.box.width >= 20 && d.box.height >= 20 &&
            (!best || d.confidence > best->confidence)) {
          best = &d;
        }
      }

      if (best) {
        last_box_ = best->box;

        const auto sample = vision::SampleBackboardDepth(processed_->depth, best->box,

                                                         21, // center ROI size
                                                         7,  // strip width
                                                         7,  // strip step
                                                         28, // max strip width

                                                         .10, // inner margin
                                                         .10, // valid depth ratio

                                                         .10,  // min depth
                                                         2.50, // max depth

                                                         true);

        if (sample.center && sample.left && sample.right) {
          const auto pose = vision_core::EstimateGoalPoseFromBackboardDepths(best->box.x + best->box.width * .5,

                                                                             *sample.center,

                                                                             sample.regions.left_u, *sample.left,

                                                                             sample.regions.right_u, *sample.right,

                                                                             *processed_->K, best->confidence);

          if (pose.valid) {
            // Hardcoded hoop radius: 0.10 m.

            last_point_ = Reading{pose.x_m - .10 * std::sin(pose.yaw_rad),

                                  pose.z_m - .10 * std::cos(pose.yaw_rad)};
          }
        }
      }
    }

    // ============================================
    // 10-frame stationary observation
    // ============================================

    recent_.push_back(last_point_);

    if (recent_.size() > 10) {
      recent_.pop_front();
    }
  }

  std::optional<Reading> Average() const {
    if (recent_.size() < 10) {
      return {};
    }

    const int min_hits = mode_ == Mode::kBall ? 5 : mode_ == Mode::kHurdle ? 5 : 7;

    Reading sum{};

    int n = 0;

    for (const auto &x : recent_) {
      if (!x) {
        continue;
      }

      sum.a += x->a;
      sum.b += x->b;

      ++n;
    }

    if (n < min_hits) {
      return {};
    }

    return Reading{sum.a / n, sum.b / n};
  }

  // ================================================
  // BEFORE / AFTER
  // ================================================

  void Capture(bool before, const std::optional<Reading>* snapshot=nullptr) {
    if (stage_ != Stage::kReady || !active_motions_.empty()) {
      message_ = "Wait for setup / motion DONE";

      return;
    }

    auto mean = snapshot ? *snapshot : Average();

    if (!mean) {
      message_ = "Need 10 new frames and enough valid detections";

      return;
    }

    if (before) {
      before_ = mean;
    } else {
      after_ = mean;
    }

    // Force fresh frames for the next measurement.
    ResetMeasurements();

    message_ = before ? "BEFORE saved; execute RQT motion" : "AFTER saved";

    RCLCPP_INFO(get_logger(), "%s (%+.5f,%+.5f)", before ? "BEFORE" : "AFTER", mean->a, mean->b);

    if (before_ && after_) {
      RCLCPP_INFO(get_logger(), "DELTA (%+.5f,%+.5f)", after_->a - before_->a, after_->b - before_->b);
    }
  }

  // ================================================
  // Hardcoded debug guides
  // ================================================

  static void DrawGuides(cv::Mat &view, Mode mode) {
    // ============================================
    // BALL: Pickup polygon
    // ============================================

    if (mode == Mode::kBall) {
      const vision_core::PickupPolygon original = {{384, 338}, {370, 282}, {394, 256},
                                                   {438, 254}, {450, 298}, {433, 338}};

      // Initial 10% shrink, matching live BALL debug.

      const auto polygon = vision_core::ScalePickupPolygon(original, .90);

      std::vector<cv::Point> vertices;

      for (const auto &p : polygon) {
        vertices.emplace_back(static_cast<int>(std::lround(p.u)), static_cast<int>(std::lround(p.v)));
      }

      cv::polylines(view, std::vector<std::vector<cv::Point>>{vertices}, true, {0, 255, 255}, 1, cv::LINE_AA);

      PutText(view, "PICKUP REGION (initial 10% inset)", {8, 28}, .49, {0, 255, 255});
    }

    // ============================================
    // HURDLE: DOWN horizontal lines
    // ============================================

    else if (mode == Mode::kHurdle) {
      // down_one_step_v_px = 120

      cv::line(view, {0, 120}, {639, 120}, {255, 120, 255}, 1, cv::LINE_AA);

      // fine_mode_v_px = 240

      cv::line(view, {0, 240}, {639, 240}, {200, 0, 200}, 1, cv::LINE_AA);

      // contact_v_px = 240
      // Same physical row as the FINE threshold.

      cv::line(view, {0, 240}, {639, 240}, {0, 255, 255}, 1, cv::LINE_AA);

      PutText(view, "DOWN search v=120", {8, 110}, .47, {255, 120, 255});

      PutText(view, "FINE / CONTACT v=240", {8, 228}, .47, {0, 255, 255});
    }

    // ============================================
    // GOAL: AUTO vertical lines
    // ============================================

    else if (mode == Mode::kGoal) {
      // auto_approach_center_u_px = 320
      // auto_approach_half_width_px = 100
      //
      // LEFT  = 220
      // RIGHT = 420

      cv::line(view, {220, 0}, {220, 479}, {0, 160, 160}, 1, cv::LINE_AA);

      cv::line(view, {420, 0}, {420, 479}, {0, 160, 160}, 1, cv::LINE_AA);

      PutText(view, "GOAL AUTO: x=220 / x=420", {8, 28}, .49, {0, 160, 160});
    }
  }

  // ================================================
  // GUI
  // ================================================

  void Draw() {
    cv::Mat canvas(kCanvasH, kCanvasW, CV_8UC3, cv::Scalar(27, 30, 33));

    // ============================================
    // Mode selection menu
    // ============================================

    if (stage_ == Stage::kMenu) {
      PutText(canvas, "JANDI MOTION CALIBRATION", {300, 95}, 1.05, {250, 250, 250});

      PutText(canvas, "1. Choose a measurement  2. Click OK", {313, 143}, .65);

      Button(canvas, kMenuBall, "BALL - PIXEL", true, selected_ == Mode::kBall);

      Button(canvas, kMenuHurdle, "HURDLE - O/H", true, selected_ == Mode::kHurdle);

      Button(canvas, kMenuGoal, "GOAL - X/Z", true, selected_ == Mode::kGoal);

      Button(canvas, kMenuOK, "OK - APPLY CAMERA", selected_ != Mode::kNone);

      Button(canvas, kMenuQuit, "EXIT");

      PutText(canvas, "No robot commands until OK is clicked.", {295, 560}, .62, {180, 210, 230});

      PutText(canvas, message_, {70, 615}, .50, {255, 195, 130});
    }

    // ============================================
    // Measurement screen
    // ============================================

    else {
      PutText(canvas, std::string("JANDI | ") + ModeName(mode_), {22, 45}, .9, {250, 250, 250});

      PutText(canvas,
              stage_ == Stage::kReady  ? "CAMERA READY"
              : stage_ == Stage::kPose ? "WAIT FINE POSE DONE"
                                       : "WAIT CAMERA DONE",
              {697, 102}, .64, {0, 210, 255});

      cv::Mat video = processed_ ? processed_->rgb.clone() : cv::Mat::zeros(480, 640, CV_8UC3);

      if (stage_ == Stage::kReady) {
        DrawGuides(video, mode_);
      }

      if (last_box_ && stage_ == Stage::kReady) {
        cv::rectangle(video, *last_box_, {0, 240, 0}, 2, cv::LINE_AA);

        if (mode_ == Mode::kBall) {
          cv::drawMarker(video,
                         {last_box_->x + last_box_->width / 2,

                          last_box_->y + last_box_->height / 2},
                         {0, 255, 255}, cv::MARKER_CROSS, 14, 2);
        }
      }

      if (stage_ != Stage::kReady) {
        PutText(video, "SETTING ROBOT POSE / CAMERA...", {70, 235}, .74, {0, 210, 255});
      }

      video.copyTo(canvas(kVideo));

      cv::rectangle(canvas, kVideo, {110, 110, 110}, 1);

      const bool ready = stage_ == Stage::kReady && active_motions_.empty();

      Button(canvas, kBefore, "SAVE BEFORE", ready);

      Button(canvas, kAfter, "SAVE AFTER", ready);

      Button(canvas, kClear, "CLEAR MEASUREMENT", stage_ == Stage::kReady);

      Button(canvas, kSetup, "REAPPLY FINE + CAMERA", ready);

      Button(canvas, kBack, "CHOOSE MODE", ready);

      Button(canvas, kQuit, "EXIT");

      const std::string unit = mode_ == Mode::kBall     ? "u,v (px)"
                               : mode_ == Mode::kHurdle ? "Offset (px), Heading (deg)"
                                                        : "Rim X,Z (m)";

      PutText(canvas, unit, {24, 606}, .55, {160, 230, 235});

      PutText(canvas, before_ ? Format("BEFORE", before_->a, before_->b) : "BEFORE: --", {22, 630}, .53);

      PutText(canvas, after_ ? Format("AFTER", after_->a, after_->b) : "AFTER: --", {355, 630}, .53);

      if (before_ && after_) {
        PutText(canvas, Format("DELTA", after_->a - before_->a, after_->b - before_->b), {700, 630}, .55,
                {70, 255, 160});
      }

      PutText(canvas, message_, {22, 662}, .49, {220, 200, 120});
    }

    debug_view_->Present(canvas, WebContext{web_revision_,measurement_generation_,stage_,mode_,selected_,Average()},
                         web_revision_ ^ (measurement_generation_ << 32));
    if (!debug_view_->Local()) return;
    cv::imshow(kWindow, canvas);

    // Process mouse events.
    // Program termination is handled by the EXIT button.
    cv::waitKey(1);
  }

  // ================================================
  // Main timer
  // ================================================

  void Tick() {
    try {
      for (const auto &event : debug_view_->Inputs()) {
        const auto &context=std::any_cast<const WebContext&>(event.context);
        if (context.revision!=web_revision_ || context.generation!=measurement_generation_ ||
            context.stage!=stage_ || context.mode!=mode_ || context.selected!=selected_) continue;
        if (event.input.kind==vision::DebugMjpegServer::Input::Kind::Click && event.input.button==0)
          OnClick({event.input.x,event.input.y}, &context.mean);
      }
      const auto now = Clock::now();

      // Retry with the same ID until ACK.

      if (stage_ == Stage::kPose && !pose_ack_ &&
          (last_send_ == Clock::time_point{} || std::chrono::duration<double>(now - last_send_).count() >= 1.0)) {
        SendPose();
      }

      if (stage_ == Stage::kCamera && !camera_ack_ &&
          (last_send_ == Clock::time_point{} || std::chrono::duration<double>(now - last_send_).count() >= 1.0)) {
        SendCamera();
      }

      ProcessFrame();

      Draw();
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Calibration GUI failed: %s", e.what());

      rclcpp::shutdown();
    }
  }

  // ================================================
  // Member variables
  // ================================================

  std::unique_ptr<camera_overlay::DebugView> debug_view_;
  std::uint64_t web_revision_{0};
  std::uint64_t measurement_generation_{0};
  Mode selected_{Mode::kNone};
  Mode mode_{Mode::kNone};

  Stage stage_{Stage::kMenu};

  std::string message_{"Select BALL / HURDLE / GOAL"};

  std::unique_ptr<vision::YoloTrtEngine> yolo_;

  rclcpp::Publisher<vision::msg::ActionCommand>::SharedPtr action_pub_;

  rclcpp::Publisher<vision::msg::CameraCommand>::SharedPtr camera_pub_;

  rclcpp::Subscription<Status>::SharedPtr action_status_sub_;

  rclcpp::Subscription<Status>::SharedPtr camera_status_sub_;

  rclcpp::Subscription<Image>::SharedPtr rgb_sub_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;

  sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_;

  message_filters::Subscriber<Image> rgb_sync_;
  message_filters::Subscriber<Image> depth_sync_;

  std::shared_ptr<Synchronizer> sync_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<Frame> latest_;
  std::shared_ptr<Frame> processed_;

  std::uint64_t pose_id_{0};
  std::uint64_t camera_id_{0};

  bool pose_ack_{false};
  bool camera_ack_{false};

  std::unordered_set<std::uint64_t> active_motions_;

  Clock::time_point last_send_{};
  Clock::time_point last_infer_{};

  std::deque<std::optional<Reading>> recent_;

  std::optional<Reading> last_point_;
  std::optional<Reading> before_;
  std::optional<Reading> after_;

  std::optional<cv::Rect> last_box_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  int result = 0;

  try {
    rclcpp::spin(std::make_shared<MotionCalibrationNode>());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("motion_calibration_node"), "%s", e.what());

    result = 1;
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  return result;
}
