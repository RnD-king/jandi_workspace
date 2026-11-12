// Two independent live viewers. FORWARD never commands the camera; GOAL only
// requests GOAL once (same-ID retries until actual DONE). Neither commands body motion.
#include "camera_overlay/debug_view.hpp"
#include "camera_overlay/yolo_inspection.hpp"
#include <vision/yolo_trt_engine.hpp>
#include <vision/msg/camera_command.hpp>
#include <vision/msg/command_status.hpp>
#include <vision_core/config_loader.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <rclcpp/parameter_map.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <sys/utsname.h>
#include <unistd.h>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
#ifdef CAMERA_OVERLAY_GOAL_INSPECTION
constexpr bool kGoal = true;
constexpr const char* kNodeName = "goal_yolo_node";
#else
constexpr bool kGoal = false;
constexpr const char* kNodeName = "forward_yolo_node";
#endif
using Clock = std::chrono::steady_clock;
using Image = sensor_msgs::msg::Image;
using Status = vision::msg::CommandStatus;

std::string DefaultEnginePath() {
  if (const char* path = std::getenv("YOLO26_ENGINE_PATH"); path && *path) return path;
  const char* override_path = std::getenv("YOLO26_RUNTIME_CONFIG");
  const auto path = override_path && *override_path ? std::string(override_path) :
      ament_index_cpp::get_package_share_directory("vision") + "/config/yolo26_runtime.yaml";
  struct utsname info {};
  const bool jetson = uname(&info) == 0 &&
      (std::string(info.machine) == "aarch64" || std::string(info.machine) == "arm64");
  const auto parameters = rclcpp::parameter_map_from_yaml_file(path);
  for (const auto* name : {"/yolo26_runtime", "yolo26_runtime"}) {
    const auto node = parameters.find(name);
    if (node == parameters.end()) continue;
    for (const auto& parameter : node->second)
      if (parameter.get_name() == (jetson ? "jetson_engine_path" : "pc_engine_path"))
        return parameter.as_string();
  }
  throw std::runtime_error("YOLO runtime configuration has no engine path: " + path);
}

void Label(cv::Mat& image, const std::string& text, int y) {
  cv::putText(image, text, {8, y}, cv::FONT_HERSHEY_SIMPLEX, .46,
      {0, 0, 0}, 3, cv::LINE_AA);
  cv::putText(image, text, {8, y}, cv::FONT_HERSHEY_SIMPLEX, .46,
      {240, 240, 240}, 1, cv::LINE_AA);
}
}  // namespace

class YoloInspectionNode final : public rclcpp::Node {
public:
  YoloInspectionNode() : Node(kNodeName), config_(vision_core::LoadDefaultAlgorithmConfig()) {
    // All settings describe this process lifetime; changing pose requires a new viewer.
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    const auto rgb_topic = declare_parameter<std::string>("image_topic",
        "/camera/camera/color/image_raw", descriptor);
    const auto engine_path = declare_parameter<std::string>("engine_path", DefaultEnginePath(), descriptor);
    const auto confidence = declare_parameter<double>("conf_thres", config_.line_detection.confidence, descriptor);
    const auto hz = declare_parameter<double>("inference_hz", 15., descriptor);
    if (rgb_topic.empty() || engine_path.empty())
      throw std::invalid_argument("image_topic and engine_path must not be empty");
    if (!std::isfinite(confidence) || confidence < 0 || confidence > 1 ||
        !std::isfinite(hz) || hz < 1 || hz > 60)
      throw std::invalid_argument("conf_thres must be in [0,1]; inference_hz in [1,60]");
    yolo_ = std::make_unique<vision::YoloTrtEngine>(engine_path, 640, 640, confidence);
    view_ = std::make_unique<camera_overlay::DebugView>(*this);
    if (view_->Local()) cv::namedWindow(kNodeName, cv::WINDOW_AUTOSIZE);

    rgb_sub_ = create_subscription<Image>(rgb_topic, rclcpp::SensorDataQoS(),
        [this](Image::ConstSharedPtr image) { latest_rgb_ = image; });
    if (kGoal) {
      const auto depth_topic = declare_parameter<std::string>("depth_topic",
          "/camera/camera/aligned_depth_to_color/image_raw", descriptor);
      if (depth_topic.empty()) throw std::invalid_argument("depth_topic must not be empty");
      depth_sub_ = create_subscription<Image>(depth_topic, rclcpp::SensorDataQoS(),
          [this](Image::ConstSharedPtr depth) {
            depths_.push_back(depth);
            while (depths_.size() > 10) depths_.pop_front();
          });
      camera_id_ = static_cast<std::uint64_t>(Clock::now().time_since_epoch().count()) ^
          (static_cast<std::uint64_t>(getpid()) << 32);
      if (camera_id_ == 0) camera_id_ = 1;
      camera_pub_ = create_publisher<vision::msg::CameraCommand>("/jandi_vision/camera_cmd", 10);
      camera_status_sub_ = create_subscription<Status>("/jandi_vision/camera_status", 10,
          [this](Status::ConstSharedPtr status) {
            if (camera_done_ || !camera_sent_ || status->command_id != camera_id_) return;
            if (status->status == Status::ACK) camera_ack_ = true;
            if (status->status == Status::DONE) {
              camera_done_ = true;
              camera_done_stamp_ = get_clock()->now().nanoseconds();
              latest_rgb_.reset();
              depths_.clear();
              RCLCPP_INFO(get_logger(), "GOAL camera DONE; live inspection only");
            }
          });
    }
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1. / hz)), [this]() { Tick(); });
    RCLCPP_INFO(get_logger(), "%s | RGB: %s | engine: %s | no START required",
        kNodeName, rgb_topic.c_str(), engine_path.c_str());
  }

  ~YoloInspectionNode() override {
    if (view_ && view_->Local()) {
      try { cv::destroyWindow(kNodeName); } catch (const cv::Exception&) {}
    }
  }

private:
  void PrepareCamera() {
    if (!kGoal || camera_done_) return;
    if (camera_pub_->get_subscription_count() == 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
          "Waiting for forward_walk/main_node on /jandi_vision/camera_cmd");
      return;
    }
    const auto now = Clock::now();
    if (camera_sent_ && now - last_camera_send_ < std::chrono::milliseconds(500)) return;
    vision::msg::CameraCommand command;
    command.command_id = camera_id_;
    command.request = vision::msg::CameraCommand::GOAL;
    // Retry the same transaction even after ACK so a lost DONE can be replayed.
    camera_pub_->publish(command);
    camera_sent_ = true;
    last_camera_send_ = now;
  }

  cv::Mat MatchedDepth(const Image& rgb) {
    const auto stamp = rclcpp::Time(rgb.header.stamp).nanoseconds();
    if (stamp <= 0) return {};
    Image::ConstSharedPtr match;
    auto delta = std::numeric_limits<std::int64_t>::max();
    for (const auto& depth : depths_) {
      const auto depth_stamp = rclcpp::Time(depth->header.stamp).nanoseconds();
      if (depth->width != rgb.width || depth->height != rgb.height ||
          depth_stamp <= camera_done_stamp_) continue;
      const auto difference = std::abs(depth_stamp - stamp);
      if (difference <= 30000000 && difference < delta) {
        delta = difference;
        match = depth;
      }
    }
    if (!match) return {};
    try {
      const auto depth = cv_bridge::toCvShare(match)->image;
      if (depth.type() != CV_16UC1 && depth.type() != CV_32FC1) return {};
      return depth.clone();
    } catch (const std::exception& error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
          "Aligned depth rejected: %s", error.what());
      return {};
    }
  }

  void Tick() {
    PrepareCamera();
    // This viewer accepts no browser action, START, preview, or commit input.
    view_->Inputs();
    auto rgb = std::move(latest_rgb_);
    if (rgb) {
      try {
        const auto bgr = cv_bridge::toCvShare(rgb, "bgr8")->image;
        if (!bgr.empty()) {
          std::vector<vision::Detection> detections;
          const bool ok = yolo_->Infer(bgr, detections);
          if (!ok) detections.clear();
          const bool bands = kGoal && camera_done_ &&
              rclcpp::Time(rgb->header.stamp).nanoseconds() > camera_done_stamp_;
          const auto depth = bands ? MatchedDepth(*rgb) : cv::Mat();
          displayed_ = camera_overlay::RenderYoloInspection(bgr, detections, depth, config_, bands);
          depth_available_ = !depth.empty();
          if (!ok) Label(displayed_, "YOLO inference failed", 44);
        }
      } catch (const std::exception& error) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Frame rejected: %s", error.what());
        displayed_ = cv::Mat();
      }
    }
    cv::Mat image = displayed_.empty() ? cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0)) : displayed_.clone();
    if (displayed_.empty()) Label(image, "Waiting for RGB image...", 44);
    const std::string status = !kGoal ? "FORWARD | LIVE YOLO" :
        !camera_done_ ? (camera_ack_ ? "GOAL camera moving | waiting for DONE" : "Waiting for GOAL camera executor") :
        "GOAL | LIVE YOLO + depth bands";
    Label(image, status, 20);
    if (kGoal && camera_done_ && !depth_available_)
      Label(image, "Aligned depth unavailable (RGB-D match <= 30ms required)", image.rows - 10);
    view_->Present(image);
    if (view_->Local()) {
      cv::imshow(kNodeName, image);
      const int key = cv::waitKey(1) & 0xff;
      if (key == 27 || key == 'q') rclcpp::shutdown();
    }
  }

  vision_core::MissionControllerConfig config_;
  std::unique_ptr<vision::YoloTrtEngine> yolo_;
  std::unique_ptr<camera_overlay::DebugView> view_;
  rclcpp::Subscription<Image>::SharedPtr rgb_sub_, depth_sub_;
  rclcpp::Publisher<vision::msg::CameraCommand>::SharedPtr camera_pub_;
  rclcpp::Subscription<Status>::SharedPtr camera_status_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  Image::ConstSharedPtr latest_rgb_;
  std::deque<Image::ConstSharedPtr> depths_;
  cv::Mat displayed_;
  std::uint64_t camera_id_{};
  std::int64_t camera_done_stamp_{};
  bool camera_sent_{false}, camera_ack_{false}, camera_done_{false}, depth_available_{false};
  Clock::time_point last_camera_send_{};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<YoloInspectionNode>());
  } catch (const std::exception& error) {
    std::cerr << kNodeName << ": " << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
