// Read-only YOLO TensorRT ball pixel inspection. Enter freezes the displayed
// image AND its inference result, so the reported center belongs to that image.
#include "vision/yolo_trt_engine.hpp"
#include "vision/pixel_inspection.hpp"
#include "vision_core/config_loader.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/parameter_map.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <sys/utsname.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

bool IsJetson() {
  struct utsname info {};
  if (uname(&info) != 0) return false;
  return std::string(info.machine) == "aarch64" ||
         std::string(info.machine) == "arm64";
}

// Follow line_perception_node's runtime YAML and environment override rules.
std::string DefaultEnginePath() {
  if (const char *value = std::getenv("YOLO26_ENGINE_PATH");
      value != nullptr && value[0] != '\0') return value;

  std::string path =
      ament_index_cpp::get_package_share_directory("vision") +
      "/config/yolo26_runtime.yaml";
  if (const char *value = std::getenv("YOLO26_RUNTIME_CONFIG");
      value != nullptr && value[0] != '\0') path = value;

  const char *key = IsJetson() ? "jetson_engine_path" : "pc_engine_path";
  try {
    const auto params = rclcpp::parameter_map_from_yaml_file(path);
    for (const char *node_name : {"/yolo26_runtime", "yolo26_runtime"}) {
      const auto it = params.find(node_name);
      if (it == params.end()) continue;
      for (const auto &param : it->second) {
        if (param.get_name() == key) return param.as_string();
      }
    }
  } catch (const std::exception &error) {
    std::cerr << "[ball_pixel_node] Failed to read " << path << ": "
              << error.what() << std::endl;
  }
  return {};
}

void PrintLine(cv::Mat &image, const std::string &text, int row) {
  const cv::Point p(10, 25 + row * 23);
  cv::putText(image, text, p, cv::FONT_HERSHEY_SIMPLEX, 0.53,
              cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
  cv::putText(image, text, p, cv::FONT_HERSHEY_SIMPLEX, 0.53,
              cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
}

} // namespace

class BallPixelNode final : public rclcpp::Node {
public:
  BallPixelNode() : Node("ball_pixel_node") {
    const auto config = vision_core::LoadDefaultAlgorithmConfig();
    const auto image_topic = declare_parameter<std::string>(
        "image_topic", "/camera/camera/color/image_raw");
    const auto engine_path =
        declare_parameter<std::string>("engine_path", DefaultEnginePath());
    ball_class_id_ = declare_parameter<int>(
        "ball_class_id", config.object_targets.ball_class_id);
    const double conf = declare_parameter<double>(
        "conf_thres", config.line_detection.confidence);
    const double ball_conf = declare_parameter<double>(
        "ball_conf_thres", config.object_targets.ball_confidence);
    const double inference_hz = declare_parameter<double>("inference_hz", 15.0);

    if (image_topic.empty() || engine_path.empty())
      throw std::invalid_argument("image_topic and engine_path must not be empty");
    if (!std::isfinite(conf) || conf < 0 || conf > 1 ||
        !std::isfinite(ball_conf) || ball_conf < 0 || ball_conf > 1)
      throw std::invalid_argument("conf_thres and ball_conf_thres must be in [0,1]");
    if (!std::isfinite(inference_hz) || inference_hz < 1 || inference_hz > 60)
      throw std::invalid_argument("inference_hz must be within [1,60]");
    if (ball_class_id_ < 0)
      throw std::invalid_argument("ball_class_id must be nonnegative");

    ball_confidence_ = static_cast<float>(ball_conf);
    engine_ = std::make_unique<vision::YoloTrtEngine>(
        engine_path, 640, 640, static_cast<float>(conf));

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
          if (frozen_) return;
          try {
            auto frame = std::make_shared<Frame>();
            frame->image = cv_bridge::toCvShare(message, "bgr8")->image.clone();
            frame->stamp_ns = rclcpp::Time(message->header.stamp).nanoseconds();
            if (!frame->image.empty()) latest_ = std::move(frame);
          } catch (const std::exception &error) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                                 "Image conversion failed: %s", error.what());
          }
        });

    cv::namedWindow(kWindow, cv::WINDOW_AUTOSIZE);
    const auto period =
        std::chrono::milliseconds(static_cast<int>(1000.0 / inference_hz));
    timer_ = create_wall_timer(period, [this]() { Draw(); });
    RCLCPP_INFO(get_logger(),
                "RGB: %s | TensorRT: %s | ball class=%d | conf=%.2f/%.2f | ENTER: freeze + center | SPACE/c: live | q/Esc: exit",
                image_topic.c_str(), engine_path.c_str(), ball_class_id_,
                conf, ball_conf);
    RCLCPP_INFO(get_logger(),
                "This is RAW YOLO output; no mission depth/size/temporal acceptance applied.");
  }

  ~BallPixelNode() override {
    try { cv::destroyWindow(kWindow); } catch (const cv::Exception &) {}
  }

private:
  static constexpr const char *kWindow = "YOLO ball - bbox and center";
  struct Frame {
    cv::Mat image;
    std::int64_t stamp_ns{0};
    bool infer_ok{false};
    std::vector<vision::Detection> detections;
  };

  void Draw() {
    try {
      // spin() is single threaded: subscription, inference and window callbacks
      // never mutate the same snapshot concurrently.
      if (!frozen_ && latest_ && latest_ != displayed_) {
        auto candidate = latest_;
        try {
          candidate->infer_ok =
              engine_->Infer(candidate->image, candidate->detections);
        } catch (const std::exception &error) {
          candidate->infer_ok = false;
          candidate->detections.clear();
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000,
                                "YOLO inference failed: %s", error.what());
        }
        displayed_ = candidate;
      }

      const auto frame = frozen_ ? frozen_ : displayed_;
      cv::Mat view = frame ? frame->image.clone()
                           : cv::Mat::zeros(480, 640, CV_8UC3);

      if (!frame) {
        PrintLine(view, "Waiting for RGB image...", 0);
      } else {
        // Overlay every RAW ball detection and its center (not a core target).
        int count = 0;
        for (const auto &d : frame->detections) {
          if (d.class_id != ball_class_id_ ||
              d.confidence < ball_confidence_ ||
              d.box.width <= 0 || d.box.height <= 0) continue;
          ++count;
          cv::rectangle(view, d.box, cv::Scalar(0, 230, 0), 2);
          const auto center = vision::BoxCenter(d.box);
          cv::circle(view, center, 4, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
          char conf_text[64];
          std::snprintf(conf_text, sizeof(conf_text), "BALL %.2f", d.confidence);
          const cv::Point label(
              std::clamp(d.box.x, 0, std::max(0, view.cols - 100)),
              std::clamp(d.box.y - 6, 13, std::max(13, view.rows - 1)));
          cv::putText(view, conf_text, label, cv::FONT_HERSHEY_SIMPLEX, 0.43,
                      cv::Scalar(0, 230, 0), 1, cv::LINE_AA);
        }
        PrintLine(view, frozen_ ? "FROZEN | SPACE/c: live"
                                : "LIVE | ENTER: freeze and show BALL center", 0);
        char status[160];
        std::snprintf(status, sizeof(status),
                      "raw balls=%d | image=%dx%d | stamp=%.6f",
                      count, view.cols, view.rows, frame->stamp_ns * 1e-9);
        PrintLine(view, status, 1);
        if (!frame->infer_ok) {
          PrintLine(view, "Inference unavailable on displayed frame", 2);
        } else if (frozen_) {
          if (selected_ball_) {
            const auto center = vision::BoxCenter(selected_ball_->box);
            cv::drawMarker(view, center, cv::Scalar(0, 255, 255),
                           cv::MARKER_CROSS, 23, 2, cv::LINE_AA);
            std::snprintf(status, sizeof(status),
                          "BALL CENTER (u,v) = (%d, %d) | conf=%.3f",
                          center.x, center.y, selected_ball_->confidence);
            PrintLine(view, status, 2);
          } else {
            PrintLine(view, "NO BALL at ENTER (frame frozen)", 2);
          }
        } else {
          PrintLine(view, "RAW YOLO; highest-confidence ball selected on ENTER", 2);
        }
      }

      cv::imshow(kWindow, view);
      const int key = cv::waitKey(1);
      if (key == 27 || key == 'q' || key == 'Q') {
        rclcpp::shutdown();
      } else if (key == ' ' || key == 'c' || key == 'C') {
        frozen_.reset();
        selected_ball_.reset();
      } else if ((key == 10 || key == 13) && frame && !frozen_) {
        // Freeze exactly the image/detections just shown, NOT latest_.
        frozen_ = frame;
        selected_ball_ =
            vision::SelectHighestConfidenceBall(frame->detections, ball_class_id_, ball_confidence_);
        if (selected_ball_) {
          const auto center = vision::BoxCenter(selected_ball_->box);
          RCLCPP_INFO(get_logger(),
                      "FROZEN BALL center (u,v)=(%d,%d) confidence=%.3f stamp=%.9f bbox=(%d,%d,%d,%d)",
                      center.x, center.y, selected_ball_->confidence,
                      frame->stamp_ns * 1e-9, selected_ball_->box.x,
                      selected_ball_->box.y, selected_ball_->box.width,
                      selected_ball_->box.height);
        } else {
          RCLCPP_WARN(get_logger(),
                      "FROZEN: no ball above threshold on displayed frame.");
        }
      }
    } catch (const cv::Exception &error) {
      RCLCPP_ERROR(get_logger(), "OpenCV window failed: %s", error.what());
      rclcpp::shutdown();
    }
  }

  int ball_class_id_{1};
  float ball_confidence_{0.6f};
  std::unique_ptr<vision::YoloTrtEngine> engine_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<Frame> latest_, displayed_;
  std::shared_ptr<const Frame> frozen_;
  std::optional<vision::Detection> selected_ball_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<BallPixelNode>());
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("ball_pixel_node"), "%s", error.what());
    result = 1;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return result;
}
