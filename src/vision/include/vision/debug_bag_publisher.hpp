#pragma once

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <utility>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace vision {

// Recording is external. This class only publishes the rendered debug views.
// cv::Mat buffers passed to Submit must be owned and not modified afterwards.
class DebugBagPublisher {
public:
  struct Request {
    bool image{false}, panel{false}, yolo{false};
    explicit operator bool() const { return image || panel || yolo; }
  };

  explicit DebugBagPublisher(rclcpp::Node &node, bool include_yolo = true)
      : logger_(node.get_logger()) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    descriptor.description = "Publish debug image topics for an external rosbag recorder (startup only)";
    enabled_ = node.declare_parameter<bool>("rosbag", false, descriptor);
    descriptor.description = "Maximum debug image publication rate; rendering is limited to processed camera frames";
    const double fps = node.declare_parameter<double>("rosbag_fps", 10.0, descriptor);
    if (!std::isfinite(fps) || fps <= 0.0) {
      throw std::invalid_argument("rosbag_fps must be finite and > 0");
    }
    period_ = std::chrono::duration<double>(1.0 / fps);
    if (!enabled_) return;

    // Never make the perception loop wait for reliable delivery of debug frames.
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    image_pub_ = node.create_publisher<sensor_msgs::msg::Image>("/jandi_vision/debug/image", qos);
    panel_pub_ = node.create_publisher<sensor_msgs::msg::Image>("/jandi_vision/debug/panel", qos);
    if (include_yolo) {
      yolo_pub_ = node.create_publisher<sensor_msgs::msg::Image>("/jandi_vision/debug/yolo", qos);
    }
    worker_ = std::thread([this]() { PublishLoop(); });
    RCLCPP_INFO(logger_, "[ROSBAG] debug image topics enabled, max %.1f fps; waiting for subscribers", fps);
  }

  ~DebugBagPublisher() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
      pending_.reset();
    }
    condition_.notify_one();
    if (worker_.joinable()) worker_.join();
  }

  DebugBagPublisher(const DebugBagPublisher &) = delete;
  DebugBagPublisher &operator=(const DebugBagPublisher &) = delete;

  // Called only from the node's image-processing callback. The steady clock
  // keeps the rate bound even if ROS time is paused or jumps during replay.
  Request RequestFrame() {
    if (!enabled_) return {};
    const auto now = std::chrono::steady_clock::now();
    if (last_frame_ && now - *last_frame_ < period_) return {};
    Request request{HasSubscribers(image_pub_), HasSubscribers(panel_pub_), HasSubscribers(yolo_pub_)};
    if (request) last_frame_ = now;
    return request;
  }

  void Submit(const Request &request, const std_msgs::msg::Header &header,
              const cv::Mat &image, const cv::Mat &panel, const cv::Mat &yolo = {}) {
    if (!request) return;
    Frame frame{header, request.image ? image : cv::Mat{},
                request.panel ? panel : cv::Mat{}, request.yolo ? yolo : cv::Mat{}};
    {
      std::lock_guard<std::mutex> lock(mutex_);
      // At most one waiting frame: replace old work instead of accumulating lag.
      pending_ = std::move(frame);
    }
    condition_.notify_one();
  }

private:
  using Publisher = rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr;
  struct Frame {
    std_msgs::msg::Header header;
    cv::Mat image, panel, yolo;
  };

  static bool HasSubscribers(const Publisher &publisher) {
    return publisher && (publisher->get_subscription_count() > 0 ||
                         publisher->get_intra_process_subscription_count() > 0);
  }

  static void Publish(const Publisher &publisher, const std_msgs::msg::Header &header,
                      const cv::Mat &image) {
    if (image.empty() || !HasSubscribers(publisher)) return;
    publisher->publish(*cv_bridge::CvImage(header, "bgr8", image).toImageMsg());
  }

  void PublishLoop() {
    while (true) {
      Frame frame;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this]() { return stopping_ || pending_.has_value(); });
        if (stopping_) return;
        frame = std::move(*pending_);
        pending_.reset();
      }
      try {
        // Message allocation, pixel copying, serialization and DDS publication
        // happen on this worker, outside the perception/control callback.
        Publish(image_pub_, frame.header, frame.image);
        Publish(panel_pub_, frame.header, frame.panel);
        Publish(yolo_pub_, frame.header, frame.yolo);
      } catch (const std::exception &error) {
        RCLCPP_WARN(logger_, "[ROSBAG] debug frame dropped: %s", error.what());
      }
    }
  }

  rclcpp::Logger logger_;
  bool enabled_{false};
  std::chrono::duration<double> period_{};
  std::optional<std::chrono::steady_clock::time_point> last_frame_;
  Publisher image_pub_, panel_pub_, yolo_pub_;
  std::mutex mutex_;
  std::condition_variable condition_;
  std::optional<Frame> pending_;
  bool stopping_{false};
  std::thread worker_;
};

}  // namespace vision
