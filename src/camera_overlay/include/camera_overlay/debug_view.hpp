#pragma once

#include <vision/debug_mjpeg_server.hpp>
#include <rclcpp/rclcpp.hpp>
#include <any>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <stdexcept>
#include <utility>
#include <vector>

namespace camera_overlay {
// Node-thread adapter only. Workers transport bytes/input; they never access
// measurements or publish robot commands. Snapshots refer to submitted pixels.
class DebugView {
public:
  struct Event {
    vision::DebugMjpegServer::Input input;
    std::any context;
  };
  explicit DebugView(rclcpp::Node &node) : node_(node) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    const auto display = std::getenv("DISPLAY"), wayland = std::getenv("WAYLAND_DISPLAY");
    const bool has_display = (display && *display) || (wayland && *wayland);
    local_ = node.declare_parameter<bool>("show_debug_view", has_display, descriptor);
    if (local_ && !has_display) {
      local_ = false;
      RCLCPP_WARN(node.get_logger(), "No desktop display: OpenCV windows disabled");
    }
    const bool stream = node.declare_parameter<bool>("stream_debug_view", false, descriptor);
    const int port = node.declare_parameter<int>("stream_debug_port", 8080, descriptor);
    if (port < 1 || port > 65535) throw std::invalid_argument("invalid stream_debug_port");
    if (stream) {
      try {
        server_ = std::make_unique<vision::DebugMjpegServer>(port, "0.0.0.0", true);
        RCLCPP_INFO(node.get_logger(), "Camera test: http://<JETSON_IP>:%d (mouse/keyboard enabled)", port);
      } catch (const std::exception &error) {
        RCLCPP_ERROR(node.get_logger(), "Test stream disabled: %s", error.what());
      }
    }
  }
  bool Local() const { return local_; }
  bool Streaming() const { return static_cast<bool>(server_); }

  void Present(const cv::Mat &image, std::any context = {}, std::uint64_t revision = 0) {
    if (!server_ || image.empty() || !server_->RequestFrame()) return;
    // No repeated encoding of a frozen/menu image. Drawing is unchanged.
    if (revision == last_revision_ && SameImage(image, last_image_)) return;
    const auto id = ++next_id_;
    if (!server_->Submit(image, id)) return;
    snapshots_.emplace_back(id, std::move(context));
    while (snapshots_.size() > 32) snapshots_.pop_front();
    last_image_ = image;
    last_revision_ = revision;
  }
  std::vector<Event> Inputs() {
    std::vector<Event> result;
    if (!server_) return result;
    vision::DebugMjpegServer::Input input;
    for (unsigned n = 0; n < 32 && server_->TakeInput(input); ++n) {
      bool found = false;
      for (const auto &snapshot : snapshots_) {
        if (snapshot.first == input.frame_id) {
          result.push_back({input, snapshot.second});
          found = true;
          break;
        }
      }
      if (!found) RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
          "Expired browser frame: input ignored; use the current screen");
    }
    return result;
  }

private:
  static bool SameImage(const cv::Mat &a, const cv::Mat &b) {
    if (a.size() != b.size() || a.type() != b.type() || b.empty()) return false;
    const auto bytes = static_cast<std::size_t>(a.cols) * a.elemSize();
    for (int row = 0; row < a.rows; ++row)
      if (std::memcmp(a.ptr(row), b.ptr(row), bytes) != 0) return false;
    return true;
  }
  rclcpp::Node &node_;
  bool local_{};
  std::unique_ptr<vision::DebugMjpegServer> server_;
  // Reject requests left over from a previous node lifetime, even after restart.
  std::uint64_t next_id_{static_cast<std::uint64_t>(
      std::chrono::steady_clock::now().time_since_epoch().count())};
  std::uint64_t last_revision_{};
  cv::Mat last_image_;
  std::deque<std::pair<std::uint64_t, std::any>> snapshots_;
};
}  // namespace camera_overlay
