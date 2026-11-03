// Read-only RGB pixel inspection. Clicking freezes the exact frame shown.
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <memory>
#include <optional>
#include <string>

class ClickPixelNode final : public rclcpp::Node {
public:
  ClickPixelNode() : Node("click_pixel_node") {
    const auto topic = declare_parameter<std::string>(
        "image_topic", "/camera/camera/color/image_raw");
    if (topic.empty()) {
      throw std::invalid_argument("image_topic must not be empty");
    }

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        topic, rclcpp::SensorDataQoS(),
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
    cv::setMouseCallback(kWindow, &ClickPixelNode::OnMouse, this);
    timer_ = create_wall_timer(std::chrono::milliseconds(33),
                               [this]() { Draw(); });
    RCLCPP_INFO(get_logger(),
                "RGB: %s | click: freeze + show (u,v) | SPACE/c: live | q/Esc: exit",
                topic.c_str());
  }

  ~ClickPixelNode() override {
    try { cv::destroyWindow(kWindow); } catch (const cv::Exception &) {}
  }

private:
  static constexpr const char *kWindow = "Click pixel - RGB coordinates";
  struct Frame {
    cv::Mat image;
    std::int64_t stamp_ns{0};
  };

  static void PrintLine(cv::Mat &image, const std::string &text, int row) {
    const cv::Point p(10, 25 + row * 23);
    cv::putText(image, text, p, cv::FONT_HERSHEY_SIMPLEX, 0.53,
                cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
    cv::putText(image, text, p, cv::FONT_HERSHEY_SIMPLEX, 0.53,
                cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
  }

  static void OnMouse(int event, int x, int y, int, void *data) {
    if (event != cv::EVENT_LBUTTONDOWN) return;
    auto *self = static_cast<ClickPixelNode *>(data);
    // 'displayed_' is the exact RGB frame passed to imshow, NOT latest_.
    const auto frame = self->displayed_;
    if (!frame || x < 0 || y < 0 ||
        x >= frame->image.cols || y >= frame->image.rows) return;
    self->frozen_ = frame;
    self->pixel_ = cv::Point(x, y);
    RCLCPP_INFO(self->get_logger(), "CLICK pixel (u,v)=(%d,%d) stamp=%.9f",
                x, y, frame->stamp_ns * 1e-9);
  }

  void Draw() {
    try {
      const auto frame = frozen_ ? frozen_ : latest_;
      cv::Mat view = frame ? frame->image.clone()
                           : cv::Mat::zeros(480, 640, CV_8UC3);
      if (!frame) {
        PrintLine(view, "Waiting for RGB image...", 0);
      } else {
        PrintLine(view, frozen_ ? "FROZEN | click again: move point | SPACE/c: live"
                                : "LIVE | click a pixel to freeze", 0);
        char status[128];
        std::snprintf(status, sizeof(status),
                      "image=%dx%d | stamp=%.6f | q/Esc: exit",
                      view.cols, view.rows, frame->stamp_ns * 1e-9);
        PrintLine(view, status, 1);
        if (frozen_ && pixel_) {
          cv::drawMarker(view, *pixel_, cv::Scalar(0, 255, 255),
                         cv::MARKER_CROSS, 19, 2, cv::LINE_AA);
          std::snprintf(status, sizeof(status), "CLICK (u,v) = (%d, %d)",
                        pixel_->x, pixel_->y);
          PrintLine(view, status, 2);
        }
      }
      cv::imshow(kWindow, view);
      displayed_ = frame;
      const int key = cv::waitKey(1);
      if (key == 27 || key == 'q' || key == 'Q') rclcpp::shutdown();
      if (key == ' ' || key == 'c' || key == 'C') {
        frozen_.reset();
        pixel_.reset();
      }
    } catch (const cv::Exception &error) {
      RCLCPP_ERROR(get_logger(), "OpenCV window failed: %s", error.what());
      rclcpp::shutdown();
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<const Frame> latest_, displayed_, frozen_;
  std::optional<cv::Point> pixel_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<ClickPixelNode>());
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("click_pixel_node"), "%s", error.what());
    result = 1;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return result;
}
