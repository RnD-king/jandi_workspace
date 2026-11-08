#include "camera_overlay/debug_view.hpp"
// ball_pixel_pin_node.cpp
// ROS 2 Humble / OpenCV HighGUI / existing vision::YoloTrtEngine
//
// LEFT CLICK : freeze the currently displayed frame and select the pixel.
// ENTER      : freeze and select the highest-confidence YOLO BALL center.
// I          : freeze and type a pixel coordinate, e.g. 320,240 -> ENTER.
//              ESC cancels coordinate entry and restores previous live/frozen mode.
// POPUP 1/2/3: green pin / red pin / cancel (mouse selection also works).
// RIGHT CLICK: reopen pin menu for the selected coordinate.
// P          : toggle polygon created from green pins (70% transparent fill).
// CTRL+C     : resume live video (both terminal SIGINT and GUI key when supported).
// Q or ESC   : quit (ESC during coordinate entry only cancels entry).
//
// Coordinates always mean original camera image pixels: u=x, v=y, starting at 0.

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/parameter_map.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "vision/yolo_trt_engine.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <sys/utsname.h>

namespace {
// Same engine selection and configuration reader as vision/line_perception_node.
bool IsJetsonTarget() {
  struct utsname info {};
  if (uname(&info) != 0) return false;
  const std::string machine(info.machine);
  return machine == "aarch64" || machine == "arm64";
}

std::string RuntimeConfigPath() {
  if (const char *p = std::getenv("YOLO26_RUNTIME_CONFIG");
      p != nullptr && p[0] != '\0') {
    return std::string(p);
  }
  return ament_index_cpp::get_package_share_directory("vision") +
         "/config/yolo26_runtime.yaml";
}

std::string ReadRuntimeString(const std::string &parameter_name) {
  try {
    const auto map = rclcpp::parameter_map_from_yaml_file(RuntimeConfigPath());
    for (const auto &node_name : {"/yolo26_runtime", "yolo26_runtime"}) {
      const auto it = map.find(node_name);
      if (it == map.end()) continue;
      for (const auto &param : it->second) {
        if (param.get_name() == parameter_name) return param.as_string();
      }
    }
  } catch (const std::exception &e) {
    std::cerr << "[WARN] Failed to load YOLO26 runtime config: "
              << e.what() << std::endl;
  }
  return {};
}

std::string DefaultYolo26EnginePath() {
  if (const char *p = std::getenv("YOLO26_ENGINE_PATH");
      p != nullptr && p[0] != '\0') {
    return std::string(p);
  }
  return ReadRuntimeString(IsJetsonTarget() ? "jetson_engine_path"
                                           : "pc_engine_path");
}

volatile std::sig_atomic_t g_unfreeze_requested = 0;
void SigintHandler(int) { g_unfreeze_requested = 1; }

const cv::Scalar kGreen{0, 255, 0};
const cv::Scalar kRed{0, 0, 255};
const cv::Scalar kYellow{0, 255, 255};
constexpr int kPopupWidth = 170;
constexpr int kPopupRowHeight = 30;
constexpr int kPopupHeight = 3 * kPopupRowHeight;

void Label(cv::Mat &img, const std::string &text, cv::Point at,
           double scale = 0.46) {
  cv::putText(img, text, at, cv::FONT_HERSHEY_SIMPLEX, scale,
              {0, 0, 0}, 3, cv::LINE_AA);
  cv::putText(img, text, at, cv::FONT_HERSHEY_SIMPLEX, scale,
              {255, 255, 255}, 1, cv::LINE_AA);
}
}  // namespace

class BallPixelPinNode final : public rclcpp::Node {
 public:
  BallPixelPinNode() : rclcpp::Node("ball_pixel_pin_node") {
    const std::string image_topic = declare_parameter<std::string>(
        "image_topic", "/camera/camera/color/image_raw");
    // Explicit ROS override > YOLO26_ENGINE_PATH > PC/Jetson YAML selection.
    const std::string engine_path = declare_parameter<std::string>(
        "engine_path", DefaultYolo26EnginePath());
    ball_class_id_ = declare_parameter<int>("ball_class_id", 1);
    const double conf_thres = declare_parameter<double>("conf_thres", 0.60);
    inference_hz_ = declare_parameter<double>("inference_hz", 15.0);

    if (image_topic.empty() || engine_path.empty())
      throw std::invalid_argument(
          "image_topic or engine_path is empty; verify vision/config/yolo26_runtime.yaml "
          "or pass -p engine_path:=/path/to/best.engine");
    if (ball_class_id_ < 0)
      throw std::invalid_argument("ball_class_id must be >= 0");
    if (!std::isfinite(conf_thres) || conf_thres <= 0.0 || conf_thres > 1.0)
      throw std::invalid_argument("conf_thres must be in (0,1]");
    if (!std::isfinite(inference_hz_) ||
        inference_hz_ <= 0.0 || inference_hz_ > 60.0)
      throw std::invalid_argument("inference_hz must be in (0,60]");

    // Reuse the existing YOLO26 TensorRT engine, not a second implementation.
    yolo_ = std::make_unique<vision::YoloTrtEngine>(
        engine_path, 640, 640, static_cast<float>(conf_thres));

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { ReceiveImage(msg); });

    debug_view_ = std::make_unique<camera_overlay::DebugView>(*this);
    if (debug_view_->Local()) {
      cv::namedWindow(kWindow, cv::WINDOW_AUTOSIZE | cv::WINDOW_GUI_NORMAL);
      cv::setMouseCallback(kWindow, &BallPixelPinNode::MouseCallback, this);
    }
    timer_ = create_wall_timer(std::chrono::milliseconds(20),
                               [this]() { DrawFrame(); });

    RCLCPP_INFO(get_logger(), "Camera: %s", image_topic.c_str());
    RCLCPP_INFO(get_logger(), "Engine: %s | BALL class=%d | conf>=%.2f",
                engine_path.c_str(), ball_class_id_, conf_thres);
    RCLCPP_INFO(get_logger(),
                "Click / Enter / I manual (u,v) | popup 1/2/3 | P polygon | "
                "Ctrl+C live | Q/Esc exit");
  }

  ~BallPixelPinNode() override {
    if (!debug_view_->Local()) return;
    try { cv::destroyWindow(kWindow); }
    catch (const cv::Exception &) {}
  }

 private:
  static constexpr const char *kWindow = "BALL Pixel Pins";
  using Image = sensor_msgs::msg::Image;

  struct Frame {
    cv::Mat image;
    std::int64_t stamp_ns{0};
    std::vector<vision::Detection> balls;
  };
  struct WebContext { std::shared_ptr<const Frame> frame; std::uint64_t revision; };

  struct Popup {
    bool visible{false};
    cv::Point origin{0, 0};
    int hover{-1};
  };

  static cv::Point Center(const vision::Detection &det) {
    return {det.box.x + det.box.width / 2,
            det.box.y + det.box.height / 2};
  }

  static bool Inside(const std::shared_ptr<const Frame> &frame,
                     const cv::Point &p) {
    return frame && p.x >= 0 && p.y >= 0 &&
           p.x < frame->image.cols && p.y < frame->image.rows;
  }

  void ReceiveImage(const Image::ConstSharedPtr &msg) {
    if (frozen_) return;
    try {
      auto frame = std::make_shared<Frame>();
      frame->image = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
      if (frame->image.empty()) return;
      frame->stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
      newest_ = std::move(frame);
    } catch (const std::exception &e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                           "Image conversion error: %s", e.what());
    }
  }

  void InferNewest() {
    if (frozen_ || !newest_ || newest_ == displayed_) return;
    const auto now = std::chrono::steady_clock::now();
    if (inferred_before_ &&
        std::chrono::duration<double>(now - last_inference_).count() <
            1.0 / inference_hz_) return;

    std::vector<vision::Detection> detections;
    bool success = false;
    try {
      success = yolo_->Infer(newest_->image, detections);
    } catch (const std::exception &e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                           "YOLO exception: %s", e.what());
    }
    last_inference_ = std::chrono::steady_clock::now();
    inferred_before_ = true;

    if (!success) {
      status_ = "YOLO inference failed";
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                           "YOLO inference failed");
    } else {
      status_.clear();
    }

    const cv::Rect bounds(0, 0, newest_->image.cols, newest_->image.rows);
    for (const auto &d : detections) {
      if (d.class_id != ball_class_id_ || d.box.width <= 0 ||
          d.box.height <= 0 || !std::isfinite(d.confidence) ||
          (d.box & bounds).empty()) continue;
      newest_->balls.push_back(d);
    }
    displayed_ = newest_;
  }

  void OpenMenu(cv::Point near, cv::Size size) {
    if (size.width < kPopupWidth || size.height < kPopupHeight) {
      status_ = "Image too small for pin menu";
      return;
    }
    popup_.origin = {
        std::clamp(near.x + 10, 0, size.width - kPopupWidth),
        std::clamp(near.y + 10, 0, size.height - kPopupHeight)};
    popup_.hover = -1;
    popup_.visible = true;
  }

  void Select(const std::shared_ptr<const Frame> &frame, cv::Point p) {
    if (!Inside(frame, p)) return;
    frozen_ = frame;                 // Freeze exactly the displayed image.
    selected_ = p;
    entry_active_ = false;
    entry_error_.clear();
    status_.clear();
    OpenMenu(p, frame->image.size());
    RCLCPP_INFO(get_logger(), "Selected pixel (u,v)=(%d,%d)", p.x, p.y);
  }

  void SelectBall() {
    const auto frame = frozen_ ? frozen_ : displayed_;
    if (!frame || frame->balls.empty()) {
      status_ = "No YOLO BALL detected in the displayed frame";
      return;
    }
    const auto best = std::max_element(
        frame->balls.begin(), frame->balls.end(),
        [](const vision::Detection &a, const vision::Detection &b) {
          return a.confidence < b.confidence;
        });
    Select(frame, Center(*best));
  }

  // Manual entry captures the image at the moment I is pressed. This prevents
  // selecting a point on an image that changes while the user is typing.
  void BeginEntry() {
    const auto frame = frozen_ ? frozen_ : displayed_;
    if (!frame) {
      status_ = "No camera frame yet; cannot enter a coordinate";
      return;
    }
    entry_was_live_ = !frozen_;
    frozen_ = frame;
    popup_.visible = false;
    entry_active_ = true;
    entry_text_.clear();
    entry_error_.clear();
    status_.clear();
  }

  void CancelEntry() {
    if (!entry_active_) return;
    entry_active_ = false;
    entry_text_.clear();
    entry_error_.clear();
    if (entry_was_live_) {
      frozen_.reset();
      displayed_.reset();
      newest_.reset();
      inferred_before_ = false;
    }
  }

  void SubmitEntry() {
    if (!entry_active_) return;
    // Accept only two unsigned decimal integers separated by one comma.
    const auto comma = entry_text_.find(',');
    if (comma == std::string::npos || comma == 0 ||
        comma + 1 == entry_text_.size() ||
        entry_text_.find(',', comma + 1) != std::string::npos) {
      entry_error_ = "Use u,v  (example: 320,240)";
      return;
    }
    try {
      const int u = std::stoi(entry_text_.substr(0, comma));
      const int v = std::stoi(entry_text_.substr(comma + 1));
      const cv::Point p(u, v);
      if (!Inside(frozen_, p)) {
        char msg[128];
        std::snprintf(msg, sizeof(msg),
                      "Out of range: 0<=u<%d, 0<=v<%d",
                      frozen_->image.cols, frozen_->image.rows);
        entry_error_ = msg;
        return;
      }
      const auto frame = frozen_;
      Select(frame, p);
    } catch (const std::exception &) {
      entry_error_ = "Invalid integer; example: 320,240";
    }
  }

  void ApplyMenu(int row) {
    if (!selected_) return;
    if (row == 0) {
      green_pins_.push_back(*selected_);
      RCLCPP_INFO(get_logger(), "GREEN pin: (%d,%d)",
                  selected_->x, selected_->y);
    } else if (row == 1) {
      red_pins_.push_back(*selected_);
      RCLCPP_INFO(get_logger(), "RED pin: (%d,%d)",
                  selected_->x, selected_->y);
    } else if (row == 2) {
      RCLCPP_INFO(get_logger(), "Pin canceled (frame remains frozen)");
    } else {
      return;
    }
    popup_.visible = false;
  }

  int MenuRow(cv::Point p) const {
    if (!popup_.visible) return -1;
    const int x = p.x - popup_.origin.x;
    const int y = p.y - popup_.origin.y;
    if (x < 0 || x >= kPopupWidth || y < 0 || y >= kPopupHeight)
      return -1;
    return y / kPopupRowHeight;
  }

  static void MouseCallback(int event, int x, int y, int, void *userdata) {
    static_cast<BallPixelPinNode *>(userdata)->OnMouse(event, {x, y});
  }

  void OnMouse(int event, cv::Point p) {
    if (entry_active_) return; // Typing coordinates has priority over mouse.
    if (event==cv::EVENT_LBUTTONDOWN || event==cv::EVENT_RBUTTONDOWN) ++web_revision_;
    if (event == cv::EVENT_MOUSEMOVE) {
      popup_.hover = MenuRow(p);
    } else if (event == cv::EVENT_LBUTTONDOWN) {
      const int row = MenuRow(p);
      if (row >= 0) {
        ApplyMenu(row);
      } else {
        const auto frame = frozen_ ? frozen_ : displayed_;
        if (Inside(frame, p)) Select(frame, p);
      }
    } else if (event == cv::EVENT_RBUTTONDOWN) {
      const auto frame = frozen_ ? frozen_ : displayed_;
      if (!Inside(frame, p)) return;
      if (!selected_) Select(frame, p);
      else OpenMenu(p, frame->image.size());
    }
  }

  void Unfreeze() {
    frozen_.reset();
    displayed_.reset();
    newest_.reset();
    selected_.reset();
    popup_.visible = false;
    entry_active_ = false;
    entry_text_.clear();
    entry_error_.clear();
    status_.clear();
    inferred_before_ = false;
    RCLCPP_INFO(get_logger(), "LIVE resumed; pins retained");
  }

  void Polygon(cv::Mat &canvas) const {
    if (!polygon_visible_ || green_pins_.size() < 3) return;
    const std::vector<std::vector<cv::Point>> polygons{green_pins_};
    cv::Mat tinted = canvas.clone();
    cv::fillPoly(tinted, polygons, kGreen, cv::LINE_AA);
    // 70% transparent = 30% green + 70% unchanged image.
    cv::addWeighted(tinted, 0.30, canvas, 0.70, 0.0, canvas);
    cv::polylines(canvas, polygons, true, kGreen, 1, cv::LINE_AA);
  }

  void Detections(cv::Mat &canvas, const Frame &frame) const {
    const cv::Rect bounds(0, 0, canvas.cols, canvas.rows);
    for (const auto &d : frame.balls) {
      const cv::Rect rect = d.box & bounds;
      if (rect.empty()) continue;
      const cv::Point p = Center(d);
      cv::rectangle(canvas, rect, {0, 200, 255}, 2, cv::LINE_AA);
      cv::drawMarker(canvas, p, {255, 0, 255}, cv::MARKER_CROSS,
                     12, 1, cv::LINE_AA);
      char label[96];
      std::snprintf(label, sizeof(label), "BALL %.2f (%d,%d)",
                    d.confidence, p.x, p.y);
      Label(canvas, label,
            {std::clamp(rect.x, 0, std::max(0, canvas.cols - 160)),
             std::max(14, rect.y - 5)}, 0.42);
    }
  }

  void Pins(cv::Mat &canvas) const {
    for (const auto &p : green_pins_) {
      cv::circle(canvas, p, 4, {0, 0, 0}, cv::FILLED, cv::LINE_AA);
      cv::circle(canvas, p, 3, kGreen, cv::FILLED, cv::LINE_AA);
    }
    for (const auto &p : red_pins_) {
      cv::circle(canvas, p, 4, {0, 0, 0}, cv::FILLED, cv::LINE_AA);
      cv::circle(canvas, p, 3, kRed, cv::FILLED, cv::LINE_AA);
    }
  }

  void DrawMenu(cv::Mat &canvas) const {
    if (!popup_.visible || entry_active_) return;
    const char *names[] = {"Green pin  [1]", "Red pin    [2]", "Cancel     [3]"};
    const cv::Scalar colors[] = {kGreen, kRed, {185, 185, 185}};
    cv::rectangle(canvas, {popup_.origin.x, popup_.origin.y,
                           kPopupWidth, kPopupHeight},
                  {40, 40, 40}, cv::FILLED);
    cv::rectangle(canvas, {popup_.origin.x, popup_.origin.y,
                           kPopupWidth, kPopupHeight}, {230, 230, 230}, 1);
    for (int row = 0; row < 3; ++row) {
      const int top = popup_.origin.y + row * kPopupRowHeight;
      if (popup_.hover == row) {
        cv::rectangle(canvas, {popup_.origin.x + 1, top + 1,
                               kPopupWidth - 2, kPopupRowHeight - 1},
                      {90, 90, 90}, cv::FILLED);
      }
      cv::circle(canvas, {popup_.origin.x + 14, top + 15},
                 4, colors[row], cv::FILLED, cv::LINE_AA);
      cv::putText(canvas, names[row], {popup_.origin.x + 29, top + 20},
                  cv::FONT_HERSHEY_SIMPLEX, 0.45, {245, 245, 245},
                  1, cv::LINE_AA);
    }
  }

  void DrawEntry(cv::Mat &canvas) const {
    if (!entry_active_) return;
    const int w = std::min(440, std::max(1, canvas.cols - 16));
    const int h = std::min(155, std::max(1, canvas.rows - 16));
    const int x = (canvas.cols - w) / 2;
    const int y = (canvas.rows - h) / 2;
    const cv::Rect box{x, y, w, h};
    cv::rectangle(canvas, box, {35, 35, 35}, cv::FILLED);
    cv::rectangle(canvas, box, {235, 235, 235}, 2);
    cv::putText(canvas, "INPUT PIXEL (u,v)", {x + 14, y + 29},
                cv::FONT_HERSHEY_SIMPLEX, 0.61, {255, 255, 255},
                1, cv::LINE_AA);
    cv::rectangle(canvas, {x + 13, y + 43, w - 26, 38},
                  {70, 70, 70}, cv::FILLED);
    cv::putText(canvas, entry_text_ + "_", {x + 23, y + 70},
                cv::FONT_HERSHEY_SIMPLEX, 0.68, {0, 255, 255},
                2, cv::LINE_AA);
    cv::putText(canvas, "Example: 320,240 | Enter: OK | Esc: Cancel",
                {x + 14, y + 102}, cv::FONT_HERSHEY_SIMPLEX,
                0.40, {235, 235, 235}, 1, cv::LINE_AA);
    if (!entry_error_.empty()) {
      cv::putText(canvas, entry_error_, {x + 14, y + 132},
                  cv::FONT_HERSHEY_SIMPLEX, 0.42, {80, 120, 255},
                  1, cv::LINE_AA);
    }
  }

  void HandleKey(int key) {
    if (key < 0) return;
    const int c = key & 0xff;
    if (!entry_active_ || c==3 || c==27 || c==10 || c==13) ++web_revision_;

    // Ctrl+C should unfreeze even if coordinate entry is active.
    if (c == 3) {
      Unfreeze();
      return;
    }

    if (entry_active_) {
      if (c == 27) {
        CancelEntry();
      } else if (c == 10 || c == 13) {
        SubmitEntry();
      } else if (c == 8 || c == 127) {
        if (!entry_text_.empty()) entry_text_.pop_back();
        entry_error_.clear();
      } else if (c >= '0' && c <= '9' && entry_text_.size() < 15) {
        entry_text_.push_back(static_cast<char>(c));
        entry_error_.clear();
      } else if ((c == ',' || c == ' ') &&
                 entry_text_.find(',') == std::string::npos &&
                 !entry_text_.empty() && entry_text_.size() < 15) {
        entry_text_.push_back(',');
        entry_error_.clear();
      }
      return;
    }

    if (c == 27 || c == 'q' || c == 'Q') {
      rclcpp::shutdown();
    } else if (c == 'i' || c == 'I') {
      BeginEntry();
    } else if (c == 10 || c == 13) {
      SelectBall();
    } else if (c == 'p' || c == 'P') {
      if (!polygon_visible_ && green_pins_.size() < 3)
        status_ = "Need 3 GREEN pins to draw a polygon";
      else {
        polygon_visible_ = !polygon_visible_;
        status_.clear();
      }
    } else if (popup_.visible && c >= '1' && c <= '3') {
      ApplyMenu(c - '1');
    }
  }

  void DrawFrame() {
    try {
      for (const auto &event : debug_view_->Inputs()) {
        const auto &context=std::any_cast<const WebContext&>(event.context);
        if (context.revision!=web_revision_) continue;
        const auto saved=displayed_;
        if (!frozen_) displayed_=context.frame;
        const auto &input=event.input;
        if (input.kind==vision::DebugMjpegServer::Input::Kind::Click)
          OnMouse(input.button==2 ? cv::EVENT_RBUTTONDOWN : cv::EVENT_LBUTTONDOWN,{input.x,input.y});
        else HandleKey(input.key);
        if (displayed_==context.frame) displayed_=saved;
      }
      if (g_unfreeze_requested) {
        g_unfreeze_requested = 0;
        Unfreeze();
      }
      if (!frozen_) InferNewest();
      const auto frame = frozen_ ? frozen_ : displayed_;
      cv::Mat canvas = frame ? frame->image.clone()
                             : cv::Mat::zeros(480, 640, CV_8UC3);

      Polygon(canvas);
      if (frame) Detections(canvas, *frame);
      Pins(canvas);
      if (selected_ && !entry_active_) {
        cv::drawMarker(canvas, *selected_, kYellow,
                       cv::MARKER_CROSS, 16, 2, cv::LINE_AA);
      }

      char info[260];
      std::snprintf(info, sizeof(info),
                    "%s | BALL:%zu | GREEN:%zu RED:%zu | POLY:%s",
                    frozen_ ? "FROZEN" : "LIVE",
                    frame ? frame->balls.size() : std::size_t(0),
                    green_pins_.size(), red_pins_.size(),
                    polygon_visible_ ? "ON" : "OFF");
      Label(canvas, info, {9, 20}, 0.46);

      if (selected_) {
        std::snprintf(info, sizeof(info),
                      "SELECTED (u,v) = (%d,%d)", selected_->x, selected_->y);
        Label(canvas, info, {9, 42}, 0.48);
      } else if (!frame) {
        Label(canvas, "Waiting for camera + YOLO...", {9, 42});
      }
      if (!status_.empty() && !entry_active_)
        Label(canvas, status_, {9, 64}, 0.43);

      Label(canvas,
            "CLICK / ENTER ball / I input / P polygon / Ctrl+C live / Q quit",
            {8, canvas.rows - 10}, 0.37);

      DrawMenu(canvas);
      DrawEntry(canvas);
      debug_view_->Present(canvas, WebContext{frame,web_revision_}, web_revision_);
      if (debug_view_->Local()) {
        cv::imshow(kWindow, canvas);
        HandleKey(cv::waitKeyEx(1));
      }
    } catch (const cv::Exception &e) {
      RCLCPP_ERROR(get_logger(), "OpenCV window error: %s", e.what());
      rclcpp::shutdown();
    }
  }

  std::unique_ptr<camera_overlay::DebugView> debug_view_;
  std::uint64_t web_revision_{0};
  rclcpp::Subscription<Image>::SharedPtr image_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<vision::YoloTrtEngine> yolo_;
  std::shared_ptr<Frame> newest_;
  std::shared_ptr<const Frame> displayed_, frozen_;
  std::optional<cv::Point> selected_;
  Popup popup_;
  std::vector<cv::Point> green_pins_, red_pins_;
  bool polygon_visible_{false};
  int ball_class_id_{1};
  double inference_hz_{15.0};
  bool inferred_before_{false};
  std::chrono::steady_clock::time_point last_inference_{};
  std::string status_;

  bool entry_active_{false};
  bool entry_was_live_{false};
  std::string entry_text_;
  std::string entry_error_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv, rclcpp::InitOptions(),
               rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, SigintHandler);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<BallPixelPinNode>());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("ball_pixel_pin_node"), "%s", e.what());
    result = 1;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return result;
}
