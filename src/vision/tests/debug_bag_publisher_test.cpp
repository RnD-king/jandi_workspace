#include "vision/debug_bag_publisher.hpp"

#include <array>
#include <iostream>
#include <stdexcept>

namespace {
void Check(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}

void TestPublisher() {
  auto disabled_node = std::make_shared<rclcpp::Node>("disabled_debug_test");
  vision::DebugBagPublisher disabled(*disabled_node);
  Check(!disabled_node->get_parameter("rosbag").as_bool(), "rosbag must default to false");
  Check(!disabled.RequestFrame(), "disabled publisher requested a render");

  auto options = rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("rosbag", true), rclcpp::Parameter("rosbag_fps", 1.0)});
  auto node = std::make_shared<rclcpp::Node>("enabled_debug_test", options);
  vision::DebugBagPublisher publisher(*node);
  Check(!publisher.RequestFrame(), "no subscribers must mean no debug rendering");

  auto listener = std::make_shared<rclcpp::Node>("debug_listener_test");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(listener);
  std::array<sensor_msgs::msg::Image::ConstSharedPtr, 3> received{};
  std::array<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr, 3> subscriptions;
  const std::array<const char *, 3> topics{
      "/jandi_vision/debug/image", "/jandi_vision/debug/panel", "/jandi_vision/debug/yolo"};
  for (std::size_t i = 0; i < topics.size(); ++i) {
    subscriptions[i] = listener->create_subscription<sensor_msgs::msg::Image>(
        topics[i], rclcpp::SensorDataQoS(),
        [&, i](sensor_msgs::msg::Image::ConstSharedPtr message) { received[i] = message; });
  }
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < deadline &&
         (subscriptions[0]->get_publisher_count() == 0 ||
          subscriptions[1]->get_publisher_count() == 0 ||
          subscriptions[2]->get_publisher_count() == 0)) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  const auto request = publisher.RequestFrame();
  Check(request.image && request.panel && request.yolo, "subscribed views were not requested");
  Check(!publisher.RequestFrame(), "debug fps limit did not suppress the next frame");
  std_msgs::msg::Header header;
  header.stamp.sec = 123;
  header.stamp.nanosec = 456;
  header.frame_id = "source_camera_frame";
  {
    // Submit owns the rendered buffers through cv::Mat references, even after
    // this callback scope ends. No GUI or camera is needed for recording.
    cv::Mat image(8, 12, CV_8UC3, cv::Scalar(11, 22, 33));
    cv::Mat panel(5, 10, CV_8UC3, cv::Scalar(44, 55, 66));
    cv::Mat yolo(8, 12, CV_8UC3, cv::Scalar(77, 88, 99));
    publisher.Submit(request, header, image, panel, yolo);
  }
  const auto receive_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < receive_deadline &&
         (!received[0] || !received[1] || !received[2])) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  for (const auto &message : received) {
    Check(message != nullptr, "worker did not deliver a subscribed debug view");
    Check(message->header == header, "debug message did not preserve source frame header");
    Check(message->encoding == "bgr8", "unexpected debug image encoding");
  }
  Check(received[0]->width == 12 && received[0]->data[0] == 11, "camera debug pixels corrupted");
  Check(received[1]->height == 5 && received[1]->data[0] == 44, "state panel pixels corrupted");
  Check(received[2]->data[0] == 77, "YOLO view pixels corrupted");
}
}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int status = 0;
  try {
    TestPublisher();
    std::cout << "Debug bag publisher tests passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
