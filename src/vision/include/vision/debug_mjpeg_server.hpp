#pragma once

#include <cstdint>
#include <memory>
#include <opencv2/core.hpp>

namespace vision {

// Linux HTTP transport, independent of ROS and robot state. Only the caller of
// RequestFrame/Submit accesses the producer rate limiter. Submitted images must
// own their pixels and remain immutable (RenderMissionDebug supplies these).
class DebugMjpegServer {
public:
  struct Stats {
    std::uint64_t encoded_frames{}, sent_frames{}, encoded_bytes{};
    std::uint64_t encode_errors{}, dropped_submissions{}, slow_disconnects{};
    double last_encode_ms{}, mean_encode_ms{};
    unsigned clients{};
  };

  // Port 0 is for isolated loopback tests; the ROS node always uses 8080.
  explicit DebugMjpegServer(std::uint16_t port = 8080,
                            const char *bind_address = "0.0.0.0");
  ~DebugMjpegServer();
  DebugMjpegServer(const DebugMjpegServer &) = delete;
  DebugMjpegServer &operator=(const DebugMjpegServer &) = delete;

  bool RequestFrame();
  // No pixel copy, encoding, socket I/O or blocking mutex acquisition here.
  void Submit(const cv::Mat &image);
  Stats GetStats() const;
  std::uint16_t Port() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace vision
