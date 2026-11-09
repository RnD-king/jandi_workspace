#pragma once

#include <cmath>
#include <cstdint>
#include <optional>

namespace vision {

// Capture the first valid CameraInfo for the actual RGB image dimensions.
// A later CameraInfo or resolution change must not silently retune geometry.
struct CameraInfoCalibration {
  std::uint32_t width{0}, height{0};
  double fx{0}, fy{0}, cx{0}, cy{0};
};

class CameraInfoLatch {
public:
  void ObserveImage(std::uint32_t width, std::uint32_t height) {
    if (width == 0 || height == 0) return;
    image_width_ = width;
    image_height_ = height;
    TryLatch();
  }

  void ObserveInfo(CameraInfoCalibration info) {
    if (latched_ || !Valid(info)) return;
    pending_ = info;
    TryLatch();
  }

  bool Ready() const { return latched_.has_value(); }
  bool MatchesImage(std::uint32_t width, std::uint32_t height) const {
    return latched_ && latched_->width == width && latched_->height == height;
  }
  const CameraInfoCalibration& Calibration() const { return *latched_; }

private:
  static bool Valid(const CameraInfoCalibration& info) {
    return info.width > 0 && info.height > 0 &&
        std::isfinite(info.fx) && info.fx > 0.0 &&
        std::isfinite(info.fy) && info.fy > 0.0 &&
        std::isfinite(info.cx) && std::isfinite(info.cy);
  }

  void TryLatch() {
    if (latched_ || !pending_ || image_width_ == 0 || image_height_ == 0)
      return;
    if (pending_->width == image_width_ && pending_->height == image_height_) {
      latched_ = *pending_;
      pending_.reset();
    }
  }

  std::uint32_t image_width_{0}, image_height_{0};
  std::optional<CameraInfoCalibration> pending_;
  std::optional<CameraInfoCalibration> latched_;
};

}  // namespace vision
