#pragma once
#include <cmath>
#include <optional>
#include "vision_core/types.hpp"

namespace vision_core {
struct ScanBodyCorrectionConfig {
  double measured_turn_deg{12.0}; // Calibration only; executor request stays 15 degrees.
  double near_angle_deg{30.0}; // <=30: two JSONs, >30: three JSONs.
  int max_attempts{3};
};
struct ScanBodyCorrection { bool left{false}; int turns{0}; };
// Approximate bearing, intentionally without pitch compensation. Robot head
// yaw is left-positive; image u is right-positive.
inline std::optional<ScanBodyCorrection> ObjectScanCorrection(
    double head_yaw_deg, std::optional<double> mean_u, const Intrinsics& camera,
    const ScanBodyCorrectionConfig& config) {
  if (!mean_u || !std::isfinite(*mean_u) || !std::isfinite(camera.fx) ||
      camera.fx<=0 || !std::isfinite(camera.cx) || !std::isfinite(head_yaw_deg))
    return std::nullopt;
  const double angle=head_yaw_deg-std::atan((*mean_u-camera.cx)/camera.fx)*
      (180.0/3.14159265358979323846);
  const double magnitude=std::abs(angle);
  return ScanBodyCorrection{angle>0, magnitude<=config.measured_turn_deg*.5 ? 0 :
      magnitude<=config.near_angle_deg ? 2 : 3};
}
inline bool ScanFirstLeft(std::optional<double> u, double center, double margin,
                          bool fallback_left) {
  return u && std::isfinite(*u) && std::abs(*u-center)>margin ? *u<center : fallback_left;
}
} // namespace vision_core
