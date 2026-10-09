#pragma once

#include <vector>

#include "vision_core/types.hpp"

namespace vision_core {

std::vector<Point2> RectifyPixelPoints(const std::vector<Point2> &points,
                                       const Intrinsics &intrinsics,
                                       double roll_rad, double pitch_rad);

} // namespace vision_core
