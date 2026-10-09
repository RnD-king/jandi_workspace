#pragma once

#include <vector>

#include "vision_core/types.hpp"

namespace vision_core {

Features ComputeLineFeatures(const std::vector<Point2> &points, int image_width,
                             int image_height, const FeatureConfig &config);

} // namespace vision_core
