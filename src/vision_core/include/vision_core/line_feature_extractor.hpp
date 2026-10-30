#pragma once

#include <vector>

#include "vision_core/types.hpp"

namespace vision_core {

Features ComputeLineFeatures(const std::vector<Point2> &points, int image_width,
                             int image_height, const FeatureConfig &config,
                             int minimum_input_points = 4);

} // namespace vision_core
