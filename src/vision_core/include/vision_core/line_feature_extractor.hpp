#pragma once

#include <vector>

#include "vision_core/types.hpp"

namespace vision_core {

Features ComputeLineFeatures(const std::vector<Point2> &points, int image_width,
                             int image_height, const FeatureConfig &config,
                             int minimum_input_points = 4);

// Fits the lower four LINE-RANSAC survivors; fits upper original points plus
// the highest lower survivor (shared anchor). Never changes ordinary LINE O/H.
CurveEvidence ComputeCurveEvidence(const std::vector<Point2>& points,
                                    int image_width, int image_height,
                                    const FeatureConfig& line_config,
                                    const CurveSearchConfig& curve_config);

} // namespace vision_core
