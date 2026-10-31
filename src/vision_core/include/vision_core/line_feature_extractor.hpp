#pragma once

#include <vector>

#include "vision_core/types.hpp"

namespace vision_core {

// A separate diagnostic path: it never edits normal LINE RANSAC inputs or O/H.
struct CurveSearchConfig {
  bool enabled{true};
  int stable_window{10}, stable_min_hits{7};
  double min_abs_angle_deg{15.0};
  double residual_threshold_px{20.0};
  double max_fit_rmse_px{12.0};
  double min_pair_v_gap_px{20.0};
  double min_segment_v_span_px{30.0};
  double min_total_v_span_px{80.0};
  double max_join_gap_px{70.0};
};
struct CurveEvidence {
  bool valid{false}; // Both independent segments passed geometry/fit checks.
  int direction{0}; // +1 right, -1 left, 0 straight or below threshold.
  double angle_rad{0.0};
  int points{0}, near_inliers{0}, far_inliers{0};
};
CurveEvidence ComputeCurveEvidence(const std::vector<Point2>& original_centers,
    int image_width, int image_height, const CurveSearchConfig& config);

Features ComputeLineFeatures(const std::vector<Point2> &points, int image_width,
                             int image_height, const FeatureConfig &config,
                             int minimum_input_points = 4);

} // namespace vision_core
