#include "vision/pixel_inspection.hpp"

#include <cassert>
#include <cmath>
#include <limits>
#include <vector>

int main() {
  const std::vector<vision::Detection> detections = {
      {cv::Rect(10, 20, 40, 30), 0.75f, 1},
      {cv::Rect(120, 200, 80, 100), 0.94f, 1},
      {cv::Rect(0, 0, 50, 50), 0.99f, 0},
      {cv::Rect(4, 6, 0, 30), 1.0f, 1},
      {cv::Rect(2, 3, 20, 20),
       std::numeric_limits<float>::quiet_NaN(), 1},
  };
  const auto selected =
      vision::SelectHighestConfidenceBall(detections, 1, 0.60f);
  assert(selected.has_value());
  assert(selected->confidence == 0.94f);
  assert((vision::BoxCenter(selected->box) == cv::Point(160, 250)));

  const auto stricter =
      vision::SelectHighestConfidenceBall(detections, 1, 0.95f);
  assert(!stricter);

  const auto absent =
      vision::SelectHighestConfidenceBall(detections, 2, 0.50f);
  assert(!absent);

  const auto odd_center = vision::BoxCenter(cv::Rect(5, 7, 3, 5));
  assert((odd_center == cv::Point(6, 9)));
  return 0;
}
