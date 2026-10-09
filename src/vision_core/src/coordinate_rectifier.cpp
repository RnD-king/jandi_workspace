// 처리 순서: [점선 분기 1단계] 카메라/IMU 입력 뒤, 특징 계산 전에 호출한다.
// 역할: roll/pitch와 카메라 내부 파라미터로 픽셀 점(Point2)을 보정한다.
// 값 전달: 호출자가 점 목록을 인자로 주고, 보정된 새 점 목록을 반환받는다.
// 다음 단계: 반환값은 line_feature_extractor.cpp의 ComputeLineFeatures()로 간다.

#include "vision_core/coordinate_rectifier.hpp"

#include <cmath>

namespace vision_core {

std::vector<Point2> RectifyPixelPoints(const std::vector<Point2> &points,
                                       const Intrinsics &k, double roll_rad,
                                       double pitch_rad) {
  std::vector<Point2> output;
  output.reserve(points.size());
  const double cr = std::cos(-roll_rad);
  const double sr = std::sin(-roll_rad);
  const double cp = std::cos(-pitch_rad);
  const double sp = std::sin(-pitch_rad);
  constexpr double eps = 1e-9;

  for (const Point2 &point : points) {
    const double x = (point.u - k.cx) / (k.fx + eps);
    const double y = (point.v - k.cy) / (k.fy + eps);
    const double x1 = cr * x - sr * y;
    const double y1 = sr * x + cr * y;
    const double x2 = x1;
    const double y2 = cp * y1 - sp;
    const double z2 = sp * y1 + cp;
    if (std::abs(z2) < 1e-6) {
      output.push_back(point);
    } else {
      output.push_back({k.fx * (x2 / z2) + k.cx,
                        k.fy * (y2 / z2) + k.cy});
    }
  }
  return output;
}

} // namespace vision_core
