// 처리 순서: [점선 분기 2단계] coordinate_rectifier 뒤에 호출한다.
// 역할: 보정된 점선 중심점들로부터 P2P용 LineGuide를 만든다.

#include "vision_core/line_feature_extractor.hpp"

#include <algorithm>
#include <cmath>

namespace vision_core {
namespace {
double Clamp(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

struct LineFit {
  bool valid{false};
  double a{0.0};
  double b{0.0};
};

LineFit FitLine(const std::vector<Point2> &points, std::size_t begin,
                std::size_t end) {
  if (end <= begin + 1 || end > points.size())
    return {};

  double sum_v = 0.0;
  double sum_u = 0.0;
  for (std::size_t index = begin; index < end; ++index) {
    sum_v += points[index].v;
    sum_u += points[index].u;
  }
  const double inv_n = 1.0 / static_cast<double>(end - begin);
  const double mean_v = sum_v * inv_n;
  const double mean_u = sum_u * inv_n;

  double var_v = 0.0;
  double cov_vu = 0.0;
  for (std::size_t index = begin; index < end; ++index) {
    const double dv = points[index].v - mean_v;
    var_v += dv * dv;
    cov_vu += dv * (points[index].u - mean_u);
  }
  if (std::abs(var_v) <= 1e-9)
    return {};

  const double a = cov_vu / var_v;
  return {true, a, mean_u - a * mean_v};
}

double MeanV(const std::vector<Point2> &points, std::size_t begin,
             std::size_t end) {
  if (end <= begin || end > points.size())
    return 0.0;
  double sum = 0.0;
  for (std::size_t index = begin; index < end; ++index) {
    sum += points[index].v;
  }
  return sum / static_cast<double>(end - begin);
}

double FitRmsePx(const std::vector<Point2> &points, std::size_t begin,
                 std::size_t end, const LineFit &fit) {
  if (!fit.valid || end <= begin || end > points.size())
    return 0.0;
  double squared_error = 0.0;
  for (std::size_t index = begin; index < end; ++index) {
    const double residual = points[index].u - (fit.a * points[index].v + fit.b);
    squared_error += residual * residual;
  }
  return std::sqrt(squared_error / static_cast<double>(end - begin));
}

double WrapAngle(double angle) {
  constexpr double kPi = 3.14159265358979323846;
  while (angle > kPi)
    angle -= 2.0 * kPi;
  while (angle < -kPi)
    angle += 2.0 * kPi;
  return angle;
}

LineGuide ComputeLineGuide(const std::vector<Point2> &points, double cx,
                           double denom, const FeatureConfig &cfg) {
  LineGuide guide;
  if (cfg.guide_min_points < 3 ||
      points.size() < static_cast<std::size_t>(cfg.guide_min_points))
    return guide;

  const int desired_near_points =
      std::max(cfg.guide_min_points, cfg.curve_local_fit_points);
  const std::size_t local_count = std::min(
      static_cast<std::size_t>(desired_near_points),
      points.size());
  const std::size_t far_begin = points.size() - local_count;
  const LineFit near_fit = FitLine(points, 0, local_count);
  if (!near_fit.valid) return guide;

  const double near_reference_v = MeanV(points, 0, local_count);
  const double near_reference_u = near_fit.a * near_reference_v + near_fit.b;
  guide.offset = (near_reference_u - cx) / denom;
  // FitLine의 a=du/dv이고 영상에서 진행 방향은 v 감소 방향이다.
  // 따라서 -atan(a)가 진행 방향 기준 image-right(+) 각도다.
  guide.heading_rad = -std::atan(near_fit.a);
  guide.valid = std::isfinite(guide.offset) &&
                std::isfinite(guide.heading_rad);
  if (!guide.valid) return guide;

  const double full_v_span = points.front().v - points.back().v;
  const bool curvature_geometry_valid =
      static_cast<int>(points.size()) >= cfg.curve_min_points &&
      full_v_span >= cfg.curve_min_v_span_px;
  const LineFit far_fit = curvature_geometry_valid
                              ? FitLine(points, far_begin, points.size())
                              : LineFit{};
  if (far_fit.valid) {
    const double far_heading_rad = -std::atan(far_fit.a);
    const double curvature = WrapAngle(far_heading_rad - guide.heading_rad);
    if (std::isfinite(curvature)) {
      guide.curvature_rad = curvature;
      guide.curvature_valid = true;
    }
  }

  // confidence는 정상 P2P 제어에 쓰는 가까운 O/H fit 품질이다. 먼 점군
  // curvature fit의 성공/실패나 잔차와 결합하지 않는다.
  const double point_confidence =
      Clamp(static_cast<double>(local_count) /
                static_cast<double>(desired_near_points),
            0.0, 1.0);
  const double v_span = points.front().v - points[local_count - 1].v;
  const double span_confidence =
      Clamp(v_span / std::max(1.0, cfg.curve_min_v_span_px), 0.0, 1.0);
  const double near_rmse = FitRmsePx(points, 0, local_count, near_fit);
  const double fit_confidence =
      1.0 - Clamp(near_rmse /
                      std::max(1.0, cfg.guide_fit_rmse_full_scale_px),
                  0.0, 1.0);
  guide.confidence =
      Clamp(point_confidence * span_confidence * fit_confidence, 0.0, 1.0);
  return guide;
}
} // namespace

Features ComputeLineFeatures(const std::vector<Point2> &input, int image_width,
                             int image_height, const FeatureConfig &cfg) {
  Features f{};
  if (image_width <= 1 || image_height <= 1) return f;

  std::vector<Point2> points;
  points.reserve(input.size());
  for (const Point2 &point : input) {
    if (std::isfinite(point.u) && std::isfinite(point.v)) {
      points.push_back({Clamp(point.u, 0.0, image_width - 1.0),
                        Clamp(point.v, 0.0, image_height - 1.0)});
    }
  }
  std::sort(points.begin(), points.end(), [](const Point2 &a, const Point2 &b) {
    return (a.v == b.v) ? (a.u < b.u) : (a.v > b.v);
  });
  points.resize(std::min(static_cast<std::size_t>(std::max(1, cfg.max_centers)),
                         points.size()));

  f.n_visible = static_cast<double>(points.size());
  if (points.empty()) return f;

  const double cx = cfg.image_center_u >= 0.0
                        ? cfg.image_center_u
                        : 0.5 * static_cast<double>(image_width);
  // 중심점이 principal point로 이동해도 좌/우 정규화 크기가 달라지지 않도록
  // 영상 반폭을 분모로 사용한다.
  const double denom = std::max(0.5 * static_cast<double>(image_width), 1.0);
  f.guide = ComputeLineGuide(points, cx, denom, cfg);
  return f;
}

} // namespace vision_core
