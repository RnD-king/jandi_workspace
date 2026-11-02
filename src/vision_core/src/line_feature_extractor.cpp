// 처리 순서: [점선 분기 2단계] coordinate_rectifier 뒤에 호출한다.
// 역할: 보정된 점선 중심점들로부터 P2P용 LineGuide를 만든다.

#include "vision_core/line_feature_extractor.hpp"
#include "vision_core/object_approach.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace vision_core {
namespace {
constexpr double kMinVSpanPx = 5.0;
constexpr std::size_t kMaxRansacPoints = 8;
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
  if (points[begin].v - points[end - 1].v <= kMinVSpanPx)
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

// RANSAC chooses a reference only for ranking suspicious points. Its line
// is never passed to the O/H calculation, and no consensus refit is needed.
void RemoveRansacOutliers(std::vector<Point2> &points, const FeatureConfig &cfg,
                         const std::vector<std::size_t> &source_indices,
                         std::vector<std::size_t> &removed_indices) {
  if (!cfg.ransac_enabled || cfg.ransac_max_outliers == 0 ||
      points.size() < static_cast<std::size_t>(cfg.ransac_min_input_points))
    return;
  LineFit best;
  int best_hits = -1;
  double best_error = std::numeric_limits<double>::infinity();
  unsigned best_near_mask = 0;
  constexpr double tie_epsilon = 1e-9;
  for (std::size_t first = 0; first < points.size(); ++first) {
    for (std::size_t second = first + 1; second < points.size(); ++second) {
      const double dv = points[first].v - points[second].v;
      // Reject this pair only, never either point. The five-pixel guard
      // remains in force even if the configurable gap is tuned smaller.
      if (dv <= kMinVSpanPx || dv < cfg.ransac_min_sample_v_gap_px)
        continue;
      const double a = (points[first].u - points[second].u) / dv;
      const LineFit candidate{true, a, points[first].u - a * points[first].v};
      if (!std::isfinite(candidate.a) || !std::isfinite(candidate.b)) continue;
      int hits = 0;
      double error = 0.0;
      unsigned near_mask = 0;
      for (std::size_t index = 0; index < points.size(); ++index) {
        const double residual = std::abs(points[index].u -
            (candidate.a * points[index].v + candidate.b));
        if (residual <= cfg.ransac_residual_threshold_px) {
          ++hits;
          error += residual * residual;
          near_mask |= 1U << (kMaxRansacPoints - 1 - index);
        }
      }
      if (hits > best_hits ||
          (hits == best_hits && (error < best_error - tie_epsilon ||
           (std::abs(error - best_error) <= tie_epsilon && near_mask > best_near_mask)))) {
        best = candidate;
        best_hits = hits;
        best_error = error;
        best_near_mask = near_mask;
      }
    }
  }
  // No eligible pair is not a frame failure: retain the original points.
  if (!best.valid) return;
  std::vector<std::pair<double, std::size_t>> suspects;
  for (std::size_t index = 0; index < points.size(); ++index) {
    const double residual = std::abs(points[index].u - (best.a * points[index].v + best.b));
    if (residual > cfg.ransac_residual_threshold_px)
      suspects.emplace_back(residual, index);
  }
  std::sort(suspects.begin(), suspects.end(), [](const auto &a, const auto &b) {
    // Equal residual: retain the nearer point (smaller sorted index).
    return a.first == b.first ? a.second > b.second : a.first > b.first;
  });
  const auto count = std::min({suspects.size(), points.size() - 3,
      static_cast<std::size_t>(std::max(0, std::min(2, cfg.ransac_max_outliers)))});
  std::vector<bool> remove(points.size(), false);
  for (std::size_t i = 0; i < count; ++i) remove[suspects[i].second] = true;
  std::size_t write = 0;
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (remove[i]) removed_indices.push_back(source_indices[i]);
    else points[write++] = points[i];
  }
  points.resize(write);
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
                           double denom, const FeatureConfig &cfg, double* reference_v) {
  LineGuide guide;
  if (cfg.guide_min_points < 2 ||
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
  *reference_v=near_reference_v;
  guide=ImageLineGuide(near_fit.a,near_reference_u,cx,
                       static_cast<int>(2*denom),0.0);
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
                             int image_height, const FeatureConfig &cfg,
                             int minimum_input_points) {
  Features f{};
  if (image_width <= 1 || image_height <= 1 ||
      minimum_input_points < 2 || minimum_input_points > 4) return f;

  std::vector<std::pair<Point2,std::size_t>> indexed;
  indexed.reserve(input.size());
  for (std::size_t index=0;index<input.size();++index) {
    const auto &point=input[index];
    if (std::isfinite(point.u) && std::isfinite(point.v)) {
      indexed.push_back({{Clamp(point.u, 0.0, image_width - 1.0),
                         Clamp(point.v, 0.0, image_height - 1.0)},index});
    }
  }
  std::sort(indexed.begin(), indexed.end(), [](const auto &left, const auto &right) {
    const auto &a=left.first,&b=right.first;
    return (a.v == b.v) ? (a.u < b.u) : (a.v > b.v);
  });
  indexed.resize(std::min({static_cast<std::size_t>(std::max(1, cfg.max_centers)),
                          kMaxRansacPoints, indexed.size()}));
  std::vector<Point2> points;
  std::vector<std::size_t> source_indices;
  points.reserve(indexed.size());source_indices.reserve(indexed.size());
  for (const auto &item:indexed) {
    points.push_back(item.first);source_indices.push_back(item.second);
  }

  f.n_visible = static_cast<double>(points.size());
  // Normal LINE keeps its four-point input gate. The DOWN-view HURDLE caller
  // explicitly permits two points without changing normal LINE behavior.
  if (points.size() < static_cast<std::size_t>(minimum_input_points) ||
      points.front().v - points.back().v <= kMinVSpanPx)
    return f;
  RemoveRansacOutliers(points, cfg, source_indices, f.ransac_outlier_indices);
  f.n_visible = static_cast<double>(points.size());

  const double cx = cfg.image_center_u >= 0.0
                        ? cfg.image_center_u
                        : 0.5 * static_cast<double>(image_width);
  // 중심점이 principal point로 이동해도 좌/우 정규화 크기가 달라지지 않도록
  // 영상 반폭을 분모로 사용한다.
  const double denom = std::max(0.5 * static_cast<double>(image_width), 1.0);
  f.guide = ComputeLineGuide(points, cx, denom, cfg, &f.guide_reference_v_px);
  return f;
}

CurveEvidence ComputeCurveEvidence(const std::vector<Point2>& input,
                                    int image_width, int image_height,
                                    const FeatureConfig& cfg,
                                    const CurveSearchConfig& curve_cfg) {
  CurveEvidence out;
  if (!curve_cfg.enabled || image_width<=1 || image_height<=1) return out;
  std::vector<Point2> original;
  for (const auto& point:input)
    if (std::isfinite(point.u) && std::isfinite(point.v))
      original.push_back({Clamp(point.u,0.0,image_width-1.0),
                          Clamp(point.v,0.0,image_height-1.0)});
  std::sort(original.begin(),original.end(),[](const Point2& a,const Point2& b) {
    return a.v==b.v ? a.u<b.u : a.v>b.v;
  });
  original.resize(std::min({original.size(),
      static_cast<std::size_t>(std::max(1,cfg.max_centers)),kMaxRansacPoints}));
  if (original.size()<5) return out;
  std::vector<Point2> lower_candidates=original;
  std::vector<std::size_t> indices,removed;
  for(std::size_t i=0;i<lower_candidates.size();++i) indices.push_back(i);
  RemoveRansacOutliers(lower_candidates,cfg,indices,removed);
  if (lower_candidates.size()<4) return out;
  // The LINE RANSAC already selected these points; do not change its result.
  // Keep the original lower LINE RANSAC and upper RANSAC candidate policy.
  // P4 is the common anchor in the six-/seven-point cases.
  const Point2 anchor=lower_candidates[3];
  std::vector<Point2> upper{anchor};
  for(const auto& pt:original)
    if(pt.v<anchor.v) upper.push_back(pt);
  std::sort(upper.begin(),upper.end(),[](const Point2& a,const Point2& b){
    return a.v==b.v ? a.u<b.u : a.v>b.v;
  });
  if(upper.size()<static_cast<std::size_t>(curve_cfg.min_upper_points))
    return out;
  if(upper.size()>=4) {
    std::vector<std::size_t> upper_indices,upper_removed;
    for(std::size_t i=0;i<upper.size();++i) upper_indices.push_back(i);
    FeatureConfig upper_cfg=cfg;
    upper_cfg.ransac_max_outliers=1;
    RemoveRansacOutliers(upper,upper_cfg,upper_indices,upper_removed);
  }
  if(upper.size()<2) return out;
  // Reject the frame rather than fit two non-overlapping lines when upper
  // RANSAC rejects the shared lower anchor.
  if(std::none_of(upper.begin(),upper.end(),[&](const Point2& point) {
       return point.u==anchor.u && point.v==anchor.v;
     })) return out;
  // Five usable points: P1..P3 / P3..P5 (3/3).
  // Six: P1..P4 / P4..P6 (4/3).
  // Seven or more: P1..P4 / P4..P7 (4/4).
  const bool five_points=upper.size()==2;
  std::vector<Point2> lower(lower_candidates.begin(),
                            lower_candidates.begin()+(five_points?3:4));
  if(five_points) upper.insert(upper.begin(),lower_candidates[2]);
  if(upper.size()>4) upper.resize(4);
  const auto lower_fit=FitLine(lower,0,lower.size());
  const auto upper_fit=FitLine(upper,0,upper.size());
  if(!lower_fit.valid || !upper_fit.valid) return out;
  constexpr double rad_to_deg=180.0/3.14159265358979323846;
  double delta=(-std::atan(upper_fit.a)+std::atan(lower_fit.a))*rad_to_deg;
  // Lines are unoriented: normalize the difference modulo 180 degrees.
  if(delta>90.0) delta-=180.0;
  if(delta< -90.0) delta+=180.0;
  out.valid=std::isfinite(delta) && std::abs(delta)>=curve_cfg.threshold_deg;
  out.delta_deg=delta;
  out.direction=out.valid?(delta>0?1:-1):0;
  out.lower_points=static_cast<int>(lower.size());
  out.upper_points=static_cast<int>(upper.size());
  return out;
}

} // namespace vision_core
