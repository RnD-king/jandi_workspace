#include "vision_core/config_loader.hpp"
#include "vision_core/line_feature_extractor.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <vector>

using namespace vision_core;

static std::vector<Point2> Straight(int count=8) {
  std::vector<Point2> points;
  for (int i=0;i<count;++i) points.push_back({320.,450.-50.*i});
  return points;
}
static void Centered(const Features& f,int count) {
  assert(f.guide.valid && f.n_visible==count);
  assert(std::abs(f.guide.offset)<1e-12 && std::abs(f.guide.heading_rad)<1e-12);
}
int main() {
  auto c=LoadDefaultAlgorithmConfig().line_features;
  c.image_center_u=320;c.curve_local_fit_points=4;c.guide_min_points=3;
  c.ransac_enabled=true;c.ransac_min_input_points=4;
  c.ransac_residual_threshold_px=20;c.ransac_min_sample_v_gap_px=20;c.ransac_max_outliers=2;
  const auto compute=[&](const std::vector<Point2>& points) {
    return ComputeLineFeatures(points,640,480,c);
  };
  Centered(compute(Straight()),8);
  for (int count=0;count<=3;++count) assert(!compute(Straight(count)).guide.valid);
  auto one=Straight();one[0].u=560;
  Centered(compute(one),7);
  assert(compute(one).ransac_outlier_indices==std::vector<std::size_t>({0}));
  auto two=one;two[2].u=80;
  Centered(compute(two),6);
  assert(compute(two).ransac_outlier_indices==std::vector<std::size_t>({0,2}));
  auto four=Straight(4);four[0].u=560;
  Centered(compute(four),3); // Initial four, filtered three is valid.

  // More than two threshold violations are tolerated. Only the largest
  // two are removed, with a far-first tie break; the near O/H stays intact.
  auto curved=Straight();curved[5].u=500;curved[6].u=600;curved[7].u=500;
  const auto curve=compute(curved);
  Centered(curve,6);
  assert(curve.guide.curvature_valid);

  // Competing three-point consensuses are ranked, not rejected as ambiguous.
  const std::vector<Point2> competing={{280,450},{360,400},{280,350},
                                      {360,300},{280,250},{360,200}};
  const auto ranked=compute(competing);
  assert(ranked.guide.valid && ranked.n_visible==4);
  assert(std::abs(ranked.guide.offset+.0625)<1e-12);
  for (auto points : {one,two,curved,competing}) {
    const auto reference=compute(points);
    std::reverse(points.begin(),points.end());
    const auto reverse=compute(points);
    assert(reverse.n_visible==reference.n_visible);
    assert(reverse.guide.valid==reference.guide.valid);
    assert(reverse.guide.offset==reference.guide.offset);
    assert(reverse.guide.heading_rad==reference.guide.heading_rad);
    assert(reverse.ransac_outlier_indices.size()==reference.ransac_outlier_indices.size());
    for(std::size_t i=0;i<reference.ransac_outlier_indices.size();++i) {
      const auto original_index=reference.ransac_outlier_indices[i];
      assert(reverse.ransac_outlier_indices[i]==points.size()-1-original_index);
    }
  }

  // Keep three even when a four-point frame has two suspect points.
  const auto capped=compute({{560,450},{80,400},{320,350},{320,300}});
  assert(capped.guide.valid && capped.n_visible==3);

  // The RANSAC reference must not replace the original near-four OLS.
  auto noisy=Straight();noisy[0].u=320;noisy[1].u=324;noisy[2].u=319;noisy[3].u=322;
  const auto fitted=compute(noisy);
  assert(fitted.n_visible==8 && fitted.guide.valid);
  assert(std::abs(fitted.guide.offset-(321.25-320)/320)<1e-12);
  assert(std::abs(fitted.guide.heading_rad-std::atan(.002))<1e-12);

  // Exactly the residual threshold is an inlier; it must not be trimmed.
  auto boundary=Straight();boundary[0].u=340;
  assert(compute(boundary).n_visible==8);
  c.ransac_residual_threshold_px=.5;
  Centered(compute(boundary),7);
  c.ransac_residual_threshold_px=20;

  // Pair spacing rejects hypotheses, not points. At least one 20px pair
  // can still explain both closely spaced near points.
  Centered(compute({{320,450},{320,448},{320,400},{320,350},{320,300}}),5);
  const std::vector<Point2> gap_boundary={{320,450},{520,445},{320,435},{320,430}};
  Centered(compute(gap_boundary),3); // Exactly 20px is eligible.
  c.ransac_min_sample_v_gap_px=20.01;
  const auto no_pair=compute(gap_boundary);
  assert(no_pair.guide.valid && no_pair.n_visible==4); // Unfiltered OLS fallback.
  c.ransac_min_sample_v_gap_px=20;
  assert(compute({{320,450},{500,446},{320,442},{320,438}}).guide.valid);

  // Inclusive five-pixel guard before RANSAC and before the final near fit.
  assert(!compute({{320,450},{320,450},{320,450},{320,450}}).guide.valid);
  assert(!compute({{320,450},{320,449},{320,448},{320,445}}).guide.valid);
  assert(compute({{320,450},{320,449},{320,448},{320,444.99}}).guide.valid);
  assert(!compute({{320,450},{320,449},{320,448},{320,445},
                   {320,400},{320,350}}).guide.valid);
  auto nonfinite=Straight(3);
  nonfinite.push_back({std::numeric_limits<double>::quiet_NaN(),300});
  assert(!compute(nonfinite).guide.valid);

  auto over_limit=Straight();over_limit.push_back({600,10});
  c.max_centers=16;
  Centered(compute(over_limit),8); // Never more than 28 pair hypotheses.
  assert(compute(over_limit).ransac_outlier_indices.empty()); // Truncation is not a RANSAC removal.
  c.max_centers=8;
  c.ransac_max_outliers=1;
  assert(compute(two).n_visible==7);
  c.ransac_max_outliers=0;
  assert(compute(two).n_visible==8);
  c.ransac_max_outliers=2;
  c.ransac_min_input_points=5;
  assert(compute(four).n_visible==4); // Too few to run RANSAC, but enough for O/H.
  c.ransac_min_input_points=4;
  c.ransac_enabled=false;
  assert(compute(two).n_visible==8);
  assert(compute(two).ransac_outlier_indices.empty());
  assert(!compute(Straight(3)).guide.valid); // Initial-count gate is independent.
  assert(!compute({{320,450},{320,449},{320,448},{320,445}}).guide.valid);
}
