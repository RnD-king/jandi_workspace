#include "vision_core/line_feature_extractor.hpp"
#include "vision_core/line_p2p_controller.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
using namespace vision_core;
int main() {
  FeatureConfig c{8,3,50,5,20,3,10};
  auto f=ComputeLineFeatures({{50,90},{50,70},{50,50}},100,100,c);
  assert(f.guide.valid); assert(std::abs(f.guide.offset)<1e-12);
  assert(std::abs(f.guide.heading_rad)<1e-12);
  assert(!f.guide.curvature_valid);  // curve_min_points 미달
  auto split=ComputeLineFeatures({{50,90},{50,70},{50,50},
                                  {30,20},{40,20},{50,20}},100,100,c);
  assert(split.guide.valid);
  assert(!split.guide.curvature_valid);
  assert(std::isfinite(split.guide.confidence));
  auto short_span=ComputeLineFeatures({{50,90},{50,87},{50,84},
                                      {49,81},{48,78}},100,100,c);
  assert(short_span.guide.valid);
  assert(!short_span.guide.curvature_valid);
  auto curved=ComputeLineFeatures({{50,90},{50,70},{50,50},
                                   {45,30},{35,10}},100,100,c);
  assert(curved.guide.valid);
  assert(curved.guide.curvature_valid);
  auto bad=ComputeLineFeatures({{50,90}},100,100,c);
  assert(!bad.guide.valid);
  auto two=ComputeLineFeatures({{50,90},{50,70}},100,100,c);
  assert(!two.guide.valid);

  // Default local fit 4 still uses all three points when only three exist.
  c.curve_local_fit_points=4;
  const LineP2pController selector(LineP2pConfig{1.0,1.0,.10});
  for (const int count : {3,4}) {
    for (const double u : {30.0,50.0,70.0}) {
      std::vector<Point2> points;
      for (int i=0;i<count;++i) points.push_back({u,90.0-i*20.0});
      const auto guide=ComputeLineFeatures(points,100,100,c).guide;
      assert(guide.valid);
      assert(std::abs(guide.offset-(u-50.0)/50.0)<1e-12);
      assert(std::abs(guide.heading_rad)<1e-12);
      const auto expected=u<50.0 ? CruiseDirection::kLeft :
          u>50.0 ? CruiseDirection::kRight : CruiseDirection::kStraight;
      assert(selector.Compute(guide).direction==expected);
    }
  }
  c.guide_min_points=5;
  assert(!ComputeLineFeatures({{50,90},{50,70},{50,50},{50,30}},
                              100,100,c).guide.valid);
  const auto five=ComputeLineFeatures(
      {{40,90},{40,70},{40,50},{40,30},{90,10}},100,100,c);
  assert(five.guide.valid);
  assert(std::abs(five.guide.offset)<1e-12); // near fit uses 5, not 4.
  return 0;
}
