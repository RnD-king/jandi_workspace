#include "vision_core/ball_fine_motion.hpp"
#include <algorithm>
#include <array>
#include <cmath>

namespace vision_core {
namespace {
bool Calibrated(Point2 p) {
  return std::isfinite(p.u) && std::isfinite(p.v) && (p.u!=0.0 || p.v!=0.0);
}
double Excess(double value,double lo,double hi) {
  return std::max({lo-value,0.0,value-hi})/((hi-lo)*0.5);
}
double LeftAt(double v,double u1,double v1,double v2,double inset) {
  const double t=std::clamp((v-v1)/(v2-v1),0.0,1.0);
  return u1+(1.0-t)*inset;
}
double RightAt(double v,double u2,double v1,double v2,double inset) {
  const double t=std::clamp((v-v1)/(v2-v1),0.0,1.0);
  return u2-(1.0-t)*inset;
}
double Error(Point2 p,double u1,double u2,double v1,double v2,double inset) {
  const double left=LeftAt(p.v,u1,v1,v2,inset);
  const double right=RightAt(p.v,u2,v1,v2,inset);
  const double u=Excess(p.u,left,right),v=Excess(p.v,v1,v2);
  return u*u+v*v;
}
bool PassesOppositeEdge(double start,double predicted,double lo,double hi) {
  if (start<lo) return predicted>hi;
  if (start>hi) return predicted<lo;
  return predicted<lo || predicted>hi;
}
}
bool BallFineMotionConfig::HasCalibration() const {
  return Calibrated(forward_delta_px) || Calibrated(backward_delta_px) ||
      Calibrated(left_delta_px) || Calibrated(right_delta_px);
}
BallFineMotionPlan PlanBallFineMotion(const BallFineMotionConfig& config,
    Point2 center,double u1,double u2,double v1,double v2,
    int image_width,int image_height,int repetition_limit,double top_inset_px) {
  BallFineMotionPlan best;
  if (image_width<=1 || image_height<=1 || !std::isfinite(center.u) ||
      !std::isfinite(center.v) || !std::isfinite(u1) || !std::isfinite(u2) ||
      !std::isfinite(v1) || !std::isfinite(v2) || u1>=u2 || v1>=v2 ||
      !std::isfinite(top_inset_px) || top_inset_px<0 ||
      (top_inset_px*2>=u2-u1 && top_inset_px>0) ||
      center.u<0 || center.u>=image_width || center.v<0 || center.v>=image_height)
    return best;
  const std::array<MissionAction,4> actions{MissionAction::kStepForwardHalf,
      MissionAction::kStepBack,MissionAction::kLeftSideStep,MissionAction::kRightSideStep};
  const std::array<Point2,4> deltas{config.forward_delta_px,config.backward_delta_px,
      config.left_delta_px,config.right_delta_px};
  const int limit=std::min({3,config.max_repetitions,repetition_limit});
  double best_error=Error(center,u1,u2,v1,v2,top_inset_px);
  if (!std::isfinite(best_error) || best_error<=1e-9) return best;
  for (std::size_t i=0;i<actions.size();++i) {
    if (!Calibrated(deltas[i])) continue;
    double previous_error=Error(center,u1,u2,v1,v2,top_inset_px);
    for (int n=1;n<=limit;++n) {
      const Point2 predicted{center.u+n*deltas[i].u,center.v+n*deltas[i].v};
      if (!std::isfinite(predicted.u) || !std::isfinite(predicted.v) ||
          predicted.u<0 || predicted.u>=image_width || predicted.v<0 || predicted.v>=image_height ||
          PassesOppositeEdge(center.u,predicted.u,
              LeftAt(center.v,u1,v1,v2,top_inset_px),
              RightAt(center.v,u2,v1,v2,top_inset_px)) ||
          PassesOppositeEdge(center.v,predicted.v,v1,v2)) break;
      const double error=Error(predicted,u1,u2,v1,v2,top_inset_px);
      if (!std::isfinite(error) || error>=previous_error-1e-9) break;
      if (error<best_error-1e-9 ||
          (best.repetitions>0 && std::abs(error-best_error)<=1e-9 && n<best.repetitions)) {
        best={actions[i],n,predicted};best_error=error;
      }
      previous_error=error;
    }
  }
  return best;
}
} // namespace vision_core
