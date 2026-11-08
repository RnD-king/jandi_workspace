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
              LeftAt(predicted.v,u1,v1,v2,top_inset_px),
              RightAt(predicted.v,u2,v1,v2,top_inset_px)) ||
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

BallFineMotionPlan PlanBallFineMotion(const BallFineMotionConfig& config,
    Point2 center,const PickupPolygon& polygon,
    int image_width,int image_height,int repetition_limit) {
  BallFineMotionPlan best;
  if(image_width<=1 || image_height<=1 || polygon.size()<3 ||
     !std::isfinite(center.u) || !std::isfinite(center.v) ||
     center.u<0 || center.u>=image_width || center.v<0 || center.v>=image_height ||
     PickupPolygonContains(polygon,center))
    return best;
  const std::array<MissionAction,4> actions{MissionAction::kStepForwardHalf,
      MissionAction::kStepBack,MissionAction::kLeftSideStep,MissionAction::kRightSideStep};
  const std::array<Point2,4> deltas{config.forward_delta_px,config.backward_delta_px,
      config.left_delta_px,config.right_delta_px};
  const int limit=std::max(0,std::min({3,config.max_repetitions,repetition_limit}));
  const double initial_distance=PickupPolygonDistance(polygon,center);
  double best_distance=initial_distance;
  bool best_inside=false;
  // If the ball is above the pickup region and starts within the
  // configured lateral margin, prioritize measured forward movement.
  // Its predicted u may leave that margin: vertical approach comes first.
  double min_u=polygon.front().u,max_u=min_u,min_v=polygon.front().v,max_v=min_v;
  for(const auto& vertex:polygon) {
    min_u=std::min(min_u,vertex.u);
    max_u=std::max(max_u,vertex.u);
    min_v=std::min(min_v,vertex.v);
    max_v=std::max(max_v,vertex.v);
  }
  const bool forward_first=center.v<min_v &&
      center.u>=min_u-config.forward_first_max_lateral_error_px &&
      center.u<=max_u+config.forward_first_max_lateral_error_px;
  if(forward_first && Calibrated(deltas[0]) && deltas[0].v>0) {
    BallFineMotionPlan forward_plan;
    for(int n=1;n<=limit;++n) {
      const Point2 predicted{center.u+n*deltas[0].u,center.v+n*deltas[0].v};
      if(!std::isfinite(predicted.u) || !std::isfinite(predicted.v) ||
         predicted.u<0 || predicted.u>=image_width ||
         predicted.v<0 || predicted.v>=image_height) break;
      const bool inside=PickupPolygonContains(polygon,predicted);
      // Stop before passing below the whole pickup region.
      if(!inside && predicted.v>max_v) break;
      if(!inside) {
        int crossings=0;
        for(std::size_t edge=0;edge<polygon.size();++edge)
          if(PickupSegmentsIntersect(center,predicted,polygon[edge],
                  polygon[(edge+1)%polygon.size()])) ++crossings;
        if(crossings>=2) break;
      }
      forward_plan={MissionAction::kStepForwardHalf,n,predicted};
      // Once the vertical band is reached, reobserve and correct u.
      if(inside || predicted.v>=min_v) break;
    }
    if(forward_plan.repetitions>0) return forward_plan;
  }
  for(std::size_t i=0;i<actions.size();++i) {
    if(!Calibrated(deltas[i])) continue;
    double previous=initial_distance;
    for(int n=1;n<=limit;++n) {
      const Point2 predicted{center.u+n*deltas[i].u,center.v+n*deltas[i].v};
      if(!std::isfinite(predicted.u) || !std::isfinite(predicted.v) ||
         predicted.u<0 || predicted.u>=image_width ||
         predicted.v<0 || predicted.v>=image_height) break;
      const bool inside=PickupPolygonContains(polygon,predicted);
      // Never select a step which leaps across the whole target and exits
      // its opposite side even if the endpoint happens to be nearer.
      if(!inside) {
        int crossings=0;
        for(std::size_t edge=0;edge<polygon.size();++edge)
          if(PickupSegmentsIntersect(center,predicted,polygon[edge],
                  polygon[(edge+1)%polygon.size()])) ++crossings;
        if(crossings>=2) break;
      }
      const double distance=PickupPolygonDistance(polygon,predicted);
      if(!inside && distance>=previous-1e-9) break;
      if(inside) {
        if(!best_inside || n<best.repetitions) {
          best={actions[i],n,predicted};
          best_distance=0.0;
          best_inside=true;
        }
        break; // Do not continue past the first successful prefix.
      }
      if(!best_inside &&
          (distance<best_distance-1e-9 ||
           (std::abs(distance-best_distance)<=1e-9 &&
            best.repetitions>0 && n<best.repetitions))) {
        best={actions[i],n,predicted};
        best_distance=distance;
      }
      previous=distance;
    }
  }
  return best;
}
} // namespace vision_core
