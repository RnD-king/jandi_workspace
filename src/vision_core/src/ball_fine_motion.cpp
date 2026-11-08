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
  // The robot's measured forward motion is oblique in the DOWN image.
  // If the ball is still above pickup and not excessively far sideways,
  // prefer a safe improving forward prefix before fine lateral correction.
  double min_u=polygon.front().u,max_u=min_u,min_v=polygon.front().v;
  for(const auto& vertex:polygon) {
    min_u=std::min(min_u,vertex.u);
    max_u=std::max(max_u,vertex.u);
    min_v=std::min(min_v,vertex.v);
  }
  const bool forward_first=center.v<min_v &&
      center.u>=min_u-config.forward_first_max_lateral_error_px &&
      center.u<=max_u+config.forward_first_max_lateral_error_px;
  BallFineMotionPlan forward_plan;
  double forward_distance=initial_distance;
  bool forward_inside=false;
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
      if(i==0 && forward_first && predicted.v>center.v &&
         predicted.u>=min_u-config.forward_first_max_lateral_error_px &&
         predicted.u<=max_u+config.forward_first_max_lateral_error_px) {
        if (inside && (!forward_inside || n<forward_plan.repetitions)) {
          forward_plan={actions[i],n,predicted};
          forward_inside=true;
          forward_distance=0.0;
        } else if (!forward_inside && distance<forward_distance-1e-9) {
          forward_plan={actions[i],n,predicted};
          forward_distance=distance;
        }
      }
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
  // Entering the polygon is always better than an outside-only forward
  // preference. Otherwise retain the forward-first policy and reobserve
  // only after the chosen 1..3-action batch.
  if(forward_plan.repetitions>0 && (forward_inside || !best_inside))
    return forward_plan;
  return best;
}
} // namespace vision_core
