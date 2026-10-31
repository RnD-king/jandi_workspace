#include "vision_core/object_approach.hpp"
#include "vision_core/coordinate_rectifier.hpp"
#include <cmath>

namespace vision_core {
namespace {
constexpr double kRad=3.14159265358979323846/180.0;
struct Vec { double x,y,z; };
Vec RayToBody(Vec p, const ApproachCameraPose& c) {
  const double cp=std::cos(c.pitch_deg*kRad),sp=std::sin(c.pitch_deg*kRad);
  const double cy=std::cos(c.yaw_deg*kRad),sy=std::sin(c.yaw_deg*kRad);
  const Vec v{p.x,-cp*p.y-sp*p.z,-sp*p.y+cp*p.z};
  return {cy*v.x+sy*v.z,v.y,-sy*v.x+cy*v.z};
}
Point2 Project(Vec p,const ApproachCameraPose& c,const Intrinsics& k) {
  p={p.x-c.right_m,p.y-c.height_m,p.z-c.forward_m};
  const double cp=std::cos(c.pitch_deg*kRad),sp=std::sin(c.pitch_deg*kRad);
  const double cy=std::cos(c.yaw_deg*kRad),sy=std::sin(c.yaw_deg*kRad);
  const double x=cy*p.x-sy*p.z,z=sy*p.x+cy*p.z;
  const double camera_y=-cp*p.y-sp*z,camera_z=-sp*p.y+cp*z;
  if (camera_z<=1e-6) return {NAN,NAN};
  return {k.cx+k.fx*x/camera_z,k.cy+k.fy*camera_y/camera_z};
}
}
LineGuide ImageLineGuide(double slope,double u,double reference_u,int width,double confidence) {
  LineGuide result;
  if (width<=1 || !std::isfinite(slope) || !std::isfinite(u) || !std::isfinite(reference_u)) return result;
  result.offset=(u-reference_u)/(0.5*width);
  result.heading_rad=-std::atan(slope);
  result.confidence=confidence;
  result.valid=std::isfinite(result.offset) && std::isfinite(result.heading_rad);
  return result;
}
LineGuide ObjectApproachGuide(const ObjectTarget& t,bool board,double depth,
    const Intrinsics& k,const ApproachCameraPose& c,int width,int height,
    double reference_u,double reference_v,bool rectify,double roll,double pitch,
    double* body_bearing_rad) {
  if (!board) {
    if (width<=1 || height<=1 || !std::isfinite(reference_u) ||
        !std::isfinite(reference_v)) return {};
    // BALL and far GOAL use the same image path slope calculation as LINE,
    // without a 3-D ray or a floor-plane projection.
    Point2 robot{reference_u,static_cast<double>(height-1)};
    Point2 ball=t.center_px;
    if (rectify) {
      const auto pixels=RectifyPixelPoints({robot,ball},k,roll,pitch);
      robot=pixels[0];ball=pixels[1];
    }
    if (!std::isfinite(robot.u) || !std::isfinite(robot.v) ||
        !std::isfinite(ball.u) || !std::isfinite(ball.v) ||
        std::abs(ball.v-robot.v)<1e-6) return {};
    const double slope=(ball.u-robot.u)/(ball.v-robot.v);
    return ImageLineGuide(slope,robot.u+slope*(reference_v-robot.v),
                          reference_u,width,t.confidence);
  }
  if (width<=1 || height<=1 || !std::isfinite(k.fx) || k.fx<=0 ||
      !std::isfinite(k.fy) || k.fy<=0 || !std::isfinite(k.cx) || !std::isfinite(k.cy)) return {};
  const Point2 pixel=t.center_px;
  const auto ray=RayToBody({(pixel.u-k.cx)/k.fx,(pixel.v-k.cy)/k.fy,1},c);
  double scale=depth;
  if (!std::isfinite(scale) || scale<=0) return {};
  const double x=c.right_m+scale*ray.x,z=c.forward_m+scale*ray.z;
  if (!std::isfinite(x) || !std::isfinite(z) || z<=0.05) return {};
  if (body_bearing_rad) *body_bearing_rad=std::atan2(x,z);
  // Two points describe the chosen ground path, not extra sensor evidence.
  // Extend the line for projection even when the target itself is above ground.
  auto a=Project({x/z*0.5,0,0.5},c,k);
  auto b=Project({x/z*1.5,0,1.5},c,k);
  if (rectify) {
    const auto points=RectifyPixelPoints({a,b},k,roll,pitch);
    a=points[0];b=points[1];
  }
  if (!std::isfinite(a.u) || !std::isfinite(a.v) || !std::isfinite(b.u) ||
      !std::isfinite(b.v) || std::abs(b.v-a.v)<1e-6) return {};
  const double slope=(b.u-a.u)/(b.v-a.v);
  return ImageLineGuide(slope,a.u+slope*(reference_v-a.v),reference_u,width,t.confidence);
}
}
