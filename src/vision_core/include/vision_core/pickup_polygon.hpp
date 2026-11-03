#pragma once

#include "vision_core/types.hpp"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace vision_core {

using PickupPolygon = std::vector<Point2>;

// Image-pixel coordinates, ordered around the perimeter; a convex polygon
// is not required. Keep this geometry independent from OpenCV/ROS so both
// mission decisions and the debug renderer use exactly the same region.
inline double PickupCross(Point2 a, Point2 b, Point2 c) {
  return (b.u-a.u)*(c.v-a.v)-(b.v-a.v)*(c.u-a.u);
}
inline double PickupDistanceSquared(Point2 a, Point2 b) {
  const double u=a.u-b.u,v=a.v-b.v;
  return u*u+v*v;
}
inline bool PickupOnSegment(Point2 p, Point2 a, Point2 b) {
  constexpr double eps=1e-7;
  return std::abs(PickupCross(a,b,p))<=eps &&
      p.u>=std::min(a.u,b.u)-eps && p.u<=std::max(a.u,b.u)+eps &&
      p.v>=std::min(a.v,b.v)-eps && p.v<=std::max(a.v,b.v)+eps;
}
inline bool PickupSegmentsIntersect(Point2 a,Point2 b,Point2 c,Point2 d) {
  constexpr double eps=1e-7;
  const double ab_c=PickupCross(a,b,c),ab_d=PickupCross(a,b,d);
  const double cd_a=PickupCross(c,d,a),cd_b=PickupCross(c,d,b);
  if (((ab_c>eps && ab_d < -eps)||(ab_c < -eps && ab_d>eps)) &&
      ((cd_a>eps && cd_b < -eps)||(cd_a < -eps && cd_b>eps)))
    return true;
  return (std::abs(ab_c)<=eps && PickupOnSegment(c,a,b)) ||
         (std::abs(ab_d)<=eps && PickupOnSegment(d,a,b)) ||
         (std::abs(cd_a)<=eps && PickupOnSegment(a,c,d)) ||
         (std::abs(cd_b)<=eps && PickupOnSegment(b,c,d));
}
inline double PickupSignedArea2(const PickupPolygon& polygon) {
  double area2=0.0;
  for(std::size_t i=0;i<polygon.size();++i) {
    const auto& a=polygon[i];
    const auto& b=polygon[(i+1)%polygon.size()];
    area2+=a.u*b.v-b.u*a.v;
  }
  return area2;
}
inline void ValidatePickupPolygon(const PickupPolygon& polygon,
                                  int width=640,int height=480) {
  if(polygon.size()<3 || polygon.size()>64)
    throw std::invalid_argument("pickup polygon requires 3..64 vertices");
  for(std::size_t i=0;i<polygon.size();++i) {
    const auto& p=polygon[i];
    if(!std::isfinite(p.u) || !std::isfinite(p.v) ||
       p.u<0 || p.v<0 || p.u>=width || p.v>=height)
      throw std::invalid_argument("pickup polygon vertices must be inside 640x480 image");
    if(PickupDistanceSquared(p,polygon[(i+1)%polygon.size()])<1e-12)
      throw std::invalid_argument("pickup polygon has repeated consecutive vertices");
  }
  if(std::abs(PickupSignedArea2(polygon))<1e-6)
    throw std::invalid_argument("pickup polygon has zero area");
  const auto n=polygon.size();
  for(std::size_t i=0;i<n;++i) {
    for(std::size_t j=i+1;j<n;++j) {
      if(j==i+1 || (i==0 && j==n-1)) continue;
      if(PickupSegmentsIntersect(polygon[i],polygon[(i+1)%n],
                                 polygon[j],polygon[(j+1)%n]))
        throw std::invalid_argument("pickup polygon edges intersect");
    }
  }
}
inline Point2 PickupPolygonCentroid(const PickupPolygon& polygon) {
  const double area2=PickupSignedArea2(polygon);
  if(std::abs(area2)<1e-9) throw std::invalid_argument("pickup polygon has zero area");
  Point2 center{};
  for(std::size_t i=0;i<polygon.size();++i) {
    const auto& a=polygon[i];
    const auto& b=polygon[(i+1)%polygon.size()];
    const double cross=a.u*b.v-b.u*a.v;
    center.u+=(a.u+b.u)*cross;
    center.v+=(a.v+b.v)*cross;
  }
  center.u/=3.0*area2;
  center.v/=3.0*area2;
  return center;
}
inline PickupPolygon ScalePickupPolygon(const PickupPolygon& polygon,double scale) {
  const auto center=PickupPolygonCentroid(polygon);
  PickupPolygon scaled;
  scaled.reserve(polygon.size());
  for(const auto& p:polygon)
    scaled.push_back({center.u+(p.u-center.u)*scale,
                      center.v+(p.v-center.v)*scale});
  return scaled;
}
inline bool PickupPolygonContains(const PickupPolygon& polygon,Point2 p) {
  if(polygon.size()<3) return false;
  bool inside=false;
  for(std::size_t i=0,j=polygon.size()-1;i<polygon.size();j=i++) {
    const auto& a=polygon[j];
    const auto& b=polygon[i];
    if(PickupOnSegment(p,a,b)) return true;
    if((a.v>p.v)!=(b.v>p.v)) {
      const double x=a.u+(p.v-a.v)*(b.u-a.u)/(b.v-a.v);
      if(p.u<x) inside=!inside;
    }
  }
  return inside;
}
inline Point2 ClosestPickupBoundaryPoint(const PickupPolygon& polygon,Point2 p) {
  Point2 closest{};
  double best=std::numeric_limits<double>::infinity();
  for(std::size_t i=0;i<polygon.size();++i) {
    const auto& a=polygon[i];
    const auto& b=polygon[(i+1)%polygon.size()];
    const double du=b.u-a.u,dv=b.v-a.v;
    const double length2=du*du+dv*dv;
    if(length2<=0.0) continue;
    const double t=std::clamp(((p.u-a.u)*du+(p.v-a.v)*dv)/length2,0.0,1.0);
    const Point2 q{a.u+t*du,a.v+t*dv};
    const double d=PickupDistanceSquared(p,q);
    if(d<best) { best=d;closest=q; }
  }
  return closest;
}
inline double PickupPolygonDistance(const PickupPolygon& polygon,Point2 p) {
  if(PickupPolygonContains(polygon,p)) return 0.0;
  return std::sqrt(PickupDistanceSquared(p,ClosestPickupBoundaryPoint(polygon,p)));
}
inline std::string FormatPickupPolygonPoints(const PickupPolygon& polygon) {
  std::ostringstream stream;
  stream<<std::setprecision(12);
  for(std::size_t i=0;i<polygon.size();++i) {
    if(i) stream<<';';
    stream<<polygon[i].u<<','<<polygon[i].v;
  }
  return stream.str();
}
inline PickupPolygon ParsePickupPolygonPoints(const std::string& value) {
  const auto trim=[](std::string str) {
    const auto nonspace=[](unsigned char c){return !std::isspace(c);};
    auto first=std::find_if(str.begin(),str.end(),nonspace);
    auto last=std::find_if(str.rbegin(),str.rend(),nonspace).base();
    return first>=last ? std::string() : std::string(first,last);
  };
  const auto number=[&](const std::string& value) {
    const auto token=trim(value);
    std::size_t consumed=0;
    if(token.empty()) throw std::invalid_argument("pickup polygon coordinate is empty");
    double parsed=0.0;
    try {parsed=std::stod(token,&consumed);}
    catch(const std::exception&) {throw std::invalid_argument("pickup polygon coordinate is not numeric");}
    if(consumed!=token.size() || !std::isfinite(parsed))
      throw std::invalid_argument("pickup polygon coordinate must be finite");
    return parsed;
  };
  PickupPolygon polygon;
  std::size_t cursor=0;
  while(cursor<=value.size()) {
    const auto end=value.find(';',cursor);
    const auto piece=trim(value.substr(cursor,end==std::string::npos
        ? std::string::npos : end-cursor));
    const auto comma=piece.find(',');
    if(comma==std::string::npos || piece.find(',',comma+1)!=std::string::npos)
      throw std::invalid_argument("pickup polygon format: u,v;u,v;u,v");
    polygon.push_back({number(piece.substr(0,comma)),number(piece.substr(comma+1))});
    if(polygon.size()>64)
      throw std::invalid_argument("pickup polygon requires at most 64 vertices");
    if(end==std::string::npos) break;
    cursor=end+1;
  }
  ValidatePickupPolygon(polygon);
  return polygon;
}

} // namespace vision_core
