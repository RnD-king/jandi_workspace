#pragma once

#include "camera_overlay/goal_measurement.hpp"
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

namespace camera_overlay {

inline cv::Mat DrawGoalMeasurementTopView(const std::optional<GoalMeasurement>& value,
                                         double radius_m) {
  cv::Mat image(620,760,CV_8UC3,cv::Scalar(24,24,24));
  const cv::Scalar point_color(230,230,230),board_color(0,220,255),rim_color(255,80,230);
  const auto text=[&](const std::string& label,int y,cv::Scalar color=cv::Scalar(220,220,220)) {
    cv::putText(image,label,{16,y},cv::FONT_HERSHEY_SIMPLEX,.48,color,1,cv::LINE_AA);
  };
  text("CAMERA X/Z | +X right, +Z forward | metres",24);
  text("Manual board assumption: click=B, sample bands near u +/- 30 px",46);
  char label[200];
  if (value) {
    const auto& m=*value;
    if (m.point.valid) {
      std::snprintf(label,sizeof(label),"CLICK 5x5: X=%+.4f  Z=%.4f",m.point.x_m,m.point.z_m);
      text(label,70,point_color);
    } else text("CLICK 5x5: X/Z unavailable (needs depth + matching CameraInfo)",70);
    if (m.board.valid) {
      std::snprintf(label,sizeof(label),"BOARD 3x3: X=%+.4f  Z=%.4f | phi=%+.2f deg",
          m.board.x_m,m.board.z_m,m.board.yaw_rad*180/std::acos(-1.0));
      text(label,94,board_color);
    } else text("BOARD pose unavailable",94);
    if (m.rim.valid) {
      std::snprintf(label,sizeof(label),"RIM: X=%+.4f  Z=%.4f | radius=%.4f m",m.rim.x_m,m.rim.z_m,radius_m);
      text(label,118,rim_color);
    } else text("RIM unavailable: "+m.status,118,rim_color);
  } else text("Click in the camera window to measure. SPACE/c: live; q/Esc: exit.",80);

  const cv::Rect plot(65,160,630,365);
  double max_x=.30,max_z=.60,min_z=-.15;
  std::vector<GoalPointXZ> points;
  if (value) {
    const auto& m=*value;
    points={m.point,m.left,m.right,m.rim,{m.board.valid,m.board.x_m,m.board.z_m}};
    for (const auto& p:points) if (p.valid) {
      max_x=std::max(max_x,std::abs(p.x_m)+.10);
      max_z=std::max(max_z,p.z_m+.15);
    }
    if (m.rim.valid) {
      max_x=std::max(max_x,std::abs(m.rim.x_m)+radius_m+.05);
      max_z=std::max(max_z,m.rim.z_m+radius_m+.05);
      min_z=std::min(min_z,m.rim.z_m-radius_m-.05);
    }
  }
  const double scale=std::min(plot.width/(2*max_x),plot.height/(max_z-min_z));
  const cv::Point origin(plot.x+plot.width/2,plot.y+plot.height+static_cast<int>(std::lround(min_z*scale)));
  const auto pixel=[&](double x,double z) {
    return cv::Point(origin.x+static_cast<int>(std::lround(x*scale)),
                     origin.y-static_cast<int>(std::lround(z*scale)));
  };
  cv::rectangle(image,plot,{80,80,80},1);
  const double step=std::max(.1,std::pow(10.0,std::floor(std::log10(max_z))));
  for (double z=0;z<=max_z;z+=step) {
    const auto p=pixel(0,z);
    cv::line(image,{plot.x,p.y},{plot.x+plot.width,p.y},{45,45,45},1);
    std::snprintf(label,sizeof(label),"%.1f",z);
    cv::putText(image,label,{plot.x-42,p.y+4},cv::FONT_HERSHEY_SIMPLEX,.38,{140,140,140},1);
  }
  cv::arrowedLine(image,{plot.x,origin.y},{plot.x+plot.width,origin.y},{140,140,140},1);
  cv::arrowedLine(image,pixel(0,min_z),{origin.x,plot.y},{140,140,140},1);
  cv::putText(image,"+X",{plot.x+plot.width+5,origin.y},cv::FONT_HERSHEY_SIMPLEX,.45,{180,180,180},1);
  cv::putText(image,"+Z",{origin.x+8,plot.y+16},cv::FONT_HERSHEY_SIMPLEX,.45,{180,180,180},1);
  cv::drawMarker(image,origin,{255,255,255},cv::MARKER_CROSS,15,1);
  cv::putText(image,"Camera (0,0)",origin+cv::Point(10,18),cv::FONT_HERSHEY_SIMPLEX,.4,{220,220,220},1);
  if (value) {
    const auto& m=*value;
    if (m.left.valid && m.right.valid) {
      const auto l=pixel(m.left.x_m,m.left.z_m),r=pixel(m.right.x_m,m.right.z_m);
      cv::line(image,l,r,{255,180,0},2,cv::LINE_AA);
      cv::circle(image,l,4,{255,180,0},-1);cv::circle(image,r,4,{0,220,255},-1);
      cv::putText(image,"L",l+cv::Point(-18,-6),cv::FONT_HERSHEY_SIMPLEX,.45,{255,180,0},1);
      cv::putText(image,"R",r+cv::Point(6,-6),cv::FONT_HERSHEY_SIMPLEX,.45,{0,220,255},1);
    }
    if (m.point.valid) cv::drawMarker(image,pixel(m.point.x_m,m.point.z_m),point_color,cv::MARKER_CROSS,17,1);
    if (m.board.valid) {
      const auto b=pixel(m.board.x_m,m.board.z_m);
      cv::rectangle(image,cv::Rect(b.x-4,b.y-4,9,9),board_color,2);
      cv::putText(image,"B",b+cv::Point(7,-9),cv::FONT_HERSHEY_SIMPLEX,.45,board_color,1);
      // Draw the actual estimated tangent through B. The raw L/R segment
      // may be offset if the three median depth samples are inconsistent.
      const double dx=.10*std::cos(m.board.yaw_rad),dz=-.10*std::sin(m.board.yaw_rad);
      cv::line(image,pixel(m.board.x_m-dx,m.board.z_m-dz),
          pixel(m.board.x_m+dx,m.board.z_m+dz),board_color,2,cv::LINE_AA);
      if (m.rim.valid) {
        const auto g=pixel(m.rim.x_m,m.rim.z_m);
        cv::arrowedLine(image,b,g,rim_color,1,cv::LINE_AA);
        cv::circle(image,g,std::max(1,static_cast<int>(std::lround(radius_m*scale))),rim_color,1,cv::LINE_AA);
        cv::circle(image,g,4,rim_color,-1);
        cv::putText(image,"RIM",g+cv::Point(7,16),cv::FONT_HERSHEY_SIMPLEX,.45,rim_color,1);
      }
    }
    const auto depth_text=[](const char* name,const std::optional<double>& z,
                             const DepthRegionPixelStats& stats) {
      char out[96];
      if (z) std::snprintf(out,sizeof(out),"%s=%.4fm %d/%d",name,*z,stats.valid,stats.total);
      else std::snprintf(out,sizeof(out),"%s=-- %d/%d",name,stats.valid,stats.total);
      return std::string(out);
    };
    text(depth_text("L",m.left_depth_m,m.left_stats)+" | "+
         depth_text("B",m.center_depth_m,m.center_stats)+" | "+
         depth_text("R",m.right_depth_m,m.right_stats),554);
    text("Status: "+m.status,580);
  }
  text("Optical camera origin; no body/IMU transform or automatic board detection.",607);
  return image;
}

}  // namespace camera_overlay
