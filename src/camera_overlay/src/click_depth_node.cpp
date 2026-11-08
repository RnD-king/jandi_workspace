#include "camera_overlay/debug_view.hpp"
#include "camera_overlay/goal_measurement.hpp"
#include "camera_overlay/goal_measurement_view.hpp"

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

class ClickDepthNode final : public rclcpp::Node {
public:
  ClickDepthNode() : Node("click_depth") {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only=true;
    descriptor.description="Existing color image topic; set at startup.";
    const auto image_topic=declare_parameter<std::string>(
        "image_topic","/camera/camera/color/image_raw",descriptor);
    descriptor.description="Depth registered to the color pixels; set at startup.";
    const auto depth_topic=declare_parameter<std::string>(
        "depth_topic","/camera/camera/aligned_depth_to_color/image_raw",descriptor);
    descriptor.description="Color CameraInfo topic; K is used exactly as in GOAL.";
    const auto info_topic=declare_parameter<std::string>(
        "camera_info_topic","/camera/camera/color/camera_info",descriptor);
    descriptor.description="Local rim radius copied from GOAL; metres.";
    config_.hoop_radius_m=declare_parameter<double>("hoop_radius_m",config_.hoop_radius_m,descriptor);
    descriptor.description="Local GOAL sample strip width; pixels.";
    config_.backboard_depth_strip_width_px=declare_parameter<int>(
        "backboard_depth_strip_width_px",config_.backboard_depth_strip_width_px,descriptor);
    descriptor.description="Local GOAL minimum board-center depth; metres.";
    config_.backboard_min_depth_m=declare_parameter<double>(
        "backboard_min_depth_m",config_.backboard_min_depth_m,descriptor);
    descriptor.description="Local GOAL maximum board-center depth; metres.";
    config_.backboard_max_depth_m=declare_parameter<double>(
        "backboard_max_depth_m",config_.backboard_max_depth_m,descriptor);
    camera_overlay::ValidateGoalMeasurementConfig(config_);
    descriptor.description="Maximum color/depth timestamp difference in milliseconds.";
    const double max_skew_ms=declare_parameter<double>("max_sync_skew_ms",30.0,descriptor);
    if (image_topic.empty() || depth_topic.empty() || info_topic.empty())
      throw std::invalid_argument("image_topic, depth_topic and camera_info_topic must not be empty");
    if (!std::isfinite(max_skew_ms) || max_skew_ms<=0 || max_skew_ms>1000)
      throw std::invalid_argument("max_sync_skew_ms must be in (0,1000]");

    // All subscriptions, the timer and HighGUI callbacks use the main thread.
    // ApproximateTime bounds frame mismatch instead of sampling latest depth.
    color_sub_.subscribe(this,image_topic,rmw_qos_profile_sensor_data);
    depth_sub_.subscribe(this,depth_topic,rmw_qos_profile_sensor_data);
    info_sub_=create_subscription<sensor_msgs::msg::CameraInfo>(info_topic,rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr info) {camera_info_=std::move(info);});
    synchronizer_=std::make_shared<Synchronizer>(Policy(10),color_sub_,depth_sub_);
    synchronizer_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(max_skew_ms*.001));
    synchronizer_->registerCallback(std::bind(
        &ClickDepthNode::OnPair,this,std::placeholders::_1,std::placeholders::_2));
    debug_view_ = std::make_unique<camera_overlay::DebugView>(*this);
    if (debug_view_->Local()) {
      cv::namedWindow(kWindow,cv::WINDOW_AUTOSIZE | cv::WINDOW_GUI_NORMAL);
      cv::namedWindow(kTopView,cv::WINDOW_AUTOSIZE | cv::WINDOW_GUI_NORMAL);
      cv::setMouseCallback(kWindow,&ClickDepthNode::OnMouse,this);
    }
    timer_=create_wall_timer(std::chrono::milliseconds(33),[this]() {DrawFrame();});
    RCLCPP_INFO(get_logger(),"Color: %s | aligned depth: %s | max skew %.1f ms",
        image_topic.c_str(),depth_topic.c_str(),max_skew_ms);
    RCLCPP_INFO(get_logger(),"Click: freeze displayed frame and measure 5x5 median Z; SPACE/c: live; q/Esc: exit.");
    RCLCPP_INFO(get_logger(),"CameraInfo: %s | standalone GOAL geometry | R=%.4f m | strip=%d px | Z=[%.3f,%.3f] m",
        info_topic.c_str(),config_.hoop_radius_m,config_.backboard_depth_strip_width_px,
        config_.backboard_min_depth_m,config_.backboard_max_depth_m);
  }

  ~ClickDepthNode() override {
    if (!debug_view_->Local()) return;
    try {cv::destroyWindow(kWindow);} catch (const cv::Exception&) {}
    try {cv::destroyWindow(kTopView);} catch (const cv::Exception&) {}
  }

private:
  using Image=sensor_msgs::msg::Image;
  using Policy=message_filters::sync_policies::ApproximateTime<Image,Image>;
  using Synchronizer=message_filters::Synchronizer<Policy>;
  static constexpr const char* kWindow="Click Depth: 5x5 Median";
  static constexpr const char* kTopView="GOAL XZ: Manual Board + Rim";
  struct Frame {
    cv::Mat color,depth;
    std::int64_t stamp_ns{0};
    double skew_ms{0};
    std::optional<camera_overlay::Intrinsics> intrinsics;
    std::string calibration_status{"NO CAMERA INFO"};
  };
  struct WebContext { std::shared_ptr<const Frame> frame; std::uint64_t revision; };

  void OnPair(const Image::ConstSharedPtr& color,const Image::ConstSharedPtr& depth) {
    if (!color->width || !color->height ||
        color->width!=depth->width || color->height!=depth->height) {
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),3000,
          "Color/depth sizes differ (%ux%u / %ux%u); use depth aligned to color. Pair ignored.",
          color->width,color->height,depth->width,depth->height);
      return;
    }
    if (depth->encoding!=sensor_msgs::image_encodings::TYPE_16UC1 &&
        depth->encoding!=sensor_msgs::image_encodings::TYPE_32FC1) {
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),3000,
          "Unsupported depth encoding '%s'; expected 16UC1 (mm) or 32FC1 (m).",depth->encoding.c_str());
      return;
    }
    try {
      auto frame=std::make_shared<Frame>();
      frame->color=cv_bridge::toCvShare(color,"bgr8")->image.clone();
      frame->depth=cv_bridge::toCvShare(depth)->image.clone();
      frame->stamp_ns=rclcpp::Time(color->header.stamp).nanoseconds();
      frame->skew_ms=std::abs((rclcpp::Time(color->header.stamp)-
                             rclcpp::Time(depth->header.stamp)).seconds())*1000.0;
      // Freeze calibration along with the displayed RGB-D pair. Never let a
      // later CameraInfo silently reinterpret an already frozen measurement.
      if (camera_info_) {
        const auto& info=*camera_info_;
        if (info.width!=color->width || info.height!=color->height) {
          frame->calibration_status="CAMERA INFO SIZE MISMATCH";
        } else if (!info.header.frame_id.empty() && !color->header.frame_id.empty() &&
                   info.header.frame_id!=color->header.frame_id) {
          frame->calibration_status="CAMERA INFO FRAME MISMATCH";
        } else if (!std::isfinite(info.k[0]) || !std::isfinite(info.k[4]) ||
                   !std::isfinite(info.k[2]) || !std::isfinite(info.k[5]) ||
                   info.k[0]<=0 || info.k[4]<=0) {
          frame->calibration_status="INVALID CAMERA INFO K";
        } else {
          frame->intrinsics=camera_overlay::Intrinsics{info.k[0],info.k[4],info.k[2],info.k[5]};
          frame->calibration_status="COLOR K READY";
        }
      }
      latest_=std::move(frame);
    } catch (const std::exception& error) {
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),3000,"Image conversion failed: %s",error.what());
    }
  }

  static void OnMouse(int event,int x,int y,int,void* userdata) {
    if (event!=cv::EVENT_LBUTTONDOWN) return;
    auto* node=static_cast<ClickDepthNode*>(userdata);
    // Use the pair associated with imshow, never a newer subscription frame.
    node->MeasureAt(node->displayed_, {x,y});
  }
  void MeasureAt(const std::shared_ptr<const Frame> &frame, cv::Point point) {
    const int x=point.x,y=point.y;
    auto* node=this;
    if (!frame || x<0 || y<0 || x>=frame->color.cols || y>=frame->color.rows) return;
    ++web_revision_;
    node->frozen_=frame;
    node->measurement_=camera_overlay::MeasureGoalAtClick(frame->depth,{x,y},frame->intrinsics,node->config_);
    const auto& goal=*node->measurement_;
    const auto& m=goal.point_depth;
    if (m.median_m) {
      RCLCPP_INFO(node->get_logger(),
          "(%d,%d) median Z=%.6f m (%.3f mm), valid=%zu/%d%s, color stamp=%.9f, skew=%.3f ms",
          x,y,*m.median_m,*m.median_m*1000.0,m.valid_pixels,m.roi.area(),
          m.clipped ? " CLIPPED" : "",frame->stamp_ns*1e-9,frame->skew_ms);
    } else {
      RCLCPP_WARN(node->get_logger(),"(%d,%d) NO VALID DEPTH (0/%d%s); no previous value reused.",
          x,y,m.roi.area(),m.clipped ? " CLIPPED" : "");
    }
    if (goal.point.valid)
      RCLCPP_INFO(node->get_logger(),"CLICK 5x5 camera X/Z=(%+.6f,%.6f) m",goal.point.x_m,goal.point.z_m);
    if (goal.rim.valid) {
      RCLCPP_INFO(node->get_logger(),
          "MANUAL BOARD 3x3 X/Z=(%+.6f,%.6f) m; phi=%+.4f deg; RIM X/Z=(%+.6f,%.6f) m; R=%.4f m",
          goal.board.x_m,goal.board.z_m,goal.board.yaw_rad*180.0/std::acos(-1.0),
          goal.rim.x_m,goal.rim.z_m,node->config_.hoop_radius_m);
    } else {
      RCLCPP_WARN(node->get_logger(),"RIM unavailable: %s | %s",goal.status.c_str(),frame->calibration_status.c_str());
    }
  }

  static void PutLine(cv::Mat& image,const std::string& text,int line) {
    const cv::Point origin(10,24+line*23);
    cv::putText(image,text,origin,cv::FONT_HERSHEY_SIMPLEX,.5,{0,0,0},3,cv::LINE_AA);
    cv::putText(image,text,origin,cv::FONT_HERSHEY_SIMPLEX,.5,{255,255,255},1,cv::LINE_AA);
  }

  void DrawFrame() {
    try {
      for (const auto &event : debug_view_->Inputs()) {
        const auto &context = std::any_cast<const WebContext&>(event.context);
        if (context.revision != web_revision_) continue;
        const auto &input=event.input;
        if (input.kind == vision::DebugMjpegServer::Input::Kind::Click && input.button==0)
          MeasureAt(context.frame,{input.x,input.y});
        else if (input.kind == vision::DebugMjpegServer::Input::Kind::Key) HandleKey(input.key);
      }
      const auto frame=frozen_ ? frozen_ : latest_;
      cv::Mat display;
      if (frame) display=frame->color.clone();
      else display=cv::Mat::zeros(480,640,CV_8UC3);
      if (!frame) {
        PutLine(display,"Waiting for color + aligned depth...",0);
        PutLine(display,"Enable align_depth.enable in the camera driver.",1);
      } else {
        PutLine(display,frozen_ ? "FROZEN | click: remeasure | SPACE/c: live"
                              : "LIVE | click: freeze + measure 5x5 median Z",0);
        char text[192];
        std::snprintf(text,sizeof(text),"stamp=%.6f s | color/depth dt=%.3f ms | q/Esc: exit",
            frame->stamp_ns*1e-9,frame->skew_ms);
        PutLine(display,text,1);
        if (measurement_) {
          const auto& goal=*measurement_;
          const auto& m=goal.point_depth;
          cv::rectangle(display,m.roi,{0,255,255},1);
          cv::drawMarker(display,m.pixel,{0,255,255},cv::MARKER_CROSS,15,1);
          std::snprintf(text,sizeof(text),"pixel=(%d,%d) | 5x5 valid=%zu/%d%s",
              m.pixel.x,m.pixel.y,m.valid_pixels,m.roi.area(),m.clipped ? " CLIPPED" : "");
          PutLine(display,text,2);
          if (m.median_m) {
            std::snprintf(text,sizeof(text),"Median Z = %.4f m = %.1f mm",*m.median_m,*m.median_m*1000);
            PutLine(display,text,3);
          } else {
            PutLine(display,"Median Z = NO VALID DEPTH",3);
          }
          const cv::Rect regions[]={goal.left_roi,goal.center_roi,goal.right_roi};
          const char* labels[]={"L (-30px)","B (3x3)","R (+30px)"};
          const cv::Scalar colors[]={{255,180,0},{255,255,255},{0,220,255}};
          for (int i=0;i<3;++i) {
            if (regions[i].empty()) continue;
            cv::rectangle(display,regions[i],colors[i],1);
            const cv::Point label(std::clamp(regions[i].x-10,0,std::max(0,display.cols-100)),
                std::clamp(regions[i].y+24,12,std::max(12,display.rows-1)));
            cv::putText(display,labels[i],label,cv::FONT_HERSHEY_SIMPLEX,.4,colors[i],1,cv::LINE_AA);
          }
          if (goal.point.valid) {
            std::snprintf(text,sizeof(text),"CLICK X/Z = (%+.4f, %.4f) m",goal.point.x_m,goal.point.z_m);
            PutLine(display,text,4);
          } else PutLine(display,"CLICK X/Z = unavailable | "+
              (frame->intrinsics ? std::string("NO VALID CLICK DEPTH") : frame->calibration_status),4);
          if (goal.rim.valid) {
            std::snprintf(text,sizeof(text),"RIM X/Z = (%+.4f, %.4f) m | phi=%+.2f deg",
                goal.rim.x_m,goal.rim.z_m,goal.board.yaw_rad*180.0/std::acos(-1.0));
            PutLine(display,text,5);
          } else PutLine(display,"RIM = unavailable | "+goal.status,5);
        }
      }
      debug_view_->Present(display, WebContext{frame,web_revision_}, web_revision_);
      displayed_=frame;
      if (!debug_view_->Local()) return;
      cv::imshow(kWindow,display);
      cv::imshow(kTopView,camera_overlay::DrawGoalMeasurementTopView(measurement_,config_.hoop_radius_m));
      HandleKey(cv::waitKey(1));
    } catch (const cv::Exception& error) {
      RCLCPP_ERROR(get_logger(),"Display failed: %s",error.what());
      rclcpp::shutdown();
    }
  }
  void HandleKey(int key) {
      if (key==27 || key=='q' || key=='Q') rclcpp::shutdown();
      if (key==' ' || key=='c' || key=='C') {
        ++web_revision_;
        frozen_.reset();
        measurement_.reset();
      }
  }

  std::unique_ptr<camera_overlay::DebugView> debug_view_;
  std::uint64_t web_revision_{0};
  message_filters::Subscriber<Image> color_sub_,depth_sub_;
  std::shared_ptr<Synchronizer> synchronizer_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_;
  camera_overlay::GoalMeasurementConfig config_;
  std::shared_ptr<const Frame> latest_,displayed_,frozen_;
  std::optional<camera_overlay::GoalMeasurement> measurement_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  int result=0;
  try {
    rclcpp::spin(std::make_shared<ClickDepthNode>());
  } catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("click_depth"),"%s",error.what());
    result=1;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return result;
}
