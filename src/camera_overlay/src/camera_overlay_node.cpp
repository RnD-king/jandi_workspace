#include "camera_overlay/debug_view.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

class CameraOverlayNode final : public rclcpp::Node {
public:
  CameraOverlayNode() : Node("camera_overlay") {
    rcl_interfaces::msg::ParameterDescriptor topic_descriptor;
    topic_descriptor.description="Existing 640x480 image topic; set at startup.";
    topic_descriptor.read_only=true;
    image_topic_=declare_parameter<std::string>(
        "image_topic","/camera/camera/color/image_raw",topic_descriptor);
    if (image_topic_.empty()) throw std::invalid_argument("image_topic must not be empty");
    declare_parameter<int>("box_width_px",160,SizeDescriptor("Rectangle width in pixels",kWidth));
    declare_parameter<int>("box_height_px",120,SizeDescriptor("Rectangle height in pixels",kHeight));
    rcl_interfaces::msg::ParameterDescriptor alpha_descriptor;
    alpha_descriptor.description="Red fill opacity: 0.0 transparent, 1.0 opaque.";
    declare_parameter<double>("box_alpha",0.35,alpha_descriptor);
    const auto initial=ValidateParameters(get_parameters({"box_width_px","box_height_px","box_alpha"}));
    if (!initial.successful) throw std::invalid_argument(initial.reason);
    parameter_callback_=add_on_set_parameters_callback(
        [](const std::vector<rclcpp::Parameter>& parameters) {return ValidateParameters(parameters);});

    // All callbacks and HighGUI events run on the main thread via spin().
    subscription_=create_subscription<sensor_msgs::msg::Image>(
        image_topic_,rclcpp::SensorDataQoS().keep_last(1),
        [this](sensor_msgs::msg::Image::ConstSharedPtr message) {OnImage(message);});
    debug_view_ = std::make_unique<camera_overlay::DebugView>(*this);
    if (debug_view_->Local()) {
      cv::namedWindow(kWindow,cv::WINDOW_AUTOSIZE | cv::WINDOW_GUI_NORMAL);
      cv::setMouseCallback(kWindow,&CameraOverlayNode::OnMouse,this);
    }
    timer_=create_wall_timer(std::chrono::milliseconds(33),[this]() {DrawFrame();});
    RCLCPP_INFO(get_logger(),"Subscribed to %s (640x480). Click to move the box; Esc/q to exit.",
        image_topic_.c_str());
  }

  ~CameraOverlayNode() override {
    if (!debug_view_->Local()) return;
    try {cv::destroyWindow(kWindow);} catch (const cv::Exception&) {}
  }

private:
  static constexpr int kWidth=640,kHeight=480;
  static constexpr const char* kWindow="Camera Rectangle Overlay";

  static rcl_interfaces::msg::ParameterDescriptor SizeDescriptor(
      const std::string& description,int maximum) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description=description;
    rcl_interfaces::msg::IntegerRange range;
    range.from_value=1;range.to_value=maximum;range.step=1;
    descriptor.integer_range.push_back(range);
    return descriptor;
  }

  static rcl_interfaces::msg::SetParametersResult ValidateParameters(
      const std::vector<rclcpp::Parameter>& parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful=true;
    // Validate only: read committed parameters while drawing, so a rejected
    // batch never changes the visible rectangle.
    for (const auto& parameter:parameters) {
      const auto& name=parameter.get_name();
      if (name=="box_width_px" || name=="box_height_px") {
        const int maximum=name=="box_width_px" ? kWidth : kHeight;
        if (parameter.get_type()!=rclcpp::ParameterType::PARAMETER_INTEGER ||
            parameter.as_int()<1 || parameter.as_int()>maximum) {
          result.successful=false;
          result.reason=name+" must be an integer in [1,"+std::to_string(maximum)+"]";
          return result;
        }
      } else if (name=="box_alpha") {
        if (parameter.get_type()!=rclcpp::ParameterType::PARAMETER_DOUBLE ||
            !std::isfinite(parameter.as_double()) || parameter.as_double()<0 || parameter.as_double()>1) {
          result.successful=false;
          result.reason="box_alpha must be a finite double in [0.0,1.0]";
          return result;
        }
      }
    }
    return result;
  }

  void OnImage(const sensor_msgs::msg::Image::ConstSharedPtr& message) {
    if (message->width!=kWidth || message->height!=kHeight) {
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),3000,
          "Expected 640x480, received %ux%u; frame ignored (no resizing).",
          message->width,message->height);
      return;
    }
    try {
      latest_frame_=cv_bridge::toCvShare(message,"bgr8")->image.clone();
    } catch (const cv_bridge::Exception& error) {
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),3000,"Image conversion failed: %s",error.what());
    }
  }

  static void OnMouse(int event,int x,int y,int,void* userdata) {
    auto* node=static_cast<CameraOverlayNode*>(userdata);
    if (event!=cv::EVENT_LBUTTONDOWN || node->latest_frame_.empty() ||
        x<0 || x>=kWidth || y<0 || y>=kHeight) return;
    node->center_={x,y};
    ++node->web_revision_;
    RCLCPP_INFO(node->get_logger(),"Box center: (%d,%d) px",x,y);
  }

  void DrawFrame() {
    try {
      for (const auto &event : debug_view_->Inputs()) {
        if (std::any_cast<std::uint64_t>(event.context) != web_revision_) continue;
        const auto &input = event.input;
        if (input.kind == vision::DebugMjpegServer::Input::Kind::Click && input.button == 0)
          OnMouse(cv::EVENT_LBUTTONDOWN, input.x, input.y, 0, this);
        else if (input.kind == vision::DebugMjpegServer::Input::Kind::Key &&
                 (input.key == 27 || input.key == 'q' || input.key == 'Q')) rclcpp::shutdown();
      }
      cv::Mat display;
      if (latest_frame_.empty()) {
        display=cv::Mat::zeros(kHeight,kWidth,CV_8UC3);
        cv::putText(display,"Waiting for 640x480 image...",{16,40},
            cv::FONT_HERSHEY_SIMPLEX,0.65,{255,255,255},1,cv::LINE_AA);
      } else {
        display=latest_frame_.clone();
        const int width=static_cast<int>(get_parameter("box_width_px").as_int());
        const int height=static_cast<int>(get_parameter("box_height_px").as_int());
        const double alpha=get_parameter("box_alpha").as_double();
        const cv::Rect requested(center_.x-width/2,center_.y-height/2,width,height);
        const cv::Rect visible=requested & cv::Rect(0,0,kWidth,kHeight);
        if (!visible.empty()) {
          cv::Mat region=display(visible);
          const cv::Mat red(visible.size(),CV_8UC3,cv::Scalar(0,0,255));
          cv::addWeighted(region,1.0-alpha,red,alpha,0.0,region);
          cv::rectangle(display,visible,{0,0,255},1);
        }
        cv::drawMarker(display,center_,{255,255,255},cv::MARKER_CROSS,11,1,cv::LINE_AA);
        char text[128];
        std::snprintf(text,sizeof(text),"BOX %dx%d px | center=(%d,%d) | alpha=%.2f%s",
            width,height,center_.x,center_.y,alpha,visible==requested ? "" : " | CLIPPED");
        cv::putText(display,text,{10,22},cv::FONT_HERSHEY_SIMPLEX,0.48,{0,0,0},3,cv::LINE_AA);
        cv::putText(display,text,{10,22},cv::FONT_HERSHEY_SIMPLEX,0.48,{255,255,255},1,cv::LINE_AA);
      }
      debug_view_->Present(display, web_revision_, web_revision_);
      if (!debug_view_->Local()) return;
      cv::imshow(kWindow,display);
      const int key=cv::waitKey(1);
      // Some HighGUI backends report an unmapped/unsupported window as not
      // visible at startup. Only an explicit exit key should stop the node.
      if (key==27 || key=='q' || key=='Q') {
        rclcpp::shutdown();
      }
    } catch (const cv::Exception& error) {
      RCLCPP_ERROR(get_logger(),"Display failed: %s",error.what());
      rclcpp::shutdown();
    }
  }

  std::string image_topic_;
  cv::Point center_{kWidth/2,kHeight/2};
  cv::Mat latest_frame_;
  std::unique_ptr<camera_overlay::DebugView> debug_view_;
  std::uint64_t web_revision_{0};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};

int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  int result=0;
  try {
    rclcpp::spin(std::make_shared<CameraOverlayNode>());
  } catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("camera_overlay"),"%s",error.what());
    result=1;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return result;
}
