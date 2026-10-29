#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_map.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include "vision/msg/action_command.hpp"
#include "vision/msg/command_status.hpp"
#include "vision/msg/camera_command.hpp"
#include "vision/msg/shoot_tuning_pose.hpp"
#include "vision/yolo_trt_engine.hpp"
#include "vision/object_depth.hpp"
#include "vision/shoot_tuning.hpp"
#include "vision_core/config_loader.hpp"
#include "vision_core/control_command.hpp"

#include <QApplication>
#include <QCloseEvent>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QLabel>
#include <QMouseEvent>
#include <QPainter>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QSplitter>
#include <QTabWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <sys/utsname.h>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <map>
#include <memory>
#include <stdexcept>

namespace vision {
namespace {
using Clock=std::chrono::steady_clock;
using vision_core::MissionAction;
using vision_core::ActionCategory;

// Render and hit-test the same letterboxed rectangle, including after resize.
class ImageWidget final : public QWidget {
public:
  explicit ImageWidget(QWidget* parent=nullptr) : QWidget(parent) {
    setMinimumSize(320,240);
  }
  std::function<void(int,int)> clicked;
  void SetImage(const cv::Mat& bgr) {
    if (bgr.empty()) { image_=QImage(); update(); return; }
    cv::Mat rgb;
    cv::cvtColor(bgr,rgb,cv::COLOR_BGR2RGB);
    image_=QImage(rgb.data,rgb.cols,rgb.rows,static_cast<int>(rgb.step),
                  QImage::Format_RGB888).copy();
    update();
  }
protected:
  void paintEvent(QPaintEvent*) override {
    QPainter painter(this);
    painter.fillRect(rect(),QColor(24,24,24));
    if (!image_.isNull()) painter.drawImage(ImageRect(),image_);
  }
  void mousePressEvent(QMouseEvent* event) override {
    const auto area=ImageRect();
    if (event->button()!=Qt::LeftButton || image_.isNull() ||
        !area.contains(event->pos()) || !clicked) return;
    const int u=(event->pos().x()-area.x())*image_.width()/area.width();
    const int v=(event->pos().y()-area.y())*image_.height()/area.height();
    clicked(u,v);
  }
private:
  QRect ImageRect() const {
    if (image_.isNull()) return {};
    const auto size=image_.size().scaled(this->size(),Qt::KeepAspectRatio);
    return {(width()-size.width())/2,(height()-size.height())/2,
            size.width(),size.height()};
  }
  QImage image_;
};

struct Setting {
  const char* name;
  const char* label;
  double* value;
  double low,high,step;
};
std::vector<Setting> Settings(vision_core::MissionControllerConfig& c) {
  return {
    {"conf_thres","YOLO 1차 confidence 하한",&c.line_detection.confidence,0,1,.01},
    {"goal_hoop_radius_m","림 반지름 (m)",&c.goal.hoop_radius_m,0,2,.005},
    {"goal_throw_landing_x_m","슛 도달점 X / 오른쪽 (m)",&c.goal.throw_landing_x_m,-10,10,.01},
    {"goal_throw_landing_z_m","슛 도달점 Y / 전방 (m)",&c.goal.throw_landing_z_m,.001,10,.01},
    {"goal_position_tolerance_m","슛 위치 허용오차 (m)",&c.goal.position_tolerance_m,0,2,.005},
    {"goal_shoot_yaw_limit_deg","허리 회전 한계 (deg)",&c.goal.shoot_yaw_limit_deg,0,180,1},
    {"backboard_conf_thres","백보드 confidence 하한",&c.object_targets.backboard_confidence,0,1,.01},
    {"goal_conf_thres","골대 confidence 하한",&c.object_targets.goal_confidence,0,1,.01},
    {"backboard_min_box_width","백보드 최소 너비 (px)",&c.object_targets.backboard_min_box_width,0,4096,1},
    {"backboard_min_box_height","백보드 최소 높이 (px)",&c.object_targets.backboard_min_box_height,0,4096,1},
    {"goal_min_box_width","골대 최소 너비 (px)",&c.object_targets.goal_min_box_width,0,4096,1},
    {"goal_min_box_height","골대 최소 높이 (px)",&c.object_targets.goal_min_box_height,0,4096,1},
    {"backboard_min_depth_m","백보드 깊이 하한 (m)",&c.backboard_min_depth_m,.001,10,.01},
    {"backboard_max_depth_m","백보드 깊이 상한 (m)",&c.backboard_max_depth_m,.001,10,.01},
  };
}

std::string DefaultEnginePath() {
  if (const char* path=std::getenv("YOLO26_ENGINE_PATH"); path && *path) return path;
  std::string path=ament_index_cpp::get_package_share_directory("vision")+
      "/config/yolo26_runtime.yaml";
  if (const char* env=std::getenv("YOLO26_RUNTIME_CONFIG"); env && *env) path=env;
  struct utsname machine{};
  const bool jetson=uname(&machine)==0 &&
      (std::string(machine.machine)=="aarch64" || std::string(machine.machine)=="arm64");
  const auto entries=rclcpp::parameter_map_from_yaml_file(path);
  for (const auto& entry:entries)
    for (const auto& parameter:entry.second)
      if (parameter.get_name()==(jetson ? "jetson_engine_path" : "pc_engine_path"))
        return parameter.as_string();
  throw std::runtime_error("YOLO26 runtime config has no engine path");
}

double StampSeconds(const std_msgs::msg::Header& header) {
  return header.stamp.sec+1e-9*header.stamp.nanosec;
}

struct Frame {
  cv::Mat bgr,depth;
  std::vector<Detection> detections;
  std_msgs::msg::Header header;
  vision_core::PerceptionFrameInput input;
  vision_core::PerceptionMissionFrameResult result;
  vision_core::MissionControllerConfig config;
  Clock::time_point received;
  bool valid{false},camera_info_valid{false};
};
} // namespace

class ShootTuningNode final : public rclcpp::Node {
public:
  ShootTuningNode() : Node("shoot_tuning_node"),
      config_(vision_core::LoadDefaultAlgorithmConfig()) {
    // Bench mode observes a single frame; it never enters the mission FSM.
    // PreviewPerception works on a disposable copy with command transport off.
    config_.enable_ball=config_.enable_hurdle=config_.enable_goal=false;
    config_.initial_has_ball=config_.goal_trial_start_goal=false;
    config_.test_mission=vision_core::MissionType::kNone;
    config_.test_stage=0;
    for (const auto& setting:Settings(config_)) {
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.description=setting.label;
      rcl_interfaces::msg::FloatingPointRange range;
      range.from_value=setting.low; range.to_value=setting.high;
      descriptor.floating_point_range.push_back(range);
      *setting.value=declare_parameter<double>(setting.name,*setting.value,descriptor);
    }
    config_.backboard_depth_strip_width_px=ReadOnly<int>(
        "backboard_depth_strip_width_px",config_.backboard_depth_strip_width_px);
    click_yaw_deg_=declare_parameter<double>("click_backboard_yaw_deg",0.0);
    prepare_duration_sec_=declare_parameter<double>("shoot_prepare_duration_sec",2.0);
    if (!std::isfinite(prepare_duration_sec_) || prepare_duration_sec_<0.2 || prepare_duration_sec_>60)
      throw std::invalid_argument("shoot_prepare_duration_sec must be in [0.2,60]");
    ValidateSettings(config_,click_yaw_deg_);

    const auto image_topic=ReadOnly<std::string>("image_topic","/camera/camera/color/image_raw");
    const auto depth_topic=ReadOnly<std::string>("depth_topic","/camera/camera/aligned_depth_to_color/image_raw");
    const auto info_topic=ReadOnly<std::string>("camera_info_topic","/camera/camera/color/camera_info");
    const auto imu_topic=ReadOnly<std::string>("imu_topic","/camera/imu_tilt");
    const auto cmd_topic=ReadOnly<std::string>("action_cmd_topic","/jandi_vision/action_cmd");
    const auto status_topic=ReadOnly<std::string>("action_status_topic","/jandi_vision/action_status");
    const auto pose_topic=ReadOnly<std::string>("shoot_pose_cmd_topic","/jandi_vision/shoot_tuning_pose");
    const auto camera_topic=ReadOnly<std::string>("camera_cmd_topic","/jandi_vision/camera_cmd");
    const auto camera_status_topic=ReadOnly<std::string>("camera_status_topic","/jandi_vision/camera_status");
    max_depth_age_=ReadOnly<double>("max_depth_age_sec",.20);
    max_frame_age_=ReadOnly<double>("max_frame_age_sec",.50);
    const double hz=ReadOnly<double>("inference_hz",15.0);
    use_imu_=ReadOnly<bool>("use_imu_rectification",false);
    assume_zero_imu_=ReadOnly<bool>("assume_zero_imu",false);
    imu_limit_=ReadOnly<double>("imu_abs_limit_deg",45.0)*kDebugPi/180;
    if (!std::isfinite(hz) || hz<=0 || hz>120 ||
        !std::isfinite(max_depth_age_) || max_depth_age_<0 ||
        !std::isfinite(max_frame_age_) || max_frame_age_<=0 ||
        !std::isfinite(imu_limit_) || imu_limit_<0)
      throw std::invalid_argument("Invalid inference rate, frame/depth age, or IMU limit");
    const auto engine=ReadOnly<std::string>("engine_path",DefaultEnginePath());
    yolo_=std::make_unique<YoloTrtEngine>(engine,640,640,
        static_cast<float>(config_.line_detection.confidence));
    observer_=std::make_unique<vision_core::MissionController>(config_);
    auto command_config=config_.command;
    command_config.first_action_id=static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count());
    transport_=std::make_unique<vision_core::ControlCommandCoordinator>(command_config);
    next_camera_id_=command_config.first_action_id;
    action_pub_=create_publisher<msg::ActionCommand>(cmd_topic,10);
    pose_pub_=create_publisher<msg::ShootTuningPose>(pose_topic,10);
    camera_pub_=create_publisher<msg::CameraCommand>(camera_topic,10);
    image_sub_=create_subscription<sensor_msgs::msg::Image>(image_topic,
        rclcpp::SensorDataQoS().keep_last(1),[this](sensor_msgs::msg::Image::SharedPtr image) {
          latest_image_=image; latest_received_=Clock::now();
        });
    depth_sub_=create_subscription<sensor_msgs::msg::Image>(depth_topic,
        rclcpp::SensorDataQoS().keep_last(1),[this](sensor_msgs::msg::Image::SharedPtr depth) {
          latest_depth_=depth;
        });
    info_sub_=create_subscription<sensor_msgs::msg::CameraInfo>(info_topic,
        rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::CameraInfo::SharedPtr info) {
          const vision_core::Intrinsics intrinsics{info->k[0],info->k[4],info->k[2],info->k[5]};
          if (std::isfinite(intrinsics.fx) && intrinsics.fx>0 &&
              std::isfinite(intrinsics.fy) && intrinsics.fy>0 &&
              std::isfinite(intrinsics.cx) && std::isfinite(intrinsics.cy)) {
            intrinsics_=intrinsics; info_width_=info->width; info_height_=info->height;
          }
        });
    imu_sub_=create_subscription<geometry_msgs::msg::Vector3Stamped>(imu_topic,
        rclcpp::SensorDataQoS(),[this](geometry_msgs::msg::Vector3Stamped::SharedPtr imu) {
          imu_valid_=std::isfinite(imu->vector.x) && std::isfinite(imu->vector.y);
          if (imu_valid_) {
            roll_=std::clamp(imu->vector.x,-imu_limit_,imu_limit_);
            pitch_=std::clamp(imu->vector.y,-imu_limit_,imu_limit_);
          }
        });
    status_sub_=create_subscription<msg::CommandStatus>(status_topic,10,
        [this](msg::CommandStatus::SharedPtr status) { OnStatus(*status); });
    camera_status_sub_=create_subscription<msg::CommandStatus>(camera_status_topic,10,
        [this](msg::CommandStatus::SharedPtr status) { OnCameraStatus(*status); });
    parameter_callback_=add_on_set_parameters_callback([this](const auto& parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful=false;
      if (MotionBusy()) {
        result.reason="Wait for executor DONE before changing parameters";
        return result;
      }
      try {
        auto candidate=config_;
        double click_yaw=click_yaw_deg_;
        double prepare_duration=prepare_duration_sec_;
        for (const auto& parameter:parameters) {
          bool found=false;
          for (const auto& setting:Settings(candidate))
            if (parameter.get_name()==setting.name) {
              *setting.value=parameter.as_double(); found=true; break;
            }
          if (parameter.get_name()=="click_backboard_yaw_deg") {
            click_yaw=parameter.as_double(); found=true;
          }
          if (parameter.get_name()=="shoot_prepare_duration_sec") {
            prepare_duration=parameter.as_double(); found=true;
          }
          if (!found) throw std::invalid_argument("Parameter is not live-tunable: "+parameter.get_name());
        }
        ValidateSettings(candidate,click_yaw);
        if (!std::isfinite(prepare_duration) || prepare_duration<0.2 || prepare_duration>60)
          throw std::invalid_argument("shoot_prepare_duration_sec must be in [0.2,60]");
        auto observer=std::make_unique<vision_core::MissionController>(candidate);
        config_=candidate; click_yaw_deg_=click_yaw;
        prepare_duration_sec_=prepare_duration;
        yolo_->SetConfidenceThreshold(static_cast<float>(config_.line_detection.confidence));
        observer_=std::move(observer);
        ++parameter_revision_;
        result.successful=true;
      } catch (const std::exception& error) { result.reason=error.what(); }
      return result;
    });
    BuildUi();
    frame_timer_=create_wall_timer(std::chrono::duration<double>(1.0/hz),
        [this]() { ProcessLatestImage(); });
    transport_timer_=create_wall_timer(std::chrono::milliseconds(100),[this]() {
      if (transport_->HasPendingAction() && !ReleaseUnavailableRequest()) AdvanceTransport({});
      PollCamera();
      UpdateUi();
    });
    window_->show();
    RCLCPP_INFO(get_logger(),"Shoot bench ready; no action is sent until a button is pressed");
  }

private:
  template<typename T> T ReadOnly(const std::string& name,const T& value) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only=true;
    return declare_parameter<T>(name,value,descriptor);
  }
  static void ValidateSettings(vision_core::MissionControllerConfig& config,double click_yaw) {
    for (const auto& setting:Settings(config))
      if (!std::isfinite(*setting.value) || *setting.value<setting.low || *setting.value>setting.high)
        throw std::invalid_argument(std::string(setting.name)+" is outside its allowed range");
    if (!std::isfinite(click_yaw) || std::abs(click_yaw)>180)
      throw std::invalid_argument("click_backboard_yaw_deg must be in [-180,180]");
    if (config.backboard_min_depth_m>=config.backboard_max_depth_m)
      throw std::invalid_argument("Backboard depth minimum must be less than maximum");
    vision_core::ValidateAlgorithmConfig(config);
  }
  static QLabel* Text(const QString& text,QVBoxLayout* layout) {
    auto* label=new QLabel(text);
    label->setWordWrap(true);
    label->setTextInteractionFlags(Qt::TextSelectableByMouse);
    layout->addWidget(label);
    return label;
  }
  void BuildUi() {
    window_=std::make_unique<QWidget>();
    window_->setWindowTitle("Shoot tuning — 단발 슛 테스트");
    auto* layout=new QHBoxLayout(window_.get());
    auto* splitter=new QSplitter;
    layout->addWidget(splitter);
    auto* tabs=new QTabWidget;
    live_view_=new ImageWidget;
    frozen_view_=new ImageWidget;
    state_view_=new ImageWidget;
    tabs->addTab(live_view_,"실시간 디버그");
    tabs->addTab(frozen_view_,"계산에 사용한 프레임");
    tabs->addTab(state_view_,"상태 디버그");
    live_view_->clicked=[this](int u,int v) { ClickPixel(live_,u,v); };
    frozen_view_->clicked=[this](int u,int v) { ClickPixel(frozen_,u,v); };
    splitter->addWidget(tabs);

    auto* controls=new QWidget;
    auto* buttons=new QVBoxLayout(controls);
    Text("좌표: 카메라 원점, X=오른쪽 / Y=전방 (m)\n기존 코어의 (x,z)를 (X,Y)로 표시",buttons);
    calculate_button_=new QPushButton("계산하기"); buttons->addWidget(calculate_button_);
    calculation_text_=Text("골대 중심 (X, Y): —",buttons);
    action_text_=Text("다음 동작: —",buttons);
    auto* shoot=new QPushButton("슛"); buttons->addWidget(shoot); motion_buttons_.push_back(shoot);
    auto* pose=new QPushButton("기본자세"); buttons->addWidget(pose); motion_buttons_.push_back(pose);
    stored_shoot_button_=new QPushButton("계산된 값에서 슛");
    buttons->addWidget(stored_shoot_button_);
    status_text_=Text("대기 — 버튼을 누르면 한 동작만 실행",buttons);
    Text("픽셀 클릭: 해당 프레임의 5×5 유효 깊이 중앙값",buttons);
    click_text_=Text("픽셀 / 깊이 / 가정한 골대 중심: —",buttons);
    adopt_click_button_=new QPushButton("클릭 결과를 계산 결과로 사용");
    buttons->addWidget(adopt_click_button_);
    auto* manual_group=new QGroupBox("골대 중심 탑뷰 좌표 직접 입력");
    auto* manual_layout=new QVBoxLayout(manual_group);
    auto* manual_form=new QFormLayout;
    manual_layout->addLayout(manual_form);
    manual_x_=new QDoubleSpinBox;
    manual_y_=new QDoubleSpinBox;
    for (auto* spin:{manual_x_,manual_y_}) {
      spin->setDecimals(4); spin->setRange(-1000000,1000000);
      spin->setSingleStep(.01); spin->setKeyboardTracking(false);
    }
    manual_x_->setValue(config_.goal.throw_landing_x_m);
    manual_y_->setValue(config_.goal.throw_landing_z_m);
    manual_form->addRow("X / 오른쪽 (m)",manual_x_);
    manual_form->addRow("Y / 전방 (m, > 0)",manual_y_);
    auto* adopt_manual=new QPushButton("입력 좌표를 계산 결과로 사용");
    manual_layout->addWidget(adopt_manual);
    target_edit_widgets_={manual_x_,manual_y_,adopt_manual};
    buttons->addWidget(manual_group);
    buttons->addStretch();
    controls->setMinimumWidth(300);
    auto* controls_scroll=new QScrollArea;
    controls_scroll->setWidgetResizable(true); controls_scroll->setWidget(controls);
    controls_scroll->setMinimumWidth(340);
    splitter->addWidget(controls_scroll);
    QObject::connect(calculate_button_,&QPushButton::clicked,[this]() { Calculate(); });
    QObject::connect(shoot,&QPushButton::clicked,[this]() {
      SendAction({MissionAction::kShoot,ActionCategory::kMission,0,false});
    });
    QObject::connect(pose,&QPushButton::clicked,[this]() {
      SendAction({MissionAction::kDefaultPoseMode,ActionCategory::kMission,0,false});
    });
    QObject::connect(stored_shoot_button_,&QPushButton::clicked,[this]() { ShootStoredResult(); });
    QObject::connect(adopt_click_button_,&QPushButton::clicked,[this]() {
      if (!clicked_target_) return;
      StoreCalculatedTarget(*clicked_target_,"픽셀 클릭",clicked_details_,&clicked_frame_);
    });
    QObject::connect(adopt_manual,&QPushButton::clicked,[this]() {
      manual_x_->interpretText(); manual_y_->interpretText();
      const GoalCenterTarget target{manual_x_->value(),manual_y_->value()};
      if (!target.Valid()) {
        status_text_->setText("입력 좌표를 적용하지 않았습니다. 유한한 X와 양수 Y를 입력해 주세요.");
        return;
      }
      StoreCalculatedTarget(target,"직접 입력","직접 입력한 골대 중심 좌표");
    });

    auto* parameter_widget=new QWidget;
    auto* parameter_layout=new QVBoxLayout(parameter_widget);
    Text("슛 / 골대 파라미터\n편집 후 Enter 또는 포커스 이동으로 적용\nrqt에서도 동일한 ROS 파라미터 조절 가능",parameter_layout);
    auto* form=new QFormLayout;
    parameter_layout->addLayout(form);
    const auto add_setting=[&](const char* name,const char* label,double value,
                               double low,double high,double step) {
      auto* spin=new QDoubleSpinBox;
      spin->setDecimals(4); spin->setRange(low,high); spin->setSingleStep(step);
      spin->setKeyboardTracking(false); spin->setValue(value);
      spin->setToolTip(name);
      form->addRow(QString::fromUtf8(label),spin);
      parameter_widgets_[name]=spin;
      QObject::connect(spin,&QDoubleSpinBox::editingFinished,[this,spin,name]() {
        const auto result=set_parameters_atomically({rclcpp::Parameter(name,spin->value())});
        if (!result.successful) status_text_->setText(QString::fromStdString(result.reason));
        SyncParameters(true);
      });
    };
    for (const auto& setting:Settings(config_))
      add_setting(setting.name,setting.label,*setting.value,setting.low,setting.high,setting.step);
    add_setting("click_backboard_yaw_deg","클릭 시 가정할 백보드 yaw (deg)",
                click_yaw_deg_,-180,180,1);
    add_setting("shoot_prepare_duration_sec","기본자세 복귀 이동 시간 (s)",
                prepare_duration_sec_,.2,60,.1);
    Text("슛 도달점은 측정한 착탄 위치를 입력합니다.\n계산된 값에서 슛은 저장한 골대 좌표와 현재 슛 파라미터를 사용합니다.\n픽셀 클릭의 yaw는 클릭 시점의 가정값입니다.",parameter_layout);
    parameter_layout->addStretch();
    auto* scroll=new QScrollArea;
    scroll->setWidgetResizable(true); scroll->setWidget(parameter_widget);
    scroll->setMinimumWidth(340);
    splitter->addWidget(scroll);
    splitter->setStretchFactor(0,1);
    window_->resize(1450,800);
    UpdateUi();
  }

  void SyncParameters(bool force=false) {
    for (const auto& entry:parameter_widgets_) {
      if (!force && entry.second->hasFocus()) continue;
      const QSignalBlocker blocker(entry.second);
      entry.second->setValue(get_parameter(entry.first).as_double());
    }
  }
  void UpdateUi() {
    const bool busy=MotionBusy();
    for (auto* button:motion_buttons_) button->setEnabled(!busy);
    calculate_button_->setEnabled(true);
    stored_shoot_button_->setEnabled(!busy && calculated_target_.has_value());
    adopt_click_button_->setEnabled(clicked_target_.has_value());
    for (auto* widget:target_edit_widgets_) widget->setEnabled(true);
    for (const auto& entry:parameter_widgets_) entry.second->setEnabled(!busy);
    SyncParameters();
    RefreshStoredResult();
  }

  void ProcessLatestImage() {
    if (!latest_image_ || latest_image_==processed_image_) return;
    const auto image=latest_image_;
    processed_image_=image;
    try {
      Frame frame;
      frame.received=latest_received_; frame.header=image->header; frame.config=config_;
      frame.bgr=cv_bridge::toCvShare(image,"bgr8")->image.clone();
      if (frame.bgr.empty()) return;
      if (!yolo_->Infer(frame.bgr,frame.detections)) {
        live_.valid=false;
        status_text_->setText("YOLO 추론 실패 — 다음 프레임 대기"); return;
      }
      // Sample aligned depth even without a YOLO board, to support any pixel click.
      if (latest_depth_ && std::abs(StampSeconds(image->header)-
          StampSeconds(latest_depth_->header))<=max_depth_age_) {
        const auto depth=cv_bridge::toCvShare(latest_depth_)->image;
        if (depth.size()==frame.bgr.size() && (depth.type()==CV_16UC1 || depth.type()==CV_32FC1))
          frame.depth=depth.clone();
      }
      frame.camera_info_valid=info_width_==image->width && info_height_==image->height;
      auto& input=frame.input;
      input.image_width=frame.bgr.cols; input.image_height=frame.bgr.rows;
      input.intrinsics=intrinsics_;
      input.enable_imu_rectification=use_imu_;
      input.imu_valid=assume_zero_imu_ || imu_valid_;
      input.roll_rad=assume_zero_imu_ ? 0 : roll_;
      input.pitch_rad=assume_zero_imu_ ? 0 : pitch_;
      input.now_sec=get_clock()->now().seconds();
      input.camera_feedback={vision_core::CameraMode::kGoal,true};
      input.command_transport_enabled=false;
      input.allow_new_commands=false;
      input.allow_new_mission_entry=false;
      input.advance_line_fsm=false;
      for (const auto& detection:frame.detections) {
        vision_core::PerceptionDetection observation;
        observation.detection={{static_cast<double>(detection.box.x),
            static_cast<double>(detection.box.y),static_cast<double>(detection.box.width),
            static_cast<double>(detection.box.height)},detection.confidence,detection.class_id};
        if (detection.class_id==config_.object_targets.backboard_class_id) {
          const auto region=DepthRegions(detection.box,frame.bgr.size(),config_.backboard_depth_strip_width_px);
          observation.center_depth_m=MedianRegionMeters(frame.depth,region.center);
          observation.left_depth_m=MedianRegionMeters(frame.depth,region.left);
          observation.right_depth_m=MedianRegionMeters(frame.depth,region.right);
          if (!region.left.empty()) observation.left_depth_u_px=region.left_u;
          if (!region.right.empty()) observation.right_depth_u_px=region.right_u;
        }
        if ((detection.class_id==config_.object_targets.ball_class_id && config_.object_targets.ball_depth_enabled) ||
            (detection.class_id==config_.object_targets.hurdle_class_id && config_.object_targets.hurdle_depth_enabled)) {
          const auto sample=SampleObjectCenterDepth(frame.depth,detection.box,config_.object_targets.depth_roi_size_px);
          observation.center_depth_m=sample.median_m;
          observation.depth_valid_pixels=sample.valid_pixels;
        }
        input.detections.push_back(observation);
      }
      frame.result=observer_->PreviewPerception(input);
      // The bench is always in fine mode, so it also permits measuring boards
      // outside the automatic mission's fine-entry distance gate.
      auto& prepared=frame.result.perception;
      const auto& board=prepared.targets.backboard;
      if (frame.camera_info_valid && board && prepared.backboard_center_depth_m &&
          prepared.backboard_left_depth_m && prepared.backboard_right_depth_m &&
          prepared.backboard_left_depth_u_px && prepared.backboard_right_depth_u_px) {
        prepared.goal_pose=vision_core::EstimateGoalPoseFromBackboardDepths(
            std::round(board->box_px.x+0.5*board->box_px.width),*prepared.backboard_center_depth_m,
            *prepared.backboard_left_depth_u_px,*prepared.backboard_left_depth_m,
            *prepared.backboard_right_depth_u_px,*prepared.backboard_right_depth_m,
            input.intrinsics,board->confidence);
      } else { prepared.goal_pose={}; }
      auto& mission=frame.result.mission;
      // Rendering metadata only; no controller phase is forced or advanced.
      mission.active_mission=vision_core::MissionType::kGoal;
      mission.foot_pose=vision_core::FootPose::kFine;
      mission.goal.mode=vision_core::GoalMode::kFineAdjust;
      mission.goal.fine_decision_pose=prepared.goal_pose;
      mission.goal.fine_decision_pose_used=prepared.goal_pose.valid;
      if (board) {
        mission.goal.tracked.visible=true;
        mission.goal.tracked.center_px=board->center_px;
      }
      if (prepared.goal_pose.valid) {
        const auto& pose=prepared.goal_pose;
        mission.goal.pose={false,true,pose.x_m,pose.z_m,pose.yaw_rad,pose.confidence};
      }
      frame.valid=true;
      live_=std::move(frame);
      live_revision_=parameter_revision_;
      const auto views=Render(live_);
      live_view_->SetImage(views.image); state_view_->SetImage(views.panel);
    } catch (const std::exception& error) {
      live_.valid=false;
      status_text_->setText(QString("프레임 처리 실패: %1").arg(error.what()));
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"Frame error: %s",error.what());
    }
  }

  MissionDebugImages Render(const Frame& frame) const {
    MissionDebugContext context;
    context.control_state=MotionBusy() ? "EXECUTING / WAIT DONE" : "SHOOT BENCH / HOLD";
    context.control_hint="CALCULATE / SHOOT / PREPARE / SHOOT STORED RESULT";
    context.waiting_action_id=transport_->PendingActionId();
    context.last_received_done_id=last_done_id_;
    context.last_processed_done_id=last_done_id_;
    context.imu_enabled=use_imu_; context.imu_ready=frame.input.imu_valid;
    context.roll_rad=frame.input.roll_rad; context.pitch_rad=frame.input.pitch_rad;
    context.performance="SINGLE FRAME / MANUAL";
    return RenderMissionDebug(frame.bgr,frame.detections,frame.depth,frame.config,
        frame.result.perception,frame.result.mission,frame.input.camera_feedback,
        frame.input.intrinsics,context);
  }

  QString MeasurementError() {
    if (pending_camera_id_!=0) return "카메라 정면 복귀 완료를 기다려 주세요.";
    if (!live_.valid) return "계산 가능한 카메라 프레임이 없습니다.";
    const double capture=StampSeconds(live_.header);
    const double age=get_clock()->now().seconds()-capture;
    if (std::chrono::duration<double>(Clock::now()-live_.received).count()>max_frame_age_ ||
        capture<=0 || age>max_frame_age_ || age < -max_depth_age_)
      return "프레임이 오래되었거나 타임스탬프가 유효하지 않습니다.";
    if (live_revision_!=parameter_revision_) return "변경된 파라미터가 적용된 새 프레임을 기다려 주세요.";
    if (capture<=last_done_sec_) return "동작 DONE 이후 촬영된 새 프레임을 기다려 주세요.";
    if (!live_.camera_info_valid) return "영상 크기와 일치하는 CameraInfo가 필요합니다.";
    if (use_imu_ && !live_.input.imu_valid) return "IMU 입력을 기다리고 있습니다.";
    if (live_.depth.empty()) return "RGB와 시간·크기가 일치하는 유효 깊이 영상이 없습니다.";
    if (!live_.result.perception.targets.backboard) return "깊이·confidence·크기 조건을 통과한 백보드가 없습니다.";
    if (!live_.result.perception.goal_pose.valid) return "백보드 좌·중앙·우 깊이로 자세를 계산할 수 없습니다.";
    return {};
  }
  void Calculate() {
    const auto error=MeasurementError();
    if (!error.isEmpty()) {
      calculated_target_.reset();
      calculation_text_->setText("계산 실패: "+error);
      action_text_->setText("다음 동작: 저장된 계산 결과 없음");
      UpdateUi(); return;
    }
    const auto landing=ComputeGoalLandingDebug(live_.config.goal,live_.result.mission);
    const auto target=GoalCenterFromLanding(landing);
    if (!target) {
      calculated_target_.reset();
      calculation_text_->setText("계산 실패: 골대 중심의 유한한 좌표와 양수 Y가 필요합니다.");
      action_text_->setText("다음 동작: 저장된 계산 결과 없음");
      UpdateUi(); return;
    }
    const auto details=QString("백보드 yaw = %1°\n프레임 %2.%3")
        .arg(live_.result.perception.goal_pose.yaw_rad*180/kDebugPi,0,'f',2)
        .arg(live_.header.stamp.sec).arg(live_.header.stamp.nanosec,9,10,QChar('0'));
    StoreCalculatedTarget(*target,"자동 인식",details,&live_);
  }
  void StoreCalculatedTarget(const GoalCenterTarget& target,const QString& source,
                             const QString& details,const Frame* frame=nullptr) {
    if (!target.Valid()) return;
    calculated_target_=target;
    calculated_source_=source; calculated_details_=details;
    if (frame) {
      frozen_=*frame;
      frozen_.result.mission.goal.action=PlanStoredGoalCenter(
          target,frozen_.config.goal,frozen_.config.camera_motion_timeout_sec);
      frozen_view_->SetImage(Render(frozen_).image);
    } else {
      frozen_=Frame();
      frozen_view_->SetImage({});
    }
    status_text_->setText(source+" 결과 저장 — 계산된 값에서 슛 버튼으로 한 동작 실행");
    UpdateUi();
  }
  void RefreshStoredResult() {
    if (!calculated_target_) return;
    calculation_text_->setText(QString("계산 결과 출처: %1\n골대 중심 (X, Y) = (%2, %3) m\n%4")
        .arg(calculated_source_).arg(calculated_target_->x_m,0,'f',4)
        .arg(calculated_target_->y_m,0,'f',4).arg(calculated_details_));
    const auto request=PlanStoredGoalCenter(*calculated_target_,config_.goal,config_.camera_motion_timeout_sec);
    action_text_->setText(QString("저장 좌표에서 다음 한 동작: %1\n허리 각도: %2°\n현재 슛 파라미터 기준 / 새 영상 계산 없음")
        .arg(DebugActionName(request.action)).arg(request.target_yaw_deg));
  }
  void ShootStoredResult() {
    if (MotionBusy()) return;
    if (!calculated_target_) {
      status_text_->setText("먼저 계산하거나 클릭 결과·입력 좌표를 계산 결과로 사용해 주세요.");
      return;
    }
    const auto request=PlanStoredGoalCenter(*calculated_target_,config_.goal,config_.camera_motion_timeout_sec);
    if (request.action==MissionAction::kNone) {
      status_text_->setText("저장된 좌표에서 실행할 동작을 구할 수 없습니다.");
      return;
    }
    SendAction(request);
  }
  void ClickPixel(const Frame& frame,int u,int v) {
    if (!frame.valid) return;
    const auto sample=SampleClickedBackboard(frame.depth,u,v,frame.input.intrinsics,
                                            frame.config.goal,click_yaw_deg_);
    clicked_target_.reset();
    if (frame.camera_info_valid) {
      clicked_target_=GoalCenterFromLanding(sample.landing);
      if (clicked_target_) {
        clicked_frame_=frame;
        clicked_frame_.bgr=frame.bgr.clone();
        cv::drawMarker(clicked_frame_.bgr,{u,v},{255,0,255},cv::MARKER_CROSS,15,2);
        auto& goal=clicked_frame_.result.mission.goal;
        goal.action={};
        goal.fine_decision_pose=sample.board_pose;
        goal.fine_decision_pose_used=sample.board_pose.valid;
        const auto& pose=sample.board_pose;
        goal.pose={false,true,pose.x_m,pose.z_m,pose.yaw_rad,pose.confidence};
        clicked_frame_.result.perception.goal_pose=sample.board_pose;
        clicked_details_=QString("픽셀 (%1, %2) / 가정 yaw = %3°\n프레임 %4.%5")
            .arg(u).arg(v).arg(click_yaw_deg_,0,'f',1)
            .arg(frame.header.stamp.sec).arg(frame.header.stamp.nanosec,9,10,QChar('0'));
      }
    }
    QString result=QString("픽셀 (%1, %2) / 유효 깊이 %3/%4\n프레임 %5.%6\n")
        .arg(u).arg(v).arg(sample.valid_pixels).arg(sample.total_pixels)
        .arg(frame.header.stamp.sec).arg(frame.header.stamp.nanosec,9,10,QChar('0'));
    if (!sample.depth_m) result+="5×5 깊이 중앙값: 유효값 없음";
    else {
      result+=QString("5×5 깊이 중앙값: %1 m\n").arg(*sample.depth_m,0,'f',4);
      if (frame.camera_info_valid && sample.landing.rim_valid)
        result+=QString("백보드 중앙 가정 → 골대 (X, Y)\n(%1, %2) m\n가정 yaw = %3°")
            .arg(sample.landing.rim_x_m,0,'f',4).arg(sample.landing.rim_z_m,0,'f',4)
            .arg(click_yaw_deg_,0,'f',1);
      else result+="골대 좌표: 유효한 CameraInfo 필요";
    }
    click_text_->setText(result);
    UpdateUi();
  }
  void SendAction(const vision_core::ActionRequest& request) {
    if (MotionBusy()) return;
    const bool preparing=request.action==MissionAction::kDefaultPoseMode;
    if ((preparing ? pose_pub_->get_subscription_count() : action_pub_->get_subscription_count())==0) {
      status_text_->setText("알림: 모션 수신 노드가 없어 동작 요청을 건너뛰었습니다. 계산·좌표 표시는 계속 사용할 수 있습니다.");
      UpdateUi(); return;
    }
    // READY never queues another command. Unavailable requests are discarded
    // locally without reporting a physical completion or sending STOP.
    current_action_=request.action;
    prepare_camera_after_done_=preparing;
    active_prepare_duration_sec_=prepare_duration_sec_;
    AdvanceTransport({},request);
    UpdateUi();
  }
  void OnStatus(const msg::CommandStatus& status) {
    if (!transport_->HasPendingAction() || status.command_id!=transport_->PendingActionId()) return;
    vision_core::CommandDeliveryFeedback feedback;
    feedback.action_id=status.command_id;
    feedback.acknowledged=status.status==msg::CommandStatus::ACK;
    feedback.done=status.status==msg::CommandStatus::DONE;
    feedback.ready=status.status==msg::CommandStatus::READY;
    AdvanceTransport(feedback);
    if (feedback.done && !transport_->HasPendingAction()) {
      last_done_id_=status.command_id;
      last_done_sec_=get_clock()->now().seconds();
      // Never reuse an in-motion frame for the next automatic calculation.
      live_.valid=false;
      if (prepare_camera_after_done_) {
        prepare_camera_after_done_=false;
        BeginForwardCamera();
      } else {
        status_text_->setText(QString("%1 완료 (DONE %2) — 다음 버튼 대기")
            .arg(DebugActionName(current_action_)).arg(static_cast<qulonglong>(last_done_id_)));
      }
    }
    UpdateUi();
  }
  void AdvanceTransport(const vision_core::CommandDeliveryFeedback& feedback,
                        const vision_core::ActionRequest& request={}) {
    const auto command=transport_->Compute(vision_core::MissionType::kGoal,0,request,
        vision_core::CameraRequest::kNone,feedback,
        std::chrono::duration<double>(Clock::now().time_since_epoch()).count());
    if (command.command_type==vision_core::CommandType::kAction) {
      msg::ActionCommand output;
      output.action_id=command.action_id; output.mission=msg::ActionCommand::MISSION_GOAL;
      output.action=static_cast<std::uint16_t>(command.action);
      output.target_yaw_deg=command.target_yaw_deg;
      if (command.action==MissionAction::kDefaultPoseMode) {
        msg::ShootTuningPose pose;
        pose.action_id=command.action_id;
        pose.duration_sec=active_prepare_duration_sec_;
        pose_pub_->publish(pose);
      } else { action_pub_->publish(output); }
      status_text_->setText(QString("%1 / id=%2 — ACK 대기")
          .arg(DebugActionName(command.action)).arg(static_cast<qulonglong>(command.action_id)));
    } else if (transport_->ActionAckTimedOut()) {
      ReleaseUnavailableRequest();
    } else if (transport_->HasPendingAction()) {
      status_text_->setText(QString("%1 / id=%2 — 실제 DONE 대기 (READY로 다음 동작을 보내지 않음)")
          .arg(DebugActionName(command.action)).arg(static_cast<qulonglong>(command.action_id)));
    }
  }

  bool ReleaseUnavailableRequest() {
    const bool connected=current_action_==MissionAction::kDefaultPoseMode
        ? pose_pub_->get_subscription_count()>0 : action_pub_->get_subscription_count()>0;
    const auto reason=ReleaseUnavailableShootRequest(*transport_,connected);
    if (reason==ShootRequestUnavailable::kNone) return false;
    const auto notice=reason==ShootRequestUnavailable::kDisconnected
        ? "모션 수신 노드 연결이 없어 동작 요청 대기를 해제했습니다."
        : "모션 ACK 응답이 없어 동작 요청 대기를 해제했습니다.";
    status_text_->setText(QString("알림: %1 계산·좌표 표시는 계속 사용할 수 있습니다.").arg(notice));
    current_action_=MissionAction::kNone;
    prepare_camera_after_done_=false;
    UpdateUi();
    return true;
  }

  bool MotionBusy() const { return transport_->HasPendingAction() || pending_camera_id_!=0; }
  void BeginForwardCamera() {
    if (camera_pub_->get_subscription_count()==0) {
      status_text_->setText("알림: 몸 기본자세 완료. 카메라 수신 노드가 없어 정면 복귀 요청을 건너뛰었습니다.");
      return;
    }
    pending_camera_id_=next_camera_id_++;
    if (next_camera_id_==0) next_camera_id_=1;
    camera_acknowledged_=false;
    camera_requested_at_=Clock::now();
    PollCamera();
  }
  void PollCamera() {
    if (pending_camera_id_==0) return;
    const double elapsed=std::chrono::duration<double>(Clock::now()-camera_requested_at_).count();
    if (camera_pub_->get_subscription_count()==0 || (!camera_acknowledged_ &&
        config_.command.action_ack_timeout_sec>0 && elapsed>=config_.command.action_ack_timeout_sec)) {
      pending_camera_id_=0;
      status_text_->setText("알림: 카메라 연결/ACK 응답이 없어 정면 복귀 대기를 해제했습니다. 계산·좌표 표시는 계속 사용할 수 있습니다.");
      return;
    }
    if (!camera_acknowledged_) {
      msg::CameraCommand command;
      command.command_id=pending_camera_id_; command.request=msg::CameraCommand::FORWARD;
      camera_pub_->publish(command);
    }
    status_text_->setText(camera_acknowledged_ ? "몸 기본자세 완료 — 카메라 정면 복귀 DONE 대기"
                                              : "몸 기본자세 완료 — 카메라 정면 복귀 ACK 대기");
  }
  void OnCameraStatus(const msg::CommandStatus& status) {
    if (pending_camera_id_==0 || status.command_id!=pending_camera_id_) return;
    if (status.status==msg::CommandStatus::ACK) camera_acknowledged_=true;
    if (status.status==msg::CommandStatus::DONE) {
      pending_camera_id_=0;
      last_done_sec_=get_clock()->now().seconds(); live_.valid=false;
      status_text_->setText("기본자세 및 카메라 정면 복귀 완료 — 다음 버튼 대기");
    }
    UpdateUi();
  }

  vision_core::MissionControllerConfig config_;
  std::unique_ptr<vision_core::MissionController> observer_;
  std::unique_ptr<vision_core::ControlCommandCoordinator> transport_;
  std::unique_ptr<YoloTrtEngine> yolo_;
  std::unique_ptr<QWidget> window_;
  ImageWidget *live_view_{},*frozen_view_{},*state_view_{};
  QLabel *calculation_text_{},*action_text_{},*status_text_{},*click_text_{};
  QPushButton *calculate_button_{},*stored_shoot_button_{},*adopt_click_button_{};
  QDoubleSpinBox *manual_x_{},*manual_y_{};
  std::vector<QWidget*> target_edit_widgets_;
  std::vector<QPushButton*> motion_buttons_;
  std::map<std::string,QDoubleSpinBox*> parameter_widgets_;
  Frame live_,frozen_,clicked_frame_;
  std::optional<GoalCenterTarget> calculated_target_,clicked_target_;
  QString calculated_source_,calculated_details_,clicked_details_;
  vision_core::Intrinsics intrinsics_;
  unsigned int info_width_{0},info_height_{0};
  bool use_imu_{false},assume_zero_imu_{false},imu_valid_{false};
  double max_depth_age_{},max_frame_age_{},click_yaw_deg_{},imu_limit_{},roll_{},pitch_{},last_done_sec_{};
  std::uint64_t parameter_revision_{0},live_revision_{0},last_done_id_{0};
  MissionAction current_action_{MissionAction::kNone};
  Clock::time_point latest_received_;
  sensor_msgs::msg::Image::SharedPtr latest_image_,latest_depth_,processed_image_;
  rclcpp::Publisher<msg::ActionCommand>::SharedPtr action_pub_;
  rclcpp::Publisher<msg::ShootTuningPose>::SharedPtr pose_pub_;
  rclcpp::Publisher<msg::CameraCommand>::SharedPtr camera_pub_;
  rclcpp::Subscription<msg::CommandStatus>::SharedPtr camera_status_sub_;
  double prepare_duration_sec_{},active_prepare_duration_sec_{};
  bool prepare_camera_after_done_{false},camera_acknowledged_{false};
  std::uint64_t next_camera_id_{1},pending_camera_id_{0};
  Clock::time_point camera_requested_at_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_,depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr imu_sub_;
  rclcpp::Subscription<msg::CommandStatus>::SharedPtr status_sub_;
  rclcpp::TimerBase::SharedPtr frame_timer_,transport_timer_;
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};
} // namespace vision

int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  // ROS arguments belong to rclcpp, not Qt's command-line parser.
  int qt_argc=1;
  QApplication application(qt_argc,argv);
  try {
    const auto node=std::make_shared<vision::ShootTuningNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    QTimer spin;
    QObject::connect(&spin,&QTimer::timeout,[&]() {
      if (!rclcpp::ok()) application.quit();
      else executor.spin_some(std::chrono::milliseconds(5));
    });
    spin.start(10);
    const int result=application.exec();
    rclcpp::shutdown();
    return result;
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("shoot_tuning_node"),"%s",error.what());
    rclcpp::shutdown();
    return 1;
  }
}
