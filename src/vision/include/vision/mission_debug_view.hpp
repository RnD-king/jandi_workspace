#pragma once
#include "vision/backboard_depth.hpp"
#include "vision/debug_guides.hpp"
#include "vision_core/mission_controller.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace vision {
inline constexpr double kDebugPi=3.14159265358979323846;
struct MissionDebugContext {
  std::string control_state{"RUNNING"},control_hint,performance;
  bool preview_ready{false},imu_enabled{false},imu_ready{false};
  int approved_repeat_scope{0};
  std::uint64_t waiting_action_id{0},last_received_done_id{0},last_processed_done_id{0};
  double roll_rad{0},pitch_rad{0};
};
struct MissionDebugImages { cv::Mat image,panel; };
struct GoalLandingDebug {
  bool valid{false},rim_valid{false},commanded{false};
  double x_m{0},z_m{0},yaw_deg{0},rim_x_m{0},rim_z_m{0};
  double position_error_m{0},range_error_m{0};
};
inline GoalLandingDebug ComputeGoalLandingDebug(
    const vision_core::GoalConfig& config,
    const vision_core::MissionFrameResult& result) {
  GoalLandingDebug landing;
  if (result.active_mission!=vision_core::MissionType::kGoal ||
      !std::isfinite(config.throw_landing_x_m) ||
      !std::isfinite(config.throw_landing_z_m)) return landing;
  const auto& goal=result.goal;
  const auto& fine=goal.fine_decision_pose;
  if (goal.fine_decision_pose_used && fine.valid &&
      std::isfinite(fine.x_m) && std::isfinite(fine.z_m) && fine.z_m>0 &&
      std::isfinite(fine.yaw_rad) && std::isfinite(config.hoop_radius_m)) {
    landing.rim_x_m=fine.x_m+config.hoop_radius_m*std::sin(fine.yaw_rad);
    landing.rim_z_m=fine.z_m-config.hoop_radius_m*std::cos(fine.yaw_rad);
    const double distance=std::hypot(landing.rim_x_m,landing.rim_z_m);
    if (std::isfinite(distance)) {
      landing.rim_valid=true;
      double angle=distance>1e-9
          ? std::atan2(config.throw_landing_x_m,config.throw_landing_z_m)
              -std::atan2(landing.rim_x_m,landing.rim_z_m) : 0.0;
      while (angle>kDebugPi) angle-=2*kDebugPi;
      while (angle<-kDebugPi) angle+=2*kDebugPi;
      // FineAction checks the landing error at this integer command angle.
      landing.yaw_deg=static_cast<double>(std::lround(angle*180/kDebugPi));
      landing.valid=true;
      landing.range_error_m=distance-std::hypot(
          config.throw_landing_x_m,config.throw_landing_z_m);
    }
  }
  // During SHOOT, keep using the issued angle even if live depth changes or
  // no new stationary decision pose is available. Do not invent a rim pose.
  if (result.command.mission==vision_core::MissionType::kGoal &&
      result.command.action==vision_core::MissionAction::kShoot) {
    landing.yaw_deg=result.command.target_yaw_deg;
    landing.valid=landing.commanded=true;
  } else if (goal.action.action==vision_core::MissionAction::kShoot) {
    landing.yaw_deg=goal.action.target_yaw_deg;
    landing.valid=true;
    landing.commanded=goal.mode==vision_core::GoalMode::kShoot;
  }
  if (landing.valid) {
    const double angle=landing.yaw_deg*kDebugPi/180;
    landing.x_m=std::cos(angle)*config.throw_landing_x_m
        -std::sin(angle)*config.throw_landing_z_m;
    landing.z_m=std::sin(angle)*config.throw_landing_x_m
        +std::cos(angle)*config.throw_landing_z_m;
    if (landing.rim_valid) landing.position_error_m=std::hypot(
        landing.rim_x_m-landing.x_m,landing.rim_z_m-landing.z_m);
  }
  return landing;
}
inline const char* DebugClassLabel(int id,const vision_core::MissionControllerConfig& c) {
  if(id==c.line_detection.class_id)return "line";
  if(id==c.object_targets.ball_class_id)return "ball";
  if(id==c.object_targets.goal_class_id)return "goal";
  if(id==c.object_targets.backboard_class_id)return "backboard";
  if(id==c.object_targets.hurdle_class_id)return "hurdle";
  return "unknown";
}

inline const char *DebugActionName(vision_core::MissionAction action) {
  using vision_core::MissionAction;
  switch (action) {
  case MissionAction::kNone: return "NONE";
  case MissionAction::kDefaultPosition: return "DEFAULT_POSITION";
  case MissionAction::kDefaultPoseMode: return "DEFAULT_POSE_MODE";
  case MissionAction::kStepForwardHalf: return "STEP_FORWARD_HALF";
  case MissionAction::kStepBack: return "STEP_BACK";
  case MissionAction::kLeftSideStep: return "LEFT_SIDE_STEP";
  case MissionAction::kRightSideStep: return "RIGHT_SIDE_STEP";
  case MissionAction::kTurnLeft: return "TURN_LEFT";
  case MissionAction::kTurnRight: return "TURN_RIGHT";
  case MissionAction::kWalkMode: return "WALK_MODE";
  case MissionAction::kStepForwardOne: return "STEP_FORWARD_ONE";
  case MissionAction::kStepForwardLeft: return "STEP_FORWARD_LEFT_THREE";
  case MissionAction::kStepForwardRight: return "STEP_FORWARD_RIGHT_THREE";
  case MissionAction::kStepForwardFive: return "STEP_FORWARD_FIVE";
  case MissionAction::kStepForwardTwo: return "STEP_FORWARD_TWO";
  case MissionAction::kStepForwardThree: return "STEP_FORWARD_THREE";
  case MissionAction::kStepForwardFour: return "STEP_FORWARD_FOUR";
  case MissionAction::kStepForwardLeftTwo: return "STEP_FORWARD_LEFT_TWO";
  case MissionAction::kStepForwardRightTwo: return "STEP_FORWARD_RIGHT_TWO";
  case MissionAction::kStepForwardRightOne: return "STEP_FORWARD_RIGHT_ONE";
  case MissionAction::kStepForwardLeftOne: return "STEP_FORWARD_LEFT_ONE";
  case MissionAction::kStepForwardRightOneHurdle: return "TURN_RIGHT_IN_PLACE";
  case MissionAction::kTurnLeftInPlace: return "TURN_LEFT_IN_PLACE";
  case MissionAction::kPickBall: return "PICK_BALL";
  case MissionAction::kRecatch: return "RECATCH";
  case MissionAction::kHurdle: return "HUDDLE";
  case MissionAction::kShoot: return "SHOOT";
  case MissionAction::kTurnLeftAndStep: return "TURN_LEFT_AND_STEP";
  case MissionAction::kTurnRightAndStep: return "TURN_RIGHT_AND_STEP";
  case MissionAction::kContactWalk: return "CONTACT_WALK_RESERVED";
  }
  return "UNKNOWN";
}
inline const char *DebugMissionName(vision_core::MissionType mission) {
  switch (mission) {
    case vision_core::MissionType::kLine: return "LINE";
    case vision_core::MissionType::kBall: return "BALL";
    case vision_core::MissionType::kGoal: return "GOAL";
    case vision_core::MissionType::kHurdle: return "HURDLE";
    default: return "NONE";
  }
}
inline std::string DebugRepeatLabel(int scope, const vision_core::MissionControllerConfig& config) {
  using vision_core::BallMode; using vision_core::GoalMode; using vision_core::HurdleMode;
  if (scope==200+static_cast<int>(BallMode::kDownSearch))
    return "SEARCH <="+std::to_string(config.ball.down_search_max_steps)+" steps";
  if (scope==400+static_cast<int>(HurdleMode::kDownSearch))
    return "SEARCH <="+std::to_string(config.hurdle.down_search_max_steps)+" steps";
  if (scope==200+static_cast<int>(BallMode::kPostPickupBackAway)) return "BACK x"+std::to_string(config.ball.post_pickup_back_steps);
  if (scope==300+static_cast<int>(GoalMode::kPostShootBackAway)) return "BACK x"+std::to_string(config.goal.post_shoot_back_steps);
  if (scope==300+static_cast<int>(GoalMode::kEdgeTurn)) return "EDGE TURN x2";
  if (scope==200+static_cast<int>(BallMode::kMidTurnBeforeRecovery)) return "TURN x1 -> FORWARD RECOVERY";
  if (scope==200+static_cast<int>(BallMode::kFineSidePair)) return "SIDE STEP x2";
  if (scope==200+static_cast<int>(BallMode::kPostPickupLineRecovery) ||
      scope==300+static_cast<int>(GoalMode::kHeadingRecovery)) return "LINE SEARCH <="+std::to_string(config.mission_recovery_max_turns)+" turns";
  if (scope==400+static_cast<int>(HurdleMode::kContactWalk)) return "CONTACT x"+std::to_string(config.hurdle.contact_steps);
  return "ONE ACTION";
}
template<class DetectionRange>
inline void DrawRawDebugDetections(cv::Mat &vis, const DetectionRange& dets,
                                   const vision_core::MissionControllerConfig& config) {
  for (const auto &det : dets) {
    if (det.confidence < config.line_detection.confidence) {
      continue;
    }

    const cv::Rect box(static_cast<int>(std::lround(det.box.x)), static_cast<int>(std::lround(det.box.y)),
                       static_cast<int>(std::lround(det.box.width)), static_cast<int>(std::lround(det.box.height)));
    cv::Rect clipped = box & cv::Rect(0, 0, vis.cols, vis.rows);
    if (clipped.width <= 0 || clipped.height <= 0) {
      continue;
    }

    const int cx = clipped.x + clipped.width / 2;
    const int cy = clipped.y + clipped.height / 2;

    cv::Scalar box_color(160, 160, 160);
    cv::Scalar center_color(80, 80, 80);
    if (det.class_id == config.line_detection.class_id) {
      box_color = cv::Scalar(0, 255, 0);
      center_color = cv::Scalar(0, 0, 255);
    } else if (det.class_id == config.object_targets.ball_class_id) {
      box_color = cv::Scalar(0, 165, 255);
      center_color = cv::Scalar(255, 0, 0);
    } else if (det.class_id == config.object_targets.goal_class_id) {
      box_color = cv::Scalar(255, 255, 0);
      center_color = cv::Scalar(0, 255, 255);
    } else if (det.class_id == config.object_targets.backboard_class_id) {
      box_color = cv::Scalar(255, 128, 0);
      center_color = cv::Scalar(0, 128, 255);
    } else if (det.class_id == config.object_targets.hurdle_class_id) {
      box_color = cv::Scalar(255, 0, 255);
      center_color = cv::Scalar(255, 255, 255);
    }

    cv::rectangle(vis, clipped, box_color, 2);
    cv::circle(vis, cv::Point(cx, cy), 3, center_color, -1);

    char buf[40];
    std::snprintf(buf, sizeof(buf), "%s %.2f", DebugClassLabel(det.class_id,config), det.confidence);

    int tx = clipped.x;
    int ty = clipped.y - 6;
    if (ty < 12)
      ty = clipped.y + 16;

    cv::putText(vis, buf, cv::Point(tx, ty), cv::FONT_HERSHEY_SIMPLEX,
                0.52, box_color, 1);
  }
}
inline void DrawDecisionDetections(
    cv::Mat &vis, const std::vector<vision_core::Point2> &line_centers,
    bool line_confirmed,
    const std::optional<vision_core::ObjectTarget> &ball_target,
    bool ball_confirmed,
    const std::optional<vision_core::ObjectTarget> &backboard_target,
    bool backboard_confirmed,
    const std::optional<vision_core::ObjectTarget> &hurdle_target,
    bool hurdle_confirmed) {
  if (line_confirmed) {
    for (const auto &point : line_centers) {
      cv::circle(vis,
                 cv::Point(static_cast<int>(std::round(point.u)),
                           static_cast<int>(std::round(point.v))),
                 5, cv::Scalar(0, 255, 0), 2);
    }
  }

  const auto draw_target = [&vis](
                               const std::optional<vision_core::ObjectTarget> &target,
                               bool confirmed, const char *label,
                               const cv::Scalar &color) {
    if (!confirmed || !target) return;
    const auto &box = target->box_px;
    const cv::Rect raw_box(static_cast<int>(std::round(box.x)),
                           static_cast<int>(std::round(box.y)),
                           static_cast<int>(std::round(box.width)),
                           static_cast<int>(std::round(box.height)));
    const cv::Rect clipped = raw_box & cv::Rect(0, 0, vis.cols, vis.rows);
    if (clipped.width <= 0 || clipped.height <= 0) return;
    cv::rectangle(vis, clipped, color, 2);
    // Boxes show recognition evidence; the controller's decision center is
    // drawn separately, matching the tuning node.
    const int text_y = clipped.y >= 14 ? clipped.y - 4 : clipped.y + 14;
    cv::putText(vis, label, cv::Point(clipped.x, text_y),
                cv::FONT_HERSHEY_SIMPLEX, 0.60, color, 2);
  };

  draw_target(ball_target, ball_confirmed, "BALL",
              cv::Scalar(0, 165, 255));
  draw_target(backboard_target, backboard_confirmed, "BACKBOARD APPROACH",
              cv::Scalar(255, 128, 0));
  draw_target(hurdle_target, hurdle_confirmed, "HURDLE",
              cv::Scalar(255, 0, 255));
}
template<class DetectionRange>
inline MissionDebugImages RenderMissionDebug(const cv::Mat& bgr,const DetectionRange& dets,
    const cv::Mat& aligned_depth,const vision_core::MissionControllerConfig& config,
    const vision_core::PreparedPerceptionFrame& prepared,const vision_core::MissionFrameResult& mission_result,
    const vision_core::CameraFeedback& camera,const vision_core::Intrinsics& intrinsics,
    const MissionDebugContext& view) {
  const auto& feats=mission_result.line_features;
  const auto& ball_cmd=mission_result.ball;
  const auto& goal_cmd=mission_result.goal;
  const auto& hurdle_cmd=mission_result.hurdle;
  const auto& ball_target=prepared.targets.ball;
  const auto& backboard_target=prepared.targets.backboard;
  const auto& hurdle_target=prepared.targets.hurdle;
  const bool selected_ball=mission_result.active_mission==vision_core::MissionType::kBall;
  const bool selected_goal=mission_result.active_mission==vision_core::MissionType::kGoal;
  const bool selected_hurdle=mission_result.active_mission==vision_core::MissionType::kHurdle;
  const bool line_confirmed=mission_result.line_computed && feats.guide.valid && (!mission_result.line_in_recovery || mission_result.line_score_recovery);
  cv::Mat vis = bgr.clone();
  for (const auto& detection:dets) if (detection.class_id==config.object_targets.backboard_class_id)
    vision::DrawDepthRegions(vis,cv::Rect(static_cast<int>(std::lround(detection.box.x)), static_cast<int>(std::lround(detection.box.y)), static_cast<int>(std::lround(detection.box.width)), static_cast<int>(std::lround(detection.box.height))),config.backboard_depth_strip_width_px,aligned_depth);
  DrawRawDebugDetections(vis, dets, config);
  const int ref_u = static_cast<int>(std::lround(
      config.line_features.image_center_u));
  const bool line_view=mission_result.active_mission==vision_core::MissionType::kLine ||
      ball_cmd.mode==vision_core::BallMode::kPostPickupLineRecovery ||
      goal_cmd.mode==vision_core::GoalMode::kHeadingRecovery;
  if (line_view && ref_u >= 0 && ref_u < vis.cols) {
    cv::line(vis, cv::Point(ref_u, 0), cv::Point(ref_u, vis.rows - 1),
             cv::Scalar(255, 255, 0), 1, cv::LINE_AA);
  }
  // Overlay the retained decision boxes/centers on the live detections.
  // READY object boxes may be moving-window averages.
  DrawDecisionDetections(
      vis, prepared.line_centers, line_confirmed, ball_target,
      ball_target.has_value(), backboard_target,
      backboard_target.has_value(), hurdle_target,
      hurdle_target.has_value());

  const auto draw_decision_center = [&vis](
      const vision_core::Point2 &point, const cv::Scalar &color) {
    if (!std::isfinite(point.u) || !std::isfinite(point.v) ||
        point.u < 0.0 || point.u >= vis.cols ||
        point.v < 0.0 || point.v >= vis.rows) return;
    const cv::Point center(
        std::clamp(static_cast<int>(std::lround(point.u)),0,vis.cols-1),
        std::clamp(static_cast<int>(std::lround(point.v)),0,vis.rows-1));
    cv::circle(vis, center, 7, color, 2, cv::LINE_AA);
    cv::drawMarker(vis, center, color, cv::MARKER_CROSS, 14, 2, cv::LINE_AA);
  };

  // LINE uses an O/H guide: it has a horizontal decision reference,
  // but no unique 2D object center. Do not invent a vertical point.
  if (mission_result.active_mission ==
          vision_core::MissionType::kLine &&
      (!mission_result.line_in_recovery || mission_result.line_score_recovery)) {
    const auto &guide = mission_result.line_decision_guide
        ? *mission_result.line_decision_guide : feats.guide;
    if (guide.valid && std::isfinite(guide.offset) && vis.cols > 1) {
      const double u = config.line_features.image_center_u +
          0.5 * static_cast<double>(vis.cols) * guide.offset;
      if (std::isfinite(u) && u >= 0.0 && u < vis.cols) {
        const int x = static_cast<int>(std::lround(u));
        cv::line(vis, cv::Point(x, 0), cv::Point(x, vis.rows - 1),
                 cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
      }
    }
  }

  if (selected_ball && ball_cmd.tracked.visible)
    draw_decision_center(ball_cmd.tracked.center_px,
                         cv::Scalar(0, 0, 255));

  if (selected_goal) {
    const auto &fine = goal_cmd.fine_decision_pose;
    if (goal_cmd.fine_decision_pose_used && fine.valid &&
        std::isfinite(fine.x_m) && std::isfinite(fine.z_m) &&
        fine.z_m > 0.0 && std::isfinite(intrinsics.fx) &&
        std::isfinite(intrinsics.cx) && vis.cols > 1) {
      // FineAction consumes (x,z,yaw) in 3D, not normalized (u,v).
      // Projection gives the exact horizontal bearing of that
      // decision; the unused vertical coordinate is not fabricated.
      const double u = intrinsics.fx *
          fine.x_m / fine.z_m + intrinsics.cx;
      if (std::isfinite(u) && u >= 0.0 && u < vis.cols) {
        const int x = static_cast<int>(std::lround(u));
        cv::line(vis, cv::Point(x, 0), cv::Point(x, vis.rows - 1),
                 cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
      }
    } else if (goal_cmd.tracked.visible &&
               !goal_cmd.fine_decision_pose_used) {
      draw_decision_center(goal_cmd.tracked.center_px,
                           cv::Scalar(0, 255, 255));
    }
  }

  // The marker is the recent weighted coordinate used for step size
  // and fine distance. Raw DOWN trigger values are shown in the panel.
  if (selected_hurdle && hurdle_cmd.tracked.visible)
    draw_decision_center(hurdle_cmd.tracked.center_px, cv::Scalar(255, 0, 255));


  const auto draw_v_line=[&](double v,const cv::Scalar& color) {
    if (!std::isfinite(v) || v < 0 || v >= vis.rows) return;
    const int y=std::clamp(static_cast<int>(std::lround(v)),0,vis.rows-1);
    cv::line(vis,{0,y},{vis.cols-1,y},color,1,cv::LINE_AA);
  };
  const auto draw_u_line=[&](double u,const cv::Scalar& color) {
    if (!std::isfinite(u) || u < 0 || u >= vis.cols) return;
    const int x=std::clamp(static_cast<int>(std::lround(u)),0,vis.cols-1);
    cv::line(vis,{x,0},{x,vis.rows-1},color,1,cv::LINE_AA);
  };
  if (selected_ball) {
    const auto& c=config.ball;
    if (camera.actual_mode==vision_core::CameraMode::kForward &&
        (ball_cmd.mode==vision_core::BallMode::kApproachBall ||
         ball_cmd.mode==vision_core::BallMode::kApproachTwoBeforeDown ||
         ball_cmd.mode==vision_core::BallMode::kWaitCameraDown ||
         ball_cmd.mode==vision_core::BallMode::kBallRecoveryForward)) {
      draw_u_line(c.far_u_des_px-c.approach_u_deadband_px,cv::Scalar(0,100,255));
      draw_u_line(c.far_u_des_px+c.approach_u_deadband_px,cv::Scalar(0,100,255));
      draw_v_line(c.approach_short_v_px,cv::Scalar(0,200,255));
      draw_v_line(c.approach_two_step_v_px,cv::Scalar(0,100,255));
      // The extra boundary belongs only to the center column (zone 8).
      if (std::isfinite(c.approach_direct_down_v_px) &&
          c.approach_direct_down_v_px>=0 && c.approach_direct_down_v_px<vis.rows) {
        const int left=std::max(0,static_cast<int>(std::ceil(c.far_u_des_px-c.approach_u_deadband_px)));
        const int right=std::min(vis.cols-1,static_cast<int>(std::floor(c.far_u_des_px+c.approach_u_deadband_px)));
        const int y=std::clamp(static_cast<int>(std::lround(c.approach_direct_down_v_px)),0,vis.rows-1);
        if (left<=right) {
          const cv::Scalar color(0,230,255);
          cv::line(vis,{left,y},{right,y},color,2,cv::LINE_AA);
          cv::putText(vis,"1 STEP -> DOWN",{left+4,std::max(14,y-6)},
              cv::FONT_HERSHEY_SIMPLEX,0.45,color,1,cv::LINE_AA);
        }
      }
    } else if (camera.actual_mode==vision_core::CameraMode::kDown) {
      draw_v_line(c.down_one_step_v_px,cv::Scalar(0,200,255));
      draw_v_line(std::min(c.fine_mode_v_px,c.pickup_v1_px),cv::Scalar(0,100,255));
    }
  }
  if (selected_goal && (goal_cmd.mode == vision_core::GoalMode::kForwardApproach ||
      goal_cmd.mode == vision_core::GoalMode::kApproach ||
      goal_cmd.mode == vision_core::GoalMode::kSearch)) {
    const auto& c=config.goal;
    draw_u_line(c.target_u_px,cv::Scalar(0,255,255));
    draw_u_line(c.target_u_px-c.approach_u_deadband_px,cv::Scalar(0,160,160));
    draw_u_line(c.target_u_px+c.approach_u_deadband_px,cv::Scalar(0,160,160));
  }
  if (selected_goal && mission_result.foot_pose==vision_core::FootPose::kWalk) {
    draw_u_line(config.goal.edge_margin_px,cv::Scalar(60,60,255));
    draw_u_line(vis.cols-config.goal.edge_margin_px,cv::Scalar(60,60,255));
  }
  if (selected_hurdle) {
    const auto& c=config.hurdle;
    if (camera.actual_mode==vision_core::CameraMode::kForward &&
        (hurdle_cmd.mode==vision_core::HurdleMode::kApproach ||
         hurdle_cmd.mode==vision_core::HurdleMode::kApproachTwoBeforeDown ||
         hurdle_cmd.mode==vision_core::HurdleMode::kRecoveryForward)) {
      draw_v_line(c.approach_short_v_px,cv::Scalar(255,120,255));
      draw_v_line(c.approach_two_step_v_px,cv::Scalar(200,0,200));
    } else if (camera.actual_mode==vision_core::CameraMode::kDown) {
      vision::DrawHurdleDownGuides(vis,c,
          mission_result.foot_pose==vision_core::FootPose::kFine);
    }
  }

  // The pickup region uses the same four pixel edges as FineAction.
  // Its single decision-center marker is above.
  if (selected_ball && camera.actual_mode==vision_core::CameraMode::kDown) {
    const auto &c = config.ball;
    const cv::Rect image_bounds(0, 0, vis.cols, vis.rows);
    const int left = static_cast<int>(std::lround(c.pickup_u1_px));
    const int top = static_cast<int>(std::lround(c.pickup_v1_px));
    const int right = static_cast<int>(std::lround(c.pickup_u2_px));
    const int bottom = static_cast<int>(std::lround(c.pickup_v2_px));
    const cv::Rect box = cv::Rect(left, top,
        std::max(1, right - left + 1), std::max(1, bottom - top + 1)) & image_bounds;
    if (!box.empty())
      cv::rectangle(vis, box, cv::Scalar(0,255,255), 2);
  }
  // Raw depth-edge points are estimator inputs, not the GOAL
  // fine-control decision coordinate; suppress them here.

  // Both ROS nodes share this panel. Collect diagnostics before laying them
  // out so optional rows and wrapped text cannot overlap or be clipped.
  struct PanelRow { std::string text; cv::Scalar color; bool heading{false}; };
  using PanelRows = std::vector<PanelRow>;
  PanelRows header_rows,footer_rows,line_rows,ball_rows,hurdle_rows,goal_rows;
  PanelRows ball_approach_rows,ball_fine_rows,ball_recovery_rows;
  PanelRows hurdle_approach_rows,hurdle_fine_rows,hurdle_recovery_rows;
  PanelRows goal_approach_rows,goal_fine_rows,goal_recovery_rows;
  const cv::Scalar text_color(235,235,235),muted(150,150,150);
  const cv::Scalar line_color(0,230,0),ball_color(0,165,255);
  const cv::Scalar hurdle_color(0,255,255),goal_color(255,210,100);
  const auto add_row=[](PanelRows& rows,const std::string& text,const cv::Scalar& color) {
    rows.push_back({text,color,false});
  };
  const auto state=mission_result.terminal_hold ? std::string("HOLD / RESET REQUIRED") : view.control_state;
  add_row(header_rows,"CONTROL: "+state+" | RGB: LIVE",mission_result.terminal_hold ? cv::Scalar(0,0,255) : ball_color);
  char buf[256];
  std::snprintf(buf,sizeof(buf),"ACTION: %s | mission=%s",DebugActionName(mission_result.command.action),DebugMissionName(mission_result.active_mission));
  add_row(header_rows,std::string(buf)+" | "+DebugRepeatLabel(view.preview_ready ? mission_result.repeat_scope : view.approved_repeat_scope,config),text_color);
  const int ball_index=view.control_state=="WAIT_START" ? config.test_ball_mission_index : mission_result.ball_mission_index;
  const int goal_index=view.control_state=="WAIT_START" ? config.test_goal_mission_index : mission_result.goal_mission_index;
  std::snprintf(buf,sizeof(buf),"turns=%d/%d",
      mission_result.recovery_turns_completed,mission_result.recovery_max_turns);
  add_row(header_rows,buf,text_color);
  add_row(header_rows,"TEST STAGE: "+std::to_string(config.test_stage)+
      (mission_result.trial_complete ? " COMPLETE / SELECT STAGE OR RESET" : mission_result.test_executing ? " EXECUTING / RECOVERY" : mission_result.test_ready ? " READY" : config.test_stage==3 ? " ALIGN IN STAGE 2 FIRST" : ""),text_color);
  add_row(header_rows,view.waiting_action_id
      ? "FEEDBACK: WAIT EXECUTOR DONE id="+std::to_string(view.waiting_action_id)
      : "FEEDBACK: DONE "+std::to_string(view.last_received_done_id)+
        (view.last_received_done_id!=view.last_processed_done_id ? " / WAIT FRESH IMAGE" : " / APPLIED"),muted);
  add_row(header_rows,std::string("FEET: ")+(mission_result.foot_pose==vision_core::FootPose::kWalk ? "WALK" : "FINE")+
      (mission_result.foot_transition ? " | TRANSITION / WAIT_DONE" : ""),text_color);
  const auto &lc = config.line_p2p;
  add_row(line_rows,"LINE",line_color);
  std::snprintf(buf, sizeof(buf),
      "reference=%.1fpx  Ko=%.3f  Kh=%.3f  deadband=%.4f  turn>%.3f",
      config.line_features.image_center_u,
      lc.offset_gain, lc.heading_gain, lc.steering_deadband, lc.in_place_turn_score);
  add_row(line_rows,buf,line_color);
  const auto &decision_line = mission_result.line_decision_guide
      ? *mission_result.line_decision_guide : feats.guide;
  if (mission_result.line_computed && decision_line.valid &&
      std::isfinite(decision_line.offset) &&
      std::isfinite(decision_line.heading_rad)) {
    const double offset_term = lc.offset_gain * decision_line.offset;
    const double heading_term = lc.heading_gain * decision_line.heading_rad;
    const double score = offset_term + heading_term;
    std::snprintf(buf, sizeof(buf), "O=%+.4f  x  Ko=%.3f  =  %+.4f",
        decision_line.offset, lc.offset_gain, offset_term);
    add_row(line_rows,buf,line_color);
    std::snprintf(buf, sizeof(buf), "H=%+.4frad  x  Kh=%.3f  =  %+.4f",
        decision_line.heading_rad, lc.heading_gain, heading_term);
    add_row(line_rows,buf,line_color);
    // Recovery is selected by the FSM, not by this steering score.
    const char *direction = mission_result.line_in_recovery && !mission_result.line_score_recovery
        ? "RECOVERY_FSM" : (score < -lc.in_place_turn_score ? "TURN_LEFT_IN_PLACE" :
            score > lc.in_place_turn_score ? "TURN_RIGHT_IN_PLACE" :
            score < -lc.steering_deadband ? "LEFT" :
            score > lc.steering_deadband ? "RIGHT" : "STRAIGHT");
    std::snprintf(buf, sizeof(buf),
        "SCORE=%+.4f  |  band=[%+.4f,%+.4f]  |  %s",
        score, -lc.steering_deadband, lc.steering_deadband, direction);
    add_row(line_rows,buf,line_color);
  } else {
    add_row(line_rows,"O x Ko = --",line_color);
    add_row(line_rows,"H x Kh = --",line_color);
    add_row(line_rows,"SCORE: -- (no valid LINE decision)",line_color);
  }

  std::snprintf(buf,sizeof(buf),"LINE BBOX MIN: width=%.1fpx height=%.1fpx",
      config.line_detection.min_box_width,config.line_detection.min_box_height);
  add_row(line_rows,buf,line_color);
  std::snprintf(buf,sizeof(buf),"SCORE RECOVERY: %s | turns=%d/%d DONE",
      mission_result.line_score_recovery ? (mission_result.terminal_hold ? "HOLD / RESET" : "ACTIVE") : "IDLE",
      mission_result.line_score_recovery_turns_completed,lc.recovery_max_turns);
  add_row(line_rows,buf,line_color);

  const auto &bc = config.ball;
  add_row(ball_rows,std::string("BALL  |  ") +
      vision_core::BallController::ModeName(ball_cmd.mode),ball_color);
  std::snprintf(buf,sizeof(buf),"BALL #%d %s",ball_index,ball_index%2 ? "RIGHT" : "LEFT");
  add_row(ball_rows,buf,ball_color);
  std::snprintf(buf, sizeof(buf), "target u=%.1fpx band=%.1fpx | ZONE 8: 2-step v=[%.1f,%.1f)",
      bc.far_u_des_px, bc.approach_u_deadband_px, bc.approach_two_step_v_px,bc.approach_direct_down_v_px);
  add_row(ball_approach_rows,buf,ball_color);
  std::snprintf(buf,sizeof(buf),"ZONE 8: v>=%.1fpx -> 1-step DONE then DOWN",bc.approach_direct_down_v_px);
  add_row(ball_approach_rows,buf,ball_color);
  std::snprintf(buf,sizeof(buf),"DOWN SEARCH: need=%d/%d (FINE stable=%d)",
      bc.down_search_min_hits,bc.stable_window,bc.stable_min_hits);
  add_row(ball_approach_rows,buf,ball_color);
  if (selected_ball && ball_cmd.mode == vision_core::BallMode::kApproachBall &&
      ball_cmd.tracked.visible) {
    std::snprintf(buf, sizeof(buf),
        "far u=%.1fpx | error=%+.1fpx | band=[%+.1f,%+.1f]px",
        ball_cmd.tracked.center_px.u, ball_cmd.tracked.center_px.u - bc.far_u_des_px,
        -bc.approach_u_deadband_px, bc.approach_u_deadband_px);
    add_row(ball_approach_rows,buf,ball_color);
  } else {
    add_row(ball_approach_rows,"far u-target: --",ball_color);
  }
  std::snprintf(buf, sizeof(buf), "PICK u=[%.1f,%.1f] v=[%.1f,%.1f] | SIDE x2 excess>=%.1fpx",
      bc.pickup_u1_px, bc.pickup_u2_px, bc.pickup_v1_px, bc.pickup_v2_px, bc.fine_side_two_step_margin_px);
  add_row(ball_fine_rows,buf,ball_color);
  const bool ball_fine = selected_ball && ball_cmd.tracked.visible &&
      (ball_cmd.mode == vision_core::BallMode::kFineAdjustForPickup ||
       ball_cmd.mode == vision_core::BallMode::kFineSidePair ||
       ball_cmd.mode == vision_core::BallMode::kPickupBall);
  if (ball_fine) {
    std::snprintf(buf, sizeof(buf), "PICK u=%.1fpx  v=%.1fpx | forward error=%+.1fpx",
        ball_cmd.tracked.center_px.u, ball_cmd.tracked.center_px.v,
        bc.PickupCenterV()-ball_cmd.tracked.center_px.v);
    add_row(ball_fine_rows,buf,ball_color);
  } else {
    add_row(ball_fine_rows,"PICK u/v / forward error: --",ball_color);
  }
  std::snprintf(buf, sizeof(buf), "ENTRY v<=%.0f | hits=%d/%d need=%d | frames=%d/%d",
      bc.upper_acquire_v_px, ball_cmd.entry_hits, bc.stable_window, bc.entry_min_hits,
      ball_cmd.entry_frames, bc.stable_window);
  add_row(ball_approach_rows,buf,ball_color);

  const auto &gc = config.goal;
  add_row(goal_rows,std::string("GOAL  |  ") +
      vision_core::GoalController::ModeName(goal_cmd.mode),goal_color);
  std::snprintf(buf,sizeof(buf),"GOAL #%d %s",goal_index,goal_index%2 ? "RIGHT" : "LEFT");
  add_row(goal_rows,buf,goal_color);
  std::snprintf(buf, sizeof(buf), "far target u=%.1fpx  deadband=%.1fpx  fine z<=%.3fm",
      gc.target_u_px, gc.approach_u_deadband_px, gc.fine_adjust_start_z_m);
  add_row(goal_approach_rows,buf,goal_color);
  if (selected_goal && (goal_cmd.mode == vision_core::GoalMode::kApproach ||
      goal_cmd.mode == vision_core::GoalMode::kForwardApproach) &&
      goal_cmd.tracked.visible) {
    std::snprintf(buf, sizeof(buf),
        "far u=%.1fpx | error=%+.1fpx | band=[%+.1f,%+.1f]px",
        goal_cmd.tracked.center_px.u, goal_cmd.tracked.center_px.u - gc.target_u_px,
        -gc.approach_u_deadband_px, gc.approach_u_deadband_px);
    add_row(goal_approach_rows,buf,goal_color);
  } else {
    add_row(goal_approach_rows,"far u-target: --",goal_color);
  }
  std::snprintf(buf, sizeof(buf), "SHOOT tol=%.1fcm  limit=%.0fdeg",
      100.0*gc.position_tolerance_m, gc.shoot_yaw_limit_deg);
  add_row(goal_fine_rows,buf,goal_color);
  const auto landing=ComputeGoalLandingDebug(gc,mission_result);
  const auto &fine=goal_cmd.fine_decision_pose;
  if (landing.rim_valid) {
    std::snprintf(buf,sizeof(buf),"board=(%+.1f,%.1f)cm yaw=%+.1fdeg",
        100*fine.x_m,100*fine.z_m,fine.yaw_rad*180/kDebugPi);
    add_row(goal_fine_rows,buf,goal_color);
    std::snprintf(buf,sizeof(buf),"radial err=%+.2fcm",100*landing.range_error_m);
    add_row(goal_fine_rows,buf,goal_color);
  } else if (selected_goal && prepared.backboard_center_depth_m &&
      std::isfinite(*prepared.backboard_center_depth_m)) {
    std::snprintf(buf,sizeof(buf),"backboard center depth=%.4fm | fine z<=%.3fm",
        *prepared.backboard_center_depth_m,gc.fine_adjust_start_z_m);
    add_row(goal_fine_rows,buf,goal_color);
  } else {
    add_row(goal_fine_rows,"fine input x/z/yaw: --",goal_color);
  }
  // Share the exact coordinate strings between the panel and camera overlay.
  std::vector<std::string> landing_rows;
  std::snprintf(buf,sizeof(buf),"REF 0deg X/Z=(%+.1f,%.1f)cm",
      100*gc.throw_landing_x_m,100*gc.throw_landing_z_m);
  landing_rows.emplace_back(buf);
  if (landing.valid) {
    std::snprintf(buf,sizeof(buf),"PRED LAND %s X/Z=(%+.1f,%.1f)cm yaw=%+.0fdeg",
        landing.commanded ? "CMD" : "PLAN",
        100*landing.x_m,100*landing.z_m,landing.yaw_deg);
    landing_rows.emplace_back(buf);
  } else landing_rows.push_back("PRED LAND X/Z: -- (no shot command / decision pose)");
  for (const auto& text:landing_rows) add_row(goal_fine_rows,text,goal_color);

  const auto &hc = config.hurdle;
  add_row(hurdle_rows,std::string("HURDLE  |  ") +
      vision_core::HurdleController::ModeName(hurdle_cmd.mode),hurdle_color);
  std::snprintf(buf, sizeof(buf), "entry v<=%.1fpx | 2-step then DOWN v>=%.1fpx",
      hc.upper_acquire_v_px, hc.approach_two_step_v_px);
  add_row(hurdle_approach_rows,buf,hurdle_color);
  std::snprintf(buf,sizeof(buf),"DOWN SEARCH: need=%d/%d (FINE stable=%d)",
      hc.down_search_min_hits,hc.stable_window,hc.stable_min_hits);
  add_row(hurdle_approach_rows,buf,hurdle_color);
  if (selected_hurdle && hurdle_cmd.mode == vision_core::HurdleMode::kApproach &&
      hurdle_target && vis.rows > 1) {
    const double raw_v = hurdle_target->center_px.v;
    std::snprintf(buf, sizeof(buf),
        "raw v=%.1fpx | v-acquire=%+.1fpx | v-DOWN=%+.1fpx",
        raw_v, raw_v - hc.upper_acquire_v_px, raw_v - hc.approach_two_step_v_px);
    add_row(hurdle_approach_rows,buf,hurdle_color);
  } else {
    add_row(hurdle_approach_rows,"raw v / threshold errors: --",hurdle_color);
  }

  std::snprintf(buf,sizeof(buf),"BALL v lines: 3-step=%.1fpx 2-step=%.1fpx",
      bc.approach_short_v_px,bc.approach_two_step_v_px);
  add_row(ball_approach_rows,buf,ball_color);
  std::snprintf(buf,sizeof(buf),"DOWN 1-step=%.1fpx FINE=%.1fpx",
      bc.down_one_step_v_px,std::min(bc.fine_mode_v_px,bc.pickup_v1_px));
  add_row(ball_fine_rows,buf,ball_color);
  std::snprintf(buf,sizeof(buf),"HURDLE v: 3-step=%.1fpx 2-step=%.1fpx",
      hc.approach_short_v_px,hc.approach_two_step_v_px);
  add_row(hurdle_approach_rows,buf,hurdle_color);
  std::snprintf(buf,sizeof(buf),"DOWN 1-step=%.1fpx FINE=%.1fpx contact=%.1fpx",
      hc.down_one_step_v_px,hc.fine_mode_v_px,hc.contact_v_px);
  add_row(hurdle_fine_rows,buf,hurdle_color);
  std::snprintf(buf,sizeof(buf),"CONTACT: steps=%d | inter-action hold=%.2fs after DONE",
      hc.contact_steps,hc.contact_inter_action_hold_sec);
  add_row(hurdle_fine_rows,buf,hurdle_color);
  add_row(footer_rows,mission_result.recovery_failed ? "LINE SEARCH LIMIT: HOLD / RESET" :
      std::string("Approved: ")+DebugRepeatLabel(view.approved_repeat_scope,config),text_color);
  if (mission_result.goal_walk_gate_active) {
    std::snprintf(buf, sizeof(buf), "GOAL LINE walks %d / %d | 4-step DONE=%s | %s",
        mission_result.goal_walk_actions_completed, gc.post_pickup_walk_fallback_actions,
        mission_result.goal_five_step_completed ? "YES" : "NO",
        mission_result.goal_walk_actions_remaining == 0 ||
        mission_result.goal_five_step_completed ? "CAMERA NEXT" : "WAITING WALK DONE");
    add_row(goal_approach_rows,buf,goal_color);
  }
  if (ball_cmd.mode == vision_core::BallMode::kDownSearch ||
      hurdle_cmd.mode == vision_core::HurdleMode::kDownSearch) {
    if (selected_ball)
      std::snprintf(buf, sizeof(buf), "BALL DOWN SEARCH | remaining %d steps",
          ball_cmd.down_search_steps_remaining);
    else
      std::snprintf(buf, sizeof(buf), "HURDLE DOWN SEARCH | remaining %d steps",
          hurdle_cmd.down_search_steps_remaining);
    add_row(selected_ball ? ball_approach_rows : hurdle_approach_rows,buf,selected_ball ? ball_color : hurdle_color);
  }
  if (ball_cmd.mode == vision_core::BallMode::kBallRecoveryForward ||
      hurdle_cmd.mode == vision_core::HurdleMode::kRecoveryForward ||
      goal_cmd.mode == vision_core::GoalMode::kRecoverySearch ||
      ball_cmd.mode == vision_core::BallMode::kBallRecoveryDown ||
      hurdle_cmd.mode == vision_core::HurdleMode::kRecoveryDown ||
      goal_cmd.mode == vision_core::GoalMode::kFineRecovery ||
      goal_cmd.mode == vision_core::GoalMode::kCoarseRecovery ||
      (selected_goal && goal_cmd.mode==vision_core::GoalMode::kFailed && goal_cmd.recovery_observations_completed>0) ||
      (selected_ball && ball_cmd.mode == vision_core::BallMode::kFailed && ball_cmd.recovery_observations_completed > 0) ||
      (selected_hurdle && hurdle_cmd.mode == vision_core::HurdleMode::kFailed && hurdle_cmd.recovery_observations_completed > 0)) {
    const int completed = selected_ball ? ball_cmd.recovery_observations_completed : selected_goal ? goal_cmd.recovery_observations_completed : hurdle_cmd.recovery_observations_completed;
    const int limit = selected_ball ? bc.recovery_max_observations : selected_goal ? gc.recovery_max_observations : hc.recovery_max_observations;
    const double elapsed = selected_ball ? ball_cmd.recovery_observation_elapsed_sec : selected_goal ? goal_cmd.recovery_observation_elapsed_sec : hurdle_cmd.recovery_observation_elapsed_sec;
    std::snprintf(buf, sizeof(buf), "STATIONARY RECOVERY %d / %d | %.2fs | HOLD", std::min(completed+1, limit), limit, elapsed);
    add_row(selected_ball ? ball_recovery_rows : selected_goal ? goal_recovery_rows : hurdle_recovery_rows,buf,selected_ball ? ball_color : selected_goal ? goal_color : hurdle_color);
  }
  if (mission_result.recovery_verifying) {
    std::snprintf(buf,sizeof(buf),"LINE confirming after rotation | frames=%d/10 | HOLD",
        mission_result.recovery_observation_frames);
    add_row(line_rows,buf,line_color);
  }
  if (selected_hurdle && hurdle_cmd.tracked.visible) {
    std::snprintf(buf, sizeof(buf), "HURDLE weighted v=%.1fpx | error=%+.1fpx | contact margin=%+.1fpx | u alignment OFF",
        hurdle_cmd.tracked.center_px.v, hc.contact_v_px-hurdle_cmd.tracked.center_px.v,
        hurdle_cmd.tracked.center_px.v-hc.contact_v_px);
    add_row(hurdle_fine_rows,buf,hurdle_color);
  }

  std::snprintf(buf,sizeof(buf),"GOAL edge margin=%.1fpx | WALK edge correction: 2 turns / DONE per turn",config.goal.edge_margin_px);
  add_row(goal_approach_rows,buf,goal_color);
  std::snprintf(buf,sizeof(buf),"OBS: %s elapsed=%.2fs frames=%zu valid=%d/%zu",
      mission_result.post_motion_observing ? "HOLD" : "IDLE",mission_result.post_motion_elapsed_sec,
      mission_result.post_motion_frames,mission_result.post_motion_valid,mission_result.post_motion_window_frames);
  add_row(footer_rows,buf,text_color);
  if (!view.imu_enabled) add_row(footer_rows,"IMU: DISABLED",muted);
  else if (!view.imu_ready) add_row(footer_rows,"IMU: NOT READY",muted);
  else { std::snprintf(buf,sizeof(buf),"IMU: roll=%+.1fdeg pitch=%+.1fdeg",view.roll_rad*180/kDebugPi,view.pitch_rad*180/kDebugPi); add_row(footer_rows,buf,text_color); }
  const auto depth_text=[](const std::optional<double>& depth) {
    if(!depth || !std::isfinite(*depth))return std::string("--");
    char value[32];std::snprintf(value,sizeof(value),"%.3fm",*depth);return std::string(value);
  };
  add_row(goal_fine_rows,"BACKBOARD DEPTH: center="+depth_text(prepared.backboard_center_depth_m)+
      " left="+depth_text(prepared.backboard_left_depth_m)+" right="+depth_text(prepared.backboard_right_depth_m),goal_color);
  const auto summary=[&](PanelRows& rows,const char* label,const std::optional<vision_core::ObjectTarget>& target,const cv::Scalar& color) {
    if(!target) {add_row(rows,std::string(label)+": --",color);return;}
    const auto p=target->center_rectified ? target->rectified_center_px : target->center_px;
    std::snprintf(buf,sizeof(buf),"%s conf=%.2f u=%.1f v=%.1f h=%.0f area=%.0f",label,target->confidence,p.u,p.v,target->height_px,target->area_px);
    add_row(rows,buf,color);
  };
  summary(ball_rows,"BALL BBOX",ball_target,ball_color);
  summary(hurdle_rows,"HURDLE BBOX",hurdle_target,hurdle_color);
  summary(goal_rows,"BACKBOARD BBOX",backboard_target,goal_color);
  {
    const auto minimum=config.object_targets.MinimumBoxSize(config.object_targets.ball_class_id);
    std::snprintf(buf,sizeof(buf),"BALL BBOX MIN: width=%.1fpx height=%.1fpx",minimum.first,minimum.second);
    add_row(ball_rows,buf,ball_color);
  }
  {
    const auto minimum=config.object_targets.MinimumBoxSize(config.object_targets.hurdle_class_id);
    std::snprintf(buf,sizeof(buf),"HURDLE BBOX MIN: width=%.1fpx height=%.1fpx",minimum.first,minimum.second);
    add_row(hurdle_rows,buf,hurdle_color);
  }
  {
    const auto minimum=config.object_targets.MinimumBoxSize(config.object_targets.goal_class_id);
    std::snprintf(buf,sizeof(buf),"GOAL BBOX MIN: width=%.1fpx height=%.1fpx",minimum.first,minimum.second);
    add_row(goal_rows,buf,goal_color);
  }
  {
    const auto minimum=config.object_targets.MinimumBoxSize(config.object_targets.backboard_class_id);
    std::snprintf(buf,sizeof(buf),"BACKBOARD BBOX MIN: width=%.1fpx height=%.1fpx",minimum.first,minimum.second);
    add_row(goal_rows,buf,goal_color);
  }

  std::snprintf(buf,sizeof(buf),"BALL TRACKING: stable=%d visible=%d",
      ball_cmd.tracked.stable,ball_cmd.tracked.visible);
  add_row(ball_rows,buf,ball_color);
  std::snprintf(buf,sizeof(buf),"HURDLE TRACKING: stable=%d visible=%d",
      hurdle_cmd.tracked.stable,hurdle_cmd.tracked.visible);
  add_row(hurdle_rows,buf,hurdle_color);
  std::snprintf(buf,sizeof(buf),"GOAL TRACKING: stable=%d visible=%d pose=%d",
      goal_cmd.tracked.stable,goal_cmd.tracked.visible,goal_cmd.pose.visible);
  add_row(goal_rows,buf,goal_color);
  add_row(footer_rows,view.control_hint,muted);
  if(goal_cmd.pose.visible) {
    std::snprintf(buf,sizeof(buf),"GOAL POSE: x=%+.3fm z=%.3fm backboard_yaw=%+.1fdeg stable=%d",
        goal_cmd.pose.x_m,goal_cmd.pose.z_m,goal_cmd.pose.yaw_rad*180/kDebugPi,goal_cmd.pose.stable);
    add_row(goal_fine_rows,buf,goal_color);
  } else add_row(goal_fine_rows,"GOAL POSE: --",goal_color);
  if(landing.valid || goal_cmd.pose.visible || goal_cmd.mode==vision_core::GoalMode::kShoot) {
    std::snprintf(buf,sizeof(buf),"SHOOT ROTATION YAW: %+.1fdeg",
        landing.valid ? landing.yaw_deg : goal_cmd.shoot_yaw_rad*180/kDebugPi);
    add_row(goal_fine_rows,buf,goal_color);
  } else add_row(goal_fine_rows,"SHOOT ROTATION YAW: --",goal_color);
  PanelRows left_rows,right_rows;
  const auto append=[](PanelRows& column,const PanelRows& rows) {
    column.insert(column.end(),rows.begin(),rows.end());
  };
  const auto section=[&](PanelRows& column,const char* title,const PanelRows& rows,
                         const cv::Scalar& color) {
    if (rows.empty()) return;
    column.push_back({title,color,true});
    append(column,rows);
  };
  append(left_rows,line_rows);
  left_rows.push_back({"",text_color,false});
  append(left_rows,ball_rows);
  section(left_rows,"APPROACH / FORWARD + DOWN",ball_approach_rows,ball_color);
  section(left_rows,"FINE / PICKUP",ball_fine_rows,ball_color);
  section(left_rows,"RECOVERY",ball_recovery_rows,ball_color);
  append(right_rows,hurdle_rows);
  section(right_rows,"APPROACH / FORWARD + DOWN",hurdle_approach_rows,hurdle_color);
  section(right_rows,"FINE / CONTACT",hurdle_fine_rows,hurdle_color);
  section(right_rows,"RECOVERY",hurdle_recovery_rows,hurdle_color);
  right_rows.push_back({"",text_color,false});
  append(right_rows,goal_rows);
  section(right_rows,"APPROACH",goal_approach_rows,goal_color);
  section(right_rows,"FINE / DEPTH + SHOOT",goal_fine_rows,goal_color);
  section(right_rows,"RECOVERY",goal_recovery_rows,goal_color);

  constexpr int panel_width=1280,margin=16,gutter=24,line_height=22;
  constexpr int column_width=(panel_width-2*margin-gutter)/2;
  constexpr double panel_font_scale=0.48;
  struct PlacedText { cv::Point position; PanelRow row; };
  std::vector<PlacedText> placed;
  const auto layout=[&](const PanelRows& rows,int x,int y,int width) {
    const auto place=[&](const std::string& text,const PanelRow& row,int& baseline) {
      placed.push_back({{x,baseline},{text,row.color,row.heading}});
      baseline+=line_height;
    };
    for (const auto& row:rows) {
      if (row.text.empty()) {y+=line_height/2;continue;}
      if (row.heading) y+=6;
      std::string remaining=row.text;
      while (!remaining.empty()) {
        std::size_t count=remaining.size();
        int baseline=0;
        while (count>1 && cv::getTextSize(remaining.substr(0,count),
            cv::FONT_HERSHEY_SIMPLEX,panel_font_scale,1,&baseline).width>width)
          --count;
        if (count<remaining.size()) {
          const auto space=remaining.rfind(' ',count);
          if (space!=std::string::npos && space>0) count=space;
        }
        place(remaining.substr(0,count),row,y);
        remaining.erase(0,count);
        const auto first=remaining.find_first_not_of(' ');
        remaining.erase(0,first==std::string::npos ? remaining.size() : first);
      }
    }
    return y;
  };
  const int columns_top=layout(header_rows,margin,28,panel_width-2*margin)+12;
  const int left_bottom=layout(left_rows,margin,columns_top,column_width);
  const int right_bottom=layout(right_rows,margin+column_width+gutter,
                                columns_top,column_width);
  const int footer_top=std::max(left_bottom,right_bottom)+12;
  const int panel_bottom=layout(footer_rows,margin,footer_top,panel_width-2*margin);
  cv::Mat state_panel(std::max(720,panel_bottom+margin),panel_width,CV_8UC3,
                      cv::Scalar(20,20,20));
  cv::line(state_panel,{panel_width/2,columns_top-16},
      {panel_width/2,footer_top-12},cv::Scalar(65,65,65),1);
  cv::line(state_panel,{margin,footer_top-line_height},
      {panel_width-margin,footer_top-line_height},cv::Scalar(65,65,65),1);
  for (const auto& item:placed)
    cv::putText(state_panel,item.row.text,item.position,cv::FONT_HERSHEY_SIMPLEX,
        panel_font_scale,item.row.color,1,cv::LINE_AA);
  cv::putText(vis,view.performance,{10,24},cv::FONT_HERSHEY_SIMPLEX,0.7,{0,255,0},2);
  cv::putText(vis,std::string(DebugMissionName(mission_result.active_mission))+" | "+DebugActionName(mission_result.command.action),
      {10,48},cv::FONT_HERSHEY_SIMPLEX,0.55,{0,200,255},2);
  if (selected_goal) {
    int y=72;
    for (const auto& text:landing_rows) {
      int baseline=0;
      const auto size=cv::getTextSize(text,cv::FONT_HERSHEY_SIMPLEX,0.5,1,&baseline);
      const double scale=0.5*std::min(1.0,
          static_cast<double>(std::max(1,vis.cols-20))/std::max(1,size.width));
      // An outline keeps coordinates readable over bright camera pixels.
      cv::putText(vis,text,{10,y},cv::FONT_HERSHEY_SIMPLEX,scale,{0,0,0},3,cv::LINE_AA);
      cv::putText(vis,text,{10,y},cv::FONT_HERSHEY_SIMPLEX,scale,goal_color,1,cv::LINE_AA);
      y+=22;
    }
  }
  return {std::move(vis),std::move(state_panel)};
}
} // namespace vision
