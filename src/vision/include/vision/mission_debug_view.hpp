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
    return "SEARCH <="+std::to_string(config.ball.down_search_timeout_sec)+"s";
  if (scope==400+static_cast<int>(HurdleMode::kDownSearch))
    return "SEARCH <="+std::to_string(config.hurdle.down_search_max_steps)+" steps";
  if (scope==200+static_cast<int>(BallMode::kPostPickupBackAway)) return "BACK x"+std::to_string(config.ball.post_pickup_back_steps);
  if (scope==300+static_cast<int>(GoalMode::kPostShootBackAway)) return "BACK x"+std::to_string(config.goal.post_shoot_back_steps);
  if (scope==300+static_cast<int>(GoalMode::kEdgeTurn)) return "EDGE TURN x2";
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
  const bool line_confirmed=mission_result.line_computed && feats.guide.valid && !mission_result.line_in_recovery;
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
      !mission_result.line_in_recovery) {
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
         ball_cmd.mode==vision_core::BallMode::kBallRecoveryForward)) {
      draw_u_line(c.far_u_des_px-c.approach_u_deadband_px,cv::Scalar(0,100,255));
      draw_u_line(c.far_u_des_px+c.approach_u_deadband_px,cv::Scalar(0,100,255));
      draw_v_line(c.approach_short_v_px,cv::Scalar(0,200,255));
      draw_v_line(c.approach_two_step_v_px,cv::Scalar(0,100,255));
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

  cv::Mat state_panel(1370, 1100, CV_8UC3, cv::Scalar(20,20,20));
  const cv::Scalar text_color(235,235,235), muted(150,150,150);
  const cv::Scalar line_color(0,230,0), ball_color(0,165,255);
  const cv::Scalar goal_color(0,230,230), hurdle_color(230,100,230);
  const auto tune_line = [&](int y,const std::string& text,const cv::Scalar& color) {
    cv::putText(state_panel,text,{24,y>=112 ? y+60 : y>=107 ? y+24 : y},
        cv::FONT_HERSHEY_SIMPLEX,0.54,color,1,cv::LINE_AA);
  };
  const auto state=mission_result.terminal_hold ? std::string("HOLD / RESET REQUIRED") : view.control_state;
  tune_line(32,"CONTROL: "+state+" | RGB: LIVE",mission_result.terminal_hold ? cv::Scalar(0,0,255) : ball_color);
  char buf[256];
  std::snprintf(buf,sizeof(buf),"ACTION: %s | mission=%s",DebugActionName(mission_result.command.action),DebugMissionName(mission_result.active_mission));
  tune_line(62,std::string(buf)+" | "+DebugRepeatLabel(view.preview_ready ? mission_result.repeat_scope : view.approved_repeat_scope,config),text_color);
  const int ball_index=view.control_state=="WAIT_START" ? config.test_ball_mission_index : mission_result.ball_mission_index;
  const int goal_index=view.control_state=="WAIT_START" ? config.test_goal_mission_index : mission_result.goal_mission_index;
  std::snprintf(buf,sizeof(buf),"BALL #%d %s | GOAL #%d %s | turns=%d/%d",
      ball_index,ball_index%2 ? "RIGHT" : "LEFT",
      goal_index,goal_index%2 ? "RIGHT" : "LEFT",
      mission_result.recovery_turns_completed,mission_result.recovery_max_turns);
  tune_line(84,buf,text_color);
  tune_line(102,"TEST STAGE: "+std::to_string(config.test_stage)+
      (mission_result.trial_complete ? " COMPLETE / SELECT STAGE OR RESET" : mission_result.test_executing ? " EXECUTING / RECOVERY" : mission_result.test_ready ? " READY" : config.test_stage==3 ? " ALIGN IN STAGE 2 FIRST" : ""),text_color);
  tune_line(107,view.waiting_action_id
      ? "FEEDBACK: WAIT EXECUTOR DONE id="+std::to_string(view.waiting_action_id)
      : "FEEDBACK: DONE "+std::to_string(view.last_received_done_id)+
        (view.last_received_done_id!=view.last_processed_done_id ? " / WAIT FRESH IMAGE" : " / APPLIED"),muted);
  tune_line(112,std::string("FEET: ")+(mission_result.foot_pose==vision_core::FootPose::kWalk ? "WALK" : "FINE")+
      (mission_result.foot_transition ? " | TRANSITION / WAIT_DONE" : ""),text_color);
  const auto &lc = config.line_p2p;
  tune_line(130, "LINE", line_color);
  std::snprintf(buf, sizeof(buf),
      "reference=%.1fpx  Ko=%.3f  Kh=%.3f  deadband=%.4f",
      config.line_features.image_center_u,
      lc.offset_gain, lc.heading_gain, lc.steering_deadband);
  tune_line(158, buf, text_color);
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
    tune_line(186, buf, text_color);
    std::snprintf(buf, sizeof(buf), "H=%+.4frad  x  Kh=%.3f  =  %+.4f",
        decision_line.heading_rad, lc.heading_gain, heading_term);
    tune_line(214, buf, text_color);
    // Recovery is selected by the FSM, not by this steering score.
    const char *direction = mission_result.line_in_recovery
        ? "RECOVERY_FSM" : (score < -lc.steering_deadband ? "LEFT" :
            (score > lc.steering_deadband ? "RIGHT" : "STRAIGHT"));
    std::snprintf(buf, sizeof(buf),
        "SCORE=%+.4f  |  band=[%+.4f,%+.4f]  |  %s",
        score, -lc.steering_deadband, lc.steering_deadband, direction);
    const bool inside_band = std::abs(score) <= lc.steering_deadband;
    tune_line(242, buf, mission_result.line_in_recovery ? muted :
        (inside_band ? line_color : ball_color));
  } else {
    tune_line(186, "O x Ko = --", muted);
    tune_line(214, "H x Kh = --", muted);
    tune_line(242, "SCORE: -- (no valid LINE decision)", muted);
  }

  const auto &bc = config.ball;
  tune_line(282, std::string("BALL  |  ") +
      vision_core::BallController::ModeName(ball_cmd.mode), ball_color);
  std::snprintf(buf, sizeof(buf), "target u=%.1fpx band=%.1fpx | 2-step then DOWN v>=%.1fpx",
      bc.far_u_des_px, bc.approach_u_deadband_px, bc.approach_two_step_v_px);
  tune_line(310, buf, text_color);
  if (selected_ball && ball_cmd.mode == vision_core::BallMode::kApproachBall &&
      ball_cmd.tracked.visible) {
    std::snprintf(buf, sizeof(buf),
        "far u=%.1fpx | error=%+.1fpx | band=[%+.1f,%+.1f]px",
        ball_cmd.tracked.center_px.u, ball_cmd.tracked.center_px.u - bc.far_u_des_px,
        -bc.approach_u_deadband_px, bc.approach_u_deadband_px);
    tune_line(338, buf, ball_color);
  } else {
    tune_line(338, "far u-target: --", muted);
  }
  std::snprintf(buf, sizeof(buf), "PICK u=[%.1f,%.1f] v=[%.1f,%.1f] | SIDE x2 excess>=%.1fpx",
      bc.pickup_u1_px, bc.pickup_u2_px, bc.pickup_v1_px, bc.pickup_v2_px, bc.fine_side_two_step_margin_px);
  tune_line(366, buf, text_color);
  const bool ball_fine = selected_ball && ball_cmd.tracked.visible &&
      (ball_cmd.mode == vision_core::BallMode::kFineAdjustForPickup ||
       ball_cmd.mode == vision_core::BallMode::kFineSidePair ||
       ball_cmd.mode == vision_core::BallMode::kPickupBall);
  if (ball_fine) {
    std::snprintf(buf, sizeof(buf), "PICK u=%.1fpx  v=%.1fpx | forward error=%+.1fpx",
        ball_cmd.tracked.center_px.u, ball_cmd.tracked.center_px.v,
        bc.PickupCenterV()-ball_cmd.tracked.center_px.v);
    tune_line(394, buf, ball_color);
  } else {
    tune_line(394, "PICK u/v / forward error: --", muted);
  }
  std::snprintf(buf, sizeof(buf), "ENTRY v<=%.0f | hits=%d/%d need=%d | frames=%d/%d",
      bc.upper_acquire_v_px, ball_cmd.entry_hits, bc.stable_window, bc.entry_min_hits,
      ball_cmd.entry_frames, bc.stable_window);
  tune_line(422, buf, ball_color);

  const auto &gc = config.goal;
  tune_line(462, std::string("GOAL  |  ") +
      vision_core::GoalController::ModeName(goal_cmd.mode), goal_color);
  std::snprintf(buf, sizeof(buf), "far target u=%.1fpx  deadband=%.1fpx  fine z<=%.3fm",
      gc.target_u_px, gc.approach_u_deadband_px, gc.fine_adjust_start_z_m);
  tune_line(490, buf, text_color);
  if (selected_goal && (goal_cmd.mode == vision_core::GoalMode::kApproach ||
      goal_cmd.mode == vision_core::GoalMode::kForwardApproach) &&
      goal_cmd.tracked.visible) {
    std::snprintf(buf, sizeof(buf),
        "far u=%.1fpx | error=%+.1fpx | band=[%+.1f,%+.1f]px",
        goal_cmd.tracked.center_px.u, goal_cmd.tracked.center_px.u - gc.target_u_px,
        -gc.approach_u_deadband_px, gc.approach_u_deadband_px);
    tune_line(518, buf, goal_color);
  } else {
    tune_line(518, "far u-target: --", muted);
  }
  std::snprintf(buf, sizeof(buf), "SHOOT 0deg landing=(%+.1f,%.1f)cm  tol=%.1fcm  limit=%.0fdeg",
      100.0*gc.throw_landing_x_m, 100.0*gc.throw_landing_z_m,
      100.0*gc.position_tolerance_m, gc.shoot_yaw_limit_deg);
  tune_line(546, buf, text_color);
  const auto &fine = goal_cmd.fine_decision_pose;
  if (selected_goal && goal_cmd.fine_decision_pose_used && fine.valid &&
      std::isfinite(fine.x_m) && std::isfinite(fine.z_m) &&
      fine.z_m > 0.0 && std::isfinite(fine.yaw_rad)) {
    // Diagnostic geometry matches GoalController::FineAction's input;
    // this display never selects or publishes an action.
    const double rim_x = fine.x_m + gc.hoop_radius_m * std::sin(fine.yaw_rad);
    const double rim_z = fine.z_m - gc.hoop_radius_m * std::cos(fine.yaw_rad);
    const double distance = std::hypot(rim_x, rim_z);
    const double throw_range = std::hypot(gc.throw_landing_x_m, gc.throw_landing_z_m);
    const double range_error = distance - throw_range;
    const double shoot_yaw_rad = distance > 1e-9
        ? std::atan2(gc.throw_landing_x_m, gc.throw_landing_z_m)
            - std::atan2(rim_x, rim_z) : 0.0;
    const double shoot_yaw_deg = shoot_yaw_rad * 180.0 / kDebugPi;
    const double cosine = std::cos(shoot_yaw_rad), sine = std::sin(shoot_yaw_rad);
    const double land_x = cosine*gc.throw_landing_x_m-sine*gc.throw_landing_z_m;
    const double land_z = sine*gc.throw_landing_x_m+cosine*gc.throw_landing_z_m;
    const double position_error = std::hypot(rim_x-land_x, rim_z-land_z);
    std::snprintf(buf, sizeof(buf),
        "board=(%+.1f,%.1f)cm yaw=%+.1fdeg  rim=(%+.1f,%.1f)cm",
        100.0*fine.x_m,100.0*fine.z_m,fine.yaw_rad*180.0/kDebugPi,
        100.0*rim_x,100.0*rim_z);
    tune_line(574, buf, goal_color);
    std::snprintf(buf, sizeof(buf),
        "land=(%+.1f,%.1f)cm  2D err=%.2fcm  radial err=%+.2fcm",
        100.0*land_x,100.0*land_z,100.0*position_error,100.0*range_error);
    tune_line(602, buf, position_error <= gc.position_tolerance_m
        ? line_color : ball_color);
    std::snprintf(buf, sizeof(buf), "shoot yaw=%+.2fdeg  |  limit=[%+.1f,%+.1f]deg",
        shoot_yaw_deg, -gc.shoot_yaw_limit_deg, gc.shoot_yaw_limit_deg);
    tune_line(630, buf, goal_color);
  } else {
    if (selected_goal && prepared.backboard_center_depth_m &&
        std::isfinite(*prepared.backboard_center_depth_m)) {
      std::snprintf(buf, sizeof(buf), "backboard center depth=%.4fm  |  fine z<=%.3fm",
          *prepared.backboard_center_depth_m, gc.fine_adjust_start_z_m);
      tune_line(574, buf, goal_color);
    } else {
      tune_line(574, "fine input x/z/yaw: --", muted);
    }
    tune_line(602, "predicted landing / 2D error: --", muted);
    tune_line(630, "shoot yaw: --", muted);
  }

  const auto &hc = config.hurdle;
  tune_line(670, std::string("HURDLE  |  ") +
      vision_core::HurdleController::ModeName(hurdle_cmd.mode), hurdle_color);
  std::snprintf(buf, sizeof(buf), "entry v<=%.1fpx | 2-step then DOWN v>=%.1fpx",
      hc.upper_acquire_v_px, hc.approach_two_step_v_px);
  tune_line(698, buf, text_color);
  if (selected_hurdle && hurdle_cmd.mode == vision_core::HurdleMode::kApproach &&
      hurdle_target && vis.rows > 1) {
    const double raw_v = hurdle_target->center_px.v;
    std::snprintf(buf, sizeof(buf),
        "raw v=%.1fpx | v-acquire=%+.1fpx | v-DOWN=%+.1fpx",
        raw_v, raw_v - hc.upper_acquire_v_px, raw_v - hc.approach_two_step_v_px);
    tune_line(726, buf, hurdle_color);
  } else {
    tune_line(726, "raw v / threshold errors: --", muted);
  }

  std::snprintf(buf,sizeof(buf),"BALL v lines: 3-step=%.1fpx 2-step=%.1fpx | DOWN 1-step=%.1fpx FINE=%.1fpx",
      bc.approach_short_v_px,bc.approach_two_step_v_px,
      bc.down_one_step_v_px,
      std::min(bc.fine_mode_v_px,bc.pickup_v1_px));
  tune_line(766,buf,ball_color);
  std::snprintf(buf,sizeof(buf),"HURDLE v: 3-step=%.1fpx 2-step=%.1fpx | DOWN 1-step=%.1fpx FINE=%.1fpx contact=%.1fpx",
      hc.approach_short_v_px,hc.approach_two_step_v_px,
      hc.down_one_step_v_px,
      hc.fine_mode_v_px,hc.contact_v_px);
  tune_line(794,buf,hurdle_color);
  tune_line(822,mission_result.recovery_failed ? "LINE SEARCH LIMIT: HOLD / RESET" :
      std::string("Approved: ")+DebugRepeatLabel(view.approved_repeat_scope,config),text_color);
  if (mission_result.goal_walk_gate_active) {
    std::snprintf(buf, sizeof(buf), "GOAL LINE walks %d / %d | 4-step DONE=%s | %s",
        mission_result.goal_walk_actions_completed, gc.post_pickup_walk_fallback_actions,
        mission_result.goal_five_step_completed ? "YES" : "NO",
        mission_result.goal_walk_actions_remaining == 0 ||
        mission_result.goal_five_step_completed ? "CAMERA NEXT" : "WAITING WALK DONE");
    tune_line(850, buf, goal_color);
  }
  if (ball_cmd.mode == vision_core::BallMode::kDownSearch ||
      hurdle_cmd.mode == vision_core::HurdleMode::kDownSearch) {
    if (selected_ball)
      std::snprintf(buf, sizeof(buf), "BALL DOWN SEARCH | remaining %.2fs",
          ball_cmd.down_search_remaining_sec);
    else
      std::snprintf(buf, sizeof(buf), "HURDLE DOWN SEARCH | remaining %d steps",
          hurdle_cmd.down_search_steps_remaining);
    tune_line(878, buf, text_color);
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
    tune_line(906, buf, text_color);
  }
  if (mission_result.recovery_verifying) {
    std::snprintf(buf,sizeof(buf),"LINE confirming after rotation | frames=%d/10 | HOLD",
        mission_result.recovery_observation_frames);
    tune_line(934,buf,line_color);
  }
  if (selected_hurdle && hurdle_cmd.tracked.visible) {
    std::snprintf(buf, sizeof(buf), "HURDLE weighted v=%.1fpx | error=%+.1fpx | contact margin=%+.1fpx | u alignment OFF",
        hurdle_cmd.tracked.center_px.v, hc.contact_v_px-hurdle_cmd.tracked.center_px.v,
        hurdle_cmd.tracked.center_px.v-hc.contact_v_px);
    tune_line(962, buf, hurdle_color);
  }

  std::snprintf(buf,sizeof(buf),"GOAL edge margin=%.1fpx | WALK edge correction: 2 turns / DONE per turn",config.goal.edge_margin_px);
  tune_line(1000,buf,goal_color);
  std::snprintf(buf,sizeof(buf),"OBS: %s elapsed=%.2fs frames=%zu valid=%d/%zu",
      mission_result.post_motion_observing ? "HOLD" : "IDLE",mission_result.post_motion_elapsed_sec,
      mission_result.post_motion_frames,mission_result.post_motion_valid,mission_result.post_motion_window_frames);
  tune_line(1028,buf,text_color);
  if (!view.imu_enabled) tune_line(1056,"IMU: DISABLED",muted);
  else if (!view.imu_ready) tune_line(1056,"IMU: NOT READY",muted);
  else { std::snprintf(buf,sizeof(buf),"IMU: roll=%+.1fdeg pitch=%+.1fdeg",view.roll_rad*180/kDebugPi,view.pitch_rad*180/kDebugPi); tune_line(1056,buf,text_color); }
  const auto depth_text=[](const std::optional<double>& depth) {
    if(!depth || !std::isfinite(*depth))return std::string("--");
    char value[32];std::snprintf(value,sizeof(value),"%.3fm",*depth);return std::string(value);
  };
  tune_line(1084,"BACKBOARD DEPTH: center="+depth_text(prepared.backboard_center_depth_m)+
      " left="+depth_text(prepared.backboard_left_depth_m)+" right="+depth_text(prepared.backboard_right_depth_m),goal_color);
  const auto summary=[&](int y,const char* label,const std::optional<vision_core::ObjectTarget>& target,const cv::Scalar& color) {
    if(!target) {tune_line(y,std::string(label)+": --",muted);return;}
    const auto p=target->center_rectified ? target->rectified_center_px : target->center_px;
    std::snprintf(buf,sizeof(buf),"%s conf=%.2f u=%.1f v=%.1f h=%.0f area=%.0f",label,target->confidence,p.u,p.v,target->height_px,target->area_px);
    tune_line(y,buf,color);
  };
  summary(1112,"BALL BBOX",ball_target,ball_color);
  summary(1140,"HURDLE BBOX",hurdle_target,hurdle_color);
  summary(1168,"BACKBOARD BBOX",backboard_target,goal_color);
  std::snprintf(buf,sizeof(buf),"TRACKING: BALL stable=%d visible=%d | GOAL stable=%d visible=%d pose=%d | HURDLE stable=%d visible=%d",
      ball_cmd.tracked.stable,ball_cmd.tracked.visible,goal_cmd.tracked.stable,goal_cmd.tracked.visible,
      goal_cmd.pose.visible,hurdle_cmd.tracked.stable,hurdle_cmd.tracked.visible);
  tune_line(1196,buf,text_color);
  tune_line(1224,view.control_hint,muted);
  if(goal_cmd.pose.visible) {
    std::snprintf(buf,sizeof(buf),"GOAL POSE: x=%+.3fm z=%.3fm backboard_yaw=%+.1fdeg stable=%d",
        goal_cmd.pose.x_m,goal_cmd.pose.z_m,goal_cmd.pose.yaw_rad*180/kDebugPi,goal_cmd.pose.stable);
    tune_line(1252,buf,goal_color);
  } else tune_line(1252,"GOAL POSE: --",muted);
  if(goal_cmd.pose.visible || goal_cmd.mode==vision_core::GoalMode::kShoot) {
    std::snprintf(buf,sizeof(buf),"SHOOT ROTATION YAW: %+.1fdeg",goal_cmd.shoot_yaw_rad*180/kDebugPi);
    tune_line(1280,buf,goal_color);
  } else tune_line(1280,"SHOOT ROTATION YAW: --",muted);
  cv::putText(vis,view.performance,{10,24},cv::FONT_HERSHEY_SIMPLEX,0.7,{0,255,0},2);
  cv::putText(vis,std::string(DebugMissionName(mission_result.active_mission))+" | "+DebugActionName(mission_result.command.action),
      {10,48},cv::FONT_HERSHEY_SIMPLEX,0.55,{0,200,255},2);
  return {std::move(vis),std::move(state_panel)};
}
} // namespace vision
