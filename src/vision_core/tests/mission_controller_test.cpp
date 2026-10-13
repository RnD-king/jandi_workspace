#include "vision_core/config_loader.hpp"
#include "vision_core/mission_controller.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <limits>
#include <stdexcept>
using namespace vision_core;
static MissionFrameInput Frame(double now) {
  MissionFrameInput f; f.image_width=640; f.image_height=480; f.now_sec=now;
  f.command_transport_enabled=true;
  f.line_centers={{320,450},{320,400},{320,350},{320,300},{320,250}};
  return f;
}
static LineGuide Guide(double offset) {
  LineGuide g; g.valid=true; g.offset=offset; g.heading_rad=0; return g;
}
static void ShiftLine(MissionFrameInput &frame, double u) {
  for (auto &point : frame.line_centers) point.u = u;
}
static ObjectTarget Ball(double u_norm, double v_norm) {
  ObjectTarget target;
  target.center_px = {u_norm * 640.0, v_norm * 480.0};
  target.width_px = 20.0;
  target.height_px = 20.0;
  target.area_px = 400.0;
  target.confidence = 1.0;
  return target;
}
int main() {
  auto c=LoadDefaultAlgorithmConfig();
  c.line_features.image_center_u=320;
  c.line_p2p.offset_gain=c.line_p2p.heading_gain=1.0;
  c.enable_ball=c.enable_hurdle=c.enable_goal=false;
  MissionController two_point_controller(c);
  auto two_point_input=Frame(0);
  two_point_input.line_centers.resize(2);
  auto two_point_result=two_point_controller.Step(two_point_input);
  assert(!two_point_result.line_features.guide.valid);
  assert(two_point_result.line_in_recovery);
  assert(two_point_result.command.action_id==0);
  assert(two_point_result.command.mission_phase==1); // FailureObserve.
  for (int i=1;i<=4;++i) {
    two_point_input=Frame(.4*i);
    two_point_result=two_point_controller.Step(two_point_input);
    assert(two_point_result.command.action_id==0);
  }
  two_point_input=Frame(2.0);
  two_point_result=two_point_controller.Step(two_point_input);
  assert(!two_point_result.line_in_recovery);
  assert(two_point_result.command.action==MissionAction::kStepForwardFive);
  MissionController controller(c);
  auto in=Frame(0); auto r=controller.Step(in);
  assert(r.command.action==MissionAction::kStepForwardFive);
  assert(r.command.action_id!=0);
  const auto id=r.command.action_id;
  in=Frame(.1); in.delivery_feedback={id,true,false,false};
  r=controller.Step(in); assert(r.command.command_type==CommandType::kHold);
  for (double time : {0.8, 1.4, 1.6, 1.8, 2.0, 2.2}) {
    in=Frame(time); r=controller.Step(in);
  }
  in=Frame(2.55); in.delivery_feedback={id,true,false,true};
  r=controller.Step(in);
  assert(r.command.action_id!=0 && r.command.action_id!=id);
  assert(r.command.action==MissionAction::kStepForwardFive);
  assert(r.command.control_phase==ControlPhase::kWaitingQueuedActionAck);
  assert(r.line_window_stats && r.line_window_stats->used_frames >= 5);
  // Debug telemetry must report the SAME READY average used to select B.
  const auto ready_decision_guide = controller.LastReadyLineGuide();
  assert(ready_decision_guide && r.line_decision_guide);
  assert(std::abs(r.line_decision_guide->offset - ready_decision_guide->offset) < 1e-9);
  assert(std::abs(r.line_decision_guide->heading_rad -
                  ready_decision_guide->heading_rad) < 1e-9);

  // ACK 전 정지 관측은 long-action 후반 accumulator에 포함하지 않는다.
  // ACK 뒤 관측은 모두 straight이므로 READY 예약도 straight여야 한다.
  controller.Reset();
  in=Frame(0); r=controller.Step(in);
  const auto delayed_ack_id=r.command.action_id;
  in=Frame(.1); ShiftLine(in, 500); r=controller.Step(in);
  in=Frame(.7); ShiftLine(in, 500); r=controller.Step(in);
  in=Frame(.8); in.delivery_feedback={delayed_ack_id,true,false,false};
  r=controller.Step(in);
  for (double time : {1.3, 1.7, 1.8, 2.0, 2.1, 2.3}) {
    in=Frame(time); r=controller.Step(in);
  }
  in=Frame(2.55); in.delivery_feedback={delayed_ack_id,true,false,true};
  r=controller.Step(in);
  assert(r.command.action==MissionAction::kStepForwardFive);
  assert(r.command.control_phase==ControlPhase::kWaitingQueuedActionAck);
  assert(r.line_window_stats && r.line_window_stats->used_frames >= 5);

  // READY 집계가 invalid여도 moving action 중에 2초 관측 timer를 시작하지
  // 않는다. stationary failure observation은 current DONE 뒤 시작한다.
  controller.Reset();
  in=Frame(0); r=controller.Step(in);
  const auto moving_id=r.command.action_id;
  in=Frame(.1); in.line_centers.clear();
  in.delivery_feedback={moving_id,true,false,true};
  r=controller.Step(in); assert(!r.line_in_recovery);
  in=Frame(5.0); in.line_centers.clear();
  in.delivery_feedback={moving_id,true,true,false};
  r=controller.Step(in); assert(r.line_in_recovery);
  controller.Reset();
  in=Frame(0); in.line_centers.clear(); r=controller.Step(in);
  assert(r.command.command_type==CommandType::kHold);
  assert(r.line_in_recovery);

  // 안정된 마지막 조향 방향만 기억하고, 첫 실패 관측 2초 뒤 같은 방향으로
  // 양수 15도 TURN을 발행한다.
  controller.Reset();
  in=Frame(0); in.line_decision_guide_override=Guide(-.2);
  r=controller.Step(in);
  assert(r.command.action==MissionAction::kStepForwardLeft);
  const auto left_id=r.command.action_id;
  in=Frame(.1); in.line_centers.clear(); in.line_decision_guide_override.reset();
  in.delivery_feedback={left_id,true,true,false};
  r=controller.Step(in); assert(r.command.command_type==CommandType::kHold);
  in=Frame(2.2); in.line_centers.clear();
  r=controller.Step(in);
  // Recovery turns use the FINE stance: the real pose transition must
  // complete before TURN_LEFT can be dispatched.
  assert(r.command.action==MissionAction::kDefaultPoseMode);
  const auto recovery_pose_id=r.command.action_id;
  assert(recovery_pose_id!=0);
  in=Frame(2.3); in.line_centers.clear();
  in.delivery_feedback={recovery_pose_id,true,true,false};
  r=controller.Step(in);
  assert(r.foot_pose==FootPose::kFine);
  in=Frame(2.4); in.line_centers.clear();
  r=controller.Step(in);
  // DEFAULT_POSE_MODE DONE starts a fresh 1s stationary observation window.
  // Never start turning while that post-pose window is active.
  assert(r.post_motion_observing);
  assert(r.command.action==MissionAction::kNone);
  in=Frame(3.4); in.line_centers.clear();
  r=controller.Step(in);
  assert(!r.post_motion_observing);
  assert(r.command.action==MissionAction::kTurnLeft);
  assert(r.command.target_yaw_deg==15);

  // 방향 evidence가 없으면 최초 관측 + 5회 추가 관측 뒤 FINAL HOLD이며,
  // 다음 유효 frame 하나로 자동 재시작하지 않는다.
  controller.Reset();
  in=Frame(0); in.line_centers.clear(); r=controller.Step(in);
  for (int i=1;i<=6;++i) {
    in=Frame(2.1*i); in.line_centers.clear(); r=controller.Step(in);
    assert(r.command.action==MissionAction::kNone);
  }
  in=Frame(13.0); in.line_decision_guide_override=Guide(0);
  r=controller.Step(in);
  assert(r.command.action==MissionAction::kNone);
  assert(r.command.mission_phase==4);

  // 정상 RIGHT 뒤 READY의 reliable STRAIGHT 판단은 과거 방향 evidence를
  // 지운다. 이후 line loss는 TURN_RIGHT가 아니라 stationary retry다.
  controller.Reset();
  in=Frame(0); in.line_decision_guide_override=Guide(.3);
  r=controller.Step(in);
  const auto right_id=r.command.action_id;
  in=Frame(.1); in.delivery_feedback={right_id,true,false,false};
  r=controller.Step(in);
  for (double time : {.6, .7, .8, .9}) {
    in=Frame(time); r=controller.Step(in);
  }
  in=Frame(1.0); in.delivery_feedback={right_id,true,false,true};
  r=controller.Step(in);
  const auto straight_id=r.command.action_id;
  assert(r.command.action==MissionAction::kStepForwardFive);
  in=Frame(1.1); in.delivery_feedback={straight_id,true,false,false};
  r=controller.Step(in);
  in=Frame(1.2); in.delivery_feedback={right_id,true,true,false};
  r=controller.Step(in);
  in=Frame(1.3); in.line_centers.clear();
  in.delivery_feedback={straight_id,true,true,false};
  r=controller.Step(in);
  in=Frame(3.4); in.line_centers.clear();
  r=controller.Step(in);
  assert(r.command.action==MissionAction::kNone);
  assert(r.command.mission_phase==1);

  // FINAL HOLD에서는 유효한 object detection도 새 mission을 시작하지 않는다.
  auto terminal_config=c;
  terminal_config.line_p2p.failure_observation_sec=0.0;
  terminal_config.line_p2p.no_evidence_max_retries=0;
  terminal_config.enable_ball=true;
  terminal_config.enable_hurdle=false;
  terminal_config.ball.stable_window=1;
  terminal_config.ball.stable_min_hits=1;
  MissionController terminal(terminal_config);
  in=Frame(0); in.line_centers.clear(); r=terminal.Step(in);
  in=Frame(.1); in.line_centers.clear(); r=terminal.Step(in);
  assert(r.command.mission_phase==4);
  in=Frame(.2); in.line_centers.clear(); in.ball_target=Ball(.5,.3);
  r=terminal.Step(in);
  assert(r.active_mission==MissionType::kLine);
  assert(r.command.action==MissionAction::kNone);
  assert(r.command.mission_phase==4);

  // directional recovery는 정확히 설정된 5회만 TURN하고 그 뒤 terminal이다.
  auto turn_config=c;
  turn_config.line_p2p.failure_observation_sec=0.0;
  turn_config.line_p2p.recovery_observation_sec=0.0; // Legacy instant-turn fixture.
  turn_config.line_p2p.recovery_max_turns=5;
  MissionController turning(turn_config);
  in=Frame(0); in.line_decision_guide_override=Guide(-.3);
  r=turning.Step(in);
  auto action_id=r.command.action_id;
  in=Frame(.1); in.line_centers.clear();
  in.delivery_feedback={action_id,true,true,false};
  r=turning.Step(in);
  in=Frame(.2); in.line_centers.clear(); r=turning.Step(in);
  assert(r.command.action==MissionAction::kDefaultPoseMode);
  const auto turn_pose_id=r.command.action_id;
  assert(turn_pose_id!=0);
  in=Frame(.25); in.line_centers.clear();
  in.delivery_feedback={turn_pose_id,true,true,false};
  r=turning.Step(in);
  assert(r.foot_pose==FootPose::kFine);
  in=Frame(.3); in.line_centers.clear(); r=turning.Step(in);
  assert(r.post_motion_observing);
  assert(r.command.action==MissionAction::kNone);
  in=Frame(1.3); in.line_centers.clear(); r=turning.Step(in);
  assert(!r.post_motion_observing);
  for (int turn=1; turn<=5; ++turn) {
    assert(r.command.action==MissionAction::kTurnLeft);
    assert(r.command.target_yaw_deg==15);
    action_id=r.command.action_id;
    const double done_time=1.3+turn*.2-.1;
    in=Frame(done_time); in.line_centers.clear();
    in.delivery_feedback={action_id,true,true,false};
    r=turning.Step(in);
    in=Frame(done_time+.1); in.line_centers.clear();
    r=turning.Step(in);
  }
  assert(r.command.action==MissionAction::kNone);
  assert(r.command.mission_phase==4);

  // one-shot gate는 기존 action ACK/DONE을 처리하되 READY queue와 DONE
  // successor를 새로 만들지 않는다.
  MissionController gated(c);
  in=Frame(0); r=gated.Step(in);
  const auto gated_id=r.command.action_id;
  assert(gated_id!=0);
  in=Frame(.1); in.allow_new_line_action=false;
  in.delivery_feedback={gated_id,true,false,false};
  r=gated.Step(in);
  assert(r.command.action_id==gated_id);
  assert(r.command.control_phase==ControlPhase::kWaitingActionDone);
  in=Frame(.2); in.allow_new_line_action=false;
  in.delivery_feedback={gated_id,true,false,true};
  r=gated.Step(in);
  assert(r.command.action_id==gated_id);
  assert(r.command.control_phase==ControlPhase::kWaitingActionDone);
  in=Frame(.3); in.allow_new_line_action=false;
  in.delivery_feedback={gated_id,true,true,false};
  r=gated.Step(in);
  assert(r.command.action_id==0);
  assert(r.command.action==MissionAction::kNone);

  // gate가 내려가기 전에 만들어진 queued action은 취소하지 않고 한 번
  // 승격·완료시키며, 그 뒤 successor만 금지한다.
  MissionController queued(c);
  in=Frame(0); r=queued.Step(in);
  const auto current_id=r.command.action_id;
  in=Frame(.1); in.delivery_feedback={current_id,true,false,false};
  r=queued.Step(in);
  // Simulate a 2.8s long LINE action: READY occurs ~0.25s before DONE.
  // The estimated 40%-to-READY window must actually contain valid frames.
  for (double time : {0.8, 1.4, 1.6, 1.8, 2.0, 2.2}) {
    in=Frame(time); r=queued.Step(in);
  }
  in=Frame(2.55); in.delivery_feedback={current_id,true,false,true};
  r=queued.Step(in);
  const auto queued_id=r.command.action_id;
  assert(queued_id!=0 && queued_id!=current_id);
  assert(r.command.control_phase==ControlPhase::kWaitingQueuedActionAck);
  assert(r.line_window_stats && r.line_window_stats->window_frames >= 5);
  assert(r.line_window_stats->used_frames >= 5);
  in=Frame(2.6); in.allow_new_line_action=false;
  in.delivery_feedback={queued_id,true,false,false};
  r=queued.Step(in);
  assert(r.command.action_id==queued_id);
  assert(r.command.control_phase==ControlPhase::kWaitingQueuedActionStart);
  in=Frame(2.8); in.allow_new_line_action=false;
  in.delivery_feedback={current_id,true,true,false};
  r=queued.Step(in);
  assert(r.command.action_id==queued_id);
  assert(r.command.control_phase==ControlPhase::kWaitingActionDone);
  in=Frame(5.35); in.allow_new_line_action=false;
  in.delivery_feedback={queued_id,true,true,false};
  r=queued.Step(in);
  assert(r.command.action_id==0);

  // runtime tuning은 완성 config로 검증한 뒤 원자적으로 적용한다.
  MissionController tuning(c);
  auto valid_tuning=c.line_p2p;
  valid_tuning.offset_gain=0.0;
  assert(tuning.UpdateLineP2pTuning(valid_tuning));
  auto invalid_tuning=valid_tuning;
  invalid_tuning.offset_gain=-1.0;
  bool rejected=false;
  try { (void)tuning.UpdateLineP2pTuning(invalid_tuning); }
  catch (const std::runtime_error &) { rejected=true; }
  assert(rejected);
  invalid_tuning=valid_tuning;
  invalid_tuning.steering_deadband=-1.0;
  rejected=false;
  try { (void)tuning.UpdateLineP2pTuning(invalid_tuning); }
  catch (const std::runtime_error &) { rejected=true; }
  assert(rejected);
  invalid_tuning=valid_tuning;
  invalid_tuning.heading_gain=std::numeric_limits<double>::quiet_NaN();
  rejected=false;
  try { (void)tuning.UpdateLineP2pTuning(invalid_tuning); }
  catch (const std::runtime_error &) { rejected=true; }
  assert(rejected);
  in=Frame(0); in.line_decision_guide_override=Guide(.5);
  r=tuning.Step(in);
  assert(r.command.action==MissionAction::kStepForwardFive);

  // Tuning observations compute features but never start production recovery.
  MissionController frozen(c);
  for (int i=0; i<=30; ++i) {
    in=Frame(i); in.line_centers.clear();
    in.advance_line_fsm=false; in.allow_new_line_action=false;
    r=frozen.Step(in);
    assert(r.line_computed && !r.line_features.guide.valid);
    assert(!r.line_in_recovery && r.command.mission_phase==0);
    assert(r.command.action_id==0);
  }
  // No hidden active observation remains to block the next trial's tuning.
  assert(frozen.UpdateLineP2pTuning(c.line_p2p));
  in=Frame(31); in.line_decision_guide_override=Guide(-.3);
  r=frozen.Step(in);
  assert(r.command.action==MissionAction::kStepForwardLeft);
  const auto frozen_id=r.command.action_id;
  in=Frame(32); in.line_centers.clear();
  in.advance_line_fsm=false; in.allow_new_line_action=false;
  in.delivery_feedback={frozen_id,true,false,false};
  r=frozen.Step(in);
  assert(r.command.action_id==frozen_id);
  assert(r.command.control_phase==ControlPhase::kWaitingActionDone);
  in.now_sec=33; in.delivery_feedback={frozen_id,true,false,true};
  r=frozen.Step(in);
  assert(r.command.action_id==frozen_id); // No READY successor.
  assert(r.command.control_phase==ControlPhase::kWaitingActionDone);
  for (int i=34; i<45; ++i) {
    in.now_sec=i; in.delivery_feedback={};
    r=frozen.Step(in);
    assert(!r.line_in_recovery && r.command.action_id==frozen_id);
  }
  in.now_sec=45; in.delivery_feedback={frozen_id,true,true,false};
  r=frozen.Step(in);
  assert(r.command.action_id==0 && !r.line_in_recovery);
  assert(frozen.UpdateLineP2pTuning(c.line_p2p));
  in=Frame(46); in.line_decision_guide_override=Guide(.3);
  r=frozen.Step(in);
  assert(r.command.action==MissionAction::kStepForwardRight);
  assert(r.command.action_id>frozen_id); // No Reset between trials.

  // PerceptionFrameInput must forward the same freeze contract.
  MissionController frozen_perception(c);
  PerceptionFrameInput observation;
  observation.image_width=640; observation.image_height=480;
  observation.command_transport_enabled=true;
  observation.advance_line_fsm=false;
  // Even with allow_new_line_action=true, a frozen FSM makes no decisions.
  for (int i=0; i<20; ++i) {
    observation.now_sec=i;
    const auto p=frozen_perception.StepPerception(observation);
    assert(p.mission.line_computed && !p.mission.line_in_recovery);
    assert(p.mission.command.action_id==0);
  }
  observation.now_sec=20; observation.advance_line_fsm=true;
  observation.line_decision_guide_override=Guide(0);
  const auto p=frozen_perception.StepPerception(observation);
  assert(p.mission.command.action==MissionAction::kStepForwardFive);

  // One common timeout is injected into all three object controllers. Use a
  // non-default value so an accidental per-mission/default 3s is detectable.
  auto camera_config=c;
  camera_config.camera_motion_timeout_sec=.5;
  camera_config.enable_ball=true;
  camera_config.ball.stable_window=camera_config.ball.stable_min_hits=1;
  camera_config.ball.smooth_alpha=1;
  camera_config.ball.upper_acquire_v_px=480.0;

  MissionController ball_timeout(camera_config);
  in=Frame(0); in.ball_target=Ball(.5,.8);
  r=ball_timeout.Step(in);
  assert(r.ball.mode==BallMode::kApproachTwoBeforeDown);
  const auto ball_two_id=r.command.action_id;
  in.now_sec=.01; in.delivery_feedback={ball_two_id,true,true,false};
  r=ball_timeout.Step(in); in.delivery_feedback={};
  assert(r.ball.mode==BallMode::kWaitCameraDown);
  in.now_sec=.4; r=ball_timeout.Step(in);
  assert(r.ball.mode==BallMode::kWaitCameraDown);
  in.now_sec=.51; r=ball_timeout.Step(in);
  assert(r.ball.mode==BallMode::kFailed);

  camera_config.enable_ball=false; camera_config.enable_hurdle=true;
  camera_config.hurdle.stable_window=camera_config.hurdle.stable_min_hits=1;
  camera_config.hurdle.smooth_alpha=1;
  camera_config.hurdle.upper_acquire_v_px=480;

  MissionController hurdle_timeout(camera_config);
  in=Frame(0); in.hurdle_target=Ball(.5,.8);
  r=hurdle_timeout.Step(in);
  assert(r.hurdle.mode==HurdleMode::kApproachTwoBeforeDown);
  const auto hurdle_two_id=r.command.action_id;
  in.now_sec=.01; in.delivery_feedback={hurdle_two_id,true,true,false};
  r=hurdle_timeout.Step(in); in.delivery_feedback={};
  assert(r.hurdle.mode==HurdleMode::kWaitCameraDown);
  in.now_sec=.4; r=hurdle_timeout.Step(in);
  assert(r.hurdle.mode==HurdleMode::kWaitCameraDown);
  in.now_sec=.51; r=hurdle_timeout.Step(in);
  assert(r.hurdle.mode==HurdleMode::kFailed);

  // Suspend association during camera motion and use only fresh settled IDs.
  auto association_config=c;
  MissionController camera_tracking(association_config);
  PerceptionFrameInput frame;
  frame.image_width=640; frame.image_height=480;
  frame.command_transport_enabled=false;
  frame.camera_feedback={CameraMode::kForward,true};
  PerceptionDetection camera_ball;
  camera_ball.detection.class_id=association_config.object_targets.ball_class_id;
  camera_ball.detection.confidence=1.0;
  camera_ball.detection.box={80,80,40,40};
  frame.detections={camera_ball};
  auto scan=camera_tracking.StepPerception(frame);
  assert(scan.perception.targets.ball.has_value());
  frame.camera_feedback={CameraMode::kTransition,false};
  camera_ball.detection.box={310,80,40,40};
  frame.detections={camera_ball};
  scan=camera_tracking.StepPerception(frame);
  assert(!scan.perception.targets.ball.has_value());
  frame.camera_feedback={CameraMode::kDown,true};
  camera_ball.detection.box={550,80,40,40};
  frame.detections={camera_ball};
  scan=camera_tracking.StepPerception(frame);
  assert(scan.perception.targets.ball.has_value());
  assert(scan.perception.targets.ball->center_px.u>500.0);

  camera_config.enable_hurdle=false; camera_config.enable_goal=true;
  camera_config.initial_has_ball=true;
  camera_config.goal.post_pickup_wait_sec=0;
  camera_config.line.line_stable_window=camera_config.line.line_stable_min_hits=1;
  camera_config.goal.stable_window=camera_config.goal.stable_min_hits=1;
  MissionController goal_timeout(camera_config);
  in=Frame(0); in.backboard_target=Ball(.5,.3); in.backboard_center_depth_m=.7;
  r=goal_timeout.Step(in);
  assert(r.goal.mode==GoalMode::kWaitCameraGoal);
  in.now_sec=.4; r=goal_timeout.Step(in);
  assert(r.goal.mode==GoalMode::kWaitCameraGoal);
  in.now_sec=.5; r=goal_timeout.Step(in);
  assert(r.goal.mode==GoalMode::kFailed);
  // GOAL camera DONE starts a fresh 1-second observation. Its final mean
  // must also satisfy recent 10/7 object evidence without extending 1 s.
  auto camera_gate_cfg = c;
  camera_gate_cfg.enable_ball = camera_gate_cfg.enable_hurdle = false;
  camera_gate_cfg.enable_goal = true;
  camera_gate_cfg.initial_has_ball = true;
  camera_gate_cfg.goal.post_pickup_wait_sec = 0.0;
  camera_gate_cfg.post_motion_observation_sec = 1.0;
  camera_gate_cfg.goal.stable_window = 10;
  camera_gate_cfg.goal.stable_min_hits = 7;
  // This test checks the RIGHT-side action, not live calibrated 380 +/-160 ROI.
  camera_gate_cfg.goal.target_u_px = 320.0;
  camera_gate_cfg.goal.approach_u_deadband_px = 16.0;
  MissionController camera_gate(camera_gate_cfg);
  for (int i = 0; i < 7; ++i) {
    in = Frame(.1 * i);
    in.allow_new_line_action = false;
    in.backboard_target=Ball(.5,.3); in.backboard_center_depth_m=.7;
    r = camera_gate.Step(in);
  }
  assert(r.active_mission == MissionType::kGoal);
  assert(r.goal.mode == GoalMode::kWaitCameraGoal);
  in = Frame(1.0);
  in.camera_feedback = {CameraMode::kTransition, false};
  r = camera_gate.Step(in);
  assert(r.command.action_id == 0);
  in = Frame(1.1);
  in.camera_feedback = {CameraMode::kGoal, true};
  r = camera_gate.Step(in);
  assert(r.goal.mode == GoalMode::kSearch);
  assert(r.command.action_id == 0);
  for (int i = 0; i < 9; ++i) {
    in = Frame(1.2 + 0.1 * i);
    in.camera_feedback = {CameraMode::kGoal, true};
    if (i < 7) in.backboard_target = Ball(.8, .3);
    r = camera_gate.Step(in);
    assert(r.command.action_id == 0);
    assert(r.command.action == MissionAction::kNone);
    assert(r.post_motion_observing);
    assert(r.post_motion_frames == static_cast<std::size_t>(i+1));
  }
  in = Frame(2.1);
  in.camera_feedback = {CameraMode::kGoal, true};
  in.backboard_target = Ball(.8, .3);
  r = camera_gate.Step(in);
  assert(r.active_mission == MissionType::kGoal);
  assert(r.command.action == MissionAction::kStepForwardRightTwo);
  assert(r.command.action_id != 0);
  // The next OBJECT action must use 10 NEW frames after this actual Action DONE.
  // Its camera remained GOAL throughout, so only the Action event resets evidence.
  const std::uint64_t approach_id = r.command.action_id;
  in = Frame(2.15);
  in.camera_feedback = {CameraMode::kGoal, true};
  in.backboard_target = Ball(.8, .3);
  in.delivery_feedback = {approach_id, true, false, false};
  r = camera_gate.Step(in);
  assert(r.command.command_type == CommandType::kHold);
  in = Frame(2.2);
  in.camera_feedback = {CameraMode::kGoal, true};
  in.backboard_target = Ball(.8, .3);
  in.delivery_feedback = {approach_id, true, true, false};
  r = camera_gate.Step(in);
  assert(r.post_motion_observing);
  for (int i = 1; i <= 9; ++i) {
    in = Frame(2.2 + 0.1 * i);
    in.camera_feedback = {CameraMode::kGoal, true};
    in.backboard_target = Ball(.8, .3);
    r = camera_gate.Step(in);
    assert(r.command.command_type == CommandType::kHold);
    assert(r.post_motion_observing);
  }
  assert(r.post_motion_window_frames == 10);
  assert(r.post_motion_valid == 10);
  in = Frame(3.2);
  in.camera_feedback = {CameraMode::kGoal, true};
  in.backboard_target = Ball(.8, .3);
  r = camera_gate.Step(in);
  assert(!r.post_motion_observing);
  assert(r.command.command_type == CommandType::kAction);
  assert(r.command.action_id != 0 && r.command.action_id != approach_id);

  // Manual NEXT: a 10/7-acquired BALL may switch camera automatically,
  // but previewing PICK must not increment attempts or change the real FSM.
  auto manual_config=c;
  manual_config.enable_ball=true;
  manual_config.ball.stable_window=manual_config.ball.stable_min_hits=1;

  manual_config.ball.fine_settle_duration_sec=0;
  manual_config.ball.upper_acquire_v_px=480.0;
  manual_config.ball.pickup_u1_px=281.6; manual_config.ball.pickup_u2_px=358.4;
  manual_config.ball.pickup_v1_px=307.2; manual_config.ball.pickup_v2_px=364.8;
  manual_config.post_motion_observation_sec=1;
  MissionController manual(manual_config);
  auto mi=Frame(0);
  mi.manual_observation_control=true; mi.allow_new_body_actions=false;
  mi.ball_target=Ball(.5,.30);
  auto mr=manual.Step(mi);
  assert(mr.active_mission==MissionType::kBall);
  assert(mr.command.action_id==0);
  mi=Frame(.1); mi.manual_observation_control=true; mi.allow_new_body_actions=false;
  mi.ball_target=Ball(.5,.85);
  mr=manual.Step(mi);
  assert(mr.command.camera_request==CameraRequest::kNone);
  mi.manual_observation_control=true; mi.allow_new_body_actions=true;
  mr=manual.Step(mi);
  assert(mr.command.action==MissionAction::kStepForwardTwo);
  mi.now_sec=.11; mi.manual_observation_control=true; mi.allow_new_body_actions=false;
  mi.delivery_feedback={mr.command.action_id,true,true,false};
  mr=manual.Step(mi);
  assert(mr.command.camera_request==CameraRequest::kDown);
  mi=Frame(.2); mi.manual_observation_control=true; mi.allow_new_body_actions=false;
  mi.camera_feedback={CameraMode::kDown,true};
  mi.ball_target=Ball(.5,.7);
  mr=manual.Step(mi);
  mi=Frame(.3); mi.manual_observation_control=true; mi.allow_new_body_actions=false;
  mi.camera_feedback={CameraMode::kDown,true};
  mi.ball_target=Ball(.5,.7);
  mr=manual.Step(mi);
  assert(mr.ball.mode==BallMode::kCoarseAdjust);
  mi.now_sec=.31; mi.manual_observation_control=true; mi.allow_new_body_actions=true;
  mr=manual.Step(mi);
  assert(mr.command.action==MissionAction::kDefaultPoseMode);
  mi.now_sec=.32; mi.manual_observation_control=true; mi.allow_new_body_actions=false;
  mi.delivery_feedback={mr.command.action_id,true,true,false};
  mr=manual.Step(mi);
  mi.now_sec=.33; mi.delivery_feedback={}; mr=manual.Step(mi);
  assert(mr.ball.mode==BallMode::kFineAdjustForPickup);
  assert(mr.ball.pickup_attempt_count==0);
  assert(mr.command.action_id==0);
  mi.manual_observation_control=true; mi.allow_new_body_actions=true;
  MissionController speculative_manual(manual);
  const auto preview=speculative_manual.Step(mi);
  assert(preview.command.action==MissionAction::kPickBall);
  mi.manual_observation_control=true; mi.allow_new_body_actions=false;
  mr=manual.Step(mi);
  assert(mr.ball.mode==BallMode::kFineAdjustForPickup);
  assert(mr.ball.pickup_attempt_count==0);
  mi.manual_observation_control=true; mi.allow_new_body_actions=true;
  mr=manual.Step(mi);
  assert(mr.command.action==MissionAction::kPickBall);
  assert(mr.ball.pickup_attempt_count==1);

  // Separate copy-preview does not consume an ID or modify real LINE FSM.
  MissionController line_manual(c);
  PerceptionFrameInput line_frame;
  line_frame.image_width=640; line_frame.image_height=480;
  line_frame.intrinsics={600,600,320,240};
  line_frame.command_transport_enabled=true;
  for (int i=0;i<5;++i) {
    PerceptionDetection d;
    d.detection.class_id=c.line_detection.class_id;
    d.detection.confidence=1.0;
    d.detection.box={310.0,450.0-40.0*i,20.0,20.0};
    line_frame.detections.push_back(d);
  }
  const auto ephemeral=line_manual.PreviewPerception(line_frame);
  assert(ephemeral.mission.line_decision_guide);
  assert(ephemeral.mission.line_decision_guide->valid);
  assert(std::abs(ephemeral.mission.line_decision_guide->offset -
                  ephemeral.mission.line_features.guide.offset) < 1e-9);
  assert(ephemeral.mission.command.action_id==1);
  assert(ephemeral.mission.command.action==MissionAction::kStepForwardFive);
  auto ref_adjust=c;
  ref_adjust.line_features.image_center_u=250.0;
  assert(line_manual.UpdateTuning(ref_adjust));
  const auto shifted=line_manual.PreviewPerception(line_frame);
  assert(shifted.mission.line_decision_guide);
  assert(shifted.mission.line_decision_guide->offset >
         ephemeral.mission.line_decision_guide->offset + 0.10);
  assert(std::abs(shifted.mission.line_decision_guide->offset -
                  shifted.mission.line_features.guide.offset) < 1e-9);
  assert(shifted.mission.command.action==MissionAction::kStepForwardRight);
  const auto issued=line_manual.StepPerception(line_frame);
  assert(issued.mission.command.action_id==1);
  assert(issued.mission.command.action==MissionAction::kStepForwardRight);

  // A 1s post-camera gate must RELEASE even with fewer than seven
  // detections. The object recovery/search FSM, not the observation gate,
  // decides what to do with missing observations.
  MissionController sparse_gate(camera_gate_cfg);
  for (int i=0;i<7;++i) {
    in=Frame(.1*i);
    in.allow_new_line_action=false;
    r=sparse_gate.Step(in);
  }
  in=Frame(1.0); in.camera_feedback={CameraMode::kTransition,false};
  r=sparse_gate.Step(in);
  in=Frame(1.1); in.camera_feedback={CameraMode::kGoal,true};
  r=sparse_gate.Step(in);
  for (int i=0;i<9;++i) {
    in=Frame(1.2+.1*i);
    in.camera_feedback={CameraMode::kGoal,true};
    // Only one valid frame: previously this would block indefinitely.
    if (i==7) in.backboard_target=Ball(.8,.3);
    r=sparse_gate.Step(in);
    assert(r.post_motion_observing);
  }
  in=Frame(2.1); in.camera_feedback={CameraMode::kGoal,true};
  r=sparse_gate.Step(in);
  assert(!r.post_motion_observing);
  assert(r.active_mission==MissionType::kGoal);
  // One accidental detection near the end cannot become a tracked mean.
  assert(!r.goal.tracked.visible);
  assert(r.command.action == MissionAction::kNone);

  // HURDLE initial stable acquisition is now recent 10 frames / 7 hits.
  const auto defaults = LoadDefaultAlgorithmConfig();
  assert(defaults.hurdle.stable_window == 10);
  assert(defaults.hurdle.stable_min_hits == 7);
  assert(defaults.post_motion_observation_sec == 1.0);
  assert(defaults.line_p2p.failure_observation_sec == 2.0);
  assert(defaults.line_p2p.recovery_observation_sec == 1.0);
  // PREVIEW honors the original LINE failure observation duration (2 s)
  // rather than silently truncating it to the normal stationary 1 s.
  MissionController manual_recovery(c);
  auto recovery_frame = Frame(0.0);
  recovery_frame.line_centers.clear();
  recovery_frame.manual_observation_control = true;
  recovery_frame.allow_new_commands = false;
  (void)manual_recovery.Step(recovery_frame);
  assert(manual_recovery.ManualPreviewDurationSec() >=
         c.line_p2p.failure_observation_sec);
  // Manual tuning: camera/DONE must not spontaneously open a 1s gate.
  // An explicit PREVIEW starts that gate and only the expiry releases it.
  MissionController explicitly_previewed(c);
  auto manual_input = Frame(0.0);
  manual_input.manual_observation_control = true;
  manual_input.allow_new_commands = false;
  manual_input.allow_new_body_actions = false;
  manual_input.camera_feedback = {CameraMode::kTransition, false};
  auto manual_result = explicitly_previewed.Step(manual_input);
  assert(!manual_result.post_motion_observing);
  manual_input = Frame(0.1);
  manual_input.manual_observation_control = true;
  manual_input.allow_new_commands = false;
  manual_input.allow_new_body_actions = false;
  manual_input.camera_feedback = {CameraMode::kDown, true};
  manual_result = explicitly_previewed.Step(manual_input);
  assert(!manual_result.post_motion_observing);
  manual_input = Frame(0.2);
  manual_input.manual_observation_control = true;
  manual_input.begin_manual_observation = true;
  manual_input.allow_new_commands = false;
  manual_input.allow_new_body_actions = false;
  manual_input.camera_feedback = {CameraMode::kDown, true};
  manual_result = explicitly_previewed.Step(manual_input);
  assert(manual_result.post_motion_observing);
  manual_input.begin_manual_observation = false;
  manual_input.now_sec = 0.7;
  manual_result = explicitly_previewed.Step(manual_input);
  assert(manual_result.post_motion_observing);
  manual_input.now_sec = 1.2;
  manual_result = explicitly_previewed.Step(manual_input);
  assert(!manual_result.post_motion_observing);

  // Global PAUSE gate must hold new LINE requests, then release with fresh
  // evidence. A RESET in the same executor session preserves action IDs.
  MissionController pausable(c);
  auto paused_frame=Frame(0.0);
  paused_frame.allow_new_commands=false;
  auto paused_result=pausable.Step(paused_frame);
  assert(paused_result.command.action_id==0);
  assert(paused_result.command.action==MissionAction::kNone);
  paused_frame=Frame(.1);
  paused_frame.allow_new_commands=true;
  paused_result=pausable.Step(paused_frame);
  assert(paused_result.command.action_id==1);
  pausable.ClearObservationEvidence();
  pausable.Reset(true);
  paused_frame=Frame(.2);
  paused_result=pausable.Step(paused_frame);
  assert(paused_result.command.action_id==2);
  pausable.Reset();
  paused_frame=Frame(.3);
  paused_result=pausable.Step(paused_frame);
  assert(paused_result.command.action_id==1);

  // BALL: 4-step 40%-READY mean should select 2-step at DONE even if
  // the raw DONE frame would incorrectly select four-step.
  auto ready_cfg=LoadDefaultAlgorithmConfig();
  ready_cfg.enable_ball=true;
  ready_cfg.enable_goal=ready_cfg.enable_hurdle=false;
  ready_cfg.ball.stable_window=ready_cfg.ball.stable_min_hits=1;
  ready_cfg.ball.upper_acquire_v_px=480;
  MissionController ball_ready(ready_cfg);
  auto ready_frame=Frame(0.0);
  ready_frame.ball_target=Ball(.5,.2);
  auto ready_result=ball_ready.Step(ready_frame);
  assert(ready_result.command.action==MissionAction::kStepForwardFour);
  const auto ball_ready_id=ready_result.command.action_id;
  for (int i=1;i<=27;++i) {
    ready_frame=Frame(i*.1);
    ready_frame.ball_target=Ball(.5,i>=12 ? .8 : .2);
    ready_frame.delivery_feedback={ball_ready_id,true,false,i==27};
    ready_result=ball_ready.Step(ready_frame);
    assert(ready_result.command.action_id==ball_ready_id);
  }
  ready_frame=Frame(3.0); ready_frame.ball_target=Ball(.5,.2);
  ready_frame.delivery_feedback={ball_ready_id,true,true,false};
  ready_result=ball_ready.Step(ready_frame);
  assert(!ready_result.post_motion_observing);
  assert(ready_result.ball.mode==BallMode::kApproachTwoBeforeDown);
  assert(ready_result.command.action==MissionAction::kStepForwardTwo);
  assert(ready_result.command.action_id!=ball_ready_id);

  // Missing READY: reject moving samples and retain stationary 1s gate.
  MissionController no_ready(ready_cfg);
  ready_frame=Frame(0.0); ready_frame.ball_target=Ball(.5,.2);
  ready_result=no_ready.Step(ready_frame);
  const auto no_ready_id=ready_result.command.action_id;
  for (int i=1;i<=27;++i) {
    ready_frame=Frame(i*.1); ready_frame.ball_target=Ball(.5,.8);
    ready_frame.delivery_feedback={no_ready_id,true,false,false};
    ready_result=no_ready.Step(ready_frame);
  }
  ready_frame=Frame(3.0); ready_frame.ball_target=Ball(.5,.8);
  ready_frame.delivery_feedback={no_ready_id,true,true,false};
  ready_result=no_ready.Step(ready_frame);
  assert(ready_result.post_motion_observing);
  assert(ready_result.command.action_id==0);

  // HURDLE: four-step mean leads to three-step, observation_complete
  // must be set when consuming verified READY evidence at DONE.
  auto hurdle_ready_cfg=ready_cfg;
  hurdle_ready_cfg.enable_ball=false; hurdle_ready_cfg.enable_hurdle=true;
  hurdle_ready_cfg.hurdle.stable_window=hurdle_ready_cfg.hurdle.stable_min_hits=1;
  hurdle_ready_cfg.hurdle.upper_acquire_v_px=480;
  MissionController hurdle_ready(hurdle_ready_cfg);
  ready_frame=Frame(0.0); ready_frame.hurdle_target=Ball(.5,.3);
  ready_result=hurdle_ready.Step(ready_frame);
  assert(ready_result.command.action==MissionAction::kStepForwardFour);
  const auto hurdle_ready_id=ready_result.command.action_id;
  for (int i=1;i<=27;++i) {
    ready_frame=Frame(i*.1);
    ready_frame.hurdle_target=Ball(.5,i>=12 ? .7 : .3);
    ready_frame.delivery_feedback={hurdle_ready_id,true,false,i==27};
    ready_result=hurdle_ready.Step(ready_frame);
  }
  ready_frame=Frame(3.0); ready_frame.hurdle_target=Ball(.5,.9);
  ready_frame.delivery_feedback={hurdle_ready_id,true,true,false};
  ready_result=hurdle_ready.Step(ready_frame);
  assert(!ready_result.post_motion_observing);
  assert(ready_result.command.action==MissionAction::kStepForwardThree);

  // GOAL: center-depth mean prevents premature camera transition at DONE.
  auto goal_ready_cfg=ready_cfg;
  goal_ready_cfg.enable_ball=false; goal_ready_cfg.enable_goal=true;
  goal_ready_cfg.goal_trial_start_forward=true;
  goal_ready_cfg.goal.stable_window=goal_ready_cfg.goal.stable_min_hits=1;
  MissionController goal_ready(goal_ready_cfg);
  ready_frame=Frame(0.0); ready_frame.backboard_target=Ball(.5,.3);
  ready_frame.backboard_center_depth_m=1.8;
  ready_result=goal_ready.Step(ready_frame);
  assert(ready_result.command.action==MissionAction::kStepForwardFour);
  const auto goal_ready_id=ready_result.command.action_id;
  for (int i=1;i<=27;++i) {
    ready_frame=Frame(i*.1);
    ready_frame.backboard_target=Ball(.5,.3);
    ready_frame.backboard_center_depth_m=i>=12 ? 1.0 : 1.8;
    ready_frame.delivery_feedback={goal_ready_id,true,false,i==27};
    ready_result=goal_ready.Step(ready_frame);
  }
  ready_frame=Frame(3.0); ready_frame.backboard_target=Ball(.5,.3);
  ready_frame.backboard_center_depth_m=.5;
  ready_frame.delivery_feedback={goal_ready_id,true,true,false};
  ready_result=goal_ready.Step(ready_frame);
  assert(!ready_result.post_motion_observing);
  assert(ready_result.command.action==MissionAction::kStepForwardThree);
  return 0;
}
