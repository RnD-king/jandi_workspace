#include "vision_core/config_loader.hpp"
#include "vision_core/mission_controller.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
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
  in=Frame(.2); in.delivery_feedback={id,true,false,true};
  r=controller.Step(in);
  assert(r.command.action_id!=0);
  assert(r.command.action==MissionAction::kStepForwardFive);

  // ACK 전 정지 관측은 long-action 후반 accumulator에 포함하지 않는다.
  // ACK 뒤 관측은 모두 straight이므로 READY 예약도 straight여야 한다.
  controller.Reset();
  in=Frame(0); r=controller.Step(in);
  const auto delayed_ack_id=r.command.action_id;
  in=Frame(.1); ShiftLine(in, 500); r=controller.Step(in);
  in=Frame(.7); ShiftLine(in, 500); r=controller.Step(in);
  in=Frame(.8); in.delivery_feedback={delayed_ack_id,true,false,false};
  r=controller.Step(in);
  in=Frame(1.0); in.delivery_feedback={delayed_ack_id,true,false,true};
  r=controller.Step(in);
  assert(r.command.action==MissionAction::kStepForwardFive);

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
  turn_config.line_p2p.recovery_max_turns=5;
  MissionController turning(turn_config);
  in=Frame(0); in.line_decision_guide_override=Guide(-.3);
  r=turning.Step(in);
  auto action_id=r.command.action_id;
  in=Frame(.1); in.line_centers.clear();
  in.delivery_feedback={action_id,true,true,false};
  r=turning.Step(in);
  in=Frame(.2); in.line_centers.clear(); r=turning.Step(in);
  for (int turn=1; turn<=5; ++turn) {
    assert(r.command.action==MissionAction::kTurnLeft);
    assert(r.command.target_yaw_deg==15);
    action_id=r.command.action_id;
    const double done_time=.2+turn*.2-.1;
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
  in=Frame(.2); in.delivery_feedback={current_id,true,false,true};
  r=queued.Step(in);
  const auto queued_id=r.command.action_id;
  assert(queued_id!=0 && queued_id!=current_id);
  assert(r.command.control_phase==ControlPhase::kWaitingQueuedActionAck);
  in=Frame(.3); in.allow_new_line_action=false;
  in.delivery_feedback={queued_id,true,false,false};
  r=queued.Step(in);
  assert(r.command.action_id==queued_id);
  assert(r.command.control_phase==ControlPhase::kWaitingQueuedActionStart);
  in=Frame(.4); in.allow_new_line_action=false;
  in.delivery_feedback={current_id,true,true,false};
  r=queued.Step(in);
  assert(r.command.action_id==queued_id);
  assert(r.command.control_phase==ControlPhase::kWaitingActionDone);
  in=Frame(.5); in.allow_new_line_action=false;
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
  camera_config.ball.upper_acquire_v_norm=1;
  camera_config.ball.tilt_down_window=camera_config.ball.tilt_down_min_hits=1;
  MissionController ball_timeout(camera_config);
  in=Frame(0); in.ball_target=Ball(.5,.8);
  r=ball_timeout.Step(in);
  assert(r.ball.mode==BallMode::kWaitCameraDown);
  in.now_sec=.4; r=ball_timeout.Step(in);
  assert(r.ball.mode==BallMode::kWaitCameraDown);
  in.now_sec=.5; r=ball_timeout.Step(in);
  assert(r.ball.mode==BallMode::kFailed);

  camera_config.enable_ball=false; camera_config.enable_hurdle=true;
  camera_config.hurdle.stable_window=camera_config.hurdle.stable_min_hits=1;
  camera_config.hurdle.smooth_alpha=1;
  camera_config.hurdle.acquire_min_v_norm=0;
  camera_config.hurdle.tilt_trigger_window=
      camera_config.hurdle.tilt_trigger_min_hits=1;
  MissionController hurdle_timeout(camera_config);
  in=Frame(0); in.hurdle_target=Ball(.5,.8);
  r=hurdle_timeout.Step(in);
  assert(r.hurdle.mode==HurdleMode::kWaitCameraDown);
  in.now_sec=.4; r=hurdle_timeout.Step(in);
  assert(r.hurdle.mode==HurdleMode::kWaitCameraDown);
  in.now_sec=.5; r=hurdle_timeout.Step(in);
  assert(r.hurdle.mode==HurdleMode::kFailed);

  camera_config.enable_hurdle=false; camera_config.enable_goal=true;
  camera_config.initial_has_ball=true;
  camera_config.goal.post_pickup_wait_sec=0;
  camera_config.line.line_stable_window=camera_config.line.line_stable_min_hits=1;
  MissionController goal_timeout(camera_config);
  in=Frame(0); r=goal_timeout.Step(in);
  assert(r.goal.mode==GoalMode::kWaitCameraGoal);
  in.now_sec=.4; r=goal_timeout.Step(in);
  assert(r.goal.mode==GoalMode::kWaitCameraGoal);
  in.now_sec=.5; r=goal_timeout.Step(in);
  assert(r.goal.mode==GoalMode::kFailed);
  return 0;
}
