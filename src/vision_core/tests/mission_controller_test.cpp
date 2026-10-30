#include "vision_core/config_loader.hpp"
#include "vision_core/mission_controller.hpp"
#include "verified_entry_fixture.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <limits>
#include <stdexcept>
using namespace vision_core;
static MissionFrameInput Frame(double now) {
  MissionFrameInput f; f.image_width=640; f.image_height=480; f.now_sec=(now) + 3.0;
  f.command_transport_enabled=true;
  f.line_centers={{320,450},{320,400},{320,350},{320,300},{320,250}};
  return f;
}
// These regressions isolate the stationary/manual GOAL admission contract;
// after admission, normal AUTO observation/READY behavior remains under test.
static MissionFrameResult StepStationaryGoalAdmission(MissionController& core, MissionFrameInput frame) {
  if (core.MissionStateKey()/10000==static_cast<int>(MissionType::kLine))
    frame.manual_observation_control=true;
  return core.Step(frame);
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
static void TestNearBallTurnThenDown() {
  auto config=LoadDefaultAlgorithmConfig(); config.transition_settle_sec=0;
  config.enable_ball=true;
  config.enable_goal=config.enable_hurdle=false;
  config.post_motion_observation_sec=1.0;
  config.ball.entry_min_hits=1; config.ball.down_search_min_hits=1; config.ball.stable_window=config.ball.stable_min_hits=1;
  config.ball.pickup_verify_min_hits=1; // This fixture uses a one-frame window.
  config.ball.upper_acquire_v_px=480;
  config.ball.far_u_des_px=320;
  config.ball.approach_u_deadband_px=32;
  config.ball.approach_left_top_u_px=288;config.ball.approach_right_top_u_px=352;
  config.ball.approach_short_v_px=300;
  config.ball.approach_two_step_v_px=336;
  for (double u : {.3,.7}) for (bool found : {false,true}) {
    MissionController controller(config);
  AdmitObjectAtRest(controller, Ball(u,.8));
    auto input=Frame(0);
    input.ball_target=Ball(u,.8);
    auto result=controller.Step(input);
    const auto turn=u < .5 ? MissionAction::kTurnLeftInPlace
                           : MissionAction::kStepForwardRightOneHurdle;
    assert(result.command.action==turn);
    const auto first_id=result.command.action_id;
    assert(first_id!=0);
    input=Frame(.1);input.delivery_feedback={first_id,true,false,false};
    result=controller.Step(input);
    assert(result.command.camera_request==CameraRequest::kNone);
    input=Frame(.2);input.delivery_feedback={first_id,true,true,false};
    result=controller.Step(input);
    assert(result.post_motion_observing);
    for (int i=1;i<=9;++i) {
      input=Frame(.2+.1*i);
      result=controller.Step(input);
      assert(result.post_motion_observing);
      assert(result.command.camera_request==CameraRequest::kNone);
    }
    input=Frame(1.21);
    result=controller.Step(input);
    assert(!result.post_motion_observing);
    assert(result.ball.mode==BallMode::kNearTurnBeforeDown);
    assert(result.command.action==turn);
    const auto retry_id=result.command.action_id;
    assert(retry_id!=0 && retry_id!=first_id);
    assert(result.command.camera_request==CameraRequest::kNone);
    input=Frame(1.3);input.delivery_feedback={retry_id,true,false,false};
    result=controller.Step(input);
    assert(result.command.camera_request==CameraRequest::kNone);
    input=Frame(1.4);input.delivery_feedback={retry_id,true,true,false};
    result=controller.Step(input);
    // Retry DONE also starts the existing stationary observation window.
    assert(result.post_motion_observing);
    assert(result.command.camera_request==CameraRequest::kNone);
    for (int i=1;i<=9;++i) {
      input=Frame(1.4+.1*i);
      if (found) input.ball_target=Ball(.5,.8);
      result=controller.Step(input);
      assert(result.post_motion_observing);
      assert(result.command.camera_request==CameraRequest::kNone);
    }
    input=Frame(2.41);
    if (found) input.ball_target=Ball(.5,.8);
    result=controller.Step(input);
    assert(!result.post_motion_observing);
    assert(result.ball.mode==(found ? BallMode::kApproachBall : BallMode::kWaitCameraDown));
    assert(result.command.camera_request==(found ? CameraRequest::kNone : CameraRequest::kDown));
    assert(result.command.action==MissionAction::kNone);
    input=Frame(2.5);
    if (found) input.ball_target=Ball(.5,.8);
    else input.camera_feedback={CameraMode::kDown,true};
    result=controller.Step(input);
    assert(result.ball.mode==(found ? BallMode::kApproachTwoBeforeDown : BallMode::kDownSearch));
    if (found) assert(result.command.action==MissionAction::kStepForwardTwo);
  }
}
int main() {
  TestNearBallTurnThenDown();
  auto c=LoadDefaultAlgorithmConfig(); c.transition_settle_sec=0;
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
  for (int i=1;i<=9;++i) {
    two_point_input=Frame(.2*i);
    two_point_result=two_point_controller.Step(two_point_input);
    assert(two_point_result.command.action_id==0);
  }
  two_point_input=Frame(2.0);
  two_point_result=two_point_controller.Step(two_point_input);
  assert(!two_point_result.line_in_recovery);
  assert(two_point_result.command.action==MissionAction::kLineStraightUnit);
  MissionController controller(c);
  auto in=Frame(0); auto r=controller.Step(in);
  assert(r.command.action==MissionAction::kLineStraightUnit);
  assert(r.command.action_id!=0);
  const auto id=r.command.action_id;
  in=Frame(.1); in.delivery_feedback={id,true,false,false};
  r=controller.Step(in); assert(r.command.command_type==CommandType::kHold);
  // A single newly-invalid three-point frame during execution must not
  // bypass the existing READY average or begin an immediate recovery.
  in=Frame(.2); in.line_centers.resize(3);
  r=controller.Step(in);
  assert(!r.line_features.guide.valid && !r.line_in_recovery);
  assert(r.command.command_type==CommandType::kHold);
  for (double time : {0.5, 0.8, 1.4, 1.6, 1.8, 2.0, 2.2}) {
    in=Frame(time); r=controller.Step(in);
  }
  in=Frame(2.55); in.delivery_feedback={id,true,false,true};
  r=controller.Step(in);
  assert(r.command.action_id!=0 && r.command.action_id!=id);
  assert(r.command.action==MissionAction::kLineStraightUnit);
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
  for (double time : {1.0, 1.3, 1.5, 1.7, 1.8, 2.0, 2.1, 2.3}) {
    in=Frame(time); r=controller.Step(in);
  }
  in=Frame(2.55); in.delivery_feedback={delayed_ack_id,true,false,true};
  r=controller.Step(in);
  assert(r.command.action==MissionAction::kLineStraightUnit);
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

  // 마지막 유효 점수의 부호를 기억하고, 실패 관측 뒤 WALK 자세로 회전한다.
  controller.Reset();
  in=Frame(0); in.line_decision_guide_override=Guide(-.2);
  r=controller.Step(in);
  assert(r.command.action==MissionAction::kLineLeftUnit);
  const auto left_id=r.command.action_id;
  in=Frame(.1); in.line_centers.clear(); in.line_decision_guide_override.reset();
  in.delivery_feedback={left_id,true,true,false};
  r=controller.Step(in); assert(r.command.command_type==CommandType::kHold);
  for(int i=1;i<=10;++i) {
    in=Frame(.1+.1*i);in.line_centers.clear();r=controller.Step(in);
  }
  in=Frame(2.2); in.line_centers.clear();
  r=controller.Step(in);
  assert(r.command.action==MissionAction::kTurnLeftInPlace);
  assert(r.foot_pose==FootPose::kWalk);

  // 방향 근거가 없으면 회전하지 않고 다섯 정지 관측 후 최종 정지한다.
  controller.Reset();
  in=Frame(0); in.line_centers.clear(); r=controller.Step(in);
  for (int i=1;i<=200 && r.command.mission_phase!=4;++i) {
    in=Frame(.1*i); in.line_centers.clear();
    r=controller.Step(in);
    assert(r.command.action==MissionAction::kNone);
  }
  assert(r.command.mission_phase==4);
  in=Frame(21.0); in.line_decision_guide_override=Guide(0);
  r=controller.Step(in);
  assert(r.command.action==MissionAction::kNone);
  assert(r.command.mission_phase==4);

  // 중앙 점수 0은 방향 근거가 아니므로 임의의 우회전을 하지 않는다.
  controller.Reset();
  in=Frame(0); in.line_decision_guide_override=Guide(0);
  r=controller.Step(in);
  assert(r.command.action==MissionAction::kLineStraightUnit);
  const auto straight_id=r.command.action_id;
  in=Frame(.1); in.line_centers.clear();
  in.delivery_feedback={straight_id,true,true,false};
  r=controller.Step(in);
  for(int i=1;i<=100;++i) {
    in=Frame(.1+i*.1); in.line_centers.clear(); r=controller.Step(in);
    assert(r.command.action==MissionAction::kNone);
  }
  assert(r.command.mission_phase==4);

  // FINAL HOLD에서는 유효한 object detection도 새 mission을 시작하지 않는다.
  auto terminal_config=c;
  terminal_config.line_p2p.failure_observation_sec=0.0;
  terminal_config.line_p2p.last_score_recovery_max_turns=0;
  terminal_config.enable_ball=true;
  terminal_config.enable_hurdle=false;
  terminal_config.ball.entry_min_hits=1; terminal_config.ball.down_search_min_hits=1; terminal_config.ball.stable_window=1;
  terminal_config.ball.pickup_verify_min_hits=1; // This fixture uses a one-frame window.
  terminal_config.ball.stable_min_hits=1;
  MissionController terminal(terminal_config);
  in=Frame(0); in.line_centers.clear(); r=terminal.Step(in);
  for(int i=1;i<=50;++i) {
    in=Frame(.1*i); in.line_centers.clear(); r=terminal.Step(in);
  }
  assert(r.command.mission_phase==4);
  in=Frame(5.1); in.line_centers.clear(); in.ball_target=Ball(.5,.3);
  r=terminal.Step(in);
  assert(r.active_mission==MissionType::kLine);
  assert(r.command.action==MissionAction::kNone);
  assert(r.command.mission_phase==4);

  // 방향성 복구는 설정된 5회만 WALK 회전하고 그 뒤 terminal이다.
  auto turn_config=c;
  turn_config.line_p2p.failure_observation_sec=0.0;
  turn_config.line_p2p.recovery_observation_sec=0.0; // Frame count still requires a full 10-frame window.
  turn_config.line_p2p.last_score_recovery_max_turns=5;
  MissionController turning(turn_config);
  in=Frame(0); in.line_decision_guide_override=Guide(-.3);
  r=turning.Step(in);
  auto action_id=r.command.action_id;
  in=Frame(.1); in.line_centers.clear();
  in.delivery_feedback={action_id,true,true,false};
  r=turning.Step(in);
  double turn_time=.1;
  for (int turn=1; turn<=5; ++turn) {
    for(int sample=0;sample<10;++sample) {
      turn_time+=.1;
      in=Frame(turn_time);in.line_centers.clear();r=turning.Step(in);
    }
    assert(r.command.action==MissionAction::kTurnLeftInPlace);
    action_id=r.command.action_id;
    turn_time+=.1;
    in=Frame(turn_time); in.line_centers.clear();
    in.delivery_feedback={action_id,true,true,false};
    r=turning.Step(in);
  }
  for(int sample=0;sample<10;++sample) {
    turn_time+=.1;in=Frame(turn_time);in.line_centers.clear();r=turning.Step(in);
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
  // ACK부터 READY까지의 창에 10프레임 이상, 그중 5개 이상 유효해야 한다.
  for (double time : {.2, .4, .6, .8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.2}) {
    in=Frame(time); r=queued.Step(in);
  }
  in=Frame(2.55); in.delivery_feedback={current_id,true,false,true};
  r=queued.Step(in);
  const auto queued_id=r.command.action_id;
  assert(queued_id!=0 && queued_id!=current_id);
  assert(r.command.control_phase==ControlPhase::kWaitingQueuedActionAck);
  assert(r.line_window_stats && r.line_window_stats->window_frames >= 10);
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
  assert(r.command.action==MissionAction::kLineStraightUnit);

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
  assert(r.command.action==MissionAction::kLineLeftUnit);
  const auto frozen_id=r.command.action_id;
  in=Frame(32); in.line_centers.clear();
  in.advance_line_fsm=false; in.allow_new_line_action=false;
  in.delivery_feedback={frozen_id,true,false,false};
  r=frozen.Step(in);
  assert(r.command.action_id==frozen_id);
  assert(r.command.control_phase==ControlPhase::kWaitingActionDone);
  in.now_sec=(33) + 3.0; in.delivery_feedback={frozen_id,true,false,true};
  r=frozen.Step(in);
  assert(r.command.action_id==frozen_id); // No READY successor.
  assert(r.command.control_phase==ControlPhase::kWaitingActionDone);
  for (int i=34; i<45; ++i) {
    in.now_sec=(i) + 3.0; in.delivery_feedback={};
    r=frozen.Step(in);
    assert(!r.line_in_recovery && r.command.action_id==frozen_id);
  }
  in.now_sec=(45) + 3.0; in.delivery_feedback={frozen_id,true,true,false};
  r=frozen.Step(in);
  assert(r.command.action_id==0 && !r.line_in_recovery);
  assert(frozen.UpdateLineP2pTuning(c.line_p2p));
  in=Frame(46); in.line_decision_guide_override=Guide(.3);
  r=frozen.Step(in);
  assert(r.command.action==MissionAction::kLineRightUnit);
  assert(r.command.action_id>frozen_id); // No Reset between trials.

  // PerceptionFrameInput must forward the same freeze contract.
  MissionController frozen_perception(c);
  PerceptionFrameInput observation;
  observation.image_width=640; observation.image_height=480;
  observation.command_transport_enabled=true;
  observation.advance_line_fsm=false;
  // Even with allow_new_line_action=true, a frozen FSM makes no decisions.
  for (int i=0; i<20; ++i) {
    observation.now_sec=(i) + 3.0;
    const auto p=frozen_perception.StepPerception(observation);
    assert(p.mission.line_computed && !p.mission.line_in_recovery);
    assert(p.mission.command.action_id==0);
  }
  observation.now_sec=(20) + 3.0; observation.advance_line_fsm=true;
  observation.line_decision_guide_override=Guide(0);
  const auto p=frozen_perception.StepPerception(observation);
  assert(p.mission.command.action==MissionAction::kLineStraightUnit);

  // One common timeout is injected into all three object controllers. Use a
  // non-default value so an accidental per-mission/default 3s is detectable.
  auto camera_config=c;
  camera_config.camera_motion_timeout_sec=.5;
  camera_config.enable_ball=true;
  camera_config.ball.entry_min_hits=1; camera_config.ball.down_search_min_hits=1; camera_config.ball.stable_window=camera_config.ball.stable_min_hits=1;
  camera_config.ball.pickup_verify_min_hits=1; // This fixture uses a one-frame window.

  camera_config.ball.upper_acquire_v_px=480.0;

  MissionController ball_timeout(camera_config);
  AdmitObjectAtRest(ball_timeout, Ball(.5,.8));
  in=Frame(0); in.ball_target=Ball(.5,.8);
  r=ball_timeout.Step(in);
  assert(r.ball.mode==BallMode::kApproachTwoBeforeDown);
  const auto ball_two_id=r.command.action_id;
  in.now_sec=(.01) + 3.0; in.delivery_feedback={ball_two_id,true,true,false};
  r=ball_timeout.Step(in); in.delivery_feedback={};
  assert(r.ball.mode==BallMode::kWaitCameraDown);
  in.now_sec=(.4) + 3.0; r=ball_timeout.Step(in);
  assert(r.ball.mode==BallMode::kWaitCameraDown);
  in.now_sec=(.51) + 3.0; r=ball_timeout.Step(in);
  assert(r.ball.mode==BallMode::kFailed);

  camera_config.enable_ball=false; camera_config.enable_hurdle=true;
  camera_config.hurdle.down_search_min_hits=1; camera_config.hurdle.stable_window=camera_config.hurdle.stable_min_hits=1;

  camera_config.hurdle.upper_acquire_v_px=480;

  MissionController hurdle_timeout(camera_config);
  AdmitObjectAtRest(hurdle_timeout, Ball(.5,.8), MissionType::kHurdle);
  in=Frame(0); in.hurdle_target=Ball(.5,.8);
  r=hurdle_timeout.Step(in);
  assert(r.hurdle.mode==HurdleMode::kApproachTwoBeforeDown);
  const auto hurdle_two_id=r.command.action_id;
  in.now_sec=(.01) + 3.0; in.delivery_feedback={hurdle_two_id,true,true,false};
  r=hurdle_timeout.Step(in); in.delivery_feedback={};
  assert(r.hurdle.mode==HurdleMode::kWaitCameraDown);
  in.now_sec=(.4) + 3.0; r=hurdle_timeout.Step(in);
  assert(r.hurdle.mode==HurdleMode::kWaitCameraDown);
  in.now_sec=(.51) + 3.0; r=hurdle_timeout.Step(in);
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
  camera_config.goal.post_pickup_walk_fallback_actions=0;
  camera_config.line.line_stable_window=camera_config.line.line_stable_min_hits=1;
  camera_config.goal.stable_window=camera_config.goal.stable_min_hits=1;
  MissionController goal_timeout(camera_config);
  in=Frame(0); in.backboard_target=Ball(.5,.3); in.backboard_center_depth_m=.7;
  r=StepStationaryGoalAdmission(goal_timeout,in);
  assert(r.goal.mode==GoalMode::kWaitCameraGoal);
  in.now_sec=(.4) + 3.0; r=StepStationaryGoalAdmission(goal_timeout,in);
  assert(r.goal.mode==GoalMode::kWaitCameraGoal);
  in.now_sec=(.5) + 3.0; r=StepStationaryGoalAdmission(goal_timeout,in);
  assert(r.goal.mode==GoalMode::kFailed);
  // GOAL camera DONE starts a fresh 1-second observation. Its final mean
  // must also satisfy recent 10/7 object evidence without extending 1 s.
  auto camera_gate_cfg = c;
  camera_gate_cfg.enable_ball = camera_gate_cfg.enable_hurdle = false;
  camera_gate_cfg.enable_goal = true;
  camera_gate_cfg.initial_has_ball = true;
  camera_gate_cfg.goal.post_pickup_walk_fallback_actions = 0;
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
    r = StepStationaryGoalAdmission(camera_gate,in);
  }
  assert(r.active_mission == MissionType::kGoal);
  assert(r.goal.mode == GoalMode::kWaitCameraGoal);
  in = Frame(1.0);
  in.camera_feedback = {CameraMode::kTransition, false};
  r = StepStationaryGoalAdmission(camera_gate,in);
  assert(r.command.action_id == 0);
  in = Frame(1.1);
  in.camera_feedback = {CameraMode::kGoal, true};
  r = StepStationaryGoalAdmission(camera_gate,in);
  assert(r.goal.mode == GoalMode::kSearch);
  assert(r.command.action_id == 0);
  assert(r.post_motion_observing);
  assert(r.post_motion_frames == 0 && r.post_motion_window_frames == 0);
  for (int i = 0; i < 9; ++i) {
    in = Frame(1.2 + 0.1 * i);
    in.camera_feedback = {CameraMode::kGoal, true};
    if (i < 7) in.backboard_target = Ball(.8, .3);
    r = StepStationaryGoalAdmission(camera_gate,in);
    assert(r.command.action_id == 0);
    assert(r.command.action == MissionAction::kNone);
    assert(r.post_motion_observing);
    assert(r.post_motion_frames == static_cast<std::size_t>(i+1));
  }
  in = Frame(2.1);
  in.camera_feedback = {CameraMode::kGoal, true};
  in.backboard_target = Ball(.8, .3);
  r = StepStationaryGoalAdmission(camera_gate,in);
  assert(r.active_mission == MissionType::kGoal);
  assert(r.command.action == MissionAction::kStepForwardRightOne);
  assert(r.command.action_id != 0);
  // The next OBJECT action must use 10 NEW frames after this actual Action DONE.
  // Its camera remained GOAL throughout, so only the Action event resets evidence.
  const std::uint64_t approach_id = r.command.action_id;
  in = Frame(2.15);
  in.camera_feedback = {CameraMode::kGoal, true};
  in.backboard_target = Ball(.8, .3);
  in.delivery_feedback = {approach_id, true, false, false};
  r = StepStationaryGoalAdmission(camera_gate,in);
  assert(r.command.command_type == CommandType::kHold);
  in = Frame(2.2);
  in.camera_feedback = {CameraMode::kGoal, true};
  in.backboard_target = Ball(.8, .3);
  in.delivery_feedback = {approach_id, true, true, false};
  r = StepStationaryGoalAdmission(camera_gate,in);
  assert(r.post_motion_observing);
  for (int i = 1; i <= 9; ++i) {
    in = Frame(2.2 + 0.1 * i);
    in.camera_feedback = {CameraMode::kGoal, true};
    in.backboard_target = Ball(.8, .3);
    r = StepStationaryGoalAdmission(camera_gate,in);
    assert(r.command.command_type == CommandType::kHold);
    assert(r.post_motion_observing);
  }
  assert(r.post_motion_window_frames == 10);
  assert(r.post_motion_valid == 10);
  in = Frame(3.2);
  in.camera_feedback = {CameraMode::kGoal, true};
  in.backboard_target = Ball(.8, .3);
  r = StepStationaryGoalAdmission(camera_gate,in);
  assert(!r.post_motion_observing);
  assert(r.command.command_type == CommandType::kAction);
  assert(r.command.action_id != 0 && r.command.action_id != approach_id);

  // Manual NEXT: a 10/7-acquired BALL may switch camera automatically,
  // but previewing PICK must not increment attempts or change the real FSM.
  auto manual_config=c;
  manual_config.enable_ball=true;
  manual_config.ball.entry_min_hits=1; manual_config.ball.down_search_min_hits=1; manual_config.ball.stable_window=manual_config.ball.stable_min_hits=1;
  manual_config.ball.pickup_verify_min_hits=1; // This fixture uses a one-frame window.


  manual_config.ball.upper_acquire_v_px=480.0;
  manual_config.ball.pickup_u1_px=281.6; manual_config.ball.pickup_u2_px=358.4;
  manual_config.ball.pickup_v1_px=307.2; manual_config.ball.pickup_v2_px=364.8;
  manual_config.post_motion_observation_sec=1;
  MissionController manual(manual_config);
  AdmitObjectAtRest(manual, Ball(.5,.30));
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
  mi.now_sec=(.11) + 3.0; mi.manual_observation_control=true; mi.allow_new_body_actions=false;
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
  assert(mr.ball.mode==BallMode::kWaitFinePose);
  mi.now_sec=(.31) + 3.0; mi.manual_observation_control=true; mi.allow_new_body_actions=true;
  mr=manual.Step(mi);
  assert(mr.command.action==MissionAction::kDefaultPoseMode);
  mi.now_sec=(.32) + 3.0; mi.manual_observation_control=true; mi.allow_new_body_actions=false;
  mi.delivery_feedback={mr.command.action_id,true,true,false};
  mr=manual.Step(mi);
  mi.now_sec=(.33) + 3.0; mi.delivery_feedback={}; mr=manual.Step(mi);
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
  assert(ephemeral.mission.command.action==MissionAction::kLineStraightUnit);
  auto ref_adjust=c;
  ref_adjust.line_features.image_center_u=250.0;
  assert(line_manual.UpdateTuning(ref_adjust));
  const auto shifted=line_manual.PreviewPerception(line_frame);
  assert(shifted.mission.line_decision_guide);
  assert(shifted.mission.line_decision_guide->offset >
         ephemeral.mission.line_decision_guide->offset + 0.10);
  assert(std::abs(shifted.mission.line_decision_guide->offset -
                  shifted.mission.line_features.guide.offset) < 1e-9);
  assert(shifted.mission.command.action==MissionAction::kLineRightUnit);
  const auto issued=line_manual.StepPerception(line_frame);
  assert(issued.mission.command.action_id==1);
  assert(issued.mission.command.action==MissionAction::kLineRightUnit);

  // A 1s post-camera gate must RELEASE even with fewer than seven
  // detections. The object recovery/search FSM, not the observation gate,
  // decides what to do with missing observations.
  MissionController sparse_gate(camera_gate_cfg);
  for (int i=0;i<7;++i) {
    in=Frame(.1*i);
    in.allow_new_line_action=false;
    r=StepStationaryGoalAdmission(sparse_gate,in);
  }
  in=Frame(1.0); in.camera_feedback={CameraMode::kTransition,false};
  r=StepStationaryGoalAdmission(sparse_gate,in);
  in=Frame(1.1); in.camera_feedback={CameraMode::kGoal,true};
  r=StepStationaryGoalAdmission(sparse_gate,in);
  for (int i=0;i<9;++i) {
    in=Frame(1.2+.1*i);
    in.camera_feedback={CameraMode::kGoal,true};
    // Only one valid frame: previously this would block indefinitely.
    if (i==7) in.backboard_target=Ball(.8,.3);
    r=StepStationaryGoalAdmission(sparse_gate,in);
    assert(r.post_motion_observing);
  }
  in=Frame(2.1); in.camera_feedback={CameraMode::kGoal,true};
  r=StepStationaryGoalAdmission(sparse_gate,in);
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
  manual_input.now_sec = (0.7) + 3.0;
  manual_result = explicitly_previewed.Step(manual_input);
  assert(manual_result.post_motion_observing);
  for(int i=1;i<=8;++i) {
    manual_input.now_sec=(.7+.05*i)+3.0;
    manual_result=explicitly_previewed.Step(manual_input);
  }
  manual_input.now_sec = (1.2) + 3.0;
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

  // BALL: 3-step 40%-READY mean should select 2-step at DONE even if
  // the raw DONE frame would incorrectly select three-step.
  auto ready_cfg=LoadDefaultAlgorithmConfig(); ready_cfg.transition_settle_sec=0;
  ready_cfg.object_ready_mean_actions={22,23};
  ready_cfg.enable_ball=true;
  ready_cfg.enable_goal=ready_cfg.enable_hurdle=false;
  ready_cfg.ball.entry_min_hits=1; ready_cfg.ball.down_search_min_hits=1; ready_cfg.ball.stable_window=ready_cfg.ball.stable_min_hits=1;
  ready_cfg.ball.pickup_verify_min_hits=1; // This fixture uses a one-frame window.
  ready_cfg.ball.upper_acquire_v_px=480;
  MissionController ball_ready(ready_cfg);
  AdmitObjectAtRest(ball_ready, Ball(.5,.2));
  auto ready_frame=Frame(0.0);
  ready_frame.ball_target=Ball(.5,.2);
  auto ready_result=ball_ready.Step(ready_frame);
  assert(ready_result.command.action==MissionAction::kStepForwardThree);
  const auto ball_ready_id=ready_result.command.action_id;
  for (int i=1;i<=27;++i) {
    ready_frame=Frame(i*.1);
    ready_frame.ball_target=Ball(.5,i>=12 ? .8 : .2);
    ready_frame.delivery_feedback={ball_ready_id,true,false,i==27};
    ready_result=ball_ready.Step(ready_frame);
    if (i<27) assert(ready_result.command.action_id==ball_ready_id);
  }
  // A=22, B=21: READY must reserve B, not wait for A's DONE.
  const auto ball_queued_id=ready_result.command.action_id;
  assert(ball_queued_id!=0 && ball_queued_id!=ball_ready_id);
  assert(!ready_result.object_ready_mean_used);
  assert(ready_result.ball.mode==BallMode::kApproachBall);
  assert(ready_result.command.action==MissionAction::kStepForwardTwo);
  assert(ready_result.command.control_phase==ControlPhase::kWaitingQueuedActionAck);
  ready_frame=Frame(2.8); ready_frame.ball_target=Ball(.5,.2);
  ready_frame.delivery_feedback={ball_queued_id,true,false,false};
  ready_result=ball_ready.Step(ready_frame);
  assert(ready_result.command.action_id==ball_queued_id);
  assert(ready_result.command.control_phase==ControlPhase::kWaitingQueuedActionStart);
  ready_frame=Frame(3.0); ready_frame.ball_target=Ball(.5,.2);
  ready_frame.delivery_feedback={ball_ready_id,true,true,false};
  ready_result=ball_ready.Step(ready_frame);
  assert(!ready_result.post_motion_observing);
  assert(ready_result.object_ready_mean_used);
  assert(ready_result.ball.mode==BallMode::kApproachTwoBeforeDown);
  assert(ready_result.command.action==MissionAction::kStepForwardTwo);
  assert(ready_result.command.action_id==ball_queued_id);
  assert(ready_result.command.control_phase==ControlPhase::kWaitingActionDone);
  // Duplicate completion cannot invent a third action.
  ready_frame=Frame(3.1); ready_frame.ball_target=Ball(.5,.2);
  ready_frame.delivery_feedback={ball_ready_id,true,true,false};
  ready_result=ball_ready.Step(ready_frame);
  assert(ready_result.command.action_id==ball_queued_id);

  // READY may reserve with 7/10 even when trailing misses leave the latest
  // detection more than 0.3 s old. The existing hit gate remains mandatory.
  auto missing_ready_cfg=ready_cfg;
  missing_ready_cfg.ball.stable_window=10;missing_ready_cfg.ball.stable_min_hits=7;
  MissionController missing_ready(missing_ready_cfg);
  AdmitObjectAtRest(missing_ready,Ball(.5,.2));
  ready_frame=Frame(0);ready_frame.ball_target=Ball(.5,.2);
  ready_result=missing_ready.Step(ready_frame);
  assert(ready_result.command.action==MissionAction::kStepForwardThree);
  const auto missing_ready_id=ready_result.command.action_id;
  for(int i=1;i<=27;++i) {
    ready_frame=Frame(i==27?2.85:i*.1);
    if(i<25)ready_frame.ball_target=Ball(.5,i>=12?.8:.2);
    ready_frame.delivery_feedback={missing_ready_id,true,false,i==27};
    ready_result=missing_ready.Step(ready_frame);
  }
  const auto missing_queued_id=ready_result.command.action_id;
  assert(missing_queued_id!=0 && missing_queued_id!=missing_ready_id);
  assert(ready_result.command.action==MissionAction::kStepForwardTwo);
  assert(ready_result.command.control_phase==ControlPhase::kWaitingQueuedActionAck);
  ready_frame=Frame(3);ready_frame.delivery_feedback={missing_ready_id,true,true,false};
  ready_result=missing_ready.Step(ready_frame);
  assert(ready_result.object_ready_mean_used && !ready_result.post_motion_observing);
  assert(ready_result.command.action_id==missing_queued_id);

  // Feedback callbacks can accept A DONE and B DONE before the next image.
  // A's late sensory DONE must NOT create a second B with a fresh ID.
  MissionController early_done(ready_cfg);
  AdmitObjectAtRest(early_done, Ball(.5,.2));
  ready_frame=Frame(0.0); ready_frame.ball_target=Ball(.5,.2);
  ready_result=early_done.Step(ready_frame);
  const auto early_a=ready_result.command.action_id;
  for (int i=1;i<=27;++i) {
    ready_frame=Frame(i*.1);
    ready_frame.ball_target=Ball(.5,i>=12 ? .8 : .2);
    ready_frame.delivery_feedback={early_a,true,false,i==27};
    ready_result=early_done.Step(ready_frame);
  }
  const auto early_b=ready_result.command.action_id;
  assert(early_b!=early_a && early_b!=0);
  early_done.AcceptDeliveryFeedback({early_b,true,false,false},2.8+3.0);
  early_done.AcceptDeliveryFeedback({early_a,true,true,false},2.9+3.0);
  early_done.AcceptDeliveryFeedback({early_b,true,true,false},2.95+3.0);
  ready_frame=Frame(3.0); ready_frame.ball_target=Ball(.5,.2);
  ready_frame.delivery_feedback={early_a,true,true,false};
  ready_result=early_done.Step(ready_frame);
  assert(ready_result.object_ready_mean_used);
  assert(ready_result.command.action_id==0);
  ready_frame=Frame(3.1);
  ready_frame.delivery_feedback={early_b,true,true,false};
  ready_result=early_done.Step(ready_frame);
  assert(ready_result.command.action_id==0);
  assert(ready_result.command.camera_request==CameraRequest::kDown);

  // A=22 can reserve B=25. B's number never restricts READY queue admission.
  MissionController turn_successor(ready_cfg);
  AdmitObjectAtRest(turn_successor, Ball(.5,.2));
  ready_frame=Frame(0.0); ready_frame.ball_target=Ball(.5,.2);
  ready_result=turn_successor.Step(ready_frame);
  assert(ready_result.command.action==MissionAction::kStepForwardThree);
  const auto turn_source_id=ready_result.command.action_id;
  for (int i=1;i<=27;++i) {
    ready_frame=Frame(i*.1);
    ready_frame.ball_target=Ball(i>=12 ? .95 : .5,.2);
    ready_frame.delivery_feedback={turn_source_id,true,false,i==27};
    ready_result=turn_successor.Step(ready_frame);
  }
  assert(ready_result.command.command_type==CommandType::kAction);
  assert(ready_result.command.control_phase==ControlPhase::kWaitingQueuedActionAck);
  assert(ready_result.command.action==MissionAction::kStepForwardRightTwo);
  const auto turn_successor_id=ready_result.command.action_id;
  ready_frame=Frame(3.0); ready_frame.ball_target=Ball(.5,.2);
  ready_frame.delivery_feedback={turn_source_id,true,true,false};
  ready_result=turn_successor.Step(ready_frame);
  assert(ready_result.object_ready_mean_used);
  assert(ready_result.command.action_id==turn_successor_id);
  assert(ready_result.command.action==MissionAction::kStepForwardRightTwo);
  // B=25 gets READY from the executor, but no B->C reservation: 25 is not
  // in the object's YAML current-action allowlist.
  for (int i=31;i<=40;++i) {
    ready_frame=Frame(i*.1);
    ready_frame.ball_target=Ball(.5,.2);
    ready_frame.delivery_feedback={turn_successor_id,true,false,i==40};
    ready_result=turn_successor.Step(ready_frame);
    assert(ready_result.command.action_id==turn_successor_id);
    assert(ready_result.command.control_phase!=ControlPhase::kWaitingQueuedActionAck);
  }

  // Directional object approaches (24/25) change the camera heading
  // during the motion. Even a fully valid READY 10/7 window must be
  // DISCARDED: wait for fresh post-DONE observations in final coordinates.
  for (double lateral_u : {.05, .95}) {
    MissionController directional(ready_cfg);
  AdmitObjectAtRest(directional, Ball(lateral_u,.2));
    ready_frame=Frame(0.0);
    ready_frame.ball_target=Ball(lateral_u,.2);
    ready_result=directional.Step(ready_frame);
    assert(ready_result.command.action==
        (lateral_u<.5 ? MissionAction::kStepForwardLeftTwo
                       : MissionAction::kStepForwardRightTwo));
    const auto lateral_id=ready_result.command.action_id;
    for (int i=1;i<=27;++i) {
      ready_frame=Frame(i*.1);
      ready_frame.ball_target=Ball(i>=12 ? .5 : lateral_u,i>=12 ? .8 : .2);
      ready_frame.delivery_feedback={lateral_id,true,false,i==27};
      ready_result=directional.Step(ready_frame);
      assert(!ready_result.object_ready_mean_used);
    }
    ready_frame=Frame(3.0);
    ready_frame.ball_target=Ball(lateral_u,.2);
    ready_frame.delivery_feedback={lateral_id,true,true,false};
    ready_result=directional.Step(ready_frame);
    assert(!ready_result.object_ready_mean_used);
    assert(ready_result.post_motion_observing);
    assert(ready_result.command.action_id==0 ||
           ready_result.command.action_id==lateral_id);
    for (int i=1;i<=9;++i) {
      ready_frame=Frame(3.0+i*.1);
      ready_frame.ball_target=Ball(.5,.8);
      ready_result=directional.Step(ready_frame);
      assert(!ready_result.object_ready_mean_used);
      assert(ready_result.post_motion_observing);
    }
    ready_frame=Frame(4.0);
    ready_frame.ball_target=Ball(.5,.8);
    ready_result=directional.Step(ready_frame);
    assert(!ready_result.object_ready_mean_used);
    assert(!ready_result.post_motion_observing);
    assert(ready_result.command.action==MissionAction::kStepForwardTwo);
    assert(ready_result.command.action_id!=lateral_id);
  }

  // Changing only the allowlist enables the exact same READY path for
  // directional 24/25, without modifying controller source.
  auto turn_enabled_cfg = ready_cfg;
  turn_enabled_cfg.object_ready_mean_actions = {22, 23, 24, 25};
  ValidateAlgorithmConfig(turn_enabled_cfg);
  for (double lateral_u : {.05, .95}) {
    MissionController turn_enabled(turn_enabled_cfg);
  AdmitObjectAtRest(turn_enabled, Ball(lateral_u,.2));
    ready_frame=Frame(0.0); ready_frame.ball_target=Ball(lateral_u,.2);
    ready_result=turn_enabled.Step(ready_frame);
    assert(ready_result.command.action ==
        (lateral_u<.5 ? MissionAction::kStepForwardLeftTwo
                       : MissionAction::kStepForwardRightTwo));
    const auto turn_id=ready_result.command.action_id;
    for (int i=1;i<=27;++i) {
      ready_frame=Frame(i*.1);
      ready_frame.ball_target=Ball(i>=12 ? .5 : lateral_u,i>=12 ? .8 : .2);
      ready_frame.delivery_feedback={turn_id,true,false,i==27};
      ready_result=turn_enabled.Step(ready_frame);
    }
    ready_frame=Frame(3.0);
    ready_frame.ball_target=Ball(lateral_u,.2);
    ready_frame.delivery_feedback={turn_id,true,true,false};
    ready_result=turn_enabled.Step(ready_frame);
    assert(ready_result.object_ready_mean_used);
    assert(!ready_result.post_motion_observing);
    assert(ready_result.command.action==MissionAction::kStepForwardTwo);
  }

  // Empty allowlist disables the object READY mean even for straight 22/23.
  auto disabled_ready_cfg=ready_cfg;
  disabled_ready_cfg.object_ready_mean_actions.clear();
  ValidateAlgorithmConfig(disabled_ready_cfg);
  MissionController disabled_ready(disabled_ready_cfg);
  AdmitObjectAtRest(disabled_ready, Ball(.5,.2));
  ready_frame=Frame(0.0); ready_frame.ball_target=Ball(.5,.2);
  ready_result=disabled_ready.Step(ready_frame);
  assert(ready_result.command.action==MissionAction::kStepForwardThree);
  const auto disabled_id=ready_result.command.action_id;
  for (int i=1;i<=27;++i) {
    ready_frame=Frame(i*.1); ready_frame.ball_target=Ball(.5,.8);
    ready_frame.delivery_feedback={disabled_id,true,false,i==27};
    ready_result=disabled_ready.Step(ready_frame);
  }
  ready_frame=Frame(3.0); ready_frame.ball_target=Ball(.5,.8);
  ready_frame.delivery_feedback={disabled_id,true,true,false};
  ready_result=disabled_ready.Step(ready_frame);
  assert(!ready_result.object_ready_mean_used);
  assert(ready_result.post_motion_observing);

  // Disabling object READY never disables LINE's own READY policy (23).
  auto line_ready_cfg=disabled_ready_cfg;
  line_ready_cfg.enable_ball=false;
  line_ready_cfg.line_features.image_center_u=320;
  MissionController line_ready(line_ready_cfg);
  ready_result=line_ready.Step(Frame(0.0));
  assert(ready_result.command.mission==MissionType::kLine);
  assert(ready_result.command.action==MissionAction::kLineStraightUnit);
  const auto line_ready_id=ready_result.command.action_id;
  for (int i=1;i<=27;++i) {
    ready_frame=Frame(i*.1);
    ready_frame.delivery_feedback={line_ready_id,true,false,i==27};
    ready_result=line_ready.Step(ready_frame);
  }
  assert(ready_result.command.mission==MissionType::kLine);
  assert(ready_result.command.action==MissionAction::kLineStraightUnit);
  assert(ready_result.command.action_id!=0 && ready_result.command.action_id!=line_ready_id);
  assert(ready_result.command.control_phase==ControlPhase::kWaitingQueuedActionAck);

  // Valid READY evidence is insufficient when PAUSE prevents reserving B.
  // Test the global command gate and the separate body-action gate.
  for (bool pause_commands : {false,true}) {
    MissionController unreserved(ready_cfg);
  AdmitObjectAtRest(unreserved, Ball(.5,.2));
    ready_frame=Frame(0.0); ready_frame.ball_target=Ball(.5,.2);
    ready_result=unreserved.Step(ready_frame);
    const auto unreserved_id=ready_result.command.action_id;
    assert(unreserved_id!=0);
    for (int i=1;i<=27;++i) {
      ready_frame=Frame(i*.1);
      ready_frame.ball_target=Ball(.5,i>=12 ? .8 : .2);
      ready_frame.delivery_feedback={unreserved_id,true,false,i==27};
      if (i==27) {
        ready_frame.allow_new_commands=!pause_commands;
        ready_frame.allow_new_body_actions=pause_commands;
      }
      ready_result=unreserved.Step(ready_frame);
    }
    assert(ready_result.command.action_id==unreserved_id);
    assert(ready_result.command.control_phase!=ControlPhase::kWaitingQueuedActionAck);
    ready_frame=Frame(3.0); ready_frame.ball_target=Ball(.5,.8);
    ready_frame.delivery_feedback={unreserved_id,true,true,false};
    ready_result=unreserved.Step(ready_frame);
    assert(!ready_result.object_ready_mean_used && ready_result.post_motion_observing);
    assert(ready_result.command.command_type==CommandType::kHold);
    assert(ready_result.command.action_id==unreserved_id);
    for (int i=1;i<=9;++i) {
      ready_frame=Frame(3.0+i*.1); ready_frame.ball_target=Ball(.5,.8);
      ready_result=unreserved.Step(ready_frame);
      assert(ready_result.post_motion_observing && !ready_result.object_ready_mean_used);
      assert(ready_result.command.action_id==unreserved_id);
    }
    ready_frame=Frame(4.01); ready_frame.ball_target=Ball(.5,.8);
    ready_result=unreserved.Step(ready_frame);
    assert(!ready_result.post_motion_observing && !ready_result.object_ready_mean_used);
    assert(ready_result.command.action==MissionAction::kStepForwardTwo);
    assert(ready_result.command.action_id!=0 && ready_result.command.action_id!=unreserved_id);
  }

  // Straight two-step is deliberately not a moving READY candidate.
  MissionController straight_two(ready_cfg);
  AdmitObjectAtRest(straight_two, Ball(.5,.85));
  ready_frame=Frame(0.0); ready_frame.ball_target=Ball(.5,.85);
  ready_result=straight_two.Step(ready_frame);
  assert(ready_result.command.action==MissionAction::kStepForwardTwo);
  const auto two_id=ready_result.command.action_id;
  for (int i=1;i<=27;++i) {
    ready_frame=Frame(i*.1); ready_frame.ball_target=Ball(.5,.85);
    ready_frame.delivery_feedback={two_id,true,false,i==27};
    ready_result=straight_two.Step(ready_frame);
  }
  ready_frame=Frame(3.0);
  ready_frame.delivery_feedback={two_id,true,true,false};
  ready_result=straight_two.Step(ready_frame);
  assert(!ready_result.object_ready_mean_used);
  assert(ready_result.command.camera_request==CameraRequest::kDown);

  // Missing READY: reject moving samples and retain stationary 1s gate.
  MissionController no_ready(ready_cfg);
  AdmitObjectAtRest(no_ready, Ball(.5,.2));
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
  assert(!ready_result.object_ready_mean_used);
  // During the 1s stationary window, DONE is deferred and the acknowledged
  // action's ID remains pending as a HOLD. Zero ID is NOT the contract.
  assert(ready_result.command.command_type==CommandType::kHold);
  assert(ready_result.command.action_id==no_ready_id);
  // The deferred DONE must be consumed only after fresh stationary evidence.
  for (int i=1;i<=9;++i) {
    ready_frame=Frame(3.0+i*.1);
    ready_frame.ball_target=Ball(.5,.8);
    ready_result=no_ready.Step(ready_frame);
    assert(ready_result.post_motion_observing);
    assert(!ready_result.object_ready_mean_used);
    assert(ready_result.command.command_type==CommandType::kHold);
    assert(ready_result.command.action_id==no_ready_id);
  }
  ready_frame=Frame(4.0);
  ready_frame.ball_target=Ball(.5,.8);
  ready_result=no_ready.Step(ready_frame);
  assert(!ready_result.post_motion_observing);
  assert(!ready_result.object_ready_mean_used);
  assert(ready_result.command.action==MissionAction::kStepForwardTwo);
  assert(ready_result.command.action_id!=no_ready_id);

  // HURDLE: far-center three-step mean leads to three-step, observation_complete
  // must be set when consuming verified READY evidence at DONE.
  auto hurdle_ready_cfg=ready_cfg;
  hurdle_ready_cfg.enable_ball=false; hurdle_ready_cfg.enable_hurdle=true;
  hurdle_ready_cfg.hurdle.down_search_min_hits=1; hurdle_ready_cfg.hurdle.stable_window=hurdle_ready_cfg.hurdle.stable_min_hits=1;
  hurdle_ready_cfg.hurdle.upper_acquire_v_px=480;
  MissionController hurdle_ready(hurdle_ready_cfg);
  AdmitObjectAtRest(hurdle_ready, Ball(.5,.3), MissionType::kHurdle);
  ready_frame=Frame(0.0); ready_frame.hurdle_target=Ball(.5,.3);
  ready_result=hurdle_ready.Step(ready_frame);
  assert(ready_result.command.action==MissionAction::kStepForwardThree);
  const auto hurdle_ready_id=ready_result.command.action_id;
  for (int i=1;i<=27;++i) {
    ready_frame=Frame(i*.1);
    ready_frame.hurdle_target=Ball(.5,i>=12 ? .6 : .3);
    ready_frame.delivery_feedback={hurdle_ready_id,true,false,i==27};
    ready_result=hurdle_ready.Step(ready_frame);
  }
  assert(ready_result.command.control_phase==ControlPhase::kWaitingQueuedActionAck);
  assert(ready_result.command.action==MissionAction::kStepForwardThree);
  const auto hurdle_queued_id=ready_result.command.action_id;
  ready_frame=Frame(3.0); ready_frame.hurdle_target=Ball(.5,.9);
  ready_frame.delivery_feedback={hurdle_ready_id,true,true,false};
  ready_result=hurdle_ready.Step(ready_frame);
  assert(!ready_result.post_motion_observing);
  assert(ready_result.object_ready_mean_used);
  assert(ready_result.command.action==MissionAction::kStepForwardThree);
  assert(ready_result.command.action_id==hurdle_queued_id);

  // Full AUTO GOAL uses shared units, and near commitment drains its real
  // current unit instead of queuing another far walk from a READY mean.
  auto goal_ready_cfg=ready_cfg;
  goal_ready_cfg.enable_ball=false;goal_ready_cfg.enable_goal=true;
  goal_ready_cfg.goal_trial_start_goal=true;
  goal_ready_cfg.goal.stable_window=goal_ready_cfg.goal.stable_min_hits=1;
  MissionController goal_ready(goal_ready_cfg);
  for(int i=0;i<40;++i) {
    auto startup=Frame(.05*i);startup.camera_feedback={CameraMode::kGoal,true};
    startup.backboard_target=Ball(.5,.3);startup.backboard_center_depth_m=1.8;
    startup.allow_new_body_actions=false;
    assert(goal_ready.Step(startup).command.action_id==0);
  }
  ready_frame=Frame(2);ready_frame.camera_feedback={CameraMode::kGoal,true};
  ready_frame.backboard_target=Ball(.5,.3);ready_frame.backboard_center_depth_m=1.8;
  ready_result=goal_ready.Step(ready_frame);
  assert(LinePlan::IsUnit(ready_result.command.action));
  const auto goal_ready_id=ready_result.command.action_id;
  for(int i=1;i<=27;++i) {
    ready_frame=Frame(2+i*.1);ready_frame.camera_feedback={CameraMode::kGoal,true};
    ready_frame.backboard_target=Ball(.5,.3);
    ready_frame.backboard_center_depth_m=i==3 ? .5 : 1.8;
    ready_frame.delivery_feedback={goal_ready_id,true,false,i==27};
    ready_result=goal_ready.Step(ready_frame);
  }
  assert(ready_result.approach_near && ready_result.command.action_id==goal_ready_id);
  assert(ready_result.command.control_phase!=ControlPhase::kWaitingQueuedActionAck);
  ready_frame.delivery_feedback={goal_ready_id,true,true,false};ready_frame.now_sec+=.1;
  ready_result=goal_ready.Step(ready_frame);
  assert(ready_result.active_mission==MissionType::kLine);
  for(int j=1;j<=11 && ready_result.active_mission==MissionType::kLine;++j) {
    ready_frame.now_sec+=.1;
    ready_frame.delivery_feedback={};
    ready_result=goal_ready.Step(ready_frame);
  }
  assert(ready_result.active_mission==MissionType::kGoal);
  assert(ready_result.command.action==MissionAction::kDefaultPoseMode);
  assert(ready_result.command.action_id && ready_result.command.action_id!=goal_ready_id);
  // A physical RESET must publish a fresh camera FORWARD request even when
  // the last software camera feedback already claims FORWARD/settled.
  MissionController reset_probe(c);
  reset_probe.BeginPhysicalReset();
  auto reset_step=reset_probe.StepPhysicalReset({}, {CameraMode::kForward,true},0);
  assert(reset_step.command.action==MissionAction::kWalkMode);
  assert(reset_step.command.camera_request==CameraRequest::kForward);
  assert(!reset_step.physical_reset_complete);
  const auto reset_id=reset_step.command.action_id;
  assert(reset_id!=0);
  reset_step=reset_probe.StepPhysicalReset(
      {reset_id,true,true,false},{CameraMode::kTransition,false},1);
  assert(!reset_step.physical_reset_complete);
  reset_step=reset_probe.StepPhysicalReset({},{CameraMode::kForward,true},2);
  assert(reset_step.physical_reset_complete);

  // The Stage setup stance is the ONLY body command allowed before COMMIT.
  auto stage_cfg=c;
  stage_cfg.test_mission=MissionType::kBall;
  stage_cfg.test_stage=3;
  MissionController stage_probe(stage_cfg);
  assert(stage_probe.TestSetupPending());
  auto stage_input=Frame(0);
  stage_input.camera_feedback={CameraMode::kDown,true};
  stage_input.allow_new_body_actions=false;
  stage_input.allow_test_setup_body_actions=true;
  auto stage_step=stage_probe.Step(stage_input);
  assert(stage_step.command.action==MissionAction::kDefaultPoseMode);
  assert(stage_step.command.mission_phase==903);
  const auto stage_id=stage_step.command.action_id;
  assert(stage_id!=0);
  stage_input=Frame(1);
  stage_input.camera_feedback={CameraMode::kDown,true};
  stage_input.allow_new_body_actions=false;
  stage_input.allow_test_setup_body_actions=true;
  stage_input.delivery_feedback={stage_id,true,true,false};
  stage_step=stage_probe.Step(stage_input);
  assert(!stage_probe.TestSetupPending());
  assert(stage_step.command.action_id==0);

  // Without the dedicated stage setup permission manual tuning cannot
  // dispatch its initial pose automatically.
  MissionController gated_stage(stage_cfg);
  auto gated_stage_input=Frame(0);
  gated_stage_input.camera_feedback={CameraMode::kDown,true};
  gated_stage_input.allow_new_body_actions=false;
  auto gated_stage_result=gated_stage.Step(gated_stage_input);
  assert(gated_stage_result.command.action_id==0);
  assert(gated_stage.TestSetupPending());

  // GOAL camera after pickup: wall time and READY alone cannot trigger it.
  // A completed four-step LINE cruise (23) triggers GOAL, without queuing B.
  auto post_pickup_cfg = LoadDefaultAlgorithmConfig(); post_pickup_cfg.transition_settle_sec=0;
  post_pickup_cfg.enable_ball=post_pickup_cfg.enable_hurdle=false;
  post_pickup_cfg.enable_goal=true;
  post_pickup_cfg.initial_has_ball=true; // stands in for confirmed BALL recovery
  post_pickup_cfg.goal.post_pickup_walk_fallback_actions=3;
  post_pickup_cfg.line_features.image_center_u=320;
  post_pickup_cfg.line.line_stable_window=post_pickup_cfg.line.line_stable_min_hits=1;
  MissionController five_step_goal(post_pickup_cfg);
  auto goal_frame=Frame(0.0);
  auto goal_step=five_step_goal.Step(goal_frame);
  assert(goal_step.command.action==MissionAction::kLineStraightUnit);
  const auto five_id=goal_step.command.action_id;
  goal_frame=Frame(.1); goal_frame.delivery_feedback={five_id,true,false,false};
  goal_step=five_step_goal.Step(goal_frame);
  assert(goal_step.active_mission==MissionType::kLine);
  for (double time : {0.8,1.4,1.6,1.8,2.0,2.2}) {
    goal_frame=Frame(time); goal_step=five_step_goal.Step(goal_frame);
  }
  goal_frame=Frame(2.55);
  goal_frame.delivery_feedback={five_id,true,false,true};
  goal_step=five_step_goal.Step(goal_frame);
  assert(goal_step.command.action_id==five_id); // no B reserved at 13 READY
  assert(goal_step.command.camera_request==CameraRequest::kNone);
  goal_frame=Frame(2.8);
  goal_frame.delivery_feedback={five_id,true,true,false};
  goal_step=five_step_goal.Step(goal_frame);
  assert(goal_step.goal_five_step_completed);
  assert(goal_step.goal_walk_actions_completed==1);
  assert(goal_step.active_mission==MissionType::kLine);
  assert(goal_step.command.camera_request==CameraRequest::kGoal);

  // No straight-five choice: three finished turning-forward (12) walks
  // trigger GOAL. The first two may queue their successors, but the third
  // READY must not book a fourth action.
  MissionController three_walk_goal(post_pickup_cfg);
  auto three_frame=Frame(0.0); ShiftLine(three_frame,500);
  auto three_result=three_walk_goal.Step(three_frame);
  assert(three_result.command.action==MissionAction::kLineRightUnit);
  std::uint64_t current_walk=three_result.command.action_id;
  const auto send_three = [&](double at, std::uint64_t id,
                              bool ack, bool done, bool ready) {
    auto frame=Frame(at); ShiftLine(frame,500);
    frame.delivery_feedback={id,ack,done,ready};
    return three_walk_goal.Step(frame);
  };
  three_result=send_three(.1,current_walk,true,false,false);
  for (double t : {.8,1.4,1.6,1.8,2.,2.2})
    three_result=send_three(t,0,false,false,false);
  three_result=send_three(2.55,current_walk,true,false,true);
  assert(three_result.command.control_phase==ControlPhase::kWaitingQueuedActionAck);
  std::uint64_t second_walk=three_result.command.action_id;
  assert(second_walk!=current_walk);
  three_result=send_three(2.6,second_walk,true,false,false);
  three_result=send_three(2.8,current_walk,true,true,false);
  assert(three_result.goal_walk_actions_completed==1);
  current_walk=second_walk;
  for(double t : {2.9,3.3,3.9,4.2,4.4,4.6,4.8,5.0,5.1})
    three_result=send_three(t,0,false,false,false);
  three_result=send_three(5.35,current_walk,true,false,true);
  assert(three_result.command.control_phase==ControlPhase::kWaitingQueuedActionAck);
  const auto third_walk=three_result.command.action_id;
  three_result=send_three(5.4,third_walk,true,false,false);
  three_result=send_three(5.6,current_walk,true,true,false);
  assert(three_result.goal_walk_actions_completed==2);
  current_walk=third_walk;
  for(double t : {5.7,6.1,6.7,7.0,7.2,7.4,7.6,7.8,8.0})
    three_result=send_three(t,0,false,false,false);
  three_result=send_three(8.15,current_walk,true,false,true);
  assert(three_result.command.action_id==current_walk); // no fourth queue
  assert(three_result.command.camera_request==CameraRequest::kNone);
  three_result=send_three(8.4,current_walk,true,true,false);
  assert(three_result.goal_walk_actions_completed==3);
  assert(!three_result.goal_five_step_completed);
  assert(three_result.active_mission==MissionType::kLine);
  assert(three_result.command.camera_request==CameraRequest::kGoal);
  three_result=send_three(8.5,current_walk,true,true,false); // duplicate DONE
  assert(three_result.goal_walk_actions_completed==3);

  // 복구용 제자리 회전은 post-pickup LINE 보행 블록으로 세지 않는다.
  MissionController rotation_only(post_pickup_cfg);
  auto turn_frame=Frame(0.0); ShiftLine(turn_frame,500);
  auto turn_result=rotation_only.Step(turn_frame);
  assert(turn_result.command.action==MissionAction::kLineRightUnit);
  const auto first_walk_id=turn_result.command.action_id;
  // Keep LINE valid until DONE: with a one-frame stability window, an
  // earlier loss aborts the logical plan and it must not count as completed.
  turn_frame=Frame(.1); ShiftLine(turn_frame,500);
  turn_frame.delivery_feedback={first_walk_id,true,false,false};
  turn_result=rotation_only.Step(turn_frame);
  assert(turn_result.goal_walk_actions_completed==0);
  turn_frame=Frame(.2); turn_frame.line_centers.clear();
  turn_frame.delivery_feedback={first_walk_id,true,true,false};
  turn_result=rotation_only.Step(turn_frame);
  assert(turn_result.goal_walk_actions_completed==1);
  turn_frame=Frame(2.3); turn_frame.line_centers.clear();
  turn_result=rotation_only.Step(turn_frame);
  assert(turn_result.command.action==MissionAction::kTurnLeftInPlace ||
         turn_result.command.action==MissionAction::kStepForwardRightOneHurdle);
  const auto recovery_turn_id=turn_result.command.action_id;
  turn_frame=Frame(3.9); turn_frame.line_centers.clear();
  turn_frame.delivery_feedback={recovery_turn_id,true,true,false};
  turn_result=rotation_only.Step(turn_frame);
  assert(turn_result.goal_walk_actions_completed==1);
  assert(turn_result.active_mission==MissionType::kLine);
  assert(turn_result.command.camera_request==CameraRequest::kNone);

  // Switching GOAL trial start mode only changes the next explicit Reset.
  {
    auto trial_cfg=LoadDefaultAlgorithmConfig(); trial_cfg.transition_settle_sec=0;
    trial_cfg.enable_ball=false;
    trial_cfg.enable_hurdle=false;
    trial_cfg.enable_goal=true;
    trial_cfg.initial_has_ball=true;
    MissionController trial_toggle(trial_cfg);
    auto trial_line=trial_toggle.Step(Frame(0.));
    assert(trial_line.active_mission==MissionType::kLine);
    const auto original_id=trial_line.command.action_id;
    assert(original_id!=0);
    trial_toggle.SetGoalTrialStartGoal(true);
    // Setter itself must never interrupt the ongoing LINE mission.
    assert(trial_toggle.Step(Frame(.1)).active_mission==MissionType::kLine);
    trial_toggle.Reset(true);
    const auto near_start=trial_toggle.Step(Frame(1.));
    assert(near_start.active_mission==MissionType::kLine);
    assert(near_start.command.action_id==0 && near_start.command.camera_request==CameraRequest::kGoal);
    trial_toggle.SetGoalTrialStartGoal(false);
    trial_toggle.Reset(true);
    const auto line_again=trial_toggle.Step(Frame(2.));
    assert(line_again.active_mission==MissionType::kLine);
    assert(line_again.command.action_id>original_id);
  }

  return 0;
}
