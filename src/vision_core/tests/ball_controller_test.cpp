#include "vision_core/ball_controller.hpp"
#include "vision_core/config_loader.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <utility>
#include <vector>

using namespace vision_core;
static ObjectTarget Ball(double u, double v) {
  ObjectTarget t; t.center_px = {u * 640.0, v * 480.0};
  t.height_px = 10.0; t.area_px = 100.0; t.confidence = 1.0; return t;
}
static BallResult EnterPickupVerification(BallController &controller,
                                          double start_sec) {
  BallResult result;
  for (int i = 0; i < 7; ++i) {
    result = controller.Compute(Ball(0.5, 0.3), 640,480,
                                start_sec + 0.1 * i, {}, {});
  }
  assert(result.mode == BallMode::kApproachBall);
  ActionExecutionFeedback done;
  done.action_done = true;
  controller.ClearObservationHistory(); // A fresh post-motion view starts a new window.
  result = controller.Compute(Ball(0.5, 0.8), 640,480,
                              start_sec + 0.7, {}, done);
  assert(result.action.action == MissionAction::kStepForwardTwo);
  result = controller.Compute(Ball(.5,.8),640,480,start_sec+.71,{},done);
  assert(result.camera_request == CameraRequest::kDown);
  const CameraFeedback down{CameraMode::kDown, true};
  for (int i=0;i<8;++i)
    result=controller.Compute(Ball(.5,.7),640,480,start_sec+.8+.1*i,down,{});
  result=controller.Compute(Ball(.5,.7),640,480,start_sec+1.6,down,{});
  assert(result.mode==BallMode::kWaitFinePose);
  assert(result.action.action==MissionAction::kDefaultPoseMode);
  result=controller.Compute(Ball(.5,.7),640,480,start_sec+1.61,down,done);
  assert(result.mode==BallMode::kFineAdjustForPickup);
  for(int i=0;i<10;++i)
    result=controller.Compute(Ball(.5,.7),640,480,
                              start_sec+1.62+.01*i,down,{});
  assert(result.action.action==MissionAction::kPickBall);
  result=controller.Compute(Ball(.5,.7),640,480,start_sec+1.8,down,done);
  assert(result.mode==BallMode::kVerifyPickupObservation);
  assert(result.action.action==MissionAction::kNone);
  return result;
}
int main() {
  BallConfig c=LoadDefaultAlgorithmConfig().ball; c.fine_settle_duration_sec=0; c.recovery_settle_duration_sec=0; c.stable_window = c.stable_min_hits = 1;
  c.lost_frames = 2; c.smooth_alpha = 1.0; c.far_u_des_px = 320.0;
  c.approach_u_deadband_px = 32.0; c.upper_acquire_v_px = 480.0;
  c.approach_short_v_px = 300.0;
  c.approach_two_step_v_px = 336.0;
  c.pickup_u1_px=288; c.pickup_u2_px=352;
  c.pickup_v1_px=312; c.pickup_v2_px=360;
  c.pickup_max_attempts = 2;
  c.recovery_timeout_sec = 5.0; c.recovery_reacquire_min_hits = 1;
  c.recovery_center_tolerance_px = 32.0;
  // Ball approach grid: NORMAL LINE cruise actions 11/12/13 are forbidden.
  for (const auto& scenario : std::vector<std::pair<ObjectTarget,MissionAction>>{
      {Ball(.5,.3),MissionAction::kStepForwardFour},    // zone 2
      {Ball(.5,.65),MissionAction::kStepForwardThree},  // zone 5
      {Ball(.5,.8),MissionAction::kStepForwardTwo},     // zone 8
      {Ball(.3,.8),MissionAction::kTurnLeftInPlace}, // zone 7
      {Ball(.7,.8),MissionAction::kStepForwardRightOneHurdle}, // zone 9
      {Ball(.3,.3),MissionAction::kStepForwardLeftTwo}, // zone 1
      {Ball(.7,.3),MissionAction::kStepForwardRightTwo}, // zone 3
      {Ball(.3,.65),MissionAction::kStepForwardLeftOne}, // zone 4
      {Ball(.7,.65),MissionAction::kStepForwardRightOne}, // zone 6
      {Ball(.3,300.0/480),MissionAction::kStepForwardLeftOne}, // row boundary
      {Ball(.3,336.0/480),MissionAction::kTurnLeftInPlace}, // near boundary
      {Ball(.45,.8),MissionAction::kStepForwardTwo}, // inclusive left center edge
      {Ball(.55,.8),MissionAction::kStepForwardTwo}}) { // inclusive right center edge
    BallController grid(c,3.0);
    const auto v=grid.Compute(scenario.first,640,480,0,{}, {});
    assert(v.action.action==scenario.second);
    assert(v.action.category==ActionCategory::kLocomotion);
    if (scenario.second==MissionAction::kStepForwardLeftOne ||
        scenario.second==MissionAction::kTurnLeftInPlace ||
        scenario.second==MissionAction::kStepForwardRightOneHurdle)
      assert(v.camera_request==CameraRequest::kNone);
  }
  BallController controller(c, 3.0);
  auto r = controller.Compute(Ball(0.3, 0.3), 640,480, 0, {}, {});
  assert(r.mode == BallMode::kApproachBall);
  assert(r.action.action == MissionAction::kStepForwardLeftTwo);
  ActionExecutionFeedback active; active.action_active = true;
  r = controller.Compute(Ball(0.5, 0.8), 640,480, 0.1, {}, active);
  assert(r.camera_request == CameraRequest::kNone);
  ActionExecutionFeedback done; done.action_done = true;
  r = controller.Compute(Ball(0.5, 0.8), 640,480, 0.2, {}, done);
  assert(r.action.action == MissionAction::kStepForwardTwo);
  assert(r.camera_request == CameraRequest::kNone);
  r=controller.Compute(Ball(.5,.8),640,480,.21,{},done);
  assert(r.camera_request == CameraRequest::kDown);
  CameraFeedback down{CameraMode::kDown, true};
  r = controller.Compute(Ball(0.4, 0.6), 640,480, 0.3, down, {});
  r = controller.Compute(Ball(0.4, 0.6), 640,480, 0.4, down, {});
  r = controller.Compute(Ball(.4,.6),640,480,.401,down,{});
  assert(r.action.action==MissionAction::kDefaultPoseMode);
  r=controller.Compute(Ball(.4,.6),640,480,.402,down,done);
  r=controller.Compute(Ball(.4,.6),640,480,.403,down,{});
  // With identical normalized overruns, lateral is the deterministic tie-break.
  assert(r.action.action == MissionAction::kLeftSideStep);
  // The farther OUTSIDE-axis wins; v dominates even if u is also outside.
  r = controller.Compute(Ball(0.62, 0.40), 640,480, 0.45, down, {});
  assert(r.action.action == MissionAction::kStepForwardHalf); // Fine stance never resumes 21.
  r = controller.Compute(Ball(0.5, 0.6), 640,480, 0.5, down, {});
  assert(r.action.action == MissionAction::kStepForwardHalf);
  r = controller.Compute(Ball(0.5, 0.8), 640,480, 0.6, down, {});
  assert(r.action.action == MissionAction::kStepBack);
  r = controller.Compute(Ball(0.5, 0.7), 640,480, 0.7, down, {});
  assert(r.action.action == MissionAction::kPickBall);
  assert(r.pickup_attempt_count == 1);
  r = controller.Compute(Ball(0.5, 0.7), 640,480, 0.8, down, done);
  assert(r.mode==BallMode::kVerifyPickupObservation);
  assert(r.action.action == MissionAction::kNone);

  // Losing an already acquired DOWN target starts stationary recovery,
  // never the initial blind-forward search or a new camera movement.
  BallController recovery(c, 3.0);
  r = recovery.Compute(Ball(0.3, 0.3), 640,480, 0, {}, {});
  r = recovery.Compute(Ball(0.5, 0.8), 640,480, .1, {}, done);
  assert(r.action.action==MissionAction::kStepForwardTwo);
  recovery.Compute({},640,480,.11,{},done);
  r = recovery.Compute(Ball(0.3, 0.6), 640,480, .2, down, {});
  // DONE(.2) clears FORWARD-view tracking. A NEW DOWN-view sighting
  // is required before recovery may use the remembered left direction.
  r = recovery.Compute(Ball(0.3, 0.6), 640,480, .25, down, {});
  r = recovery.Compute(Ball(.3,.6),640,480,.26,down,{});
  assert(r.action.action==MissionAction::kDefaultPoseMode);
  recovery.Compute({},640,480,.27,down,done);
  r = recovery.Compute(std::nullopt, 640,480, .3, down, {});
  r = recovery.Compute(std::nullopt, 640,480, .4, down, {});
  assert(r.mode == BallMode::kBallRecoveryDown);
  r = recovery.Compute(std::nullopt, 640,480, .5, down, {});
  assert(r.action.action == MissionAction::kNone);
  assert(r.camera_request == CameraRequest::kNone);

  // Default post-pickup recovery skips all backward steps and requests FORWARD
  // camera immediately on the completed stand-up action.
  assert(c.post_pickup_back_steps == 0);
  r = controller.Compute(std::nullopt, 640,480, .9, down, {});
  assert(r.action.action == MissionAction::kDefaultPosition);
  assert(r.mode == BallMode::kStandUpAfterPickup && r.has_ball);
  r = controller.Compute(std::nullopt, 640,480, 1.0, down, done);
  assert(r.mode == BallMode::kReturnCameraToLine);
  assert(r.action.action == MissionAction::kNone);
  assert(r.camera_request == CameraRequest::kForward);
  r = controller.Compute(std::nullopt, 640,480, 1.1, down, {});
  assert(r.action.action == MissionAction::kNone);
  assert(r.camera_request == CameraRequest::kForward);
  r = controller.Compute(std::nullopt, 640,480, 1.2, {CameraMode::kForward, true}, {});
  assert(r.mode == BallMode::kPostPickupLineRecovery);
  assert(r.action.action == MissionAction::kNone);

  BallConfig verify_config = c;
  verify_config.stable_window = 10;
  verify_config.stable_min_hits = 7;
  auto fine_window = [&](BallController& ctrl,double u,double v,double& time) {
    BallResult observed, first;
    for(int i=0;i<10;++i) {
      observed=ctrl.Compute(Ball(u,v),640,480,time,down,{});
      if(i==0) first=observed;
      time+=.1;
    }
    assert(!first.tracked.stable && first.action.action==MissionAction::kNone);
    return observed;
  };

  // 9 frame까지는 7회 검출됐더라도 판정하지 않고, 10번째 frame에서만
  // 공이 남아 있다고 판단하여 fine으로 복귀한다.
  BallController seven_hits(verify_config, 3.0);
  r = EnterPickupVerification(seven_hits, 10.0);
  for (int i = 0; i < 7; ++i)
    r = seven_hits.Compute(Ball(0.5, 0.7), 640,480, 12.0 + 0.1 * i,
                           down, {});
  for (int i = 0; i < 2; ++i)
    r = seven_hits.Compute(std::nullopt, 640,480, 12.7 + 0.1 * i,
                           down, {});
  assert(r.mode == BallMode::kVerifyPickupObservation);
  assert(!r.has_ball && !r.pickup_failed);
  r = seven_hits.Compute(std::nullopt, 640,480, 12.9, down, {});
  assert(r.mode == BallMode::kFineAdjustForPickup);
  assert(r.action.action == MissionAction::kNone);
  assert(r.pickup_attempt_count == 1);
  double seven_time=13.0;
  r=fine_window(seven_hits,0.2,0.7,seven_time);
  assert(r.action.action == MissionAction::kLeftSideStep);
  assert(r.pickup_attempt_count == 1);
  seven_hits.ClearObservationHistory(); // Fresh post-lateral-motion coordinates.
  r=fine_window(seven_hits,0.5,0.7,seven_time);
  assert(r.action.action == MissionAction::kPickBall);
  assert(r.pickup_attempt_count == 2);

  // 두 번째 attempt에서도 10 frame 중 7회 검출이면 기존 max-attempt
  // 실패 흐름으로 종료한다.
  r = seven_hits.Compute(Ball(0.5, 0.7), 640,480, seven_time, down, done);
  seven_time+=.1;
  for (int i = 0; i < 7; ++i)
    r = seven_hits.Compute(Ball(0.5, 0.7), 640,480,
                           seven_time + 0.1 * i, down, {});
  for (int i = 0; i < 3; ++i)
    r = seven_hits.Compute(std::nullopt, 640,480,
                           seven_time + 0.7 + 0.1 * i, down, {});
  assert(r.mode == BallMode::kStandUpAfterPickup);
  assert(r.pickup_failed && !r.has_ball);

  // 정확히 10 frame 중 6회만 검출되면 7-hit 실패 조건의 여집합으로
  // pickup 성공을 확정한다.
  BallController six_hits(verify_config, 3.0);
  r = EnterPickupVerification(six_hits, 20.0);
  for (int i = 0; i < 6; ++i)
    r = six_hits.Compute(Ball(0.5, 0.7), 640,480, 22.0 + 0.1 * i,
                         down, {});
  for (int i = 0; i < 3; ++i)
    r = six_hits.Compute(std::nullopt, 640,480, 22.6 + 0.1 * i,
                         down, {});
  assert(r.mode == BallMode::kVerifyPickupObservation);
  assert(!r.has_ball && !r.pickup_failed);
  r = six_hits.Compute(std::nullopt, 640,480, 22.9, down, {});
  assert(r.mode == BallMode::kStandUpAfterPickup);
  assert(r.has_ball && !r.pickup_failed);
  assert(r.action.action == MissionAction::kDefaultPosition);

  // Fine retries preserve attempt count and never permit a fourth PICK.
  verify_config.pickup_max_attempts = 3;
  BallController three_attempts(verify_config, 3.0);
  r = EnterPickupVerification(three_attempts, 30.0);
  double retry_time = 32.0;
  int actual_picks = 1;
  for (int attempt = 1; attempt <= 3; ++attempt) {
    for (int i = 0; i < 10; ++i) {
      r = three_attempts.Compute(Ball(0.9, 0.7), 640,480,
                                retry_time, down, {});
      retry_time += 0.1;
    }
    assert(r.pickup_attempt_count == attempt);
    assert(!r.has_ball);
    if (attempt == 3) {
      assert(r.pickup_failed);
      assert(r.mode == BallMode::kStandUpAfterPickup);
      assert(r.action.action == MissionAction::kDefaultPosition);
      break;
    }
    assert(r.mode == BallMode::kFineAdjustForPickup);
    assert(r.action.action == MissionAction::kNone);
    r=fine_window(three_attempts,0.5,0.6,retry_time);
    assert(r.action.action == MissionAction::kStepForwardHalf);
    three_attempts.ClearObservationHistory(); // Next action uses fresh observation.
    r=fine_window(three_attempts,0.5,0.8,retry_time);
    assert(r.action.action == MissionAction::kStepBack);
    three_attempts.ClearObservationHistory(); // Next action uses fresh observation.
    r=fine_window(three_attempts,0.8,0.7,retry_time);
    assert(r.action.action == MissionAction::kRightSideStep);
    assert(r.pickup_attempt_count == attempt);
    three_attempts.ClearObservationHistory(); // Next action uses fresh observation.
    r=fine_window(three_attempts,0.5,0.7,retry_time);
    assert(r.action.action == MissionAction::kPickBall);
    assert(r.pickup_attempt_count == attempt + 1);
    ++actual_picks;
    r = three_attempts.Compute(Ball(0.5, 0.7), 640,480,
                              retry_time, down, done);
    retry_time += 0.1;
    assert(r.action.action == MissionAction::kNone);
    assert(r.mode == BallMode::kVerifyPickupObservation);
  }
  // Camera-moving frames must not survive into the DOWN-view fine adjustment.
  BallController fresh_down(c,3.0);
  fresh_down.Compute(Ball(.3,.3),640,480,0,{},{});
  r=fresh_down.Compute(Ball(.5,.8),640,480,.1,{},done);
  assert(r.mode==BallMode::kApproachTwoBeforeDown);
  r=fresh_down.Compute({},640,480,.11,{},done);
  assert(r.mode==BallMode::kWaitCameraDown);
  r=fresh_down.Compute(Ball(.9,.9),640,480,.15,
                       {CameraMode::kTransition,false},{});
  assert(r.mode==BallMode::kWaitCameraDown && r.tracked.u_norm < .6);
  r=fresh_down.Compute(Ball(.9,.9),640,480,.2,down,{});
  assert(r.mode==BallMode::kDownSearch && !r.tracked.visible);
  r=fresh_down.Compute(Ball(.2,.7),640,480,.3,down,{});
  r=fresh_down.Compute(Ball(.2,.7),640,480,.31,down,{});
  assert(r.action.action==MissionAction::kDefaultPoseMode);
  r=fresh_down.Compute({},640,480,.32,down,done);
  r=fresh_down.Compute(Ball(.2,.7),640,480,.33,down,{});
  assert(r.action.action==MissionAction::kLeftSideStep);

  assert(actual_picks == verify_config.pickup_max_attempts);
  r = three_attempts.Compute(Ball(0.5, 0.7), 640,480,
                            retry_time, down, {});
  assert(r.mode == BallMode::kStandUpAfterPickup);
  assert(r.pickup_attempt_count == 3);
  assert(r.action.action != MissionAction::kPickBall);
  // A 7/10 stable left detection is averaged before the 5 lost frames.
  BallConfig walk_c=verify_config;
  walk_c.lost_frames=5;
  walk_c.recovery_observation_sec=1.0;
  walk_c.recovery_max_observations=2;
  walk_c.recovery_walk_turn_max=5;
  walk_c.recovery_center_tolerance_px=40;
  BallController walk_recovery(walk_c,3.0);
  double walk_time=50.0;
  for(int i=0;i<7;++i) {
    const double u=i==6 ? .45 : .20; // last raw frame drifts towards center
    r=walk_recovery.Compute(Ball(u,.3),640,480,walk_time,{}, {});
    walk_time+=.1;
  }
  assert(r.mode==BallMode::kApproachBall);
  for(int i=0;i<5;++i) {
    r=walk_recovery.Compute({},640,480,walk_time,{}, {});
    walk_time+=.1;
  }
  assert(r.mode==BallMode::kBallRecoveryForward);
  for(int i=0;i<18 && r.mode!=BallMode::kBallRecoveryWalkTurn;++i) {
    r=walk_recovery.Compute({},640,480,walk_time,{}, {});
    walk_time+=.1;
  }
  assert(r.mode==BallMode::kBallRecoveryWalkTurn);
  for(int turn=0;turn<5;++turn) {
    r=walk_recovery.Compute({},640,480,walk_time,{}, {});
    assert(r.action.action==MissionAction::kStepForwardLeftOne); // action 27
    walk_time+=.1;
    ActionExecutionFeedback moving; moving.action_active=true;
    r=walk_recovery.Compute({},640,480,walk_time,{},moving);
    assert(r.action.action==MissionAction::kNone);
    walk_time+=.1;
    ActionExecutionFeedback completed; completed.action_done=true;
    completed.observation_complete=true;
    r=walk_recovery.Compute({},640,480,walk_time,{},completed);
    assert(r.action.action==MissionAction::kNone);
    assert(walk_recovery.RecoveryWalkTurnsCompleted()==turn+1);
    walk_time+=.1;
  }
  assert(r.mode==BallMode::kBallRecoveryForward);
  for(int i=0;i<24 && r.mode!=BallMode::kFailed;++i) {
    r=walk_recovery.Compute({},640,480,walk_time,{}, {});
    walk_time+=.1;
  }
  assert(r.mode==BallMode::kFailed); // original 2 stationary attempts preserved

  BallController found_after_turn(walk_c,3.0);
  double find_time=80.0;
  for(int i=0;i<7;++i)
    found_after_turn.Compute(Ball(.8,.3),640,480,find_time+=.1,{},{});
  for(int i=0;i<5;++i)
    r=found_after_turn.Compute({},640,480,find_time+=.1,{},{});
  for(int i=0;i<18 && r.mode!=BallMode::kBallRecoveryWalkTurn;++i)
    r=found_after_turn.Compute({},640,480,find_time+=.1,{},{});
  assert(r.mode==BallMode::kBallRecoveryWalkTurn);
  r=found_after_turn.Compute({},640,480,find_time+=.1,{},{});
  assert(r.action.action==MissionAction::kStepForwardRightOneHurdle); // 28
  ActionExecutionFeedback moving2; moving2.action_active=true;
  for(int i=0;i<10;++i)
    r=found_after_turn.Compute(Ball(.8,.3),640,480,find_time+=.1,{},moving2);
  ActionExecutionFeedback completed2; completed2.action_done=true;
  completed2.observation_complete=true;
  r=found_after_turn.Compute(Ball(.8,.3),640,480,find_time+=.1,{},completed2);
  assert(r.mode==BallMode::kApproachBall);
  assert(found_after_turn.RecoveryWalkTurnsCompleted()==1);

  return 0;
}
