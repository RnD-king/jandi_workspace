#include "vision_core/ball_controller.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>

using namespace vision_core;
static ObjectTarget Ball(double u, double v) {
  ObjectTarget t; t.center_px = {u * 100.0, v * 100.0};
  t.height_px = 10.0; t.area_px = 100.0; t.confidence = 1.0; return t;
}
static BallResult EnterPickupVerification(BallController &controller,
                                          double start_sec) {
  BallResult result;
  for (int i = 0; i < 7; ++i) {
    result = controller.Compute(Ball(0.5, 0.3), 100, 100,
                                start_sec + 0.1 * i, {}, {});
  }
  assert(result.mode == BallMode::kApproachBall);
  ActionExecutionFeedback done;
  done.action_done = true;
  result = controller.Compute(Ball(0.5, 0.8), 100, 100,
                              start_sec + 0.7, {}, done);
  assert(result.camera_request == CameraRequest::kDown);
  const CameraFeedback down{CameraMode::kDown, true};
  result = controller.Compute(Ball(0.5, 0.7), 100, 100,
                              start_sec + 0.8, down, {});
  result = controller.Compute(Ball(0.5, 0.7), 100, 100,
                              start_sec + 0.9, down, {});
  assert(result.action.action == MissionAction::kPickBall);
  result = controller.Compute(Ball(0.5, 0.7), 100, 100,
                              start_sec + 1.0, down, done);
  assert(result.action.action == MissionAction::kRecatch);
  result = controller.Compute(Ball(0.5, 0.7), 100, 100,
                              start_sec + 1.1, down, done);
  assert(result.mode == BallMode::kVerifyPickupObservation);
  return result;
}
int main() {
  BallConfig c{}; c.stable_window = c.stable_min_hits = 1;
  c.lost_frames = 2; c.smooth_alpha = 1.0; c.far_u_des_norm = 0.5;
  c.approach_u_deadband = 0.05; c.upper_acquire_v_norm = 1.0;
  c.tilt_down_v_norm = 0.7; c.tilt_down_window = c.tilt_down_min_hits = 1;
  c.fine_target_u_norm = 0.5;
  c.fine_target_v_norm = 0.7; c.fine_u_deadband = c.fine_v_deadband = 0.05;
  c.pickup_max_attempts = 2;
  c.recovery_timeout_sec = 5.0; c.recovery_reacquire_min_hits = 1;
  c.recovery_center_tolerance_norm = 0.05;
  BallController controller(c, 3.0);
  auto r = controller.Compute(Ball(0.3, 0.3), 100, 100, 0, {}, {});
  assert(r.mode == BallMode::kApproachBall);
  assert(r.action.action == MissionAction::kStepForwardLeft);
  ActionExecutionFeedback active; active.action_active = true;
  r = controller.Compute(Ball(0.5, 0.8), 100, 100, 0.1, {}, active);
  assert(r.camera_request == CameraRequest::kNone);
  ActionExecutionFeedback done; done.action_done = true;
  r = controller.Compute(Ball(0.5, 0.8), 100, 100, 0.2, {}, done);
  assert(r.camera_request == CameraRequest::kDown);
  CameraFeedback down{CameraMode::kDown, true};
  r = controller.Compute(Ball(0.4, 0.6), 100, 100, 0.3, down, {});
  r = controller.Compute(Ball(0.4, 0.6), 100, 100, 0.4, down, {});
  // u와 v가 모두 틀려도 lateral을 먼저 보정한다.
  assert(r.action.action == MissionAction::kLeftSideStep);
  r = controller.Compute(Ball(0.5, 0.6), 100, 100, 0.5, down, {});
  assert(r.action.action == MissionAction::kStepForwardHalf);
  r = controller.Compute(Ball(0.5, 0.8), 100, 100, 0.6, down, {});
  assert(r.action.action == MissionAction::kStepBack);
  r = controller.Compute(Ball(0.5, 0.7), 100, 100, 0.7, down, {});
  assert(r.action.action == MissionAction::kPickBall);
  assert(r.pickup_attempt_count == 1);
  r = controller.Compute(Ball(0.5, 0.7), 100, 100, 0.8, down, done);
  assert(r.action.action == MissionAction::kRecatch);

  // DOWN fine recovery는 이미 내려간 카메라를 다시 요청하지 않고 TURN만 한다.
  BallController recovery(c, 3.0);
  r = recovery.Compute(Ball(0.3, 0.3), 100, 100, 0, {}, {});
  r = recovery.Compute(Ball(0.3, 0.8), 100, 100, .1, {}, done);
  r = recovery.Compute(Ball(0.3, 0.6), 100, 100, .2, down, {});
  r = recovery.Compute(std::nullopt, 100, 100, .3, down, {});
  r = recovery.Compute(std::nullopt, 100, 100, .4, down, {});
  assert(r.mode == BallMode::kBallRecoveryDown);
  r = recovery.Compute(std::nullopt, 100, 100, .5, down, {});
  assert(r.action.action == MissionAction::kTurnLeft);
  assert(r.camera_request == CameraRequest::kNone);

  // pickup 성공 뒤 STEP_BACK 완료 시 즉시 camera-forward 단계로 넘어가며
  // 같은 back-away action을 다시 발행하지 않는다.
  r = controller.Compute(std::nullopt, 100, 100, .9, down, done);
  r = controller.Compute(std::nullopt, 100, 100, 1.0, down, {});
  assert(r.action.action == MissionAction::kDefaultPosition);
  r = controller.Compute(std::nullopt, 100, 100, 1.1, down, done);
  assert(r.action.action == MissionAction::kStepBack);
  r = controller.Compute(std::nullopt, 100, 100, 1.2, down, done);
  assert(r.action.action == MissionAction::kNone);
  assert(r.camera_request == CameraRequest::kForward);

  BallConfig verify_config = c;
  verify_config.stable_window = 10;
  verify_config.stable_min_hits = 7;

  // 9 frame까지는 7회 검출됐더라도 판정하지 않고, 10번째 frame에서만
  // 공이 남아 있다고 판단하여 fine으로 복귀한다.
  BallController seven_hits(verify_config, 3.0);
  r = EnterPickupVerification(seven_hits, 10.0);
  for (int i = 0; i < 7; ++i)
    r = seven_hits.Compute(Ball(0.5, 0.7), 100, 100, 12.0 + 0.1 * i,
                           down, {});
  for (int i = 0; i < 2; ++i)
    r = seven_hits.Compute(std::nullopt, 100, 100, 12.7 + 0.1 * i,
                           down, {});
  assert(r.mode == BallMode::kVerifyPickupObservation);
  assert(!r.has_ball && !r.pickup_failed);
  r = seven_hits.Compute(std::nullopt, 100, 100, 12.9, down, {});
  assert(r.mode == BallMode::kFineAdjustForPickup);
  assert(r.action.action == MissionAction::kNone);
  assert(r.pickup_attempt_count == 1);
  r = seven_hits.Compute(Ball(0.2, 0.7), 100, 100, 13.0, down, {});
  assert(r.action.action == MissionAction::kLeftSideStep);
  assert(r.pickup_attempt_count == 1);
  r = seven_hits.Compute(Ball(0.5, 0.7), 100, 100, 13.1, down, {});
  assert(r.action.action == MissionAction::kPickBall);
  assert(r.pickup_attempt_count == 2);

  // 두 번째 attempt에서도 10 frame 중 7회 검출이면 기존 max-attempt
  // 실패 흐름으로 종료한다.
  r = seven_hits.Compute(Ball(0.5, 0.7), 100, 100, 13.2, down, done);
  r = seven_hits.Compute(Ball(0.5, 0.7), 100, 100, 13.3, down, done);
  for (int i = 0; i < 7; ++i)
    r = seven_hits.Compute(Ball(0.5, 0.7), 100, 100, 13.4 + 0.1 * i,
                           down, {});
  for (int i = 0; i < 3; ++i)
    r = seven_hits.Compute(std::nullopt, 100, 100, 14.1 + 0.1 * i,
                           down, {});
  assert(r.mode == BallMode::kStandUpAfterPickup);
  assert(r.pickup_failed && !r.has_ball);

  // 정확히 10 frame 중 6회만 검출되면 7-hit 실패 조건의 여집합으로
  // pickup 성공을 확정한다.
  BallController six_hits(verify_config, 3.0);
  r = EnterPickupVerification(six_hits, 20.0);
  for (int i = 0; i < 6; ++i)
    r = six_hits.Compute(Ball(0.5, 0.7), 100, 100, 22.0 + 0.1 * i,
                         down, {});
  for (int i = 0; i < 3; ++i)
    r = six_hits.Compute(std::nullopt, 100, 100, 22.6 + 0.1 * i,
                         down, {});
  assert(r.mode == BallMode::kVerifyPickupObservation);
  assert(!r.has_ball && !r.pickup_failed);
  r = six_hits.Compute(std::nullopt, 100, 100, 22.9, down, {});
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
      r = three_attempts.Compute(Ball(0.9, 0.7), 100, 100,
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
    r = three_attempts.Compute(Ball(0.5, 0.6), 100, 100,
                              retry_time, down, {});
    retry_time += 0.1;
    assert(!r.tracked.stable); // New history, not the 10 verification hits.
    assert(r.action.action == MissionAction::kStepForwardHalf);
    r = three_attempts.Compute(Ball(0.5, 0.8), 100, 100,
                              retry_time, down, {});
    retry_time += 0.1;
    assert(r.action.action == MissionAction::kStepBack);
    r = three_attempts.Compute(Ball(0.8, 0.7), 100, 100,
                              retry_time, down, {});
    retry_time += 0.1;
    assert(r.action.action == MissionAction::kRightSideStep);
    assert(r.pickup_attempt_count == attempt);
    r = three_attempts.Compute(Ball(0.5, 0.7), 100, 100,
                              retry_time, down, {});
    retry_time += 0.1;
    assert(r.action.action == MissionAction::kPickBall);
    assert(r.pickup_attempt_count == attempt + 1);
    ++actual_picks;
    r = three_attempts.Compute(Ball(0.5, 0.7), 100, 100,
                              retry_time, down, done);
    retry_time += 0.1;
    assert(r.action.action == MissionAction::kRecatch);
    r = three_attempts.Compute(Ball(0.5, 0.7), 100, 100,
                              retry_time, down, done);
    retry_time += 0.1;
    assert(r.mode == BallMode::kVerifyPickupObservation);
  }
  assert(actual_picks == verify_config.pickup_max_attempts);
  r = three_attempts.Compute(Ball(0.5, 0.7), 100, 100,
                            retry_time, down, {});
  assert(r.mode == BallMode::kStandUpAfterPickup);
  assert(r.pickup_attempt_count == 3);
  assert(r.action.action != MissionAction::kPickBall);
  return 0;
}
