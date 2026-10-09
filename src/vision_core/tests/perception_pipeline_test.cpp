#include "vision_core/config_loader.hpp"
#include "vision_core/mission_controller.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <utility>
using namespace vision_core;

namespace {
PerceptionDetection BallDetection(double u, double v, double confidence=1.0,
                                  double size=20.0) {
  PerceptionDetection d;
  d.detection.class_id=1;
  d.detection.confidence=confidence;
  d.detection.box={u*640.0-size/2, v*480.0-size/2, size, size};
  return d;
}

MissionControllerConfig BallPipelineConfig() {
  auto c=LoadDefaultAlgorithmConfig();
  c.enable_hurdle=c.enable_goal=false;
  c.ball.smooth_alpha=1.0;
  c.ball.upper_acquire_v_norm=1.0;
  c.object_association.max_center_distance_norm=.1;
  return c;
}

struct BallPipeline {
  MissionController controller{BallPipelineConfig()};
  PerceptionFrameInput input;
  PerceptionMissionFrameResult result;
  BallPipeline() {
    input.image_width=640; input.image_height=480;
    input.command_transport_enabled=true;
    // Isolate BALL tests from unrelated LINE decisions without skipping BALL.
    input.advance_line_fsm=false;
  }
  const PerceptionMissionFrameResult &Tick(
      std::vector<PerceptionDetection> detections, bool done=false) {
    input.now_sec+=.1;
    input.detections=std::move(detections);
    input.delivery_feedback={};
    if (done && result.mission.command.action_id!=0)
      input.delivery_feedback={result.mission.command.action_id,true,true,false};
    result=controller.StepPerception(input);
    return result;
  }
  void EnterVerification() {
    // Keep the production 10/7 acquisition and close-trigger thresholds.
    for (int i=0;i<7;++i) Tick({BallDetection(.5,.8)});
    assert(result.mission.ball.mode==BallMode::kWaitCameraDown);
    input.camera_feedback={CameraMode::kDown,true};
    Tick({BallDetection(.5,.7)});
    assert(result.mission.ball.mode==BallMode::kFineAdjustForPickup);
    for (int i=0;i<7;++i) {
      Tick({BallDetection(.5,.7)});
      if (result.mission.ball.mode==BallMode::kPickupBall) break;
    }
    assert(result.mission.command.action==MissionAction::kPickBall);
    assert(result.mission.ball.pickup_attempt_count==1);
    Tick({BallDetection(.5,.7)},true);
    assert(result.mission.command.action==MissionAction::kRecatch);
    Tick({BallDetection(.5,.7)},true);
    assert(result.mission.ball.mode==BallMode::kVerifyPickupObservation);
    assert(result.mission.command.action_id==0);
  }
};

void TestNormalBallAssociation() {
  BallPipeline pipeline;
  for (int i=0;i<7;++i) pipeline.Tick({BallDetection(.2,.3)});
  assert(pipeline.result.mission.ball.mode==BallMode::kApproachBall);
  const auto &matched=pipeline.Tick(
      {BallDetection(.2,.3,.61),BallDetection(.9,.3,.99)});
  assert(matched.perception.targets.ball);
  assert(matched.perception.targets.ball->confidence==.61);
  const auto &rejected=pipeline.Tick({BallDetection(.9,.3,.99)});
  assert(!rejected.perception.targets.ball);
  assert(!rejected.mission.ball.tracked.visible);
  assert(rejected.mission.ball.mode==BallMode::kApproachBall);
}

void TestVerificationBypassesAssociationAndFineReacquires() {
  BallPipeline pipeline;
  pipeline.EnterVerification();
  // All highest-ranked raw candidates are outside the previous identity gate.
  for (int i=0;i<7;++i) {
    const auto &r=pipeline.Tick({BallDetection(.2,.7,.98,50),
                                BallDetection(.9,.7,.99,20),
                                BallDetection(.85,.7,.99,30)});
    assert(r.perception.targets.ball);
    assert(r.perception.targets.ball->center_px.u==.85*640.0);
    assert(r.perception.targets.ball->area_px==900.0);
    assert(r.mission.ball.mode==BallMode::kVerifyPickupObservation);
  }
  pipeline.Tick({}); pipeline.Tick({});
  assert(pipeline.result.mission.ball.mode==BallMode::kVerifyPickupObservation);
  pipeline.Tick({});
  assert(pipeline.result.mission.ball.mode==BallMode::kFineAdjustForPickup);
  assert(pipeline.result.mission.ball.pickup_attempt_count==1);
  assert(pipeline.result.mission.command.action_id==0);
  const auto &fresh=pipeline.Tick({BallDetection(.1,.7)});
  assert(fresh.perception.targets.ball); // New identity acquired immediately.
  assert(fresh.mission.ball.tracked.u_norm==.1);
  assert(!fresh.mission.ball.tracked.stable); // Verification history cleared.
  assert(fresh.mission.ball.action.action==MissionAction::kNone); // Settle.
  for (int i=0;i<6;++i) {
    pipeline.Tick({BallDetection(.1,.7)});
    if (pipeline.result.mission.ball.action.action!=MissionAction::kNone) break;
  }
  assert(pipeline.result.mission.ball.action.action==MissionAction::kLeftSideStep);
  assert(pipeline.result.mission.command.action==MissionAction::kLeftSideStep);
  assert(pipeline.result.mission.command.action_id!=0);
  // Normal association is active again even in the resumed fine state.
  const auto &rejected=pipeline.Tick({BallDetection(.9,.7)});
  assert(!rejected.perception.targets.ball);
  assert(rejected.mission.ball.action.action==MissionAction::kNone);
}

void TestVerificationSixHitsSucceeds() {
  BallPipeline pipeline;
  pipeline.EnterVerification();
  for (int i=0;i<6;++i) pipeline.Tick({BallDetection(.9,.7)});
  for (int i=0;i<3;++i) pipeline.Tick({});
  assert(pipeline.result.mission.ball.mode==BallMode::kVerifyPickupObservation);
  pipeline.Tick({});
  assert(pipeline.result.mission.ball.has_ball);
  assert(!pipeline.result.mission.ball.pickup_failed);
  assert(pipeline.result.mission.ball.mode==BallMode::kStandUpAfterPickup);
}
} // namespace

int main() {
  auto c=LoadDefaultAlgorithmConfig();
  c.enable_ball=c.enable_hurdle=c.enable_goal=false;
  MissionController controller(c);
  PerceptionFrameInput in; in.image_width=640; in.image_height=480;
  in.intrinsics={600,600,320,240}; in.command_transport_enabled=true;
  for (int i=0;i<5;++i) {
    PerceptionDetection d;
    d.detection.class_id=c.line_detection.class_id;
    d.detection.confidence=1;
    d.detection.box={310.0,400.0-i*40.0,20.0,20.0};
    in.detections.push_back(d);
  }
  const auto out=controller.StepPerception(in);
  assert(out.perception.raw_line_centers.size()==5);
  assert(out.mission.line_features.guide.valid);
  assert(out.mission.command.action==MissionAction::kStepForwardFive);
  TestNormalBallAssociation();
  TestVerificationBypassesAssociationAndFineReacquires();
  TestVerificationSixHitsSucceeds();
  return 0;
}
