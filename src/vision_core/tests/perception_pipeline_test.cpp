#include "vision_core/config_loader.hpp"
#include "vision_core/mission_controller.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <utility>
using namespace vision_core;

namespace {
void TestSharedLineRansacAfterRectification() {
  for (bool imu : {false,true}) {
    auto c=LoadDefaultAlgorithmConfig();
    c.enable_ball=c.enable_hurdle=c.enable_goal=false;
    c.line_features.image_center_u=320;
    MissionController controller(c);
    PerceptionFrameInput in;in.image_width=640;in.image_height=480;
    in.intrinsics={600,600,320,240};in.advance_line_fsm=false;
    in.enable_imu_rectification=imu;in.imu_valid=imu;
    in.roll_rad=.03;in.pitch_rad=-.02;
    for (int i=0;i<8;++i) {
      PerceptionDetection d;
      d.detection.class_id=c.line_detection.class_id;d.detection.confidence=1;
      d.detection.box={i==0 ? 550. : 310.,440.-50.*i,20.,20.};
      in.detections.push_back(d);
    }
    const auto result=controller.StepPerception(in);
    assert(result.perception.raw_line_centers.size()==8);
    assert(result.perception.imu_rectification_applied==imu);
    assert(result.mission.line_features.n_visible==7);
    auto clean=result.perception.line_centers;
    clean.erase(clean.begin()); // The raw bottom point is the false detection.
    const auto expected=ComputeLineFeatures(clean,640,480,c.line_features);
    assert(result.mission.line_features.guide.valid && expected.guide.valid);
    assert(std::abs(result.mission.line_features.guide.offset-expected.guide.offset)<1e-12);
    assert(std::abs(result.mission.line_features.guide.heading_rad-expected.guide.heading_rad)<1e-12);
  }
}

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
  c.ball.upper_acquire_v_px=480.0;
  // Exercise the historical pickup rectangle independently of YAML tuning.
  c.ball.pickup_u1_px=281.6; c.ball.pickup_u2_px=358.4;
  c.ball.pickup_v1_px=307.2; c.ball.pickup_v2_px=364.8;
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
    input.manual_observation_control=true; // This fixture explicitly drives action completion.
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
    // Keep the frozen 10/7 acquisition and close-trigger thresholds.
    for (int i=0;i<18;++i) Tick({BallDetection(.5,.8)});
    assert(result.mission.ball.mode==BallMode::kApproachTwoBeforeDown);
    Tick({},true);
    assert(result.mission.ball.mode==BallMode::kWaitCameraDown);
    input.camera_feedback={CameraMode::kDown,true};
    Tick({BallDetection(.5,.7)});
    assert(result.mission.ball.mode==BallMode::kDownSearch);
    for (int i=0;i<80;++i) {
      Tick({BallDetection(.5,.7)},true);
      if (result.mission.ball.mode==BallMode::kPickupBall) break;
    }
    assert(result.mission.command.action==MissionAction::kPickBall);
    assert(result.mission.ball.pickup_attempt_count==1);
    Tick({BallDetection(.5,.7)},true);
    assert(result.mission.command.action==MissionAction::kNone);
    assert(result.mission.ball.mode==BallMode::kVerifyPickupObservation);
    assert(result.mission.command.action_id==0);
  }
};

void TestNormalBallAssociation() {
  BallPipeline pipeline;
  for (int i=0;i<18;++i) pipeline.Tick({BallDetection(.2,.3)});
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
  for (int i=0;i<9;++i) {
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

void TestPauseDoesNotAcquireNewMission() {
  auto config=BallPipelineConfig();
  config.ball.entry_min_hits=1; config.ball.down_search_min_hits=1; config.ball.stable_window=1;
  config.ball.stable_min_hits=1;
  config.line.line_stable_window=1;
  config.line.line_stable_min_hits=1;
  MissionController controller(config);
  PerceptionFrameInput input;
  input.image_width=640;
  input.image_height=480;
  input.command_transport_enabled=true;
  input.allow_new_commands=false;
  input.allow_new_mission_entry=false;
  input.detections={BallDetection(.50,.30)};
  input.now_sec=0.1;
  const auto frozen=controller.StepPerception(input);
  assert(frozen.mission.active_mission==MissionType::kLine);
  assert(frozen.mission.command.action_id==0);
  input.now_sec=0.2;
  input.allow_new_commands=true;
  input.allow_new_mission_entry=true;
  auto active=controller.StepPerception(input);
  assert(active.mission.active_mission==MissionType::kLine);
  assert(active.mission.candidate_mission==MissionType::kBall);
  for(int i=0;i<12;++i) { input.now_sec=.3+.1*i; active=controller.StepPerception(input); }
  assert(active.mission.active_mission==MissionType::kBall);
}

void TestGlobalPauseGateForBallAndCamera() {
  BallPipeline pipeline;
  pipeline.input.allow_new_commands=false;
  // Even if the BALL FSM advances into APPROACH/WAIT_CAMERA, the global
  // pause gate must forbid both NEW action IDs and camera requests.
  for (int i=0;i<15;++i) {
    const auto &result=pipeline.Tick({BallDetection(.50,.80)});
    assert(result.mission.command.action_id==0);
    assert(result.mission.command.camera_request==CameraRequest::kNone);
  }
  // Resume: preserving mission phase must not permanently disable commands.
  pipeline.input.allow_new_commands=true;
  pipeline.Tick({BallDetection(.50,.80)});
  for(int i=0;i<12 && pipeline.result.mission.active_mission==MissionType::kLine;++i)
    pipeline.Tick({BallDetection(.50,.80)});
  const auto &resumed=pipeline.result;
  assert(resumed.mission.command.camera_request!=CameraRequest::kNone ||
         resumed.mission.command.action_id!=0 ||
         resumed.mission.ball.mode!=BallMode::kLineFollow);
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
  assert(pipeline.result.mission.command.action==MissionAction::kDefaultPosition);
}
void TestBackboardValidDepthCanBeRetuned() {
  auto c=LoadDefaultAlgorithmConfig();
  c.enable_ball=c.enable_hurdle=c.enable_goal=false;
  MissionController controller(c);
  PerceptionFrameInput input;
  input.image_width=640; input.image_height=480;
  input.intrinsics={600,600,320,240};
  PerceptionDetection board;
  board.detection.class_id=c.object_targets.backboard_class_id;
  board.detection.confidence=1.0;
  board.detection.box={300.0,200.0,40.0,40.0};
  board.center_depth_m=3.0;
  input.detections={board};
  const auto rejected=controller.StepPerception(input);
  assert(!rejected.perception.targets.backboard); // Initially capped at 2.5 m.
  c.backboard_max_depth_m=4.0;
  assert(controller.UpdateTuning(c));
  const auto accepted=controller.StepPerception(input);
  assert(accepted.perception.targets.backboard);
  c.backboard_min_depth_m=3.5;
  assert(controller.UpdateTuning(c));
  const auto rejected_again=controller.StepPerception(input);
  assert(!rejected_again.perception.targets.backboard);
}

void TestDebugCandidatesUseCoreFiltersBeforeTemporalVoting() {
  auto config=LoadDefaultAlgorithmConfig();
  config.object_targets.ball_min_box_width=20;
  config.object_targets.ball_min_box_height=30;
  config.object_targets.hurdle_min_box_width=200;
  config.object_targets.hurdle_min_box_height=10;
  config.object_targets.ball_forward_min_depth_m=.5;
  config.object_targets.ball_forward_max_depth_m=2.2;
  MissionController controller(config);
  PerceptionFrameInput input;
  input.image_width=640; input.image_height=480;
  input.camera_feedback={CameraMode::kForward,true};
  const auto add=[&](int class_id, Box2 box, double confidence, double depth) {
    PerceptionDetection detection;
    detection.detection={box,confidence,class_id};
    detection.center_depth_m=depth;
    input.detections.push_back(detection);
  };
  const int ball=config.object_targets.ball_class_id;
  const int hurdle=config.object_targets.hurdle_class_id;
  add(ball,{100,200,40,40},.95,1.0);  // Filter pass, before 6/10.
  add(ball,{150,200,40,40},.95,3.0);  // Depth reject.
  add(ball,{200,200,80,30},.95,1.0);  // Aspect reject.
  add(ball,{250,200,10,10},.95,1.0);  // BBox reject.
  add(hurdle,{300,250,220,20},.95,1.0); // Filter pass.
  const auto result=controller.StepPerception(input);
  const auto &accepted=result.perception.accepted_object_detections;
  assert(accepted.size()==2);
  assert(accepted[0].class_id==ball && accepted[0].box.x==100);
  assert(accepted[1].class_id==hurdle && accepted[1].box.x==300);
  assert(result.perception.accepted_object_indices==std::vector<std::size_t>({0,4}));
  assert(result.perception.detection_track_ids.size()==5);
  assert(result.perception.detection_track_ids[0] && result.perception.detection_track_ids[4]);
  for(int index:{1,2,3})assert(result.perception.detection_track_ids[index]==0);
  assert(result.mission.active_mission==MissionType::kLine);
}

} // namespace

int main() {
  TestSharedLineRansacAfterRectification();
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
  assert(out.mission.command.action==MissionAction::kLineStraightUnit);
  TestNormalBallAssociation();
  TestVerificationBypassesAssociationAndFineReacquires();
  TestVerificationSixHitsSucceeds();
  TestGlobalPauseGateForBallAndCamera();
  TestPauseDoesNotAcquireNewMission();
  TestBackboardValidDepthCanBeRetuned();
  TestDebugCandidatesUseCoreFiltersBeforeTemporalVoting();
  return 0;
}
