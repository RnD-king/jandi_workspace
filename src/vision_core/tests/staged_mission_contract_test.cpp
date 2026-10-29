#include "vision_core/config_loader.hpp"
#include "vision_core/object_observation_window.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <stdexcept>
using namespace vision_core;
static CruiseDecision StraightLine() { CruiseDecision d; d.applicable=true; d.direction=CruiseDirection::kStraight; return d; }
namespace {
ObjectTarget Target(double u=320,double v=336) {
  ObjectTarget t; t.center_px={u,v};t.box_px={u-10,v-10,20,20};
  t.width_px=t.height_px=20;t.area_px=400;t.confidence=1;return t;
}
MissionControllerConfig Config(MissionType mission,int stage) {
  auto c=LoadDefaultAlgorithmConfig(); c.transition_settle_sec=0;c.test_mission=mission;c.test_stage=stage;
  c.enable_ball=mission==MissionType::kBall;c.enable_goal=mission==MissionType::kGoal;
  c.enable_hurdle=mission==MissionType::kHurdle;

  // Preserve the pre-tuning test geometry for pickup/contact assertions.
  c.ball.pickup_u1_px=281.6; c.ball.pickup_u2_px=358.4;
  c.ball.pickup_v1_px=307.2; c.ball.pickup_v2_px=364.8;
  c.goal.throw_landing_z_m=.48; // Explicit legacy geometry for these mission scenarios.
  c.hurdle.contact_steps=3; c.hurdle.contact_v_px=408.0;
  return c;
}
struct Trial {
  MissionController core;MissionFrameInput in;MissionFrameResult out;
  Trial(MissionType mission,int stage):core(Config(mission,stage)) {
    in.image_width=640;in.image_height=480;in.manual_observation_control=true;
    in.command_transport_enabled=true;in.allow_new_body_actions=false;in.advance_line_fsm=false;
  }
  void Tick() {in.now_sec+=.12;out=core.Step(in);in.delivery_feedback={};}
  void TargetNow(MissionType mission) {
    if(mission==MissionType::kBall)in.ball_target=Target();
    else if(mission==MissionType::kHurdle)in.hurdle_target=Target(320,420);
    else {in.backboard_target=Target();in.backboard_center_depth_m=.6;in.goal_pose={true,0,.6,0,1};}
  }
  MissionFrameResult Preview() const {auto trial=core;auto input=in;input.allow_new_body_actions=true;input.delivery_feedback={};return trial.Step(input);}
  void Commit(MissionAction action) {
    const auto candidate=Preview();assert(candidate.command.action==action);
    assert(candidate.command.command_type==CommandType::kAction);
    const auto old_attempt=out.ball.pickup_attempt_count;
    auto input=in;input.allow_new_body_actions=true;input.delivery_feedback={};
    out=core.Step(input);assert(out.command.action_id==candidate.command.action_id);
    assert(out.command.action==action);
    if(action==MissionAction::kPickBall)assert(out.ball.pickup_attempt_count==old_attempt+1);
  }
  void Done() {in.delivery_feedback={out.command.action_id,true,true,false};Tick();}
  std::uint64_t PrepareStage(MissionType mission, int stage) {
    const CameraMode camera=stage==0 ||
        (stage==1 && mission!=MissionType::kGoal)
        ? CameraMode::kForward
        : (mission==MissionType::kGoal ? CameraMode::kGoal
                                       : CameraMode::kDown);
    in.camera_feedback={camera,true};
    if(mission==MissionType::kHurdle && stage<=1)
      in.line_centers={{410,450},{410,400},{410,350},{410,300},{410,250}};
    in.allow_test_setup_body_actions=true;
    Tick();
    const MissionAction required=stage==3 ? MissionAction::kDefaultPoseMode
                                          : MissionAction::kWalkMode;
    assert(out.command.action==required);
    assert(out.command.mission_phase==903);
    const std::uint64_t setup_id=out.command.action_id;
    assert(setup_id!=0);
    in.delivery_feedback={setup_id,true,true,false};
    Tick();
    assert(!core.TestSetupPending());
    assert(out.command.action_id==0);
    in.allow_test_setup_body_actions=false;
    return setup_id;
  }
};
// Stage START always executes its own stance before normal preview/COMMIT.
void FarStageStopsAfterCamera(MissionType mission) {
  Trial t(mission,1);
  t.TargetNow(mission);
  if(mission==MissionType::kBall)
    t.in.ball_target=Target(380,400); // grid zone 8, not left near turn
  if(mission==MissionType::kHurdle)
    t.in.hurdle_target=Target(320,400); // grid zone 8
  if(mission==MissionType::kGoal) {
    t.in.backboard_center_depth_m=1.5;
    t.in.goal_pose={};
  }
  t.PrepareStage(mission,1);
  assert(t.out.foot_pose==FootPose::kWalk);
  if(mission!=MissionType::kGoal) {
    for(int n=0;n<12 && t.Preview().command.action!=MissionAction::kStepForwardTwo;++n)
      t.Tick();
    t.Commit(MissionAction::kStepForwardTwo);
    const auto id=t.out.command.action_id;
    t.in.delivery_feedback={id,true,false,false};t.Tick();
    assert(!t.out.trial_complete); // ACK alone cannot finish
    t.in.delivery_feedback={id,true,true,false};t.Tick();
    assert(t.out.command.camera_request==CameraRequest::kDown);
    t.in.camera_feedback={CameraMode::kDown,true};
    for(int n=0;n<10 && !t.out.trial_complete;++n)t.Tick();
    assert(t.out.trial_complete);
  } else {
    t.Tick(); // GOAL camera known and settled, enter search
    for(int n=0;n<12 && t.out.goal.mode!=GoalMode::kApproach;++n)t.Tick();
    assert(t.out.goal.mode==GoalMode::kApproach);
    assert(t.Preview().command.action==MissionAction::kStepForwardThree);
    t.in.backboard_center_depth_m=.7;
    for(int n=0;n<10 && !t.out.trial_complete;++n)t.Tick();
    assert(t.out.trial_complete);
  }
  assert(t.out.command.action_id==0);
}
void GoalFarStageAlreadyInGoalCamera() {
  Trial t(MissionType::kGoal,1);
  t.TargetNow(MissionType::kGoal);
  t.in.backboard_center_depth_m=1.5;
  t.in.goal_pose={};
  t.in.camera_feedback={CameraMode::kGoal,true};
  // Even when GOAL camera and WALK stance were previously reported,
  // START must send its own fresh WALK command.
  const auto id=t.PrepareStage(MissionType::kGoal,1);
  assert(id!=0 && t.out.command.action_id==0);
  t.Tick();
  assert(t.out.goal.mode==GoalMode::kSearch);
}
void NearStageStopsBeforeMission(MissionType mission) {
  Trial t(mission,2);t.TargetNow(mission);
  const auto prepare_id=t.PrepareStage(mission,2);
  assert(prepare_id!=0 && t.out.foot_pose==FootPose::kWalk);
  for(int i=0;i<20 && t.Preview().command.action!=MissionAction::kDefaultPoseMode;++i)t.Tick();
  t.Commit(MissionAction::kDefaultPoseMode);t.Done();
  assert(t.out.foot_pose==FootPose::kFine);
  t.in.allow_new_body_actions=true;
  for(int i=0;i<20 && !t.out.trial_complete;++i)t.Tick();
  assert(t.out.trial_complete && t.out.command.action_id==0);
  assert(t.out.ball.pickup_attempt_count==0);
  // Stage 3 must command its FINE initial stance again after Stage 2.
  t.in.allow_new_body_actions=false;
  assert(t.core.SetTestStage(mission,3));
  const auto setup_id=t.PrepareStage(mission,3);
  assert(setup_id>prepare_id && t.out.foot_pose==FootPose::kFine);
  for(int i=0;i<12;++i)t.Tick();
  assert(!t.out.trial_complete);
  t.Commit(mission==MissionType::kBall ? MissionAction::kPickBall :
      mission==MissionType::kGoal ? MissionAction::kShoot : MissionAction::kStepForwardHalf);
}
void ExecuteStageNeedsRealPoseAndStableReadiness(MissionType mission) {
  Trial t(mission,3);t.TargetNow(mission);
  const auto id=t.PrepareStage(mission,3);
  assert(t.out.foot_pose==FootPose::kFine && id!=0);
  for(int i=0;i<9;++i){t.Tick();assert(!t.out.test_ready && t.out.command.action_id==0);}
  t.Tick();assert(t.out.test_ready && !t.out.trial_complete);
  auto outside=t.core;auto input=t.in;input.allow_new_body_actions=false;
  if(mission==MissionType::kBall)input.ball_target=Target(100,336);
  else if(mission==MissionType::kHurdle)input.hurdle_target=Target(320,350);
  else input.goal_pose.x_m=1.0;
  MissionFrameResult held;
  for(int i=0;i<20;++i){
    input.now_sec+=.12;held=outside.Step(input);assert(held.command.action_id==0);
  }
  input.allow_new_body_actions=true;input.now_sec+=.12;held=outside.Step(input);
  assert(held.command.action_id==0);
  assert(!held.test_ready && !held.test_executing);
  t.Commit(mission==MissionType::kBall ? MissionAction::kPickBall :
      mission==MissionType::kGoal ? MissionAction::kShoot : MissionAction::kStepForwardHalf);
  assert(t.out.command.action_id>id);
}
void WeightedCoordinatesUseHistory() {
  ObjectObservationWindow w;
  for(int i=0;i<7;++i)w.Add(Target(320,144),10);
  w.Add(Target(320,384),10);
  assert(w.Mean()->center_px.v>144 && w.Mean()->center_px.v<360);
  assert(std::abs(w.Mean(false)->center_px.v-174)<1e-9);
}
void FineAdjustmentUsesFreshEqualMeans() {
  auto c=LoadDefaultAlgorithmConfig(); c.transition_settle_sec=0;
  // This test isolates averaging behavior using its legacy synthetic
  // landing geometry; calibrated -8cm/48cm geometry has a dedicated test.
  c.goal.throw_landing_x_m=0.0;
  c.goal.throw_landing_z_m=0.4;
  c.ball.pickup_u1_px=281.6; c.ball.pickup_u2_px=358.4;
  c.ball.pickup_v1_px=307.2; c.ball.pickup_v2_px=364.8;
  BallController ball(c.ball,3);
  HurdleController hurdle(c.hurdle,3);
  GoalController goal(c.goal,3);
  ball.StartTestExecute(); hurdle.StartTestExecute(); goal.StartTestExecute();
  BallResult b; HurdleResult h; GoalResult g;
  const CameraFeedback down{CameraMode::kDown,true};
  const CameraFeedback goal_camera{CameraMode::kGoal,true};
  for(int i=0;i<10;++i) {
    const bool recent=i>=7;
    const double now=.12*(i+1);
    b=ball.Compute(Target(recent ? 380 : 280,336),640,480,now,down,{},false);
    h=hurdle.Compute(Target(320,recent ? 450 : 390),640,480,now,down,{},StraightLine(),false);
    g=goal.Compute({},Target(),{true,0,recent ? .6 : .4,0,1},
                   640,480,now,false,goal_camera,{},false);
  }
  assert(std::abs(b.tracked.center_px.u-310)<1e-9);
  assert(std::abs(h.tracked.center_px.v-408)<1e-9);
  assert(g.fine_decision_pose_used && std::abs(g.fine_decision_pose.z_m-.46)<1e-9);
  assert(b.action.action==MissionAction::kPickBall);
  assert(h.action.action==MissionAction::kStepForwardHalf);
  assert(g.action.action==MissionAction::kShoot);
  for(int i=0;i<10;++i)
    g=goal.Compute({},Target(),{true,0,.5,i<5 ? 3.12 : -3.12,1},
                   640,480,1.21+.005*i,false,goal_camera,{},false);
  assert(g.fine_decision_pose_used &&
         std::abs(std::abs(g.fine_decision_pose.yaw_rad)-3.141592653589793)<.03);
  goal.UseNextObservationAsMean();
  g=goal.Compute({},Target(),{true,0,.5,0,1},640,480,1.26,false,
                 goal_camera,{},false);
  assert(g.action.action==MissionAction::kShoot);
  g=goal.Compute({},Target(),{true,0,.8,0,1},640,480,1.27,false,
                 goal_camera,{},false);
  assert(g.action.action==MissionAction::kShoot &&
         std::abs(g.fine_decision_pose.z_m-.5)<1e-9);

  ActionExecutionFeedback done; done.action_done=true;
  ball.Compute(Target(600,336),640,480,1.3,down,done,false);
  hurdle.Compute(Target(320,470),640,480,1.3,down,done,StraightLine(),false);
  goal.Compute({},Target(),{true,0,1.0,0,1},640,480,1.3,false,
               goal_camera,done,false);
  b=ball.Compute(Target(300,336),640,480,1.4,down,{},false);
  h=hurdle.Compute(Target(320,420),640,480,1.4,down,{},StraightLine(),false);
  g=goal.Compute({},Target(),{true,0,.5,0,1},640,480,1.4,false,
                 goal_camera,{},false);
  assert(b.action.action==MissionAction::kNone);
  assert(h.action.action==MissionAction::kNone);
  assert(g.action.action==MissionAction::kNone);
}

void ExecuteStageCompletesRecovery(MissionType mission) {
  Trial t(mission,3);
  t.TargetNow(mission);
  t.in.allow_new_body_actions=true;
  t.in.camera_feedback={mission==MissionType::kGoal
      ? CameraMode::kGoal : CameraMode::kDown,true};
  std::uint64_t delivered=0;
  int picked=0, recaught=0, stood=0, shot=0, backed=0;
  int contacted=0, crossed=0, turned=0, walk_pose=0;
  bool forward_camera=false;
  for(int i=0;i<400 && !t.out.trial_complete;++i) {
    t.in.delivery_feedback={};
    const auto command=t.out.command;
    if(command.action_id && command.action_id!=delivered) {
      delivered=command.action_id;
      t.in.delivery_feedback={delivered,true,true,false};
      switch(command.action) {
      case MissionAction::kPickBall:
        ++picked; t.in.ball_target.reset(); break;
      case MissionAction::kRecatch: ++recaught; break;
      case MissionAction::kDefaultPosition: ++stood; break;
      case MissionAction::kShoot: ++shot; break;
      case MissionAction::kStepBack: ++backed; break;
      case MissionAction::kStepForwardHalf: ++contacted; break;
      case MissionAction::kHurdle: ++crossed; break;
      case MissionAction::kTurnLeft:
      case MissionAction::kTurnRight:
        ++turned;
        t.in.line_centers={{420,450},{420,350},{420,250},{420,150}};
        break;
      case MissionAction::kWalkMode: ++walk_pose; break;
      default: break;
      }
    }
    t.Tick();
    if(t.out.command.camera_request==CameraRequest::kForward) {
      forward_camera=true;
      t.in.camera_feedback={CameraMode::kForward,true};
    }
  }
  assert(t.out.trial_complete && t.out.active_mission==MissionType::kLine);
  assert(t.out.foot_pose==FootPose::kWalk && t.out.command.action_id==0);
  assert(walk_pose==1 && forward_camera);
  if(mission==MissionType::kBall)
    assert(picked==1 && recaught==0 && stood==1 && backed==0 && turned>=1);
  else if(mission==MissionType::kGoal)
    assert(shot==1 && backed==0 && turned>=1);
  else
    assert(contacted==3 && crossed==1 && backed==0 && turned==0);
}
void RecoveryUsesCompleteWindow() {
  auto c=LoadDefaultAlgorithmConfig(); c.transition_settle_sec=0;c.ball.lost_frames=c.hurdle.lost_frames=1;
  c.ball.recovery_max_steps=0; // Isolate stationary-window validation from motion search.
  BallController ball(c.ball,3);HurdleController hurdle(c.hurdle,3);
  BallResult b;HurdleResult h;double now=0;
  for(int i=0;i<7;++i){now+=.12;b=ball.Compute(Target(320,144),640,480,now,{}, {},false);h=hurdle.Compute(Target(320,200),640,480,now,{}, {},StraightLine(),false);}
  // Establish a successful first observation; this case isolates later stationary recovery.
  now+=.12;
  b=ball.Compute(Target(320,144),640,480,now,{}, {false,false,true},false);
  h=hurdle.Compute(Target(320,200),640,480,now,{}, {false,false,true},StraightLine(),false);
  now+=.12;b=ball.Compute({},640,480,now,{}, {},false);h=hurdle.Compute({},640,480,now,{}, {},StraightLine(),false);
  assert(b.mode==BallMode::kBallRecoveryForward && h.mode==HurdleMode::kRecoveryForward);
  for(int i=0;i<10;++i) {
    now+=.12;const auto target=i==3 || i==4 || i==5 ? std::optional<ObjectTarget>{} : std::optional<ObjectTarget>{Target(320,200)};
    b=ball.Compute(target,640,480,now,{}, {},false);h=hurdle.Compute(target,640,480,now,{}, {},StraightLine(),false);
    assert(b.action.action==MissionAction::kNone && h.action.action==MissionAction::kNone);
    if(i<9)assert(b.mode==BallMode::kBallRecoveryForward && h.mode==HurdleMode::kRecoveryForward);
  }
  assert(b.mode==BallMode::kApproachBall && h.mode==HurdleMode::kApproach);
  assert(ball.RecoveryObservationsCompleted()==1 && hurdle.RecoveryObservationsCompleted()==1);
}
void ManualCoreKeepsRecoveryLive() {
  auto c=Config(MissionType::kBall,0);MissionController core(c);MissionFrameInput in;
  in.image_width=640;in.image_height=480;in.command_transport_enabled=true;
  in.manual_observation_control=true;in.allow_new_body_actions=false;
  in.allow_test_setup_body_actions=true;
  MissionFrameResult out=core.Step(in);
  const auto setup_id=out.command.action_id;
  assert(setup_id!=0 && out.command.action==MissionAction::kWalkMode);
  in.delivery_feedback={setup_id,true,true,false};
  out=core.Step(in);
  assert(!core.TestSetupPending());
  in.delivery_feedback={};in.allow_test_setup_body_actions=false;
  for(int i=0;i<7;++i){in.now_sec+=.12;in.ball_target=Target(320,144);out=core.Step(in);}
  for(int i=0;i<5;++i){in.now_sec+=.12;in.ball_target.reset();out=core.Step(in);}
  assert(core.NeedsStationaryObservation());
  for(int i=0;i<10;++i){in.now_sec+=.12;in.ball_target=i==3 || i==4 || i==5 ? std::optional<ObjectTarget>{} : std::optional<ObjectTarget>{Target(320,200)};out=core.Step(in);assert(out.command.action_id==0);}
  assert(out.ball.mode==BallMode::kApproachBall && !core.NeedsStationaryObservation());
}
void NearGoalWithoutPoseCannotStride() {
  auto c=LoadDefaultAlgorithmConfig().goal;c.stable_window=10;c.stable_min_hits=7;
  GoalController goal(c,3);goal.StartTestNear();CameraFeedback camera{CameraMode::kGoal,true};
  goal.Compute({}, {}, {},640,480,0,true,camera,{});
  GoalResult result;
  for(int i=0;i<8;++i) result=goal.Compute({},Target(),{},640,480,.12*(i+1),true,camera,{},true,.5);
  assert(result.mode==GoalMode::kWaitFinePose);
  assert(result.action.action==MissionAction::kDefaultPoseMode); // Pose first, never a WALK stride.
}
void AutoPoseDonePrecedesFreshObservation() {
  auto c=Config(MissionType::kBall,2);MissionController core(c);MissionFrameInput in;
  in.image_width=640;in.image_height=480;in.command_transport_enabled=true;
  in.camera_feedback={CameraMode::kDown,true};in.ball_target=Target();
  MissionFrameResult out;bool reached_pose=false;
  for(int i=0;i<100;++i) {
    in.now_sec+=.12;in.delivery_feedback={};
    if(out.command.action_id) {
      const auto id=out.command.action_id;
      core.AcceptDeliveryFeedback({id,true,true,false},in.now_sec);
      in.delivery_feedback={id,true,true,false};
    }
    const bool was_pose=out.command.action==MissionAction::kDefaultPoseMode;
    out=core.Step(in);
    if(was_pose) {
      assert(out.ball.mode==BallMode::kFineAdjustForPickup);
      assert(!out.post_motion_observing && out.foot_pose==FootPose::kFine);
      reached_pose=true;break;
    }
  }
  assert(reached_pose);
  in.delivery_feedback={};in.now_sec+=.12;out=core.Step(in);
  assert(out.post_motion_observing && out.command.action_id==0);
}

void RejectZeroObservation() {
  auto c=LoadDefaultAlgorithmConfig(); c.transition_settle_sec=0;c.post_motion_observation_sec=0;bool rejected=false;
  try{ValidateAlgorithmConfig(c);}catch(const std::runtime_error&){rejected=true;}
  assert(rejected);
}
}
int main() {
  for(auto mission:{MissionType::kBall,MissionType::kHurdle,MissionType::kGoal}) {
    FarStageStopsAfterCamera(mission);NearStageStopsBeforeMission(mission);
    ExecuteStageNeedsRealPoseAndStableReadiness(mission);
    ExecuteStageCompletesRecovery(mission);
  }
  GoalFarStageAlreadyInGoalCamera();
  WeightedCoordinatesUseHistory();FineAdjustmentUsesFreshEqualMeans();
  RecoveryUsesCompleteWindow();ManualCoreKeepsRecoveryLive();
  NearGoalWithoutPoseCannotStride();AutoPoseDonePrecedesFreshObservation();RejectZeroObservation();
}
