#include "vision_core/config_loader.hpp"
#include "vision_core/mission_controller.hpp"
#include <cmath>
#include <array>
#include <utility>
#include <stdexcept>
#include <limits>

using namespace vision_core;
namespace {
void Check(bool value,const char* message) { if(!value) throw std::runtime_error(message); }
ObjectTarget Target(double u=320,double v=100,int cls=1) {
  ObjectTarget t;t.class_id=cls;t.confidence=.95;t.center_px={u,v};
  t.box_px={u-20,v-20,40,40};t.width_px=t.height_px=40;t.area_px=1600;return t;
}
MissionControllerConfig Config() {
  auto c=LoadDefaultAlgorithmConfig();
  c.enable_ball=true;c.enable_goal=c.enable_hurdle=false;
  c.initial_has_ball=c.goal_trial_start_goal=false;
  c.test_mission=MissionType::kNone;
  c.line_features.image_center_u=320;
  c.goal.auto_approach_center_u_px=280; // Dedicated GOAL vertical zone center.
  c.line_p2p.left_repetitions=c.line_p2p.straight_repetitions=c.line_p2p.right_repetitions=2;
  c.ball.entry_min_hits=4;c.ball.stable_window=10;c.ball.stable_min_hits=7;
  c.ball.approach_short_v_px=100;c.ball.approach_two_step_v_px=240;c.ball.approach_direct_down_v_px=355;
  c.hurdle.approach_short_v_px=100;c.hurdle.approach_two_step_v_px=240;
  c.object_association.max_center_distance_norm=.5;
  // This fixture changes the BALL center from v=100 to v=260 in one update.
  // Its test YAML intentionally has an 80px candidate gate; give this
  // direct-controller fixture the same wider gate the old association used.
  c.ball_candidate_tracking.max_center_distance_px=400.0;
  c.ball_candidate_tracking.max_size_log_ratio=c.object_association.max_size_log_ratio;
  c.object_approach.lock_blocks=3;
  return c;
}
struct Driver {
  MissionController core;
  MissionFrameInput in;
  MissionFrameResult out;
  std::uint64_t current{}, last_done{};
  explicit Driver(const MissionControllerConfig& c):core(c) {
    in.image_width=640;in.image_height=480;
    in.camera_feedback={CameraMode::kForward,true};in.command_transport_enabled=true;
    in.line_centers={{320,450},{320,400},{320,350},{320,300},{320,250}};
    in.ball_target=Target();in.ball_track_id=41;
  }
  void Tick(CommandDeliveryFeedback fb={},double dt=.05) {
    in.now_sec+=dt;in.delivery_feedback=fb;out=core.Step(in);
  }
  void Warm() {
    Tick();current=out.command.action_id;
    Check(current!=0,"initial LINE unit missing");
    Tick({current,true,false,false});
    for(int i=0;i<12;++i) Tick();
  }
  void Block() {
    for(int n=0;n<2;++n) {
      Tick({current,true,false,false});
      for(int i=0;i<12;++i) Tick();
      Tick({current,true,false,true});
      const auto next=out.command.action_id;
      Check(next && next!=current && LinePlan::IsUnit(out.command.action),
            "continuous READY did not reserve the next shared unit");
      Tick({next,true,false,false});
      last_done=current;
      Tick({current,true,true,false});current=next;
    }
  }
  void MissingBlock() {
    in.ball_target.reset();
    for(int i=0;i<12;++i) Tick();
    Check(out.approach_mission==MissionType::kBall,
          "one missing frame released an object before block READY");
    Tick({current,true,false,true});
    const auto second=out.command.action_id;
    Check(second && second!=current && LinePlan::IsUnit(out.command.action),
          "missing first unit incorrectly cancelled the rest of its 2-unit block");
    Tick({second,true,false,false});
    Tick({current,true,true,false});current=second;
    for(int i=0;i<12;++i) Tick();
    Tick({current,true,false,true});
    // An unqualified BALL block may now reserve exactly one remembered-direction
    // pre-lock probe at final READY. Qualification must remain false.
    Check(!out.approach_sequence_qualified,
          "unqualified object block was incorrectly marked qualified");
    Tick({current,true,true,false});
  }
};
void Geometry() {
  const Intrinsics k{600,600,320,240};const ApproachCameraPose c{0,.45,0,20,0};
  auto t=Target(320,150);
  auto guide=ObjectApproachGuide(t,false,0,k,c,640,480,320,400,false,0,0);
  Check(guide.valid && std::abs(guide.offset)<1e-9 && std::abs(guide.heading_rad)<1e-9,
        "centered floor path should be straight");
  t=Target(430,150);
  auto right=ObjectApproachGuide(t,false,0,k,c,640,480,320,400,false,0,0);
  t=Target(210,150);
  auto left=ObjectApproachGuide(t,false,0,k,c,640,480,320,400,false,0,0);
  Check(right.valid && left.valid && right.offset>0 && right.heading_rad>0 &&
        left.offset<0 && left.heading_rad<0,"virtual path O/H signs differ from LINE");
  auto shifted=c;shifted.right_m=.1;
  const auto offset=ObjectApproachGuide(Target(320,150),false,0,k,shifted,640,480,320,400,false,0,0);
  Check(offset.valid && std::abs(offset.offset-guide.offset)<1e-9,
        "BALL RGB path unexpectedly depended on unmeasured camera offset");
  Check(!ObjectApproachGuide(Target(),true,0,k,c,640,480,320,400,false,0,0).valid,
        "missing board depth generated a valid path");
  auto bad=k;bad.fx=0;
  Check(ObjectApproachGuide(Target(),false,0,bad,c,640,480,320,400,false,0,0).valid,
        "BALL RGB path unexpectedly required depth intrinsics");
  const auto goal=ObjectApproachGuide(Target(),true,2,k,{0,.45,0,0,0},640,480,320,400,false,0,0);
  Check(goal.valid && std::abs(goal.offset)<1e-9,"GOAL view could not project an elevated target onto the floor");
  double bearing=0;
  const auto shifted_goal=ObjectApproachGuide(Target(),true,2,k,{-.017,.45,0,0,0},
      640,480,320,400,false,0,0,&bearing);
  Check(shifted_goal.valid && std::abs(bearing-std::atan2(-.017,2.0))<1e-9,
        "GOAL body bearing ignored the measured camera lateral offset");
}
void VirtualHeadingOnly() {
  auto c=Config();
  Driver ball(c);ball.in.ball_target=Target(500,100);
  ball.Warm();
  Check(!ball.out.approach_path_guide,
        "virtual path appeared before an object-owned approach unit was issued");
  Driver queued_line(c);queued_line.in.ball_target=Target(500,100);
  queued_line.Warm();
  queued_line.Tick({queued_line.current,true,false,true});
  Check(queued_line.out.command.action_id!=queued_line.current &&
        LinePlan::IsUnit(queued_line.out.command.action) &&
        !queued_line.out.approach_path_guide,
        "an already queued LINE unit appeared as an object approach path");
  ball.Block();
  Check(ball.out.approach_mission==MissionType::kBall &&
        ball.out.approach_path_guide && ball.out.approach_path_guide->offset>0 &&
        ball.out.command.action==MissionAction::kLineRightUnit,
        "BALL virtual offset still changed the H-only approach direction");
  const double ball_heading=ball.out.approach_path_guide->heading_rad;
  auto opposite_config=c;
  Driver opposite(opposite_config);
  opposite.in.line_centers={{400,450},{400,400},{400,350},{400,300},{400,250}};
  opposite.in.ball_target=Target(180,100);
  opposite.Warm();opposite.Block();
  Check(opposite.out.command.action==MissionAction::kLineLeftUnit &&
        opposite.out.approach_path_guide &&
        opposite.out.approach_path_guide->heading_rad<0,
        "first BALL approach unit inherited the previous real LINE mean");
  ball.MissingBlock();
  // The new first-LINE admission preserves a qualified, weighted BALL
  // coordinate at READY. After one more unqualified physical block, the
  // existing bounded pre-lock probe must use that memory instead of stopping.
  Check(ball.out.approach_mission==MissionType::kBall &&
        ball.out.approach_probe_active &&
        LinePlan::IsUnit(ball.out.command.action),
        "qualified first BALL READY did not start its one bounded pre-lock probe");

  c=Config();c.enable_ball=false;c.enable_goal=true;c.initial_has_ball=true;
  c.goal_trial_start_goal=true;
  Driver goal(c);goal.in.ball_target.reset();goal.in.line_centers.clear();
  goal.in.camera_feedback={CameraMode::kGoal,true};
  goal.in.backboard_target=Target(500,100);goal.in.backboard_track_id=7;
  goal.in.backboard_center_depth_m=2;goal.in.goal_pose={true,.6,2,0,1};
  for(int i=0;i<12;++i)goal.Tick();
  Check(goal.out.approach_mission==MissionType::kGoal &&
        goal.out.approach_path_guide && goal.out.line_decision_guide &&
        std::abs(goal.out.approach_path_guide->heading_rad-
                 goal.out.line_decision_guide->heading_rad)<1e-9 &&
        std::abs(goal.out.approach_path_guide->heading_rad-
                 ball_heading)<1e-9 &&
        goal.out.line_decision_guide->heading_rad>0.25 &&
        goal.out.command.action==MissionAction::kLineRightUnit,
        "GOAL did not steer from the dedicated vertical pixel zone");

  c.object_approach.steering_deadband_deg=30.0;
  Driver tuned(c);tuned.in.ball_target.reset();tuned.in.line_centers.clear();
  tuned.in.camera_feedback={CameraMode::kGoal,true};
  tuned.in.backboard_target=Target(340,100);tuned.in.backboard_track_id=7;
  tuned.in.backboard_center_depth_m=2;tuned.in.goal_pose={true,.6,2,0,1};
  for(int i=0;i<12;++i)tuned.Tick();
  Check(tuned.out.command.action==MissionAction::kLineStraightUnit,
        "GOAL center zone did not command straight walking");
}
void WeightedProbeDirection() {
  auto c=Config();
  c.object_approach.steering_deadband_deg=2.0;
  c.object_approach.in_place_turn_deg=100.0;
  Driver d(c);d.in.ball_target=Target(180,100);
  d.Warm();d.Block(); // Drain the original LINE block.
  d.Block();          // Finish a left-heading object block and cache its mean.
  d.in.ball_target=Target(460,100);
  d.Tick(); // Last valid frame now points right; this block remains unqualified.
  d.MissingBlock();             // Current object block never qualifies.
  if (d.out.command.action!=MissionAction::kLineLeftUnit ||
      d.out.approach_completed_blocks!=2)
    throw std::runtime_error("pre-lock probe mean wrong: action="+
        std::to_string(static_cast<int>(d.out.command.action))+
        " blocks="+std::to_string(d.out.approach_completed_blocks)+
        " mission="+std::to_string(static_cast<int>(d.out.approach_mission)));
}
void FinishProbe(Driver& d) {
  d.current=d.out.command.action_id;
  Check(d.current && LinePlan::IsUnit(d.out.command.action),
        "probe fixture did not start an ordinary approach block");
  d.Tick({d.current,true,false,false});
  for(int i=0;i<12;++i)d.Tick();
  d.Tick({d.current,true,false,true});
  const auto second=d.out.command.action_id;
  if (!(second && second!=d.current && LinePlan::IsUnit(d.out.command.action)))
    throw std::runtime_error("probe did not retain the configured second unit: action="+
        std::to_string(static_cast<int>(d.out.command.action))+" id="+
        std::to_string(second)+" current="+std::to_string(d.current));
  d.Tick({second,true,false,false});
  d.Tick({d.current,true,true,false});d.current=second;
  for(int i=0;i<12;++i)d.Tick();
  d.Tick({d.current,true,false,true});
  const bool return_to_line=d.out.approach_mission==MissionType::kBall &&
      d.out.remembered_line_score.has_value();
  if(return_to_line) {
    // BALL's failed recovery unit must reserve exactly one FLOOR-LINE unit
    // at READY; the successor is not allowed to count toward BALL lock.
    const auto next=d.out.command.action_id;
    Check(next && next!=d.current && LinePlan::IsUnit(d.out.command.action),
          "BALL probe failed to reserve remembered floor LINE at READY");
    d.Tick({next,true,false,false}); // Queue ACK, not physical motion DONE.
  } else {
    Check(d.out.command.action_id==d.current,
          "HURDLE/GOAL probe unexpectedly reserved a successor at READY");
  }
  d.Tick({d.current,true,true,false});
}
void SoftLoss(int blocks) {
  Driver d(Config());d.Warm();
  d.Block(); // The physical first LINE block now counts as qualified BALL 1/3.
  Check(d.out.approach_completed_blocks==1,
        "qualified first LINE block was not counted as BALL 1/3");
  for(int i=1;i<blocks;++i)d.Block();
  const int expected=std::max(1,blocks);
  Check(d.out.approach_completed_blocks==expected && !d.out.approach_locked,
        "soft target locked before three actual completed blocks");
  const auto before=d.out.approach_completed_blocks;
  d.Tick({d.last_done,true,true,false});
  Check(d.out.approach_completed_blocks==before,
        "duplicate DONE counted another BALL block");
  d.MissingBlock();
  Check(d.out.active_mission==MissionType::kLine &&
        d.out.approach_mission==MissionType::kBall &&
        d.out.approach_completed_blocks==expected &&
        LinePlan::IsUnit(d.out.command.action),
        "pre-lock loss failed to issue one remembered READY probe");
  FinishProbe(d);
  Check(d.out.approach_mission==MissionType::kNone &&
        !d.out.candidate_phase,"failed pre-lock probe did not return to LINE");
  d.in.ball_target=Target();
  for(int i=0;i<12;++i)d.Tick();
  Check(LinePlan::IsUnit(d.out.command.action) &&
        d.out.approach_completed_blocks==0,
        "soft loss did not reconnect LINE without retaining old object lock");
}
void ProbeDisabled() {
  auto c=Config();c.object_approach.prelock_loss_extra_blocks=0;
  Driver d(c);d.Warm();d.Block();d.MissingBlock();
  Check(d.out.approach_mission==MissionType::kNone &&
        d.out.active_mission==MissionType::kLine,
        "zero pre-lock probe budget did not restore direct LINE release");
}
void ProbeRediscoveryAndLineMemory() {
  auto c=Config();
  Driver d(c);
  d.in.line_centers={{280,450},{280,400},{280,350},{280,300},{280,250}};
  d.in.ball_target=Target(480,100);
  for(int i=0;i<12;++i)d.Tick();
  const auto expected=LineP2pController(c.line_p2p).Compute(d.out.line_features.guide);
  Check(d.out.approach_mission==MissionType::kBall &&
        !d.out.approach_path_guide && d.out.remembered_line_score &&
        std::abs(*d.out.remembered_line_score-expected.error)<1e-9,
        "pre-commit BALL candidate changed real LINE memory or drew its path");
  const auto remembered=*d.out.remembered_line_score;
  d.in.line_centers.clear();
  for(int i=0;i<12;++i)d.Tick();
  Check(d.out.remembered_line_score &&
        std::abs(*d.out.remembered_line_score-remembered)<1e-9,
        "invalid actual LINE erased the far-approach directional memory");

  // The admission LINE block now counts toward the 3-block lock. Start with
  // one qualified block so a successful rediscovery still remains pre-lock.
  Driver recovered(Config());recovered.Warm();recovered.Block();
  // A successful object block establishes the READY mean needed for probing.
  recovered.MissingBlock();
  const int completed=recovered.out.approach_completed_blocks;
  recovered.current=recovered.out.command.action_id;
  Check(recovered.current && LinePlan::IsUnit(recovered.out.command.action),
        "rediscovery fixture did not start a probe");
  recovered.Tick({recovered.current,true,false,false});
  recovered.in.ball_target=Target();
  recovered.Tick();
  Check(recovered.out.approach_mission==MissionType::kBall &&
        recovered.out.approach_completed_blocks==completed &&
        recovered.out.approach_missing_frames==0,
        "rediscovery did not immediately clear loss while preserving lock blocks");
  for(int i=0;i<12;++i)recovered.Tick();
  recovered.Tick({recovered.current,true,false,true});
  const auto next=recovered.out.command.action_id;
  if (!(next && next!=recovered.current && LinePlan::IsUnit(recovered.out.command.action)))
    throw std::runtime_error("rediscovery did not replace an unissued probe repeat with normal approach: action="+
        std::to_string(static_cast<int>(recovered.out.command.action))+" id="+
        std::to_string(next)+" current="+std::to_string(recovered.current)+
        " line_recovery="+std::to_string(recovered.out.line_in_recovery)+
        " probe_active="+std::to_string(recovered.out.approach_probe_active)+
        " probe_used="+std::to_string(recovered.out.approach_probe_used)+
        " loss_pending="+std::to_string(recovered.out.approach_loss_pending)+
        " frames="+std::to_string(recovered.out.approach_sequence_frames)+
        " hits="+std::to_string(recovered.out.approach_sequence_hits)+
        " qualified="+std::to_string(recovered.out.approach_sequence_qualified)+
        " blocks="+std::to_string(recovered.out.approach_completed_blocks)+
        " guide="+std::to_string(recovered.out.line_decision_guide && recovered.out.line_decision_guide->valid)+
        " mode="+std::to_string(static_cast<int>(recovered.out.command.control_phase)));
  recovered.Tick({next,true,false,false});
  recovered.Tick({recovered.current,true,true,false});
  Check(recovered.out.approach_completed_blocks==completed &&
        recovered.out.approach_mission==MissionType::kBall &&
        LinePlan::IsUnit(recovered.out.command.action),
        "aborted probe counted toward lock or failed to resume normal approach");
  recovered.current=next;
  recovered.Block();
  Check(recovered.out.approach_completed_blocks==completed+1,
        "normal block after rediscovery did not retain the original lock count");
  recovered.MissingBlock();
  Check(recovered.out.approach_mission==MissionType::kBall &&
        LinePlan::IsUnit(recovered.out.command.action) &&
        recovered.out.approach_completed_blocks==completed+1,
        "a later distinct loss did not permit one new probe");
}
void PrelockProbeForMission(MissionType mission) {
  auto c=Config();c.enable_ball=false;
  if(mission==MissionType::kGoal) c.goal.head_scan_enabled=false;
  c.enable_hurdle=mission==MissionType::kHurdle;
  c.enable_goal=mission==MissionType::kGoal;
  c.initial_has_ball=c.goal_trial_start_goal=mission==MissionType::kGoal;
  Driver d(c);d.in.ball_target.reset();
  if(mission==MissionType::kHurdle) {
    d.in.hurdle_target=Target();d.in.hurdle_track_id=5;
    d.Warm();d.Block();d.Block();
  } else {
    d.in.camera_feedback={CameraMode::kGoal,true};
    d.in.line_centers.clear();d.in.backboard_target=Target();
    d.in.backboard_track_id=9;d.in.backboard_center_depth_m=2;
    d.in.goal_pose={true,0,2,0,1};
    for(int i=0;i<12;++i)d.Tick();
    d.current=d.out.command.action_id;
    Check(d.current && LinePlan::IsUnit(d.out.command.action),
          "far GOAL probe fixture did not start an approach block");
    d.Block();
  }
  const int completed=d.out.approach_completed_blocks;
  Check(completed<3,"probe fixture locked before object loss");
  if(mission==MissionType::kHurdle)
    Check(d.out.remembered_line_score.has_value(),
          "far HURDLE approach did not remember its actual LINE");
  d.in.hurdle_target.reset();d.in.backboard_target.reset();
  d.in.backboard_center_depth_m.reset();d.in.goal_pose={};
  for(int i=0;i<12;++i)d.Tick();
  d.Tick({d.current,true,false,true});
  const auto second=d.out.command.action_id;
  Check(second && second!=d.current && LinePlan::IsUnit(d.out.command.action),
        "pre-lock loss interrupted the running HURDLE/GOAL block");
  d.Tick({second,true,false,false});
  d.Tick({d.current,true,true,false});d.current=second;
  for(int i=0;i<12;++i)d.Tick();
  d.Tick({d.current,true,false,true});
  Check(d.out.command.action_id==d.current,
        "HURDLE/GOAL loss booked a normal successor without evidence");
  d.Tick({d.current,true,true,false});
  Check(d.out.approach_mission==mission &&
        d.out.approach_completed_blocks==completed &&
        LinePlan::IsUnit(d.out.command.action),
        "pre-lock HURDLE/GOAL loss did not start one shared probe block");
  FinishProbe(d);
  Check(d.out.approach_mission==MissionType::kNone,
        "failed HURDLE/GOAL probe did not release to LINE");
  if(mission==MissionType::kHurdle)
    Check(d.out.remembered_line_score.has_value(),
          "HURDLE probe release erased the actual LINE memory");
  if(mission==MissionType::kGoal)
    Check(d.out.command.camera_request==CameraRequest::kBallViewLine,
          "failed GOAL probe did not request BALL_VIEW before LINE recovery");
}
void LockAndLoss() {
  Driver d(Config());d.Warm();d.Block();
  d.Block(); // Initial LINE + first object-owned block = 2/3.
  Check(!d.out.approach_locked,"two blocks already locked the target");
  // No READY/ACK, including the final block's last READY, may commit early.
  d.Tick({d.current,true,false,true});
  Check(!d.out.approach_locked,"READY counted as physical completion");
  auto next=d.out.command.action_id;
  Check(next && next!=d.current,"third block first successor missing");
  d.Tick({next,true,false,false});d.Tick({d.current,true,true,false});d.current=next;
  for(int i=0;i<12;++i)d.Tick();
  d.Tick({d.current,true,false,true});
  Check(!d.out.approach_locked,"third block final READY committed too early");
  next=d.out.command.action_id;d.Tick({next,true,false,false});
  d.Tick({d.current,true,true,false});d.current=next;
  Check(d.out.approach_completed_blocks==3 && d.out.approach_locked,
        "third complete same-identity block did not lock");
  d.in.ball_target.reset();for(int i=0;i<3;++i)d.Tick();
  Check(d.out.approach_locked && d.out.active_mission==MissionType::kLine,
        "locked loss released the object or entered recovery before issued motion DONE");
  d.MissingBlock();
  Check(d.out.active_mission==MissionType::kBall && d.out.ball.active,
        "locked loss did not hand off to the existing BALL recovery");
  Check(!d.out.ball.entry_recovery && d.out.ball.recovery_step_limit==Config().ball.recovery_max_steps,
        "locked BALL reused the pre-lock entry-loss budget");
}

void GoalLineReturnThreePointSteering() {
  struct SteeringCase { double offset, slope; MissionAction action; };
  const std::array<SteeringCase,5> cases{{
      {0,0,MissionAction::kLineStraightUnit},
      {-80,0,MissionAction::kLineLeftUnit},
      {80,0,MissionAction::kLineRightUnitTurnFirst},
      {0,.5,MissionAction::kLineLeftUnit},
      {0,-.5,MissionAction::kLineRightUnitTurnFirst}}};
  for (int points : {3,4,5}) for (const auto& steering : cases) {
    auto c=Config();
    c.enable_ball=false;c.enable_goal=true;c.initial_has_ball=true;
    c.goal_trial_start_goal=true;c.goal.head_scan_enabled=false;c.goal.lost_frames=3;
    c.goal.post_pickup_straight_blocks_required=0;
    c.goal.post_pickup_walk_fallback_actions=1;
    c.line_p2p.left_repetitions=c.line_p2p.straight_repetitions=
        c.line_p2p.right_repetitions=1;
    c.line_p2p.right_unit_action=36;
    c.line_p2p.offset_gain=.45;c.line_p2p.heading_gain=.3;
    c.line_p2p.steering_deadband=.08;c.line_p2p.in_place_turn_score=.5;
    c.line_features.curve_local_fit_points=4;
    Driver d(c);d.in.ball_target.reset();
    d.in.camera_feedback={CameraMode::kGoal,true};
    bool returned=false;
    for(int i=0;i<100 && !returned;++i) {
      d.Tick();returned=d.out.command.camera_request==CameraRequest::kBallViewLine;
    }
    Check(returned,"three-point fixture never returned from empty GOAL to LINE");
    d.in.camera_feedback={CameraMode::kTransition,false};d.Tick();
    d.in.line_centers.clear();
    const double mean_v=450-50*(points-1);
    for(int i=0;i<points;++i) {
      const double v=450-100*i;
      d.in.line_centers.push_back({320+steering.offset+steering.slope*(v-mean_v),v});
    }
    d.in.camera_feedback={CameraMode::kBallView,true};
    for(int i=0;i<80 && !d.out.command.action_id;++i)d.Tick();
    const auto id=d.out.command.action_id;
    Check(id && d.out.command.action==steering.action && !d.out.line_in_recovery,
          "GOAL floor return did not select normal LINE from three-point O/H");
    // Five points retain the existing nearest-four fit reference (v + 50).
    const double expected_offset=steering.offset+(points==5 ? 50*steering.slope : 0);
    Check(d.out.line_features.guide.valid &&
          std::abs(d.out.line_features.guide.offset-expected_offset/320)<1e-9 &&
          std::abs(d.out.line_features.guide.heading_rad+std::atan(steering.slope))<1e-9,
          "GOAL floor O/H changed its geometry for three/four/five points");
    d.Tick({id,true,false,false});
    for(int i=0;i<12;++i)d.Tick();
    d.Tick({id,true,false,true});
    Check(d.out.line_window_stats && d.out.line_window_stats->used_frames>=5 &&
          d.out.command.camera_request==CameraRequest::kNone &&
          d.out.command.action_id==id,
          "GOAL floor return skipped its READY mean or changed camera before real DONE");
    d.Tick({id,true,true,false});
    bool goal_view=d.out.command.camera_request==CameraRequest::kGoal;
    for(int i=0;i<40 && !goal_view;++i) {
      d.Tick();goal_view=d.out.command.camera_request==CameraRequest::kGoal;
    }
    Check(goal_view && d.out.goal_walk_actions_completed==1,
          "one normal GOAL floor LINE block did not recheck the backboard");
  }
}

void GoalLineReturnSparseRecoveryUnchanged() {
  for(int points : {0,1,2,3}) {
    auto c=Config();c.enable_ball=false;c.enable_goal=true;c.initial_has_ball=true;
    c.goal.post_pickup_straight_blocks_required=0;
    c.goal.post_pickup_walk_fallback_actions=100;
    auto old=c;old.enable_goal=false; // Keeps the existing four-point input gate.
    Driver d(c), baseline(old);
    d.in.ball_target.reset();baseline.in.ball_target.reset();
    d.in.camera_feedback=baseline.in.camera_feedback={CameraMode::kBallView,true};
    d.in.line_centers.clear();
    for(int i=0;i<points;++i)
      d.in.line_centers.push_back({300.0+10*i,points==3 ? 400.0 : 450.0-100*i});
    // The three-point case is degenerate (no vertical span), hence also invalid.
    baseline.in.line_centers=d.in.line_centers;
    std::uint64_t action_id=0;int age=0;
    for(int i=0;i<120;++i) {
      CommandDeliveryFeedback feedback;
      if(d.out.command.action_id && d.out.command.action_id!=action_id) {
        action_id=d.out.command.action_id;age=0;
        feedback={action_id,true,false,false};
      } else if(action_id && ++age==14) feedback={action_id,true,true,false};
      d.Tick(feedback);baseline.Tick(feedback);
      Check(d.out.command.command_type==baseline.out.command.command_type &&
            d.out.command.action==baseline.out.command.action &&
            d.out.command.action_id==baseline.out.command.action_id &&
            d.out.command.camera_request==baseline.out.command.camera_request &&
            d.out.line_in_recovery==baseline.out.line_in_recovery &&
            d.out.line_features.guide.valid==baseline.out.line_features.guide.valid,
            "GOAL floor three-point policy changed existing sparse/degenerate recovery");
    }
  }
}

void GoalLineThreePointScope() {
  for(int scenario=0;scenario<6;++scenario) {
    auto c=Config();c.enable_ball=false;c.enable_goal=true;c.initial_has_ball=true;
    c.goal.head_scan_enabled=false;
    c.goal.post_pickup_straight_blocks_required=0;
    c.goal.post_pickup_walk_fallback_actions=100;
    if(scenario==3)c.initial_has_ball=false;
    if(scenario==4)c.enable_goal=false;
    Driver d(c);d.in.ball_target.reset();
    d.in.line_centers={{320,450},{320,350},{320,250}};
    const auto mode=scenario==0 ? CameraMode::kForward :
        scenario==1 ? CameraMode::kDown :
        scenario==2 ? CameraMode::kGoal : CameraMode::kBallView;
    d.in.camera_feedback={mode,true};
    if(scenario==5)d.in.manual_observation_control=true;
    bool checked=false;
    for(int i=0;i<50;++i) {
      d.Tick();
      if(d.out.line_computed) {
        checked=true;
        Check(!d.out.line_features.guide.valid,
              "GOAL return three-point gate leaked into another camera/mission/manual mode");
      }
    }
    Check(checked,"three-point scope fixture never computed LINE features");
  }
}

void EmptyGoalRetryLimit() {
  auto c=Config();c.enable_ball=false;c.enable_goal=true;c.initial_has_ball=true;
  c.goal_trial_start_goal=true;c.goal.lost_frames=3;
  // Preserve the old fallback-budget test separately from the new GOAL scan.
  c.goal.head_scan_enabled=false;
  c.object_approach.goal_empty_max_retries=3;
  c.goal.post_pickup_walk_fallback_actions=1;
  Driver d(c);d.in.ball_target.reset();d.in.camera_feedback={CameraMode::kGoal,true};
  int returns=0,age=0;std::uint64_t active=0,queued=0,done=0;
  bool ready=false;
  for(int frame=0;frame<600 && !d.out.terminal_hold;++frame) {
    CommandDeliveryFeedback fb;
    const auto id=d.out.command.action_id;
    if(id && LinePlan::IsUnit(d.out.command.action) && id!=done && id!=active && id!=queued) {
      if(!active) { active=id;age=0;ready=false; }
      else queued=id;
      fb={id,true,false,false};
    } else if(active && ++age>=12 && !ready) {
      ready=true;fb={active,true,false,true};
    } else if(active && age>=14) {
      done=active;fb={active,true,true,false};
      active=queued;queued=0;age=0;ready=false;
    }
    if(d.out.command.camera_request==CameraRequest::kBallViewLine &&
        d.in.camera_feedback.actual_mode!=CameraMode::kBallView) {
      ++returns;d.in.camera_feedback={CameraMode::kBallView,true};
    } else if(d.out.command.camera_request==CameraRequest::kGoal)
      d.in.camera_feedback={CameraMode::kGoal,true};
    d.Tick(fb,.1);
  }
  Check(returns==3 && d.out.terminal_hold && d.out.approach_goal_search_failed,
        "empty GOAL view did not stop after exactly three LINE/recheck retries");
  Check(d.out.goal.entry_line_retries==3,"GOAL retry diagnostics lost the consumed budget");
  d.in.backboard_target=Target();d.in.backboard_center_depth_m=2;
  for(int i=0;i<20;++i) {
    d.Tick();Check(d.out.terminal_hold && d.out.command.command_type!=CommandType::kAction &&
        d.out.command.camera_request==CameraRequest::kNone,
        "GOAL retry exhaustion resumed automatically");
  }
  d.core.Reset(true);d.in.backboard_target.reset();d.in.backboard_center_depth_m.reset();d.Tick();
  Check(!d.out.approach_goal_search_failed && !d.out.goal.entry_line_retries,
        "RESET retained empty GOAL failure or retry count");
}
void NearPixelVotesWithGaps(MissionType mission) {
  auto c=Config();
  c.enable_ball=mission==MissionType::kBall;
  c.enable_hurdle=mission==MissionType::kHurdle;
  // CTest deliberately uses an older fixture with near_v_min_hits=3.
  // Explicitly exercise the new production 10/5 hurdle near policy.
  if(mission==MissionType::kHurdle)c.hurdle.near_v_min_hits=5;
  Driver d(c);
  if (mission==MissionType::kHurdle) {
    d.in.ball_target.reset();
    d.in.hurdle_target=Target();
    d.in.hurdle_track_id=5;
  }
  d.Warm();
  Check(d.out.approach_mission==mission,"near-v test did not acquire object");
  const auto sample=[&](std::optional<double> v) {
    if (mission==MissionType::kBall) d.in.ball_target=v ? std::optional<ObjectTarget>{Target(320,*v)} : std::nullopt;
    else d.in.hurdle_target=v ? std::optional<ObjectTarget>{Target(320,*v)} : std::nullopt;
    d.Tick();
  };
  sample(260);Check(!d.out.approach_near,"first near-v vote entered near");
  sample(std::nullopt);Check(!d.out.approach_near,"missing frame counted as near");
  sample(260);Check(!d.out.approach_near,"two near-v votes entered near");
  sample(std::nullopt);Check(!d.out.approach_near,"missing frame completed near vote");
  sample(260);
  if(mission==MissionType::kHurdle){
    Check(!d.out.approach_near,"HURDLE latched before five v>=240 votes");
    sample(260);sample(260); // Five near votes, seven total valid frames.
  }
  Check(d.out.approach_near,"configured nonconsecutive near-v votes did not latch");
}
void NearFinalActionSequence(double u,bool left,bool extreme) {
  auto c=Config();c.ball.approach_extreme_turn_margin_px=90.0;
  Driver d(c);d.in.ball_target=Target(u,100);
  d.Warm();d.Block();d.Block();
  d.in.ball_target=Target(u,260);
  for(int n=0;n<3;++n)d.Tick();
  Check(d.out.approach_near,"near sequence failed to latch three v votes");
  d.Tick({d.current,true,false,true});
  d.Tick({d.current,true,true,false});
  for(int n=0;n<35 && d.out.command.action_id==0;++n)d.Tick();
  const MissionAction turn=left ? MissionAction::kTurnLeftInPlace :
      MissionAction::kStepForwardRightOneHurdle;
  if (d.out.command.action!=turn || !d.out.approach_near_final_motion)
    throw std::runtime_error("near final first: action="+
        std::to_string(static_cast<int>(d.out.command.action))+
        " expected="+std::to_string(static_cast<int>(turn))+
        " near_motion="+std::to_string(d.out.approach_near_final_motion)+
        " near_observing="+std::to_string(d.out.approach_near_observing)+
        " mission="+std::to_string(static_cast<int>(d.out.active_mission))+
        " id="+std::to_string(d.out.command.action_id));
  std::uint64_t id=d.out.command.action_id;
  d.Tick({id,true,false,false});
  d.Tick({id,true,true,false});
  if (extreme) {
    Check(d.out.command.action==turn && d.out.command.action_id!=id,
          "extreme zone did not request second turn after first DONE");
    id=d.out.command.action_id;
    d.Tick({id,true,false,false});
    d.Tick({id,true,true,false});
  }
  // A committed near chain must now move forward without an intermediate
  // observation after the 1/2 rotation actions.
  Check(d.out.command.action==MissionAction::kStepForwardTwo &&
        d.out.command.action_id!=id && d.out.approach_near_final_motion,
        "near 5-zone did not append two forward steps after the turns");
  id=d.out.command.action_id;
  d.Tick({id,true,false,false});
  Check(d.out.active_mission==MissionType::kLine,
        "BALL near switched camera before committed forward DONE");
  d.Tick({id,true,true,false});
  Check(d.out.active_mission==MissionType::kBall &&
        d.out.command.camera_request==CameraRequest::kBallView,
        "BALL near committed turn + forward chain did not hand off to DOWN");

}
void ReadyCenterAccumulator() {
  LineGuideAccumulator acc(5,0.0,10,5);
  acc.Begin(42,0.0);
  for(int n=1;n<=10;++n)
    acc.AddCenter(42,n*0.1,Point2{n<=5?200.0:500.0,160.0});
  const auto mean=acc.FinishCenter(42,1.0,0.0);
  Check(mean && mean->u>350 && mean->u<500 && std::abs(mean->v-160)<1e-9,
        "READY coordinate mean did not weight late detections 1-to-3");
  Check(!acc.FinishCenter(42,1.0,0.0),"consumed READY mean reused old pixels");
  acc.Begin(43,0.0);
  for(int n=1;n<=10;++n)
    acc.AddCenter(43,n*0.1,n<=4 ? std::optional<Point2>{Point2{320,120}} : std::nullopt);
  Check(!acc.FinishCenter(43,1.0,0.0),
        "READY averaged insufficient valid coordinate samples");

  // A BALL-owned block uses its 10/4 detector gate, not LINE's 10/5.
  auto c=Config();c.line_p2p.straight_repetitions=1;
  LinePlan plan(c.line_p2p,10,5);
  plan.Begin(MissionAction::kStepForwardFour,77);
  ControlCommand issued;issued.command_type=CommandType::kAction;
  issued.action=MissionAction::kLineStraightUnit;issued.action_id=750;
  plan.Issued(issued,0);
  plan.Feedback({750,true,false,false},0,0);
  for(int n=1;n<=10;++n) {
    const bool detected=n<=4;
    plan.Observe(n*.1,{}, {},77,
                 detected ? std::optional<Point2>{Point2{365.0,120.0}} : std::nullopt,true);
    plan.ObserveObject(n*.1,77,detected,10,4);
  }
  Check(plan.ObjectQualified(750),"10/4 BALL evidence was not qualified");
  const auto center=plan.FinishObjectCenter(750,1.0,0.0,false,plan.ObjectQualified(750));
  Check(center && std::abs(center->u-365.0)<1e-6,
        "qualified 10/4 BALL detection failed the READY average due to LINE 10/5");
}
void NearAndIdentity() {
  Driver d(Config());d.Warm();d.Block();d.Block();
  d.in.ball_target=Target(320,260);
  d.Tick();Check(!d.out.approach_near,"BALL near entered on one vote");
  d.Tick();Check(!d.out.approach_near,"BALL near entered on two votes");
  d.Tick();
  Check(d.out.approach_near && !d.out.approach_locked,"near approach unnecessarily waited for three blocks");
  d.Tick({d.current,true,false,true});
  Check(d.out.command.action_id==d.current && d.out.command.camera_request==CameraRequest::kNone,
        "near boundary queued another walk or moved the camera before DONE");
  d.Tick({d.current,true,true,false});
  Check(d.out.active_mission==MissionType::kLine && d.out.approach_near,
        "near BALL skipped the fresh stopped observation");
  for(int n=0;n<30 && d.out.command.action_id==0;++n)d.Tick();
  const auto near_step=d.out.command.action_id;
  Check(near_step && d.out.command.action==MissionAction::kStepForwardTwo,
        "near BALL did not choose the grid's upper-center two-step action");
  d.Tick({near_step,true,false,false});
  d.Tick({near_step,true,true,false});
  Check(d.out.active_mission==MissionType::kBall,"near object did not use the original near BALL controller");
  Check(d.out.ball.mode==BallMode::kWaitCameraDown && d.out.command.camera_request==CameraRequest::kBallView &&
      !d.out.remembered_line_score,"near BALL did not discard LINE memory and enter DOWN immediately");
  // BALL_VIEW physical DONE must NOT introduce a generic extra 1s window.
  // The first 10-step moving search should issue within the next two frames.
  d.in.camera_feedback={CameraMode::kTransition,false};
  d.Tick();
  d.in.camera_feedback={CameraMode::kBallView,true};
  d.in.ball_target.reset();
  for(int n=0;n<3 && d.out.command.action!=MissionAction::kStepForwardOne;++n)d.Tick();
  if (!(d.out.command.action==MissionAction::kStepForwardOne &&
        !d.out.post_motion_observing))
    throw std::runtime_error("BALL_VIEW camera DONE inserted a stationary 1s window before moving DOWN search: mode="+
        std::to_string(static_cast<int>(d.out.ball.mode))+
        " action="+std::to_string(static_cast<int>(d.out.command.action))+
        " action_id="+std::to_string(d.out.command.action_id)+
        " phase="+std::to_string(static_cast<int>(d.out.command.control_phase))+
        " mission_phase="+std::to_string(d.out.command.mission_phase)+
        " mission="+std::to_string(static_cast<int>(d.out.active_mission))+
        " observing="+std::to_string(d.out.post_motion_observing)+
        " camera_request="+std::to_string(static_cast<int>(d.out.command.camera_request))+
        " planned_ball_action="+std::to_string(static_cast<int>(d.out.ball.action.action))+
        " repeat_scope="+std::to_string(d.out.repeat_scope)+
        " ball_active="+std::to_string(d.out.ball.active)+
        " approach_missing="+std::to_string(d.out.approach_missing_frames));
  Driver replacement(Config());replacement.Warm();replacement.Block();replacement.Block();
  replacement.in.ball_track_id=99;
  for(int i=0;i<3;++i) replacement.Tick();
  Check(replacement.out.approach_completed_blocks==2,
        "replacement identity cancelled a walking block before its evidence window");
  replacement.MissingBlock();
  Check(replacement.out.approach_mission==MissionType::kBall &&
        replacement.out.approach_completed_blocks==2,
        "replacement identity incorrectly inherited a new lock during probe");
  FinishProbe(replacement);
  replacement.in.ball_target=Target();
  for(int i=0;i<12;++i) replacement.Tick();
  Check(replacement.out.approach_completed_blocks==0 && !replacement.out.approach_locked,
        "a replacement identity inherited completed object walks");
}
void HurdleAndGoal() {
  auto c=Config();c.enable_ball=false;c.enable_hurdle=true;
  Driver h(c);h.in.ball_target.reset();h.in.hurdle_target=Target();h.in.hurdle_track_id=5;
  h.Warm();h.Block();h.Block();
  Check(h.out.approach_mission==MissionType::kHurdle && h.out.approach_completed_blocks==1,
        "HURDLE did not use the common LINE block planner");
  h.in.hurdle_target=Target(320,260);
  for(int n=0;n<c.hurdle.near_v_min_hits+4 && !h.out.approach_near;++n)h.Tick();
  Check(h.out.approach_near,"HURDLE failed to latch its configured v>=240 evidence");
  h.Tick({h.current,true,false,true});
  // Unlike the legacy near-at-DONE sequence, the one final 32/33/36
  // must be reserved from the normal LINE READY O/H mean.
  Check(h.out.command.control_phase==ControlPhase::kWaitingQueuedActionAck &&
        h.out.command.action_id!=h.current &&
        LinePlan::IsUnit(h.out.command.action),
        "HURDLE READY failed to reserve its single O/H-selected unit");
  const auto hurdle_unit=h.out.command.action_id;
  h.Tick({hurdle_unit,true,false,false}); // queued ACK is not motion DONE
  h.Tick({h.current,true,true,false});
  Check(h.out.active_mission==MissionType::kLine && h.out.approach_near,
        "near HURDLE switched before its final unit physically DONE");
  Check(hurdle_unit && h.out.command.action==MissionAction::kLineStraightUnit,
        "HURDLE near did not choose one additional O/H-steered 33 unit");
  h.Tick({hurdle_unit,true,false,false});h.Tick({hurdle_unit,true,true,false});
  Check(h.out.active_mission==MissionType::kHurdle &&
        h.out.command.camera_request==CameraRequest::kDown,
        "HURDLE did not hand off to DOWN after its one extra LINE unit");

  c=Config();c.enable_ball=false;c.enable_goal=true;c.initial_has_ball=true;c.goal_trial_start_goal=true;
  Driver g(c);g.in.ball_target.reset();g.in.camera_feedback={CameraMode::kGoal,true};
  g.in.line_centers.clear();g.in.backboard_target=Target();g.in.backboard_track_id=9;
  g.in.backboard_center_depth_m=2;g.in.goal_pose={true,0,2,0,1};
  for(int i=0;i<12;++i)g.Tick();
  Check(g.out.approach_mission==MissionType::kGoal && LinePlan::IsUnit(g.out.command.action) &&
        g.out.line_decision_guide && g.out.line_decision_guide->valid && !g.out.line_features.guide.valid,
        "GOAL approach required floor-line detections or failed to generate a common O/H guide");
  g.current=g.out.command.action_id;g.Block();
  g.in.backboard_center_depth_m=.5;g.in.goal_pose.z_m=.5;
  for(int n=0;n<2;++n) {
    g.Tick();
    Check(!g.out.approach_near,"GOAL entered near approach before three depth samples");
  }
  g.Tick();
  Check(g.out.approach_near,"GOAL did not enter near approach after three depth samples");
  g.Tick({g.current,true,false,true});
  Check(g.out.command.action_id==g.current,"near GOAL reserved another shared unit");
  g.Tick({g.current,true,true,false});
  for(int n=0;n<30 && g.out.active_mission==MissionType::kLine;++n)g.Tick();
  Check(g.out.active_mission==MissionType::kGoal,"near GOAL did not enter existing fine approach");
  Check(g.out.goal_mission_index==c.test_goal_mission_index,"direct GOAL handoff incremented an already-started trial index");
}
void ManualAndReset() {
  auto c=Config();
  Driver d(c);d.in.manual_observation_control=true;d.Warm();
  Check(d.out.candidate_mission==MissionType::kBall && !d.out.approach_locked &&
        d.out.approach_mission==MissionType::kNone,"manual approval lost stationary candidate verification");
  Driver reset(Config());reset.Warm();reset.Block();reset.Block();reset.Block();reset.Block();
  Check(reset.out.approach_locked,"reset fixture did not reach lock");
  reset.core.Reset(true);reset.in.ball_target.reset();reset.Tick();
  Check(!reset.out.approach_locked && !reset.out.approach_completed_blocks &&
        reset.out.approach_mission==MissionType::kNone,"RESET retained approach commitment");
}
void CameraDrainAndProbe() {
  auto c=Config();c.enable_ball=false;c.enable_goal=true;c.initial_has_ball=true;
  c.goal.head_scan_enabled=false; // Old pre-lock fallback contract; scan tested separately.
  c.goal.post_pickup_walk_fallback_actions=3;
  Driver d(c);d.in.ball_target.reset();d.Warm();
  d.Tick({d.current,true,false,true});
  const auto second=d.out.command.action_id;
  Check(second && second!=d.current,"probe fixture did not reserve second LINE unit");
  d.Tick({second,true,false,false});d.Tick({d.current,true,true,false});d.current=second;
  for(int i=0;i<12;++i)d.Tick();
  d.Tick({d.current,true,false,true});
  Check(d.out.command.action_id==d.current && d.out.command.camera_request==CameraRequest::kNone,
        "GOAL camera moved at READY or a new block was booked across the carrying gate");
  d.Tick({d.current,true,true,false});
  Check(d.out.command.camera_request==CameraRequest::kGoal && d.out.command.command_type!=CommandType::kAction,
        "GOAL probe did not wait for all issued locomotion DONE");
  d.in.line_centers.clear();d.in.camera_feedback={CameraMode::kTransition,false};
  for(int i=0;i<3;++i) {
    d.Tick();Check(d.out.command.command_type!=CommandType::kAction,"camera transition issued locomotion");
  }
  d.in.camera_feedback={CameraMode::kGoal,true};d.in.backboard_target=Target();d.in.backboard_track_id=9;
  d.in.backboard_center_depth_m=2;d.in.goal_pose={true,0,2,0,1};
  for(int i=0;i<10;++i)d.Tick();
  Check(LinePlan::IsUnit(d.out.command.action) && !d.out.line_in_recovery,
        "GOAL camera acquisition was incorrectly routed into missing-floor-LINE recovery");
  d.current=d.out.command.action_id;d.Tick({d.current,true,false,false});
  d.in.backboard_target.reset();d.in.backboard_center_depth_m.reset();d.in.goal_pose={};
  for(int i=0;i<3;++i)d.Tick();
  Check(d.out.command.camera_request==CameraRequest::kNone,"soft GOAL loss moved the head during locomotion");
  d.Tick({d.current,true,false,true});
  const auto second_goal_unit=d.out.command.action_id;
  Check(second_goal_unit && second_goal_unit!=d.current && LinePlan::IsUnit(d.out.command.action),
        "one missing GOAL window cancelled the remainder of its walking block");
  d.Tick({second_goal_unit,true,false,false});
  d.Tick({d.current,true,true,false});d.current=second_goal_unit;
  for(int i=0;i<12;++i)d.Tick();
  d.Tick({d.current,true,false,true});
  Check(d.out.command.action_id==d.current,"unqualified GOAL block booked another walk");
  d.Tick({d.current,true,true,false});
  // No qualified GOAL READY mean was observed before the loss. The controller
  // must not infer a search direction from an instantaneous last detection.
  Check(d.out.approach_mission==MissionType::kNone &&
        d.out.command.camera_request==CameraRequest::kBallViewLine,
        "soft GOAL loss without READY mean should use BALL_VIEW without a blind probe");
}
void GoalTurnObservationUsesSelectedPath() {
  auto c=Config();c.enable_ball=false;c.enable_goal=true;c.initial_has_ball=true;
  c.object_approach.steering_deadband_deg=2.0;
  c.object_approach.in_place_turn_deg=2.5;
  Driver d(c);d.in.ball_target.reset();d.in.line_centers.clear();
  d.in.camera_feedback={CameraMode::kGoal,true};d.in.backboard_target=Target(500,100);
  d.in.backboard_track_id=7;d.in.backboard_center_depth_m=2;d.in.goal_pose={true,.6,2,0,1};
  for(int i=0;i<12;++i)d.Tick();
  Check(d.out.command.action==MissionAction::kStepForwardRightOneHurdle,
        "GOAL path did not reuse the LINE in-place steering guard");
  const auto turn=d.out.command.action_id;d.Tick({turn,true,false,false});
  d.in.backboard_target=Target();d.in.goal_pose.x_m=0;
  d.Tick({turn,true,true,false});
  for(int i=0;i<20;++i)d.Tick({},.10);
  Check(LinePlan::IsUnit(d.out.command.action) && !d.out.line_in_recovery &&
        !d.out.approach_completed_blocks,
        "GOAL post-turn observation used invisible floor lines or counted the turn as an object block");
}

void FarGoalDepthRecovery(bool found,bool acquired=false) {
  auto c=Config();c.enable_ball=false;c.enable_goal=true;c.initial_has_ball=true;
  c.goal_trial_start_goal=true;c.goal.lost_frames=3;
  c.goal.far_depth_probe_max_attempts=3;c.line_p2p.straight_repetitions=1;
  Driver d(c);d.in.ball_target.reset();d.in.line_centers.clear();
  d.in.camera_feedback={CameraMode::kGoal,true};
  d.in.backboard_image_without_depth=true;d.in.raw_backboard_u=320;
  if(!acquired) {
    // Initial image-only GOAL must not trigger a blind Depth WALK, regardless
    // of how long its neck scan takes to start.
    for(int n=0;n<30;++n){
      d.Tick({},.1);
      Check(d.out.command.action==MissionAction::kNone,
          "initial YOLO-only GOAL started a Depth recovery WALK before acquisition");
    }
    return;
  }
  d.in.backboard_target=Target();d.in.backboard_track_id=9;
  d.in.backboard_image_without_depth=false;
  d.in.backboard_center_depth_m=2;d.in.goal_pose={true,0,2,0,1};
  for(int n=0;n<15 && !d.out.command.action_id;++n)d.Tick();
  auto first=d.out.command.action_id;
  Check(first && d.out.approach_mission==MissionType::kGoal,
        "Depth fixture failed to start a real GOAL approach");
  d.Tick({first,true,false,false});
  for(int n=0;n<12;++n)d.Tick();
  // Lose coordinate data while A is still executing; no successor unit may
  // be reserved from the pre-loss GOAL mean.
  d.in.backboard_target.reset();d.in.backboard_center_depth_m.reset();
  d.in.goal_pose={};d.in.backboard_image_without_depth=true;
  d.Tick({first,true,true,false});
  // One real GOAL-owned DONE, not ACK or READY, arms the new far-depth path.
  std::uint64_t last=first;
  for(int n=1;n<=3;++n){
    for(int t=0;t<100 && (d.out.command.action_id==0 ||
         d.out.command.action_id==last);++t)d.Tick({},.1);
    const auto id=d.out.command.action_id;
    Check(id && id!=last && d.out.command.action==MissionAction::kStepForwardOne,
        "acquired image-only GOAL failed to schedule one direction-aware probe");
    d.Tick({id,true,false,false});
    d.Tick({id,true,true,false});last=id;
    if(found){
      d.in.backboard_target=Target();d.in.backboard_track_id=9;
      d.in.backboard_center_depth_m=2;d.in.goal_pose={true,0,2,0,1};
      d.in.backboard_image_without_depth=false;
    }
    for(int t=0;t<26;++t)d.Tick({},.1);
    if(found){
      Check(!d.out.approach_goal_depth_failed,
          "successful stopped Depth observation entered terminal failure");
      return;
    }
  }
  Check(!d.out.terminal_hold && !d.out.approach_goal_depth_failed,
        "bounded far-depth probe must return to LINE instead of terminal hold");
  Check(d.out.command.camera_request==CameraRequest::kBallViewLine ||
        d.out.approach_mission==MissionType::kNone,
        "third failed GOAL Depth probe did not return to LINE");
}

void GoalStableNewIdentityKeepsApproach() {
  auto c=Config();c.enable_ball=false;c.enable_goal=true;
  c.initial_has_ball=c.goal_trial_start_goal=true;
  c.line_p2p.straight_repetitions=1;
  Driver d(c);d.in.ball_target.reset();d.in.line_centers.clear();
  d.in.camera_feedback={CameraMode::kGoal,true};
  d.in.backboard_target=Target(320,100,3);
  d.in.backboard_track_id=1;
  d.in.backboard_entry_track_hits=7;
  d.in.backboard_entry_track_frames=10;
  d.in.backboard_center_depth_m=2.0;
  d.in.goal_pose={true,0,2,0,1};
  for(int n=0;n<14;++n)d.Tick();
  Check(d.out.approach_mission==MissionType::kGoal &&
        d.out.approach_track_id==1,
        "GOAL initial ID was not acquired");
  const auto first=d.out.command.action_id;
  Check(first,"GOAL first motion was not issued");
  // Give the physical first block its complete 10/7 window and final
  // READY.  A bare DONE with no READY would trigger a separate post-DONE
  // observation and could (correctly) mark an unqualified block lost.
  d.Tick({first,true,false,false});
  for(int n=0;n<12;++n)d.Tick();
  d.Tick({first,true,false,true});
  const auto next=d.out.command.action_id;
  Check(next && next!=first && LinePlan::IsUnit(d.out.command.action),
        "GOAL final READY did not reserve successor");
  d.Tick({next,true,false,false});
  d.Tick({first,true,true,false});
  const auto blocks=d.out.approach_completed_blocks;
  Check(blocks==1,"GOAL first qualified block DONE not counted");
  d.in.backboard_target=Target(375,100,3);
  d.in.backboard_track_id=2;
  d.in.backboard_entry_track_hits=6; // New ID not yet stable.
  d.in.backboard_entry_track_frames=10;
  d.Tick();
  Check(d.out.approach_track_id==1,
        "unstable GOAL ID replacement was adopted immediately");
  d.in.backboard_entry_track_hits=7;
  d.Tick();
  if(!(d.out.approach_track_id==2 && d.out.approach_missing_frames==0 &&
       d.out.approach_completed_blocks==blocks &&
       !d.out.approach_loss_pending))
    throw std::runtime_error("stable GOAL ID replacement: id="+
        std::to_string(d.out.approach_track_id)+" blocks="+
        std::to_string(d.out.approach_completed_blocks)+" expected="+
        std::to_string(blocks)+" missing="+
        std::to_string(d.out.approach_missing_frames)+" loss="+
        std::to_string(d.out.approach_loss_pending)+
        " active="+std::to_string(static_cast<int>(d.out.approach_mission)));
  // A different RGB-only ID is still a Depth-recovery input, not RGB loss.
  d.in.backboard_target.reset();
  d.in.backboard_track_id=3;
  d.in.backboard_image_without_depth=true;
  d.in.raw_backboard_u=320;
  for(int n=0;n<20;++n)d.Tick();
  Check(d.out.approach_mission==MissionType::kGoal &&
        !d.out.approach_loss_pending,
        "GOAL RGB-only depth episode incorrectly counted as RGB disappearance");
}
void GoalTrueRgbLossAndRangeRejection() {
  auto c=Config();c.enable_ball=false;c.enable_goal=true;
  c.initial_has_ball=c.goal_trial_start_goal=true;
  c.goal.lost_frames=3;c.line_p2p.straight_repetitions=1;
  // First verify that 3 truly empty RGB frames, not a changed ID, cause loss.
  Driver lost(c);lost.in.ball_target.reset();lost.in.line_centers.clear();
  lost.in.camera_feedback={CameraMode::kGoal,true};
  lost.in.backboard_target=Target(320,100,3);
  lost.in.backboard_track_id=1;
  lost.in.backboard_center_depth_m=2;lost.in.goal_pose={true,0,2,0,1};
  for(int n=0;n<12;++n)lost.Tick();
  Check(lost.out.approach_mission==MissionType::kGoal,"RGB loss fixture failed GOAL acquisition");
  lost.in.backboard_target.reset();
  lost.in.backboard_center_depth_m.reset();lost.in.goal_pose={};
  lost.in.raw_backboard_target.reset();lost.in.raw_backboard_u.reset();
  const auto lost_motion=lost.out.command.action_id;
  Check(lost_motion,"RGB loss fixture lost its first active GOAL action");
  lost.Tick({lost_motion,true,false,false});
  for(int n=0;n<2;++n)lost.Tick();
  Check(!lost.out.approach_loss_pending,"RGB loss confirmed before 3 missing frames");
  lost.Tick();
  Check(!lost.out.approach_loss_pending,
        "RGB loss cancelled an executing physical GOAL block");
  lost.Tick({lost_motion,true,false,true});
  lost.Tick({lost_motion,true,true,false});
  bool scanning=false;
  for(int n=0;n<30 && !scanning;++n) {
    lost.Tick();
    scanning=lost.out.command.camera_request==CameraRequest::kGoalScanLeft;
  }
  Check(scanning,"3+ GOAL RGB misses failed to scan after first actual DONE");
  // A forbidden numerical Depth must never authorize the missing-Depth probe.
  Driver range(c);range.in.ball_target.reset();range.in.line_centers.clear();
  range.in.camera_feedback={CameraMode::kGoal,true};
  range.in.backboard_target=Target(320,100,3);
  range.in.backboard_track_id=1;
  range.in.backboard_center_depth_m=2;range.in.goal_pose={true,0,2,0,1};
  for(int n=0;n<12;++n)range.Tick();
  const auto first=range.out.command.action_id;
  Check(first,"out-of-range GOAL fixture did not start");
  range.Tick({first,true,false,false});
  range.in.backboard_target.reset();
  range.in.backboard_center_depth_m.reset();range.in.goal_pose={};
  range.in.backboard_image_out_of_depth_range=true;
  range.in.raw_backboard_u=320;
  range.in.raw_backboard_target=Target(320,100,3);
  range.Tick({first,true,true,false});
  bool returned=false;
  for(int n=0;n<30 && !returned;++n) {
    range.Tick();
    Check(range.out.command.action!=MissionAction::kStepForwardOne,
          "out-of-range GOAL Depth issued blind forward recovery");
    returned=range.out.command.camera_request==CameraRequest::kBallViewLine;
  }
  Check(returned || range.out.approach_mission==MissionType::kNone,
        "out-of-range GOAL did not reach bounded LINE fallback");
}
void GoalEdgeTenInFourRawBBox() {
  // GOAL is initially image-visible but has no central Depth. The raw edge
  // guard must win before the first head scan or any blind Depth recovery.
  for(const auto left_turn:{true,false}) {
    auto c=Config();c.enable_ball=false;c.enable_goal=true;
    c.initial_has_ball=c.goal_trial_start_goal=true;
    c.goal.lost_frames=100;
    c.goal.edge_stable_window=10;c.goal.edge_stable_min_hits=4;
    Driver d(c);d.in.ball_target.reset();d.in.line_centers.clear();
    d.in.camera_feedback={CameraMode::kGoal,true};
    d.in.backboard_image_without_depth=true;
    d.in.raw_backboard_observation_available=true;
    for(int n=0;n<22;++n){
      // Four edge frames early in the window and a centered bbox afterwards.
      d.in.raw_backboard_target=Target(n<4?(left_turn?20:620):320);
      d.Tick({},.1);
      if(d.out.command.action==MissionAction::kTurnLeftInPlace ||
         d.out.command.action==MissionAction::kStepForwardRightOneHurdle)break;
    }
    const auto expected=left_turn?MissionAction::kTurnLeftInPlace :
                                  MissionAction::kStepForwardRightOneHurdle;
    const auto first=d.out.command.action_id;
    Check(first && d.out.command.action==expected,
          "raw bbox 10/4 edge latch did not prioritize the correct WALK turn");
    d.Tick({first,true,false,false});
    d.Tick({first,true,true,false});
    const auto second=d.out.command.action_id;
    Check(second && second!=first && d.out.command.action==expected,
          "first edge turn DONE did not request the second matching turn");
    d.Tick({second,true,false,false});
    d.Tick({second,true,true,false});
    for(int n=0;n<9;++n){
      d.in.raw_backboard_target=Target(320);
      d.Tick({},.1);
      Check(d.out.command.action!=expected,
          "edge guard issued a third turn before fresh stopped observation");
    }
  }
}

void NearLossBeforeLock(MissionType mission) {
  auto c=Config();c.enable_ball=mission==MissionType::kBall;
  c.enable_hurdle=mission==MissionType::kHurdle;c.enable_goal=mission==MissionType::kGoal;
  c.initial_has_ball=c.goal_trial_start_goal=mission==MissionType::kGoal;
  Driver d(c);
  if(mission==MissionType::kBall) d.Warm();
  else if(mission==MissionType::kHurdle) {
    d.in.ball_target.reset();d.in.hurdle_target=Target();d.in.hurdle_track_id=5;d.Warm();
  } else {
    d.in.ball_target.reset();d.in.camera_feedback={CameraMode::kGoal,true};
    d.in.line_centers.clear();d.in.backboard_target=Target();d.in.backboard_track_id=9;
    d.in.backboard_center_depth_m=2;d.in.goal_pose={true,0,2,0,1};
    for(int i=0;i<12;++i)d.Tick();d.current=d.out.command.action_id;
    Check(d.current!=0,"near GOAL fixture did not start common approach");
    d.Tick({d.current,true,false,false});
  }
  if(mission==MissionType::kBall)d.in.ball_target=Target(320,260);
  else if(mission==MissionType::kHurdle)d.in.hurdle_target=Target(320,260);
  else { d.in.backboard_center_depth_m=.5;d.in.goal_pose.z_m=.5; }
  d.Tick();
  if(mission!=MissionType::kGoal) {
    Check(!d.out.approach_near,"one BALL/HURDLE near vote latched early");
    d.Tick();Check(!d.out.approach_near,"two BALL/HURDLE near votes latched early");
    d.Tick();
  }
  if(mission==MissionType::kGoal) {
    Check(!d.out.approach_near,"one near GOAL depth sample entered FINE early");
    d.Tick();
    Check(!d.out.approach_near,"two near GOAL depth samples entered FINE early");
    d.Tick();
  }
  if(mission==MissionType::kHurdle)
    for(int n=0;n<c.hurdle.near_v_min_hits+4 && !d.out.approach_near;++n)d.Tick();
  Check(d.out.approach_near && !d.out.approach_locked,"pre-lock near boundary was not latched");
  // HURDLE intentionally retains floor O/H until one additional 32/33/36
  // finishes; unlike BALL/GOAL it cannot drop LINE steering at near latch.
  if(mission!=MissionType::kHurdle)
    Check(!d.out.remembered_line_score,"near boundary retained pre-entry LINE steering memory");
  // Even a visible next frame outside the boundary cannot undo near commitment.
  if(mission==MissionType::kBall)d.in.ball_target=Target();
  else if(mission==MissionType::kHurdle)d.in.hurdle_target=Target();
  else { d.in.backboard_center_depth_m=2;d.in.goal_pose.z_m=2; }
  d.Tick();Check(d.out.approach_near,"near boundary was undone by image/depth jitter");
  d.in.ball_target.reset();d.in.hurdle_target.reset();d.in.backboard_target.reset();
  d.in.backboard_center_depth_m.reset();d.in.goal_pose={};
  for(int i=0;i<3;++i)d.Tick();
  Check(d.out.approach_near && d.out.approach_mission==mission,
        "pre-lock near loss was discarded as a far soft candidate");
  d.Tick({d.current,true,true,false});
  if(mission==MissionType::kHurdle) {
    // After the triggering walk's DONE, HURDLE has exactly one final
    // O/H-selected LINE unit. No camera DOWN until that unit's actual DONE.
    for(int i=0;i<40 && d.out.command.action_id==0;++i)d.Tick({},.1);
    const auto final_unit=d.out.command.action_id;
    Check(final_unit && LinePlan::IsUnit(d.out.command.action),
          "near HURDLE lost its final O/H-selected LINE unit");
    d.Tick({final_unit,true,false,false});
    d.Tick({final_unit,true,true,false});
  }
  for(int i=0;i<120 && d.out.active_mission==MissionType::kLine;++i)d.Tick({},.1);
  Check(d.out.active_mission==mission && !d.out.remembered_line_score,
        "near loss returned to LINE instead of the committed object mission");
  if(mission!=MissionType::kGoal) {
    Check(d.out.command.camera_request==(mission==MissionType::kBall ?
        CameraRequest::kBallView : CameraRequest::kDown),
        "BALL/HURDLE near loss did not request its own camera pose");
    d.in.camera_feedback={mission==MissionType::kBall ?
        CameraMode::kBallView : CameraMode::kDown,true};
  } else Check(d.out.command.camera_request!=CameraRequest::kForward,
               "near GOAL loss restored FORWARD/LINE");
  d.in.allow_new_body_actions=false;
  for(int i=0;i<30;++i)d.Tick();
  Check(d.out.active_mission==mission,"near recovery escaped to LINE after camera/observation changes");
}

// Initial near qualification is stationary; when its fresh 10-frame window
// fails, only an off-center remembered BALL READY may trigger TWO walk-stance
// turns. Both must DONE before any new stationary observation or camera change.
void BallNearMissingReadyTurnsAndCentralFallback(bool side) {
  auto c=Config();
  c.ball.recovery_max_observations=5;
  Driver d(c);
  const double u=side ? 500.0 : 320.0;
  d.in.ball_target=Target(u,100);
  d.Warm();d.Block();d.Block();
  d.in.ball_target=Target(u,260);
  for(int n=0;n<3;++n)d.Tick();
  Check(d.out.approach_near,"near loss turn fixture failed to latch near");
  d.Tick({d.current,true,false,true});
  d.Tick({d.current,true,true,false});
  d.in.ball_target.reset();
  bool saw_turn=false;
  for(int n=0;n<25 && !saw_turn;++n) {
    d.Tick({},.1);
    if(d.out.command.action_id &&
       d.out.command.action==MissionAction::kStepForwardRightOneHurdle)
      saw_turn=true;
  }
  if(!side) {
    Check(!saw_turn,"central remembered BALL READY caused a loss turn");
  } else {
    Check(saw_turn,"off-center remembered BALL READY did not cause first recovery turn");
    const auto first=d.out.command.action_id;
    d.Tick({first,true,false,false});
    d.Tick({first,true,true,false});
    Check(d.out.command.action==MissionAction::kStepForwardRightOneHurdle &&
          d.out.command.action_id!=first,
          "near loss turn sequence did not issue its second walk-stance turn");
    const auto second=d.out.command.action_id;
    d.Tick({second,true,false,false});
    d.Tick({second,true,true,false});
    Check(d.out.command.camera_request==CameraRequest::kNone,
          "BALL near loss lowered camera before post-turn stationary observation");
    for(int n=0;n<17;++n) {
      d.Tick({},.1);
      Check(d.out.command.action!=MissionAction::kStepForwardRightOneHurdle,
            "BALL near loss repeated turns beyond exactly two");
    }
  }
  bool down=false;
  for(int n=0;n<100 && !down;++n) {
    d.Tick({},.1);
    down=d.out.active_mission==MissionType::kBall &&
        d.out.command.camera_request==CameraRequest::kBallView;
  }
  Check(down,"near loss retries failed to enter the ordinary BALL_VIEW path");
}

void NearGoalPreemptsDepthWalk() {
  auto c=Config();c.enable_ball=false;c.enable_goal=true;
  c.initial_has_ball=c.goal_trial_start_goal=true;c.goal.lost_frames=3;
  Driver d(c);d.in.ball_target.reset();d.in.line_centers.clear();
  d.in.camera_feedback={CameraMode::kGoal,true};d.in.backboard_image_without_depth=true;
  for(int i=0;i<30;++i){
    d.Tick();
    Check(d.out.command.action==MissionAction::kNone,
          "first GOAL sighting with invalid Depth must not issue a blind WALK");
  }
}

void SoftGoalLossClearsDepthEpisode() {
  auto c=Config();c.enable_ball=false;c.enable_goal=true;
  c.initial_has_ball=c.goal_trial_start_goal=true;c.goal.lost_frames=3;
  c.line_p2p.straight_repetitions=1;
  Driver d(c);d.in.ball_target.reset();d.in.line_centers.clear();
  d.in.camera_feedback={CameraMode::kGoal,true};d.in.backboard_target=Target();
  d.in.backboard_track_id=9;d.in.backboard_center_depth_m=2;
  d.in.raw_backboard_u=320;d.in.goal_pose={true,0,2,0,1};
  for(int n=0;n<12;++n)d.Tick();
  const auto unit=d.out.command.action_id;
  Check(unit && d.out.approach_mission==MissionType::kGoal,
        "soft-loss fixture did not acquire GOAL");
  d.Tick({unit,true,false,false});
  for(int n=0;n<12;++n)d.Tick();
  d.in.backboard_target.reset();
  d.in.backboard_center_depth_m.reset();
  d.in.goal_pose={};
  d.in.backboard_image_without_depth=true;
  d.Tick({unit,true,true,false}); // exactly one physical GOAL walk DONE
  for(int n=0;n<100 && d.out.command.action!=MissionAction::kStepForwardOne;++n)
    d.Tick({},.1);
  const auto walk=d.out.command.action_id;
  Check(walk && d.out.command.action==MissionAction::kStepForwardOne,
        "post-acquisition image-only GOAL did not issue a bounded Depth probe");
  d.Tick({walk,true,false,false});
  // YOLO loss during an ACCEPTED Depth WALK cannot cancel its real DONE.
  d.in.backboard_image_without_depth=false;
  d.in.raw_backboard_u.reset();
  for(int n=0;n<3;++n)d.Tick();
  Check(d.out.command.action_id==walk,
        "brief YOLO loss cancelled an accepted Depth WALK");
  d.Tick({walk,true,true,false});
  bool scan=false;
  for(int n=0;n<60 && !scan;++n){
    d.Tick({},.1);
    scan=d.out.command.camera_request==CameraRequest::kGoalScanLeft;
  }
  Check(scan,"YOLO disappeared after Depth WALK but ordinary GOAL head scan did not start");
}

void HurdleLineLossUsesLineRecovery() {
  auto c=Config();c.enable_ball=false;c.enable_hurdle=true;
  Driver d(c);d.in.ball_target.reset();d.in.hurdle_target=Target();d.in.hurdle_track_id=5;
  d.Warm();d.in.line_centers.clear();
  for(int i=0;i<15;++i)d.Tick();
  d.Tick({d.current,true,true,false});
  d.current=d.out.command.action_id;
  Check(d.current && LinePlan::IsUnit(d.out.command.action),
        "HURDLE fixture did not finish the first unit of its walking block");
  d.Tick({d.current,true,false,false});
  for(int i=0;i<15;++i)d.Tick();
  d.Tick({d.current,true,false,true});
  d.Tick({d.current,true,true,false});
  d.current=d.out.command.action_id;
  Check(d.current && LinePlan::IsUnit(d.out.command.action),
        "qualified earlier LINE block did not start the next block");
  for(int n=0;n<2;++n) {
    d.Tick({d.current,true,false,false});
    for(int i=0;i<15;++i)d.Tick();
    d.Tick({d.current,true,false,true});
    const auto next=d.out.command.action_id;
    d.Tick({d.current,true,true,false});
    if(n==0) {
      Check(next && next!=d.current,"first unit did not continue its block");
      d.current=next;
    }
  }
  bool recovering=d.out.line_in_recovery;
  for(int i=0;i<35 && !recovering;++i) {d.Tick({},.1);recovering=d.out.line_in_recovery;}
  Check(recovering && !d.out.terminal_hold,
        "visible far HURDLE with missing floor LINE stayed in router HOLD forever");
}

void FarReacquisitionReturnsToSharedPath(MissionType mission) {
  auto c=Config();c.enable_ball=mission==MissionType::kBall;
  if(mission==MissionType::kGoal) c.goal.head_scan_enabled=false;
  c.enable_hurdle=mission==MissionType::kHurdle;c.enable_goal=mission==MissionType::kGoal;
  c.initial_has_ball=c.goal_trial_start_goal=mission==MissionType::kGoal;
  Driver d(c);
  auto visible=[&](bool value) {
    d.in.ball_target.reset();d.in.hurdle_target.reset();d.in.backboard_target.reset();
    if(mission==MissionType::kBall) {if(value)d.in.ball_target=Target();}
    else if(mission==MissionType::kHurdle) {if(value)d.in.hurdle_target=Target();d.in.hurdle_track_id=5;}
    else {
      if(value)d.in.backboard_target=Target();d.in.backboard_track_id=9;
      d.in.backboard_center_depth_m=value ? std::optional<double>(2) : std::nullopt;
      d.in.goal_pose=value ? GoalPoseObservation{true,0,2,0,1} : GoalPoseObservation{};
    }
  };
  visible(true);
  if(mission==MissionType::kGoal) {
    d.in.camera_feedback={CameraMode::kGoal,true};d.in.line_centers.clear();
    for(int i=0;i<12;++i)d.Tick();d.current=d.out.command.action_id;
    Check(d.current!=0,"GOAL far recovery fixture never started");d.Tick({d.current,true,false,false});
  } else d.Warm();
  for(int i=0;i<5 && !d.out.approach_locked;++i)d.Block();
  Check(d.out.approach_locked,"far recovery fixture did not lock the object");
  const auto fail_block=[&] {
    visible(false);
    for(int i=0;i<12;++i)d.Tick();
    d.Tick({d.current,true,false,true});
    const auto second=d.out.command.action_id;
    Check(second && second!=d.current && LinePlan::IsUnit(d.out.command.action),
          "missing far object cancelled a walking block after one unit");
    d.Tick({second,true,false,false});d.Tick({d.current,true,true,false});d.current=second;
    for(int i=0;i<12;++i)d.Tick();
    d.Tick({d.current,true,false,true});
    Check(d.out.command.action_id==d.current,
          "missing far object booked another block without 10/n evidence");
    d.Tick({d.current,true,true,false});
  };
  fail_block();
  Check(d.out.active_mission==mission,"locked far loss did not start bounded object recovery");
  const int ball_index=d.out.ball_mission_index,goal_index=d.out.goal_mission_index;
  visible(true);
  std::uint64_t pending=0,done=0;bool resumed=false;
  for(int i=0;i<160;++i) {
    CommandDeliveryFeedback fb;
    const auto id=d.out.command.action_id;
    if(pending) {
      done=pending;fb={pending,true,true,false};pending=0;
    } else if(id && id!=done && d.out.command.command_type==CommandType::kAction && !LinePlan::IsUnit(d.out.command.action)) {
      pending=id;fb={id,true,false,false};
    }
    d.Tick(fb,.1);
    if(d.out.active_mission==MissionType::kLine && d.out.approach_mission==mission && LinePlan::IsUnit(d.out.command.action)) {
      resumed=true;break;
    }
  }
  Check(resumed && d.out.approach_locked && d.out.approach_completed_blocks>=3,
        "confirmed far reacquisition did not resume 32/33/34 with the same lock/block history");
  Check(d.out.ball_mission_index==ball_index && d.out.goal_mission_index==goal_index,
        "resuming virtual approach changed the object's mission sequence number");
  d.current=d.out.command.action_id;d.Tick({d.current,true,false,false});
  fail_block();
  Check(d.out.active_mission==mission && d.out.ball_mission_index==ball_index && d.out.goal_mission_index==goal_index,
        "a repeated far loss admitted a new mission instead of preserving the existing mission");
}
void InitialLineMeanProbeAfterIdentityExpiry() {
  // Admission already qualified inside the current physical LINE block.
  // Its READY must use its own cached BALL center, even when RGB is now gone.
  auto c=Config();
  c.line_p2p.left_repetitions=c.line_p2p.straight_repetitions=
      c.line_p2p.right_repetitions=1;
  Driver d(c);
  d.in.ball_track_id=12;
  d.in.ball_entry_mean_center=Point2{320,100};
  d.Warm();
  Check(d.out.approach_mission==MissionType::kBall &&
        d.out.approach_completed_blocks==0,
        "BALL admission was not active before first LINE DONE");
  d.in.ball_target.reset();
  for(int i=0;i<18;++i)d.Tick();
  d.Tick({d.current,true,false,true});
  const auto next=d.out.command.action_id;
  Check(next && next!=d.current && LinePlan::IsUnit(d.out.command.action) &&
        !d.out.approach_probe_active && !d.out.post_motion_observing,
        "first BALL-admitted LINE READY failed to reserve its normal successor");
  d.Tick({next,true,false,false});
  d.Tick({d.current,true,true,false});
  Check(d.out.approach_completed_blocks==1 && !d.out.approach_locked,
        "admitted LINE first DONE did not count as exactly one BALL block");
  d.current=next;
  for(int i=0;i<12;++i)d.Tick();
  d.Tick({d.current,true,false,true});
  const auto probe=d.out.command.action_id;
  Check(probe && probe!=d.current && d.out.approach_probe_active &&
        LinePlan::IsUnit(d.out.command.action),
        "first lost BALL-owned block failed to reserve one remembered READY probe");
  d.Tick({probe,true,false,false});
  d.Tick({d.current,true,true,false});
  d.current=probe;
  for(int i=0;i<12;++i)d.Tick();
  d.Tick({probe,true,false,true});
  d.Tick({probe,true,true,false});
  Check(d.out.approach_mission==MissionType::kNone,
        "failed probe did not return to LINE after the first admitted LINE block");
}
void FirstBallAdmissionReadyOverwritesInitialMean() {
  auto c=Config();
  c.line_p2p.straight_repetitions=1;
  c.ball_candidate_tracking.max_center_distance_px=800.0;
  Driver d(c);
  d.in.ball_target=Target(90,100);
  d.in.ball_entry_mean_center=Point2{90,100};
  d.Warm();
  for(int i=0;i<20;++i) {
    d.in.ball_target=Target(600,100);
    d.Tick();
  }
  d.Tick({d.current,true,false,true});
  const auto successor=d.out.command.action_id;
  Check(successor && successor!=d.current &&
        (d.out.command.action==MissionAction::kLineRightUnit ||
         d.out.command.action==MissionAction::kLineRightUnitTurnFirst),
        "new final READY mean was not used before selecting BALL successor");
  Check(d.out.approach_completed_blocks==0,
        "READY counted the admitted LINE block before physical DONE");
  d.Tick({successor,true,false,false});
  d.Tick({d.current,true,true,false});
  Check(d.out.approach_completed_blocks==1 && !d.out.approach_locked,
        "admitted LINE block was not counted exactly once on its actual DONE");
}
void RelaxedHurdleBackboardContinuousIdentity() {
  // The per-object candidate tracker already permits 800px / 100x size,
  // so the continuous router must not reject a stable ID using the stricter
  // common association limits.
  for (const auto mission : {MissionType::kHurdle,MissionType::kGoal}) {
    auto c=Config();
    c.enable_ball=false;
    c.enable_hurdle=mission==MissionType::kHurdle;
    c.enable_goal=mission==MissionType::kGoal;
    c.initial_has_ball=c.goal_trial_start_goal=mission==MissionType::kGoal;
    c.object_association.max_center_distance_norm=.25;
    c.object_association.max_size_log_ratio=.80;
    c.hurdle_candidate_tracking.max_center_distance_px=800;
    c.hurdle_candidate_tracking.max_size_log_ratio=std::log(100.0);
    c.backboard_candidate_tracking.max_center_distance_px=800;
    c.backboard_candidate_tracking.max_size_log_ratio=std::log(100.0);
    Driver d(c);
    d.in.ball_target.reset();
    if (mission==MissionType::kHurdle) {
      d.in.hurdle_target=Target(200,100,4);
      d.in.hurdle_track_id=5;
    } else {
      d.in.camera_feedback={CameraMode::kGoal,true};
      d.in.line_centers.clear();
      d.in.backboard_target=Target(200,100,3);
      d.in.backboard_track_id=9;
      d.in.backboard_center_depth_m=2;
      d.in.goal_pose={true,0,2,0,1};
    }
    for (int i=0;i<12;++i) d.Tick();
    Check(d.out.approach_mission==mission && d.out.approach_missing_frames==0,
          "HURDLE/GOAL initial identity fixture did not acquire target");
    auto larger=Target(530,100,mission==MissionType::kHurdle?4:3);
    larger.box_px={430,0,200,200};
    larger.width_px=larger.height_px=200;
    larger.area_px=40000;
    if (mission==MissionType::kHurdle) d.in.hurdle_target=larger;
    else d.in.backboard_target=larger;
    d.Tick();
    Check(d.out.approach_mission==mission && d.out.approach_missing_frames==0 &&
          !d.out.approach_loss_pending,
          "HURDLE/GOAL same-ID candidate was rejected by a second, stricter identity gate");
  }
}

void AcquiredGoalLossScansBeforeLineReturn() {
  auto c=Config();
  c.enable_ball=false;c.enable_goal=true;c.initial_has_ball=true;
  c.goal_trial_start_goal=true;c.goal.lost_frames=3;
  c.goal.head_scan_enabled=true;
  Driver d(c);
  d.in.ball_target.reset();d.in.line_centers.clear();
  d.in.camera_feedback={CameraMode::kGoal,true};
  d.in.backboard_target=Target(280,100);d.in.backboard_track_id=9;
  d.in.backboard_center_depth_m=2;
  d.in.goal_pose={true,-.1,2,0,1};
  for(int i=0;i<12;++i)d.Tick();
  const auto first=d.out.command.action_id;
  Check(first && d.out.approach_mission==MissionType::kGoal,
        "acquired-loss scan fixture did not start GOAL approach");
  d.Tick({first,true,false,false});
  d.in.backboard_target.reset();
  d.in.backboard_center_depth_m.reset();d.in.goal_pose={};
  for(int i=0;i<12;++i)d.Tick();
  d.Tick({first,true,false,true});
  const auto second=d.out.command.action_id;
  Check(second && second!=first && LinePlan::IsUnit(d.out.command.action),
        "GOAL scan test accidentally interrupted accepted first block");
  d.Tick({second,true,false,false});
  d.Tick({first,true,true,false});
  for(int i=0;i<12;++i)d.Tick();
  d.Tick({second,true,false,true});
  Check(d.out.command.camera_request==CameraRequest::kNone,
        "GOAL head scan started while a walking unit was still active");
  d.Tick({second,true,true,false});
  bool started=false;
  for(int i=0;i<30 && !started;++i){
    d.Tick();
    started=d.out.command.camera_request==CameraRequest::kGoalScanLeft;
  }
  Check(started,"acquired far GOAL loss skipped left head scan and returned to LINE");
  d.in.camera_scan.phase=1;d.in.camera_scan.finished=false;
  d.Tick();
  d.in.camera_scan.phase=4;d.in.camera_scan.finished=true;
  d.Tick();
  d.in.camera_scan.phase=0;d.in.camera_scan.finished=false;
  bool right=false;
  for(int i=0;i<8 && !right;++i){
    d.Tick();right=d.out.command.camera_request==CameraRequest::kGoalScanRight;
  }
  Check(right,"GOAL lost-object scan never tried right after failed left");
  d.in.camera_scan.phase=1;d.in.camera_scan.finished=false;d.Tick();
  d.in.camera_scan.phase=4;d.in.camera_scan.finished=true;d.Tick();
  d.in.camera_scan.phase=0;d.in.camera_scan.finished=false;
  bool returned=false;
  for(int i=0;i<12 && !returned;++i){
    d.Tick();returned=d.out.command.camera_request==CameraRequest::kBallViewLine;
  }
  Check(returned,"GOAL scan exhausted both sides without BALL_VIEW_LINE fallback");
}
void GoalAutoThreePixelZones() {
  for(const auto &item:std::array<std::pair<double,MissionAction>,3>{{
       {179.0,MissionAction::kLineLeftUnit},
       {280.0,MissionAction::kLineStraightUnit},
       {381.0,MissionAction::kLineRightUnitTurnFirst}}}) {
    auto c=Config();
    c.enable_ball=false;c.enable_goal=true;c.initial_has_ball=true;
    c.goal_trial_start_goal=true;c.line_p2p.right_unit_action=36;
    c.goal.auto_approach_center_u_px=280;
    c.goal.auto_approach_half_width_px=100;
    Driver d(c);d.in.ball_target.reset();d.in.line_centers.clear();
    d.in.camera_feedback={CameraMode::kGoal,true};
    d.in.backboard_target=Target(item.first,100);d.in.backboard_track_id=7;
    d.in.backboard_center_depth_m=2;d.in.goal_pose={true,0,2,0,1};
    for(int i=0;i<12;++i)d.Tick();
    Check(d.out.command.action==item.second,
        "GOAL AUTO 180/380 pixel boundaries did not select left/straight/right");
  }
}
void GoalAutoReferenceIsIsolated() {
  for(double legacy_reference : {310.0,380.0}) {
    auto c=Config();
    c.enable_ball=false;c.enable_goal=true;c.initial_has_ball=true;
    c.goal_trial_start_goal=true;
    c.goal.auto_approach_reference_u_px=legacy_reference;
    c.goal.auto_approach_center_u_px=280.0;
    c.goal.auto_approach_half_width_px=100.0;
    Driver d(c);d.in.ball_target.reset();d.in.line_centers.clear();
    d.in.camera_feedback={CameraMode::kGoal,true};
    d.in.backboard_target=Target(500,100);d.in.backboard_track_id=7;
    d.in.backboard_center_depth_m=2;d.in.goal_pose={true,.6,2,0,1};
    for(int i=0;i<12;++i)d.Tick();
    Check(d.out.approach_path_guide && d.out.approach_path_guide->valid &&
          std::abs(d.out.approach_path_reference_px.u-280.0)<1e-9,
          "GOAL AUTO leaked legacy virtual H reference into zone steering");
  }
  const auto right=ObjectZoneGuide(Target(381,100),180,380,15,640);
  const auto center=ObjectZoneGuide(Target(280,400),180,380,15,640);
  const auto left=ObjectZoneGuide(Target(179,470),180,380,15,640);
  Check(right.heading_rad>0 && center.heading_rad==0 &&
        left.heading_rad<0,"GOAL vertical-zone decision depended on target v");
  auto b=Config().ball;
  b.approach_left_top_u_px=345.0;
  b.approach_right_top_u_px=415.0;
  const auto upper=b.ApproachBounds(0,480), lower=b.ApproachBounds(479,480);
  Check(upper.first>lower.first && upper.second<lower.second,
        "BALL far bounds do not converge toward image top");
  const auto near=b.ExtremeApproachBounds(260,480);
  Check(near.first<b.ApproachBounds(260,480).first &&
        near.second>b.ApproachBounds(260,480).second,
        "BALL near outer diagonal zones were not added");

}
}
int main() {
  GoalLineReturnThreePointSteering();
  GoalLineReturnSparseRecoveryUnchanged();
  GoalLineThreePointScope();
  RelaxedHurdleBackboardContinuousIdentity();
  GoalAutoReferenceIsIsolated();
  GoalAutoThreePixelZones();
  AcquiredGoalLossScansBeforeLineReturn();
  GoalEdgeTenInFourRawBBox();
  GoalStableNewIdentityKeepsApproach();GoalTrueRgbLossAndRangeRejection();
  Geometry();VirtualHeadingOnly();WeightedProbeDirection();SoftLoss(0);SoftLoss(1);SoftLoss(2);ProbeDisabled();ProbeRediscoveryAndLineMemory();PrelockProbeForMission(MissionType::kHurdle);PrelockProbeForMission(MissionType::kGoal);LockAndLoss();NearAndIdentity();HurdleAndGoal();ManualAndReset();CameraDrainAndProbe();GoalTurnObservationUsesSelectedPath();EmptyGoalRetryLimit();FarGoalDepthRecovery(false);FarGoalDepthRecovery(true);
  BallNearMissingReadyTurnsAndCentralFallback(false);
  BallNearMissingReadyTurnsAndCentralFallback(true);
  NearPixelVotesWithGaps(MissionType::kBall);NearPixelVotesWithGaps(MissionType::kHurdle);
  ReadyCenterAccumulator();InitialLineMeanProbeAfterIdentityExpiry();
  FirstBallAdmissionReadyOverwritesInitialMean();
  NearFinalActionSequence(150,true,true); NearFinalActionSequence(240,true,false);
  NearFinalActionSequence(575,false,true); NearFinalActionSequence(500,false,false);
  NearGoalPreemptsDepthWalk();SoftGoalLossClearsDepthEpisode();HurdleLineLossUsesLineRecovery();
  FarGoalDepthRecovery(false,true);FarGoalDepthRecovery(true,true);
  for(auto mission:{MissionType::kBall,MissionType::kHurdle,MissionType::kGoal}) {
    NearLossBeforeLock(mission);FarReacquisitionReturnsToSharedPath(mission);
  }
}
