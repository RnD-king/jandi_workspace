#include "vision_core/config_loader.hpp"
#include "vision_core/mission_controller.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <algorithm>
#include <cassert>
#include <limits>
using namespace vision_core;
namespace {
ObjectTargetCandidate Candidate(double u,double v=120,double size=40,std::size_t index=0) {
  ObjectTarget t;
  t.class_id=1; t.confidence=.9; t.center_px={u,v}; t.rectified_center_px=t.center_px;
  t.width_px=t.height_px=size; t.area_px=size*size; t.box_px={u-size/2,v-size/2,size,size};
  return {t,index};
}
PerceptionDetection DetectionAt(double u,int cls=1,double w=40,double h=40,
                                std::optional<double> depth=1.0,double v=120) {
  PerceptionDetection d;
  d.detection={ {u-w/2,v-h/2,w,h},.9,cls}; d.center_depth_m=depth;
  return d;
}
void TrackletRegressions() {
  const auto c=LoadDefaultAlgorithmConfig();
  BallCandidateTracker tracker;
  const auto update=[&](std::vector<ObjectTargetCandidate> ds) {
    tracker.Update(ds,c.ball_candidate_tracking,c.object_association,10,470);
  };
  for(int i=0;i<6;++i) update({Candidate(i%2 ? 520 : 100)});
  assert(tracker.Tracks().size()==2);
  for(const auto& t:tracker.Tracks()) assert(t.HitCount()==3 && t.hits.size()<=6);
  update({Candidate(100,120,40,0),Candidate(520,120,40,1)});
  for(const auto& t:tracker.Tracks()) assert(t.HitCount()==4);
  const auto a_id=tracker.Tracks().front().id;
  // One ambiguous detection can increase only one identity's count.
  const int before=tracker.Tracks()[0].HitCount()+tracker.Tracks()[1].HitCount();
  update({Candidate(100)});
  assert(tracker.Tracks()[0].HitCount()+tracker.Tracks()[1].HitCount()==before+1);
  BallCandidateTracker overlap;
  overlap.Update({Candidate(100),Candidate(180,120,40,1)},c.ball_candidate_tracking,c.object_association,10,470);
  overlap.Update({Candidate(140)},c.ball_candidate_tracking,c.object_association,10,470);
  assert(overlap.Tracks().size()==2);
  assert(overlap.Tracks()[0].HitCount()+overlap.Tracks()[1].HitCount()==3);
  // All five missing frames are false, retaining identity for reappearance.
  for(int i=0;i<5;++i) update({});
  assert(tracker.Find(a_id));
  update({Candidate(105)});
  assert(tracker.Find(a_id) && tracker.Find(a_id)->detection_index);
  assert(tracker.Find(a_id)->HitCount()==4); // Last ten: three earlier hits + this hit.
  for(int i=0;i<6;++i) update({});
  assert(!tracker.Find(a_id));
  update({Candidate(105)});
  assert(tracker.Best()->id>a_id && tracker.Best()->HitCount()==1);
  const auto id=tracker.Best()->id;
  update({Candidate(105,120,100)}); // log(2.5) exceeds 0.50.
  assert(tracker.Tracks().size()==2 && tracker.Find(id)->missing_frames==1);
  tracker.Reset();
  update({Candidate(100),Candidate(181,120,40,1)});
  assert(tracker.Tracks().size()==2);
  // Six detections before the first full window must already qualify.
  tracker.Reset();
  for(int i=0;i<6;++i) update({Candidate(100)});
  assert(tracker.Best()->HitCount()==6 && tracker.Best()->hits.size()==6);
  // Entry-row failures still update identity but never count for admission.
  for(int i=0;i<10;++i) update({Candidate(100,480)});
  assert(tracker.Best()->HitCount()==0);
  BallCandidateTracker forward,reverse;
  for(int i=0;i<10;++i) {
    const std::vector<ObjectTargetCandidate> ds={Candidate(100+i,120,40,0),Candidate(500-i,120,40,1)};
    auto reversed=ds; std::reverse(reversed.begin(),reversed.end());
    forward.Update(ds,c.ball_candidate_tracking,c.object_association,10,470);
    reverse.Update(reversed,c.ball_candidate_tracking,c.object_association,10,470);
  }
  assert(forward.Tracks().size()==reverse.Tracks().size());
  for(std::size_t i=0;i<forward.Tracks().size();++i) {
    assert(forward.Tracks()[i].id==reverse.Tracks()[i].id);
    assert(forward.Tracks()[i].target.center_px.u==reverse.Tracks()[i].target.center_px.u);
  }
  auto bounded=c.ball_candidate_tracking; bounded.max_tracks=2;
  tracker.Reset();
  tracker.Update({Candidate(100),Candidate(300),Candidate(500)},bounded,c.object_association,10,470);
  assert(tracker.Tracks().size()==2);
}
void LockedAssociationDoesNotAutoAcquire() {
  auto c=LoadDefaultAlgorithmConfig();
  ObjectAssociationTracker locked(c.object_association),legacy(c.object_association);
  locked.Seed(Candidate(100).target); legacy.Seed(Candidate(100).target);
  for(int i=0;i<20;++i) assert(!locked.Update({Candidate(520)},640,480,false).target);
  assert(locked.HasIdentity() && locked.MissingFrames()==20);
  assert(locked.Update({Candidate(105)},640,480,false).target);
  for(int i=0;i<6;++i) legacy.Update({Candidate(520)},640,480);
  assert(legacy.MissingFrames()==0); // Other missions keep legacy acquisition.
}
void RecoveryIdentityChangeRetainsBudget() {
  StationaryObjectRecovery recovery;
  recovery.Begin(0);
  for(int i=0;i<6;++i)
    assert(recovery.Observe(true,.1*i,10,7,1,2)==StationaryObjectRecovery::Result::kWaiting);
  recovery.DiscardDetections();
  StationaryObjectRecovery::Result result=StationaryObjectRecovery::Result::kWaiting;
  for(int i=6;i<=10;++i) result=recovery.Observe(true,.1*i,10,7,1,2);
  assert(result==StationaryObjectRecovery::Result::kRetry);
  assert(recovery.Completed()==1); // Six detections of old identity cannot help B.
  recovery.DiscardDetections();
  for(int i=11;i<=20;++i) result=recovery.Observe(false,.1*i,10,7,1,2);
  assert(result==StationaryObjectRecovery::Result::kFailed && recovery.Completed()==2);
}
void FilterRegressions() {
  auto c=LoadDefaultAlgorithmConfig();
  c.enable_ball=c.enable_hurdle=c.enable_goal=false;
  PerceptionFrameInput in; in.image_width=640; in.image_height=480;
  in.advance_line_fsm=false; in.camera_feedback={CameraMode::kForward,true};
  const auto accepts=[&](PerceptionDetection d,CameraMode mode=CameraMode::kForward) {
    MissionController controller(c);
    in.camera_feedback={mode,true}; in.detections={d};
    const auto out=controller.StepPerception(in);
    return d.detection.class_id==1 ? out.perception.targets.ball.has_value() :
        d.detection.class_id==4 ? out.perception.targets.hurdle.has_value() : out.perception.targets.backboard.has_value();
  };
  assert(accepts(DetectionAt(300,1,40,40,2.5)));
  assert(!accepts(DetectionAt(300,1,40,40,2.501)));
  assert(!accepts(DetectionAt(300,1,80,40,1.0)));
  assert(!accepts(DetectionAt(300,1,20,40,1.0)));
  assert(accepts(DetectionAt(300,1,60,40,1.0))); // Inclusive ratio 1.5.
  c.object_targets.ball_down_max_depth_m=.8;
  assert(accepts(DetectionAt(300,1,40,40,1.0),CameraMode::kForward));
  assert(!accepts(DetectionAt(300,1,40,40,1.0),CameraMode::kDown));
  assert(accepts(DetectionAt(300,1,40,40,.8),CameraMode::kDown));
  for(auto depth:{std::optional<double>{},std::optional<double>{0},
      std::optional<double>{std::numeric_limits<double>::quiet_NaN()},
      std::optional<double>{std::numeric_limits<double>::infinity()}})
    assert(accepts(DetectionAt(300,1,40,40,depth)));
  c.object_targets.ball_allow_missing_depth=false;
  assert(!accepts(DetectionAt(300,1,40,40,std::nullopt)));
  c.object_targets.ball_depth_enabled=false;
  assert(accepts(DetectionAt(300,1,40,40,100.0)));
  for(double z:{.3,2.0}) assert(accepts(DetectionAt(300,4,200,80,z)));
  for(double z:{.299,2.001}) assert(!accepts(DetectionAt(300,4,200,80,z)));
  assert(!accepts(DetectionAt(300,4,200,81,1.0))); // Below ratio 2.5.
  assert(accepts(DetectionAt(300,4,500,10,1.0))); // No upper ratio gate.
  assert(accepts(DetectionAt(300,4,200,80,std::nullopt)));
  c.object_targets.hurdle_allow_missing_depth=false;
  assert(!accepts(DetectionAt(300,4,200,80,std::nullopt)));
  // BACKBOARD retains mandatory depth and its original range.
  assert(!accepts(DetectionAt(300,3,40,40,std::nullopt)));
  assert(!accepts(DetectionAt(300,3,40,40,2.51)));
  assert(accepts(DetectionAt(300,3,40,40,2.5)));
  // Real retuning must replace acceptance, rather than updating only ROS values.
  c.object_targets.ball_depth_enabled=true;
  MissionController live(c);
  in.camera_feedback={CameraMode::kForward,true}; in.detections={DetectionAt(300)};
  assert(live.StepPerception(in).perception.targets.ball);
  c.object_targets.ball_forward_max_depth_m=.5;
  live.SetLivePerceptionTuning(c);
  assert(!live.StepPerception(in).perception.targets.ball);
  c.object_targets.ball_forward_max_depth_m=2.5;
  assert(live.UpdateTuning(c));
  assert(live.StepPerception(in).perception.targets.ball);
}
struct Pipeline {
  MissionControllerConfig config;
  MissionController controller;
  PerceptionFrameInput input;
  PerceptionMissionFrameResult result;
  Pipeline():config(LoadDefaultAlgorithmConfig()),controller(Config(config)) {
    input.image_width=640; input.image_height=480;
    input.camera_feedback={CameraMode::kForward,true};
    input.now_sec=3; input.advance_line_fsm=false;
    input.allow_new_body_actions=false;
    input.command_transport_enabled=true;
  }
  static MissionControllerConfig Config(MissionControllerConfig& c) {
    c.enable_hurdle=c.enable_goal=false; c.transition_settle_sec=0;
    // These scenarios exercise production's 6/10 entry contract, independently
    // of the historical fixture's seven-hit entry threshold.
    c.ball.entry_min_hits=6; c.ball.stable_window=10; c.ball.stable_min_hits=7;
    c.ball.recovery_max_steps=0; // These motion-disabled tests isolate identity revalidation.
    return c;
  }
  void Tick(std::vector<PerceptionDetection> ds) {
    input.now_sec+=.1; input.detections=std::move(ds); result=controller.StepPerception(input);
  }
  void Admit() {
    for(int i=0;i<18;++i) Tick({DetectionAt(120)});
    assert(result.mission.active_mission==MissionType::kBall);
    assert(result.perception.ball_target_locked && result.perception.ball_target_id);
  }
};
void EntryAndRecoveryRegressions() {
  Pipeline alternating;
  for(int i=0;i<10;++i) alternating.Tick({DetectionAt(i%2 ? 520 : 120)});
  assert(alternating.result.mission.candidate_mission==MissionType::kNone);
  assert(alternating.result.mission.active_mission==MissionType::kLine);
  Pipeline majority;
  for(bool a:{true,true,false,true,false,true,false,true,false,true})
    majority.Tick({DetectionAt(a ? 120 : 520)});
  assert(majority.result.mission.candidate_mission==MissionType::kBall);
  assert(majority.result.perception.targets.ball->center_px.u==120);
  Pipeline bad_validation;
  for(int i=0;i<6;++i) bad_validation.Tick({DetectionAt(120)});
  assert(bad_validation.result.mission.candidate_mission==MissionType::kBall);
  assert(bad_validation.result.mission.active_mission==MissionType::kLine);
  // Stop at the initial observation's decision edge. Subsequent search is
  // explicitly allowed to observe a different provisional target.
  for(int i=0;i<10;++i) bad_validation.Tick({DetectionAt(520)});
  assert(bad_validation.result.mission.active_mission==MissionType::kLine);
  assert(!bad_validation.result.perception.targets.ball);
  assert(bad_validation.result.mission.post_motion_valid==0);
  Pipeline good;
  good.Admit();
  const auto id=good.result.perception.ball_target_id;
  auto a=DetectionAt(125),b=DetectionAt(520); a.detection.confidence=.61; b.detection.confidence=.99;
  good.Tick({b,a});
  assert(good.result.perception.targets.ball->confidence==.61);
  assert(good.result.perception.ball_target_id==id);
  // Clearing PAUSE/RESUME observation history must preserve the lock.
  good.controller.ClearObservationEvidence();
  good.Tick({b});
  assert(!good.result.perception.targets.ball && good.result.perception.ball_target_id==id);
  // Existing five-miss handling enters the existing recovery FSM.
  for(int i=0;i<4;++i) good.Tick({b});
  assert(good.result.mission.ball.mode==BallMode::kBallRecoveryForward);
  good.Tick({b});
  assert(good.result.perception.ball_reacquiring);
  assert(good.result.mission.ball.mode==BallMode::kBallRecoveryForward);
  // Recovery can formally validate B, with no inherited A hits.
  for(int i=0;i<15;++i) good.Tick({b});
  assert(good.result.mission.ball.mode==BallMode::kApproachBall);
  assert(good.result.perception.ball_target_locked && good.result.perception.ball_target_id!=id);
  good.controller.Reset();
  good.Tick({a});
  assert(!good.result.perception.ball_target_locked);
  assert(good.result.perception.ball_tracks.front().HitCount()==1);
  const auto reset_id=good.result.perception.ball_tracks.front().id;
  good.input.camera_feedback={CameraMode::kTransition,false};
  good.Tick({a});
  assert(!good.result.perception.targets.ball && good.result.perception.ball_tracks.empty());
  good.input.camera_feedback={CameraMode::kForward,true};
  good.Tick({a});
  assert(good.result.perception.ball_tracks.front().HitCount()==1);
  assert(good.result.perception.ball_tracks.front().id>reset_id);
  Pipeline manual;
  manual.Admit();
  const auto locked_id=manual.result.perception.ball_target_id;
  manual.input.manual_observation_control=true;
  manual.input.manual_ready_mean=true;
  manual.input.manual_object_hits=std::deque<bool>(10,true);
  manual.input.ball_target_id=locked_id+100;
  manual.Tick({DetectionAt(120)});
  assert(!manual.result.perception.targets.ball);
  assert(manual.result.mission.command.action_id==0);
  manual.input.ball_target_id=locked_id;
  manual.Tick({DetectionAt(120)});
  assert(manual.result.perception.targets.ball && manual.result.perception.ball_target_id==locked_id);
}
void SharedObjectTrackingRegressions() {
  const auto c=LoadDefaultAlgorithmConfig();
  for(auto tracking:{c.hurdle_candidate_tracking,c.backboard_candidate_tracking}) {
    ObjectTargetTracker tracker;
    auto observe=[&](std::vector<ObjectTargetCandidate> detections,bool recovery=false) {
      return tracker.Observe(detections,tracking,c.object_association,640,480,10,470,true,recovery);
    };
    for(int i=0;i<7;++i)assert(observe({Candidate(100)}).target);
    assert(tracker.Selected()->HitCount()==7);
    const auto id=tracker.TargetId();tracker.Lock();
    assert(tracker.Locked());
    // A different, high-confidence object cannot replace a locked target.
    for(int i=0;i<20;++i) {
      assert(!observe({Candidate(520)}).target);
      assert(tracker.TargetId()==id && tracker.Locked());
    }
    assert(observe({Candidate(105)}).target && tracker.TargetId()==id);
    assert(observe({Candidate(520)},true).target);
    assert(tracker.TargetId()!=id && tracker.Selected()->HitCount()==1);
    assert(tracker.IdentityChanged() && tracker.Reacquiring());
    const auto replacement=tracker.TargetId();
    for(int i=0;i<6;++i)observe({Candidate(520)},true);
    assert(tracker.Selected()->HitCount()==7);tracker.Lock();
    assert(!tracker.Observe({Candidate(520)},tracking,c.object_association,
        640,480,10,470,true,false,true,id).target);
    assert(tracker.Observe({Candidate(520)},tracking,c.object_association,
        640,480,10,470,true,false,true,replacement).target);
    tracker.Reset();observe({Candidate(100)});
    assert(tracker.TargetId()>replacement && tracker.Selected()->HitCount()==1);
  }
  auto strict=c.hurdle_candidate_tracking,loose=c.backboard_candidate_tracking;
  strict.max_center_distance_px=10;loose.max_center_distance_px=60;
  ObjectTargetTracker hurdle,board;
  for(auto* tracker:{&hurdle,&board}) {
    const auto& settings=tracker==&hurdle?strict:loose;
    tracker->Observe({Candidate(100)},settings,c.object_association,640,480,10,470,true,false);
    tracker->Lock();
    const auto result=tracker->Observe({Candidate(130)},settings,c.object_association,640,480,10,470,true,false);
    assert(result.target.has_value()==(tracker==&board));
  }
  strict.max_center_distance_px=loose.max_center_distance_px=200;
  strict.max_size_log_ratio=.1;loose.max_size_log_ratio=1;
  hurdle.Reset();board.Reset();
  for(auto* tracker:{&hurdle,&board}) {
    const auto& settings=tracker==&hurdle?strict:loose;
    tracker->Observe({Candidate(100)},settings,c.object_association,640,480,10,470,true,false);
    tracker->Lock();
    const auto result=tracker->Observe({Candidate(100,120,60)},settings,c.object_association,640,480,10,470,true,false);
    assert(result.target.has_value()==(tracker==&board));
  }
}
void HurdleIdentityEntryAndCameraReset() {
  auto c=LoadDefaultAlgorithmConfig();c.enable_ball=c.enable_goal=false;c.enable_hurdle=true;
  c.transition_settle_sec=0;c.hurdle.stable_window=10;c.hurdle.stable_min_hits=7;
  MissionController controller(c);
  PerceptionFrameInput in;in.image_width=640;in.image_height=480;
  in.camera_feedback={CameraMode::kForward,true};in.command_transport_enabled=true;
  in.advance_line_fsm=false;in.allow_new_body_actions=false;
  PerceptionMissionFrameResult out;
  auto tick=[&](std::vector<PerceptionDetection> detections) {
    in.now_sec+=.1;in.detections=std::move(detections);out=controller.StepPerception(in);
  };
  const auto a=DetectionAt(120,4,200,60),b=DetectionAt(520,4,200,60);
  for(int i=0;i<12;++i)tick({i%2?a:b});
  assert(out.mission.candidate_mission==MissionType::kNone);
  assert(out.perception.hurdle_tracks.size()==2);
  for(const auto& track:out.perception.hurdle_tracks)assert(track.HitCount()==5);
  for(int i=0;i<25;++i)tick({a});
  assert(out.mission.active_mission==MissionType::kHurdle);
  assert(out.perception.hurdle_target_locked);
  const auto id=out.perception.hurdle_target_id;
  for(int i=0;i<10;++i) {
    tick({b,a});assert(out.perception.targets.hurdle->center_px.u==120);
    assert(out.perception.hurdle_target_id==id);
  }
  in.manual_observation_control=true;in.manual_ready_mean=true;
  in.manual_object_hits=std::deque<bool>(10,true);in.hurdle_target_id=id+100;
  tick({a});assert(!out.perception.targets.hurdle && !out.mission.command.action_id);
  in.hurdle_target_id=id;tick({a});assert(out.perception.targets.hurdle);
  in.manual_ready_mean=false;in.manual_observation_control=false;
  in.camera_feedback={CameraMode::kTransition,false};tick({a});
  assert(out.perception.hurdle_tracks.empty() && !out.perception.hurdle_target_id);
  in.camera_feedback={CameraMode::kDown,true};tick({a});
  assert(out.perception.hurdle_target_id>id && !out.perception.hurdle_target_locked);
  assert(out.perception.hurdle_tracks.size()==1 && out.perception.hurdle_tracks.front().HitCount()==1);
}
void HurdleObservationRestartsAfterCamera() {
  auto c=LoadDefaultAlgorithmConfig();c.enable_ball=c.enable_goal=false;c.enable_hurdle=true;
  c.transition_settle_sec=0;c.hurdle.stable_window=10;c.hurdle.stable_min_hits=7;
  MissionController controller(c);
  PerceptionFrameInput in;in.image_width=640;in.image_height=480;
  in.camera_feedback={CameraMode::kForward,true};in.command_transport_enabled=true;
  in.advance_line_fsm=false;in.allow_new_body_actions=false;
  in.detections={DetectionAt(120,4,200,60)};
  PerceptionMissionFrameResult out;
  for(int i=0;i<13;++i) {in.now_sec+=.1;out=controller.StepPerception(in);}
  assert(out.mission.candidate_mission==MissionType::kHurdle && out.mission.post_motion_observing);
  const auto id=out.perception.hurdle_target_id;
  in.camera_feedback={CameraMode::kTransition,false};in.now_sec+=.5;
  out=controller.StepPerception(in);
  assert(out.mission.active_mission==MissionType::kLine);
  in.camera_feedback={CameraMode::kDown,true};in.now_sec+=.1;
  out=controller.StepPerception(in);
  assert(out.perception.hurdle_target_id>id && out.mission.post_motion_valid==1);
  const double begin=in.now_sec;
  for(int i=1;i<=10;++i) {
    in.now_sec=begin+.1*i;out=controller.StepPerception(in);
    if(i<10)assert(out.mission.active_mission==MissionType::kLine);
  }
  assert(out.mission.active_mission==MissionType::kHurdle && out.perception.hurdle_target_locked);
}
void BackboardIdentityAndCameraReset() {
  auto c=LoadDefaultAlgorithmConfig();c.enable_ball=c.enable_hurdle=false;c.enable_goal=true;
  c.goal_trial_start_goal=true;c.transition_settle_sec=0;
  c.goal.stable_window=10;c.goal.stable_min_hits=7;
  MissionController controller(c);
  PerceptionFrameInput in;in.image_width=640;in.image_height=480;
  in.intrinsics={600,600,320,240};in.camera_feedback={CameraMode::kGoal,true};
  in.command_transport_enabled=true;in.allow_new_body_actions=false;
  PerceptionMissionFrameResult out;
  auto tick=[&](std::vector<PerceptionDetection> detections) {
    in.now_sec+=.1;in.detections=std::move(detections);out=controller.StepPerception(in);
  };
  auto a=DetectionAt(180,3,80,40,.6),b=DetectionAt(540,3,80,40,.6);
  a.left_depth_m=a.right_depth_m=.6;b.left_depth_m=b.right_depth_m=.6;
  for(int i=0;i<30;++i)tick({a});
  assert(out.mission.active_mission==MissionType::kGoal && out.perception.backboard_target_locked);
  const auto id=out.perception.backboard_target_id;
  for(int i=0;i<10;++i) {
    tick({b,a});assert(out.perception.targets.backboard->center_px.u==180);
    assert(out.perception.backboard_target_id==id);
  }
  in.manual_observation_control=true;in.manual_ready_mean=true;
  in.manual_object_hits=std::deque<bool>(10,true);in.manual_pose_hits=in.manual_object_hits;
  in.backboard_target_id=id+100;tick({a});
  assert(!out.perception.targets.backboard && !out.mission.command.action_id);
  in.backboard_target_id=id;tick({a});assert(out.perception.targets.backboard);
  in.manual_ready_mean=false;in.manual_observation_control=false;
  in.camera_feedback={CameraMode::kTransition,false};tick({a});
  assert(!out.perception.backboard_target_id && out.perception.backboard_tracks.empty());
  in.camera_feedback={CameraMode::kForward,true};tick({a});
  assert(out.perception.backboard_target_id>id && !out.perception.backboard_target_locked);
  assert(out.perception.backboard_tracks.front().HitCount()==1);
}
void CandidateWinsReadyAndUsesFreshDoneWindow() {
  auto c=LoadDefaultAlgorithmConfig();
  c.enable_hurdle=c.enable_goal=false; c.transition_settle_sec=0;
  c.ball.entry_min_hits=6; c.ball.stable_window=10; c.ball.stable_min_hits=7;
  c.line.line_stable_window=c.line.line_stable_min_hits=1;
  MissionController controller(c);
  PerceptionFrameInput in;
  in.image_width=640; in.image_height=480; in.command_transport_enabled=true;
  in.camera_feedback={CameraMode::kForward,true};
  const auto image=[&](bool ball,double u=120) {
    in.detections.clear();
    for(int i=0;i<5;++i) in.detections.push_back(DetectionAt(c.line_features.image_center_u,c.line_detection.class_id,20,20,std::nullopt,450-50*i));
    if(ball) in.detections.push_back(DetectionAt(u));
  };
  in.now_sec=3; image(false);
  auto out=controller.StepPerception(in);
  const auto walk=out.mission.command.action_id;
  assert(walk && out.mission.command.action==MissionAction::kLineStraightUnit);
  in.now_sec=3.1; in.delivery_feedback={walk,true,false,false}; image(false);
  controller.StepPerception(in);
  for(int i=1;i<=6;++i) {
    in.now_sec=3.1+.1*i; in.delivery_feedback={}; image(true);
    if(i==6) in.delivery_feedback={walk,true,false,true};
    out=controller.StepPerception(in);
  }
  assert(out.mission.active_mission==MissionType::kLine && out.mission.candidate_mission==MissionType::kBall);
  assert(out.mission.command.action_id==walk); // No successor reservation at the same READY.
  assert(!out.mission.post_motion_observing);
  // Movement shifts the selected target 80 pixels before its physical DONE.
  in.now_sec=4; in.delivery_feedback={walk,true,true,false}; image(true,200);
  out=controller.StepPerception(in);
  assert(out.mission.post_motion_observing && out.mission.post_motion_valid==1);
  assert(out.mission.active_mission==MissionType::kLine);
  for(int i=1;i<=10;++i) {
    in.now_sec=4+.1*i; in.delivery_feedback={}; image(i!=2 && i!=4 && i!=6,200);
    out=controller.StepPerception(in);
    if(i<10) assert(out.mission.active_mission==MissionType::kLine);
  }
  assert(out.mission.active_mission==MissionType::kBall && out.perception.ball_target_locked);
  assert(out.mission.post_motion_valid==7);
}
}
void ExtendedCandidateGraceAndDistance() {
  auto config=LoadDefaultAlgorithmConfig();
  auto tracking=config.ball_candidate_tracking;
  tracking.max_center_distance_px=180;tracking.missing_frame_limit=15;
  BallCandidateTracker tracker;
  auto update=[&](std::vector<ObjectTargetCandidate> ds) {
    tracker.Update(ds,tracking,config.object_association,10,470);
  };
  update({Candidate(100)});const auto id=tracker.Best()->id;
  update({Candidate(279)});
  assert(tracker.Find(id) && tracker.Find(id)->target.center_px.u==279);
  for(int i=0;i<15;++i)update({});
  assert(tracker.Find(id) && tracker.Find(id)->missing_frames==15);
  update({Candidate(280)});assert(tracker.Find(id)->missing_frames==0);
  for(int i=0;i<16;++i)update({});
  assert(!tracker.Find(id));
  update({Candidate(280)});assert(tracker.Best()->id!=id);
}
int main() {
  ExtendedCandidateGraceAndDistance();
  SharedObjectTrackingRegressions();HurdleIdentityEntryAndCameraReset();BackboardIdentityAndCameraReset();
  HurdleObservationRestartsAfterCamera();
  TrackletRegressions(); LockedAssociationDoesNotAutoAcquire();
  RecoveryIdentityChangeRetainsBudget();
  FilterRegressions(); EntryAndRecoveryRegressions();
  CandidateWinsReadyAndUsesFreshDoneWindow();
}
