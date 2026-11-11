#include "vision_core/config_loader.hpp"
#include "vision_core/mission_controller.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cstdio>
using namespace vision_core;


void ZeroDelayCurveRegression() {
  auto cfg=LoadDefaultAlgorithmConfig();
  cfg.enable_ball=true;cfg.enable_goal=cfg.enable_hurdle=false;
  cfg.curve_search.enabled=true;cfg.curve_search.scan_delay_walk_sequences=0;
  cfg.curve_search.confirm_extra_sequences=1;
  cfg.line_p2p.left_repetitions=cfg.line_p2p.straight_repetitions=
      cfg.line_p2p.right_repetitions=1;
  MissionController core(cfg);MissionFrameInput in;MissionFrameResult out;
  in.image_width=640;in.image_height=480;in.intrinsics={600,600,320,240};
  in.command_transport_enabled=true;
  in.camera_feedback={CameraMode::kForward,true};
  in.line_centers={{380,450},{380,400},{380,350},{380,300},{430,250}};
  auto tick=[&](CommandDeliveryFeedback fb={}) {
    in.now_sec+=.07;in.delivery_feedback=fb;out=core.Step(in);
  };
  tick();
  for(int block=0;block<2;++block) {
    const auto id=out.command.action_id;
    assert(id && out.command.action==MissionAction::kLineStraightUnit);
    tick({id,true,false,false});
    for(int j=0;j<12;++j)tick();
    tick({id,true,false,true});
    if(block==1)assert(out.command.action_id==id); // No third block queued at READY.
    tick({id,true,true,false});
    if(block==0)assert(out.curve_search_phase==1);
  }
  assert(out.curve_search_phase==4);
  assert(out.command.camera_request==CameraRequest::kScanLeft);
}

int main() {
  auto cfg=LoadDefaultAlgorithmConfig();
  cfg.enable_ball=true; cfg.enable_hurdle=false; cfg.enable_goal=false;
  cfg.line_p2p.left_repetitions=1;
  cfg.line_p2p.straight_repetitions=1;
  cfg.line_p2p.right_repetitions=1;
  cfg.curve_search.threshold_deg=20;
  cfg.curve_search.confirm_extra_sequences=1;
  cfg.curve_search.scan_delay_walk_sequences=1;
  cfg.curve_search.stable_window=10;
  cfg.curve_search.stable_min_hits=5;
  MissionController controller(cfg);
  MissionFrameInput in;
  in.image_width=640;in.image_height=480;in.intrinsics={600,600,320,240};
  in.camera_feedback={CameraMode::kForward,true};
  in.command_transport_enabled=true;
  // Four lower RANSAC survivors and one upper original point: shared-anchor
  // upper fit has two points, and normal O/H remains approximately straight.
  in.line_centers={{380,450},{380,400},{380,350},{380,300},{430,250}};
  const auto evidence=ComputeCurveEvidence(in.line_centers,640,480,
      cfg.line_features,cfg.curve_search);
  assert(evidence.lower_points==3 && evidence.upper_points==3);
  const auto six=ComputeCurveEvidence(
      {{380,450},{380,400},{380,350},{380,300},{410,250},{440,200}},
      640,480,cfg.line_features,cfg.curve_search);
  assert(six.valid && six.lower_points==4 && six.upper_points==3);
  const auto seven=ComputeCurveEvidence(
      {{380,450},{380,400},{380,350},{380,300},{450,250},{520,200},{590,150}},
      640,480,cfg.line_features,cfg.curve_search);
  assert(seven.valid && seven.lower_points==4 && seven.upper_points==4);
  std::fprintf(stderr,"evidence=%d dir=%d angle=%.2f upper=%d\\n",
      int(evidence.valid),evidence.direction,evidence.delta_deg,evidence.upper_points);
  assert(evidence.valid && evidence.direction==1);
  MissionFrameResult out;
  const auto tick=[&](CommandDeliveryFeedback fb={}) {
    in.now_sec+=0.07;
    in.delivery_feedback=fb;
    out=controller.Step(in);
  };
  tick();
  for(int seq=0;seq<3;++seq) {
    const auto id=out.command.action_id;
    assert(id && out.command.action==MissionAction::kLineStraightUnit);
    tick({id,true,false,false});
    for(int i=0;i<12;++i) {
      tick();
      if(i==10) std::fprintf(stderr,"beforeDONE active=%d phase=%d action=%d id=%llu\\n",
          int(out.line_plan_active),out.curve_search_phase,int(out.command.action),
          (unsigned long long)out.command.action_id);
    }
    tick({id,true,true,false});
    std::fprintf(stderr,"SEQ=%d phase=%d confirm=%d action=%d id=%llu line=%d\\n",
        seq,out.curve_search_phase,out.curve_confirmations,int(out.command.action),
        (unsigned long long)out.command.action_id,int(out.active_mission));
    if(seq==0) assert(out.curve_search_phase==1); // confirm
    if(seq==1) assert(out.curve_search_phase==2); // delay
  }
  assert(out.course_curve_index==1);
  assert(out.command.camera_request==CameraRequest::kScanLeft);
  assert(out.curve_search_phase==4);
  in.camera_feedback={CameraMode::kTransition,false};
  in.camera_scan.phase=1;in.camera_scan.command_id=10;in.camera_scan.yaw_deg=25;
  in.scan_ball_u=320;
  for(int i=0;i<12;++i) {
    tick();
    assert(out.command.camera_request!=CameraRequest::kScanHold);
    assert(out.command.camera_request!=CameraRequest::kScanReturn);
  }
  in.camera_scan.phase=2;
  in.camera_scan.stop_id=1;
  in.camera_scan.frame_is_fresh=true;
  bool returning=false;
  for(int i=0;i<15;++i) {
    tick();
    if(out.command.camera_request==CameraRequest::kScanReturn) {returning=true;break;}
  }
  assert(returning);
  assert(out.curve_search_phase==6); // return found
  in.camera_scan.phase=0;in.camera_scan.finished=true;
  in.camera_feedback={CameraMode::kForward,true};
  in.scan_ball_u.reset();
  tick(); // return DONE, choose the in-place correction for a >=30 degree yaw
  assert(out.curve_search_phase==10 || out.curve_search_phase==9);
  // A confirmed BALL admission cancels only a curve scan that has not started.
  MissionController priority(cfg);
  MissionFrameInput b;
  b.image_width=640;b.image_height=480;
  b.camera_feedback={CameraMode::kForward,true};
  b.command_transport_enabled=true;
  b.line_centers={{380,450},{380,400},{380,350},{380,300},{430,250}};
  MissionFrameResult bout;
  const auto ptick=[&](CommandDeliveryFeedback fb={}) {
    b.now_sec+=.07;b.delivery_feedback=fb;bout=priority.Step(b);
  };
  ptick();
  for(int seq=0;seq<2;++seq) {
    const auto id=bout.command.action_id;
    assert(id && bout.command.action==MissionAction::kLineStraightUnit);
    ptick({id,true,false,false});
    for(int j=0;j<12;++j)ptick();
    ptick({id,true,true,false});
  }
  assert(bout.curve_search_phase==2);
  ObjectTarget ball;
  ball.class_id=cfg.object_targets.ball_class_id;
  ball.confidence=.95;ball.center_px={320,100};
  ball.width_px=ball.height_px=40;ball.area_px=1600;
  b.ball_target=ball;b.ball_track_id=12;
  b.ball_entry_track_hits=cfg.ball.entry_min_hits;
  ptick();
  assert(bout.approach_mission==MissionType::kBall);
  assert(bout.curve_search_phase==0);
  assert(bout.command.camera_request==CameraRequest::kNone);
  ZeroDelayCurveRegression();
  return 0;
}
