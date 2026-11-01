#include "vision_core/config_loader.hpp"
#include "vision_core/mission_controller.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cstdio>
using namespace vision_core;

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
  in.image_width=640;in.image_height=480;
  in.camera_feedback={CameraMode::kForward,true};
  in.command_transport_enabled=true;
  // Four lower RANSAC survivors and one upper original point: shared-anchor
  // upper fit has two points, and normal O/H remains approximately straight.
  in.line_centers={{380,450},{380,400},{380,350},{380,300},{410,250}};
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
    for(int i=0;i<12;++i) tick();
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
  in.camera_scan.phase=1;in.camera_scan.command_id=10;in.camera_scan.yaw_deg=-25;
  in.scan_ball_u=320;
  bool hold=false;
  for(int i=0;i<5;++i) {
    tick();
    if(out.command.camera_request==CameraRequest::kScanHold) hold=true;
  }
  assert(hold);
  in.camera_scan.phase=3;
  bool returning=false;
  for(int i=0;i<12;++i) {
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
  return 0;
}
