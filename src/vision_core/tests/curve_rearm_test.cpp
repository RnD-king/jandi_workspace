#include "vision_core/config_loader.hpp"
#include "vision_core/mission_controller.hpp"
#include <stdexcept>
#include <cstdio>
using namespace vision_core;

struct Harness {
  MissionController core;
  MissionFrameInput in;
  MissionFrameResult out;
  explicit Harness(MissionControllerConfig cfg):core(cfg) {
    in.image_width=640;in.image_height=480;
    in.camera_feedback={CameraMode::kForward,true};
    in.command_transport_enabled=true;
    in.line_centers={{380,450},{380,400},{380,350},{380,300},{410,250}};
    Tick();
  }
  void Tick(CommandDeliveryFeedback fb={}) {
    in.now_sec+=.07;in.delivery_feedback=fb;out=core.Step(in);
  }
  [[noreturn]] void Fail(const char* message) const {
    std::fprintf(stderr,"REARM FAIL: %s [phase=%d head=%d index=%d skip=%d unlock=%d action=%d id=%llu]\n",
        message,out.curve_search_phase,out.curve_head_attempts,out.course_curve_index,
        out.curve_skip_pending,out.curve_unlock_blocks,int(out.command.action),
        (unsigned long long)out.command.action_id);
    throw std::runtime_error(message);
  }
  void Walk() {
    for(int i=0;i<30 && !out.command.action_id;++i)Tick();
    const auto id=out.command.action_id;
    if(!id || !LinePlan::IsUnit(out.command.action))Fail("no LINE unit ready");
    Tick({id,true,false,false});
    for(int i=0;i<12;++i)Tick();
    Tick({id,true,false,true});
    Tick({id,true,true,false}); // Exactly one physical DONE per logical block.
  }
  void FailedHeadScan(int expected_attempt) {
    if(out.curve_search_phase!=4 || out.curve_head_attempts!=expected_attempt)
      Fail("head scan attempt not started");
    in.camera_feedback={CameraMode::kTransition,false};
    in.camera_scan.phase=1;in.camera_scan.finished=false;Tick();
    in.camera_scan.phase=0;in.camera_scan.finished=true;Tick();
    if(out.curve_search_phase!=7)Fail("missing scan did not enter ReturnMiss");
    in.camera_feedback={CameraMode::kForward,true};
    in.camera_scan.finished=false;Tick();
  }
};

int main() {
  auto cfg=LoadDefaultAlgorithmConfig();
  cfg.enable_ball=true;cfg.enable_goal=cfg.enable_hurdle=false;
  cfg.curve_search.enabled=true;
  cfg.curve_search.confirm_extra_sequences=1;
  cfg.curve_search.scan_delay_walk_sequences=0;
  cfg.curve_search.failure_lock_line_blocks=20;
  cfg.curve_search.head_search_max_attempts=3;
  cfg.line_p2p.left_repetitions=cfg.line_p2p.right_repetitions=
      cfg.line_p2p.straight_repetitions=1;
  Harness d(cfg);
  d.Walk();
  if(d.out.curve_search_phase!=1)d.Fail("first right curve requires CONFIRM");
  d.Walk();
  if(d.out.curve_search_phase!=4)d.Fail("second right curve requires immediate SCAN");

  for(int attempt=1;attempt<=3;++attempt) {
    d.FailedHeadScan(attempt);
    if(attempt<3) {
      if(d.out.curve_search_phase!=8)d.Fail("missed head scan should schedule probe walk");
      d.Tick(); // Book the next LINE probe using saved guide.
      d.Walk();
      if(d.out.curve_search_phase!=4)d.Fail("probe DONE should schedule next head scan");
    }
  }
  if(d.out.curve_search_phase!=12 || d.out.curve_unlock_blocks!=20 ||
     d.out.course_curve_index!=1 || !d.out.curve_skip_pending)
    d.Fail("third failed head scan did not lock 20 blocks and mark skip_pending");

  const std::vector<Point2> normal={{380,450},{380,400},{380,350},{380,300},{380,250}};
  const std::vector<Point2> right={{380,450},{380,400},{380,350},{380,300},{410,250}};
  const std::vector<Point2> left={{380,450},{380,400},{380,350},{380,300},{350,250}};
  const auto evidence=ComputeCurveEvidence(left,640,480,cfg.line_features,cfg.curve_search);
  if(!evidence.valid || evidence.direction!=-1)
    d.Fail("left curve evidence fixture is not valid");

  d.in.line_centers=normal;
  for(int completed=1;completed<=20;++completed) {
    if(completed==20) d.in.line_centers=right; // Ignore final locked-segment curve votes.
    d.Walk();
    if(completed<20 && (d.out.curve_search_phase!=12 ||
        d.out.curve_unlock_blocks!=20-completed || !d.out.curve_skip_pending))
      d.Fail("locked phase must decrement on exactly 20 LINE block DONEs");
  }
  if(d.out.curve_search_phase!=0 || d.out.curve_unlock_blocks!=0 ||
     !d.out.curve_skip_pending)
    d.Fail("twentieth DONE failed to unlock without preserving any curve evidence");
  d.in.line_centers=left;
  d.Walk();
  if(d.out.curve_search_phase!=1 || d.out.course_curve_index!=1 ||
     d.out.curve_confirmations!=0 || !d.out.curve_skip_pending)d.Fail("one fresh left curve must only confirm");
  d.Walk();
  if(d.out.curve_search_phase!=4 || d.out.course_curve_index!=3 ||
     d.out.curve_skip_pending ||
     d.out.command.camera_request!=CameraRequest::kScanRight)
    d.Fail("second fresh left curve failed to scan index three after right miss");
  std::puts("PASS: head 3 misses -> 20 physical LINE blocks -> fresh left 2 blocks -> scan");
}
