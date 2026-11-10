#include "vision_core/config_loader.hpp"
#include "vision_core/hurdle_controller.hpp"
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>
using namespace vision_core;
namespace {
void Check(bool ok,const char* why) { if(!ok) throw std::runtime_error(why); }
ObjectTarget Seen(double v) {
  ObjectTarget t;t.center_px={380,v};t.box_px={260,v-10,240,20};
  t.width_px=240;t.height_px=20;t.confidence=.95;t.area_px=4800;return t;
}
HurdleConfig Config() {
  auto c=LoadDefaultAlgorithmConfig().hurdle;
#ifdef HURDLE_FAST_VOTING_TEST
  c.race_round2_fast_enabled=true;c.stable_window=8;
  c.down_two_step_min_hits=6;c.stable_min_hits=4;
#else
  c.stable_window=10;c.down_two_step_min_hits=7;c.stable_min_hits=5;
#endif
  c.down_search_min_hits=3;c.down_v_min_hits=3;c.contact_v_min_hits=3;
  c.down_one_step_v_px=40;c.fine_mode_v_px=220;c.contact_v_px=220;
  c.contact_steps=3;c.contact_inter_action_hold_sec=0;c.pre_cross_hold_sec=.5;
  return c;
}
HurdleResult Coarse(const std::deque<std::optional<double>>& samples,double displayed_mean) {
  auto c=Config();HurdleController h(c,3);h.StartTestNear();
  CameraFeedback camera{CameraMode::kDown,true};double now=0;
  auto r=h.Compute({},640,480,now,camera,{},{});
  for(int i=0;i<c.stable_window && r.mode!=HurdleMode::kCoarseAdjust;++i)
    r=h.Compute(Seen(100),640,480,now+=.1,camera,{},{});
  Check(r.mode==HurdleMode::kCoarseAdjust,"did not reach stationary DOWN");
  std::deque<bool> hits;for(const auto& v:samples)hits.push_back(v && std::isfinite(*v));
  h.UseVerifiedObservation(Seen(displayed_mean),hits,640,480,&samples);
  ActionExecutionFeedback f;f.observation_complete=true;
  return h.Compute(Seen(displayed_mean),640,480,now+=.1,camera,f,{});
}
void DistanceVotes() {
  const auto c=Config();const auto w=c.stable_window;
  std::deque<std::optional<double>> samples(w,100.0);
  // Mean is deliberately below 220: the three raw crossings alone decide.
  for(int i=0;i<3;++i)samples[i]=220;
  Check(Coarse(samples,66).mode==HurdleMode::kWaitFinePose,"three close votes lost to low mean");
  samples[2]=219.99;
  auto r=Coarse(samples,300);
  Check(r.mode==HurdleMode::kCoarseAdjust && r.action.action==MissionAction::kStepForwardOne,
        "two close votes or a high synthetic mean falsely triggered FINE");
  samples.assign(w,100);
  for(int i=0;i<c.down_two_step_min_hits-1;++i)samples[i]=39.99;
  Check(Coarse(samples,0).action.action==MissionAction::kStepForwardOne,
        "insufficient far votes chose two steps");
  samples[c.down_two_step_min_hits-1]=39.99;
  Check(Coarse(samples,200).action.action==MissionAction::kStepForwardTwo,
        "qualified far votes did not choose two steps");
  samples.assign(w,40);
  Check(Coarse(samples,0).action.action==MissionAction::kStepForwardOne,
        "v==40 was counted as below 40");
  samples.assign(w,std::nullopt);samples[0]=220;samples[1]=220;
  samples[2]=std::numeric_limits<double>::quiet_NaN();
  Check(Coarse(samples,300).mode!=HurdleMode::kWaitFinePose,"invalid values became close votes");
}
void MovingCoarseFramesCannotTriggerFine() {
  auto c=Config();HurdleController h(c,3);h.StartTestNear();
  const CameraFeedback down{CameraMode::kDown,true};
  auto r=h.Compute({},640,480,0,down,{},{});
  double now=0;
  while (r.mode!=HurdleMode::kCoarseAdjust)
    r=h.Compute(Seen(100),640,480,now+=.1,down,{},{});
  ActionExecutionFeedback running;running.action_active=true;
  for (int i=0;i<c.stable_window+3;++i) {
    r=h.Compute(Seen(300),640,480,now+=.1,down,running,{});
    Check(r.mode==HurdleMode::kCoarseAdjust && r.action.action==MissionAction::kNone,
          "moving distance-step frames triggered FINE before actual DONE");
  }
  const std::deque<std::optional<double>> stationary(c.stable_window,100.0);
  h.UseVerifiedObservation(Seen(100),std::deque<bool>(c.stable_window,true),640,480,&stationary);
  ActionExecutionFeedback observed;observed.observation_complete=true;
  r=h.Compute(Seen(100),640,480,now+=.1,down,observed,{});
  Check(r.mode==HurdleMode::kCoarseAdjust && r.action.action==MissionAction::kStepForwardOne,
        "moving votes contaminated the fresh stationary distance decision");
}
void ContactLatch() {
  auto c=Config();HurdleController h(c,3);h.StartTestExecute();
  const CameraFeedback down{CameraMode::kDown,true};
  for(int i=0;i<c.stable_window-1;++i)h.ObserveOnly(Seen(i<3?220:0),640,480);
  Check(!h.ContactObservationReady(),"partial contact window qualified");
  h.ObserveOnly(Seen(0),640,480);
  Check(h.ContactObservationReady(),"three raw contact crossings did not latch");
  ActionExecutionFeedback running;running.action_active=true;
  auto r=h.Compute({},640,480,1,down,running,{});
  Check(r.mode==HurdleMode::kFineAdjust && r.action.action==MissionAction::kNone,
        "contact bypassed active action DONE");
  // A post-DONE gate can replace coordinates, but cannot undo a committed contact.
  h.ClearObservationHistory();
  std::deque<std::optional<double>> low(c.stable_window,0.0);
  h.UseVerifiedObservation(Seen(0),std::deque<bool>(c.stable_window,true),640,480,&low);
  r=h.Compute(Seen(0),640,480,2,down,{}, {},false);
  Check(r.mode==HurdleMode::kFineAdjust && h.ContactObservationReady(),"permission gate lost latch");
  r=h.Compute({},640,480,3,down,{},{});
  Check(r.mode==HurdleMode::kContactWalk && r.action.action==MissionAction::kStepForwardHalf,
        "later low or missing target canceled contact");
  ActionExecutionFeedback done;done.action_done=true;
  for(int i=0;i<c.contact_steps;++i)r=h.Compute({},640,480,4+i,down,done,{});
  Check(r.action.action==MissionAction::kNone,"cross skipped pre-cross hold");
  r=h.Compute({},640,480,7,down,{},{});
  Check(r.action.action==MissionAction::kHurdle,"committed contact did not finish crossing");
  h.Reset();Check(!h.ContactObservationReady(),"reset preserved previous hurdle contact");
}
void MeanCannotManufactureVotes() {
  auto c=Config();HurdleController h(c,3);h.StartTestExecute();
  h.UseVerifiedObservation(Seen(500),std::deque<bool>(c.stable_window,true),640,480);
  Check(!h.ContactObservationReady(),"mean was expanded into fictional contact samples");
  std::deque<std::optional<double>> raw(c.stable_window,0.0);
  raw[0]=raw[1]=220;
  h.UseVerifiedObservation(Seen(500),std::deque<bool>(c.stable_window,true),640,480,&raw);
  Check(!h.ContactObservationReady(),"synthetic mean replaced two actual votes");
  raw[2]=220;
  h.UseVerifiedObservation(Seen(0),std::deque<bool>(c.stable_window,true),640,480,&raw);
  Check(h.ContactObservationReady(),"verified raw votes lost at mean handoff");
  h.InvalidateTargetIdentity();Check(!h.ContactObservationReady(),"identity change retained contact latch");
}
}
int main() {DistanceVotes();MovingCoarseFramesCannotTriggerFine();ContactLatch();MeanCannotManufactureVotes();}
