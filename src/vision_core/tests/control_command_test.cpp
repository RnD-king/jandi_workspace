#include "vision_core/control_command.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
using namespace vision_core;
static_assert(static_cast<int>(MissionAction::kDefaultPosition)==1);
static_assert(static_cast<int>(MissionAction::kDefaultPoseMode)==2);
static_assert(static_cast<int>(MissionAction::kStepForwardHalf)==3);
static_assert(static_cast<int>(MissionAction::kStepBack)==4);
static_assert(static_cast<int>(MissionAction::kLeftSideStep)==5);
static_assert(static_cast<int>(MissionAction::kRightSideStep)==6);
static_assert(static_cast<int>(MissionAction::kTurnLeft)==7);
static_assert(static_cast<int>(MissionAction::kTurnRight)==8);
static_assert(static_cast<int>(MissionAction::kWalkMode)==9);
static_assert(static_cast<int>(MissionAction::kStepForwardOne)==10);
static_assert(static_cast<int>(MissionAction::kStepForwardLeft)==11);
static_assert(static_cast<int>(MissionAction::kStepForwardRight)==12);
static_assert(static_cast<int>(MissionAction::kStepForwardFive)==13);
static_assert(static_cast<int>(MissionAction::kPickBall)==14);
static_assert(static_cast<int>(MissionAction::kRecatch)==15);
static_assert(static_cast<int>(MissionAction::kHurdle)==16);
static_assert(static_cast<int>(MissionAction::kShoot)==17);
static_assert(static_cast<int>(MissionAction::kTurnLeftAndStep)==18);
static_assert(static_cast<int>(MissionAction::kTurnRightAndStep)==19);
static_assert(static_cast<int>(MissionAction::kContactWalk)==20);
static_assert(static_cast<int>(MissionAction::kStepForwardOneAlternate)==35);
int main() {
  ControlCommandCoordinator c({1,10});
  ActionRequest first{MissionAction::kStepForwardFive,
                      ActionCategory::kLocomotion,0,true};
  auto out=c.Compute(MissionType::kLine,0,first,CameraRequest::kNone,{},0);
  assert(out.command_type==CommandType::kAction && out.action_id==1);
  CommandDeliveryFeedback ack{1,true,false,false};
  out=c.Compute(MissionType::kLine,0,first,CameraRequest::kNone,ack,.1);
  assert(out.command_type==CommandType::kHold);
  ActionRequest next{MissionAction::kStepForwardLeft,
                     ActionCategory::kLocomotion,0,true};
  CommandDeliveryFeedback ready{1,true,false,true};
  out=c.Compute(MissionType::kLine,0,next,CameraRequest::kNone,ready,.2);
  assert(out.command_type==CommandType::kAction && out.action_id==2);
  assert(out.action==MissionAction::kStepForwardLeft);
  CommandDeliveryFeedback queued_ack{2,true,false,false};
  out=c.Compute(MissionType::kLine,0,next,CameraRequest::kNone,queued_ack,.3);
  assert(out.control_phase==ControlPhase::kWaitingQueuedActionStart);
  ActionRequest ignored_third{MissionAction::kStepForwardRight,
                              ActionCategory::kLocomotion,0,true};
  out=c.Compute(MissionType::kLine,0,ignored_third,CameraRequest::kNone,{},.35);
  assert(out.action_id==2 && out.action==MissionAction::kStepForwardLeft);
  CommandDeliveryFeedback done{1,true,true,false};
  out=c.Compute(MissionType::kLine,0,{},CameraRequest::kNone,done,.4);
  assert(out.action_id==2 && out.control_phase==ControlPhase::kWaitingActionDone);

  // A's READY eligibility alone permits B (including a different mission
  // and non-READY action); B's eligibility controls only B->C later.
  ControlCommandCoordinator object({200,10});
  ActionRequest object_a{MissionAction::kStepForwardFour,
                         ActionCategory::kLocomotion,0,true};
  auto obj=object.Compute(MissionType::kBall,4,object_a,CameraRequest::kNone,{},0);
  assert(obj.action_id==200);
  obj=object.Compute(MissionType::kBall,4,{},CameraRequest::kNone,
                     {200,true,false,false},.1);
  ActionRequest object_b{MissionAction::kStepForwardRightTwo,
                         ActionCategory::kLocomotion,0,false};
  obj=object.Compute(MissionType::kBall,4,object_b,CameraRequest::kNone,
                     {200,true,false,true},.2);
  assert(obj.command_type==CommandType::kAction && obj.action_id==201);
  assert(obj.mission==MissionType::kBall &&
         obj.action==MissionAction::kStepForwardRightTwo);
  obj=object.Compute(MissionType::kBall,4,{},CameraRequest::kNone,
                     {201,true,false,false},.25);
  assert(obj.control_phase==ControlPhase::kWaitingQueuedActionStart);
  obj=object.Compute(MissionType::kBall,4,{},CameraRequest::kNone,
                     {200,true,true,false},.3);
  assert(obj.action_id==201 && obj.mission==MissionType::kBall);
  obj=object.Compute(MissionType::kBall,4,object_b,CameraRequest::kNone,
                     {201,true,false,true},.4);
  assert(obj.action_id==201); // B is not a queue source.
  ControlCommandCoordinator cross_mission({400,10});
  obj=cross_mission.Compute(MissionType::kLine,1,first,CameraRequest::kNone,{},0);
  obj=cross_mission.Compute(MissionType::kLine,1,{},CameraRequest::kNone,
                            {400,true,false,false},.1);
  obj=cross_mission.Compute(MissionType::kGoal,2,object_b,CameraRequest::kNone,
                            {400,true,false,true},.2);
  assert(obj.action_id==401 && obj.mission==MissionType::kGoal);
  obj=cross_mission.Compute(MissionType::kGoal,2,{},CameraRequest::kNone,
                            {400,true,true,false},.3);
  assert(obj.action_id==401 && obj.mission==MissionType::kGoal);

  ControlCommandCoordinator missed_ready({500,10});
  auto missed=missed_ready.Compute(MissionType::kBall,4,object_a,
                                   CameraRequest::kNone,{},0);
  assert(missed.action_id==500);
  missed=missed_ready.Compute(MissionType::kBall,4,{},CameraRequest::kNone,
                              {500,true,false,true},.25);
  assert(missed.action_id==500);
  missed=missed_ready.Compute(MissionType::kBall,4,object_b,CameraRequest::kNone,
                              {},.3);
  assert(missed.action_id==500); // No delayed reservation beyond A's READY edge.

  ControlCommandCoordinator wrong_id({10,1.0});
  out=wrong_id.Compute(MissionType::kLine,0,first,CameraRequest::kNone,{},0);
  assert(out.action_id==10);
  out=wrong_id.Compute(MissionType::kLine,0,first,CameraRequest::kNone,
                       {999,true,true,true},.1);
  assert(out.action_id==10 && out.control_phase==ControlPhase::kWaitingActionAck);

  ControlCommandCoordinator timeout({20,.5});
  out=timeout.Compute(MissionType::kLine,0,first,CameraRequest::kNone,{},0);
  out=timeout.Compute(MissionType::kLine,0,first,CameraRequest::kNone,{},.6);
  assert(out.action_id==20);
  assert(out.command_type==CommandType::kHold);
  assert(out.control_phase==ControlPhase::kActionAckTimedOut);
  out=timeout.Compute(MissionType::kLine,0,first,CameraRequest::kNone,
                      {20,true,false,false},.7);
  assert(out.control_phase==ControlPhase::kActionAckTimedOut);
  // Operator RESET must not reuse an action ID still remembered by forward_walk.
  ControlCommandCoordinator preserved({100,10});
  auto p=preserved.Compute(MissionType::kLine,0,first,CameraRequest::kNone,{},0);
  assert(p.action_id==100);
  preserved.Reset(true);
  p=preserved.Compute(MissionType::kLine,0,first,CameraRequest::kNone,{},.1);
  assert(p.action_id==101);
  preserved.Reset(); // Existing cold-reset behavior still supported.
  p=preserved.Compute(MissionType::kLine,0,first,CameraRequest::kNone,{},.2);
  assert(p.action_id==100);

  // Consecutive one-step requests keep action 10 across observation and camera gaps.
  ControlCommandCoordinator steps({600,10});
  const ActionRequest one{MissionAction::kStepForwardOne,
                          ActionCategory::kLocomotion,0,false};
  auto step=steps.Compute(MissionType::kBall,0,one,CameraRequest::kNone,{},0);
  assert(step.action_id==600 && step.action==MissionAction::kStepForwardOne);
  step=steps.Compute(MissionType::kBall,0,one,CameraRequest::kNone,
                     {999,true,true,false},.1);
  assert(step.action_id==600 && step.action==MissionAction::kStepForwardOne);
  step=steps.Compute(MissionType::kBall,0,{},CameraRequest::kDown,
                     {600,true,true,false},.2);
  assert(step.action_id==0);
  step=steps.Compute(MissionType::kBall,0,{},CameraRequest::kNone,{},.3);
  step=steps.Compute(MissionType::kBall,0,one,CameraRequest::kNone,{},.4);
  assert(step.action_id==601 && step.action==MissionAction::kStepForwardOne);
  step=steps.Compute(MissionType::kBall,0,one,CameraRequest::kNone,{},.5);
  assert(step.action_id==601 && step.action==MissionAction::kStepForwardOne);
  step=steps.Compute(MissionType::kBall,0,one,CameraRequest::kNone,
                     {601,true,true,false},.6);
  assert(step.action_id==602 && step.action==MissionAction::kStepForwardOne);
  const ActionRequest turn{MissionAction::kTurnLeftInPlace,
                           ActionCategory::kLocomotion,0,false};
  step=steps.Compute(MissionType::kBall,0,turn,CameraRequest::kNone,
                     {602,true,true,false},.7);
  assert(step.action_id==603 && step.action==MissionAction::kTurnLeftInPlace);
  step=steps.Compute(MissionType::kBall,0,one,CameraRequest::kNone,
                     {603,true,true,false},.8);
  assert(step.action_id==604 && step.action==MissionAction::kStepForwardOne);
  steps.Reset(true);
  step=steps.Compute(MissionType::kBall,0,one,CameraRequest::kNone,{},.9);
  assert(step.action_id==605 && step.action==MissionAction::kStepForwardOne);

  // A READY successor retains the requested action even before A's physical DONE.
  ControlCommandCoordinator queued_steps({700,10});
  ActionRequest queue_one=one;queue_one.line_queue_eligible=true;
  step=queued_steps.Compute(MissionType::kBall,0,queue_one,CameraRequest::kNone,{},0);
  assert(step.action==MissionAction::kStepForwardOne);
  step=queued_steps.Compute(MissionType::kBall,0,one,CameraRequest::kNone,
                            {700,true,false,true},.1);
  assert(step.action_id==701 && step.action==MissionAction::kStepForwardOne);
  step=queued_steps.Compute(MissionType::kBall,0,{},CameraRequest::kNone,
                            {700,true,true,false},.2);
  assert(step.action_id==701 && step.action==MissionAction::kStepForwardOne);
  step=queued_steps.Compute(MissionType::kBall,0,one,CameraRequest::kNone,
                            {701,true,true,false},.3);
  assert(step.action_id==702 && step.action==MissionAction::kStepForwardOne);

  // A queued one-step after a two-step retains its requested action.
  ControlCommandCoordinator after_two({800,10});
  step=after_two.Compute(MissionType::kHurdle,0,one,CameraRequest::kNone,{},0);
  const ActionRequest two{MissionAction::kStepForwardTwo,
                          ActionCategory::kLocomotion,0,true};
  step=after_two.Compute(MissionType::kHurdle,0,two,CameraRequest::kNone,
                         {800,true,true,false},.1);
  assert(step.action_id==801 && step.action==MissionAction::kStepForwardTwo);
  step=after_two.Compute(MissionType::kHurdle,0,one,CameraRequest::kNone,
                         {801,true,false,true},.2);
  assert(step.action_id==802 && step.action==MissionAction::kStepForwardOne);

  // The legacy action ID remains available without automatic rewriting.
  ControlCommandCoordinator legacy({900,10});
  const ActionRequest one_legacy{MissionAction::kStepForwardOneAlternate,
                                 ActionCategory::kLocomotion,0,false};
  step=legacy.Compute(MissionType::kHurdle,0,one_legacy,CameraRequest::kNone,{},0);
  assert(step.action_id==900 && step.action==MissionAction::kStepForwardOneAlternate);

  return 0;
}
