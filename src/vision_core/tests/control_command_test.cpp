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
  return 0;
}
