#pragma once
#include "vision_core/mission_controller.hpp"
#include <cassert>

// Post-entry regressions use an explicit stationary prelude, rather than
// weakening 10/7 or assuming that the first moving detection enters a mission.
// Their scenario clock begins at 3 s, after this 0..2 s prelude.
inline void AdmitObjectAtRest(vision_core::MissionController& controller,
    const vision_core::ObjectTarget& target,
    vision_core::MissionType mission = vision_core::MissionType::kBall) {
  vision_core::MissionFrameInput frame;
  frame.image_width=640; frame.image_height=480;
  frame.camera_feedback={vision_core::CameraMode::kForward,true};
  frame.command_transport_enabled=true;
  frame.manual_observation_control=true; // This fixture explicitly tests stationary manual admission.
  frame.allow_new_line_action=false;
  frame.allow_new_body_actions=false;
  if (mission==vision_core::MissionType::kBall) frame.ball_target=target;
  else frame.hurdle_target=target;
  for (int i=0;i<30;++i) {
    frame.now_sec=.07*i;
    const auto result=controller.Step(frame);
    assert(result.command.command_type!=vision_core::CommandType::kAction);
    if (result.active_mission==mission) return;
  }
  assert(false && "fresh stationary entry prelude did not verify the object");
}
