#include "vision_core/cruise_selector.hpp"

#include <cmath>

namespace vision_core {

CruiseDecision SelectCruiseDecision(bool applicable, bool valid,
                                    double error, double deadband) {
  CruiseDecision decision;
  decision.applicable = applicable;
  decision.error = error;
  decision.deadband = deadband;
  if (!applicable || !valid || !std::isfinite(error) ||
      !std::isfinite(deadband) || deadband < 0.0) {
    return decision;
  }
  if (error < -deadband) {
    decision.direction = CruiseDirection::kLeft;
  } else if (error > deadband) {
    decision.direction = CruiseDirection::kRight;
  } else {
    decision.direction = CruiseDirection::kStraight;
  }
  return decision;
}

MissionAction CruiseAction(CruiseDirection direction) {
  switch (direction) {
  case CruiseDirection::kLeft: return MissionAction::kStepForwardLeft;
  case CruiseDirection::kStraight: return MissionAction::kStepForwardFive;
  case CruiseDirection::kRight: return MissionAction::kStepForwardRight;
  case CruiseDirection::kNone: break;
  }
  return MissionAction::kNone;
}

} // namespace vision_core
