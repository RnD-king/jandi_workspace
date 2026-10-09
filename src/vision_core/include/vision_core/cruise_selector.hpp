#pragma once

#include "vision_core/types.hpp"

namespace vision_core {

// error는 화면 오른쪽이 양수다. invalid/non-finite 관측은 NONE이며,
// deadband 경계값은 STRAIGHT에 포함한다.
CruiseDecision SelectCruiseDecision(bool applicable, bool valid,
                                    double error, double deadband);

MissionAction CruiseAction(CruiseDirection direction);

} // namespace vision_core
