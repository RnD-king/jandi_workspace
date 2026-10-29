#ifndef SHOOT_TUNING_POSE_HPP
#define SHOOT_TUNING_POSE_HPP

#include "motion_executor_state.hpp"

namespace motion_executor {
// Shooting bench only: IDs 0..20 are the final 초기자세_copy frame of
// 미세보행_미세전진_수정.json; IDs 21/22 are the fixed GOAL camera ticks.
// Do not apply walking arm/gripper overrides or replay intermediate poses.
inline RawPositions ShootTuningPoseTargets() {
    return {{0,2066}, {1,2030}, {2,2608}, {3,1496}, {4,1811}, {5,2296},
            {6,1413}, {7,2690}, {8,2066}, {9,2030}, {10,2049}, {11,2054},
            {12,2068}, {13,3062}, {14,1027}, {15,1074}, {16,3097},
            {17,1038}, {18,3124}, {19,1862}, {20,1881}, {21,2054}, {22,966}};
}
}

#endif
