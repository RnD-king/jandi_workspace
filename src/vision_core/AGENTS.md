# Shared Vision Core Agent Guide

## Project role and boundary

`vision_core` is the ROS-independent C++ perception, mission-control, and
direct-action library used by the real ROS2 `vision` adapter. It is a library,
not a standalone ROS node.

Do not introduce ROS2, sensor message, TensorRT, CUDA, camera-driver, or runtime
transport dependencies here. Avoid OpenCV unless an explicit architecture
decision requires it. Hardware and ROS adaptation belong in `vision`.

## Configuration ownership

`config/vision_algorithm.yaml` is the single source of shared algorithm
defaults. Do not duplicate them in the ROS adapter. Missing, non-finite, or
semantically invalid required configuration must fail fast instead of being
silently clamped into a different behavior.

## P2P-only mission architecture

The active control pipeline is:

perception → `MissionController` → direct `MissionAction` →
`ControlCommandCoordinator` → ACK/READY/DONE → P2P executor

There is no active continuous-velocity backend, `MotionCommand`,
`P2pMotionQuantizer`, MuJoCo velocity compatibility, or C API.

`MissionController` owns mission priority, mission locking, controller reset
sequencing, object-mission transitions, LINE recovery, and completion routing.
Individual controllers own their mission-specific FSM behavior. External
callers provide observations and executor feedback and consume the returned
`ControlCommand`; they must not reproduce mission decisions independently.

## Action and camera lifecycle

Preserve the distinction between locomotion actions, mission/discrete actions,
stationary HOLD, and camera requests. Existing action numbers, `action_id`, and
ACK/READY/DONE behavior are public execution contracts.

READY does not cancel the current action. A long LINE action may reserve at
most one direct LINE action with a new ID. The executor must ACK that ID only
after actually storing it and must start it exactly once after current DONE.

Camera triggers are latched while locomotion runs. Camera commands are emitted
only after locomotion DONE, and new locomotion stays on HOLD until the camera is
settled.

## LINE behavior

Normal LINE steering uses only the near-fit offset/heading score and directly
selects `STEP_FORWARD_LEFT`, `STEP_FORWARD_FIVE`, or `STEP_FORWARD_RIGHT`.
Curvature is diagnostic-only, has independent validity/accumulation, and must
not affect steering, mission transitions, or recovery.

Invalid direct observations produce HOLD, never a legacy fallback. LINE
failure uses stationary observation, bounded directional recovery/no-evidence
retries, and terminal FINAL HOLD until explicit Reset. Unstable observations
must not update directional memory.

## Determinism and tests

Keep core behavior deterministic for identical configuration, observations,
state, and execution feedback. Do not add hidden ROS state, wall-clock access,
global runtime state, or hardware access.

Tests are part of the product contract. Add focused coverage for behavior
changes; do not delete or weaken tests merely because an implementation fails.

Standard verification:

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$PWD/install"
cmake --build build -j
ctest --test-dir build --output-on-failure
cmake --install build
```

## Scope discipline

Inspect the relevant implementation, tests, and public callers; make the
smallest coherent change; then run focused and full verification. Preserve the
ROS-independent boundary and avoid unrelated refactors.
