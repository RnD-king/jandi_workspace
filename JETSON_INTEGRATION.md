# PC / Jetson unified integration — 2026-10-12

Source of truth:

- `src/forward_walk`: unmodified `RnD-king/2026_motion` commit `f477db208c4877f36a4497583f20d976f55fa2d6`, folder `10.13.1/forward_walk`.
- `src/vision`, `src/vision_core`: this jandi_workspace checkout. `vision/msg/ActionCommand.msg` uses the friend's exact constants and fields.
- Deployment branch: `feature/forward-walk-10-12-1-integrated-20261011`.

The friend executor includes new actions 21–26, modified right-turn manifests, cross-mission READY for the new walks, mission arm transitions and motor-health logging. Preserve the entire package. Its arm behavior remains as supplied by the friend.

`vision_core` is a standalone CMake library, intentionally marked COLCON_IGNORE. Build, test and install it before `vision`; do not mix old headers, libraries or algorithm YAML with the new adapter.

## Existing Jetson checkout (/home/rnd/jandi)

Run in the SSH terminal. Preserve existing local source changes before checking out the unified branch:

```bash
cd /home/rnd/jandi
BACKUP="$HOME/jandi_backups/unified_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$BACKUP"
cp -a src/vision src/forward_walk src/vision_core "$BACKUP/"
git stash push -u -m 'before unified mission update' -- src/vision src/forward_walk src/vision_core
git fetch origin feature/forward-walk-10-12-1-integrated-20261011
git switch feature/forward-walk-10-12-1-integrated-20261011
git pull --ff-only origin feature/forward-walk-10-12-1-integrated-20261011
git rev-parse HEAD
bash scripts/build_unified.sh /home/rnd/jandi
source /home/rnd/jandi/install/setup.bash
```

These commands assume this checkout's origin is `RnD-king/jandi_workspace`. Keep the backup/stash; do not reapply the old forward_walk or messages over the unified source.

The build script starts no camera or motor node. Executor integration tests use an in-memory Dynamixel implementation. PC verification does not establish Jetson or physical-robot behavior.

## PC

`~/sync_jandi_to_my_cv.sh` and the checked-in copy `scripts/sync_jandi_to_my_cv.sh` now default to the deployment branch above. The script requires a clean source checkout, backs up all three source trees and mirrors additions, modifications and deletions. Nested .git, build, install and log directories remain excluded. It builds the core first and then the ROS packages.

To build already synchronized PC sources:

```bash
bash /home/noh/jandi_workspace/scripts/build_unified.sh /home/noh/my_cv /home/noh/vision_core
source /home/noh/my_cv/install/setup.bash
```

## Tuning

Read [UNIFIED_MISSION_TUNING.md](src/vision/UNIFIED_MISSION_TUNING.md) for phase-level COMMIT, continued observation during repeat steps, adaptive approach/fine walks, pickup verification before RECATCH, alternating bounded line recovery and rqt scenario indices.

The dedicated `goal_tuning_node` starts with FORWARD approach and completes fine alignment, SHOOT, two backwards actions and line recovery. It returns to WAIT_START for the next manually prepared trial.

Model selection remains in `src/vision/config/yolo26_runtime.yaml`. Jetson defaults use `/home/rnd/jandi/src/dataset/best.engine`; supply `engine_path` when the engine lives elsewhere. The legacy robot_bringup launch is not the entrypoint for these tuning nodes.

Image-independent watchdogs, new DONE timeouts and carrying-aware arm changes were explicitly deferred. Detection/time/count limits stop dispatching new repeat motions; accepted motions finish with their original ACK/DONE lifecycle.
