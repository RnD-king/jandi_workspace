# Jetson integration (ROS 2 Humble)

The source packages here are copied from the independent upstream repositories:

- `src/vision/` — RnD-king/vision main, source snapshot `293073c`
- `src/vision_core/` — RnD-king/vision_core main, source snapshot `3807d1e`
- `src/forward_walk/` — Jandi-integrated P2P executor (do not overwrite with the 2026_motion original)

`vision_core` is a **standalone CMake** library (project `shared_vision_core`),
not a ROS ament package. `src/vision_core/COLCON_IGNORE` is intentional:
it prevents an uncoordinated `colcon build` from treating it as a workspace
package. Build and install it **before** building `vision`.

Run the following commands in a fresh Jetson terminal, adapting the workspace
location if it is not `~/jandi`:

```bash
source /opt/ros/humble/setup.bash
cd ~/jandi
git pull --ff-only origin main

# Make the Jetson use the core from this checkout, not a stale ~/vision_core tree.
cmake -S src/vision_core -B build/standalone_vision_core \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$HOME/vision_core/install"
cmake --build build/standalone_vision_core -j2
ctest --test-dir build/standalone_vision_core --output-on-failure
cmake --install build/standalone_vision_core

# The absolute core install path must be explicitly supplied to CMake.
export VISION_CORE_ROOT="$HOME/vision_core/install"
colcon build --symlink-install --packages-up-to forward_walk \
  --cmake-args "-DSHARED_VISION_CORE_ROOT=$VISION_CORE_ROOT"
source install/setup.bash
colcon test --packages-select forward_walk --event-handlers console_direct+
colcon test-result --verbose
```

Before trying the real robot:
- Verify the TensorRT engine path specified in
  `src/vision/config/yolo26_runtime.yaml` points to the Jetson-compatible
  engine (currently `/home/rnd/jandi/src/dataset/best.engine`).
- Verify the Dynamixel adapter is available as `/dev/jandi_dxl`.
- Do not execute `main_node` until all motor and startup safety checks pass.
- The legacy `robot_bringup` launch file still references old ROS nodes such
  as `my_cv` and `decision`, so this guide only validates the individual
  integrated packages, not the full launch.
- Local build/test and motor deployment have **not** been verified remotely.
