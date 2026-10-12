#!/usr/bin/env bash
# Build software and run hardware-free tests; never start a robot node.
set -eo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
WS=${1:-"$(cd -- "$SCRIPT_DIR/.." && pwd)"}
CORE_SOURCE=${2:-"$WS/src/vision_core"}
CORE_PREFIX=${VISION_CORE_ROOT:-"$HOME/vision_core/install"}

test -f "$CORE_SOURCE/config/vision_algorithm.yaml"
test -f "$WS/src/vision/msg/ActionCommand.msg"
test -f "$WS/src/forward_walk/src/main.cpp"
source /opt/ros/humble/setup.bash
if [[ -f "$WS/install/setup.bash" ]]; then
    source "$WS/install/setup.bash"
fi

cmake -S "$CORE_SOURCE" -B "$WS/build/shared_vision_core_debug" \
    -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON \
    -DCMAKE_INSTALL_PREFIX="$CORE_PREFIX"
cmake --build "$WS/build/shared_vision_core_debug" -j2
ctest --test-dir "$WS/build/shared_vision_core_debug" --output-on-failure

cmake -S "$CORE_SOURCE" -B "$WS/build/shared_vision_core_release" \
    -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    -DCMAKE_INSTALL_PREFIX="$CORE_PREFIX"
cmake --build "$WS/build/shared_vision_core_release" -j2
cmake --install "$WS/build/shared_vision_core_release"

cd "$WS"
colcon build --packages-select vision forward_walk --symlink-install \
    --parallel-workers 2 --cmake-clean-cache \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
        "-DSHARED_VISION_CORE_ROOT=$CORE_PREFIX"
source "$WS/install/setup.bash"
colcon test --packages-select vision forward_walk --event-handlers console_direct+
colcon test-result --test-result-base "$WS/build/forward_walk" --verbose
printf '\nUnified sources built: %s\nCore install: %s\n' "$WS" "$CORE_PREFIX"
