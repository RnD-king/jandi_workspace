# Jandi — Jetson development environment

Recorded: 2026-11-17 (KST). Versions below are derived from commands run on the competition Jetson; historical/unverified values are identified.

## Hardware and operating system

| Item | Version or description |
| --- | --- |
| Device-tree model | NVIDIA Jetson Orin Nano Engineering Reference Developer Kit Super |
| Memory | 7.4 GiB reported by `free -h` (8 GB class) |
| Storage | 119.4 GiB reported by `lsblk` (128 GB class); root: `/dev/mmcblk0p1` |
| CPU architecture | aarch64 |
| OS | Ubuntu 22.04.5 LTS (jammy) |
| L4T | R36.4.4 |
| JetPack | L4T release corresponds to JetPack 6.2.1; the `nvidia-jetpack` metapackage version was not displayed |
| Kernel | 5.15.148-tegra |
| CUDA Toolkit | 12.6, V12.6.68 |
| TensorRT | 10.3.0.30 (`libnvinfer10`; packages have `+cuda12.5` build suffix) |
| Python | 3.10.12 |
| ROS 2 | Humble |
| ROS middleware | `rmw_fastrtps_cpp` |

`ROS_DOMAIN_ID` was **unset in the shell used for inspection**; Jandi previously used 22. Do not assume it was globally configured.

## External dependencies (sources not bundled)

| Component | Observed version/state | Source/notes |
| --- | --- | --- |
| librealsense | 2.58.1, `v2.58.1-dirty` | Git commit `bf2778061d5dd29776e9aca8765f75852671760b` |
| realsense-ros | `4.58.1-dirty` | Git commit `53aceeac29e2e848e5e1ede5430124f07b2fe924` |
| Dynamixel SDK (ROS APT) | 4.0.3 | `ros-humble-dynamixel-sdk 4.0.3-1jammy.20260307.131019` |
| Intel RealSense camera | D435i | Hardware previously reported |
| RealSense camera firmware | 5.17.3.10 | Historical value; not reverified during this backup |

The `-dirty` suffix means local source tree differences from the tagged commit. **In particular the realsense-ros `realsense2_camera/launch/rs_launch.py` was modified on the Jetson (20 changed lines: 14 additions, 6 deletions)** and must be preserved separately under `docs/external_overrides/realsense-ros/`. The two `rs_launch.py.bak_*` backups are intentionally excluded. The patch and full custom launch file are **not included until explicitly committed**.

librealsense also reports dirty; its precise local differences were not inspected, so the upstream commit alone is not sufficient for a byte-identical rebuild.

## Jandi source backup

Repository: https://github.com/RnD-king/jandi_workspace

| Purpose | Branch | Verified Jetson source commit prior to documentation/exclusion updates |
| --- | --- | --- |
| Main | `main` | `7d289c592c5b32913cc19bbf049b70397b2e22ee` |
| Fast | `race-round2-fast-20261109` | `d675bbc1a05b9936946ba032cad8d7534b66d43c` |
| Original Jandi + extra YOLO weights | `backup/jandi-jetson-20261117` | `7d3ea2601f7439ed8013c9ce31c76793079c9793` |

Included own code: `vision`, `vision_core` (C++ sources/library; no `package.xml` was found in the listed package scan), `forward_walk`, `camera_overlay`, plus project configuration, motions and tracked models.

Intentionally excluded from all three final branch tips: `src/robot_msgs/` and `src/robot_bringup/`, as requested. Also excludes external source repositories, ROS build/install/log outputs, caches, and temporary files. **Removing the two packages may prevent a clean standalone build if the remaining sources refer to their interfaces.** Their earlier Git history remains accessible unless rewritten.

Main/Fast tracked model files: `src/dataset/best.pt`, `src/dataset/best.engine`; SHA-256 on both local workspaces respectively:
- `best.pt`: `916f408bbe4231978bd14a4d19ae7c15cdddb68efaf4c80132fe1496ecfb1e3e`
- `best.engine`: `95472e06ae9e16a0bc799231bc057e171462041f5b819a37f2377e153ba2caea`

The original Jandi backup also contains `best_1026`, `best_1029` and `best_1108` sets in PT / ONNX / TensorRT engine formats. TensorRT engine files may require regeneration for a different Jetson / CUDA / TensorRT environment.

## RealSense override restore (when added)

1. Check out the repository at the desired branch.
2. Install matching `realsense-ros` upstream source and dependencies.
3. Copy `docs/external_overrides/realsense-ros/rs_launch.py` to `realsense2_camera/launch/rs_launch.py` in the `realsense-ros` checkout, **after verifying the upstream revision**. Compare against `rs_launch.patch` if present.

This document records versions and backup boundaries; it does not guarantee a turnkey rebuild.
