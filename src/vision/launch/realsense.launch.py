"""Shared camera launch. Edit the three defaults below, then restart the camera.
The installed ~/.config/jandi copy is shared by MAIN and FAST; no rebuild needed.
Runtime viewer controls do not rewrite this file.
"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# ========== User depth exposure settings (RGB settings are untouched) ==========
DEPTH_AUTO_EXPOSURE = False
DEPTH_EXPOSURE = 5000
DEPTH_GAIN = 16
# =============================================================================


def generate_launch_description():
    defaults = {
        'depth_module.enable_auto_exposure': str(DEPTH_AUTO_EXPOSURE).lower(),
        'depth_module.exposure': str(DEPTH_EXPOSURE),
        'depth_module.gain': str(DEPTH_GAIN),
    }
    driver = Path(get_package_share_directory('realsense2_camera'))/'launch/rs_launch.py'
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name,value in defaults.items()],
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(driver)),
            launch_arguments={name:LaunchConfiguration(name) for name in defaults}.items()),
    ])
