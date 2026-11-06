"""Bootstrap the detached runtime; Ctrl+C/SSH loss never kills robot nodes."""
from pathlib import Path

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    executable = str(Path(get_package_prefix("vision")) / "lib/vision/jandi")
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="practice", choices=["practice", "IRC"]),
        DeclareLaunchArgument("workspace", default_value="",
                              description="Empty: derive Jetson workspace from yolo26_runtime.yaml"),
        DeclareLaunchArgument("core_prefix", default_value="",
                              description="Empty: VISION_CORE_ROOT or ~/vision_core/install"),
        ExecuteProcess(cmd=[executable, "--background", LaunchConfiguration("mode"),
                            "--workspace", LaunchConfiguration("workspace"),
                            "--core-prefix", LaunchConfiguration("core_prefix")],
                       additional_env={"ROS_DOMAIN_ID": "22"}, output="screen"),
    ])
