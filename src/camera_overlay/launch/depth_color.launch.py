"""Consume an existing RealSense depth topic; never launch the camera or motors."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {'depth_topic':('/camera/camera/depth/image_rect_raw',str),
                'camera_node':('/camera/camera',str),
                'min_depth_m':('0.1',float),'max_depth_m':('2.5',float),
                'show_debug_view':('false',bool),'stream_debug_view':('true',bool),
                'stream_debug_port':('8080',int)}
    return LaunchDescription([
        *[DeclareLaunchArgument(k,default_value=v[0]) for k,v in defaults.items()],
        Node(package='camera_overlay',executable='depth_color_viewer',output='screen',
             parameters=[{k:ParameterValue(LaunchConfiguration(k),value_type=v[1]) for k,v in defaults.items()}]),
    ])
