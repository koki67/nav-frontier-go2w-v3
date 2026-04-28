"""
Launch slam_toolbox in online async mode. Subscribes to /scan and the
odom -> base_link TF (provided externally by slam-go2w D-LIO), publishes
map -> odom and the /map occupancy grid.

Standalone launch — useful when /scan is already being published. Top-level
launch (go2w_hesai_slam_toolbox.launch.py) includes this same file.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('go2w_slam_toolbox_bringup')
    default_params = os.path.join(pkg_share, 'config', 'slam_toolbox.yaml')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to slam_toolbox parameter YAML.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock from a bag/sim instead of system time.',
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
    ])
