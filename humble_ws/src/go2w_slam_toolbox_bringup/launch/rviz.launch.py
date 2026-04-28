"""
Launch RViz2 with the bundled go2w_slam_toolbox.rviz config. Visualises:
  - TF tree (map -> odom -> base_link -> hesai_lidar)
  - /points_raw (Hesai PointCloud2)
  - /scan (projected LaserScan)
  - /map (slam_toolbox occupancy grid)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('go2w_slam_toolbox_bringup')
    default_rviz = os.path.join(pkg_share, 'config', 'go2w_slam_toolbox.rviz')

    rviz_config = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz,
            description='RViz config file to load.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock from a bag/sim instead of system time.',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
