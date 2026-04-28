"""
Top-level launch for the Go2W 2D mapping stack:

    /points_raw  -->  pointcloud_to_laserscan  -->  /scan  -->  slam_toolbox  -->  /map

Prerequisite (NOT started here): the slam-go2w bringup must already be running
to provide /points_raw, /go2w/imu, and the odom -> base_link -> hesai_lidar TF
tree via D-LIO. See the README for the exact sequence.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('go2w_slam_toolbox_bringup')
    default_p2l = os.path.join(pkg_share, 'config', 'pointcloud_to_laserscan.yaml')
    default_slam = os.path.join(pkg_share, 'config', 'slam_toolbox.yaml')
    default_rviz = os.path.join(pkg_share, 'config', 'go2w_slam_toolbox.rviz')

    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    cloud_topic = LaunchConfiguration('cloud_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    params_file_p2l = LaunchConfiguration('params_file_p2l')
    params_file_slam = LaunchConfiguration('params_file_slam')
    rviz_config = LaunchConfiguration('rviz_config')

    declared_args = [
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='Start RViz2 with the bundled config.'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use /clock (set to true when replaying bags).'),
        DeclareLaunchArgument('cloud_topic', default_value='/points_raw',
                              description='Hesai PointCloud2 topic (slam-go2w default).'),
        DeclareLaunchArgument('scan_topic', default_value='/scan',
                              description='Projected LaserScan topic for slam_toolbox.'),
        DeclareLaunchArgument('params_file_p2l', default_value=default_p2l,
                              description='pointcloud_to_laserscan parameter YAML.'),
        DeclareLaunchArgument('params_file_slam', default_value=default_slam,
                              description='slam_toolbox parameter YAML.'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz,
                              description='RViz config file.'),
    ]

    p2l_launch = PathJoinSubstitution([
        FindPackageShare('go2w_slam_toolbox_bringup'), 'launch',
        'pointcloud_to_laserscan.launch.py',
    ])
    slam_launch = PathJoinSubstitution([
        FindPackageShare('go2w_slam_toolbox_bringup'), 'launch',
        'slam_toolbox.launch.py',
    ])
    rviz_launch = PathJoinSubstitution([
        FindPackageShare('go2w_slam_toolbox_bringup'), 'launch',
        'rviz.launch.py',
    ])

    includes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(p2l_launch),
            launch_arguments={
                'params_file': params_file_p2l,
                'cloud_topic': cloud_topic,
                'scan_topic': scan_topic,
                'use_sim_time': use_sim_time,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={
                'params_file': params_file_slam,
                'use_sim_time': use_sim_time,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rviz_launch),
            launch_arguments={
                'rviz_config': rviz_config,
                'use_sim_time': use_sim_time,
            }.items(),
            condition=IfCondition(use_rviz),
        ),
    ]

    return LaunchDescription(declared_args + includes)
