"""Composite launch: Nav2 stack + frontier goal executor.

Brings up the Nav2 lifecycle nodes (planner, controller with MPPI Omni, BT
navigator, smoother, velocity_smoother, behavior server, lifecycle manager)
plus the frontier_goal_executor that consumes /frontier_goal and dispatches
NavigateToPose actions.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory("nav_frontier_go2w_planner")
    nav2_default_params = os.path.join(pkg_share, "config", "nav2_params.yaml")
    executor_config = os.path.join(pkg_share, "config", "frontier_goal_executor.yaml")
    trajectory_lines_config = os.path.join(pkg_share, "config", "mppi_trajectory_lines.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    nav2_params_file = LaunchConfiguration("nav2_params_file")

    nav2_launch = PathJoinSubstitution([
        FindPackageShare("nav_frontier_go2w_planner"),
        "launch", "nav2_navigation.launch.py",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("nav2_params_file", default_value=nav2_default_params),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "nav2_params_file": nav2_params_file,
            }.items(),
        ),
        Node(
            package="nav_frontier_go2w_planner",
            executable="frontier_goal_executor",
            name="frontier_goal_executor",
            output="screen",
            parameters=[executor_config, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="nav_frontier_go2w_planner",
            executable="mppi_trajectory_lines",
            name="mppi_trajectory_lines",
            output="screen",
            parameters=[trajectory_lines_config, {"use_sim_time": use_sim_time}],
        ),
    ])
