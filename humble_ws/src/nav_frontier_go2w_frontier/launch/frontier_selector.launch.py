"""Launch the frontier selector node with a default config file."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("nav_frontier_go2w_frontier")
    config = os.path.join(pkg_share, "config", "frontier_selector.yaml")

    score_lambda = LaunchConfiguration("score_lambda")
    info_radius_cells = LaunchConfiguration("info_radius_cells")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument("score_lambda", default_value="0.5"),
        DeclareLaunchArgument("info_radius_cells", default_value="2"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="nav_frontier_go2w_frontier",
            executable="frontier_selector",
            name="frontier_selector",
            output="screen",
            parameters=[
                config,
                {
                    "score_lambda": score_lambda,
                    "info_radius_cells": info_radius_cells,
                    "use_sim_time": use_sim_time,
                },
            ],
        ),
    ])
