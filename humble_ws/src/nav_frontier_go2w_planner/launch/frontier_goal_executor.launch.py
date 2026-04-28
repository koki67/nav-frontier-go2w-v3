"""Launch the frontier_goal_executor node."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("nav_frontier_go2w_planner")
    config = os.path.join(pkg_share, "config", "frontier_goal_executor.yaml")

    return LaunchDescription([
        Node(
            package="nav_frontier_go2w_planner",
            executable="frontier_goal_executor",
            name="frontier_goal_executor",
            output="screen",
            parameters=[config],
        ),
    ])
