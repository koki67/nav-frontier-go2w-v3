"""Launch the frontier selector node with a default config file."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("nav_frontier_go2w_frontier")
    config = os.path.join(pkg_share, "config", "frontier_selector.yaml")

    return LaunchDescription([
        Node(
            package="nav_frontier_go2w_frontier",
            executable="frontier_selector",
            name="frontier_selector",
            output="screen",
            parameters=[config],
        ),
    ])
