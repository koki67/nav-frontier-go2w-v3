"""Launch the velocity bridge with parameters loaded from velocity_bridge.yaml.

All params are exposed as launch arguments so the top-level bringup can
override them (e.g. tighten `vx_max` for early on-robot tests, or flip
`dry_run:=true` for a no-motion smoke run).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("nav_frontier_go2w_bridge")
    default_config = os.path.join(pkg_share, "config", "velocity_bridge.yaml")

    declared_args = [
        DeclareLaunchArgument("vx_max", default_value="0.30"),
        DeclareLaunchArgument("vy_max", default_value="0.20"),
        DeclareLaunchArgument("wz_max", default_value="0.50"),
        DeclareLaunchArgument("watchdog_timeout", default_value="0.5"),
        DeclareLaunchArgument("publish_rate", default_value="50.0"),
        DeclareLaunchArgument("dry_run", default_value="false"),
        DeclareLaunchArgument("emit_stop_on_idle", default_value="true"),
    ]

    node = Node(
        package="nav_frontier_go2w_bridge",
        executable="velocity_bridge",
        name="velocity_bridge",
        output="screen",
        parameters=[
            default_config,
            {
                "vx_max": LaunchConfiguration("vx_max"),
                "vy_max": LaunchConfiguration("vy_max"),
                "wz_max": LaunchConfiguration("wz_max"),
                "watchdog_timeout": LaunchConfiguration("watchdog_timeout"),
                "publish_rate": LaunchConfiguration("publish_rate"),
                "dry_run": LaunchConfiguration("dry_run"),
                "emit_stop_on_idle": LaunchConfiguration("emit_stop_on_idle"),
            },
        ],
    )

    return LaunchDescription(declared_args + [node])
