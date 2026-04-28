"""Launch the Nav2 stack (NavFn + MPPI Omni) with our parameter file."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("nav_frontier_go2w_planner")
    default_params = os.path.join(pkg_share, "config", "nav2_params.yaml")

    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    log_level = LaunchConfiguration("log_level")

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "smoother_server",
        "velocity_smoother",
    ]

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("log_level", default_value="info"),

        Node(
            package="nav2_controller", executable="controller_server",
            name="controller_server", output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
            arguments=["--ros-args", "--log-level", log_level],
            # MPPI publishes onto cmd_vel_nav; the velocity_smoother below
            # republishes its smoothed stream as cmd_vel for our bridge.
            remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
        ),
        Node(
            package="nav2_planner", executable="planner_server",
            name="planner_server", output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
            arguments=["--ros-args", "--log-level", log_level],
            remappings=remappings,
        ),
        Node(
            package="nav2_behaviors", executable="behavior_server",
            name="behavior_server", output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
            arguments=["--ros-args", "--log-level", log_level],
            remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
        ),
        Node(
            package="nav2_bt_navigator", executable="bt_navigator",
            name="bt_navigator", output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
            arguments=["--ros-args", "--log-level", log_level],
            remappings=remappings,
        ),
        Node(
            package="nav2_smoother", executable="smoother_server",
            name="smoother_server", output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
            arguments=["--ros-args", "--log-level", log_level],
            remappings=remappings,
        ),
        Node(
            package="nav2_velocity_smoother", executable="velocity_smoother",
            name="velocity_smoother", output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
            arguments=["--ros-args", "--log-level", log_level],
            remappings=remappings + [
                ("cmd_vel", "cmd_vel_nav"),       # input from controller
                ("cmd_vel_smoothed", "cmd_vel"),  # output our bridge consumes
            ],
        ),
        Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="lifecycle_manager_navigation", output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"autostart": autostart},
                {"node_names": lifecycle_nodes},
            ],
            arguments=["--ros-args", "--log-level", log_level],
        ),
    ])
