"""Top-level launch for the full Go2W frontier exploration stack.

Stages composed (in start order):
    1. Static TFs        base_link -> imu_link, base_link -> hesai_lidar
    2. Hesai LiDAR       publishes /points_raw (PointCloud2)
    3. IMU publisher     publishes /go2w/imu (sensor_msgs/Imu)
    4. D-LIO             publishes /dlio/odom_node/odom + TF odom -> base_link
                         (its 'map' topic is remapped to dlio/map_node/map to
                         avoid colliding with slam_toolbox's /map)
    5. pointcloud_to_laserscan  /points_raw -> /scan
    6. slam_toolbox      /scan -> /map + TF map -> odom
    7. Frontier selector /map -> /frontier_goal (PoseStamped)
    8. Nav2 + executor   /frontier_goal -> NavigateToPose -> /cmd_vel
    9. Velocity bridge   /cmd_vel -> /api/sport/request (Sport API)
   10. RViz              optional, gated by use_rviz launch arg

Static TFs use the Go2W extrinsics from D-LIO's dlio.yaml:
    base_link -> hesai_lidar : t=[0.1634, 0, 0.116]   yaw=+pi/2
    base_link -> imu_link    : t=[0,      0, 0]       yaw=+pi/2
"""
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _yaw_quat_z_w(yaw_rad: float) -> tuple[str, str]:
    return (str(math.sin(yaw_rad / 2.0)), str(math.cos(yaw_rad / 2.0)))


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    bridge_dry_run = LaunchConfiguration("bridge_dry_run")
    vx_max = LaunchConfiguration("vx_max")
    vy_max = LaunchConfiguration("vy_max")
    wz_max = LaunchConfiguration("wz_max")
    score_lambda = LaunchConfiguration("score_lambda")
    info_radius_cells = LaunchConfiguration("info_radius_cells")

    declared_args = [
        DeclareLaunchArgument("use_sim_time", default_value="false",
                              description="Use /clock from a bag or sim."),
        DeclareLaunchArgument("use_rviz", default_value="false",
                              description="Start RViz with the slam_toolbox config."),
        DeclareLaunchArgument("bridge_dry_run", default_value="true",
                              description="Bridge logs Move/Stop without publishing /api/sport/request."),
        DeclareLaunchArgument("vx_max", default_value="0.30"),
        DeclareLaunchArgument("vy_max", default_value="0.20"),
        DeclareLaunchArgument("wz_max", default_value="0.50"),
        DeclareLaunchArgument("score_lambda", default_value="0.5",
                              description="Frontier scoring lambda: score = info_gain - lambda * travel_cost(m)."),
        DeclareLaunchArgument("info_radius_cells", default_value="2",
                              description="Cell-radius expansion used when counting frontier information gain."),
    ]

    # ---- Static TFs (Go2W URDF extrinsics, taken from D-LIO dlio.yaml) ----
    qz, qw = _yaw_quat_z_w(math.pi / 2.0)
    static_tf_imu = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_imu_static_tf",
        arguments=[
            "--x", "0.0", "--y", "0.0", "--z", "0.0",
            "--qx", "0.0", "--qy", "0.0", "--qz", qz, "--qw", qw,
            "--frame-id", "base_link", "--child-frame-id", "imu_link",
        ],
    )
    static_tf_lidar = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_hesai_static_tf",
        arguments=[
            "--x", "0.1634", "--y", "0.0", "--z", "0.116",
            "--qx", "0.0", "--qy", "0.0", "--qz", qz, "--qw", qw,
            "--frame-id", "base_link", "--child-frame-id", "hesai_lidar",
        ],
    )

    # ---- 2. Hesai LiDAR ---------------------------------------------------
    hesai_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("hesai_lidar"), "launch", "hesai_lidar_launch.py",
        ])),
    )

    # ---- 3. IMU publisher (no launch file in the package) ----------------
    imu_node = Node(
        package="go2w_imu_publisher",
        executable="imu_publisher",
        name="go2w_imu_publisher",
        output="screen",
    )

    # ---- 4. D-LIO ---------------------------------------------------------
    dlio_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("direct_lidar_inertial_odometry"), "launch", "dlio.launch.py",
        ])),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # ---- 5+6. pointcloud_to_laserscan + slam_toolbox ---------------------
    slam_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("go2w_slam_toolbox_bringup"),
            "launch", "go2w_hesai_slam_toolbox.launch.py",
        ])),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_rviz": "false",  # delegated to the top-level use_rviz toggle
        }.items(),
    )

    # ---- 7. Frontier selector --------------------------------------------
    frontier_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("nav_frontier_go2w_frontier"),
            "launch", "frontier_selector.launch.py",
        ])),
        launch_arguments={
            "score_lambda": score_lambda,
            "info_radius_cells": info_radius_cells,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # ---- 8. Nav2 stack + frontier goal executor --------------------------
    planner_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("nav_frontier_go2w_planner"),
            "launch", "frontier_planner.launch.py",
        ])),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # ---- 9. Velocity bridge ----------------------------------------------
    bridge_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("nav_frontier_go2w_bridge"),
            "launch", "velocity_bridge.launch.py",
        ])),
        launch_arguments={
            "vx_max": vx_max,
            "vy_max": vy_max,
            "wz_max": wz_max,
            "dry_run": bridge_dry_run,
        }.items(),
    )

    # ---- 10. RViz (optional) ---------------------------------------------
    rviz_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("go2w_slam_toolbox_bringup"), "launch", "rviz.launch.py",
        ])),
        condition=IfCondition(use_rviz),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    return LaunchDescription([
        *declared_args,
        static_tf_imu,
        static_tf_lidar,
        hesai_include,
        imu_node,
        dlio_include,
        slam_include,
        frontier_include,
        planner_include,
        bridge_include,
        rviz_include,
    ])
