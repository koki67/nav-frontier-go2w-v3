from __future__ import annotations

from pathlib import Path

import pytest
import yaml


PLANNER_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PLANNER_ROOT.parent
BRIDGE_ROOT = SRC_ROOT / "nav_frontier_go2w_bridge"


def _load_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def test_nav2_uses_navfn_and_mppi_omni():
    config = _load_yaml(PLANNER_ROOT / "config" / "nav2_params.yaml")

    planner = config["planner_server"]["ros__parameters"]["GridBased"]
    controller = config["controller_server"]["ros__parameters"]["FollowPath"]

    assert planner["plugin"] == "nav2_navfn_planner::NavfnPlanner"
    assert planner["allow_unknown"] is True
    assert controller["plugin"] == "nav2_mppi_controller::MPPIController"
    assert controller["motion_model"] == "Omni"
    assert "VelocityDeadbandCritic" in controller["critics"]


def test_nav2_topics_match_dlio_and_hesai_scan_pipeline():
    config = _load_yaml(PLANNER_ROOT / "config" / "nav2_params.yaml")

    assert config["bt_navigator"]["ros__parameters"]["odom_topic"] == "/dlio/odom_node/odom"
    assert config["velocity_smoother"]["ros__parameters"]["odom_topic"] == "/dlio/odom_node/odom"

    local_costmap = config["local_costmap"]["local_costmap"]["ros__parameters"]
    global_costmap = config["global_costmap"]["global_costmap"]["ros__parameters"]

    assert local_costmap["global_frame"] == "odom"
    assert local_costmap["obstacle_layer"]["scan"]["topic"] == "/scan"
    assert global_costmap["global_frame"] == "map"
    assert global_costmap["static_layer"]["map_subscribe_transient_local"] is True
    assert global_costmap["track_unknown_space"] is True


def test_nav2_velocity_caps_match_bridge_defaults():
    nav2 = _load_yaml(PLANNER_ROOT / "config" / "nav2_params.yaml")
    bridge = _load_yaml(BRIDGE_ROOT / "config" / "velocity_bridge.yaml")

    controller = nav2["controller_server"]["ros__parameters"]["FollowPath"]
    smoother = nav2["velocity_smoother"]["ros__parameters"]
    bridge_params = bridge["velocity_bridge"]["ros__parameters"]

    assert controller["vx_max"] == pytest.approx(bridge_params["vx_max"])
    assert controller["vy_max"] == pytest.approx(bridge_params["vy_max"])
    assert controller["wz_max"] == pytest.approx(bridge_params["wz_max"])
    assert smoother["max_velocity"] == pytest.approx([
        bridge_params["vx_max"],
        bridge_params["vy_max"],
        bridge_params["wz_max"],
    ])


def test_nav2_launch_uses_dedicated_params_argument():
    nav2_launch = (PLANNER_ROOT / "launch" / "nav2_navigation.launch.py").read_text(
        encoding="utf-8",
    )
    frontier_launch = (PLANNER_ROOT / "launch" / "frontier_planner.launch.py").read_text(
        encoding="utf-8",
    )

    assert 'LaunchConfiguration("nav2_params_file")' in nav2_launch
    assert 'DeclareLaunchArgument("nav2_params_file"' in nav2_launch
    assert 'LaunchConfiguration("params_file")' not in nav2_launch
    assert 'DeclareLaunchArgument("params_file"' not in nav2_launch

    assert 'LaunchConfiguration("nav2_params_file")' in frontier_launch
    assert '"nav2_params_file": nav2_params_file' in frontier_launch
    assert 'LaunchConfiguration("params_file")' not in frontier_launch
    assert 'DeclareLaunchArgument("params_file"' not in frontier_launch
