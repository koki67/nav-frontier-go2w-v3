# Remote Visualization

The validated default keeps CycloneDDS bound to the robot/internal interface
(`eth0`). Use the optional remote-visualization mode when you want a laptop on
the robot Wi-Fi network to inspect the same ROS 2 graph in RViz.

Start the container on the robot with Wi-Fi DDS enabled:

```bash
bash docker/run.sh --remote-viz
```

By default this keeps `eth0` for the robot DDS graph and adds `wlan0` for the
remote laptop. If the robot uses another Wi-Fi interface name, pass it
explicitly:

```bash
bash docker/run.sh --remote-viz --remote-viz-iface wlan1
```

The option generates a runtime CycloneDDS profile and does not edit
`config/cyclonedds.xml`. If `wlan0` is missing, the script falls back to `eth0`
only and prints a warning.

## Laptop Setup

The laptop must use the same ROS domain and a compatible DDS implementation:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
rviz2
```

If the laptop has multiple network interfaces and discovery is unreliable, bind
CycloneDDS to the laptop Wi-Fi interface with a local profile, for example:

```bash
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="wlan0" priority="1" multicast="true" /></Interfaces></General></Domain></CycloneDDS>'
rviz2
```

Confirm discovery before opening a full RViz layout:

```bash
ros2 topic list | grep -E '^/map$|^/frontier_goal$|^/cmd_vel$'
```

### Humble Devcontainer On Ubuntu 24.04

For an Ubuntu 24.04 desktop, use the repo's `.devcontainer/` setup to run RViz
from a ROS 2 Humble container instead of mixing Jazzy on the laptop with Humble
on the robot.

Before opening the devcontainer, allow local Docker GUI access on the desktop:

```bash
xhost +local:docker
```

Then open the repository in VS Code and choose **Dev Containers: Reopen in
Container**. Inside the container, start with automatic CycloneDDS interface
selection:

```bash
source /opt/ros/humble/setup.bash
ros2 topic list | grep -E '^/map$|^/frontier_goal$|^/cmd_vel$'
rviz2 -d .devcontainer/frontier_remote.rviz
```

If the desktop has multiple interfaces and discovery is unreliable, bind
CycloneDDS to the Wi-Fi interface connected to the robot network:

```bash
source .devcontainer/setup_remote_viz.bash enp97s0
ros2 topic list | grep -E '^/map$|^/frontier_goal$|^/cmd_vel$'
rviz2 -d .devcontainer/frontier_remote.rviz
```

For your current desktop, the robot-network interface is `enp97s0` with IP
`192.168.111.100`. Replace it only if that host interface changes.

For the usual desktop setup, this helper runs the same DDS setup and opens the
saved RViz layout:

```bash
.devcontainer/start_remote_rviz.bash
```

## What To Inspect

DDS exposes the ROS 2 graph on the selected interfaces; this option does not
filter by topic. The devcontainer RViz profile sets the fixed frame to `map`
and preloads the displays below. If launching RViz manually, add the same
displays for the current debugging session.

Core frontier-navigation workflow:

| RViz display | Topic |
|--------------|-------|
| TF | `/tf`, `/tf_static` |
| Map | `/map` |
| Map | `/global_costmap/costmap` |
| Map | `/local_costmap/costmap` |
| Pose | `/frontier_goal` |
| MarkerArray | `/frontier_markers` |
| Odometry | `/dlio/odom_node/odom` |
| Path | `/dlio/odom_node/path` |

Sensor and controller diagnostics:

| RViz display | Topic | Notes |
|--------------|-------|-------|
| LaserScan | `/scan` | Lighter than the full point cloud. |
| PointCloud2 | `/points_raw` | Useful but bandwidth-heavy over Wi-Fi. |
| PointCloud2 | `/dlio/map_node/map` | D-LIO map cloud; bandwidth-heavy. |
| MarkerArray | `/mppi_trajectory_lines` | Thin-line visualization of MPPI sampled trajectories. |
| MarkerArray | `/trajectories` | Raw MPPI sampled trajectory markers, disabled by default in RViz. |
| Path | `/transformed_global_plan` | MPPI/controller path visualization when available. |

Costmaps are useful for understanding why Nav2 accepts or rejects candidate
motion. The frontier selector itself consumes `/map`, not the Nav2 costmaps, so
inspect `/map`, `/frontier_markers`, and `/frontier_goal` when debugging
frontier selection.

## Bandwidth Notes

Start with `/map`, costmaps, TF, `/frontier_goal`, `/frontier_markers`, and the
D-LIO map cloud. Keep `/points_raw` disabled unless you need raw LiDAR detail;
it is the topic most likely to saturate Wi-Fi or make RViz lag.
