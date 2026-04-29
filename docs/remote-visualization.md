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

## What To Inspect

DDS exposes the ROS 2 graph on the selected interfaces; this option does not
filter by topic. In RViz, set the fixed frame to `map` and add the displays you
need for the current debugging session.

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
| MarkerArray | `/trajectories` | MPPI sampled trajectory visualization when available. |
| Path | `/transformed_global_plan` | MPPI/controller path visualization when available. |

Costmaps are useful for understanding why Nav2 accepts or rejects candidate
motion. The frontier selector itself consumes `/map`, not the Nav2 costmaps, so
inspect `/map`, `/frontier_markers`, and `/frontier_goal` when debugging
frontier selection.

## Bandwidth Notes

Start with `/map`, costmaps, TF, `/frontier_goal`, and `/frontier_markers`.
Enable `/points_raw` and `/dlio/map_node/map` only when you need point-cloud
detail; they are the most likely topics to saturate Wi-Fi or make RViz lag.
