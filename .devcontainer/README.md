# Desktop Remote RViz Devcontainer

This devcontainer is for monitoring the robot from an Ubuntu 24.04 desktop with
a ROS 2 Humble RViz environment. It is not the robot runtime image.

On the desktop host, allow local Docker GUI access before opening the container:

```bash
xhost +local:docker
```

Open this repository in VS Code and choose **Dev Containers: Reopen in
Container**. The container uses host networking, CycloneDDS, and ROS domain `0`
by default.

If discovery works with automatic interface selection:

```bash
source /opt/ros/humble/setup.bash
ros2 topic list
rviz2
```

If the desktop has multiple network interfaces, bind CycloneDDS to the Wi-Fi
interface connected to the robot network:

```bash
source /opt/ros/humble/setup.bash
source .devcontainer/setup_remote_viz.bash enp97s0
ros2 topic list | grep -E '^/map$|^/frontier_goal$|^/cmd_vel$'
rviz2 -d .devcontainer/frontier_remote.rviz
```

The desktop interface connected to the robot network is currently expected to be
`enp97s0` with IP `192.168.111.100`. Replace it only if that host interface
changes. Keep `ROS_DOMAIN_ID` the same as the robot container.

For the normal desktop setup, the helper below does the source/setup step and
opens RViz with the preconfigured frontier-navigation layout:

```bash
.devcontainer/start_remote_rviz.bash
```

Pass a different interface only if the desktop robot-network NIC changes:

```bash
.devcontainer/start_remote_rviz.bash wlp2s0
```

## Local Rosbag Replay

For offline replay, use the copied bags under `./bags/`. This mode is local to
the devcontainer and does not need the robot DDS interface.

Open RViz first so replay starts from the beginning:

```bash
.devcontainer/start_replay_rviz.bash
```

In a second terminal, play the bag with the matching replay environment:

```bash
.devcontainer/play_frontier_bag.bash /workspaces/nav-frontier-go2w-v3/bags/frontier_results_YYYYMMDD_HHMMSS
```

`start_replay_rviz.bash` sets RViz `use_sim_time=true`, and
`play_frontier_bag.bash` publishes `/clock` with `ros2 bag play --clock`.
The RViz helper also derives `/map_known_cells` and `/map_unknown_cells` from
the replayed `/map` so the raw map's unknown-space square can be styled
separately from the known map cells.
