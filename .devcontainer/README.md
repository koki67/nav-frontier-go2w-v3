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
rviz2
```

The desktop interface connected to the robot network is currently expected to be
`enp97s0` with IP `192.168.111.100`. Replace it only if that host interface
changes. Keep `ROS_DOMAIN_ID` the same as the robot container.
