# nav-frontier-go2w-v3

Autonomous **frontier exploration** navigation framework for the Unitree **Go2W** quadruped, packaged as a single ROS 2 Humble docker image.

The robot has no pre-built map. It uses Hesai LiDAR + IMU for LiDAR-Inertial Odometry (D-LIO), builds a 2D occupancy grid with `slam_toolbox`, picks the next exploration goal from frontier cells, plans with Nav2 (NavFn + MPPI), and pipes the resulting `cmd_vel` to the Go2W Sport API.

## Pipeline

```
Hesai LiDAR /points_raw ─┐
                         ├─► D-LIO ─► /dlio/odom_node/odom + TF odom→base_link
IMU /go2w/imu ───────────┘                                      │
                                                                ▼
                  pointcloud_to_laserscan ─► /scan ─► slam_toolbox ─► /map + TF map→odom
                                                                ▼
              Frontier selector (reads /map, listens to TF) ─► /frontier_goal (PoseStamped)
                                                                ▼
                            Nav2 (NavFn global + MPPI local) ─► /cmd_vel (Twist)
                                                                ▼
                                           Velocity bridge ─► /api/sport/request
                                                                ▼
                                                          Go2W Sport API
```

## Repo layout

```
nav-frontier-go2w-v3/
├── docker/             # Single-image build: Dockerfile, entrypoint.sh, run.sh
├── humble_ws/src/      # ROS 2 packages (vendored deps + 4 in-repo packages)
├── config/             # cyclonedds.xml, nav2_params.yaml, slam_toolbox_params.yaml, ...
├── catmux/             # tmux session bringup
├── scripts/            # build_image.sh, enter.sh, smoke_test.sh
└── docs/               # architecture.md, topics.md, velocity-bridge.md, troubleshooting.md
```

## Packages

| Package                            | Origin           | Role |
|------------------------------------|------------------|------|
| `direct_lidar_inertial_odometry`   | Vendored         | LiDAR-Inertial Odometry (D-LIO) |
| `hesai_lidar`                      | Vendored         | Hesai PandarXT-16 driver |
| `go2w_imu_publisher`               | Vendored         | LowState → sensor_msgs/Imu |
| `unitree_api`, `unitree_go`        | Vendored (msg-only) | Sport API request/response types |
| `go2w_slam_toolbox_bringup`        | Vendored         | pointcloud_to_laserscan + slam_toolbox launch |
| `nav_frontier_go2w_bridge`         | This repo        | Twist → Sport API bridge (50 Hz, watchdog, dry-run) |
| `nav_frontier_go2w_frontier`       | This repo        | BFS frontier detection + selector node with expanded-neighbourhood info gain |
| `nav_frontier_go2w_planner`        | This repo        | Nav2 wrapper (NavFn + MPPI) + frontier goal executor |
| `nav_frontier_go2w_bringup`        | This repo        | Top-level `bringup.launch.py` |

## Quick start

```bash
# Build the single docker image (arm64 robot target):
bash scripts/build_image.sh

# Optional amd64 Dockerfile smoke build on an x86 host:
NAV_FRONTIER_PLATFORM=linux/amd64 NAV_FRONTIER_IMAGE=nav-frontier-go2w-v3:amd64-smoke \
    bash scripts/build_image.sh

# Enter the running container with X11 / CycloneDDS forwarded.
# docker/run.sh mirrors the host ROS_DOMAIN_ID; if unset it defaults to 0,
# which matches the stock Unitree onboard ROS graph.
bash docker/run.sh

# Optional: also expose the ROS 2 graph over Wi-Fi for laptop RViz monitoring.
bash docker/run.sh --remote-viz

# Inside the container — the workspace is built at image time. Launch the full stack:
ros2 launch nav_frontier_go2w_bringup bringup.launch.py \
    use_rviz:=false \
    bridge_dry_run:=true \
    vx_max:=0.20 vy_max:=0.10 wz_max:=0.30
```

Start with `bridge_dry_run:=true` and conservative velocity caps. Once you have validated `/cmd_vel`, `/frontier_goal`, and `/map` in RViz, set `bridge_dry_run:=false` to enable motion.

## Verify dry-run bringup

Keep the launch terminal running. Open a second terminal and enter the same
container:

```bash
bash scripts/enter.sh
source /workspace/humble_ws/install/setup.bash
```

In that terminal, confirm dry-run mode and check that the main pipeline topics
are publishing:

```bash
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
ros2 param get /velocity_bridge dry_run
ros2 topic hz /points_raw
ros2 topic hz /lowstate
ros2 topic hz /go2w/imu
ros2 topic hz /scan
ros2 topic echo --once /map
ros2 topic echo --once /frontier_goal
ros2 topic hz /cmd_vel_nav
ros2 topic hz /cmd_vel
```

The expected dry-run result is: `dry_run=True`, sensor topics are live, `/map`
and `/frontier_goal` exist, Nav2 publishes `/cmd_vel_nav`, and the velocity
smoother publishes `/cmd_vel`. Finally confirm that dry-run is not sending real
Sport API requests:

```bash
timeout 5 ros2 topic echo --once /api/sport/request
```

No message should be printed before the timeout. If any stage is missing, use
`docs/troubleshooting.md` before running with `bridge_dry_run:=false`.

## Safety notes

The velocity bridge enforces hard caps `(vx_max, vy_max, wz_max)` on every Twist before encoding it as a Sport API request. A 0.5 s `/cmd_vel` watchdog automatically emits a `StopMove` (`api_id=1003`) if the upstream stack stops publishing. Always keep the e-stop within reach for live tests.

See `docs/architecture.md` for the dataflow, `docs/topics.md` for the full topic catalog, `docs/remote-visualization.md` for laptop RViz monitoring, `docs/velocity-bridge.md` for the bridge contract, and `docs/troubleshooting.md` for common issues.

## License

MIT — see `LICENSE`.
