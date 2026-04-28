# nav-frontier-go2w-v2

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
nav-frontier-go2w-v2/
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
| `nav_frontier_go2w_frontier`       | This repo        | BFS frontier detection + selector node |
| `nav_frontier_go2w_planner`        | This repo        | Nav2 wrapper (NavFn + MPPI) + frontier goal executor |
| `nav_frontier_go2w_bringup`        | This repo        | Top-level `bringup.launch.py` |

## Quick start

```bash
# Build the single docker image (arm64 robot target):
bash scripts/build_image.sh

# Enter the running container with X11 / CycloneDDS forwarded:
bash docker/run.sh

# Inside the container — the workspace is built at image time. Launch the full stack:
ros2 launch nav_frontier_go2w_bringup bringup.launch.py \
    use_rviz:=false \
    bridge_dry_run:=true \
    vx_max:=0.20 vy_max:=0.10 wz_max:=0.30
```

Start with `bridge_dry_run:=true` and conservative velocity caps. Once you have validated `/cmd_vel`, `/frontier_goal`, and `/map` in RViz, set `bridge_dry_run:=false` to enable motion.

## Safety notes

The velocity bridge enforces hard caps `(vx_max, vy_max, wz_max)` on every Twist before encoding it as a Sport API request. A 0.5 s `/cmd_vel` watchdog automatically emits a `StopMove` (`api_id=1003`) if the upstream stack stops publishing. Always keep the e-stop within reach for live tests.

See `docs/architecture.md` for the dataflow, `docs/topics.md` for the full topic catalog, `docs/velocity-bridge.md` for the bridge contract, and `docs/troubleshooting.md` for common issues.

## License

MIT — see `LICENSE`.
