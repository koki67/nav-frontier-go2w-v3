# nav-frontier-go2w-v3

Autonomous frontier-exploration navigation for the Unitree Go2W, packaged as a
single ROS 2 Humble Docker workflow.

The stack uses Hesai LiDAR and Unitree low-state IMU data for D-LIO odometry,
builds a 2D occupancy map with `slam_toolbox`, selects frontiers from `/map`,
plans with Nav2, and sends the resulting velocity command through the Go2W
Sport API bridge.

## Dataflow

```text
Hesai /points_raw + Go2W /lowstate
        |
        v
D-LIO odometry + pointcloud_to_laserscan
        |
        v
slam_toolbox /map
        |
        v
frontier selector /frontier_goal
        |
        v
Nav2 global/local planning
        |
        v
/cmd_vel -> velocity bridge -> /api/sport/request
```

For the detailed architecture and topic catalog, see
[docs/architecture.md](docs/architecture.md) and
[docs/topics.md](docs/topics.md).

## Quick Start

Build the robot runtime image:

```bash
bash scripts/build_image.sh
```

Enter the robot container:

```bash
bash docker/run.sh
```

Launch the stack in dry-run mode:

```bash
ros2 launch nav_frontier_go2w_bringup bringup.launch.py
```

Dry-run is the default. To allow real robot motion, explicitly disable it:

```bash
ros2 launch nav_frontier_go2w_bringup bringup.launch.py \
    bridge_dry_run:=false \
    vx_max:=0.20 vy_max:=0.10 wz_max:=0.30
```

Useful launch defaults:

| Argument | Default | Meaning |
|----------|---------|---------|
| `use_rviz` | `false` | Start RViz inside the robot container. |
| `bridge_dry_run` | `true` | Log bridge commands without sending Sport API requests. |
| `vx_max`, `vy_max`, `wz_max` | `0.30`, `0.20`, `0.50` | Velocity caps applied by the bridge. |
| `record_results` | `false` | Record result topics to a timestamped rosbag. |
| `score_lambda` | `0.5` | Frontier distance penalty. |
| `info_radius_cells` | `2` | Frontier information-gain neighborhood. |

For dry-run validation and common bring-up issues, use
[docs/troubleshooting.md](docs/troubleshooting.md). For the velocity bridge
contract and safety behavior, use
[docs/velocity-bridge.md](docs/velocity-bridge.md).

## Remote RViz

Use this when you want a desktop/laptop RViz window to inspect the live robot
stack over Wi-Fi.

On the robot, start the container with remote DDS enabled:

```bash
bash docker/run.sh --remote-viz
```

Then launch the stack inside that container, usually with `use_rviz:=false`:

```bash
ros2 launch nav_frontier_go2w_bringup bringup.launch.py \
    bridge_dry_run:=true
```

On the desktop, open this repository in the VS Code devcontainer and start the
preconfigured RViz layout:

```bash
./.devcontainer/start_remote_rviz.bash
```

For interface selection, RViz display details, and bandwidth notes, see
[docs/remote-visualization.md](docs/remote-visualization.md) and
[.devcontainer/README.md](.devcontainer/README.md).

## Record Results

Use this when you want to inspect a run later offline.

On the robot, launch with recording enabled:

```bash
ros2 launch nav_frontier_go2w_bringup bringup.launch.py \
    bridge_dry_run:=false \
    record_results:=true
```

Each run creates a timestamped bag under `/external/bags` inside the container.
When using `docker/run.sh`, that maps to `./bags` in the repository on the host.

To replay copied bags on the desktop devcontainer, put the bag directory under
`./bags`, then start RViz first:

```bash
./.devcontainer/start_replay_rviz.bash
```

In a second devcontainer terminal, play the bag:

```bash
./.devcontainer/play_frontier_bag.bash \
    /workspaces/nav-frontier-go2w-v3/bags/frontier_results_YYYYMMDD_HHMMSS
```

For the recorded topic set and replay details, see
[docs/result-recording.md](docs/result-recording.md).

## Repository Layout

```text
docker/             Robot runtime image and container entrypoints
humble_ws/src/      ROS 2 packages and vendored dependencies
config/             DDS, Nav2, and slam_toolbox configuration
catmux/             tmux session bringup
scripts/            build, enter, and smoke-test helpers
docs/               Architecture, topics, recording, RViz, and troubleshooting
bags/               Local drop directory for copied rosbag results
```

Vendored source details are tracked in
[docs/vendored-upstreams.md](docs/vendored-upstreams.md).

## License

MIT, see [LICENSE](LICENSE).
