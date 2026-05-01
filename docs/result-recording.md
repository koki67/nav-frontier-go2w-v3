# Result Recording

The top-level bringup can record a timestamped rosbag of the frontier-navigation
results for offline RViz inspection. Recording is disabled by default.

Start the stack with:

```bash
ros2 launch nav_frontier_go2w_bringup bringup.launch.py \
    use_rviz:=false \
    bridge_dry_run:=false \
    record_results:=true
```

By default, bags are written inside the container to `/external/bags`, which is
the host repository mounted by `docker/run.sh`. On the host, the same bags appear
under `./bags/`.

Useful overrides:

```bash
record_bag_dir:=/external/bags
record_bag_prefix:=frontier_results
record_storage:=sqlite3
```

Each run creates a unique directory such as:

```text
/external/bags/frontier_results_20260430_153012
```

## Recorded Topics

The recorder uses a topic regex and `--include-hidden-topics`, so it captures
matching topics as they appear while the stack is running.

Default result topics:

| Data | Topics |
|------|--------|
| TF tree | `/tf`, `/tf_static` |
| Occupancy map | `/map`, `/map_updates` |
| D-LIO map and trajectory | `/dlio/map_node/map`, `/dlio/odom_node/odom`, `/dlio/odom_node/pose`, `/dlio/odom_node/path`, `/dlio/odom_node/keyframes` |
| Frontier selection | `/frontier_goal`, `/frontier_markers` |
| Nav2 costmaps | `/global_costmap/costmap`, `/global_costmap/costmap_raw`, `/local_costmap/costmap`, `/local_costmap/costmap_raw` |
| Planner/controller visualization | `/plan`, `/global_plan`, `/local_plan`, `/received_global_plan`, `/transformed_global_plan`, `/trajectories`, `/mppi_trajectory_lines`, `/optimal_trajectory` |
| Nav2 actions | `/navigate_to_pose/_action/*`, `/compute_path_to_pose/_action/*`, `/follow_path/_action/*`, `/smooth_path/_action/*` |
| Motion commands | `/cmd_vel_nav`, `/cmd_vel`, `/api/sport/request`, `/api/sport/response` |
| Lightweight sensor context | `/scan` |

Costmaps are Nav2 planning/controller products, not the frontier selector's
internal score map. The frontier selector state is represented by `/map`,
`/frontier_markers`, and `/frontier_goal`.

The default profile intentionally does not record high-rate raw sensors such as
`/points_raw`, `/go2w/imu`, or `/lowstate`. Record those separately only when
you need to debug odometry input data; they can dominate disk usage quickly.

## Replay

For desktop replay, copy the bag directory under the repository's local `bags/`
directory and use the Humble devcontainer.

Open RViz first so playback starts from the beginning:

```bash
.devcontainer/start_replay_rviz.bash
```

In a second devcontainer terminal, play the bag:

```bash
.devcontainer/play_frontier_bag.bash /workspaces/nav-frontier-go2w-v3/bags/frontier_results_YYYYMMDD_HHMMSS
```

`start_replay_rviz.bash` starts RViz with `use_sim_time=true`, and
`play_frontier_bag.bash` publishes `/clock` with `ros2 bag play --clock`.
The preconfigured RViz layout already includes `/map`, costmaps,
`/frontier_goal`, `/frontier_markers`, D-LIO odometry/path, MPPI trajectories,
and `/cmd_vel`.
