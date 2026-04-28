# Architecture

A bird's-eye view of the nav-frontier-go2w-v2 stack: what each layer does, how data flows between them, and the TF tree shape that has to hold for the system to make sense.

## Pipeline (9 stages)

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

## Per-package responsibility

| Package                            | Owns                                                                |
|------------------------------------|---------------------------------------------------------------------|
| `hesai_lidar`                      | Hesai PandarXT-16 driver. Publishes raw `PointCloud2` on `/points_raw`. |
| `go2w_imu_publisher`               | Reads Go2W `/lowstate` from the unitree DDS bridge, republishes as `sensor_msgs/Imu` on `/go2w/imu`. |
| `direct_lidar_inertial_odometry`   | LiDAR-Inertial odometry. Consumes `/points_raw` + `/go2w/imu`, publishes `/dlio/odom_node/odom` and broadcasts TF `odom → base_link`. The `map` topic on `dlio_map_node` is remapped to `dlio/map_node/map` so it doesn't collide with slam_toolbox's `/map`. |
| `go2w_slam_toolbox_bringup`        | Composite: `pointcloud_to_laserscan` (3D cloud → 2D `/scan`) + `slam_toolbox` (`/scan` → `/map` + TF `map → odom`). |
| `nav_frontier_go2w_frontier`       | Detects frontier cells in `/map`, clusters them by BFS, scores by `info_gain - λ * travel_cost`, publishes the best cluster's goal as `/frontier_goal` (`PoseStamped`) + visualization markers. |
| `nav_frontier_go2w_planner`        | Nav2 stack (NavFn global planner + `nav2_mppi_controller` local controller, Omni model) **and** a frontier_goal_executor that consumes `/frontier_goal` and dispatches `NavigateToPose` actions. Output of MPPI flows through Nav2's velocity_smoother to land on `/cmd_vel`. |
| `nav_frontier_go2w_bridge`         | Subscribes `/cmd_vel`, clamps to `(vx_max, vy_max, wz_max)`, encodes as `unitree_api/Request` (api_id=1008 Move, api_id=1003 StopMove), publishes on `/api/sport/request` at 50 Hz with a 0.5 s watchdog. |
| `nav_frontier_go2w_bringup`        | Top-level `bringup.launch.py`. Composes everything above plus the static TFs `base_link → hesai_lidar` (t=`[0.1634, 0, 0.116]`, yaw=+π/2) and `base_link → imu_link` (t=`[0,0,0]`, yaw=+π/2). |

## TF tree

```
map  (slam_toolbox publishes map → odom)
 └── odom  (D-LIO publishes odom → base_link)
      └── base_link  (static TFs from bringup)
            ├── hesai_lidar
            └── imu_link
```

If you only see `odom → base_link`, slam_toolbox isn't running or hasn't received any `/scan`. If `base_link → hesai_lidar` is missing, the `static_transform_publisher` from the bringup didn't start (check `ros2 run tf2_tools view_frames`).

## A frontier-→-motion cycle

1. `/map` updates from slam_toolbox at ~1 Hz.
2. `frontier_selector` runs at 2 Hz: detect frontier cells → cluster → score → publish best goal on `/frontier_goal`.
3. `frontier_goal_executor` receives the goal:
   - If idle, sends it to Nav2 via `BasicNavigator.goToPose()`.
   - If a goal is already active and the new one is far from it (> `min_goal_update_distance`), queue it as pending.
4. Nav2 plans a path (NavFn) and the controller (MPPI) emits velocity commands on `cmd_vel_nav`.
5. `velocity_smoother` smooths and republishes as `/cmd_vel`.
6. `velocity_bridge` clamps and forwards as `/api/sport/request` (Move) at 50 Hz.
7. When Nav2 reports SUCCEEDED / ABORTED / CANCELED, the executor clears the active slot and either dispatches the pending goal or waits for a fresh `/frontier_goal`.

## Safety properties

- **Hard velocity caps:** the bridge's `(vx_max, vy_max, wz_max)` are the floor of what reaches the robot. MPPI is configured with the same caps so it doesn't ask for more than the bridge will allow.
- **Watchdog StopMove:** if `/cmd_vel` falls silent for 0.5 s, the bridge emits a one-shot StopMove and then streams zero-Move every tick to keep the locomotion stack standing.
- **Dry-run mode:** `bridge_dry_run:=true` (the default) logs what the bridge would publish without touching the Sport API. Always start here when bringing up on a new robot or after firmware changes.
- **Goal timeout:** the executor cancels any goal that exceeds `goal_timeout_sec` (default 180 s).
- **Duplicate suppression:** /frontier_goal updates within `min_goal_update_distance` (default 0.5 m) of the active or pending goal are dropped, so the planner doesn't thrash.
