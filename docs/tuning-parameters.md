# Tuning Parameters

This document lists the main controllable parameters in the current frontier
navigation stack and explains which subsystem each parameter affects.

The first tuning rule is to inspect the failure mode in RViz before changing
values. For corridor problems, compare `/map`, `/global_costmap/costmap`,
`/local_costmap/costmap`, `/plan`, `/transformed_global_plan`, `/trajectories`,
and `/cmd_vel`. The correct knob depends on where the pipeline first becomes
wrong.

## Configuration Files

| Subsystem | File |
|-----------|------|
| Top-level launch arguments | `humble_ws/src/nav_frontier_go2w_bringup/launch/bringup.launch.py` |
| D-LIO odometry | `humble_ws/src/direct_lidar_inertial_odometry/cfg/params.yaml` |
| D-LIO frames and extrinsics | `humble_ws/src/direct_lidar_inertial_odometry/cfg/dlio.yaml` |
| Point cloud to laser scan | `humble_ws/src/go2w_slam_toolbox_bringup/config/pointcloud_to_laserscan.yaml` |
| 2D mapping | `humble_ws/src/go2w_slam_toolbox_bringup/config/slam_toolbox.yaml` |
| Frontier selector | `humble_ws/src/nav_frontier_go2w_frontier/config/frontier_selector.yaml` |
| Frontier goal executor | `humble_ws/src/nav_frontier_go2w_planner/config/frontier_goal_executor.yaml` |
| MPPI trajectory line visualization | `humble_ws/src/nav_frontier_go2w_planner/config/mppi_trajectory_lines.yaml` |
| Nav2 global/local planning and costmaps | `humble_ws/src/nav_frontier_go2w_planner/config/nav2_params.yaml` |
| Velocity bridge | `humble_ws/src/nav_frontier_go2w_bridge/config/velocity_bridge.yaml` |

## Top-Level Launch Arguments

These can be changed from the main launch command.

| Argument | Default | Effect |
|----------|---------|--------|
| `use_sim_time` | `false` | Uses `/clock`; mainly for bag replay or simulation. |
| `use_rviz` | `false` | Starts RViz inside the robot container. Usually keep false for remote RViz. |
| `bridge_dry_run` | `true` | If true, velocity bridge logs commands but does not publish Sport API requests. |
| `vx_max` | `0.30` | Forward velocity cap passed to the bridge. |
| `vy_max` | `0.20` | Lateral velocity cap passed to the bridge. |
| `wz_max` | `0.50` | Yaw-rate cap passed to the bridge. |
| `score_lambda` | `0.5` | Frontier selector distance penalty. |
| `info_radius_cells` | `2` | Frontier selector information-gain neighborhood. |
| `record_results` | `false` | Starts result rosbag recording. |
| `record_bag_dir` | `/external/bags` | Output directory for recorded bags. |
| `record_bag_prefix` | `frontier_results` | Prefix for timestamped bag directory names. |
| `record_storage` | `sqlite3` | rosbag2 storage backend. |

Keep `vx_max`, `vy_max`, and `wz_max` consistent with MPPI and
`velocity_smoother` limits. If MPPI generates commands above the bridge caps,
the bridge will clamp them, which changes the controller's assumed behavior.

## D-LIO Parameters

D-LIO provides odometry and TF for the navigation stack. The frontier selector
does not use D-LIO's dense map for selecting frontiers; it uses `/map` from
`slam_toolbox`. D-LIO tuning still matters because bad odometry produces bad
mapping, TF, and costmap alignment.

### General D-LIO Flags

| Parameter | Current Value | Effect |
|-----------|---------------|--------|
| `use_sim_time` | `false` | Uses `/clock` when replaying bags or simulation. |
| `version` | `1.1.1` | D-LIO config version marker. |
| `adaptive` | `true` | Enables D-LIO adaptive behavior. |
| `pointcloud/deskew` | `true` | Deskews each LiDAR scan using motion estimates. |
| `pointcloud/voxelize` | `true` | Enables point cloud voxelization. |
| `imu/calibration` | `true` | Enables IMU intrinsic calibration support. |
| `imu/intrinsics/accel/bias` | `[0.0, 0.0, 0.0]` | Initial accelerometer bias. |
| `imu/intrinsics/accel/sm` | identity matrix | Accelerometer scale/misalignment matrix. |
| `imu/intrinsics/gyro/bias` | `[0.0, 0.0, 0.0]` | Initial gyro bias. |

### Frames And Extrinsics

| Parameter | Current Value | Effect |
|-----------|---------------|--------|
| `frames/odom` | `odom` | D-LIO odometry frame. |
| `frames/baselink` | `base_link` | Robot base frame. |
| `frames/lidar` | `hesai_lidar` | LiDAR frame. |
| `frames/imu` | `imu_link` | IMU frame. |
| `extrinsics/baselink2imu/t` | `[0., 0., 0.]` | Base-to-IMU translation. Change only for hardware calibration. |
| `extrinsics/baselink2imu/R` | yaw +90 deg matrix | Base-to-IMU rotation. Must match static TF. |
| `extrinsics/baselink2lidar/t` | `[0.1634, 0., 0.116]` | Base-to-LiDAR translation. Change only for hardware calibration. |
| `extrinsics/baselink2lidar/R` | yaw +90 deg matrix | Base-to-LiDAR rotation. Must match static TF. |

### Odometry And Mapping

| Parameter | Current Value | Effect |
|-----------|---------------|--------|
| `map/waitUntilMove` | `true` | Delays some map publishing until motion; initial keyframe is still registered in D-LIO. |
| `map/dense/filtered` | `false` | Controls filtering of dense D-LIO map output. |
| `map/sparse/leafSize` | `0.01` | Sparse map voxel leaf size. Smaller is denser and heavier. |
| `odom/gravity` | `9.80665` | Gravity magnitude for inertial integration. |
| `odom/computeTimeOffset` | `true` | Estimates LiDAR/IMU time offset. |
| `odom/imu/approximateGravity` | `false` | Uses approximate gravity handling when enabled. |
| `odom/imu/calibration/gyro` | `true` | Calibrates gyro bias at startup. |
| `odom/imu/calibration/accel` | `true` | Calibrates accel bias at startup. |
| `odom/imu/calibration/time` | `3.0` | Startup stillness/calibration window in seconds. |
| `odom/imu/bufferSize` | `5000` | IMU buffer capacity. |
| `odom/preprocessing/cropBoxFilter/size` | `0.4` | Removes points near the sensor/robot body. |
| `odom/preprocessing/voxelFilter/res` | `0.03` | LiDAR voxel filter resolution. Larger is lighter but less detailed. |
| `odom/keyframe/threshD` | `0.4` | Translation threshold for adding a keyframe. |
| `odom/keyframe/threshR` | `30.0` | Rotation threshold in degrees for adding a keyframe. |
| `odom/submap/keyframe/knn` | `10` | Nearest keyframes used in the local submap. |
| `odom/submap/keyframe/kcv` | `10` | Convex-hull keyframes used in the submap. |
| `odom/submap/keyframe/kcc` | `10` | Concave-hull keyframes used in the submap. |

### GICP And Inertial Observer

| Parameter | Current Value | Effect |
|-----------|---------------|--------|
| `odom/gicp/minNumPoints` | `64` | Minimum points for scan registration. |
| `odom/gicp/kCorrespondences` | `16` | Neighbor count for GICP covariances. |
| `odom/gicp/maxCorrespondenceDistance` | `0.5` | Max point correspondence distance in meters. |
| `odom/gicp/maxIterations` | `32` | Registration iteration limit. |
| `odom/gicp/transformationEpsilon` | `0.01` | Translation convergence threshold. |
| `odom/gicp/rotationEpsilon` | `0.01` | Rotation convergence threshold. |
| `odom/gicp/initLambdaFactor` | `1e-9` | Initial GICP optimization damping factor. |
| `odom/geo/Kp` | `4.5` | Geometric observer position gain. |
| `odom/geo/Kv` | `11.25` | Geometric observer velocity gain. |
| `odom/geo/Kq` | `4.0` | Geometric observer attitude gain. |
| `odom/geo/Kab` | `2.25` | Accel-bias correction gain. |
| `odom/geo/Kgb` | `1.0` | Gyro-bias correction gain. |
| `odom/geo/abias_max` | `5.0` | Accel-bias correction limit. |
| `odom/geo/gbias_max` | `0.5` | Gyro-bias correction limit. |

Tune D-LIO only after confirming the robot starts still during IMU calibration
and that LiDAR/IMU extrinsics match the physical robot.

## 2D Map Generation

The actual frontier map is `/map` from `slam_toolbox`, built from `/scan`.
For corridor issues, this category is often as important as Nav2 tuning. If the
corridor is not open in `/map`, no planner tuning can fix the root cause.

### PointCloud To LaserScan

| Parameter | Current Value | Effect |
|-----------|---------------|--------|
| `target_frame` | `base_link` | Frame in which the cloud is sliced. |
| `transform_tolerance` | `0.05` | TF tolerance for point cloud transforms. |
| `min_height` | `-0.10` | Lower vertical slice in `base_link`. |
| `max_height` | `0.40` | Upper vertical slice in `base_link`. |
| `angle_min` | `-3.14159` | Start angle for the generated scan. |
| `angle_max` | `3.14159` | End angle for the generated scan. |
| `angle_increment` | `0.0087` | Angular resolution, about 0.5 degrees. |
| `scan_time` | `0.1` | Expected scan period for 10 Hz LiDAR. |
| `range_min` | `0.30` | Minimum valid scan range. |
| `range_max` | `30.0` | Maximum valid scan range. |
| `use_inf` | `true` | Uses infinity for no-return bins. |
| `inf_epsilon` | `1.0` | Offset used when representing infinity. |
| `concurrency_level` | `1` | Worker concurrency. |

For narrow corridors, first inspect `/scan` in RViz. If walls are missing,
noisy, or the floor/body is projected as obstacles, tune `min_height`,
`max_height`, `range_min`, and `range_max`.

### Slam Toolbox

| Parameter | Current Value | Effect |
|-----------|---------------|--------|
| `solver_plugin` | `solver_plugins::CeresSolver` | Graph optimization solver plugin. |
| `ceres_linear_solver` | `SPARSE_NORMAL_CHOLESKY` | Ceres linear solver. |
| `ceres_preconditioner` | `SCHUR_JACOBI` | Ceres preconditioner. |
| `ceres_trust_strategy` | `LEVENBERG_MARQUARDT` | Ceres trust-region strategy. |
| `ceres_dogleg_type` | `TRADITIONAL_DOGLEG` | Ceres dogleg strategy. |
| `ceres_loss_function` | `None` | Ceres robust loss function. |
| `odom_frame` | `odom` | Odometry frame. |
| `map_frame` | `map` | Map frame. |
| `base_frame` | `base_link` | Robot frame. |
| `scan_topic` | `/scan` | Input LaserScan topic. |
| `use_map_saver` | `true` | Enables map saver integration. |
| `mode` | `mapping` | Online mapping mode. |
| `debug_logging` | `false` | Enables verbose slam_toolbox logging. |
| `throttle_scans` | `1` | Uses every scan. |
| `transform_publish_period` | `0.05` | `map -> odom` TF publish period. |
| `map_update_interval` | `1.0` | Map publication interval. |
| `resolution` | `0.05` | Occupancy grid resolution. |
| `min_laser_range` | `0.30` | Minimum range used by SLAM. |
| `max_laser_range` | `30.0` | Maximum range used by SLAM. |
| `minimum_time_interval` | `0.1` | Minimum time between processed scans. |
| `transform_timeout` | `0.2` | TF lookup timeout. |
| `tf_buffer_duration` | `30.0` | TF buffer length. |
| `stack_size_to_use` | `40000000` | Stack size for slam_toolbox processing. |
| `enable_interactive_mode` | `true` | Enables interactive map tools. |
| `use_scan_matching` | `true` | Enables scan matching. |
| `use_scan_barycenter` | `true` | Uses scan barycenter in matching. |
| `minimum_travel_distance` | `0.5` | Movement distance before adding scans to graph. |
| `minimum_travel_heading` | `0.5` | Heading change before adding scans to graph. |
| `scan_buffer_size` | `10` | Scan buffer count. |
| `scan_buffer_maximum_scan_distance` | `10.0` | Max scan distance in buffer matching. |
| `link_match_minimum_response_fine` | `0.1` | Minimum fine response for scan links. |
| `link_scan_maximum_distance` | `1.5` | Maximum scan-link distance. |
| `do_loop_closing` | `true` | Enables loop closure. |
| `loop_search_maximum_distance` | `3.0` | Loop closure search radius. |
| `loop_match_minimum_chain_size` | `10` | Minimum chain size for loop closure. |
| `loop_match_maximum_variance_coarse` | `3.0` | Coarse loop variance threshold. |
| `loop_match_minimum_response_coarse` | `0.35` | Coarse loop response threshold. |
| `loop_match_minimum_response_fine` | `0.45` | Fine loop response threshold. |
| `correlation_search_space_dimension` | `0.5` | Local scan-match search size. |
| `correlation_search_space_resolution` | `0.01` | Local search resolution. |
| `correlation_search_space_smear_deviation` | `0.1` | Local search smear. |
| `loop_search_space_dimension` | `8.0` | Loop closure search size. |
| `loop_search_space_resolution` | `0.05` | Loop search resolution. |
| `loop_search_space_smear_deviation` | `0.03` | Loop search smear. |
| `distance_variance_penalty` | `0.5` | Scan matcher distance penalty. |
| `angle_variance_penalty` | `1.0` | Scan matcher angle penalty. |
| `fine_search_angle_offset` | `0.00349` | Fine angular search window. |
| `coarse_search_angle_offset` | `0.349` | Coarse angular search window. |
| `coarse_angle_resolution` | `0.0349` | Coarse angular resolution. |
| `minimum_angle_penalty` | `0.9` | Minimum angle penalty. |
| `minimum_distance_penalty` | `0.5` | Minimum distance penalty. |
| `use_response_expansion` | `true` | Expands matching responses. |
| `min_pass_through` | `2` | Minimum pass-through count. |
| `occupancy_threshold` | `0.1` | Occupancy threshold used by slam_toolbox. |

## Costmaps

Costmaps are Nav2 planning inputs. They are not used by the frontier selector,
but they strongly determine whether a selected frontier can actually be reached.

### Shared Costmap Knobs

| Parameter | Local | Global | Effect |
|-----------|-------|--------|--------|
| `robot_radius` | `0.35` | `0.35` | Circular robot footprint approximation. This is a primary corridor knob. |
| `resolution` | `0.05` | `0.05` | Costmap resolution. |
| `inflation_layer.cost_scaling_factor` | `3.0` | `3.0` | How quickly inflated costs decay with distance from obstacles. |
| `inflation_layer.inflation_radius` | `0.70` | `0.70` | Distance around obstacles that receives inflated cost. Primary corridor knob. |
| `always_send_full_costmap` | `true` | `true` | Publishes full costmaps for inspection. |

### Local Costmap

| Parameter | Current Value | Effect |
|-----------|---------------|--------|
| `update_frequency` | `10.0` | Local costmap update rate. |
| `publish_frequency` | `5.0` | Local costmap publish rate. |
| `global_frame` | `odom` | Local costmap frame. |
| `robot_base_frame` | `base_link` | Robot frame. |
| `rolling_window` | `true` | Costmap follows the robot. |
| `width` | `6` | Local costmap width in meters. |
| `height` | `6` | Local costmap height in meters. |
| `plugins` | `["obstacle_layer", "inflation_layer"]` | Active local layers. |
| `obstacle_layer.plugin` | `nav2_costmap_2d::ObstacleLayer` | Local obstacle layer implementation. |
| `obstacle_layer.enabled` | `true` | Enables obstacle processing. |
| `obstacle_layer.scan.topic` | `/scan` | Obstacle source. |
| `obstacle_layer.scan.data_type` | `LaserScan` | Input sensor type. |
| `obstacle_layer.scan.clearing` | `true` | Uses rays to clear free space. |
| `obstacle_layer.scan.marking` | `true` | Marks obstacles from scan returns. |
| `obstacle_layer.scan.max_obstacle_height` | `2.0` | Max obstacle height. |
| `obstacle_layer.scan.raytrace_max_range` | `8.0` | Max clearing ray length. |
| `obstacle_layer.scan.raytrace_min_range` | `0.0` | Min clearing ray length. |
| `obstacle_layer.scan.obstacle_max_range` | `6.0` | Max marking range. |
| `obstacle_layer.scan.obstacle_min_range` | `0.1` | Min marking range. |
| `inflation_layer.plugin` | `nav2_costmap_2d::InflationLayer` | Local inflation layer implementation. |

### Global Costmap

| Parameter | Current Value | Effect |
|-----------|---------------|--------|
| `update_frequency` | `1.0` | Global costmap update rate. |
| `publish_frequency` | `1.0` | Global costmap publish rate. |
| `global_frame` | `map` | Global costmap frame. |
| `robot_base_frame` | `base_link` | Robot frame. |
| `track_unknown_space` | `true` | Preserves unknown space in the costmap. |
| `plugins` | `["static_layer", "inflation_layer"]` | Active global layers. |
| `static_layer.plugin` | `nav2_costmap_2d::StaticLayer` | Uses `/map` as global costmap source. |
| `static_layer.map_subscribe_transient_local` | `true` | Receives latched map. |
| `static_layer.subscribe_to_updates` | `true` | Uses incremental map updates. |
| `inflation_layer.plugin` | `nav2_costmap_2d::InflationLayer` | Global inflation layer implementation. |

For a corridor that looks open in `/map` but blocked in costmaps, inspect
`robot_radius`, `inflation_radius`, and `cost_scaling_factor` first. Do not set
`robot_radius` smaller than the real robot clearance just to make plans appear.

## Frontier Selector

| Parameter | Current Value | Effect |
|-----------|---------------|--------|
| `map_topic` | `/map` | OccupancyGrid source. |
| `global_frame` | `map` | Goal frame and TF target frame. |
| `robot_base_frame` | `base_link` | Robot pose frame. |
| `goal_topic` | `/frontier_goal` | Published goal topic. |
| `markers_topic` | `/frontier_markers` | Published marker topic. |
| `update_rate` | `2.0` | Frontier selection rate in Hz. |
| `connectivity` | `8` | Frontier clustering connectivity. |
| `min_cluster_size` | `8` | Rejects small noisy clusters. |
| `score_lambda` | `0.5` | Distance penalty in frontier scoring. |
| `info_radius_cells` | `2` | Unknown-cell count neighborhood. |
| `publish_markers` | `true` | Publishes RViz markers. |
| `tf_timeout` | `0.2` | TF lookup timeout. |

If the robot chooses distant frontiers too aggressively, increase
`score_lambda`. If it ignores useful larger openings, decrease `score_lambda`
or increase `info_radius_cells`. If it chases noisy map speckles, increase
`min_cluster_size`.

## Frontier Goal Executor

| Parameter | Current Value | Effect |
|-----------|---------------|--------|
| `frontier_goal_topic` | `/frontier_goal` | Input from selector. |
| `global_frame` | `map` | Navigation goal frame. |
| `robot_base_frame` | `base_link` | Robot frame for duplicate suppression. |
| `min_goal_update_distance` | `0.5` | Drops similar active/pending goals within this distance. |
| `goal_timeout_sec` | `180.0` | Navigation goal timeout. |
| `result_check_rate` | `2.0` | Action-result polling rate. |
| `tf_timeout` | `0.2` | TF lookup timeout. |

If Nav2 thrashes between nearby frontiers, increase `min_goal_update_distance`.
If it waits too long on impossible goals, reduce `goal_timeout_sec`.

## MPPI Trajectory Line Visualization

This helper is visual-only. It subscribes to Nav2 MPPI's raw `/trajectories`
MarkerArray and republishes normalized thin line markers on
`/mppi_trajectory_lines` for RViz and rosbag replay.

| Parameter | Current Value | Effect |
|-----------|---------------|--------|
| `input_topic` | `/trajectories` | Raw MPPI trajectory MarkerArray. |
| `output_topic` | `/mppi_trajectory_lines` | Normalized line MarkerArray for RViz. |
| `line_width` | `0.015` | Width of sampled trajectory lines. |
| `alpha` | `0.35` | Line transparency. |
| `color_rgb` | `[80.0, 180.0, 255.0]` | Default RGB line color. Values above 1.0 are treated as 0-255. |
| `z_offset` | `0.04` | Small vertical offset to keep lines visible over maps/costmaps. |
| `max_markers` | `400` | Maximum trajectories converted per message; `0` means unlimited. |
| `point_stride` | `1` | Uses every Nth point from each incoming marker. |
| `preserve_marker_color` | `false` | Uses MPPI marker colors instead of the configured color when true. |

## Global Planner

| Parameter | Current Value | Effect |
|-----------|---------------|--------|
| `expected_planner_frequency` | `5.0` | Expected global planning rate. |
| `costmap_update_timeout` | `1.0` | Wait time for global costmap freshness. |
| `planner_plugins` | `["GridBased"]` | Active planner list. |
| `GridBased.plugin` | `nav2_navfn_planner/NavfnPlanner` | NavFn global planner. |
| `GridBased.tolerance` | `0.50` | Allows goal tolerance when exact goal is blocked. |
| `GridBased.use_astar` | `false` | Uses Dijkstra when false; A* when true. |
| `GridBased.allow_unknown` | `true` | Allows planning through unknown cells. |

If `/frontier_goal` exists but `/plan` does not enter a corridor, inspect the
global costmap first, then tune `robot_radius`, inflation, and planner
`tolerance`. `allow_unknown=true` is useful for exploration, but it can produce
optimistic plans near unmapped areas.

## Path Smoother

| Parameter | Current Value | Effect |
|-----------|---------------|--------|
| `smoother_plugins` | `["simple_smoother"]` | Active smoother list. |
| `simple_smoother.plugin` | `nav2_smoother::SimpleSmoother` | Smoother implementation. |
| `simple_smoother.tolerance` | `1.0e-10` | Optimizer convergence tolerance. |
| `simple_smoother.max_its` | `1000` | Max smoothing iterations. |
| `simple_smoother.do_refinement` | `true` | Enables refinement pass. |

If the global path is valid but too sharp for local control, inspect
`/plan_smoothed` and consider smoother tuning after costmap and MPPI tuning.

## BT Navigator And Behavior Server

| Parameter | Current Value | Effect |
|-----------|---------------|--------|
| `bt_navigator.global_frame` | `map` | Navigation global frame. |
| `bt_navigator.robot_base_frame` | `base_link` | Robot base frame. |
| `bt_navigator.odom_topic` | `/dlio/odom_node/odom` | Odom topic used by Nav2. |
| `bt_navigator.bt_loop_duration` | `10` | Behavior tree loop duration in ms. |
| `bt_navigator.default_server_timeout` | `20` | Default action server timeout. |
| `bt_navigator.wait_for_service_timeout` | `1000` | Service wait timeout. |
| `bt_navigator.action_server_result_timeout` | `900.0` | Action result timeout. |
| `bt_navigator.navigators` | `["navigate_to_pose"]` | Enabled navigator plugins. |
| `behavior_server.local_costmap_topic` | `local_costmap/costmap_raw` | Local costmap used by behaviors. |
| `behavior_server.global_costmap_topic` | `global_costmap/costmap_raw` | Global costmap used by behaviors. |
| `behavior_server.local_footprint_topic` | `local_costmap/published_footprint` | Local footprint topic. |
| `behavior_server.global_footprint_topic` | `global_costmap/published_footprint` | Global footprint topic. |
| `behavior_server.cycle_frequency` | `10.0` | Behavior server update rate. |
| `behavior_server.behavior_plugins` | `["spin", "backup", "drive_on_heading", "wait"]` | Enabled recovery behaviors. |
| `behavior_server.local_frame` | `odom` | Local behavior frame. |
| `behavior_server.global_frame` | `map` | Global behavior frame. |
| `behavior_server.robot_base_frame` | `base_link` | Robot frame. |
| `behavior_server.transform_tolerance` | `0.1` | TF tolerance. |
| `behavior_server.simulate_ahead_time` | `2.0` | Behavior simulation horizon. |
| `behavior_server.max_rotational_vel` | `1.0` | Max rotational velocity for behaviors. |
| `behavior_server.min_rotational_vel` | `0.2` | Min rotational velocity for behaviors. |
| `behavior_server.rotational_acc_lim` | `2.0` | Rotational acceleration limit. |

These are less likely to be the first corridor-tuning knobs, but they matter if
Nav2 enters recovery behavior frequently.

## MPPI Local Controller

MPPI decides the short-horizon command that follows the global path while
respecting local costmap costs.

### Main MPPI Parameters

| Parameter | Current Value | Effect |
|-----------|---------------|--------|
| `controller_frequency` | `10.0` | Controller server frequency. |
| `costmap_update_timeout` | `0.50` | Local costmap freshness wait. |
| `min_x_velocity_threshold` | `0.001` | Small x velocities below this are treated as zero. |
| `min_y_velocity_threshold` | `0.001` | Small y velocities below this are treated as zero. |
| `min_theta_velocity_threshold` | `0.001` | Small yaw velocities below this are treated as zero. |
| `progress_checker_plugins` | `["progress_checker"]` | Active progress checker plugins. |
| `goal_checker_plugins` | `["goal_checker"]` | Active goal checker plugins. |
| `controller_plugins` | `["FollowPath"]` | Active controller plugins. |
| `FollowPath.time_steps` | `30` | Number of trajectory samples in time. |
| `FollowPath.model_dt` | `0.10` | Time between trajectory states. Horizon is about 3.0 s. |
| `FollowPath.batch_size` | `500` | Number of sampled trajectories per iteration. |
| `FollowPath.iteration_count` | `1` | MPPI optimization iterations per cycle. |
| `FollowPath.prune_distance` | `1.5` | Distance behind robot to prune path. |
| `FollowPath.transform_tolerance` | `0.2` | TF tolerance. |
| `FollowPath.temperature` | `0.35` | Sampling selectivity. |
| `FollowPath.gamma` | `0.015` | Control cost weight. |
| `FollowPath.motion_model` | `Omni` | Omnidirectional motion model. |
| `FollowPath.visualize` | `true` | Publishes trajectory visualization. |
| `FollowPath.regenerate_noises` | `true` | Regenerates control noise. |

### MPPI Velocity And Acceleration Limits

| Parameter | Current Value | Effect |
|-----------|---------------|--------|
| `FollowPath.vx_max` | `0.30` | Max forward velocity. |
| `FollowPath.vx_min` | `-0.10` | Max reverse velocity. |
| `FollowPath.vy_max` | `0.20` | Max lateral velocity magnitude. |
| `FollowPath.wz_max` | `0.50` | Max yaw rate. |
| `FollowPath.ax_max` | `0.60` | Max forward acceleration. |
| `FollowPath.ax_min` | `-0.80` | Max forward deceleration/reverse acceleration. |
| `FollowPath.ay_max` | `0.50` | Max lateral acceleration. |
| `FollowPath.ay_min` | `-0.50` | Max lateral deceleration. |
| `FollowPath.az_max` | `1.00` | Max angular acceleration. |
| `FollowPath.vx_std` | `0.10` | Forward velocity sampling noise. |
| `FollowPath.vy_std` | `0.07` | Lateral velocity sampling noise. |
| `FollowPath.wz_std` | `0.18` | Angular velocity sampling noise. |

In narrow corridors, high lateral freedom can make sampled trajectories brush
inflated walls. If the robot weaves, consider reducing `vy_std` or `vy_max` and
increasing path-following weights, but verify with `/trajectories`.

### Progress And Goal Checking

| Parameter | Current Value | Effect |
|-----------|---------------|--------|
| `progress_checker.required_movement_radius` | `0.25` | Required progress distance. |
| `progress_checker.movement_time_allowance` | `15.0` | Time allowed before declaring no progress. |
| `goal_checker.xy_goal_tolerance` | `0.30` | Position tolerance at goal. |
| `goal_checker.yaw_goal_tolerance` | `0.30` | Heading tolerance at goal. |
| `goal_checker.stateful` | `true` | Keeps goal-checking state once partially satisfied. |

If the robot stops short at frontier goals, inspect goal tolerance. If it aborts
while trying to squeeze through, inspect progress checking and local costmap
costs before relaxing progress criteria.

### MPPI Critics

| Critic Parameter | Current Value | Effect |
|------------------|---------------|--------|
| `ConstraintCritic.enabled` | `true` | Enables constraint critic. |
| `ConstraintCritic.cost_power` | `1` | Critic exponent. |
| `ConstraintCritic.cost_weight` | `4.0` | Constraint cost weight. |
| `CostCritic.enabled` | `true` | Enables costmap cost critic. |
| `CostCritic.cost_power` | `1` | Critic exponent. |
| `CostCritic.cost_weight` | `4.0` | Weight for costmap cost. |
| `CostCritic.near_collision_cost` | `253` | Near-collision cost threshold. |
| `CostCritic.critical_cost` | `300.0` | Critical cost threshold. |
| `CostCritic.collision_cost` | `1000000.0` | Collision penalty. |
| `CostCritic.consider_footprint` | `false` | Uses footprint-aware cost when true. |
| `CostCritic.near_goal_distance` | `0.8` | Changes cost behavior near goal. |
| `CostCritic.trajectory_point_step` | `2` | Step between trajectory points evaluated for cost. |
| `GoalCritic.cost_weight` | `5.0` | Pulls trajectories toward goal position. |
| `GoalCritic.threshold_to_consider` | `1.2` | Distance where goal critic becomes relevant. |
| `GoalAngleCritic.cost_weight` | `3.0` | Pulls trajectories toward goal orientation. |
| `GoalAngleCritic.threshold_to_consider` | `0.5` | Distance where goal-angle critic becomes relevant. |
| `PathAlignCritic.cost_weight` | `10.0` | Penalizes misalignment with global path. |
| `PathAlignCritic.max_path_occupancy_ratio` | `0.05` | Occupancy threshold for using path alignment. |
| `PathAlignCritic.trajectory_point_step` | `4` | Step between evaluated trajectory points. |
| `PathAlignCritic.threshold_to_consider` | `0.5` | Distance where path alignment is considered. |
| `PathAlignCritic.offset_from_furthest` | `16` | Path point offset used by critic. |
| `PathAlignCritic.use_path_orientations` | `false` | Uses path orientations when true. |
| `PathFollowCritic.cost_weight` | `6.0` | Encourages progress along global path. |
| `PathFollowCritic.offset_from_furthest` | `5` | Path point offset used by critic. |
| `PathFollowCritic.threshold_to_consider` | `1.2` | Distance where path follow is considered. |
| `PathAngleCritic.cost_weight` | `2.0` | Penalizes poor path heading. |
| `PathAngleCritic.offset_from_furthest` | `4` | Path point offset used by critic. |
| `PathAngleCritic.threshold_to_consider` | `0.6` | Distance where path angle is considered. |
| `PathAngleCritic.max_angle_to_furthest` | `0.8` | Max allowed angle to path target. |
| `PathAngleCritic.mode` | `0` | Path angle critic mode. |
| `VelocityDeadbandCritic.cost_weight` | `1.0` | Penalizes velocities inside deadband. |
| `VelocityDeadbandCritic.deadband_velocities` | `[0.03, 0.03, 0.05]` | Velocity deadband for x, y, yaw. |

If `/plan` goes through a corridor but `/trajectories` avoid it, focus on local
costmap inflation and `CostCritic`. If trajectories enter the corridor but do
not stay centered, inspect `PathAlignCritic`, `PathFollowCritic`, `vy_std`, and
`wz_std`.

## Velocity Smoother And Bridge

| Parameter | Current Value | Effect |
|-----------|---------------|--------|
| `velocity_smoother.smoothing_frequency` | `20.0` | Output command rate. |
| `velocity_smoother.stamp_smoothed_velocity_with_smoothing_time` | `false` | Timestamp policy for smoothed commands. |
| `velocity_smoother.scale_velocities` | `false` | Scales velocity vector to satisfy limits when true. |
| `velocity_smoother.feedback` | `OPEN_LOOP` | Smoothing feedback mode. |
| `velocity_smoother.max_velocity` | `[0.30, 0.20, 0.50]` | Smoothed velocity caps. |
| `velocity_smoother.min_velocity` | `[-0.10, -0.20, -0.50]` | Smoothed reverse/min caps. |
| `velocity_smoother.max_accel` | `[0.60, 0.50, 1.00]` | Smoothed acceleration limits. |
| `velocity_smoother.max_decel` | `[-0.80, -0.50, -1.00]` | Smoothed deceleration limits. |
| `velocity_smoother.odom_topic` | `/dlio/odom_node/odom` | Odom source. |
| `velocity_smoother.odom_duration` | `0.1` | Odom history duration used by smoother. |
| `velocity_smoother.deadband_velocity` | `[0.0, 0.0, 0.0]` | Output deadband for x, y, yaw. |
| `velocity_smoother.velocity_timeout` | `1.0` | Timeout before zeroing command. |
| `bridge.vx_max` | `0.30` | Final forward command clamp. |
| `bridge.vy_max` | `0.20` | Final lateral command clamp. |
| `bridge.wz_max` | `0.50` | Final yaw command clamp. |
| `bridge.watchdog_timeout` | `0.5` | Emits stop if `/cmd_vel` becomes stale. |
| `bridge.publish_rate` | `50.0` | Sport API request publish rate. |
| `bridge.dry_run` | launch default `true` | Logs instead of publishing Sport API requests. |
| `bridge.emit_stop_on_idle` | `true` | Emits stop command when idle/stale. |
| `bridge.api_id_move` | `1008` | Unitree Sport API Move request id. |
| `bridge.api_id_stop` | `1003` | Unitree Sport API StopMove request id. |

Do not tune only one of MPPI, velocity smoother, and bridge caps. Treat them as
one chain: MPPI plans commands, the smoother reshapes them, and the bridge clamps
the final output.

## Narrow Corridor Tuning Order

Use this order to avoid tuning the wrong layer:

1. Inspect `/scan`. If corridor walls are wrong or floor/body noise appears,
   tune `pointcloud_to_laserscan` height and range limits.
2. Inspect `/map`. If the corridor is closed or unstable there, tune scan
   projection and `slam_toolbox` mapping parameters before Nav2.
3. Inspect `/global_costmap/costmap`. If the corridor is open in `/map` but
   closed in the global costmap, tune `robot_radius`, `inflation_radius`, and
   `cost_scaling_factor`.
4. Inspect `/plan`. If no global path enters the corridor, tune global costmap
   and NavFn `tolerance` or `allow_unknown`.
5. Inspect `/local_costmap/costmap` and `/trajectories`. If the global plan is
   good but MPPI refuses the corridor, tune local inflation, `CostCritic`, path
   critics, and MPPI sampling noise.
6. Inspect `/cmd_vel_nav` and `/cmd_vel`. If commands are reasonable before the
   smoother or bridge but not after, tune the velocity smoother and bridge caps.

Only change one small group of parameters per test run, and record the run with
`record_results:=true` so the result can be replayed side by side with the
previous configuration.
