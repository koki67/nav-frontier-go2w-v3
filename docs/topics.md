# Topics, TF, services, actions

## Topics

| Topic                          | Type                              | Direction | Owning node                       | Notes |
|--------------------------------|-----------------------------------|-----------|-----------------------------------|-------|
| `/points_raw`                  | `sensor_msgs/PointCloud2`         | pub       | `hesai_node` (hesai_lidar)        | ~10 Hz, frame `hesai_lidar`. |
| `/lowstate`                    | `unitree_go/LowState`             | sub       | `go2w_imu_publisher`              | From the unitree DDS bridge. |
| `/go2w/imu`                    | `sensor_msgs/Imu`                 | pub       | `go2w_imu_publisher`              | ~500 Hz, frame `imu_link`. |
| `/dlio/odom_node/odom`         | `nav_msgs/Odometry`               | pub       | `dlio_odom_node`                  | Consumed by Nav2 (`bt_navigator.odom_topic`). |
| `/dlio/odom_node/path`         | `nav_msgs/Path`                   | pub       | `dlio_odom_node`                  | Visualization only. |
| `/dlio/odom_node/keyframes`    | `geometry_msgs/PoseArray`         | pub       | `dlio_odom_node`                  | Visualization only. |
| `/dlio/map_node/map`           | `sensor_msgs/PointCloud2`         | pub       | `dlio_map_node`                   | Renamed from `map` so it doesn't clash with slam_toolbox. |
| `/scan`                        | `sensor_msgs/LaserScan`           | pub       | `pointcloud_to_laserscan`         | 2D projection of `/points_raw`. |
| `/map`                         | `nav_msgs/OccupancyGrid`          | pub       | `slam_toolbox`                    | TRANSIENT_LOCAL durability. |
| `/frontier_goal`               | `geometry_msgs/PoseStamped`       | pub       | `frontier_selector`               | One pose per timer tick. Frame: `map`. |
| `/frontier_markers`            | `visualization_msgs/MarkerArray`  | pub       | `frontier_selector`               | RViz overlays. |
| `/global_costmap/costmap`      | `nav_msgs/OccupancyGrid`          | pub       | `planner_server`                  | Global planning costmap for RViz Map display. |
| `/global_costmap/costmap_raw`  | `nav2_msgs/Costmap`               | pub       | `planner_server`                  | Nav2 raw global costmap. |
| `/local_costmap/costmap`       | `nav_msgs/OccupancyGrid`          | pub       | `controller_server`               | Local obstacle/inflation costmap for RViz Map display. |
| `/local_costmap/costmap_raw`   | `nav2_msgs/Costmap`               | pub       | `controller_server`               | Nav2 raw local costmap. |
| `/cmd_vel_nav`                 | `geometry_msgs/Twist`             | pub       | `controller_server` (MPPI)        | Pre-smoothing output. |
| `/cmd_vel`                     | `geometry_msgs/Twist`             | pub       | `velocity_smoother`               | What the velocity bridge consumes. |
| `/api/sport/request`           | `unitree_api/Request`             | pub       | `velocity_bridge`                 | Final motion command. Move=1008, StopMove=1003. |
| `/api/sport/response`          | `unitree_api/Response`            | sub       | (optional)                        | Sport API ack stream. |

## TF

| Parent       | Child         | Source                    | Type    |
|--------------|---------------|---------------------------|---------|
| `map`        | `odom`        | `slam_toolbox`            | dynamic |
| `odom`       | `base_link`   | `direct_lidar_inertial_odometry` | dynamic |
| `base_link`  | `hesai_lidar` | bringup `static_transform_publisher` | static  |
| `base_link`  | `imu_link`    | bringup `static_transform_publisher` | static  |

## Actions

| Action server          | Type                       | Used by                      |
|------------------------|----------------------------|------------------------------|
| `/navigate_to_pose`    | `nav2_msgs/NavigateToPose` | `frontier_goal_executor`     |

## Lifecycle gate

Nav2 nodes are managed by `nav2_lifecycle_manager_navigation`:
`controller_server`, `planner_server`, `behavior_server`, `bt_navigator`,
`smoother_server`, `velocity_smoother`. The frontier_goal_executor waits
for the bt_navigator to enter the `active` state via `BasicNavigator`
internals before sending its first goal.
