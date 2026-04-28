# Troubleshooting

Common bring-up issues and how to diagnose them.

## "robot stands but doesn't move" decision tree

1. Is `/cmd_vel` publishing? `ros2 topic hz /cmd_vel`
   - **No:** Nav2 isn't producing commands. Check `ros2 topic hz /cmd_vel_nav`; if that is active, the velocity smoother is the issue. If both are silent, skip to step 2.
   - **Yes, but values are zero:** MPPI thinks it's at the goal or the goal_checker is satisfied. Check `ros2 topic echo /frontier_goal` — is a goal even being requested?
2. Is `/frontier_goal` publishing? `ros2 topic echo /frontier_goal --once`
   - **No:** the frontier selector has no map or no robot pose. `ros2 topic echo /map --once` to verify slam_toolbox is producing one. `ros2 run tf2_ros tf2_echo map base_link` to verify the TF chain.
   - **Yes:** the executor isn't dispatching it, Nav2 is inactive, or the controller is failing. Check the executor logs for `Sent goal to Nav2` vs `TF unavailable` vs `Nav2 not active`, then check lifecycle state with `ros2 lifecycle get /controller_server`, `/planner_server`, `/bt_navigator`, and `/velocity_smoother`.
3. Is the velocity bridge publishing `/api/sport/request`? `ros2 topic hz /api/sport/request`
   - **No:** the bridge node didn't start. `ros2 node list | grep velocity_bridge`.
   - **Yes but nothing happens:** check `dry_run` — if `true`, requests are logged only.
4. Is `/api/sport/request` actually reaching the Sport API? On the robot, the unitree DDS bridge has to be running. Check with the unitree SDK tooling that the Sport API is enabled.

## Slam_toolbox isn't producing /map

- `ros2 topic hz /scan` — pointcloud_to_laserscan needs `/points_raw` and the `base_link → hesai_lidar` static TF.
- `ros2 topic hz /go2w/imu` — D-LIO needs IMU messages before it can maintain `odom → base_link`. If this is silent, check `ros2 topic hz /lowstate`; the IMU publisher only republishes the Unitree low-state stream.
- `ros2 run tf2_tools view_frames` — confirm `map → odom → base_link → hesai_lidar` are all present.
- `ros2 param get /slam_toolbox base_frame` — if it doesn't match what D-LIO publishes for `base_link`, the SLAM optimization will silently fail.

## D-LIO drift / map twisted

- Check the IMU calibration period (D-LIO needs a still period at startup to estimate biases). If you start moving immediately, expect drift.
- Verify `extrinsics/baselink2lidar` and `extrinsics/baselink2imu` in `direct_lidar_inertial_odometry/cfg/dlio.yaml` match the static TFs in the bringup. They should describe the same transform but expressed differently (matrix vs quaternion).

## Docker container can't see robot topics

If `ros2 topic list` on the robot host shows `/lowstate`, `/api/sport/request`,
and other Unitree topics, but the same command inside the container only shows
`/rosout` and `/parameter_events`, the container is on a different DDS graph.
Check the ROS domain first:

```bash
# On the robot host:
echo "${ROS_DOMAIN_ID:-0}"

# Inside the container:
echo "${ROS_DOMAIN_ID:-0}"
```

Both values must match. `docker/run.sh` forwards the host `ROS_DOMAIN_ID` and
defaults to `0`, which is the stock Unitree onboard domain. If the robot host
uses a custom value, export it before starting the container:

```bash
export ROS_DOMAIN_ID=<robot-domain-id>
bash docker/run.sh
```

After entering the container, verify discovery before launching the stack:

```bash
ros2 topic list | grep -E '^/lowstate$|^/api/sport/request$'
```

## CycloneDDS interface selection

`/etc/cyclonedds.xml` (baked into the image) defaults to `eth0`. On the Go2W
onboard computer this is usually correct. On a desktop the interface is often
`wlan0` or `en*`; edit `config/cyclonedds.xml` and rebuild the image, or start
the container with `CYCLONEDDS_URI` pointing at a custom CycloneDDS profile.

## MPPI plan thrashes / robot oscillates

- Lower `controller_server.FollowPath.batch_size` (default 500) for less noise but coarser sampling.
- Raise `PathFollowCritic.cost_weight` and `PathAlignCritic.cost_weight` if MPPI is wandering off the global path.
- Tighten `goal_checker.xy_goal_tolerance` (default 0.30 m) if the robot stops short of the goal.

## "Nav2 is not active yet"

The frontier_goal_executor logs this when bt_navigator hasn't reached the `active` lifecycle state. Cause: the lifecycle manager didn't `autostart`, or one of the Nav2 nodes failed during `configure`. Check `ros2 lifecycle nodes` and look at the failing node's stderr.

If the controller configures as `dwb_core::DWBLocalPlanner`, the wrong Nav2
parameter file was loaded. This stack should log
`nav2_mppi_controller::MPPIController` for `FollowPath`.

## RViz can't see /points_raw

- Ensure the fixed frame in RViz matches an existing TF frame (`map`, `odom`, or `base_link`). If you set `hesai_lidar` and the static TF isn't published, RViz silently drops the cloud.
- Check `ros2 topic info /points_raw -v` for QoS mismatches with RViz's subscriber QoS.
