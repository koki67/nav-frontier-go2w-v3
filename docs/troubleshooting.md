# Troubleshooting

Common bring-up issues and how to diagnose them.

## "robot stands but doesn't move" decision tree

1. Is `/cmd_vel` publishing? `ros2 topic hz /cmd_vel`
   - **No:** Nav2 isn't producing commands. Skip to step 2.
   - **Yes, but values are zero:** MPPI thinks it's at the goal or the goal_checker is satisfied. Check `ros2 topic echo /frontier_goal` — is a goal even being requested?
2. Is `/frontier_goal` publishing? `ros2 topic echo /frontier_goal --once`
   - **No:** the frontier selector has no map or no robot pose. `ros2 topic echo /map --once` to verify slam_toolbox is producing one. `ros2 run tf2_ros tf2_echo map base_link` to verify the TF chain.
   - **Yes:** the executor isn't dispatching it. Check the executor logs for `Sent goal to Nav2` vs `TF unavailable` vs `Nav2 not active`.
3. Is the velocity bridge publishing `/api/sport/request`? `ros2 topic hz /api/sport/request`
   - **No:** the bridge node didn't start. `ros2 node list | grep velocity_bridge`.
   - **Yes but nothing happens:** check `dry_run` — if `true`, requests are logged only.
4. Is `/api/sport/request` actually reaching the Sport API? On the robot, the unitree DDS bridge has to be running. Check with the unitree SDK tooling that the Sport API is enabled.

## Slam_toolbox isn't producing /map

- `ros2 topic hz /scan` — pointcloud_to_laserscan needs `/points_raw` and the `base_link → hesai_lidar` static TF.
- `ros2 run tf2_tools view_frames` — confirm `map → odom → base_link → hesai_lidar` are all present.
- `ros2 param get /slam_toolbox base_frame` — if it doesn't match what D-LIO publishes for `base_link`, the SLAM optimization will silently fail.

## D-LIO drift / map twisted

- Check the IMU calibration period (D-LIO needs a still period at startup to estimate biases). If you start moving immediately, expect drift.
- Verify `extrinsics/baselink2lidar` and `extrinsics/baselink2imu` in `direct_lidar_inertial_odometry/cfg/dlio.yaml` match the static TFs in the bringup. They should describe the same transform but expressed differently (matrix vs quaternion).

## CycloneDDS / robot can't see desktop topics

`/etc/cyclonedds.xml` (baked into the image) defaults to `eth0`. On a desktop the interface is usually `wlan0`. Override with `CYCLONEDDS_NETWORK_INTERFACE=wlan0` before launching the container, or edit `config/cyclonedds.xml` and rebuild the image.

## MPPI plan thrashes / robot oscillates

- Lower `controller_server.FollowPath.batch_size` (default 500) for less noise but coarser sampling.
- Raise `PathFollowCritic.cost_weight` and `PathAlignCritic.cost_weight` if MPPI is wandering off the global path.
- Tighten `goal_checker.xy_goal_tolerance` (default 0.30 m) if the robot stops short of the goal.

## "Nav2 is not active yet"

The frontier_goal_executor logs this when bt_navigator hasn't reached the `active` lifecycle state. Cause: the lifecycle manager didn't `autostart`, or one of the Nav2 nodes failed during `configure`. Check `ros2 lifecycle nodes` and look at the failing node's stderr.

## RViz can't see /points_raw

- Ensure the fixed frame in RViz matches an existing TF frame (`map`, `odom`, or `base_link`). If you set `hesai_lidar` and the static TF isn't published, RViz silently drops the cloud.
- Check `ros2 topic info /points_raw -v` for QoS mismatches with RViz's subscriber QoS.
