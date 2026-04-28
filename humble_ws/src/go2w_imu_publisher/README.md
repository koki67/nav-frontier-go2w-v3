# go2w-imu-publisher

A ROS 2 node that republishes IMU data from the Unitree GO2W's `/lowstate` topic as a standard `sensor_msgs/Imu` message.

## Background

The Unitree GO2 (non-wheeled) publishes IMU data on the `/sportmodestate` topic, but the **GO2W (wheeled variant)** does not populate IMU data there. On the GO2W, IMU data is only available through the `/lowstate` topic. This package bridges that gap by extracting the IMU fields from `/lowstate` and republishing them as a standard ROS 2 IMU message.

## Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/lowstate` | `unitree_go/msg/LowState` | Subscribe | Raw robot state from GO2W |
| `/go2w/imu` | `sensor_msgs/msg/Imu` | Publish | Standard IMU message (orientation, angular velocity, linear acceleration) |

## Dependencies

- ROS 2 (Humble or later)
- [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2) (provides `unitree_go` messages)

## Build

```bash
cd <your_colcon_workspace>/src
git clone https://github.com/koki67/go2w-imu-publisher.git
cd ..
source /path/to/unitree_ros2/setup.bash
colcon build --packages-select go2w_imu_publisher
```

## Run

```bash
source install/setup.bash
ros2 run go2w_imu_publisher imu_publisher
```

## License

[MIT](LICENSE)
