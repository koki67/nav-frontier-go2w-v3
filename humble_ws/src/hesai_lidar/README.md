# go2w-hesai-lidar-driver

ROS 2 driver for Hesai LiDAR sensors (PandarXT-16, PandarXT-32, PandarQT, Pandar64, Pandar40P, Pandar40M, Pandar20A, Pandar20B, PandarXTM), ported from [HesaiLidar_General_ROS](https://github.com/HesaiTechnology/HesaiLidar_General_ROS) for use on the Unitree GO2W robot.

## Background

The original Hesai driver (`HesaiLidar_General_ROS`) is ROS 1 only. This repository ports it to ROS 2, building the `HesaiLidar_General_SDK` from source for ARM (aarch64) targets such as the Jetson Orin NX.

## Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/hesai_node/points_raw` | `sensor_msgs/msg/PointCloud2` | Publish | LiDAR point cloud |
| `/hesai_node/pandar_packets` | `hesai_lidar/msg/PandarScan` | Publish | Raw UDP packets |
| `/node_control` | `std_msgs/msg/String` | Subscribe | Send `"hesailidar_stop"` or `"all_stop"` to shutdown |

## Dependencies

- ROS 2 (Foxy or later)
- `libpcap-dev`, `libpcl-dev`, `libboost-dev`

```bash
sudo apt install libpcap-dev libpcl-dev libboost-dev
```

## Build

```bash
cd <your_colcon_workspace>/src
git clone https://github.com/koki67/go2w-hesai-lidar-driver.git
cd ..
colcon build --packages-select hesai_lidar
```

## Configuration

Edit the launch file parameters or override them at launch time:

| Parameter | Default | Description |
|---|---|---|
| `server_ip` | `192.168.123.20` | LiDAR IP address |
| `lidar_recv_port` | `2368` | UDP data port |
| `gps_port` | `10110` | GPS data port |
| `lidar_type` | `PandarXT-16` | Sensor model |
| `frame_id` | `hesai_lidar` | TF frame ID |
| `publish_type` | `both` | `"points"`, `"raw"`, or `"both"` |
| `timestamp_type` | `realtime` | `""` for LiDAR time, `"realtime"` for system time |
| `pcap_file` | `""` | Path to pcap file (empty = live sensor) |
| `data_type` | `""` | `""` for live/pcap, `"rosbag"` for bag playback |
| `lidar_correction_file` | auto | Path to correction CSV |
| `coordinate_correction_flag` | `false` | Enable coordinate correction |

## Run

```bash
source install/setup.bash
ros2 launch hesai_lidar hesai_lidar_launch.py
```

## License

[Apache 2.0](LICENSE) - Original driver copyright Hesai Technology.
