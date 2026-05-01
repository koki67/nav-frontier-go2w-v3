"""Publish RViz-friendly known/unknown map layers derived from /map."""
from __future__ import annotations

import math
from collections.abc import Iterable

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


class MapVizLayersNode(Node):
    """Split an OccupancyGrid into independently styled RViz marker layers."""

    def __init__(self) -> None:
        super().__init__("map_viz_layers")

        self.declare_parameter("input_topic", "/map")
        self.declare_parameter("known_topic", "/map_known_cells")
        self.declare_parameter("unknown_topic", "/map_unknown_cells")
        self.declare_parameter("occupied_threshold", 50)
        self.declare_parameter("publish_free_cells", True)
        self.declare_parameter("cell_stride", 1)
        self.declare_parameter("cell_z", -0.02)
        self.declare_parameter("cell_thickness", 0.01)
        self.declare_parameter("known_free_rgba", [0.92, 0.92, 0.92, 0.10])
        self.declare_parameter("known_occupied_rgba", [0.02, 0.02, 0.02, 0.95])
        self.declare_parameter("unknown_rgba", [0.45, 0.45, 0.45, 0.18])

        self._input_topic = str(self.get_parameter("input_topic").value)
        self._known_topic = str(self.get_parameter("known_topic").value)
        self._unknown_topic = str(self.get_parameter("unknown_topic").value)
        self._occupied_threshold = int(self.get_parameter("occupied_threshold").value)
        self._publish_free_cells = bool(self.get_parameter("publish_free_cells").value)
        self._cell_stride = max(1, int(self.get_parameter("cell_stride").value))
        self._cell_z = float(self.get_parameter("cell_z").value)
        self._cell_thickness = max(0.001, float(self.get_parameter("cell_thickness").value))
        self._known_free_rgba = self._rgba_parameter("known_free_rgba")
        self._known_occupied_rgba = self._rgba_parameter("known_occupied_rgba")
        self._unknown_rgba = self._rgba_parameter("unknown_rgba")

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        marker_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._known_pub = self.create_publisher(MarkerArray, self._known_topic, marker_qos)
        self._unknown_pub = self.create_publisher(MarkerArray, self._unknown_topic, marker_qos)
        self._sub = self.create_subscription(OccupancyGrid, self._input_topic, self._on_map, map_qos)

        self.get_logger().info(
            "Map viz layers ready: input=%s known=%s unknown=%s stride=%d"
            % (self._input_topic, self._known_topic, self._unknown_topic, self._cell_stride)
        )

    def _rgba_parameter(self, name: str) -> tuple[float, float, float, float]:
        value = self.get_parameter(name).value
        if not isinstance(value, Iterable):
            self.get_logger().warning("Parameter %s must be an RGBA list; using white." % name)
            return (1.0, 1.0, 1.0, 1.0)

        values = [float(v) for v in value]
        if len(values) != 4:
            self.get_logger().warning("Parameter %s must contain 4 values; using white." % name)
            return (1.0, 1.0, 1.0, 1.0)
        return tuple(max(0.0, min(1.0, v)) for v in values)  # type: ignore[return-value]

    def _on_map(self, msg: OccupancyGrid) -> None:
        width = int(msg.info.width)
        height = int(msg.info.height)
        resolution = float(msg.info.resolution)
        if width <= 0 or height <= 0 or resolution <= 0.0:
            self.get_logger().warning("Skipping invalid occupancy grid dimensions.")
            return

        free_points: list[Point] = []
        occupied_points: list[Point] = []
        unknown_points: list[Point] = []

        origin = msg.info.origin
        yaw = _yaw_from_quaternion(
            origin.orientation.x,
            origin.orientation.y,
            origin.orientation.z,
            origin.orientation.w,
        )
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        for y in range(0, height, self._cell_stride):
            row_offset = y * width
            local_y = (y + 0.5) * resolution
            for x in range(0, width, self._cell_stride):
                value = int(msg.data[row_offset + x])
                point = _cell_center(
                    x=x,
                    local_y=local_y,
                    resolution=resolution,
                    origin_x=origin.position.x,
                    origin_y=origin.position.y,
                    z=self._cell_z,
                    cos_yaw=cos_yaw,
                    sin_yaw=sin_yaw,
                )
                if value < 0:
                    unknown_points.append(point)
                elif value >= self._occupied_threshold:
                    occupied_points.append(point)
                elif self._publish_free_cells:
                    free_points.append(point)

        scale_xy = resolution * self._cell_stride
        self._known_pub.publish(MarkerArray(markers=[
            self._cube_list_marker(msg, "known_free", 0, free_points, self._known_free_rgba, scale_xy),
            self._cube_list_marker(msg, "known_occupied", 1, occupied_points, self._known_occupied_rgba, scale_xy),
        ]))
        self._unknown_pub.publish(MarkerArray(markers=[
            self._cube_list_marker(msg, "unknown", 0, unknown_points, self._unknown_rgba, scale_xy),
        ]))

    def _cube_list_marker(
        self,
        msg: OccupancyGrid,
        namespace: str,
        marker_id: int,
        points: list[Point],
        rgba: tuple[float, float, float, float],
        scale_xy: float,
    ) -> Marker:
        marker = Marker()
        marker.header = msg.header
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale_xy
        marker.scale.y = scale_xy
        marker.scale.z = self._cell_thickness
        marker.color.r = rgba[0]
        marker.color.g = rgba[1]
        marker.color.b = rgba[2]
        marker.color.a = rgba[3]
        marker.points = points
        return marker


def _cell_center(
    *,
    x: int,
    local_y: float,
    resolution: float,
    origin_x: float,
    origin_y: float,
    z: float,
    cos_yaw: float,
    sin_yaw: float,
) -> Point:
    local_x = (x + 0.5) * resolution
    point = Point()
    point.x = origin_x + (cos_yaw * local_x) - (sin_yaw * local_y)
    point.y = origin_y + (sin_yaw * local_x) + (cos_yaw * local_y)
    point.z = z
    return point


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * ((w * z) + (x * y))
    cosy_cosp = 1.0 - (2.0 * ((y * y) + (z * z)))
    return math.atan2(siny_cosp, cosy_cosp)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapVizLayersNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == "__main__":
    main()
