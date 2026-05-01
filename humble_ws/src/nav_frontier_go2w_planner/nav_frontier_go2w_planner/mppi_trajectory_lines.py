"""Normalize Nav2 MPPI trajectory markers into thin line-strip markers.

Nav2 MPPI publishes sampled rollout visualization as a MarkerArray on
`/trajectories`. This helper keeps that data visual-only, but republishes it in
a consistent line style that is easier to inspect in RViz and replay bags.
"""
from __future__ import annotations

from copy import deepcopy

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class MppiTrajectoryLinesNode(Node):
    """Convert MPPI trajectory markers to line markers for RViz."""

    def __init__(self) -> None:
        super().__init__("mppi_trajectory_lines")

        self.declare_parameter("input_topic", "/trajectories")
        self.declare_parameter("output_topic", "/mppi_trajectory_lines")
        self.declare_parameter("line_width", 0.015)
        self.declare_parameter("alpha", 0.35)
        self.declare_parameter("color_rgb", [80.0, 180.0, 255.0])
        self.declare_parameter("z_offset", 0.04)
        self.declare_parameter("max_markers", 400)
        self.declare_parameter("point_stride", 1)
        self.declare_parameter("preserve_marker_color", False)

        self._input_topic = str(self.get_parameter("input_topic").value)
        self._output_topic = str(self.get_parameter("output_topic").value)
        self._line_width = max(float(self.get_parameter("line_width").value), 0.001)
        self._alpha = _clamp(float(self.get_parameter("alpha").value), 0.0, 1.0)
        self._color_rgb = _rgb_param(self.get_parameter("color_rgb").value)
        self._z_offset = float(self.get_parameter("z_offset").value)
        self._max_markers = max(int(self.get_parameter("max_markers").value), 0)
        self._point_stride = max(int(self.get_parameter("point_stride").value), 1)
        self._preserve_marker_color = bool(self.get_parameter("preserve_marker_color").value)

        self._pub = self.create_publisher(MarkerArray, self._output_topic, 10)
        self._sub = self.create_subscription(
            MarkerArray, self._input_topic, self._on_markers, 10,
        )

        self.get_logger().info(
            "MPPI trajectory line visualizer ready: %s -> %s width=%.3f alpha=%.2f"
            % (self._input_topic, self._output_topic, self._line_width, self._alpha)
        )

    def _on_markers(self, msg: MarkerArray) -> None:
        output = MarkerArray()
        output.markers.append(_delete_all())

        count = 0
        for marker in msg.markers:
            if self._max_markers and count >= self._max_markers:
                break
            line_marker = self._to_line_marker(marker, count)
            if line_marker is None:
                continue
            output.markers.append(line_marker)
            count += 1

        self._pub.publish(output)

    def _to_line_marker(self, marker: Marker, marker_id: int) -> Marker | None:
        if marker.action in (Marker.DELETE, Marker.DELETEALL):
            return None
        if len(marker.points) < 2:
            return None

        out = Marker()
        out.header = marker.header
        out.ns = "mppi_trajectory_lines"
        out.id = marker_id
        out.action = Marker.ADD
        out.pose = deepcopy(marker.pose)
        out.pose.position.z += self._z_offset
        out.type = Marker.LINE_LIST if marker.type == Marker.LINE_LIST else Marker.LINE_STRIP
        out.scale.x = self._line_width
        out.points = list(marker.points)[::self._point_stride]
        out.lifetime = marker.lifetime
        out.frame_locked = marker.frame_locked

        if len(out.points) < 2:
            return None

        if self._preserve_marker_color and marker.color.a > 0.0:
            out.color = marker.color
            out.color.a = min(marker.color.a, self._alpha)
        else:
            out.color.r = self._color_rgb[0]
            out.color.g = self._color_rgb[1]
            out.color.b = self._color_rgb[2]
            out.color.a = self._alpha

        return out


def _delete_all() -> Marker:
    marker = Marker()
    marker.action = Marker.DELETEALL
    return marker


def _rgb_param(value) -> tuple[float, float, float]:
    vals = list(value)
    if len(vals) != 3:
        return (80.0 / 255.0, 180.0 / 255.0, 255.0 / 255.0)
    return tuple(_normalize_color(float(v)) for v in vals)


def _normalize_color(value: float) -> float:
    if value > 1.0:
        value = value / 255.0
    return _clamp(value, 0.0, 1.0)


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MppiTrajectoryLinesNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
