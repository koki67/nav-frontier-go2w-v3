"""ROS 2 node that selects an exploration goal from an OccupancyGrid.

Subscribes /map, looks up the robot pose via TF (global_frame → robot_base_frame),
runs the BFS frontier pipeline, picks the best cluster by `info_gain - lambda *
travel_cost`, and publishes the goal as a PoseStamped on /frontier_goal. Also
publishes a MarkerArray for RViz visualization.
"""
from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from nav_frontier_go2w_frontier.frontier_core import (
    FrontierCluster,
    FrontierSearchResult,
    GridSpec,
    RobotPose2D,
    analyze_frontiers,
    cell_center,
)


class FrontierSelectorNode(Node):
    """Pick the best reachable frontier cluster and publish its goal pose."""

    def __init__(self) -> None:
        super().__init__("frontier_selector")

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("goal_topic", "/frontier_goal")
        self.declare_parameter("markers_topic", "/frontier_markers")
        self.declare_parameter("update_rate", 2.0)
        self.declare_parameter("connectivity", 8)
        self.declare_parameter("min_cluster_size", 8)
        self.declare_parameter("score_lambda", 0.5)
        self.declare_parameter("info_radius_cells", 2)
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("tf_timeout", 0.2)

        self._map_topic = self.get_parameter("map_topic").value
        self._global_frame = self.get_parameter("global_frame").value
        self._robot_base_frame = self.get_parameter("robot_base_frame").value
        self._goal_topic = self.get_parameter("goal_topic").value
        self._markers_topic = self.get_parameter("markers_topic").value
        self._update_rate = max(float(self.get_parameter("update_rate").value), 0.1)
        self._connectivity = int(self.get_parameter("connectivity").value)
        self._min_cluster_size = max(int(self.get_parameter("min_cluster_size").value), 1)
        self._score_lambda = float(self.get_parameter("score_lambda").value)
        self._info_radius_cells = max(int(self.get_parameter("info_radius_cells").value), 0)
        self._publish_markers = bool(self.get_parameter("publish_markers").value)
        self._tf_timeout = float(self.get_parameter("tf_timeout").value)

        if self._connectivity not in (4, 8):
            self.get_logger().warning(
                "Unsupported connectivity %d; defaulting to 8." % self._connectivity,
            )
            self._connectivity = 8

        self._latest_map: Optional[OccupancyGrid] = None
        self._last_published_goal: Optional[tuple[float, float]] = None
        self._throttle: dict[str, float] = {}

        self._tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = ReliabilityPolicy.RELIABLE

        self._goal_pub = self.create_publisher(PoseStamped, self._goal_topic, 10)
        self._marker_pub = (
            self.create_publisher(MarkerArray, self._markers_topic, 10)
            if self._publish_markers else None
        )
        self._map_sub = self.create_subscription(
            OccupancyGrid, self._map_topic, self._on_map, map_qos,
        )
        self._timer = self.create_timer(1.0 / self._update_rate, self._on_timer)

        self.get_logger().info(
            (
                "Frontier selector ready: map=%s goal=%s frame=%s base=%s "
                "connectivity=%d min_cluster=%d lambda=%.3f info_radius=%d"
            )
            % (
                self._map_topic, self._goal_topic, self._global_frame, self._robot_base_frame,
                self._connectivity, self._min_cluster_size, self._score_lambda,
                self._info_radius_cells,
            )
        )

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._latest_map = msg

    def _on_timer(self) -> None:
        if self._latest_map is None:
            self._throttled_warn("no_map", "Waiting for OccupancyGrid on %s." % self._map_topic, 5.0)
            self._publish_marker_clear()
            return

        map_msg = self._latest_map
        frame_id = map_msg.header.frame_id.strip() or self._global_frame

        robot_pose = self._lookup_robot_pose(frame_id)
        if robot_pose is None:
            self._publish_marker_clear()
            return

        grid = GridSpec(
            width=map_msg.info.width,
            height=map_msg.info.height,
            resolution=map_msg.info.resolution,
            origin_x=map_msg.info.origin.position.x,
            origin_y=map_msg.info.origin.position.y,
            origin_yaw=_yaw_from_quaternion(map_msg.info.origin.orientation),
        )

        try:
            result = analyze_frontiers(
                map_data=list(map_msg.data),
                grid=grid,
                robot_pose=robot_pose,
                connectivity=self._connectivity,
                min_cluster_size=self._min_cluster_size,
                score_lambda=self._score_lambda,
                info_radius_cells=self._info_radius_cells,
            )
        except ValueError as exc:
            self._throttled_warn("bad_map", "Skipping map: %s" % exc, 5.0)
            self._publish_marker_clear()
            return

        self._publish_markers_msg(result, grid, frame_id)

        if result.robot_seed_cell is None:
            self._throttled_warn(
                "no_seed",
                "Robot is not on free space and no nearby free seed cell was found.",
                5.0,
            )
            return
        if result.selected_cluster is None:
            self._throttled_warn("no_frontier", "No reachable frontier clusters found.", 5.0)
            return

        goal = self._build_goal_pose(result.selected_cluster, frame_id, robot_pose)
        self._goal_pub.publish(goal)

        key = (round(goal.pose.position.x, 3), round(goal.pose.position.y, 3))
        if key != self._last_published_goal:
            self._last_published_goal = key
            sel = result.selected_cluster
            self.get_logger().info(
                "Goal: x=%.2f y=%.2f info_gain=%d travel=%.2fm score=%.2f (clusters=%d)"
                % (
                    goal.pose.position.x, goal.pose.position.y,
                    sel.info_gain, sel.travel_cost or 0.0, sel.score or 0.0,
                    len(result.clusters),
                )
            )

    def _lookup_robot_pose(self, frame_id: str) -> Optional[RobotPose2D]:
        try:
            tx = self._tf_buffer.lookup_transform(
                frame_id, self._robot_base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self._tf_timeout),
            )
        except TransformException as exc:
            self._throttled_warn(
                "tf_unavailable",
                "TF %s -> %s unavailable: %s" % (frame_id, self._robot_base_frame, exc),
                5.0,
            )
            return None
        return RobotPose2D(x=tx.transform.translation.x, y=tx.transform.translation.y)

    def _build_goal_pose(
        self, cluster: FrontierCluster, frame_id: str, robot_pose: RobotPose2D,
    ) -> PoseStamped:
        assert cluster.goal_x is not None and cluster.goal_y is not None
        goal = PoseStamped()
        goal.header.frame_id = frame_id
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = cluster.goal_x
        goal.pose.position.y = cluster.goal_y
        goal.pose.position.z = 0.0
        # Face the goal from the robot's current heading vector.
        yaw = math.atan2(cluster.goal_y - robot_pose.y, cluster.goal_x - robot_pose.x)
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        return goal

    # --- Visualization helpers ----------------------------------------------

    def _publish_markers_msg(
        self, result: FrontierSearchResult, grid: GridSpec, frame_id: str,
    ) -> None:
        if self._marker_pub is None:
            return
        markers = MarkerArray()
        markers.markers.append(_delete_all())

        if result.frontier_cells:
            markers.markers.append(_points_marker(
                frame_id=frame_id,
                stamp=self.get_clock().now().to_msg(),
                ns="frontiers", marker_id=0,
                cells=result.frontier_cells, grid=grid,
                rgba=(0.95, 0.78, 0.10, 0.90),
                scale_factor=0.7, scale_floor=0.05,
            ))
        if result.selected_cluster is not None:
            sel = result.selected_cluster
            markers.markers.append(_points_marker(
                frame_id=frame_id,
                stamp=self.get_clock().now().to_msg(),
                ns="selected", marker_id=1,
                cells=sel.cells, grid=grid,
                rgba=(0.15, 0.90, 0.30, 0.95),
                scale_factor=1.0, scale_floor=0.08,
            ))
            if sel.goal_x is not None and sel.goal_y is not None:
                markers.markers.append(_sphere_marker(
                    frame_id=frame_id,
                    stamp=self.get_clock().now().to_msg(),
                    ns="goal", marker_id=2,
                    x=sel.goal_x, y=sel.goal_y,
                    rgba=(0.90, 0.15, 0.20, 0.95),
                    diameter=max(grid.resolution * 2.0, 0.18),
                ))
        self._marker_pub.publish(markers)

    def _publish_marker_clear(self) -> None:
        if self._marker_pub is None:
            return
        markers = MarkerArray()
        markers.markers.append(_delete_all())
        self._marker_pub.publish(markers)

    def _throttled_warn(self, key: str, message: str, period_s: float) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        last = self._throttle.get(key, 0.0)
        if (now - last) >= period_s:
            self._throttle[key] = now
            self.get_logger().warning(message)


# --- Marker constructors (free functions for testability) -------------------


def _delete_all() -> Marker:
    m = Marker()
    m.action = Marker.DELETEALL
    return m


def _points_marker(
    *, frame_id: str, stamp, ns: str, marker_id: int,
    cells: tuple, grid: GridSpec,
    rgba: tuple[float, float, float, float],
    scale_factor: float, scale_floor: float,
) -> Marker:
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = stamp
    m.ns = ns
    m.id = marker_id
    m.type = Marker.POINTS
    m.action = Marker.ADD
    s = max(grid.resolution * scale_factor, scale_floor)
    m.scale.x = s
    m.scale.y = s
    m.color.r, m.color.g, m.color.b, m.color.a = rgba
    m.points = [_point_from_cell(grid, c) for c in cells]
    return m


def _sphere_marker(
    *, frame_id: str, stamp, ns: str, marker_id: int,
    x: float, y: float,
    rgba: tuple[float, float, float, float],
    diameter: float,
) -> Marker:
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = stamp
    m.ns = ns
    m.id = marker_id
    m.type = Marker.SPHERE
    m.action = Marker.ADD
    m.pose.position.x = x
    m.pose.position.y = y
    m.pose.position.z = 0.0
    m.pose.orientation.w = 1.0
    m.scale.x = diameter
    m.scale.y = diameter
    m.scale.z = max(diameter * 0.4, 0.05)
    m.color.r, m.color.g, m.color.b, m.color.a = rgba
    return m


def _point_from_cell(grid: GridSpec, cell: tuple[int, int]) -> Point:
    x, y = cell_center(grid, cell)
    return Point(x=x, y=y, z=0.0)


def _yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrontierSelectorNode()
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
