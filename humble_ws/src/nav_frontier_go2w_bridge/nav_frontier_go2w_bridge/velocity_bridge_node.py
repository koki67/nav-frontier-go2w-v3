"""Twist → Sport API velocity bridge for the Unitree Go2W.

Subscribes to /cmd_vel (Twist) and republishes the latest velocity command on
/api/sport/request as a unitree_api/Request encoded for the Sport API:

  Move:  api_id = 1008, parameter = '{"x": vx, "y": vy, "z": wz}'
  Stop:  api_id = 1003, parameter = ''

A watchdog tracks the freshness of /cmd_vel using the node's clock (so
use_sim_time during bag replay works correctly). When the upstream stops
publishing for `watchdog_timeout` seconds the bridge emits a one-shot
StopMove and then streams zero-Move at the publish rate to keep the Go2W
locomotion controller standing.

The dry_run parameter logs Move/Stop intent without touching /api/sport/request,
which is the recommended way to bring the stack up before enabling motion.
"""
from __future__ import annotations

import threading

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.time import Time
from unitree_api.msg import Request

from nav_frontier_go2w_bridge.clamping import clamp_twist, encode_move_parameter


class VelocityBridgeNode(Node):
    """Continuous Twist → Sport-API velocity bridge with watchdog and dry-run."""

    def __init__(self) -> None:
        super().__init__("velocity_bridge")

        self.declare_parameter("vx_max", 0.30)
        self.declare_parameter("vy_max", 0.20)
        self.declare_parameter("wz_max", 0.50)
        self.declare_parameter("watchdog_timeout", 0.5)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("dry_run", False)
        self.declare_parameter("emit_stop_on_idle", True)
        self.declare_parameter("api_id_move", 1008)
        self.declare_parameter("api_id_stop", 1003)

        self._vx_max = float(self.get_parameter("vx_max").value)
        self._vy_max = float(self.get_parameter("vy_max").value)
        self._wz_max = float(self.get_parameter("wz_max").value)
        self._watchdog_timeout_ns = int(float(self.get_parameter("watchdog_timeout").value) * 1e9)
        publish_rate = max(float(self.get_parameter("publish_rate").value), 1.0)
        self._dry_run = bool(self.get_parameter("dry_run").value)
        self._emit_stop_on_idle = bool(self.get_parameter("emit_stop_on_idle").value)
        self._api_id_move = int(self.get_parameter("api_id_move").value)
        self._api_id_stop = int(self.get_parameter("api_id_stop").value)

        self._lock = threading.Lock()
        self._last_twist: Twist | None = None
        self._last_stamp_ns: int = 0
        # Start in idle so the first watchdog tick does NOT spuriously emit a StopMove
        # before any /cmd_vel has arrived.
        self._was_active: bool = False

        self._cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self._on_cmd_vel, 10,
        )
        self._req_pub = self.create_publisher(Request, "/api/sport/request", 10)

        self._timer = self.create_timer(1.0 / publish_rate, self._on_timer)

        self.get_logger().info(
            "Velocity bridge ready: caps=(%.2f, %.2f, %.2f) rate=%.1fHz watchdog=%.2fs dry_run=%s"
            % (
                self._vx_max, self._vy_max, self._wz_max, publish_rate,
                self._watchdog_timeout_ns / 1e9, self._dry_run,
            )
        )

    @property
    def dry_run(self) -> bool:
        return self._dry_run

    def _on_cmd_vel(self, msg: Twist) -> None:
        with self._lock:
            self._last_twist = msg
            self._last_stamp_ns = self.get_clock().now().nanoseconds

    def _on_timer(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        with self._lock:
            last_twist = self._last_twist
            last_stamp_ns = self._last_stamp_ns

        if last_twist is None or (now_ns - last_stamp_ns) > self._watchdog_timeout_ns:
            self._emit_idle()
        else:
            self._emit_active(last_twist)

    def _emit_active(self, twist: Twist) -> None:
        vx, vy, wz = clamp_twist(
            twist.linear.x, twist.linear.y, twist.angular.z,
            self._vx_max, self._vy_max, self._wz_max,
        )
        self._publish_move(encode_move_parameter(vx, vy, wz))
        self._was_active = True

    def _emit_idle(self) -> None:
        if self._was_active and self._emit_stop_on_idle:
            self._publish_stop("watchdog_timeout")
        self._was_active = False
        # Still stream a zero-Move at the publish rate so the locomotion controller
        # keeps the legs balanced under us.
        self._publish_move(encode_move_parameter(0.0, 0.0, 0.0))

    def _publish_move(self, parameter: str) -> None:
        if self._dry_run:
            self.get_logger().debug("[DRY_RUN] Move %s" % parameter)
            return
        req = Request()
        req.header.identity.api_id = self._api_id_move
        req.parameter = parameter
        self._req_pub.publish(req)

    def _publish_stop(self, reason: str) -> None:
        if self._dry_run:
            self.get_logger().info("[DRY_RUN] StopMove (%s)" % reason)
            return
        req = Request()
        req.header.identity.api_id = self._api_id_stop
        req.parameter = ""
        self._req_pub.publish(req)
        self.get_logger().info("StopMove emitted (%s)" % reason)

    def shutdown_stop(self) -> None:
        """Best-effort StopMove on shutdown so the robot doesn't run away."""
        try:
            self._publish_stop("shutdown")
        except Exception:  # pragma: no cover — defensive
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VelocityBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
