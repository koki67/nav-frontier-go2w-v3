r"""Subscribe to /frontier_goal and dispatch each goal to Nav2 NavigateToPose.

State machine:
  IDLE  --(/frontier_goal)-->  ACTIVE  --(SUCCEEDED|FAILED|CANCELED)-->  IDLE
                       \-(/frontier_goal during ACTIVE)-> queue as PENDING

Cancels the in-flight goal when it has been running longer than goal_timeout_sec.
Suppresses /frontier_goal updates that fall within `min_goal_update_distance` of
the active or pending goal so we don't thrash the planner.
"""
from __future__ import annotations

import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener

from nav_frontier_go2w_planner.goal_policy import (
    Action,
    Decision,
    GoalPolicy,
    GoalPose,
    Outcome,
    frames_match,
    should_cancel_for_timeout,
)


class FrontierGoalExecutorNode(Node):
    def __init__(self) -> None:
        super().__init__("frontier_goal_executor")

        self.declare_parameter("frontier_goal_topic", "/frontier_goal")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("min_goal_update_distance", 0.5)
        self.declare_parameter("goal_timeout_sec", 180.0)
        self.declare_parameter("result_check_rate", 2.0)
        self.declare_parameter("tf_timeout", 0.2)

        self._goal_topic = self.get_parameter("frontier_goal_topic").value
        self._global_frame = self.get_parameter("global_frame").value
        self._robot_base_frame = self.get_parameter("robot_base_frame").value
        self._min_update = max(0.0, float(self.get_parameter("min_goal_update_distance").value))
        self._goal_timeout = max(0.0, float(self.get_parameter("goal_timeout_sec").value))
        result_rate = max(0.5, float(self.get_parameter("result_check_rate").value))
        self._tf_timeout = float(self.get_parameter("tf_timeout").value)

        self._policy = GoalPolicy(min_update_distance=self._min_update)
        self._throttle: dict[str, float] = {}

        self._tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._navigator_lock = threading.RLock()
        self._navigator = BasicNavigator(node_name="frontier_goal_executor_navigator")
        self._active_started_at: Optional[float] = None
        self._timeout_requested = False
        self._shutdown_flag = False

        self._goal_sub = self.create_subscription(
            PoseStamped, self._goal_topic, self._on_goal, 10,
        )
        self._timer = self.create_timer(1.0 / result_rate, self._on_timer)

        self.get_logger().info(
            "Frontier goal executor ready: topic=%s frame=%s base=%s min_update=%.2f timeout=%.0fs",
            self._goal_topic, self._global_frame, self._robot_base_frame,
            self._min_update, self._goal_timeout,
        )

    def destroy_node(self) -> bool:
        self._shutdown_flag = True
        try:
            with self._navigator_lock:
                self._navigator.destroy_node()
        except Exception:
            pass
        return super().destroy_node()

    # --- Inputs --------------------------------------------------------------

    def _on_goal(self, msg: PoseStamped) -> None:
        frame_id = msg.header.frame_id.strip()
        if not frames_match(frame_id, self._global_frame):
            self._throttled(
                "wrong_frame",
                "Rejecting goal in frame '%s' (expected '%s')." % (frame_id or "<empty>", self._global_frame),
                5.0,
            )
            return
        candidate = GoalPose(
            frame_id=frame_id,
            x=msg.pose.position.x,
            y=msg.pose.position.y,
            qz=msg.pose.orientation.z,
            qw=msg.pose.orientation.w,
        )
        decision = self._policy.offer(candidate, can_dispatch=self._can_dispatch())
        self._apply(decision)

    def _on_timer(self) -> None:
        if self._policy.has_active():
            self._maybe_cancel_for_timeout()
            self._maybe_collect_result()
            return
        # No active goal — try to drain the pending slot if we have one and TF is OK.
        decision = self._policy.maybe_dispatch_pending(can_dispatch=self._can_dispatch())
        self._apply(decision)

    # --- Outputs -------------------------------------------------------------

    def _apply(self, decision: Decision) -> None:
        if decision.action == Action.NONE:
            if decision.reason in {"duplicate_active", "duplicate_pending"}:
                self._throttled(decision.reason, "Suppressing near-duplicate frontier goal.", 5.0)
            return
        if decision.action == Action.QUEUE and decision.goal is not None:
            self.get_logger().info(
                "Queued goal: x=%.2f y=%.2f (%s)",
                decision.goal.x, decision.goal.y, decision.reason,
            )
            return
        if decision.action == Action.DISPATCH and decision.goal is not None:
            self._dispatch(decision.goal)

    def _dispatch(self, goal: GoalPose) -> None:
        msg = self._pose_msg(goal)
        try:
            with self._navigator_lock:
                accepted = self._navigator.goToPose(msg)
        except Exception as exc:
            self.get_logger().error("Nav2 goToPose() raised: %s", exc)
            self._apply(self._policy.complete_active(Outcome.FAILED))
            return
        if not accepted:
            self.get_logger().error(
                "Nav2 rejected goal x=%.2f y=%.2f.", goal.x, goal.y,
            )
            self._apply(self._policy.complete_active(Outcome.FAILED))
            return
        self._active_started_at = time.monotonic()
        self._timeout_requested = False
        self.get_logger().info("Sent goal to Nav2: x=%.2f y=%.2f", goal.x, goal.y)

    def _maybe_cancel_for_timeout(self) -> None:
        if not should_cancel_for_timeout(
            started_at_monotonic=self._active_started_at,
            now_monotonic=time.monotonic(),
            timeout_sec=self._goal_timeout,
            already_requested=self._timeout_requested,
        ):
            return
        self._timeout_requested = True
        active = self._policy.active
        self.get_logger().error(
            "Timeout (%.0fs) on goal x=%.2f y=%.2f — canceling.",
            self._goal_timeout, active.x if active else 0.0, active.y if active else 0.0,
        )
        try:
            with self._navigator_lock:
                self._navigator.cancelTask()
        except Exception as exc:
            self.get_logger().error("cancelTask() raised: %s", exc)

    def _maybe_collect_result(self) -> None:
        try:
            with self._navigator_lock:
                if not self._navigator.isTaskComplete():
                    return
                result = self._navigator.getResult()
        except Exception as exc:
            self.get_logger().error("Nav2 result query failed: %s", exc)
            result = TaskResult.UNKNOWN

        active = self._policy.active
        was_timeout = self._timeout_requested
        self._timeout_requested = False
        self._active_started_at = None

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(
                "Goal succeeded: x=%.2f y=%.2f",
                active.x if active else 0.0, active.y if active else 0.0,
            )
            outcome = Outcome.SUCCEEDED
        elif result == TaskResult.CANCELED:
            log = self.get_logger().error if was_timeout else self.get_logger().warning
            log(
                "Goal %s: x=%.2f y=%.2f",
                "canceled (timeout)" if was_timeout else "canceled",
                active.x if active else 0.0, active.y if active else 0.0,
            )
            outcome = Outcome.CANCELED
        else:
            self.get_logger().error(
                "Goal failed: x=%.2f y=%.2f",
                active.x if active else 0.0, active.y if active else 0.0,
            )
            outcome = Outcome.FAILED

        self._apply(self._policy.complete_active(outcome))

    # --- Helpers -------------------------------------------------------------

    def _can_dispatch(self) -> bool:
        return (not self._policy.has_active()) and self._has_required_tf(log=False)

    def _has_required_tf(self, log: bool) -> bool:
        try:
            self._tf_buffer.lookup_transform(
                self._global_frame, self._robot_base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self._tf_timeout),
            )
        except TransformException as exc:
            if log:
                self._throttled(
                    "tf_unavailable",
                    "Waiting for TF %s -> %s: %s" % (self._global_frame, self._robot_base_frame, exc),
                    5.0,
                )
            return False
        return True

    def _pose_msg(self, goal: GoalPose) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = goal.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = goal.x
        msg.pose.position.y = goal.y
        msg.pose.position.z = 0.0
        msg.pose.orientation.z = goal.qz
        msg.pose.orientation.w = goal.qw
        return msg

    def _throttled(self, key: str, message: str, period_s: float) -> None:
        now = time.monotonic()
        last = self._throttle.get(key, 0.0)
        if (now - last) >= period_s:
            self._throttle[key] = now
            self.get_logger().warning(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrontierGoalExecutorNode()
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
