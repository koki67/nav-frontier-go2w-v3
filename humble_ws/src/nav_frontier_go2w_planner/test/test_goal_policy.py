"""Unit tests for the pure-Python goal_policy module."""
import pytest

from nav_frontier_go2w_planner.goal_policy import (
    Action,
    GoalPolicy,
    GoalPose,
    Outcome,
    frames_match,
    planar_distance,
    should_cancel_for_timeout,
)


def _g(x: float = 0.0, y: float = 0.0, frame: str = "map") -> GoalPose:
    return GoalPose(frame_id=frame, x=x, y=y)


class TestPureFunctions:
    def test_frames_match_strips_whitespace(self):
        assert frames_match("map", "map")
        assert frames_match("  map", "map  ")
        assert not frames_match("map", "odom")

    def test_planar_distance(self):
        assert planar_distance(_g(0, 0), _g(3, 4)) == pytest.approx(5.0)

    def test_timeout_disabled_when_started_none(self):
        assert not should_cancel_for_timeout(None, 100.0, 10.0, False)

    def test_timeout_respected(self):
        assert not should_cancel_for_timeout(0.0, 5.0, 10.0, False)  # not yet
        assert should_cancel_for_timeout(0.0, 11.0, 10.0, False)     # past deadline

    def test_timeout_already_requested_blocks_repeat(self):
        assert not should_cancel_for_timeout(0.0, 100.0, 10.0, True)


class TestGoalPolicy:
    def test_dispatch_when_idle_and_ready(self):
        p = GoalPolicy(min_update_distance=0.5)
        d = p.offer(_g(1, 2), can_dispatch=True)
        assert d.action == Action.DISPATCH
        assert p.has_active()

    def test_queue_when_active(self):
        p = GoalPolicy(min_update_distance=0.5)
        p.offer(_g(0, 0), can_dispatch=True)
        d = p.offer(_g(5, 5), can_dispatch=True)
        assert d.action == Action.QUEUE
        assert p.pending == _g(5, 5)

    def test_duplicate_active_suppressed(self):
        p = GoalPolicy(min_update_distance=0.5)
        p.offer(_g(0, 0), can_dispatch=True)
        d = p.offer(_g(0.1, 0.1), can_dispatch=True)
        assert d.action == Action.NONE
        assert d.reason == "duplicate_active"

    def test_duplicate_pending_replaced_when_far(self):
        p = GoalPolicy(min_update_distance=0.5)
        p.offer(_g(0, 0), can_dispatch=True)
        p.offer(_g(5, 5), can_dispatch=True)  # queued
        d = p.offer(_g(10, 10), can_dispatch=True)
        assert d.action == Action.QUEUE
        assert p.pending == _g(10, 10)

    def test_completion_drains_pending(self):
        p = GoalPolicy(min_update_distance=0.5)
        p.offer(_g(0, 0), can_dispatch=True)
        p.offer(_g(5, 5), can_dispatch=True)  # queued
        d = p.complete_active(Outcome.SUCCEEDED)
        assert d.action == Action.DISPATCH
        assert d.goal == _g(5, 5)
        assert p.has_active()
        assert p.pending is None

    def test_completion_idle_when_nothing_pending(self):
        p = GoalPolicy(min_update_distance=0.5)
        p.offer(_g(0, 0), can_dispatch=True)
        d = p.complete_active(Outcome.SUCCEEDED)
        assert d.action == Action.NONE
        assert not p.has_active()

    def test_offer_when_not_ready_queues(self):
        p = GoalPolicy(min_update_distance=0.5)
        d = p.offer(_g(1, 2), can_dispatch=False)
        assert d.action == Action.QUEUE
        assert d.reason == "stored_until_ready"
        assert not p.has_active()

    def test_maybe_dispatch_pending_when_ready(self):
        p = GoalPolicy(min_update_distance=0.5)
        p.offer(_g(1, 2), can_dispatch=False)  # queued
        d = p.maybe_dispatch_pending(can_dispatch=True)
        assert d.action == Action.DISPATCH
        assert p.has_active()
        assert p.pending is None

    def test_maybe_dispatch_pending_blocked_by_active(self):
        p = GoalPolicy(min_update_distance=0.5)
        p.offer(_g(1, 2), can_dispatch=True)  # dispatched
        p.offer(_g(5, 5), can_dispatch=True)  # queued
        d = p.maybe_dispatch_pending(can_dispatch=True)
        assert d.action == Action.NONE
