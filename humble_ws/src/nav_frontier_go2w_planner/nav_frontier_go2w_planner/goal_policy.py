"""Pure logic for frontier-goal queueing, duplicate suppression, and timeouts.

The state machine has two slots — `active` (currently sent to Nav2) and `pending`
(queued because we already have an active goal). Incoming goals replace the
pending slot or get dispatched immediately when nothing is active.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional


@dataclass(frozen=True)
class GoalPose:
    frame_id: str
    x: float
    y: float
    qz: float = 0.0
    qw: float = 1.0


class Action(Enum):
    NONE = auto()
    DISPATCH = auto()
    QUEUE = auto()


class Outcome(Enum):
    SUCCEEDED = auto()
    FAILED = auto()
    CANCELED = auto()


@dataclass(frozen=True)
class Decision:
    action: Action
    goal: Optional[GoalPose] = None
    reason: str = ""


def planar_distance(a: GoalPose, b: GoalPose) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def frames_match(a: str, b: str) -> bool:
    return a.strip() == b.strip()


def should_cancel_for_timeout(
    started_at_monotonic: Optional[float],
    now_monotonic: float,
    timeout_sec: float,
    already_requested: bool,
) -> bool:
    if started_at_monotonic is None or timeout_sec <= 0.0 or already_requested:
        return False
    return (now_monotonic - started_at_monotonic) >= timeout_sec


class GoalPolicy:
    """Single active + single pending goal slots with duplicate suppression."""

    def __init__(self, min_update_distance: float) -> None:
        self.min_update_distance = max(0.0, min_update_distance)
        self.active: Optional[GoalPose] = None
        self.pending: Optional[GoalPose] = None

    def has_active(self) -> bool:
        return self.active is not None

    def offer(self, candidate: GoalPose, can_dispatch: bool) -> Decision:
        """Process a freshly-received goal."""
        if self._is_duplicate(candidate, self.active):
            return Decision(Action.NONE, reason="duplicate_active")
        if self._is_duplicate(candidate, self.pending):
            return Decision(Action.NONE, reason="duplicate_pending")

        if self.active is not None:
            self.pending = candidate
            return Decision(Action.QUEUE, candidate, "queued_while_active")

        if can_dispatch:
            self.active = candidate
            return Decision(Action.DISPATCH, candidate, "dispatch_now")

        self.pending = candidate
        return Decision(Action.QUEUE, candidate, "stored_until_ready")

    def maybe_dispatch_pending(self, can_dispatch: bool) -> Decision:
        """Promote a pending goal to active when the executor becomes ready."""
        if not can_dispatch or self.active is not None or self.pending is None:
            return Decision(Action.NONE, reason="not_dispatchable")
        next_goal = self.pending
        self.pending = None
        self.active = next_goal
        return Decision(Action.DISPATCH, next_goal, "dispatch_pending")

    def complete_active(self, outcome: Outcome) -> Decision:
        """Clear the active goal after Nav2 reports a result."""
        if self.active is None:
            return Decision(Action.NONE, reason="no_active")
        self.active = None
        if self.pending is not None:
            next_goal = self.pending
            self.pending = None
            self.active = next_goal
            return Decision(Action.DISPATCH, next_goal, "dispatch_pending")
        return Decision(Action.NONE, reason="idle")

    def _is_duplicate(self, candidate: GoalPose, existing: Optional[GoalPose]) -> bool:
        if existing is None:
            return False
        if not frames_match(candidate.frame_id, existing.frame_id):
            return False
        return planar_distance(candidate, existing) < self.min_update_distance
