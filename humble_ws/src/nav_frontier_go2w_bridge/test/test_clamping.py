"""Unit tests for the pure clamp / encode helpers."""
import json

import pytest

from nav_frontier_go2w_bridge.clamping import clamp, clamp_twist, encode_move_parameter


class TestClamp:
    def test_passthrough_when_within_cap(self):
        assert clamp(0.1, 0.5) == pytest.approx(0.1)
        assert clamp(-0.4, 0.5) == pytest.approx(-0.4)

    def test_clips_above_cap(self):
        assert clamp(1.0, 0.3) == pytest.approx(0.3)

    def test_clips_below_cap(self):
        assert clamp(-1.0, 0.3) == pytest.approx(-0.3)

    def test_zero_cap_forces_zero(self):
        assert clamp(0.5, 0.0) == 0.0
        assert clamp(-0.5, 0.0) == 0.0

    def test_negative_cap_forces_zero(self):
        assert clamp(0.5, -0.1) == 0.0


class TestClampTwist:
    def test_each_axis_clipped_independently(self):
        vx, vy, wz = clamp_twist(2.0, -3.0, 5.0, 0.30, 0.20, 0.50)
        assert vx == pytest.approx(0.30)
        assert vy == pytest.approx(-0.20)
        assert wz == pytest.approx(0.50)

    def test_in_range_passthrough(self):
        vx, vy, wz = clamp_twist(0.10, -0.05, 0.20, 0.30, 0.20, 0.50)
        assert (vx, vy, wz) == pytest.approx((0.10, -0.05, 0.20))


class TestEncodeMoveParameter:
    def test_keys_and_order(self):
        encoded = encode_move_parameter(0.1, 0.2, 0.3)
        decoded = json.loads(encoded)
        assert set(decoded.keys()) == {"x", "y", "z"}
        assert decoded["x"] == pytest.approx(0.1)
        assert decoded["y"] == pytest.approx(0.2)
        assert decoded["z"] == pytest.approx(0.3)

    def test_rounds_to_default_decimals(self):
        encoded = encode_move_parameter(0.123456789, 0.0, 0.0)
        decoded = json.loads(encoded)
        # round() banker's rounding; default 4 decimals.
        assert decoded["x"] == pytest.approx(0.1235)

    def test_zero_message_is_compact(self):
        # Zero-Move is sent every tick during idle; smaller is better for the wire.
        assert encode_move_parameter(0.0, 0.0, 0.0) == '{"x": 0.0, "y": 0.0, "z": 0.0}'
