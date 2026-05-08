"""Unit tests for metrics_collector pure-math helpers (no ROS2 runtime needed)."""
import math
import sys
import os
import pytest

# Stub out every ROS2 import so the module loads without a live ROS installation
_MOCKS = [
    'rclpy', 'rclpy.lifecycle', 'rclpy.action', 'rclpy.qos',
    'rclpy.executors', 'rclpy.publisher', 'rclpy.time',
    'tf2_ros', 'tf2_ros.buffer', 'tf2_ros.transform_listener',
    'nav2_msgs', 'nav2_msgs.action',
    'nav_msgs', 'nav_msgs.msg',
    'std_msgs', 'std_msgs.msg',
    'geometry_msgs', 'geometry_msgs.msg',
    'action_msgs', 'action_msgs.msg',
]
for _m in _MOCKS:
    if _m not in sys.modules:
        from unittest.mock import MagicMock
        sys.modules[_m] = MagicMock()

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from tbot3_nav_monitor.metrics_collector import compute_accuracy, compute_efficiency


# ── compute_accuracy ───────────────────────────────────────────────────────

def test_accuracy_exact_goal():
    assert compute_accuracy((1.0, 1.0), (1.0, 1.0)) == pytest.approx(0.0)


def test_accuracy_offset():
    assert compute_accuracy((1.3, 1.0), (1.0, 1.0)) == pytest.approx(0.3, abs=1e-6)


def test_accuracy_diagonal():
    expected = math.hypot(1.0, 1.0)
    assert compute_accuracy((0.0, 0.0), (1.0, 1.0)) == pytest.approx(expected)


def test_accuracy_none_current():
    assert compute_accuracy(None, (1.0, 1.0)) == 0.0


def test_accuracy_none_goal():
    assert compute_accuracy((1.0, 1.0), None) == 0.0


def test_accuracy_both_none():
    assert compute_accuracy(None, None) == 0.0


# ── compute_efficiency ────────────────────────────────────────────────────

def test_efficiency_straight_path():
    """Actual path == straight-line → efficiency 1.0."""
    assert compute_efficiency((1.0, 0.0), (0.0, 0.0), 1.0) == pytest.approx(1.0)


def test_efficiency_longer_path():
    """Actual path twice as long → efficiency 0.5."""
    assert compute_efficiency((1.0, 0.0), (0.0, 0.0), 2.0) == pytest.approx(0.5, abs=1e-6)


def test_efficiency_never_negative():
    assert compute_efficiency((1.0, 0.0), (0.0, 0.0), 999.0) >= 0.0


def test_efficiency_zero_distance_no_crash():
    """Goal == start → no division by zero, returns 1.0."""
    assert compute_efficiency((0.0, 0.0), (0.0, 0.0), 0.0) == pytest.approx(1.0)


def test_efficiency_none_goal():
    # None pose = no data yet — return 0.0, not misleading 1.0
    assert compute_efficiency(None, (0.0, 0.0), 1.0) == pytest.approx(0.0)


def test_efficiency_none_start():
    assert compute_efficiency((1.0, 0.0), None, 1.0) == pytest.approx(0.0)
