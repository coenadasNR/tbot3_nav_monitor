"""Unit tests for metrics_collector pure-math helpers (no ROS2 runtime needed)."""
import math
import time
import sys
import os
from unittest.mock import MagicMock
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
        sys.modules[_m] = MagicMock()

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import tbot3_nav_monitor.metrics_collector as mc
from tbot3_nav_monitor.metrics_collector import compute_accuracy, compute_efficiency

@pytest.fixture(autouse=True)
def _pin_goal_status_constants():
    """Keep GoalStatus constants deterministic per-test and restore afterwards."""
    pinned = {
        'STATUS_UNKNOWN': 0,
        'STATUS_ACCEPTED': 1,
        'STATUS_EXECUTING': 2,
        'STATUS_CANCELING': 3,
        'STATUS_SUCCEEDED': 4,
        'STATUS_CANCELED': 5,
        'STATUS_ABORTED': 6,
    }
    _missing = object()
    originals = {name: getattr(mc.GoalStatus, name, _missing) for name in pinned}

    for name, value in pinned.items():
        setattr(mc.GoalStatus, name, value)

    yield

    for name, value in originals.items():
        if value is _missing:
            try:
                delattr(mc.GoalStatus, name)
            except AttributeError:
                pass
        else:
            setattr(mc.GoalStatus, name, value)


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


# ── lifecycle transitions ──────────────────────────────────────────────────

def _make_lifecycle_node():
    node = mc.MetricsCollectorNode.__new__(mc.MetricsCollectorNode)
    node.get_logger = MagicMock(return_value=MagicMock())
    node.get_parameter = MagicMock(return_value=type('P', (), {'value': 2.0})())
    node.create_timer = MagicMock()
    node.destroy_timer = MagicMock()
    node.destroy_subscription = MagicMock()
    node.destroy_publisher = MagicMock()
    node._publish_metrics = MagicMock()
    node._metrics_timer = None
    node._pose_timer = None
    node._subs = []
    node._status_sub = None
    node._feedback_sub = None
    node._has_navigated = False
    node._has_efficiency = False
    node._goals_canceled = 0
    node._nav_client = None
    node._pub_exec_time = None
    node._pub_accuracy = None
    node._pub_efficiency = None
    node._pub_battery = None
    node._pub_recovery = None
    node._pub_status = None
    node._pub_alerts = None
    return node


def test_on_activate_replaces_existing_metrics_timer():
    node = _make_lifecycle_node()
    old_timer = MagicMock()
    new_timer = MagicMock()
    node._metrics_timer = old_timer
    node.create_timer.return_value = new_timer

    node.on_activate(None)

    node.destroy_timer.assert_called_once_with(old_timer)
    node.create_timer.assert_called_once()
    assert node._metrics_timer is new_timer


def test_on_deactivate_cancels_and_destroys_metrics_timer():
    node = _make_lifecycle_node()
    timer = MagicMock()
    node._metrics_timer = timer

    node.on_deactivate(None)

    timer.cancel.assert_called_once()
    node.destroy_timer.assert_called_once_with(timer)
    assert node._metrics_timer is None


def test_on_cleanup_releases_timers_subscriptions_and_publishers():
    node = _make_lifecycle_node()
    metrics_timer = MagicMock()
    pose_timer = MagicMock()
    node._metrics_timer = metrics_timer
    node._pose_timer = pose_timer
    node._subs = [MagicMock(), MagicMock()]
    node._status_sub = MagicMock()
    node._feedback_sub = MagicMock()
    node._pub_exec_time = MagicMock()
    node._pub_accuracy = MagicMock()
    node._pub_efficiency = MagicMock()
    node._pub_battery = MagicMock()
    node._pub_recovery = MagicMock()
    node._pub_status = MagicMock()
    node._pub_alerts = MagicMock()

    node.on_cleanup(None)

    metrics_timer.cancel.assert_called_once()
    pose_timer.cancel.assert_called_once()
    assert node.destroy_timer.call_count == 2
    assert node.destroy_subscription.call_count == 4
    assert node.destroy_publisher.call_count == 7

    assert node._metrics_timer is None
    assert node._pose_timer is None
    assert node._subs == []
    assert node._status_sub is None
    assert node._feedback_sub is None
    assert node._pub_exec_time is None
    assert node._pub_accuracy is None
    assert node._pub_efficiency is None
    assert node._pub_battery is None
    assert node._pub_recovery is None
    assert node._pub_status is None
    assert node._pub_alerts is None


# ── _handle_goal_end ───────────────────────────────────────────────────────

def _make_goal_end_node(current_pose=(1.0, 0.0), target_pose=(1.0, 0.0), path_length=1.0):
    """Minimal node stub with the state _handle_goal_end reads and writes."""
    node = mc.MetricsCollectorNode.__new__(mc.MetricsCollectorNode)
    node.get_logger = MagicMock(return_value=MagicMock())
    node._current_pose        = current_pose
    node._active_target_pose  = target_pose
    node._start_pose          = (0.0, 0.0)
    node._path_length_m       = path_length
    node._goal_start_time     = time.monotonic() - 5.0   # 5 s ago
    node._last_exec_time      = 0.0
    node._last_accuracy_m     = 0.0
    node._last_efficiency     = 0.0
    node._has_efficiency      = False
    node._goals_completed     = 0
    node._goals_failed        = 0
    node._goals_canceled      = 0
    node._is_canceling        = False
    node._goal_status         = 'ACTIVE'
    node._navigation_active   = True
    node._active_goal_id      = 'test-goal-id'
    return node


def test_succeeded_records_accuracy_and_efficiency():
    node = _make_goal_end_node(current_pose=(1.1, 0.0), target_pose=(1.0, 0.0), path_length=2.0)
    node._handle_goal_end(mc.GoalStatus.STATUS_SUCCEEDED)

    assert node._last_accuracy_m == pytest.approx(0.1, abs=1e-6)
    assert 0.0 < node._last_efficiency <= 1.0
    assert node._has_efficiency is True
    assert node._goals_completed == 1
    assert node._goals_failed    == 0
    assert node._goals_canceled  == 0
    assert node._goal_status     == 'SUCCEEDED'


def test_succeeded_clears_navigation_state():
    node = _make_goal_end_node()
    node._handle_goal_end(mc.GoalStatus.STATUS_SUCCEEDED)

    assert node._navigation_active  is False
    assert node._active_goal_id     is None
    assert node._active_target_pose is None


def test_succeeded_exec_time_is_positive():
    node = _make_goal_end_node()
    node._handle_goal_end(mc.GoalStatus.STATUS_SUCCEEDED)
    assert node._last_exec_time > 0.0


def test_aborted_within_1m_records_accuracy():
    # distance = 0.5 m — within gate
    node = _make_goal_end_node(current_pose=(1.5, 0.0), target_pose=(1.0, 0.0))
    node._handle_goal_end(mc.GoalStatus.STATUS_ABORTED)

    assert node._last_accuracy_m == pytest.approx(0.5, abs=1e-6)
    assert node._goals_failed    == 1
    assert node._goals_completed == 0
    assert node._goals_canceled  == 0
    assert node._goal_status     == 'FAILED'


def test_aborted_at_exactly_1m_records_accuracy():
    """Boundary: 1.0 m is inclusive (≤ 1.0)."""
    node = _make_goal_end_node(current_pose=(2.0, 0.0), target_pose=(1.0, 0.0))
    node._handle_goal_end(mc.GoalStatus.STATUS_ABORTED)

    assert node._last_accuracy_m == pytest.approx(1.0, abs=1e-6)
    assert node._goals_failed == 1


def test_aborted_beyond_1m_does_not_update_accuracy():
    # distance = 2.0 m — beyond gate; previous accuracy value must be preserved
    node = _make_goal_end_node(current_pose=(3.0, 0.0), target_pose=(1.0, 0.0))
    node._last_accuracy_m = 0.05   # last good reading from a previous success
    node._handle_goal_end(mc.GoalStatus.STATUS_ABORTED)

    assert node._last_accuracy_m == pytest.approx(0.05, abs=1e-6)   # unchanged
    assert node._goals_failed == 1


def test_aborted_does_not_set_has_efficiency():
    node = _make_goal_end_node(current_pose=(1.5, 0.0), target_pose=(1.0, 0.0))
    node._handle_goal_end(mc.GoalStatus.STATUS_ABORTED)

    assert node._has_efficiency is False


def test_aborted_clears_navigation_state():
    node = _make_goal_end_node(current_pose=(1.5, 0.0), target_pose=(1.0, 0.0))
    node._handle_goal_end(mc.GoalStatus.STATUS_ABORTED)

    assert node._navigation_active  is False
    assert node._active_goal_id     is None
    assert node._active_target_pose is None


def test_canceled_does_not_update_accuracy_or_efficiency():
    node = _make_goal_end_node(current_pose=(0.5, 0.0), target_pose=(2.0, 0.0))
    node._last_accuracy_m = 0.08
    node._last_efficiency = 0.90
    node._handle_goal_end(mc.GoalStatus.STATUS_CANCELED)

    assert node._last_accuracy_m == pytest.approx(0.08, abs=1e-6)   # unchanged
    assert node._last_efficiency  == pytest.approx(0.90, abs=1e-6)  # unchanged
    assert node._has_efficiency   is False


def test_canceled_increments_goals_canceled_not_goals_failed():
    node = _make_goal_end_node()
    node._goals_failed = 2   # pre-existing failures
    node._handle_goal_end(mc.GoalStatus.STATUS_CANCELED)

    assert node._goals_canceled == 1
    assert node._goals_failed   == 2   # unchanged
    assert node._goals_completed == 0


def test_canceled_clears_navigation_state():
    node = _make_goal_end_node()
    node._handle_goal_end(mc.GoalStatus.STATUS_CANCELED)

    assert node._navigation_active  is False
    assert node._active_goal_id     is None
    assert node._active_target_pose is None


def test_all_statuses_always_update_exec_time():
    for status in (
        mc.GoalStatus.STATUS_SUCCEEDED,
        mc.GoalStatus.STATUS_ABORTED,
        mc.GoalStatus.STATUS_CANCELED,
    ):
        node = _make_goal_end_node()
        node._last_exec_time = 0.0
        node._handle_goal_end(status)
        assert node._last_exec_time > 0.0, f'exec_time not updated for status {status}'


def test_handle_goal_end_always_clears_is_canceling():
    for status in (
        mc.GoalStatus.STATUS_SUCCEEDED,
        mc.GoalStatus.STATUS_ABORTED,
        mc.GoalStatus.STATUS_CANCELED,
    ):
        node = _make_goal_end_node()
        node._is_canceling = True   # simulate mid-cancel state
        node._handle_goal_end(status)
        assert node._is_canceling is False, f'_is_canceling not cleared for status {status}'


def test_canceled_sets_goal_status_to_canceled():
    node = _make_goal_end_node()
    node._handle_goal_end(mc.GoalStatus.STATUS_CANCELED)
    assert node._goal_status == 'CANCELED'
