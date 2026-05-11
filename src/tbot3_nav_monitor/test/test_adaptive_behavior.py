"""Unit tests for AdaptiveBehaviorNode decision logic (no ROS2 runtime required)."""
import json
import os
import sys
from collections import deque
from unittest.mock import MagicMock
import pytest

# conftest.py sets up ROS2 stubs before this module is imported.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import tbot3_nav_monitor.adaptive_behavior as ab

WINDOW = ab.WINDOW


# ── Helpers ───────────────────────────────────────────────────────────────────

def _make_windows(recovery=0, accuracy=0.0, efficiency=1.0, n=WINDOW):
    rw = deque([recovery] * n, maxlen=WINDOW)
    aw = deque([accuracy] * n, maxlen=WINDOW)
    ew = deque([efficiency] * n, maxlen=WINDOW)
    return rw, aw, ew


def _make_node():
    """Instantiate AdaptiveBehaviorNode without running __init__ ROS2 calls."""
    node = ab.AdaptiveBehaviorNode.__new__(ab.AdaptiveBehaviorNode)
    node._recovery_window     = deque(maxlen=WINDOW)
    node._accuracy_window     = deque(maxlen=WINDOW)
    node._efficiency_window   = deque(maxlen=WINDOW)
    node._current_max_vel     = ab._DEFAULT_MAX_VEL
    node._current_goal_tol    = ab._DEFAULT_GOAL_TOL
    node._current_cost_scaling = ab._DEFAULT_COST_SCALING
    node._param_clients       = {}
    node._pub_adjustments     = MagicMock()
    node._pub_alerts          = MagicMock()

    _thresholds = {
        'recovery_threshold':   3,
        'accuracy_threshold_m': 0.40,
        'efficiency_threshold': 0.60,
    }

    def _get_param(name):
        m = MagicMock()
        m.value = _thresholds.get(name, 0)
        return m

    node.get_parameter = _get_param
    node.get_logger    = MagicMock(return_value=MagicMock())
    # get_clock().now().to_msg().sec must return a real int for json.dumps
    _clock = MagicMock()
    _clock.now.return_value.to_msg.return_value.sec = 0
    node.get_clock = MagicMock(return_value=_clock)
    return node


# ── Threshold condition checks (data-level) ───────────────────────────────────

def test_no_velocity_change_when_recovery_low():
    rw, _, _ = _make_windows(recovery=1)
    assert sum(rw) / len(rw) < 3


def test_velocity_should_reduce_when_recovery_high():
    rw, _, _ = _make_windows(recovery=5)
    assert sum(rw) / len(rw) >= 3


def test_accuracy_below_threshold_no_change():
    _, aw, _ = _make_windows(accuracy=0.10)
    assert sum(aw) / len(aw) <= 0.40


def test_accuracy_above_threshold_triggers_relax():
    _, aw, _ = _make_windows(accuracy=0.60)
    assert sum(aw) / len(aw) > 0.40


def test_high_efficiency_no_change():
    _, _, ew = _make_windows(efficiency=0.90)
    assert sum(ew) / len(ew) >= 0.60


def test_low_efficiency_should_trigger_conservative_mode():
    _, _, ew = _make_windows(efficiency=0.40)
    assert sum(ew) / len(ew) < 0.60


# ── Constant sanity ───────────────────────────────────────────────────────────

def test_reduced_vel_less_than_default():
    assert ab._REDUCED_MAX_VEL < ab._DEFAULT_MAX_VEL


def test_relaxed_goal_tol_greater_than_default():
    assert ab._RELAXED_GOAL_TOL > ab._DEFAULT_GOAL_TOL


def test_high_cost_scaling_greater_than_default():
    # Rule 3 raises cost_scaling_factor to make planner prefer corridor centers
    assert ab._HIGH_COST_SCALING > ab._DEFAULT_COST_SCALING


# ── Node behaviour (_evaluate_and_adapt) ─────────────────────────────────────

def test_high_recovery_reduces_velocity():
    node = _make_node()
    node._recovery_window.extend([5] * WINDOW)
    node._evaluate_and_adapt()
    assert node._current_max_vel == ab._REDUCED_MAX_VEL


def test_velocity_restored_after_recovery_normalizes():
    node = _make_node()
    node._recovery_window.extend([5] * WINDOW)
    node._evaluate_and_adapt()
    assert node._current_max_vel == ab._REDUCED_MAX_VEL

    node._recovery_window.extend([0] * WINDOW)
    node._evaluate_and_adapt()
    assert node._current_max_vel == ab._DEFAULT_MAX_VEL


def test_poor_accuracy_relaxes_goal_tolerance():
    node = _make_node()
    node._accuracy_window.extend([0.6] * WINDOW)
    node._evaluate_and_adapt()
    assert node._current_goal_tol == ab._RELAXED_GOAL_TOL


def test_goal_tolerance_restored_after_accuracy_recovers():
    node = _make_node()
    node._accuracy_window.extend([0.6] * WINDOW)
    node._evaluate_and_adapt()
    assert node._current_goal_tol == ab._RELAXED_GOAL_TOL

    node._accuracy_window.extend([0.05] * WINDOW)
    node._evaluate_and_adapt()
    assert node._current_goal_tol == ab._DEFAULT_GOAL_TOL


def test_low_efficiency_raises_cost_scaling_and_reduces_velocity():
    """Rule 3 (conservative mode) must raise cost_scaling_factor and reduce velocity."""
    node = _make_node()
    node._efficiency_window.extend([0.3] * WINDOW)
    node._evaluate_and_adapt()
    assert node._current_cost_scaling == ab._HIGH_COST_SCALING
    assert node._current_max_vel == ab._REDUCED_MAX_VEL


def test_cost_scaling_and_velocity_restored_after_efficiency_recovers():
    node = _make_node()
    node._efficiency_window.extend([0.3] * WINDOW)
    node._evaluate_and_adapt()
    assert node._current_cost_scaling == ab._HIGH_COST_SCALING
    assert node._current_max_vel == ab._REDUCED_MAX_VEL

    node._efficiency_window.extend([0.9] * WINDOW)
    node._evaluate_and_adapt()
    assert node._current_cost_scaling == ab._DEFAULT_COST_SCALING
    assert node._current_max_vel == ab._DEFAULT_MAX_VEL


def test_rule1_does_not_restore_velocity_while_rule3_active():
    """Bug #21 regression: when efficiency is still low, Rule 1 must not undo
    Rule 3's velocity reduction even if recovery has normalised."""
    node = _make_node()
    # Both rules trigger reduction
    node._recovery_window.extend([5] * WINDOW)
    node._efficiency_window.extend([0.3] * WINDOW)
    node._evaluate_and_adapt()
    assert node._current_max_vel == ab._REDUCED_MAX_VEL

    # Recovery normalises but efficiency stays low — velocity must remain reduced
    node._recovery_window.extend([0] * WINDOW)
    node._evaluate_and_adapt()
    assert node._current_max_vel == ab._REDUCED_MAX_VEL  # held by Rule 3


def test_no_double_reduction():
    """Repeated adapt calls at high recovery must not keep publishing changes."""
    node = _make_node()
    node._recovery_window.extend([5] * WINDOW)
    node._evaluate_and_adapt()
    node._pub_adjustments.publish.reset_mock()
    node._evaluate_and_adapt()
    node._pub_adjustments.publish.assert_not_called()


def test_partial_window_does_not_crash():
    """Startup: only 1 sample in window — must not raise."""
    node = _make_node()
    node._recovery_window.append(5)
    node._evaluate_and_adapt()


def test_empty_window_skips_evaluation():
    """No data at all — must return without error or side effects."""
    node = _make_node()
    node._evaluate_and_adapt()
    node._pub_adjustments.publish.assert_not_called()


def test_no_oscillation_at_efficiency_threshold():
    """Values alternating just below/above threshold must not cause repeated changes.
    Restore requires > 0.85, so hovering around 0.60 should switch once and stay."""
    node = _make_node()
    for val in [0.59, 0.61, 0.59, 0.61, 0.59]:
        node._efficiency_window.extend([val] * WINDOW)
        node._evaluate_and_adapt()
    assert node._current_cost_scaling == ab._HIGH_COST_SCALING
    assert node._current_max_vel == ab._REDUCED_MAX_VEL


def test_apply_param_changes_publishes_only_ready_services():
    node = _make_node()

    ready_client = MagicMock()
    ready_client.service_is_ready.return_value = True
    ready_future = MagicMock()
    ready_client.call_async.return_value = ready_future

    not_ready_client = MagicMock()
    not_ready_client.service_is_ready.return_value = False

    node._param_clients = {
        'controller_server': ready_client,
        'local_costmap': not_ready_client,
    }

    node._apply_param_changes({
        ab._PARAM_MAX_VEL: 0.1,
        ab._PARAM_COST_SCALING: 8.0,
    })

    ready_client.call_async.assert_called_once()
    not_ready_client.call_async.assert_not_called()
    node._pub_adjustments.publish.assert_called_once()

    payload = json.loads(node._pub_adjustments.publish.call_args[0][0].data)
    assert 'controller_server.FollowPath.desired_linear_vel' in payload['changes']
    assert 'local_costmap.inflation_layer.cost_scaling_factor' not in payload['changes']


def test_apply_param_changes_skips_publish_when_no_services_ready():
    node = _make_node()

    c1 = MagicMock()
    c1.service_is_ready.return_value = False
    c2 = MagicMock()
    c2.service_is_ready.return_value = False
    node._param_clients = {
        'controller_server': c1,
        'local_costmap': c2,
    }

    node._apply_param_changes({
        ab._PARAM_MAX_VEL: 0.1,
    })

    c1.call_async.assert_not_called()
    c2.call_async.assert_not_called()
    node._pub_adjustments.publish.assert_not_called()
