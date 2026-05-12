"""Unit tests for DataLoggerNode — CSV writing and alert thresholds (no ROS2 runtime)."""
import csv
import io
import json
import os
import sys
from unittest.mock import MagicMock
import pytest

# conftest.py sets up ROS2 stubs before this module is imported.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import tbot3_nav_monitor.data_logger as dl


# ── Helpers ───────────────────────────────────────────────────────────────────

def _make_node():
    """Instantiate DataLoggerNode without running __init__ ROS2/filesystem calls."""
    node = dl.DataLoggerNode.__new__(dl.DataLoggerNode)
    node.get_logger  = MagicMock(return_value=MagicMock())
    node._pub_alerts = MagicMock()
    node._row_count  = 0
    node._alert_battery_sent  = False
    node._alert_accuracy_sent = False
    node._alert_exec_sent     = False
    node._alert_recovery_sent = False

    _params = {
        'data_dir': '/tmp', 'flush_every_n': 10,
        'alert_battery_pct': 20.0, 'alert_accuracy_m': 0.50,
        'alert_exec_time_s': 45.0, 'alert_recovery_count': 5,
    }

    def _get_param(name):
        m = MagicMock()
        m.value = _params.get(name, '')
        return m

    node.get_parameter = _get_param

    buf = io.StringIO()
    node._csv_file = buf
    node._writer = csv.DictWriter(buf, fieldnames=[
        'timestamp', 'world', 'execution_time_s', 'nav_accuracy_m',
        'path_efficiency', 'recovery_count', 'battery_pct',
        'total_distance_m', 'navigation_active', 'goals_completed',
    ], extrasaction='ignore')
    node._writer.writeheader()
    return node


def _msg(data: dict):
    m = MagicMock()
    m.data = json.dumps(data)
    return m


def _good_data(**overrides):
    base = {
        'timestamp': 10.0, 'world': 'obstacles',
        'execution_time_s': 5.0, 'nav_accuracy_m': 0.1,
        'path_efficiency': 0.85, 'recovery_count': 0,
        'battery_pct': 80.0, 'total_distance_m': 2.0,
        'navigation_active': True, 'goals_completed': 0,
    }
    base.update(overrides)
    return base


# ── CSV writing ───────────────────────────────────────────────────────────────

def test_csv_header_contains_required_fields():
    node = _make_node()
    content = node._csv_file.getvalue()
    for field in ('timestamp', 'nav_accuracy_m', 'path_efficiency',
                  'recovery_count', 'battery_pct', 'goals_completed'):
        assert field in content


def test_metrics_row_written_on_callback():
    node = _make_node()
    node._metrics_cb(_msg(_good_data(nav_accuracy_m=0.25)))
    rows = list(csv.DictReader(io.StringIO(node._csv_file.getvalue())))
    assert len(rows) == 1
    assert float(rows[0]['nav_accuracy_m']) == pytest.approx(0.25)


def test_multiple_rows_accumulate():
    node = _make_node()
    for i in range(5):
        node._metrics_cb(_msg(_good_data(timestamp=float(i))))
    rows = list(csv.DictReader(io.StringIO(node._csv_file.getvalue())))
    assert len(rows) == 5


def test_malformed_json_does_not_crash():
    node = _make_node()
    bad = MagicMock()
    bad.data = "not json {"
    node._metrics_cb(bad)
    rows = list(csv.DictReader(io.StringIO(node._csv_file.getvalue())))
    assert len(rows) == 0


# ── Alert thresholds ─────────────────────────────────────────────────────────

def test_no_alert_for_normal_data():
    node = _make_node()
    node._metrics_cb(_msg(_good_data()))
    node._pub_alerts.publish.assert_not_called()


def test_low_battery_triggers_alert():
    node = _make_node()
    node._metrics_cb(_msg(_good_data(battery_pct=15.0)))
    node._pub_alerts.publish.assert_called_once()
    payload = json.loads(node._pub_alerts.publish.call_args[0][0].data)
    assert 'BATTERY' in payload['message'].upper()


def test_battery_at_threshold_triggers_alert():
    node = _make_node()
    node._metrics_cb(_msg(_good_data(battery_pct=20.0)))   # == alert threshold
    node._pub_alerts.publish.assert_called_once()


def test_battery_just_above_threshold_no_alert():
    node = _make_node()
    node._metrics_cb(_msg(_good_data(battery_pct=20.1)))   # just above threshold
    node._pub_alerts.publish.assert_not_called()


def test_poor_accuracy_triggers_alert():
    node = _make_node()
    node._metrics_cb(_msg(_good_data(nav_accuracy_m=0.6)))
    node._pub_alerts.publish.assert_called_once()
    payload = json.loads(node._pub_alerts.publish.call_args[0][0].data)
    assert 'ACCURACY' in payload['message'].upper()


def test_high_recovery_triggers_alert():
    node = _make_node()
    node._metrics_cb(_msg(_good_data(recovery_count=5)))   # == alert threshold
    node._pub_alerts.publish.assert_called_once()
    payload = json.loads(node._pub_alerts.publish.call_args[0][0].data)
    assert 'RECOVERY' in payload['message'].upper()


def test_slow_navigation_triggers_alert():
    node = _make_node()
    node._metrics_cb(_msg(_good_data(execution_time_s=46.0)))   # above 45s threshold
    node._pub_alerts.publish.assert_called_once()
