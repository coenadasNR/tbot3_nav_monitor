"""Unit tests for WebDashboardNode lifecycle resource management."""
import os
import sys
from unittest.mock import MagicMock

# conftest.py stubs ROS2 modules for test environment.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import tbot3_nav_monitor.web_dashboard as wd


def _make_node():
    node = wd.WebDashboardNode.__new__(wd.WebDashboardNode)
    node._subs = []
    node._http_server = None
    node._http_thread = None
    node.destroy_subscription = MagicMock()
    node.get_logger = MagicMock(return_value=MagicMock())
    return node


def test_on_deactivate_stops_flask_server():
    node = _make_node()
    node._stop_flask = MagicMock()

    node.on_deactivate(None)

    node._stop_flask.assert_called_once()


def test_on_cleanup_stops_flask_and_destroys_subscriptions():
    node = _make_node()
    node._stop_flask = MagicMock()
    s1, s2 = MagicMock(), MagicMock()
    node._subs = [s1, s2]

    node.on_cleanup(None)

    node._stop_flask.assert_called_once()
    node.destroy_subscription.assert_any_call(s1)
    node.destroy_subscription.assert_any_call(s2)
    assert node.destroy_subscription.call_count == 2
    assert node._subs == []


def test_stop_flask_shuts_down_server_and_clears_refs():
    node = _make_node()
    server = MagicMock()
    thread = MagicMock()
    node._http_server = server
    node._http_thread = thread

    node._stop_flask()

    server.shutdown.assert_called_once()
    server.server_close.assert_called_once()
    thread.join.assert_called_once_with(timeout=2.0)
    assert node._http_server is None
    assert node._http_thread is None


def test_stop_flask_noop_when_server_not_running():
    node = _make_node()

    node._stop_flask()

    assert node._http_server is None
    assert node._http_thread is None


def test_start_flask_is_noop_when_already_running():
    node = _make_node()
    running_server = MagicMock()
    node._http_server = running_server

    node._start_flask()

    assert node._http_server is running_server


def test_start_flask_starts_server_when_not_running(monkeypatch):
    import importlib

    fake_server = MagicMock()

    # Stub Flask ecosystem into sys.modules so the try/except import succeeds
    fake_flask = MagicMock()
    fake_flask.Flask = lambda name: MagicMock()
    fake_flask_cors = MagicMock()
    fake_flask_cors.CORS = lambda app: None
    fake_werkzeug = MagicMock()
    fake_werkzeug_serving = MagicMock()
    fake_werkzeug_serving.make_server = lambda host, port, app: fake_server

    monkeypatch.setitem(sys.modules, 'flask', fake_flask)
    monkeypatch.setitem(sys.modules, 'flask_cors', fake_flask_cors)
    monkeypatch.setitem(sys.modules, 'werkzeug', fake_werkzeug)
    monkeypatch.setitem(sys.modules, 'werkzeug.serving', fake_werkzeug_serving)

    importlib.reload(wd)

    class _FakeThread:
        def __init__(self, target=None, daemon=None):
            self.target = target
            self.daemon = daemon
            self.started = False
        def start(self):
            self.started = True

    monkeypatch.setattr(wd.threading, 'Thread', _FakeThread)

    node = _make_node()
    node.get_parameter = MagicMock(return_value=type('P', (), {'value': 8080})())
    node._latest = {}
    node._alert_log = []
    node._summary_cache = None
    node._summary_cache_time = 0.0

    node._start_flask()

    assert node._http_server is fake_server
    assert node._http_thread is not None
    assert node._http_thread.started is True
    assert node._http_thread.target == fake_server.serve_forever
