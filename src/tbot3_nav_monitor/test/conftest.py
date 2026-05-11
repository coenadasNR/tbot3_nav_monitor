"""
Pytest fixtures and module stubs shared across all tests.

These stubs let unit tests import `tbot3_nav_monitor.*` modules without a real
rclpy installation. The stubs are inserted via `setdefault` so they only apply
when the real ROS2 packages are not already loaded — running the same tests on
a system with rclpy installed will use the real classes (which is fine because
the unit tests instantiate via `__new__` and don't call ROS2 methods).
"""
import sys
import types
from unittest.mock import MagicMock


class _LifecycleNode:
    def __init__(self, *a, **kw): pass
    def get_logger(self): return MagicMock()
    def get_clock(self): return MagicMock()
    def on_activate(self, state): return 0
    def on_deactivate(self, state): return 0
    def declare_parameter(self, *a, **kw): pass
    def get_parameter(self, name): m = MagicMock(); m.value = 0; return m
    def create_subscription(self, *a, **kw): return MagicMock()
    def create_lifecycle_publisher(self, *a, **kw): return MagicMock()
    def create_client(self, *a, **kw): return MagicMock()
    def create_action_client(self, *a, **kw): return MagicMock()
    def create_timer(self, *a, **kw): return MagicMock()
    def destroy_node(self): pass


class _Node:
    def __init__(self, *a, **kw): pass
    def get_logger(self): return MagicMock()
    def declare_parameter(self, *a, **kw): pass
    def get_parameter(self, name): m = MagicMock(); m.value = 0; return m
    def create_subscription(self, *a, **kw): return MagicMock()
    def create_publisher(self, *a, **kw): return MagicMock()
    def create_timer(self, *a, **kw): return MagicMock()
    def destroy_node(self): pass


_TransitionCallbackReturn = type('TransitionCallbackReturn', (), {'SUCCESS': 0})

_lifecycle_mod = types.ModuleType('rclpy.lifecycle')
_lifecycle_mod.LifecycleNode = _LifecycleNode
_lifecycle_mod.LifecycleState = object
_lifecycle_mod.TransitionCallbackReturn = _TransitionCallbackReturn

_node_mod = types.ModuleType('rclpy.node')
_node_mod.Node = _Node

_qos_mod = types.ModuleType('rclpy.qos')
_qos_mod.QoSProfile = MagicMock()
_qos_mod.ReliabilityPolicy = MagicMock()
_qos_mod.DurabilityPolicy = MagicMock()

# setdefault preserves real rclpy when installed; only stubs in if absent.
sys.modules.setdefault('rclpy.lifecycle', _lifecycle_mod)
sys.modules.setdefault('rclpy.node', _node_mod)
sys.modules.setdefault('rclpy.qos', _qos_mod)

for _mod in ('rclpy', 'rclpy.executors', 'rclpy.publisher', 'rclpy.action',
             'rclpy.time', 'rclpy.qos',
             'rcl_interfaces', 'rcl_interfaces.msg', 'rcl_interfaces.srv',
             'std_msgs', 'std_msgs.msg',
             'lifecycle_msgs', 'lifecycle_msgs.msg',
             'tf2_ros', 'tf2_ros.buffer', 'tf2_ros.transform_listener',
             'nav2_msgs', 'nav2_msgs.action',
             'nav_msgs', 'nav_msgs.msg',
             'geometry_msgs', 'geometry_msgs.msg',
             'action_msgs', 'action_msgs.msg'):
    sys.modules.setdefault(_mod, MagicMock())
