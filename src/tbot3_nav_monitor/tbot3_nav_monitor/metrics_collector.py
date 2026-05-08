"""
Metrics Collector — passive lifecycle node that monitors Nav2 goals and tracks
execution time per navigation goal.
"""
import math
import time
import json
from typing import Optional

import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from action_msgs.msg import GoalStatusArray, GoalStatus
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, String


class MetricsCollectorNode(LifecycleNode):
    def __init__(self):
        super().__init__('metrics_collector')

        self.declare_parameter('world', 'unknown')
        self.declare_parameter('publish_rate_hz', 2.0)

        self._status_sub = None
        self._metrics_timer = None
        self._pose_timer = None
        self._subs = []

        self._current_pose: Optional[tuple] = None
        self._last_pose: Optional[tuple] = None

        self._navigation_active: bool = False
        self._active_goal_id: Optional[str] = None
        self._goal_start_time: float = 0.0

        self._total_distance_m: float = 0.0
        self._last_exec_time: float = 0.0

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._pub_exec_time = None
        self._pub_status = None

    # ── Lifecycle ─────────────────────────────────────────────────────────

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring metrics_collector')

        self._status_sub = self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status', self._status_callback, 10
        )
        self._subs.append(self.create_subscription(
            PoseStamped, '/goal_pose', lambda msg: None, 10
        ))

        self._pub_exec_time = self.create_lifecycle_publisher(Float64, '/nav_monitor/execution_time', 10)
        self._pub_status = self.create_publisher(String, '/nav_monitor/live_data', 10)

        self._pose_timer = self.create_timer(0.1, self._update_pose_from_tf)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Activating metrics_collector')
        rate = self.get_parameter('publish_rate_hz').value
        self._metrics_timer = self.create_timer(1.0 / rate, self._publish_metrics)
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        if self._metrics_timer:
            self._metrics_timer.cancel()
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        for sub in self._subs:
            self.destroy_subscription(sub)
        self._subs.clear()
        if self._status_sub:
            self.destroy_subscription(self._status_sub)
        return TransitionCallbackReturn.SUCCESS

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _update_pose_from_tf(self) -> None:
        try:
            trans = self._tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            curr_x = trans.transform.translation.x
            curr_y = trans.transform.translation.y
            self._current_pose = (curr_x, curr_y)

            if self._last_pose:
                step = math.hypot(curr_x - self._last_pose[0], curr_y - self._last_pose[1])
                self._total_distance_m += step

            self._last_pose = self._current_pose

        except TransformException:
            pass

    def _status_callback(self, msg: GoalStatusArray) -> None:
        _TERMINAL = {GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED}

        active_goal = next(
            (s for s in msg.status_list if s.status == GoalStatus.STATUS_EXECUTING), None
        )

        if self._navigation_active and self._active_goal_id:
            tracked = next(
                (s for s in msg.status_list
                 if str(s.goal_info.goal_id.uuid) == self._active_goal_id),
                None
            )
            if tracked and tracked.status in _TERMINAL:
                self._last_exec_time = time.monotonic() - self._goal_start_time
                self._navigation_active = False
                self._active_goal_id = None
            elif not tracked:
                self._last_exec_time = time.monotonic() - self._goal_start_time
                self._navigation_active = False
                self._active_goal_id = None

        if active_goal and not self._navigation_active:
            gid = str(active_goal.goal_info.goal_id.uuid)
            self._navigation_active = True
            self._active_goal_id = gid
            self._goal_start_time = time.monotonic()

    # ── Publishing ────────────────────────────────────────────────────────

    def _publish_metrics(self) -> None:
        if self._navigation_active:
            self._last_exec_time = time.monotonic() - self._goal_start_time

        stamp = self.get_clock().now().to_msg()
        self._pub_exec_time.publish(Float64(data=float(self._last_exec_time)))

        payload = {
            'timestamp':        stamp.sec + stamp.nanosec * 1e-9,
            'world':            self.get_parameter('world').value,
            'execution_time_s': round(self._last_exec_time, 1),
            'total_distance_m': round(self._total_distance_m, 2),
            'navigation_active': self._navigation_active,
        }
        self._pub_status.publish(String(data=json.dumps(payload)))


def main(args=None):
    rclpy.init(args=args)
    node = MetricsCollectorNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        node.trigger_configure()
        node.trigger_activate()
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.trigger_deactivate()
        node.trigger_cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
