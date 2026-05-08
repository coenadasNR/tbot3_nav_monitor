"""
Metrics Collector — passive lifecycle node that monitors Nav2 goals and collects
performance metrics in real-time.
"""
import math
import time
import json
from typing import Optional

import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from action_msgs.msg import GoalStatusArray, GoalStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, Int32, String


def compute_accuracy(current_pose, target_pose) -> float:
    """Euclidean distance from current_pose to target_pose. Returns 0.0 if either is None."""
    if current_pose is None or target_pose is None:
        return 0.0
    return math.hypot(current_pose[0] - target_pose[0], current_pose[1] - target_pose[1])


def compute_efficiency(goal_pose, start_pose, path_length_m: float) -> float:
    """||goal - start|| / path_length_m (straight-line vs actual path).
    Returns 0.0 when poses are None (no data yet).
    Returns 1.0 when path_length is zero and poses are valid (robot at goal)."""
    if goal_pose is None or start_pose is None:
        return 0.0
    straight_line = math.hypot(goal_pose[0] - start_pose[0], goal_pose[1] - start_pose[1])
    if path_length_m <= 0.0:
        return 1.0
    return min(1.0, straight_line / path_length_m)


class MetricsCollectorNode(LifecycleNode):
    def __init__(self):
        super().__init__('metrics_collector')

        self.declare_parameter('world', 'unknown')
        self.declare_parameter('publish_rate_hz', 2.0)
        self.declare_parameter('battery_drain_per_meter', 0.05)

        self._status_sub = None
        self._feedback_sub = None
        self._metrics_timer = None
        self._pose_timer = None
        self._subs = []

        self._current_pose: Optional[tuple] = None
        self._last_pose: Optional[tuple] = None

        # Per-goal tracking state
        self._navigation_active: bool = False
        self._active_goal_id: Optional[str] = None
        self._start_pose: Optional[tuple] = None
        self._target_pose: Optional[tuple] = None
        self._active_target_pose: Optional[tuple] = None
        self._goal_start_time: float = 0.0
        self._path_length_m: float = 0.0
        self._recovery_count: int = 0

        # Global stats
        self._total_distance_m: float = 0.0
        self._battery_pct: float = 100.0
        self._goals_completed: int = 0
        self._goals_failed: int = 0
        self._goal_status: str = 'IDLE'   # IDLE | ACTIVE | SUCCEEDED | FAILED

        # Latest calculated metrics (for publishing)
        self._last_exec_time: float = 0.0
        self._last_accuracy_m: float = 0.0
        self._last_efficiency: float = 0.0   # 0.0 = no data yet, not "perfect"
        self._has_navigated: bool = False    # gate: don't publish efficiency until first goal
        self._battery_alert_sent: bool = False

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._pub_exec_time = None
        self._pub_accuracy = None
        self._pub_efficiency = None
        self._pub_battery = None
        self._pub_recovery = None
        self._pub_status = None
        self._pub_alerts = None

    # ── Lifecycle ─────────────────────────────────────────────────────────

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring metrics_collector')

        best_effort_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._status_sub = self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status', self._status_callback, 10
        )
        self._feedback_sub = self.create_subscription(
            NavigateToPose.Impl.FeedbackMessage,
            '/navigate_to_pose/_action/feedback',
            self._feedback_callback, 10
        )

        self._pub_exec_time  = self.create_lifecycle_publisher(Float64, '/nav_monitor/execution_time', 10)
        self._pub_accuracy   = self.create_lifecycle_publisher(Float64, '/nav_monitor/nav_accuracy', 10)
        self._pub_efficiency = self.create_lifecycle_publisher(Float64, '/nav_monitor/path_efficiency', 10)
        self._pub_battery    = self.create_lifecycle_publisher(Float64, '/nav_monitor/battery_level', 10)
        self._pub_recovery   = self.create_lifecycle_publisher(Int32,   '/nav_monitor/recovery_count', 10)

        self._subs.append(self.create_subscription(
            PoseStamped, '/nav_monitor/target_pose', self._goal_pose_callback, 10
        ))
        self._subs.append(self.create_subscription(
            PoseStamped, '/goal_pose', self._goal_pose_callback, 10
        ))

        self._pose_timer = self.create_timer(0.1, self._update_pose_from_tf)

        self._pub_status = self.create_publisher(
            String, '/nav_monitor/live_data', qos_profile=best_effort_qos
        )
        self._pub_alerts = self.create_publisher(String, '/nav_monitor/alerts', 10)

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
        if self._feedback_sub:
            self.destroy_subscription(self._feedback_sub)
        return TransitionCallbackReturn.SUCCESS

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _goal_pose_callback(self, msg: PoseStamped) -> None:
        self._target_pose = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f'Target pose captured: {self._target_pose}')

    def _update_pose_from_tf(self) -> None:
        try:
            trans = self._tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            curr_x = trans.transform.translation.x
            curr_y = trans.transform.translation.y
            self._current_pose = (curr_x, curr_y)

            if self._last_pose:
                step = math.hypot(curr_x - self._last_pose[0], curr_y - self._last_pose[1])
                self._total_distance_m += step
                if self._navigation_active:
                    self._path_length_m += step

                drain = self.get_parameter('battery_drain_per_meter').value
                self._battery_pct = max(0.0, self._battery_pct - drain * step)

                if self._battery_pct < 20.0 and not self._battery_alert_sent:
                    self._send_critical_alert(f'CRITICAL: Battery Low ({self._battery_pct:.1f}%)')
                    self._battery_alert_sent = True

            self._last_pose = self._current_pose

        except TransformException:
            pass

    def _status_callback(self, msg: GoalStatusArray) -> None:
        _TERMINAL   = {GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED}
        _CANCELING  = GoalStatus.STATUS_CANCELING

        active_goal = next(
            (s for s in msg.status_list if s.status == GoalStatus.STATUS_EXECUTING), None
        )

        if self._navigation_active and self._active_goal_id:
            tracked = next(
                (s for s in msg.status_list
                 if str(s.goal_info.goal_id.uuid) == self._active_goal_id),
                None
            )

            if tracked:
                if tracked.status in _TERMINAL:
                    # Use the terminal status directly while the entry is still in the list;
                    # it is pruned quickly so the fallback below would otherwise report ABORTED.
                    # No early return — a preempting manual goal may already be EXECUTING
                    # in this same message and should be detected immediately below.
                    self._handle_goal_end(tracked.status)
                elif tracked.status == _CANCELING:
                    # Preempted goal is mid-cancellation; wait for STATUS_CANCELED before
                    # finalising metrics so the manual goal isn't missed by an early return.
                    return
            else:
                # Goal disappeared from the list before a terminal status was observed.
                self._handle_goal_end(GoalStatus.STATUS_ABORTED)

        # Detect new ACTIVE goal — runs immediately after _handle_goal_end when a manual
        # goal preempts a patrol goal in the same status message.
        if active_goal and not self._navigation_active:
            # /goal_pose and /nav_monitor/target_pose update _target_pose; if neither has
            # arrived yet we defer rather than snapshot a stale patrol waypoint.
            if self._target_pose is None:
                self.get_logger().warn(
                    'Goal executing but target pose not yet received — deferring goal start'
                )
                return
            
            gid = str(active_goal.goal_info.goal_id.uuid)
            self.get_logger().info(f'Goal active: {gid[:8]}')
            self._navigation_active = True
            self._has_navigated = True
            self._goal_status = 'ACTIVE'
            self._active_goal_id = gid
            self._start_pose = self._current_pose
            self._active_target_pose = self._target_pose
            self._goal_start_time = time.monotonic()
            self._path_length_m = 0.0
            self._recovery_count = 0


    def _handle_goal_end(self, status: int) -> None:
        self.get_logger().info(f'Goal ended with status {status}')

        self._last_exec_time  = time.monotonic() - self._goal_start_time
        self._last_accuracy_m = compute_accuracy(self._current_pose, self._active_target_pose)
        self._last_efficiency = compute_efficiency(
            self._active_target_pose, self._start_pose, self._path_length_m
        )

        if status == GoalStatus.STATUS_SUCCEEDED:
            self._goals_completed += 1
            self._goal_status = 'SUCCEEDED'
        else:
            self._goals_failed += 1
            self._goal_status = 'FAILED'

        self._navigation_active = False
        self._active_goal_id = None
        self._active_target_pose = None
        # _target_pose is intentionally NOT cleared here

    def _feedback_callback(self, feedback_msg) -> None:
        if self._navigation_active:
            self._recovery_count = feedback_msg.feedback.number_of_recoveries
            # Use _active_target_pose (snapshotted at goal start) not _target_pose,
            # which may already point to the next goal in a preemption scenario.
            self._last_efficiency = compute_efficiency(
                self._active_target_pose, self._start_pose, self._path_length_m
            )

    # ── Publishing ────────────────────────────────────────────────────────

    def _publish_metrics(self) -> None:
        if self._navigation_active:
            self._last_exec_time = time.monotonic() - self._goal_start_time

        stamp = self.get_clock().now().to_msg()

        self._pub_exec_time.publish(Float64(data=float(self._last_exec_time)))
        self._pub_accuracy.publish(Float64(data=float(self._last_accuracy_m)))
        if self._has_navigated:
            self._pub_efficiency.publish(Float64(data=float(self._last_efficiency)))
        self._pub_battery.publish(Float64(data=float(self._battery_pct)))
        self._pub_recovery.publish(Int32(data=int(self._recovery_count)))

        payload = {
            'timestamp':        stamp.sec + stamp.nanosec * 1e-9,
            'world':            self.get_parameter('world').value,
            'battery_pct':      round(self._battery_pct, 1),
            'total_distance_m': round(self._total_distance_m, 2),
            'execution_time_s': round(self._last_exec_time, 1),
            'nav_accuracy_m':   round(self._last_accuracy_m, 3),
            'path_efficiency':  round(self._last_efficiency, 2),
            'recovery_count':   self._recovery_count,
            'navigation_active': self._navigation_active,
            'goal_status':      self._goal_status,
            'goals_completed':  self._goals_completed,
            'goals_failed':     self._goals_failed,
        }
        self._pub_status.publish(String(data=json.dumps(payload)))

    def _send_critical_alert(self, message: str) -> None:
        self.get_logger().error(message)
        alert = {
            'timestamp': time.time(),
            'message': message,
        }
        self._pub_alerts.publish(String(data=json.dumps(alert)))


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
