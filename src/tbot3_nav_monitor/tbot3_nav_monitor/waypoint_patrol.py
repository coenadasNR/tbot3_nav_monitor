"""
Waypoint Patrol — cycles through world-specific waypoints via NavigateToPose.
Designed for use with a pre-built AMCL map so localization is stable.
Goals are sent programmatically, giving metrics_collector clean execution-time data.

Manual interruption: if a goal is published to /goal_pose (e.g. via RViz2 "2D Goal Pose"),
the patrol pauses, lets the robot complete the manual goal, then resumes from the next
waypoint. Two flags prevent a race-condition between patrol cancellation and manual goal start:
  _manual_pending  — /goal_pose received, waiting to see the goal enter EXECUTING state
  _awaiting_manual — patrol is paused until the manual goal finishes
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from action_msgs.msg import GoalStatus, GoalStatusArray
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

# (x, y, yaw_degrees) — positions chosen to be in clear navigable space per world
_WAYPOINTS: dict = {
    'obstacles': [
        ( 1.5,  2.0, 225),   # NE
        ( 0.5, -1.0,  45),   # SW
        ( 1.5, -1.0, 135),   # SE
        ( 0.5,  2.0, 315),   # NW
        ( 2.0,  1.0,   0),   # Off-center North
        ( 2.0,  0.0, 180),   # Off-center South
    ],
    'house': [
        (-4.0,  1.8,   0),   # left room
        (-2.5, -0.7,   0),   # left room lower
        ( 0.5,  1.8, 180),   # central hallway
        ( 6.5,  1.8, 180),   # right room upper
        ( 6.5, -0.7, 180),   # right room lower
    ],
    'narrow': [
        ( 1.5, -4.0,   0),   # C1 east end
        (-1.5, -2.2,   0),   # C3 west end  (robot traverses narrow C2 to reach here)
        ( 1.5, -2.2, 180),   # C3 east end
        (-1.5, -0.8,   0),   # C5 west end  (robot traverses narrow C4 to reach here)
        ( 1.5, -0.8, 180),   # C5 east end
        (-1.5, -4.0,   0),   # C1 west end  (return through all corridors)
    ],
}


def _make_pose(x: float, y: float, yaw_deg: float) -> PoseStamped:
    yaw = math.radians(yaw_deg)
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


class WaypointPatrolNode(Node):
    def __init__(self):
        super().__init__('waypoint_patrol')

        self.declare_parameter('world', 'obstacles')
        self.declare_parameter('loop_delay_sec', 2.0)

        world = self.get_parameter('world').value
        key = 'narrow' if 'narrow' in world else world
        self._waypoints = _WAYPOINTS.get(key, _WAYPOINTS['obstacles'])
        self._idx: int = 0
        self._goal_active: bool = False
        self._next_send: float = time.monotonic() + 5.0  # let Nav2 finish starting up

        # Manual interruption state
        self._manual_pending: bool = False   # /goal_pose arrived, waiting to see it EXECUTING
        self._awaiting_manual: bool = False  # patrol paused until manual goal finishes
        self._manual_start_time: float = 0.0
        self._MANUAL_TIMEOUT_SEC: float = 120.0  # resume patrol if manual goal never clears

        self._goal_pub = self.create_publisher(PoseStamped, '/nav_monitor/target_pose', 10)
        self._preempted_pub = self.create_publisher(String, '/nav_monitor/preempted_goal_id', 10)
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._active_goal_handle = None

        # Detect manual goals from RViz2 "2D Goal Pose"
        self.create_subscription(PoseStamped, '/goal_pose', self._on_manual_goal, 10)

        # Monitor action status to know when manual goal completes
        _status_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self._on_action_status,
            _status_qos,
        )

        self.create_timer(1.0, self._tick)
        self.get_logger().info(
            f'Waypoint patrol ready — {len(self._waypoints)} waypoints for world "{key}"'
        )

    # ── Main patrol loop ──────────────────────────────────────────────────

    def _tick(self) -> None:
        if self._awaiting_manual:
            if time.monotonic() - self._manual_start_time > self._MANUAL_TIMEOUT_SEC:
                self.get_logger().warn('Manual goal timeout — resuming patrol')
                self._awaiting_manual = False
                self._manual_pending = False
                self._advance()
            return
        if self._goal_active or time.monotonic() < self._next_send:
            return
        if not self._client.server_is_ready():
            self.get_logger().info('Waiting for navigate_to_pose action server...')
            return

        x, y, yaw = self._waypoints[self._idx]
        self.get_logger().info(
            f'Patrol [{self._idx + 1}/{len(self._waypoints)}] → ({x:.1f}, {y:.1f})'
        )
        goal = NavigateToPose.Goal()
        goal.pose = _make_pose(x, y, yaw)
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        # Publish the UUID of the goal being superseded *before* sending the new one so
        # metrics_collector can classify the incoming ABORTED as a preemption, not a failure.
        if self._active_goal_handle is not None:
            uid = ''.join(f'{b:02x}' for b in self._active_goal_handle.goal_id.uuid)
            self._preempted_pub.publish(String(data=uid))
            self._active_goal_handle = None

        self._goal_pub.publish(goal.pose)
        self._goal_active = True
        self._client.send_goal_async(goal).add_done_callback(self._on_goal_accepted)

    def _on_goal_accepted(self, future) -> None:
        handle = future.result()
        if not handle or not handle.accepted:
            self.get_logger().warn('Goal rejected — skipping to next waypoint')
            self._advance()
            return
        self._active_goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        result = future.result()
        x, y, _ = self._waypoints[self._idx]
        status = result.status if result else '?'

        # Clear the handle so _tick does not publish its UUID as "preempted"
        # on the next send — this goal has already reached a terminal state.
        self._active_goal_handle = None

        if result and result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Reached ({x:.1f}, {y:.1f})')
            self._advance()
        elif self._awaiting_manual:
            # Patrol goal was preempted by a manual RViz2 goal — don't advance yet.
            # _on_action_status will call _advance() once the manual goal finishes.
            self.get_logger().info(
                f'Patrol paused at waypoint {self._idx + 1} — waiting for manual goal'
            )
            self._goal_active = False
        else:
            self.get_logger().warn(f'Goal ({x:.1f}, {y:.1f}) ended with status {status}')
            self._advance()

    def _advance(self) -> None:
        self._idx = (self._idx + 1) % len(self._waypoints)
        self._next_send = time.monotonic() + self.get_parameter('loop_delay_sec').value
        self._goal_active = False

    # ── Manual goal handling ──────────────────────────────────────────────

    def _on_manual_goal(self, msg: PoseStamped) -> None:
        """Called when user sends a 2D Goal Pose from RViz2."""
        self.get_logger().info(
            'Manual goal received — patrol will resume at next waypoint after completion'
        )
        self._manual_pending = True
        self._awaiting_manual = True
        self._manual_start_time = time.monotonic()

    def _on_action_status(self, msg: GoalStatusArray) -> None:
        """Track the Nav2 action status to detect when the manual goal finishes."""
        if not self._awaiting_manual:
            return

        active_statuses = {GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING}
        has_active = any(s.status in active_statuses for s in msg.status_list)

        if self._manual_pending:
            if has_active:
                # Manual goal is now running — no longer just pending
                self._manual_pending = False
            return

        # Manual goal was seen running; if nothing is active now, it finished
        if not has_active:
            self.get_logger().info('Manual goal finished — resuming patrol')
            self._awaiting_manual = False
            self._advance()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointPatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
