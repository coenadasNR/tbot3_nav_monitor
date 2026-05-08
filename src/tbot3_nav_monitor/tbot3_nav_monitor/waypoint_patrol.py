"""
Waypoint Patrol — cycles through world-specific waypoints via NavigateToPose.
Designed for use with a pre-built AMCL map so localization is stable.
Goals are sent programmatically, giving metrics_collector clean execution-time data.
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

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
        (-1.5, -2.2,   0),   # C3 west end
        ( 1.5, -2.2, 180),   # C3 east end
        (-1.5, -0.4,   0),   # C5 west end
        ( 1.5, -0.4, 180),   # C5 east end
        (-1.5, -4.0,   0),   # C1 west end
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
        self._next_send: float = time.monotonic() + 5.0

        self._goal_pub = self.create_publisher(PoseStamped, '/nav_monitor/target_pose', 10)
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.create_timer(1.0, self._tick)
        self.get_logger().info(
            f'Waypoint patrol ready — {len(self._waypoints)} waypoints for world "{key}"'
        )

    # ── Patrol loop ───────────────────────────────────────────────────────

    def _tick(self) -> None:
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

        self._goal_pub.publish(goal.pose)
        self._goal_active = True
        self._client.send_goal_async(goal).add_done_callback(self._on_goal_accepted)

    def _on_goal_accepted(self, future) -> None:
        handle = future.result()
        if not handle or not handle.accepted:
            self.get_logger().warn('Goal rejected — skipping to next waypoint')
            self._advance()
            return
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        result = future.result()
        x, y, _ = self._waypoints[self._idx]
        status = result.status if result else '?'

        if result and result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Reached ({x:.1f}, {y:.1f})')
        else:
            self.get_logger().warn(f'Goal ({x:.1f}, {y:.1f}) ended with status {status}')
        self._advance()

    def _advance(self) -> None:
        self._idx = (self._idx + 1) % len(self._waypoints)
        self._next_send = time.monotonic() + self.get_parameter('loop_delay_sec').value
        self._goal_active = False


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
