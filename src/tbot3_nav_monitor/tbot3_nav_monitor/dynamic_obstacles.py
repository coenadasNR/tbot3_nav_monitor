"""
Dynamic Obstacles — publishes velocity commands to two physics-enabled box models
in the narrow passages world, making them oscillate back and forth along the x-axis.
The boxes have real collision geometry so they appear in the TurtleBot3's lidar scan
and force Nav2 to re-plan around them mid-run.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class DynamicObstaclesNode(Node):
    def __init__(self):
        super().__init__('dynamic_obstacles')

        self.declare_parameter('speed', 0.2)
        self.declare_parameter('half_period_sec', 5.0)

        self._pub1 = self.create_publisher(Twist, '/dynamic_obs_1/cmd_vel', 10)
        self._pub2 = self.create_publisher(Twist, '/dynamic_obs_2/cmd_vel', 10)

        self._start = self.get_clock().now()
        self.create_timer(0.1, self._tick)
        self.get_logger().info('Dynamic obstacles controller started')

    def _tick(self) -> None:
        speed = self.get_parameter('speed').value
        half  = self.get_parameter('half_period_sec').value
        period = 2.0 * half

        elapsed = (self.get_clock().now() - self._start).nanoseconds * 1e-9

        # Obstacle 1 — starts moving in +x direction
        phase1 = elapsed % period
        vx1 = speed if phase1 < half else -speed

        # Obstacle 2 — 180° phase offset so the two move in opposite directions
        phase2 = (elapsed + half) % period
        vx2 = speed if phase2 < half else -speed

        t1, t2 = Twist(), Twist()
        t1.linear.x = float(vx1)
        t2.linear.x = float(vx2)
        self._pub1.publish(t1)
        self._pub2.publish(t2)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstaclesNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        node._pub1.publish(stop)
        node._pub2.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
