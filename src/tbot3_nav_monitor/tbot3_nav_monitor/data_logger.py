"""
Data Logger — subscribes to all metric topics, writes timestamped CSV rows,
and publishes console/topic alerts when thresholds are breached.
Alert thresholds are ROS parameters so they can be reconfigured at runtime.
"""
import csv
import json
import os
import time
from pathlib import Path
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DataLoggerNode(Node):
    def __init__(self):
        super().__init__('data_logger')

        self.declare_parameter('data_dir', '/ros2_ws/data/csv')
        self.declare_parameter('flush_every_n', 10)
        # Alert thresholds as ROS parameters — reconfigurable at runtime
        self.declare_parameter('alert_battery_pct',      20.0)
        self.declare_parameter('alert_accuracy_m',        0.50)
        self.declare_parameter('alert_exec_time_s',      45.0)
        self.declare_parameter('alert_recovery_count',    5)

        data_dir = Path(self.get_parameter('data_dir').value)
        data_dir.mkdir(parents=True, exist_ok=True)

        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._csv_path = data_dir / f'nav_metrics_{ts}.csv'

        self._csv_file = open(self._csv_path, 'w', newline='')
        self._writer = csv.DictWriter(self._csv_file, fieldnames=[
            'timestamp', 'world', 'execution_time_s', 'nav_accuracy_m',
            'path_efficiency', 'recovery_count', 'battery_pct',
            'total_distance_m', 'navigation_active', 'goals_completed',
            'goals_failed', 'goals_canceled', 'goal_status'
        ], extrasaction='ignore')
        self._writer.writeheader()
        self._row_count = 0

        from rclpy.qos import QoSProfile, ReliabilityPolicy
        best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self._pub_alerts = self.create_publisher(String, '/nav_monitor/alerts', 10)

        self.create_subscription(String, '/nav_monitor/live_data',         self._metrics_cb,     qos_profile=best_effort)
        self.create_subscription(String, '/nav_monitor/param_adjustments', self._adjustments_cb, 10)

        # Per-alert-type deduplication flags — set when an alert fires, cleared when
        # the condition drops below threshold.  Prevents continuous re-firing at 2 Hz
        # for a condition that stays true across many live_data messages.
        self._alert_battery_sent  = False
        self._alert_accuracy_sent = False
        self._alert_exec_sent     = False
        self._alert_recovery_sent = False

        self.get_logger().info(f'Logging to {self._csv_path}')

    def _metrics_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        self._writer.writerow(data)
        self._row_count += 1

        flush_n = self.get_parameter('flush_every_n').value
        if self._row_count % flush_n == 0:
            self._csv_file.flush()

        self._check_alerts(data)

    def _adjustments_cb(self, msg: String) -> None:
        self.get_logger().info(f'[Adaptive] {msg.data}')

    def _check_alerts(self, data: dict) -> None:
        alert_battery  = self.get_parameter('alert_battery_pct').value
        alert_accuracy = self.get_parameter('alert_accuracy_m').value
        alert_exec     = self.get_parameter('alert_exec_time_s').value
        alert_recovery = self.get_parameter('alert_recovery_count').value

        alerts = []

        # Each condition uses a "sent" flag so the alert fires once when the condition
        # first becomes true, and fires again only after it has cleared and re-triggered.

        battery = data.get('battery_pct', 100)
        if battery <= alert_battery:
            if not self._alert_battery_sent:
                alerts.append(f'LOW BATTERY: {battery:.1f}%')
                self._alert_battery_sent = True
        else:
            self._alert_battery_sent = False

        accuracy = data.get('nav_accuracy_m', 0)
        if accuracy > alert_accuracy:
            if not self._alert_accuracy_sent:
                alerts.append(f'POOR ACCURACY: {accuracy:.3f}m (threshold {alert_accuracy}m)')
                self._alert_accuracy_sent = True
        else:
            self._alert_accuracy_sent = False

        exec_time = data.get('execution_time_s', 0)
        if exec_time > alert_exec:
            if not self._alert_exec_sent:
                alerts.append(f'SLOW NAVIGATION: {exec_time:.1f}s (threshold {alert_exec}s)')
                self._alert_exec_sent = True
        else:
            self._alert_exec_sent = False

        recovery = data.get('recovery_count', 0)
        if recovery >= alert_recovery:
            if not self._alert_recovery_sent:
                alerts.append(f'HIGH RECOVERY COUNT: {recovery} recoveries')
                self._alert_recovery_sent = True
        else:
            self._alert_recovery_sent = False

        for alert in alerts:
            self.get_logger().warn(f'[ALERT] {alert}')
            payload = String()
            payload.data = json.dumps({
                'timestamp': time.time(),
                'message': alert,
            })
            self._pub_alerts.publish(payload)

    def destroy_node(self):
        self._csv_file.flush()
        self._csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
