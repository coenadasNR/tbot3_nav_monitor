"""
Adaptive Behavior — lifecycle node that reads rolling metrics and dynamically
reconfigures Nav2 parameters to compensate for poor navigation performance.
"""
import json
import time
from collections import deque
from typing import Optional

import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import Float64, Int32, String


# Nav2 nodes, their SetParameters service paths, and the parameters we tune.
# local_costmap node is namespaced as local_costmap/local_costmap in Nav2.
_PARAM_MAX_VEL      = ('controller_server', 'FollowPath.desired_linear_vel')
_PARAM_GOAL_TOL     = ('controller_server', 'general_goal_checker.xy_goal_tolerance')
_PARAM_COST_SCALING = ('local_costmap',     'inflation_layer.cost_scaling_factor')
_PARAM_ROT_VEL      = ('controller_server', 'FollowPath.rotate_to_heading_angular_vel')

_SERVICE_PATHS = {
    'controller_server': '/controller_server/set_parameters',
    'local_costmap':     '/local_costmap/local_costmap/set_parameters',
}

_DEFAULT_MAX_VEL      = 0.20
_DEFAULT_GOAL_TOL     = 0.15   # matches nav2_params*.yaml xy_goal_tolerance
_DEFAULT_ROT_VEL      = 1.8    # matches nav2_params*.yaml rotate_to_heading_angular_vel
_DEFAULT_COST_SCALING = 3.0    # matches nav2_params*.yaml cost_scaling_factor

_REDUCED_MAX_VEL    = 0.10
_RELAXED_GOAL_TOL   = 0.35
_HIGH_COST_SCALING  = 8.0      # steeper cost gradient → planner prefers corridor centers
_SLOW_ROT_VEL       = 0.9

WINDOW = 5  # rolling window for metric averaging


class AdaptiveBehaviorNode(LifecycleNode):
    def __init__(self):
        super().__init__('adaptive_behavior')

        self.declare_parameter('recovery_threshold', 3)
        self.declare_parameter('accuracy_threshold_m', 0.40)
        self.declare_parameter('efficiency_threshold', 0.60)
        self.declare_parameter('check_period_sec', 5.0)

        self._recovery_window:   deque = deque(maxlen=WINDOW)
        self._accuracy_window:   deque = deque(maxlen=WINDOW)
        self._efficiency_window: deque = deque(maxlen=WINDOW)

        self._current_max_vel      = _DEFAULT_MAX_VEL
        self._current_goal_tol     = _DEFAULT_GOAL_TOL
        self._current_cost_scaling = _DEFAULT_COST_SCALING

        self._subs = []
        self._check_timer = None
        self._param_clients: dict = {}
        self._pub_adjustments: Optional[rclpy.publisher.Publisher] = None
        self._pub_alerts: Optional[rclpy.publisher.Publisher] = None

    # ── Lifecycle ─────────────────────────────────────────────────────────

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring adaptive_behavior')

        self._subs.append(self.create_subscription(
            Float64, '/nav_monitor/nav_accuracy',
            lambda m: self._accuracy_window.append(m.data), 10
        ))
        self._subs.append(self.create_subscription(
            Float64, '/nav_monitor/path_efficiency',
            lambda m: self._efficiency_window.append(m.data), 10
        ))
        self._subs.append(self.create_subscription(
            Int32, '/nav_monitor/recovery_count',
            lambda m: self._recovery_window.append(m.data), 10
        ))

        self._pub_adjustments = self.create_lifecycle_publisher(
            String, '/nav_monitor/param_adjustments', 10
        )
        self._pub_alerts = self.create_lifecycle_publisher(
            String, '/nav_monitor/alerts', 10
        )

        for node_name, svc_path in _SERVICE_PATHS.items():
            self._param_clients[node_name] = self.create_client(SetParameters, svc_path)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        period = self.get_parameter('check_period_sec').value
        self._check_timer = self.create_timer(period, self._evaluate_and_adapt)
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        if self._check_timer:
            self._check_timer.cancel()
        return super().on_deactivate(state)

    # ── Adaptation logic ──────────────────────────────────────────────────

    def _evaluate_and_adapt(self) -> None:
        if not any([self._recovery_window, self._accuracy_window, self._efficiency_window]):
            return

        avg_recovery   = sum(self._recovery_window)   / len(self._recovery_window)   if self._recovery_window   else 0
        avg_accuracy   = sum(self._accuracy_window)   / len(self._accuracy_window)   if self._accuracy_window   else 0.0
        avg_efficiency = sum(self._efficiency_window) / len(self._efficiency_window) if self._efficiency_window else 1.0

        rec_thresh = self.get_parameter('recovery_threshold').value
        acc_thresh = self.get_parameter('accuracy_threshold_m').value
        eff_thresh = self.get_parameter('efficiency_threshold').value

        changes = {}

        # Rule 1: frequent recovery → slow down (linear & rotational velocity).
        # Restore is gated on efficiency too, so Rule 3 can hold the velocity reduced
        # even when recovery is low (otherwise the two rules ping-pong each tick).
        if avg_recovery >= rec_thresh:
            if self._current_max_vel > _REDUCED_MAX_VEL:
                self._current_max_vel = _REDUCED_MAX_VEL
                changes[_PARAM_MAX_VEL] = _REDUCED_MAX_VEL
                changes[_PARAM_ROT_VEL] = _SLOW_ROT_VEL
                msg = f'Degradation: High recovery rate ({avg_recovery:.1f}). Slowing down.'
                self.get_logger().warn(msg)
                self._send_alert(msg)
        elif avg_recovery < rec_thresh * 0.5 and avg_efficiency >= eff_thresh:
            if self._current_max_vel < _DEFAULT_MAX_VEL:
                self._current_max_vel = _DEFAULT_MAX_VEL
                changes[_PARAM_MAX_VEL] = _DEFAULT_MAX_VEL
                changes[_PARAM_ROT_VEL] = _DEFAULT_ROT_VEL
                self.get_logger().info('Recovery normal, restoring speeds')

        # Rule 2: poor accuracy → relax goal tolerance
        if avg_accuracy > acc_thresh:
            if self._current_goal_tol < _RELAXED_GOAL_TOL:
                self._current_goal_tol = _RELAXED_GOAL_TOL
                changes[_PARAM_GOAL_TOL] = _RELAXED_GOAL_TOL
                msg = f'Degradation: Poor accuracy ({avg_accuracy:.3f}m). Relaxing tolerance.'
                self.get_logger().warn(msg)
                self._send_alert(msg)
        elif avg_accuracy < acc_thresh * 0.5:
            if self._current_goal_tol > _DEFAULT_GOAL_TOL:
                self._current_goal_tol = _DEFAULT_GOAL_TOL
                changes[_PARAM_GOAL_TOL] = _DEFAULT_GOAL_TOL
                self.get_logger().info('Accuracy recovered, restoring goal tolerance')

        # Rule 3: low efficiency → more conservative path planning.
        # Steeper inflation cost gradient (higher cost_scaling_factor) makes the planner
        # prefer paths down corridor centers and away from obstacle edges, without
        # shrinking navigable space. Also reduces velocity for conservative execution.
        # Inflation radius itself is kept at the per-world default (set in nav2_params*.yaml).
        if avg_efficiency < eff_thresh:
            changed = False
            if self._current_cost_scaling < _HIGH_COST_SCALING:
                self._current_cost_scaling = _HIGH_COST_SCALING
                changes[_PARAM_COST_SCALING] = _HIGH_COST_SCALING
                changed = True
            if self._current_max_vel > _REDUCED_MAX_VEL:
                self._current_max_vel = _REDUCED_MAX_VEL
                changes[_PARAM_MAX_VEL] = _REDUCED_MAX_VEL
                changes[_PARAM_ROT_VEL] = _SLOW_ROT_VEL
                changed = True
            if changed:
                msg = (f'Low efficiency ({avg_efficiency:.2f}): conservative mode '
                       f'— cost_scaling→{_HIGH_COST_SCALING}, vel→{_REDUCED_MAX_VEL}')
                self.get_logger().warn(msg)
                self._send_alert(msg)
        elif avg_efficiency > 0.85:
            changed = False
            if self._current_cost_scaling > _DEFAULT_COST_SCALING:
                self._current_cost_scaling = _DEFAULT_COST_SCALING
                changes[_PARAM_COST_SCALING] = _DEFAULT_COST_SCALING
                changed = True
            # Restore velocity only if Rule 1 also doesn't need it reduced
            if avg_recovery < rec_thresh * 0.5 and self._current_max_vel < _DEFAULT_MAX_VEL:
                self._current_max_vel = _DEFAULT_MAX_VEL
                changes[_PARAM_MAX_VEL] = _DEFAULT_MAX_VEL
                changes[_PARAM_ROT_VEL] = _DEFAULT_ROT_VEL
                changed = True
            if changed:
                self.get_logger().info(
                    f'Efficiency recovered, restoring cost_scaling → {_DEFAULT_COST_SCALING}'
                )

        if changes:
            self._apply_param_changes(changes)

    def _apply_param_changes(self, changes: dict) -> None:
        by_node: dict = {}
        for (node_name, param_name), value in changes.items():
            by_node.setdefault(node_name, {})[param_name] = value

        for node_name, params in by_node.items():
            client = self._param_clients.get(node_name)
            if client is None or not client.service_is_ready():
                self.get_logger().debug(f'SetParameters service not ready for {node_name} — skipping')
                continue

            req = SetParameters.Request()
            for pname, pval in params.items():
                p = Parameter()
                p.name = pname
                p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(pval))
                req.parameters.append(p)

            future = client.call_async(req)
            future.add_done_callback(
                lambda f, nn=node_name: self._on_param_set_done(f, nn)
            )

        msg = String()
        msg.data = json.dumps({'changes': {f'{n}.{p}': v for (n, p), v in changes.items()}})
        self._pub_adjustments.publish(msg)

    def _on_param_set_done(self, future, node_name: str) -> None:
        try:
            result = future.result()
            for r in result.results:
                if not r.successful:
                    self.get_logger().warn(f'Parameter set failed on {node_name}: {r.reason}')
        except Exception as e:
            self.get_logger().error(f'SetParameters call to {node_name} raised: {e}')

    def _send_alert(self, message: str) -> None:
        if self._pub_alerts is None:
            return
        alert = {
            'timestamp': time.time(),
            'message': message
        }
        self._pub_alerts.publish(String(data=json.dumps(alert)))


def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveBehaviorNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        node.trigger_configure()
        node.trigger_activate()
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
