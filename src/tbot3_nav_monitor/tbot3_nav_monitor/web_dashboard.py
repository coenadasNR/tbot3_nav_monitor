"""
Web Dashboard — lifecycle node that serves a live metrics dashboard on port 8080.
Flask runs in a background thread started on_activate so the ROS2 spin loop stays
unblocked. Subscriptions are created on_configure; Flask starts on_activate.
"""
import glob
import json
import os
import threading

import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

try:
    from flask import Flask, jsonify, render_template_string
    from flask_cors import CORS
    _FLASK_OK = True
except ImportError:
    _FLASK_OK = False

try:
    import pandas as pd
    _PANDAS_OK = True
except ImportError:
    _PANDAS_OK = False

_HTML = """
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>TurtleBot3 Nav Monitor</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { font-family: 'Segoe UI', sans-serif; background: #0f0f1a; color: #e0e0e0; }

  header { background: #1a1a2e; padding: 1rem 2rem; border-bottom: 2px solid #4a90d9; display:flex; align-items:center; justify-content:space-between; }
  header h1 { color: #4a90d9; font-size: 1.4rem; }
  #last-update { font-size: 0.8rem; color: #888; }

  /* World banner */
  #world-banner { display:flex; align-items:center; gap:1rem; padding:0.75rem 2rem; background:#12122a; border-bottom:1px solid #2a2a4a; }
  #world-icon  { font-size:1.8rem; }
  #world-label { font-size:0.7rem; color:#888; text-transform:uppercase; letter-spacing:1px; }
  #world-name  { font-size:1.6rem; font-weight:700; color:#fff; text-transform:uppercase; letter-spacing:3px; }
  #world-accent { flex:1; height:3px; border-radius:2px; background:#4a90d9; margin-left:1rem; }

  /* Goal tracking strip */
  #goal-strip { display:grid; grid-template-columns:2fr 1fr 1fr; gap:1rem; padding:1.5rem 2rem 0; }
  .goal-card { background:#1a1a2e; border-radius:8px; padding:1.2rem 1.5rem; display:flex; align-items:center; gap:1rem; border-left:4px solid #555; }
  .goal-card.status-idle      { border-left-color:#555; }
  .goal-card.status-active    { border-left-color:#4a90d9; }
  .goal-card.status-succeeded { border-left-color:#2ecc71; }
  .goal-card.status-failed    { border-left-color:#e74c3c; }
  .gs-icon { font-size:2rem; width:2.5rem; text-align:center; }
  .gs-text h3 { font-size:0.7rem; color:#888; text-transform:uppercase; letter-spacing:1px; }
  .gs-text .gsv { font-size:1.5rem; font-weight:700; color:#fff; }
  .goal-stat { background:#1a1a2e; border-radius:8px; padding:1.2rem 1.5rem; }
  .goal-stat h3 { font-size:0.7rem; color:#888; text-transform:uppercase; letter-spacing:1px; }
  .goal-stat .gsv { font-size:2rem; font-weight:700; color:#fff; margin:0.3rem 0; }
  .goal-stat .gsu { font-size:0.75rem; color:#888; }

  /* Metric cards */
  .grid { display:grid; grid-template-columns:repeat(auto-fit, minmax(180px, 1fr)); gap:1rem; padding:1rem 2rem; }
  .card { background:#1a1a2e; border-radius:8px; padding:1.2rem 1.5rem; border-left:4px solid #4a90d9; }
  .card.warn  { border-left-color:#f0a500; }
  .card.alert { border-left-color:#e74c3c; }
  .card h2 { font-size:0.75rem; color:#888; text-transform:uppercase; letter-spacing:1px; }
  .card .value { font-size:1.8rem; font-weight:bold; margin:0.4rem 0; color:#fff; }
  .card .unit  { font-size:0.75rem; color:#888; }

  .section { padding:0 2rem 2rem; }
  .section h2 { color:#4a90d9; font-size:1rem; margin-bottom:1rem; text-transform:uppercase; letter-spacing:1px; }

  table { width:100%; border-collapse:collapse; font-size:0.85rem; }
  th { background:#1a1a2e; padding:0.6rem 1rem; text-align:left; color:#4a90d9; font-weight:600; }
  td { padding:0.6rem 1rem; border-bottom:1px solid #1a1a2e; }
  tr:nth-child(odd) td { background:#13131f; }

  .alerts h2 { color:#e74c3c; }
  .alert-item { background:#2a1a1a; border-left:4px solid #e74c3c; padding:0.5rem 1rem; margin:0.25rem 0; border-radius:4px; font-size:0.85rem; }

  @keyframes pulse { 0%,100%{opacity:1} 50%{opacity:0.4} }
  .pulsing { animation:pulse 1.4s ease-in-out infinite; }
</style>
</head>
<body>

<header>
  <h1>&#x1F916; TurtleBot3 Navigation Monitor</h1>
  <span id="last-update">Connecting...</span>
</header>

<!-- World banner -->
<div id="world-banner">
  <span id="world-icon">&#x1F30D;</span>
  <div>
    <div id="world-label">Current Environment</div>
    <div id="world-name">—</div>
  </div>
  <div id="world-accent"></div>
</div>

<!-- Goal tracking -->
<div id="goal-strip">
  <div class="goal-card status-idle" id="goal-status-card">
    <div class="gs-icon" id="gs-icon">&#x23F8;</div>
    <div class="gs-text">
      <h3>Goal Status</h3>
      <div class="gsv" id="gs-val">IDLE</div>
    </div>
  </div>
  <div class="goal-stat">
    <h3>Goals Completed</h3>
    <div class="gsv" id="goals-completed">0</div>
    <div class="gsu">total successes</div>
  </div>
  <div class="goal-stat">
    <h3>Goals Failed</h3>
    <div class="gsv" id="goals-failed" style="color:#e74c3c">0</div>
    <div class="gsu">aborted / preempted</div>
  </div>
</div>

<!-- Metric cards -->
<div class="grid" id="cards"></div>

<!-- Summary -->
<div class="section">
  <h2>&#x1F4CA; Multi-Environment Summary</h2>
  <div id="summary-table"><span style="color:#888;font-size:0.85rem">Loading...</span></div>
</div>

<!-- Alerts -->
<div class="section alerts">
  <h2>&#x26A0; Recent Alerts</h2>
  <div id="alert-list"></div>
</div>

<script>
const WORLD_CFG = {
  obstacles: { icon: '&#x1F6A7;', color: '#f0a500' },
  house:     { icon: '&#x1F3E0;', color: '#2ecc71' },
  narrow:    { icon: '&#x1F500;', color: '#9b59b6' },
};
const DEFAULT_WORLD = { icon: '&#x1F30D;', color: '#4a90d9' };

const GOAL_CFG = {
  IDLE:      { cls: 'status-idle',      icon: '&#x23F8;', pulse: false },
  ACTIVE:    { cls: 'status-active',    icon: '&#x25B6;', pulse: true  },
  SUCCEEDED: { cls: 'status-succeeded', icon: '&#x2713;', pulse: false },
  FAILED:    { cls: 'status-failed',    icon: '&#x2717;', pulse: false },
};

async function refresh() {
  try {
    const r = await fetch('/api/metrics');
    const d = await r.json();
    document.getElementById('last-update').textContent = 'Last update: ' + new Date().toLocaleTimeString();

    // World banner
    const world = (d.world || 'unknown').toLowerCase();
    const wCfg = WORLD_CFG[world] || DEFAULT_WORLD;
    document.getElementById('world-icon').innerHTML = wCfg.icon;
    document.getElementById('world-name').textContent = (d.world || '—').toUpperCase();
    document.getElementById('world-accent').style.background = wCfg.color;

    // Goal tracking
    const status = d.goal_status || 'IDLE';
    const gCfg = GOAL_CFG[status] || GOAL_CFG.IDLE;
    document.getElementById('goal-status-card').className = `goal-card ${gCfg.cls}`;
    document.getElementById('gs-icon').innerHTML  = gCfg.icon;
    document.getElementById('gs-icon').className  = gCfg.pulse ? 'gs-icon pulsing' : 'gs-icon';
    document.getElementById('gs-val').textContent = status;
    document.getElementById('goals-completed').textContent = d.goals_completed ?? 0;
    document.getElementById('goals-failed').textContent    = d.goals_failed    ?? 0;

    // Metric cards
    const cards = [
      { label: 'Execution Time',  value: d.execution_time_s.toFixed(1),        unit: 'seconds' },
      { label: 'Nav Accuracy',    value: d.nav_accuracy_m.toFixed(3),          unit: 'meters',    cls: d.nav_accuracy_m > 0.4 ? 'warn' : '' },
      { label: 'Path Efficiency', value: (d.path_efficiency * 100).toFixed(0), unit: '%',         cls: d.path_efficiency < 0.6 && d.path_efficiency > 0 ? 'warn' : '' },
      { label: 'Recovery Count',  value: d.recovery_count,                     unit: 'this goal', cls: d.recovery_count >= 3 ? 'alert' : '' },
      { label: 'Battery',         value: d.battery_pct.toFixed(1),             unit: '%',         cls: d.battery_pct < 20 ? 'alert' : d.battery_pct < 40 ? 'warn' : '' },
      { label: 'Total Distance',  value: d.total_distance_m.toFixed(2),        unit: 'meters' },
    ];
    document.getElementById('cards').innerHTML = cards.map(c =>
      `<div class="card ${c.cls||''}"><h2>${c.label}</h2><div class="value">${c.value}</div><div class="unit">${c.unit}</div></div>`
    ).join('');

    const ar = await fetch('/api/alerts');
    const al = await ar.json();
    document.getElementById('alert-list').innerHTML = al.length
      ? al.slice(-10).reverse().map(a => {
          const t = a.ts ? new Date(a.ts * 1000).toLocaleTimeString() : '';
          return `<div class="alert-item">[${t}] ${a.msg}</div>`;
        }).join('')
      : '<div style="color:#888;font-size:0.85rem">No alerts</div>';

  } catch(e) { document.getElementById('last-update').textContent = 'Connection error'; }
}

async function loadSummary() {
  try {
    const r = await fetch('/api/summary');
    const s = await r.json();
    if (s.error || !s.worlds || Object.keys(s.worlds).length === 0) {
      document.getElementById('summary-table').innerHTML =
        '<span style="color:#888;font-size:0.85rem">No CSV data yet — run at least one patrol.</span>';
      return;
    }
    const worlds = Object.keys(s.worlds).sort();
    const rows = worlds.map(w => {
      const d = s.worlds[w];
      const wCfg = WORLD_CFG[w] || DEFAULT_WORLD;
      return `<tr>
        <td><span style="margin-right:0.5rem">${wCfg.icon}</span><strong>${w}</strong></td>
        <td>${d.runs}</td>
        <td>${d.avg_accuracy_m.toFixed(3)} m</td>
        <td>${(d.avg_efficiency * 100).toFixed(0)}%</td>
        <td>${d.avg_recovery_per_goal.toFixed(2)}</td>
        <td>${d.avg_execution_time_s.toFixed(1)} s</td>
      </tr>`;
    }).join('');
    document.getElementById('summary-table').innerHTML = `
      <table>
        <thead><tr>
          <th>World</th><th>Goals Completed</th><th>Avg Accuracy</th>
          <th>Avg Efficiency</th><th>Avg Recoveries/Goal</th><th>Avg Exec Time</th>
        </tr></thead>
        <tbody>${rows}</tbody>
      </table>`;
  } catch(e) {
    document.getElementById('summary-table').innerHTML =
      '<span style="color:#888;font-size:0.85rem">Summary unavailable.</span>';
  }
}

refresh();
setInterval(refresh, 2000);
loadSummary();
setInterval(loadSummary, 30000);
</script>
</body>
</html>
"""


class WebDashboardNode(LifecycleNode):
    def __init__(self):
        super().__init__('web_dashboard')

        self.declare_parameter('port', int(os.environ.get('DASHBOARD_PORT', 8080)))
        self.declare_parameter('world', 'unknown')
        self.declare_parameter('data_dir', '/ros2_ws/data/csv')

        self._latest: dict = {
            'timestamp': 0.0,
            'execution_time_s': 0.0,
            'nav_accuracy_m': 0.0,
            'path_efficiency': 0.0,
            'recovery_count': 0,
            'battery_pct': 100.0,
            'total_distance_m': 0.0,
            'navigation_active': False,
            'goal_status': 'IDLE',
            'goals_completed': 0,
            'goals_failed': 0,
            'world': 'unknown',
        }
        self._alert_log: list = []
        self._last_raw: str = ''
        self._summary_cache = None
        self._summary_cache_time: float = 0.0

    # ── Lifecycle callbacks ────────────────────────────────────────────────

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring web_dashboard')

        best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            String, '/nav_monitor/live_data', self._metrics_cb, qos_profile=best_effort
        )
        self.create_subscription(
            String, '/nav_monitor/alerts', self._alerts_cb, 10
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Activating web_dashboard')
        if _FLASK_OK:
            self._start_flask()
        else:
            self.get_logger().error('Flask not installed; web dashboard disabled')
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    # ── ROS callbacks ─────────────────────────────────────────────────────

    def _metrics_cb(self, msg: String) -> None:
        if msg.data == self._last_raw:
            return
        try:
            self._latest = json.loads(msg.data)
            self._last_raw = msg.data
        except json.JSONDecodeError as e:
            self.get_logger().error(f'metrics JSON parse error: {e}')

    def _alerts_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self._alert_log.append({
                'ts':  data.get('timestamp', 0),
                'msg': data.get('message', ''),
            })
            if len(self._alert_log) > 100:
                self._alert_log.pop(0)
        except json.JSONDecodeError:
            pass

    # ── Flask server ───────────────────────────────────────────────────────

    def _start_flask(self) -> None:
        app = Flask(__name__)
        CORS(app)
        node_ref = self

        @app.route('/')
        def index():
            return render_template_string(_HTML)

        @app.route('/api/metrics')
        def api_metrics():
            return jsonify(node_ref._latest)

        @app.route('/api/alerts')
        def api_alerts():
            return jsonify(node_ref._alert_log)

        @app.route('/api/summary')
        def api_summary():
            import time as _time
            _CACHE_TTL = 25.0
            if (node_ref._summary_cache is not None
                    and _time.time() - node_ref._summary_cache_time < _CACHE_TTL):
                return jsonify(node_ref._summary_cache)

            if not _PANDAS_OK:
                return jsonify({'error': 'pandas not installed', 'worlds': {}})

            data_dir = node_ref.get_parameter('data_dir').value
            files = glob.glob(os.path.join(data_dir, '*.csv'))
            if not files:
                return jsonify({'error': 'no data', 'worlds': {}})

            completed_rows = []
            for f in files:
                try:
                    fdf = pd.read_csv(f)
                    if 'goals_completed' not in fdf.columns:
                        continue
                    goal_events = fdf[fdf['goals_completed'].diff().fillna(0) > 0]
                    completed_rows.append(goal_events)
                except Exception:
                    continue

            if not completed_rows:
                return jsonify({'total_samples': 0, 'worlds': {}})

            df = pd.concat(completed_rows, ignore_index=True)
            if df.empty:
                return jsonify({'total_samples': 0, 'worlds': {}})

            summary = {'total_samples': int(len(df)), 'worlds': {}}
            for world, wdf in df.groupby('world'):
                summary['worlds'][str(world)] = {
                    'runs':                  int(len(wdf)),
                    'avg_accuracy_m':        round(float(wdf['nav_accuracy_m'].mean()),   3),
                    'avg_efficiency':        round(float(wdf['path_efficiency'].mean()),   3),
                    'avg_recovery_per_goal': round(float(wdf['recovery_count'].mean()),    2),
                    'avg_execution_time_s':  round(float(wdf['execution_time_s'].mean()),  1),
                }
            node_ref._summary_cache = summary
            node_ref._summary_cache_time = _time.time()
            return jsonify(summary)

        port = self.get_parameter('port').value
        threading.Thread(
            target=lambda: app.run(host='0.0.0.0', port=port, debug=False, use_reloader=False),
            daemon=True,
        ).start()
        self.get_logger().info(f'Web dashboard running on http://0.0.0.0:{port}')


def main(args=None):
    rclpy.init(args=args)
    node = WebDashboardNode()
    executor = rclpy.executors.MultiThreadedExecutor()
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
