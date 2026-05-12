# tbot3_nav_monitor

Real-time navigation performance monitor and adaptive behavior controller for **TurtleBot3 Burger** running on **ROS2 Humble** with **Nav2** in **Gazebo Classic 11**.

The package observes a Nav2 navigation stack from the side, computes performance metrics live, dynamically reconfigures Nav2 parameters when navigation degrades, logs every metric tick to CSV, and exposes a web dashboard outside the container.

> **Submission for the AI4I (Italian Institute of Artificial Intelligence) PhD application.**
> Demo video: **<paste link here>** (5-minute walkthrough of all three worlds, dashboard, adaptive behavior, and CSV logs)

---

## 1. Architecture

```
                        ┌──────────────────────┐
                        │      Nav2 stack      │
                        │ (Gazebo + AMCL)      │
                        └─────┬─────────┬──────┘
            /tf, /odom        │         │  /navigate_to_pose action
                              │         │
                  ┌───────────▼─────────▼──────────┐
                  │     metrics_collector          │   lifecycle node
                  │  goal start/end + TF tracking  │
                  └───┬───────────┬────────────┬───┘
        live JSON     │           │            │  per-metric topics
                      │           │            │
        ┌─────────────▼─┐  ┌──────▼─────┐  ┌───▼─────────────┐
        │ web_dashboard │  │ data_logger│  │adaptive_behavior│
        │  Flask :8080  │  │   CSV      │  │ tunes Nav2 via  │
        │  charts/table │  │  + alerts  │  │  SetParameters  │
        └───────────────┘  └────────────┘  └─────────────────┘
                                                   ▲
                                                   │ /goal_pose (RViz2)
                                ┌──────────────────┴────┐
                                │   waypoint_patrol     │
                                │  cycles per-world     │
                                │  goals, supports      │
                                │  manual interruption  │
                                └───────────────────────┘
```

Five Python nodes, three of them lifecycle nodes:

| Node | Type | Purpose |
|---|---|---|
| `metrics_collector` | LifecycleNode | Observes Nav2 action status + TF, computes execution_time / nav_accuracy / path_efficiency / recovery_count / battery_pct |
| `adaptive_behavior` | LifecycleNode | Reads rolling 5-sample windows, dynamically calls `SetParameters` on `controller_server` and `local_costmap` |
| `data_logger` | Node | Writes timestamped CSV rows; publishes alerts when configurable thresholds are breached |
| `web_dashboard` | LifecycleNode | Flask server on port 8080: live metric cards, Chart.js trend lines, multi-environment summary table from CSV history |
| `waypoint_patrol` | Node | Cycles per-world waypoints via `NavigateToPose`; pauses for manual `/goal_pose` from RViz2 and resumes after completion |

All inter-node communication uses ROS2 topics (`/nav_monitor/*`). The monitor subsystem never blocks Nav2 — it consumes published state, never preempts it.

---

## 2. Prerequisites

| Platform | Required |
|---|---|
| **All** | Docker Desktop (or Docker Engine + Compose v2) |
| **Linux** | Native X11 (already present); the Linux-specific compose override mounts `/tmp/.X11-unix` |
| **macOS** | Nothing — Gazebo/RViz2 run inside a VNC container, open `http://localhost:6080` in any browser |
| **Windows** | [VcXsrv](https://sourceforge.net/projects/vcxsrv/) — install it and `run_windows.ps1` will start it automatically with the correct flags |

No ROS2 installation is required on the host. Everything runs inside the container; the dashboard is reachable from the host browser.

---

## 3. Quick start

### Linux

```bash
# AMCL demo (uses pre-built map, runs full monitoring stack)
WORLD=obstacles MODE=amcl ./scripts/run_compose.sh
WORLD=house     MODE=amcl ./scripts/run_compose.sh
WORLD=narrow    MODE=amcl ./scripts/run_compose.sh
```

The script auto-detects Linux and mounts the host X11 socket.

### macOS

```bash
WORLD=obstacles MODE=amcl ./scripts/run_compose.sh
```

The script detects macOS and uses `docker-compose.mac.yml`, which starts an in-container virtual framebuffer (Xvfb), VNC server (x11vnc), and noVNC web proxy.
No XQuartz or any X server is needed on the host.

Once running, open **http://localhost:6080** in any browser to see Gazebo and RViz2.
Use the Openbox window manager (right-click the desktop for a menu) to move and resize windows.

### Windows (PowerShell)

```powershell
$env:WORLD = "obstacles"; $env:MODE = "amcl"; .\scripts\run_windows.ps1
```

### Open the dashboard

Once Nav2 reports ready (~30 s after launch):

```
http://localhost:8080        ← metrics dashboard (all platforms)
http://localhost:6080        ← Gazebo/RViz2 desktop (macOS only)
```

The patrol begins automatically and goals start appearing on the dashboard.

---

## 4. Mapping a new world

```bash
# 1. Run in SLAM mode (default)
WORLD=narrow ./scripts/run_compose.sh

# 2. In a second terminal, drive the robot
docker compose exec sim ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 3. When the map looks complete, save it
# Linux:
docker compose -f docker-compose.yml -f docker-compose.linux.yml exec sim \
  bash /ros2_ws/scripts/save_map.sh narrow
# macOS:
docker compose -f docker-compose.yml -f docker-compose.mac.yml exec sim \
  bash /ros2_ws/scripts/save_map.sh narrow
# Windows:
docker exec tbot3_nav_monitor-sim-1 bash /ros2_ws/scripts/save_map.sh narrow

# 4. Ctrl-C, then relaunch with MODE=amcl to use the new map
```

---

## 5. Adaptive behavior rules

Three independent rules, each with hysteresis to prevent oscillation. Rolling window: 5 samples at 0.5 Hz.

| Rule | Trigger | Action | Restore |
|---|---|---|---|
| **1 — Slow down** | avg recovery ≥ 3 | `desired_linear_vel: 0.20 → 0.10`, `rotate_to_heading_angular_vel: 1.8 → 0.9` | avg recovery < 1.5 **and** efficiency healthy |
| **2 — Relax tolerance** | avg accuracy > 0.40 m | `xy_goal_tolerance: 0.15 → 0.35` | avg accuracy < 0.20 m |
| **3 — Conservative mode** | avg efficiency < 0.60 | `cost_scaling_factor: 3.0 → 8.0` (steeper inflation gradient → planner prefers corridor centres), velocity reduced as in Rule 1 | avg efficiency > 0.85 |

Rule 1's restore is gated on Rule 3 — if efficiency is still poor, the velocity reduction is held even when recoveries normalise. This prevents the two rules from ping-ponging each tick.

Reconfiguration uses the standard ROS2 `SetParameters` service against `/controller_server` and `/local_costmap/local_costmap` — Nav2 applies the new values without restart.

---

## 6. Multi-environment testing

Three Gazebo worlds exercise progressively harder navigation challenges:

| World | Description | Source |
|---|---|---|
| `obstacles` | Open arena with scattered cylinders | `turtlebot3_world.world` |
| `house`     | Multi-room domestic environment | `turtlebot3_house.world` |
| `narrow`    | Custom corridors with sub-1 m passages | `worlds/narrow_passages.world` |

### Results

Live results are computed automatically by the dashboard's `/api/summary` endpoint, which detects goal-completion rows (`goals_completed` increments) across every CSV in `data/csv/` and aggregates per world. Indicative numbers from a sample run:

| World | Goals completed | Avg accuracy | Avg efficiency | Avg recoveries / goal | Avg execution time |
|---|---|---|---|---|---|
| `obstacles` | 39 | 0.173 m | 89% | 0.26 | 14.7 s |
| `house`     | 12 | 0.099 m | 64% | 0.08 | 46.5 s |
| `narrow`    |  9 | 0.206 m | 74% | 3.56 | 38.9 s |

**Analysis.** `obstacles` is the easiest — short distances, open space, near-optimal paths. `house` has lower efficiency because corridors and door frames force longer routes than straight-line distance, but stable AMCL gives the best accuracy. `narrow` produces by far the most recoveries — every traverse of a sub-1 m corridor exercises the adaptive `cost_scaling` rule, which kicks in within 5 ticks of efficiency dropping. Without adaptation the narrow world fails to traverse Corridor 2.

---

## 7. Web dashboard

Accessible at `http://localhost:8080` (port mapped from container).

- **World banner** — large per-world identifier with colour-coded accent
- **Goal status strip** — IDLE / ACTIVE (pulsing) / SUCCEEDED / FAILED + running counts
- **Live metric cards** — execution time, accuracy, efficiency, recovery count, battery, total distance, with amber/red threshold colouring
- **Trend charts** — last 60 s of accuracy, efficiency, recovery (Chart.js, client-side rolling buffer)
- **Multi-environment summary** — table populated from `/api/summary`, refreshes every 30 s, served from a 25 s pandas cache
- **Recent alerts** — last 10 alerts; timestamps formatted in the browser's local timezone

Three JSON endpoints: `/api/metrics`, `/api/alerts`, `/api/summary`.

---

## 8. Manual goal interruption

Send a `/goal_pose` from RViz2 ("2D Goal Pose" tool) at any time. The patrol pauses, the robot navigates to the manual goal, monitoring tracks it the same as any patrol goal, and the patrol resumes from the next waypoint. Implementation handles four race conditions:
- patrol cancellation passes through `STATUS_CANCELING` before reaching `STATUS_CANCELED`
- the manual goal may be `EXECUTING` in the same `GoalStatusArray` message that reports the patrol cancellation
- Nav2 may emit `STATUS_ABORTED` for the old goal in the same tick that the next patrol goal is accepted; `waypoint_patrol` publishes the outgoing goal's UUID to `/nav_monitor/preempted_goal_id` before calling `send_goal_async`, so `metrics_collector` can classify it as preempted rather than a genuine failure
- 120 s timeout fallback if a manual goal never produces a terminal status

---

## 9. Testing

```bash
cd src/tbot3_nav_monitor
python3 -m pytest test/ -v
```

68 unit tests (no live ROS2 required — `conftest.py` provides class stubs):

- **`test_adaptive_behavior.py`** (22) — all three rules in isolation and combined, restoration paths, threshold oscillation, partial/empty windows, ping-pong regression
- **`test_metrics_collector.py`** (29) — `compute_accuracy` / `compute_efficiency` purity, None-pose handling, zero-distance edge case, UUID-based preemption classification, ABORTED vs CANCELED branching
- **`test_data_logger.py`** (11) — CSV writing, malformed JSON handling, all four alert thresholds at boundary conditions, alert deduplication (fires once, clears on recovery, re-fires on re-trigger)
- **`test_web_dashboard.py`** (6) — Flask endpoint stubs, alert log thread-safety, `/api/metrics` and `/api/alerts` response shape

---

## 10. Repository layout

```
tbot3_nav_monitor/
├── Dockerfile                  # multi-stage: builder (colcon) + lean runtime
├── docker-compose.yml          # cross-platform: bridge net + ipc:host for DDS
├── docker-compose.linux.yml    # Linux X11 socket overlay
├── docker-compose.mac.yml      # macOS VNC overlay (Xvfb + x11vnc + noVNC)
├── scripts/
│   ├── run_compose.sh          # Linux/macOS launcher (OS-detect)
│   ├── run_windows.ps1         # Windows PowerShell launcher (auto-starts VcXsrv)
│   ├── start_vnc.sh            # macOS: starts Xvfb + x11vnc + noVNC inside container
│   ├── save_map.sh             # SLAM map serialisation
│   └── ros_entrypoint.sh       # container entrypoint
├── src/tbot3_nav_monitor/
│   ├── package.xml
│   ├── setup.py
│   ├── launch/                 # 5 launch files (per-world sim + monitor + full_stack)
│   ├── config/                 # nav2_params*.yaml + nav_monitor.rviz
│   ├── tbot3_nav_monitor/      # 5 nodes
│   └── test/                   # 68 unit tests + conftest.py
├── worlds/                     # narrow_passages.world (custom)
├── maps/                       # saved AMCL maps for all three worlds
├── data/csv/                   # runtime-generated metric logs
└── README.md
```

---

## 11. Docker Hub

```bash
docker pull <DOCKERHUB_USER>/tbot3_nav_monitor:latest
WORLD=obstacles MODE=amcl docker compose up
```

Image tag: `<DOCKERHUB_USER>/tbot3_nav_monitor:latest`.

---

## 12. Implementation notes

- **Lifecycle nodes** are used wherever the node has external dependencies (Nav2, TF, action server, Flask) — `on_configure` creates subscriptions, `on_activate` starts timers, ensuring clean startup ordering.
- **Path efficiency** is computed as `‖target − start‖ / actual_path_length`, snapshotting `target` and `start` at goal-start so a preempting manual goal cannot corrupt the metric of the goal it preempted.
- **Battery** is a fictional metric drained linearly with distance travelled (0.05 % per metre) — included to satisfy the assignment's required metric set; not derived from any real sensor.
- **CSV logging** writes 2 Hz time-series; the `/api/summary` endpoint filters this down to one row per completed goal for analysis (detecting `goals_completed` increments via pandas `diff().fillna(0) > 0`).
- **Cross-container DDS** uses `ipc: host` + a shared `/dev/shm` mount so FastDDS shared-memory transport works between the `sim` and `monitor` services on a bridge network — required for ROS2 actions to discover across containers.
- **Manual goal interruption** is purely additive: `waypoint_patrol` subscribes to `/goal_pose` and `/navigate_to_pose/_action/status` and never cancels Nav2 itself — it only refrains from sending the next patrol goal until the action server is idle again.

---

## License

Apache-2.0. See `src/tbot3_nav_monitor/package.xml`.
