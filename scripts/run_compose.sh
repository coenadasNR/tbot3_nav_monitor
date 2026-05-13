#!/bin/bash
# Launch TurtleBot3 Nav Monitor — auto-detects Linux and macOS.
# Windows users: use scripts/run_windows.ps1 instead.
#
# Usage:
#   WORLD=obstacles ./scripts/run_compose.sh                                                   ← SLAM mapping
#   WORLD=obstacles MODE=amcl ./scripts/run_compose.sh                                   ← AMCL adaptive (default)
#   WORLD=house MODE=amcl NAV_CONFIG=baseline ./scripts/run_compose.sh                  ← baseline
#   WORLD=house MODE=amcl NAV_CONFIG=params_a_adaptive ./scripts/run_compose.sh         ← custom config + adaptive
#   WORLD=house MODE=amcl NAV_CONFIG=params_a_baseline NAV_ADAPTIVE=false ./scripts/run_compose.sh  ← custom config, no adaptive
#
# Mapping workflow:
#   1. WORLD=<w> ./scripts/run_compose.sh
#   2. docker compose exec sim ros2 run teleop_twist_keyboard teleop_twist_keyboard
#   3. Drive until map is complete, then:
#      Linux: docker compose -f docker-compose.yml -f docker-compose.linux.yml exec sim bash /ros2_ws/scripts/save_map.sh <world>
#      macOS: docker compose -f docker-compose.yml -f docker-compose.mac.yml exec sim bash /ros2_ws/scripts/save_map.sh <world>
#   4. Ctrl-C, then relaunch with MODE=amcl

set -e

# ── Validate inputs ────────────────────────────────────────────────────────
if [ -z "$WORLD" ]; then
  echo "[run_compose] ERROR: WORLD not set."
  echo "Usage: WORLD=obstacles|house|narrow ./scripts/run_compose.sh"
  exit 1
fi

case "$WORLD" in
  obstacles|house|narrow) ;;
  *) echo "[run_compose] ERROR: Unknown world '$WORLD'. Choose: obstacles | house | narrow"; exit 1 ;;
esac

export WORLD
export MODE="${MODE:-slam}"
export DASHBOARD_PORT="${DASHBOARD_PORT:-8080}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export NAV_CONFIG="${NAV_CONFIG:-default}"
export NAV_ADAPTIVE="${NAV_ADAPTIVE:-true}"

# ── OS detection ───────────────────────────────────────────────────────────
OS=$(uname -s)

if [ "$OS" = "Linux" ]; then
  echo "[run_compose] Platform: Linux"
  export DISPLAY="${DISPLAY:-:0}"
  xhost +local:docker 2>/dev/null || true
  COMPOSE_FILES="-f docker-compose.yml -f docker-compose.linux.yml"

elif [ "$OS" = "Darwin" ]; then
  echo "[run_compose] Platform: macOS — using in-container VNC (no XQuartz needed)"
  echo "[run_compose] Gazebo/RViz2 will be available at http://localhost:6080"
  COMPOSE_FILES="-f docker-compose.yml -f docker-compose.mac.yml"

else
  echo "[run_compose] Windows detected — use scripts/run_windows.ps1 instead."
  exit 1
fi

# ── Build & launch ─────────────────────────────────────────────────────────
cleanup() {
  echo ""
  echo "[run_compose] Shutting down..."
  docker compose $COMPOSE_FILES down
}
trap cleanup INT TERM

DOCKER_BUILDKIT=1 docker compose $COMPOSE_FILES build

if [ "$MODE" = "amcl" ]; then
  ADAPTIVE_LABEL=$([ "${NAV_ADAPTIVE}" = "false" ] && echo "adaptive OFF" || echo "adaptive ON")
  echo "[run_compose] Demo mode  — world: $WORLD | config: ${NAV_CONFIG} (${ADAPTIVE_LABEL}) | Dashboard: http://localhost:${DASHBOARD_PORT}"
  docker compose $COMPOSE_FILES up
else
  echo "[run_compose] Mapping mode — world: $WORLD"
  echo "[run_compose] When done, save map with:"
  echo "  docker compose $COMPOSE_FILES exec sim bash /ros2_ws/scripts/save_map.sh $WORLD"
  docker compose $COMPOSE_FILES up sim
fi
