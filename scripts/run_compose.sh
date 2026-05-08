#!/bin/bash
# Launch TurtleBot3 Nav Monitor — auto-detects Linux and macOS.
# Windows users: use scripts/run_windows.ps1 instead.
#
# Usage:
#   WORLD=obstacles ./scripts/run_compose.sh              ← SLAM mapping
#   WORLD=obstacles MODE=amcl ./scripts/run_compose.sh    ← AMCL demo
#
# Mapping workflow:
#   1. WORLD=<w> ./scripts/run_compose.sh
#   2. docker compose exec sim ros2 run teleop_twist_keyboard teleop_twist_keyboard
#   3. Drive until map is complete, then:
#      docker compose -f docker-compose.yml -f docker-compose.linux.yml exec sim \
#        bash /ros2_ws/scripts/save_map.sh <world>
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

# ── OS detection ───────────────────────────────────────────────────────────
OS=$(uname -s)

if [ "$OS" = "Linux" ]; then
  echo "[run_compose] Platform: Linux"
  export DISPLAY="${DISPLAY:-:0}"
  xhost +local:docker 2>/dev/null || true
  COMPOSE_FILES="-f docker-compose.yml -f docker-compose.linux.yml"

elif [ "$OS" = "Darwin" ]; then
  echo "[run_compose] Platform: macOS"
  # Requires XQuartz — install with: brew install --cask xquartz
  if ! pgrep -qx "Xquartz" 2>/dev/null; then
    echo "[run_compose] WARNING: XQuartz does not appear to be running."
    echo "             Install: brew install --cask xquartz"
    echo "             Then open XQuartz, go to Preferences → Security, enable 'Allow connections from network clients'"
  fi
  xhost + 127.0.0.1 2>/dev/null || true
  export DISPLAY="host.docker.internal:0"
  COMPOSE_FILES="-f docker-compose.yml"

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

docker compose $COMPOSE_FILES build

if [ "$MODE" = "amcl" ]; then
  echo "[run_compose] Demo mode  — world: $WORLD | Dashboard: http://localhost:${DASHBOARD_PORT}"
  docker compose $COMPOSE_FILES up
else
  echo "[run_compose] Mapping mode — world: $WORLD"
  echo "[run_compose] When done, save map with:"
  echo "  docker compose $COMPOSE_FILES exec sim bash /ros2_ws/scripts/save_map.sh $WORLD"
  docker compose $COMPOSE_FILES up sim
fi
