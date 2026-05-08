#!/bin/bash
# Save the current SLAM map for a given world.
# Usage (inside container): ./scripts/save_map.sh <world>
#   world: obstacles | house | narrow  (default: obstacles)
#
# Maps are saved to /ros2_ws/src/tbot3_nav_monitor/maps/ so they are
# committed with the package and available for AMCL on next launch.

WORLD=${1:-obstacles}
MAP_NAME="${WORLD}_map"
MAP_DIR="/ros2_ws/maps"

mkdir -p "$MAP_DIR"

echo "[save_map] Saving map for world '${WORLD}' → ${MAP_DIR}/${MAP_NAME}..."
ros2 run nav2_map_server map_saver_cli \
    -f "${MAP_DIR}/${MAP_NAME}" \
    --ros-args -p save_map_timeout:=10.0

if [ $? -eq 0 ]; then
    echo "[save_map] Done. Files: ${MAP_NAME}.yaml + ${MAP_NAME}.pgm"
    echo "[save_map] Relaunch with: WORLD=${WORLD} MODE=amcl docker compose up"
else
    echo "[save_map] Failed — is SLAM running and the map built?"
fi
