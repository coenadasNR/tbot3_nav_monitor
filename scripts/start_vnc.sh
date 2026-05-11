#!/bin/bash
# macOS display shim — starts a virtual X framebuffer + VNC + noVNC inside
# the container, then hands off to the standard ROS entrypoint.
#
# Access Gazebo / RViz2 at http://localhost:6080  (no password)
set -e

echo "[start_vnc] Starting Xvfb on :99..."
Xvfb :99 -screen 0 1280x800x24 -ac &
export DISPLAY=:99
sleep 1

echo "[start_vnc] Starting x11vnc on port 5900..."
x11vnc -display :99 -nopw -listen localhost -xkb -forever -shared -quiet &

echo "[start_vnc] Starting noVNC on port 6080..."
websockify --web /usr/share/novnc 6080 localhost:5900 &

echo "[start_vnc] Display ready -> open http://localhost:6080 in your browser"
exec /ros_entrypoint.sh "$@"
