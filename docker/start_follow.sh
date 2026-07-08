#!/bin/bash
# The follower: waits for the sim camera, then arms/takes off/tracks.
# Headless (--no-window); acquisition snapshots land in captures/ (bind-mounted).
set -e
cd /aruco_follow
echo "[dock] waiting for camera topic ..."
until gz topic -l 2>/dev/null | grep -q "sensor/camera/image"; do sleep 2; done
exec python3 -u scripts/follow_aruco.py --no-window "$@"
