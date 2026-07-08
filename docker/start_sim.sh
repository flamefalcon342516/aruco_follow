#!/bin/bash
# Gazebo server, headless (watch it from the host with: gz sim -g)
set -e
cd /aruco_follow
exec gz sim -r -s --headless-rendering worlds/aruco_follow.sdf
