#!/bin/bash
set -e
cd "$(dirname "$0")"

# ardupilot_gazebo plugin (ArduPilotPlugin + gimbal camera plugins)
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/gz_ws/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}
# models: ardupilot_gazebo's (iris, runway, gimbal) + ours (aruco_marker)
export GZ_SIM_RESOURCE_PATH=$HOME/gz_ws/src/ardupilot_gazebo/models:$(pwd)/models:${GZ_SIM_RESOURCE_PATH}
# pin gz-transport to loopback (docker0 binding silently kills topics)
export GZ_IP=127.0.0.1

if [ "$1" = "headless" ]; then
  exec gz sim -r -s --headless-rendering worlds/aruco_follow.sdf
else
  exec gz sim -r worlds/aruco_follow.sdf
fi
