#!/bin/bash
# Start ArduPilot SITL wired to the Gazebo iris (JSON backend).
# Run AFTER run_sim.sh. MAVProxy outputs on udp:14550 (used by follow_aruco).
set -e
# -f gazebo-iris: frame params; --model JSON: the JSON FDM backend that the
# new gz-sim ArduPilotPlugin speaks (the legacy 'gazebo' model stalls).
# gimbal.parm: mount config so ArduPilot drives the camera gimbal (SERVO9).
cd "$(dirname "$0")"
exec "$HOME/ardupilot/Tools/autotest/sim_vehicle.py" \
  -v ArduCopter -f gazebo-iris --model JSON \
  --add-param-file="$(pwd)/gimbal.parm" --console "$@"
