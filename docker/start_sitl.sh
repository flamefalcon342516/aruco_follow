#!/bin/bash
# ArduPilot SITL (JSON backend -> gz ArduPilotPlugin) + MAVProxy relay.
# MAVProxy runs --daemon (no TTY in a container) and outputs on udp:14550.
set -e
cd /aruco_follow
arducopter --model JSON --speedup 1 --slave 0 \
  --defaults /ardupilot/Tools/autotest/default_params/copter.parm,/ardupilot/Tools/autotest/default_params/gazebo-iris.parm,/aruco_follow/gimbal.parm \
  --sim-address=127.0.0.1 -I0 &
AC=$!
# SITL only opens SERIAL0 (tcp 5760) once it's up — wait, then relay
until ss -tln | grep -q 5760; do
  kill -0 $AC 2>/dev/null || { echo "arducopter died"; exit 1; }
  sleep 1
done
exec mavproxy.py --daemon --master tcp:127.0.0.1:5760 --out 127.0.0.1:14550
