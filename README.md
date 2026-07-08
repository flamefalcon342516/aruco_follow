# aruco_follow — Vision target detection + follow (Gazebo + ArduPilot SITL)

An iris quadcopter with a downward gimbal camera detects a **user-configurable
ArUco marker** and follows it autonomously. On (re)acquiring the target it
saves an annotated snapshot to `captures/` (the "picture to the operator"
hook). Maneuvering is a **PD controller** on the marker's image-space error,
commanded as GUIDED body-frame velocities over MAVLink.

```
gz camera ─► OpenCV ArUco detect ─► PD (image error) ─► SET_POSITION_TARGET
   ▲                                                        (pymavlink)
   │                                                            ▼
Gazebo iris ◄── ArduPilotPlugin (JSON) ◄──────────── ArduPilot SITL
```

## Run (3 terminals)

```bash
cd ~/aruco_follow

# 1 — Gazebo (iris + gimbal + marker)
./run_sim.sh                      # or: ./run_sim.sh headless

# 2 — ArduPilot SITL (wait for "IMU0 is using GPS" / EKF ready)
./run_sitl.sh

# 3 — the follower: arms, takes off, points gimbal down, tracks
python3 scripts/follow_aruco.py
```

Then move the target and watch the drone chase it:

```bash
# 4 — either drag the marker with the Gazebo GUI translate tool, or:
python3 scripts/move_marker.py                    # 5 m circle at 0.5 m/s
python3 scripts/move_marker.py --pattern line --speed 1.0
```

`follow_aruco.py` shows the live camera view with the detection overlay
(`--no-window` for headless, `--no-fly` for vision-only without MAVLink).
Ctrl+C (or `q` in the window) switches to RTL and exits.

## Changing the target (user-configurable)

Edit `config.yaml` → `target:` (`dictionary`, `marker_id`), then:

```bash
python3 scripts/gen_marker.py     # regenerates the marker texture
```

Restart the sim. The follower reads the same config, so detection and the
marker in the world always agree.

## How it works

- **Camera**: `iris_with_gimbal`'s 640×480/10 Hz camera. Frames arrive via
  gz-transport Python bindings (no ROS involved); the topic is
  auto-discovered (`*/sensor/camera/image`). The script publishes
  `/gimbal/cmd_pitch = -1.57` so the camera looks straight down.
- **Detection**: `cv2.aruco.ArucoDetector` with the configured dictionary;
  only the configured `marker_id` is treated as the target.
- **Control mapping** (camera down, yaw locked, measured in-sim): image-y
  error → body `vx` (target toward image bottom = fly forward), image-x
  error → body `-vy` (target toward image right = fly left). Each axis has
  its own PD; output clamped to `max_vel_ms`.
- **Search**: if the marker is unseen for `lost_timeout_s`, the drone flies
  an expanding square (`search_vel_ms`, `search_leg_s`) until it reacquires
  the target; after `search_max_s` it gives up and hovers.
- **SITL wiring**: `sim_vehicle.py -f gazebo-iris` connects ArduPilot's JSON
  backend to the `ArduPilotPlugin` in the iris model (port 9002); MAVProxy
  relays MAVLink on `udp:14550`, which the follower connects to.

## Files

```
config.yaml               target / camera / mavlink / PD parameters
worlds/aruco_follow.sdf   iris_with_gimbal + runway + aruco_marker
models/aruco_marker/      movable marker platform (texture generated)
scripts/follow_aruco.py   camera → detect → PD → MAVLink velocities
scripts/move_marker.py    drives the marker (circle/line) via set_pose
scripts/gen_marker.py     regenerate marker texture from config.yaml
run_sim.sh                gazebo with plugin/model paths set
run_sitl.sh               sim_vehicle.py -v ArduCopter -f gazebo-iris
captures/                 operator snapshots on target acquisition
```

## Requirements (already on this machine)

ArduPilot SITL (`~/ardupilot`), ardupilot_gazebo plugin (`~/gz_ws/build`),
Gazebo Harmonic, OpenCV ≥ 4.7 with aruco, pymavlink, gz python bindings.
