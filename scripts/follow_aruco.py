#!/usr/bin/env python3

import argparse
import math
import os
import re
import subprocess
import threading
import time

import cv2
import numpy as np
import yaml

from gz.msgs10.image_pb2 import Image
from gz.msgs10.double_pb2 import Double
from gz.transport13 import Node as GzNode

from pymavlink import mavutil

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


# ───────────────────────────── camera feed ────────────────────────────────
class CameraFeed:
    """Latest-frame buffer fed by a gz-transport image subscription."""

    def __init__(self, topic, rotate_180=False):
        self._lock = threading.Lock()
        self._frame = None
        self._stamp = 0.0
        self._rot = rotate_180
        self._node = GzNode()
        if not self._node.subscribe(Image, topic, self._on_image):
            raise RuntimeError(f"could not subscribe to {topic}")

    def _on_image(self, msg):
        # gz camera publishes RGB_INT8
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if self._rot:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        with self._lock:
            self._frame = frame
            self._stamp = time.time()

    def latest(self):
        with self._lock:
            return self._frame, self._stamp


def discover_camera_topic():
    out = subprocess.run(["gz", "topic", "-l"], capture_output=True,
                         text=True, timeout=10).stdout
    topics = [t for t in out.splitlines()
              if re.search(r"/sensor/camera/image$", t)]
    if not topics:
        raise RuntimeError(
            "no */sensor/camera/image topic found — is the sim running?")
    return topics[0]


# ───────────────────────────── PD controller ──────────────────────────────
class PD:
    def __init__(self, kp, kd):
        self.kp, self.kd = kp, kd
        self._prev_err = None
        self._prev_t = None

    def reset(self):
        self._prev_err = None
        self._prev_t = None

    def update(self, err, t):
        d = 0.0
        if self._prev_err is not None and t > self._prev_t:
            d = (err - self._prev_err) / (t - self._prev_t)
        self._prev_err, self._prev_t = err, t
        return self.kp * err + self.kd * d


def search_velocity(t, speed, leg0):
    """Expanding-square velocities for a search at time t since search start.

    Legs cycle fwd/right/back/left in the body frame; every two legs the
    leg duration grows by leg0, sweeping an outward square spiral."""
    dirs = [(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)]
    k = 0
    acc = 0.0
    while True:
        dur = leg0 * (1 + k // 2)
        if t < acc + dur:
            dx, dy = dirs[k % 4]
            return dx * speed, dy * speed
        acc += dur
        k += 1


# ───────────────────────────── MAVLink drone ──────────────────────────────
class Drone:
    def __init__(self, connection):
        print(f"[mav] connecting {connection} ...")
        self.mav = mavutil.mavlink_connection(connection)
        self.mav.wait_heartbeat()
        print(f"[mav] heartbeat from sys {self.mav.target_system}")
        # raw link (no MAVProxy): must ask for telemetry streams ourselves
        self.mav.mav.request_data_stream_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)

    def set_mode(self, name):
        self.mav.set_mode_apm(self.mav.mode_mapping()[name])
        while True:
            hb = self.mav.recv_match(type="HEARTBEAT", blocking=True,
                                     timeout=5)
            if hb and mavutil.mode_string_v10(hb) == name:
                print(f"[mav] mode {name}")
                return

    def _pump(self, timeout=0.5):
        """Drain one message, printing any STATUSTEXT (prearm reasons etc.)."""
        msg = self.mav.recv_match(type=["STATUSTEXT", "HEARTBEAT"],
                                  blocking=True, timeout=timeout)
        if msg and msg.get_type() == "STATUSTEXT":
            print(f"[fcu] {msg.text}")
        return msg

    def arm(self):
        # EKF needs time to converge after boot — retry until prearm passes
        print("[mav] arming (waits for EKF/prearm checks) ...")
        while True:
            self.mav.arducopter_arm()
            t0 = time.time()
            while time.time() - t0 < 3.0:
                self._pump()
                if self.mav.motors_armed():
                    print("[mav] armed")
                    return

    def takeoff(self, alt):
        # retry until ArduPilot ACKs (it rejects takeoff while EKF settles)
        print(f"[mav] takeoff to {alt} m")
        while True:
            if not self.mav.motors_armed():
                print("[mav] disarmed while waiting — re-arming")
                self.arm()
            self.mav.mav.command_long_send(
                self.mav.target_system, self.mav.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                0, 0, 0, 0, 0, 0, alt)
            ack = self.mav.recv_match(type="COMMAND_ACK", blocking=True,
                                      timeout=3)
            if (ack and ack.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
                    and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED):
                print("[mav] takeoff accepted")
                break
            if ack:
                print(f"[mav] takeoff rejected (result {ack.result}) — retry")
            self._pump(1.0)
        while True:
            msg = self.mav.recv_match(type="GLOBAL_POSITION_INT",
                                      blocking=True, timeout=5)
            if msg and msg.relative_alt / 1000.0 > alt * 0.95:
                print("[mav] takeoff altitude reached")
                return

    def point_gimbal_down(self):
        """Mount pitch -90 deg via MAVLink (needs gimbal.parm loaded:
        SERVO10 = Mount1 Pitch — plugin channel 9 is SERVO10, not SERVO9)."""
        self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL, 0,
            -90, 0, 0, 0, 0, 0,
            mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING)
        ack = self.mav.recv_match(type="COMMAND_ACK", blocking=True,
                                  timeout=2)
        if (ack and ack.command == mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL
                and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED):
            print("[mav] gimbal pitch -90 (down) accepted")
        else:
            print("[mav] warning: gimbal command not ACKed — relying on "
                  "MNT1 neutral-down boot attitude")

    def send_body_velocity(self, vx, vy, vz):
        """Body-frame velocity, yaw locked (yaw_rate = 0)."""
        type_mask = (mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
                     | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
                     | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
                     | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
                     | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
                     | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
                     | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE)
        self.mav.mav.set_position_target_local_ned_send(
            0, self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED, type_mask,
            0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0)


# ─────────────────────────────── main ─────────────────────────────────────
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-window", action="store_true",
                    help="headless: no OpenCV preview window")
    ap.add_argument("--no-fly", action="store_true",
                    help="vision only: no MAVLink, just show detections")
    ap.add_argument("--connection", default=None,
                    help="override mavlink.connection from config.yaml "
                         "(e.g. tcp:127.0.0.1:5760)")
    args = ap.parse_args()

    with open(os.path.join(ROOT, "config.yaml")) as f:
        cfg = yaml.safe_load(f)

    tgt, cam, ctl = cfg["target"], cfg["camera"], cfg["control"]

    aruco_dict = cv2.aruco.getPredefinedDictionary(
        getattr(cv2.aruco, tgt["dictionary"]))
    detector = cv2.aruco.ArucoDetector(aruco_dict,
                                       cv2.aruco.DetectorParameters())

    # normalized-error scaling: pixels -> tan(angle from optical axis)
    fx = (cam["width"] / 2) / math.tan(cam["hfov_rad"] / 2)
    cx, cy = cam["width"] / 2, cam["height"] / 2

    topic = cfg["sim"]["camera_topic"] or discover_camera_topic()
    print(f"[gz ] camera topic: {topic}")
    feed = CameraFeed(topic, rotate_180=cfg["sim"].get("rotate_180", False))

    # gz-side gimbal pitch pub — only effective when SITL is NOT driving the
    # mount (e.g. --no-fly); with SITL, point_gimbal_down() does the job.
    gz_node = GzNode()
    pitch_pub = gz_node.advertise(cfg["sim"]["gimbal_pitch_topic"], Double)
    pitch_msg = Double()
    pitch_msg.data = cfg["sim"]["gimbal_pitch_rad"]

    drone = None
    if not args.no_fly:
        m = cfg["mavlink"]
        drone = Drone(args.connection or m["connection"])
        drone.point_gimbal_down()
        drone.set_mode("GUIDED")
        drone.arm()
        drone.takeoff(m["takeoff_alt_m"])
        drone.point_gimbal_down()

    # PD per axis. Image error is normalized (tan of off-axis angle), so at
    # altitude h the metric offset is ~ err * h — gains are altitude-robust
    # enough for a sim demo.
    # Mapping measured in-sim (probe flight, camera down + rotate_180):
    #   body +vx (nose fwd)   -> marker moves image UP    (dv = -101 px)
    #   marker right of body  -> appears image LEFT
    # therefore:
    #   image +y (down)  -> body +x (forward)
    #   image +x (right) -> body -y (left)
    pd_x = PD(ctl["kp"], ctl["kd"])   # body vx from image +y error
    pd_y = PD(ctl["kp"], ctl["kd"])   # body vy from image -x error

    period = 1.0 / ctl["rate_hz"]
    last_seen = time.time()   # start the lost-clock now: search if the
    last_stamp = 0.0          # marker is never seen after takeoff
    last_snapshot = 0.0
    tracking = False
    search_t0 = None

    print("[run] following — Ctrl+C to RTL and exit")
    try:
        while True:
            t0 = time.time()
            # re-assert gimbal pitch (gz pub only helps in --no-fly mode)
            if drone is None:
                pitch_pub.publish(pitch_msg)

            frame, stamp = feed.latest()
            if frame is None or stamp == last_stamp:
                time.sleep(0.01)
                continue
            last_stamp = stamp

            corners, ids, _ = detector.detectMarkers(frame)
            found = (ids is not None
                     and tgt["marker_id"] in ids.flatten().tolist())

            now = time.time()
            vx = vy = 0.0
            if found:
                i = ids.flatten().tolist().index(tgt["marker_id"])
                c = corners[i][0]                 # 4x2 marker corners
                u, v = c.mean(axis=0)             # marker center (px)
                err_x = (u - cx) / fx             # + means target is right
                err_y = (v - cy) / fx             # + means target is behind
                if abs(err_x) < ctl["deadband"]:
                    err_x = 0.0
                if abs(err_y) < ctl["deadband"]:
                    err_y = 0.0

                vx = pd_x.update(err_y, now)      # target below center -> fwd
                vy = -pd_y.update(err_x, now)     # target right in img -> left
                vmax = ctl["max_vel_ms"]
                vx = max(-vmax, min(vmax, vx))
                vy = max(-vmax, min(vmax, vy))

                if not tracking:
                    tracking = True
                    search_t0 = None
                    if now - last_snapshot > ctl["snapshot_cooldown_s"]:
                        last_snapshot = now
                        snap = frame.copy()
                        cv2.aruco.drawDetectedMarkers(snap, corners, ids)
                        path = os.path.join(
                            ROOT, "captures",
                            time.strftime("target_%Y%m%d_%H%M%S.jpg"))
                        cv2.imwrite(path, snap)
                        print(f"[cap] target acquired — snapshot: {path}")
                last_seen = now
            else:
                if tracking and now - last_seen > ctl["lost_timeout_s"]:
                    tracking = False
                    pd_x.reset()
                    pd_y.reset()
                    print("[run] target lost — searching")
                # search: expanding square until reacquired or timed out
                if not tracking and now - last_seen > ctl["lost_timeout_s"]:
                    if search_t0 is None:
                        search_t0 = now
                        print("[run] search pattern started")
                    st = now - search_t0
                    if st < ctl["search_max_s"]:
                        vx, vy = search_velocity(st, ctl["search_vel_ms"],
                                                 ctl["search_leg_s"])
                    elif st - period < ctl["search_max_s"]:
                        print("[run] search timed out — hovering")

            if drone is not None:
                # not found: send zeros (hover) so GUIDED keeps position
                drone.send_body_velocity(vx, vy, 0.0)

            if not args.no_window:
                disp = frame.copy()
                cv2.aruco.drawDetectedMarkers(disp, corners, ids)
                state = "TRACKING" if found else (
                    "COASTING" if tracking else "SEARCHING")
                cv2.putText(disp, f"{state}  vx={vx:+.2f} vy={vy:+.2f}",
                            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (0, 255, 0) if found else (0, 0, 255), 2)
                cv2.drawMarker(disp, (int(cx), int(cy)), (255, 255, 0),
                               cv2.MARKER_CROSS, 20, 1)
                cv2.imshow("aruco_follow  [q to quit]", disp)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            time.sleep(max(0.0, period - (time.time() - t0)))
    except KeyboardInterrupt:
        pass
    finally:
        if drone is not None:
            print("\n[mav] RTL")
            try:
                drone.set_mode("RTL")
            except Exception as e:
                print(f"[mav] RTL failed ({e}) — leaving current mode")
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
