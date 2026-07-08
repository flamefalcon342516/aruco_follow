#!/usr/bin/env python3
import argparse
import math
import os
import subprocess
import time

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def set_pose(world, model, x, y):
    req = f'name: "{model}" position {{ x: {x:.3f} y: {y:.3f} z: 0 }}'
    subprocess.run(
        ["gz", "service", "-s", f"/world/{world}/set_pose",
         "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
         "--timeout", "300", "--req", req],
        capture_output=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pattern", choices=["circle", "line"], default="circle")
    ap.add_argument("--speed", type=float, default=0.5, help="m/s")
    ap.add_argument("--radius", type=float, default=5.0,
                    help="circle radius / line half-length (m)")
    ap.add_argument("--cx", type=float, default=4.0, help="pattern center x")
    ap.add_argument("--cy", type=float, default=0.0, help="pattern center y")
    ap.add_argument("--rate", type=float, default=5.0, help="updates per s")
    args = ap.parse_args()

    with open(os.path.join(ROOT, "config.yaml")) as f:
        cfg = yaml.safe_load(f)
    world = cfg["sim"]["world"]
    model = cfg["sim"]["marker_model"]

    print(f"[mov] {args.pattern}, speed {args.speed} m/s, "
          f"radius {args.radius} m — Ctrl+C to stop")
    t0 = time.time()
    try:
        while True:
            s = (time.time() - t0) * args.speed
            if args.pattern == "circle":
                a = s / args.radius
                x = args.cx + args.radius * math.cos(a)
                y = args.cy + args.radius * math.sin(a)
            else:  # line: back and forth along x
                phase = (s / (2 * args.radius)) % 2.0
                off = phase * 2 * args.radius
                if phase > 1.0:
                    off = 4 * args.radius - off
                x = args.cx - args.radius + off
                y = args.cy
            set_pose(world, model, x, y)
            time.sleep(1.0 / args.rate)
    except KeyboardInterrupt:
        print("\n[mov] stopped")


if __name__ == "__main__":
    main()
