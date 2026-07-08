#!/usr/bin/env python3
import os
import sys

import cv2
import numpy as np
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    with open(os.path.join(ROOT, "config.yaml")) as f:
        cfg = yaml.safe_load(f)

    dict_name = cfg["target"]["dictionary"]
    marker_id = cfg["target"]["marker_id"]

    try:
        aruco_dict = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, dict_name))
    except AttributeError:
        sys.exit(f"Unknown ArUco dictionary '{dict_name}' "
                 f"(try DICT_4X4_50, DICT_5X5_100, DICT_6X6_250 ...)")

    # 800 px marker on a 1000 px white canvas: the white border (quiet zone)
    # is required for detection.
    marker = cv2.aruco.generateImageMarker(aruco_dict, marker_id, 800)
    canvas = np.full((1000, 1000), 255, dtype="uint8")
    canvas[100:900, 100:900] = marker

    out = os.path.join(ROOT, "models", "aruco_marker",
                       "materials", "textures", "marker.png")
    cv2.imwrite(out, canvas)
    print(f"wrote {out}  ({dict_name} id={marker_id})")


if __name__ == "__main__":
    main()
