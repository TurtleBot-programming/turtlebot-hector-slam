#!/usr/bin/env python3
"""Generate worlds/hector_slam_test.wbt and maps/ground_truth.{pgm,yaml}."""
import os

import numpy as np

from layout import OBSTACLES, START, WALL_HEIGHT, WALLS

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RES = 0.05


def box(name, cx, cy, sx, sy, colour):
    return f"""Solid {{
  translation {cx} {cy} {WALL_HEIGHT / 2}
  children [
    Shape {{
      appearance PBRAppearance {{ baseColor {colour} roughness 1 metalness 0 }}
      geometry Box {{ size {sx} {sy} {WALL_HEIGHT} }}
    }}
  ]
  name "{name}"
  boundingObject Box {{ size {sx} {sy} {WALL_HEIGHT} }}
}}
"""


def world():
    out = ["#VRML_SIM R2025a utf8", "",
           'EXTERNPROTO "../protos/Turtlebot4.proto"', "",
           "WorldInfo {", "  basicTimeStep 16", "  coordinateSystem \"ENU\"", "}",
           "Viewpoint {",
           "  orientation 0 1 0 1.5708",
           "  position 0 0 14",
           "}",
           "Background { skyColor [0.8 0.85 0.9] }",
           "DirectionalLight { direction 0.3 0.4 -1 }",
           "Solid {",
           "  translation 0 0 -0.005",
           "  children [ Shape { appearance PBRAppearance { baseColor 0.75 0.75 0.72 roughness 1 metalness 0 }"
           " geometry Box { size 12 10 0.01 } } ]",
           '  name "floor"',
           "  boundingObject Box { size 12 10 0.01 }",
           "}"]
    for i, w in enumerate(WALLS):
        out.append(box(f"wall_{i}", *w, "0.9 0.9 0.9"))
    for i, o in enumerate(OBSTACLES):
        out.append(box(f"obstacle_{i}", *o, "0.8 0.35 0.2"))
    x, y, yaw = START
    out.append(f"""Turtlebot4 {{
  translation {x} {y} 0.01
  rotation 0 0 1 {yaw}
  controller "<extern>"
}}""")
    return "\n".join(out) + "\n"


def ground_truth():
    x0, y0, x1, y1 = -5.5, -4.5, 5.5, 4.5
    w, h = int((x1 - x0) / RES), int((y1 - y0) / RES)
    img = np.full((h, w), 254, np.uint8)  # free
    for cx, cy, sx, sy in WALLS + OBSTACLES:
        c0 = max(int((cx - sx / 2 - x0) / RES), 0)
        c1 = min(int(np.ceil((cx + sx / 2 - x0) / RES)), w)
        r0 = max(int((cy - sy / 2 - y0) / RES), 0)
        r1 = min(int(np.ceil((cy + sy / 2 - y0) / RES)), h)
        img[r0:r1, c0:c1] = 0
    img = img[::-1]  # image row 0 is the top (max y)
    with open(os.path.join(ROOT, "maps", "ground_truth.pgm"), "wb") as f:
        f.write(f"P5\n{w} {h}\n255\n".encode())
        f.write(img.tobytes())
    with open(os.path.join(ROOT, "maps", "ground_truth.yaml"), "w") as f:
        f.write("image: ground_truth.pgm\nmode: trinary\n"
                f"resolution: {RES}\norigin: [{x0}, {y0}, 0.0]\n"
                "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")


if __name__ == "__main__":
    with open(os.path.join(ROOT, "worlds", "hector_slam_test.wbt"), "w") as f:
        f.write(world())
    ground_truth()
    print("generated world + ground-truth map")
