#!/usr/bin/env python3
"""Generate the two-floor test building from layout_2floor.py:

  worlds/two_floor.wbt
  maps/two_floor_f1.{pgm,yaml}, maps/two_floor_f2.{pgm,yaml}   (world frame)
  ../tb4_hector_slam/config/two_floor.yaml   (floor manager + ramp transitions)
"""
import math
import os

import numpy as np

import layout_2floor as L

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
PKG_CONFIG = os.path.join(ROOT, '..', 'tb4_hector_slam', 'config', 'two_floor.yaml')
RES = 0.05
LIDAR_Z = 0.2          # scan plane height above the floor the robot is on
ENTRY_GAP = 0.6        # ramp entry pose distance from the ramp end
RAMP_T = 0.1           # thickness of the ramp collision boxes
RAMP_OVERLAP = 0.01    # collision boxes overlap at the profile joints
OUTER_H = L.FLOOR_HEIGHT + L.WALL2_H

WHITE, BLUE, ORANGE = '0.92 0.92 0.92', '0.72 0.82 0.92', '0.95 0.6 0.2'
RUST, YELLOW, SLAB = '0.8 0.35 0.2', '0.95 0.8 0.3', '0.6 0.6 0.62'


def seg_box(seg):
    """Axis-aligned wall segment -> (cx, cy, sx, sy)."""
    (x0, y0), (x1, y1) = seg
    if abs(y1 - y0) < 1e-9:
        return (x0 + x1) / 2, y0, abs(x1 - x0) + L.WALL_T, L.WALL_T
    return x0, (y0 + y1) / 2, L.WALL_T, abs(y1 - y0) + L.WALL_T


def solid(name, cx, cy, z0, sx, sy, sz, colour, transparency=0.0):
    t = f' transparency {transparency}' if transparency else ''
    return f"""Solid {{
  translation {cx:.3f} {cy:.3f} {z0 + sz / 2:.3f}
  children [
    Shape {{
      appearance PBRAppearance {{ baseColor {colour} roughness 1 metalness 0{t} }}
      geometry Box {{ size {sx:.3f} {sy:.3f} {sz:.3f} }}
    }}
  ]
  name "{name}"
  boundingObject Box {{ size {sx:.3f} {sy:.3f} {sz:.3f} }}
}}"""


def ramp(name, r):
    """Solid ramp from the floor up to the slab top, profile from the layout."""
    (bx, by), (dx, dy) = r['bottom'], r['dir']
    nx, ny = -dy, dx                                   # across the ramp
    w = L.RAMP_W / 2
    prof = L.ramp_profile()
    pts = []

    def p(u, v, z):
        pts.append((bx + dx * u + nx * v, by + dy * u + ny * v, z))
        return len(pts) - 1

    top = [(p(u, -w, z), p(u, w, z)) for u, z in prof]       # surface
    base = [(p(u, -w, 0.0), p(u, w, 0.0)) for u, _ in prof]  # footprint
    # Counter-clockwise seen from outside, so collision normals point out.
    faces = []
    for (a0, a1), (b0, b1) in zip(top, top[1:]):
        faces.append((a0, b0, b1, a1))                       # surface strip
    for (a0, a1), (b0, b1), (c0, c1), (d0, d1) in zip(base, base[1:], top, top[1:]):
        if pts[c0][2] > 0:
            faces.append((a0, b0, d0, c0))                   # side -v
            faces.append((a1, c1, d1, b1))                   # side +v
        else:                                                # starts at the floor
            faces.append((a0, b0, d0))
            faces.append((a1, d1, b1))
    faces.append((base[0][0], base[0][1], base[-1][1], base[-1][0]))  # bottom
    faces.append((base[-1][0], base[-1][1], top[-1][1], top[-1][0]))  # end
    coords = ', '.join(f'{x:.3f} {y:.3f} {z:.3f}' for x, y, z in pts)
    index = ', '.join(', '.join(map(str, f)) + ', -1' for f in faces)
    geom = (f'IndexedFaceSet {{ coord Coordinate {{ point [ {coords} ] }} '
            f'coordIndex [ {index} ] }}')

    # Collision: one tilted box per profile segment. The wheels are cylinders,
    # and cylinder-vs-mesh contacts are unreliable in ODE (the robot stalled
    # at the foot of a mesh ramp); box contacts work for every shape.
    yaw = math.atan2(dy, dx)
    boxes = []
    n = len(prof) - 1
    for i, ((u0, z0), (u1, z1)) in enumerate(zip(prof, prof[1:])):
        th = math.atan2(z1 - z0, u1 - u0)
        # overlap neighbouring segments slightly at the inner joints
        u0e = u0 - (RAMP_OVERLAP if i > 0 else 0.0) * math.cos(th)
        u1e = u1 + (RAMP_OVERLAP if i < n - 1 else 0.0) * math.cos(th)
        z0e = z0 - (RAMP_OVERLAP if i > 0 else 0.0) * math.sin(th)
        z1e = z1 + (RAMP_OVERLAP if i < n - 1 else 0.0) * math.sin(th)
        length = math.hypot(u1e - u0e, z1e - z0e)
        um, zm = (u0e + u1e) / 2, (z0e + z1e) / 2
        uc = um + math.sin(th) * RAMP_T / 2       # move down along the normal
        zc = zm - math.cos(th) * RAMP_T / 2
        ax, ay, az, ang = axis_angle(yaw, -th)
        boxes.append(f'Pose {{ translation {bx + dx * uc:.4f} {by + dy * uc:.4f} '
                     f'{zc:.4f} rotation {ax:.6f} {ay:.6f} {az:.6f} {ang:.6f} '
                     f'children [ Box {{ size {length:.4f} {L.RAMP_W:.3f} {RAMP_T} }} ] }}')
    return f"""Solid {{
  children [
    Shape {{
      appearance PBRAppearance {{ baseColor {YELLOW} roughness 1 metalness 0 }}
      geometry {geom}
    }}
  ]
  name "{name}"
  boundingObject Group {{
    children [
      {(chr(10) + '      ').join(boxes)}
    ]
  }}
}}"""


def axis_angle(yaw, pitch):
    """Axis-angle of Rz(yaw) * Ry(pitch), for a Webots rotation field."""
    cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
    r = [[cy * cp, -sy, cy * sp],
         [sy * cp, cy, sy * sp],
         [-sp, 0.0, cp]]
    ang = math.acos(max(-1.0, min(1.0, (r[0][0] + r[1][1] + r[2][2] - 1) / 2)))
    if ang < 1e-9:
        return 0.0, 0.0, 1.0, 0.0
    if abs(ang - math.pi) < 1e-6:
        # 180 deg: axis from the diagonal
        x = math.sqrt(max(0.0, (r[0][0] + 1) / 2))
        y = math.sqrt(max(0.0, (r[1][1] + 1) / 2))
        z = math.sqrt(max(0.0, (r[2][2] + 1) / 2))
        y = math.copysign(y, r[0][1]) if x > 1e-6 else y
        z = math.copysign(z, r[0][2]) if x > 1e-6 else math.copysign(z, r[1][2])
        return x, y, z, ang
    k = 2 * math.sin(ang)
    return ((r[2][1] - r[1][2]) / k, (r[0][2] - r[2][0]) / k,
            (r[1][0] - r[0][1]) / k, ang)


def slab_boxes():
    """Floor-2 slab: the building footprint minus the ramp openings."""
    holes = [L.ramp_rect(r) for r in L.RAMPS.values()]
    xs = sorted({L.X0, L.X1, *[h[0] for h in holes], *[h[1] for h in holes]})
    ys = sorted({L.Y0, L.Y1, *[h[2] for h in holes], *[h[3] for h in holes]})
    boxes = []
    for y0, y1 in zip(ys, ys[1:]):
        run = None
        for x0, x1 in zip(xs, xs[1:]):
            cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
            hole = any(h[0] < cx < h[1] and h[2] < cy < h[3] for h in holes)
            if hole:
                if run:
                    boxes.append((run[0], run[1], y0, y1))
                run = None
            else:
                run = (run[0], x1) if run else (x0, x1)
        if run:
            boxes.append((run[0], run[1], y0, y1))
    return boxes


def world():
    out = ['#VRML_SIM R2025a utf8', '',
           'EXTERNPROTO "../protos/Turtlebot4.proto"', '',
           'WorldInfo {', '  basicTimeStep 16', '  coordinateSystem "ENU"',
           # The Create 3 casters are free-rolling balls; modelled as rigid
           # spheres they would drag (friction 1) and stall at ramp bottoms.
           '  contactProperties [ ContactProperties { material1 "caster" '
           'coulombFriction [ 0.05 ] } ]', '}',
           'Viewpoint {',
           '  orientation -0.5 0.5 0.7071 1.5708',
           '  position 0 -14 22',
           '}',
           'Background { skyColor [0.8 0.85 0.9] }',
           'DirectionalLight { direction 0.3 0.4 -1 }',
           solid('ground', 0, 0, -0.01, L.X1 - L.X0 + 1, L.Y1 - L.Y0 + 1, 0.01,
                 '0.75 0.75 0.72')]
    for i, s in enumerate(L.OUTER):
        cx, cy, sx, sy = seg_box(s)
        out.append(solid(f'outer_{i}', cx, cy, 0, sx, sy, OUTER_H, WHITE))
    for i, s in enumerate(L.WALLS_1):
        cx, cy, sx, sy = seg_box(s)
        out.append(solid(f'f1_wall_{i}', cx, cy, 0, sx, sy, L.WALL1_H, WHITE))
    for i, (cx, cy, sx, sy) in enumerate(L.OBSTACLES_1):
        out.append(solid(f'f1_obstacle_{i}', cx, cy, 0, sx, sy, 0.5, RUST))
    for i, (x0, x1, y0, y1) in enumerate(slab_boxes()):
        out.append(solid(f'slab_{i}', (x0 + x1) / 2, (y0 + y1) / 2,
                         L.FLOOR_HEIGHT - L.SLAB_T, x1 - x0, y1 - y0, L.SLAB_T,
                         SLAB, transparency=0.55))
    for i, s in enumerate(L.WALLS_2):
        cx, cy, sx, sy = seg_box(s)
        out.append(solid(f'f2_wall_{i}', cx, cy, L.FLOOR_HEIGHT, sx, sy,
                         L.WALL2_H, BLUE))
    for i, (cx, cy, sx, sy) in enumerate(L.OBSTACLES_2):
        out.append(solid(f'f2_obstacle_{i}', cx, cy, L.FLOOR_HEIGHT, sx, sy,
                         0.4, RUST))
    for i, s in enumerate(L.RAILINGS):
        cx, cy, sx, sy = seg_box(s)
        out.append(solid(f'railing_{i}', cx, cy, L.FLOOR_HEIGHT, sx, sy,
                         L.RAIL_H, ORANGE))
    for name, r in L.RAMPS.items():
        out.append(ramp(f'ramp_{name}', r))
    x, y, yaw = L.START
    out.append(f"""Turtlebot4 {{
  translation {x} {y} 0.01
  rotation 0 0 1 {yaw}
  controller "<extern>"
}}""")
    return '\n'.join(out) + '\n'


def rasterise(boxes):
    x0, y0 = L.X0 - 0.5, L.Y0 - 0.5
    w, h = int((L.X1 - L.X0 + 1) / RES), int((L.Y1 - L.Y0 + 1) / RES)
    img = np.full((h, w), 254, np.uint8)
    for cx, cy, sx, sy in boxes:
        c0 = max(int(round((cx - sx / 2 - x0) / RES)), 0)
        c1 = min(int(round((cx + sx / 2 - x0) / RES)), w)
        r0 = max(int(round((cy - sy / 2 - y0) / RES)), 0)
        r1 = min(int(round((cy + sy / 2 - y0) / RES)), h)
        img[r0:r1, c0:c1] = 0
    return img, (x0, y0)


def save_pgm(name, img, origin):
    img = img[::-1]  # image row 0 is the top (max y)
    h, w = img.shape
    with open(os.path.join(ROOT, 'maps', f'{name}.pgm'), 'wb') as f:
        f.write(f'P5\n{w} {h}\n255\n'.encode())
        f.write(img.tobytes())
    with open(os.path.join(ROOT, 'maps', f'{name}.yaml'), 'w') as f:
        f.write(f'image: {name}.pgm\nmode: trinary\n'
                f'resolution: {RES}\norigin: [{origin[0]}, {origin[1]}, 0.0]\n'
                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')


def ground_truth():
    outer = [seg_box(s) for s in L.OUTER]
    # Floor 1: a ramp blocks the scan plane where its surface is above it.
    ramp_parts = []
    for r in L.RAMPS.values():
        (bx, by), (dx, dy) = r['bottom'], r['dir']
        prof = L.ramp_profile()
        d0 = next(u0 + (LIDAR_Z - z0) / (z1 - z0) * (u1 - u0)
                  for (u0, z0), (u1, z1) in zip(prof, prof[1:]) if z1 >= LIDAR_Z)
        u0, u1 = d0, L.RAMP_LEN
        cu = (u0 + u1) / 2
        cx, cy = bx + dx * cu, by + dy * cu
        su = u1 - u0
        ramp_parts.append((cx, cy, su if dx else L.RAMP_W, L.RAMP_W if dx else su))
    f1 = outer + [seg_box(s) for s in L.WALLS_1] + list(L.OBSTACLES_1) + ramp_parts
    f2 = outer + [seg_box(s) for s in L.WALLS_2 + L.RAILINGS] + list(L.OBSTACLES_2)
    for name, boxes in (('two_floor_f1', f1), ('two_floor_f2', f2)):
        save_pgm(name, *rasterise(boxes))


def to_map(x, y, yaw):
    """World pose -> Hector map frame (the robot's spawn pose)."""
    sx, sy, syaw = L.START
    c, s = math.cos(-syaw), math.sin(-syaw)
    dx, dy = x - sx, y - sy
    return c * dx - s * dy, s * dx + c * dy, math.atan2(math.sin(yaw - syaw),
                                                        math.cos(yaw - syaw))


def ramp_config():
    names, frm, to, xs, ys, yaws, lens = [], [], [], [], [], [], []
    for name, r in L.RAMPS.items():
        (bx, by), (dx, dy) = r['bottom'], r['dir']
        up_yaw = math.atan2(dy, dx)
        tx, ty = bx + dx * L.RAMP_LEN, by + dy * L.RAMP_LEN
        for suffix, f, t, ex, ey, yaw in (
                ('up', 1, 2, bx - dx * ENTRY_GAP, by - dy * ENTRY_GAP, up_yaw),
                ('down', 2, 1, tx + dx * ENTRY_GAP, ty + dy * ENTRY_GAP,
                 up_yaw + math.pi)):
            mx, my, myaw = to_map(ex, ey, yaw)
            names.append(f'{name}_{suffix}')
            frm.append(f)
            to.append(t)
            xs.append(round(mx, 3))
            ys.append(round(my, 3))
            yaws.append(round(myaw, 4))
            lens.append(round(L.RAMP_LEN + 2 * ENTRY_GAP, 3))

    def arr(v):
        return '[' + ', '.join(f"'{x}'" if isinstance(x, str) else str(x) for x in v) + ']'

    with open(PKG_CONFIG, 'w') as f:
        f.write(f"""# Generated by webots/tools/generate_world_2floor.py from layout_2floor.py.
# Ramp transitions for the two-floor Webots building, in Hector's map frame
# (the robot's spawn pose). Each entry is one direction of one ramp: the
# entry pose in front of the ramp, the floor it leaves and the floor it
# reaches, and how far to drive from the entry to clear the other end.

floor_manager:
  ros__parameters:
    floor_height: {L.FLOOR_HEIGHT}
    floors: 2

frontier_explorer:
  ros__parameters:
    goal_timeout: 180.0   # far goals in a 20 m building take ~80 s at 0.26 m/s
    ramp_names: {arr(names)}
    ramp_from: {arr(frm)}
    ramp_to: {arr(to)}
    ramp_x: {arr(xs)}
    ramp_y: {arr(ys)}
    ramp_yaw: {arr(yaws)}
    ramp_length: {arr(lens)}
""")


if __name__ == '__main__':
    with open(os.path.join(ROOT, 'worlds', 'two_floor.wbt'), 'w') as f:
        f.write(world())
    ground_truth()
    ramp_config()
    print('generated worlds/two_floor.wbt, maps/two_floor_f{1,2}.*, '
          'tb4_hector_slam/config/two_floor.yaml')
