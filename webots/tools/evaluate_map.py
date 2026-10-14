#!/usr/bin/env python3
"""Compare a saved SLAM map with the Webots ground truth and a pose log.

  python3 evaluate_map.py MAP.yaml [--pose-log pose_error.csv] [--plot out.png]
  python3 evaluate_map.py floor2.yaml --layout layout_2floor \
      --gt ../maps/two_floor_f2.yaml --pose-log pose.csv --floor 2

MAP.yaml is a map_saver_cli map in Hector's map frame, which starts at the
robot spawn pose (START of the --layout module). Each occupied SLAM cell is
moved into the world frame and scored against the ground-truth map (--gt,
default ../maps/ground_truth.yaml):
  precision  occupied SLAM cells within --tol of a true wall/obstacle
  recall     true wall cells (visible from inside the room) within --tol of
             an occupied SLAM cell
  coverage   true free cells inside the room that the SLAM map marks free
The pose log is the CSV written by pose_logger.py (map->base_link vs
/ground_truth/pose); --floor keeps only the rows logged on that floor.
"""
import argparse
import csv
import importlib
import math
import os

import numpy as np
import yaml

START = (0.0, 0.0, 0.0)   # spawn pose, set from --layout

HERE = os.path.dirname(os.path.abspath(__file__))


def load_map(path):
    with open(path) as f:
        meta = yaml.safe_load(f)
    img_path = os.path.join(os.path.dirname(path), meta['image'])
    with open(img_path, 'rb') as f:
        magic = f.readline().strip()
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.split())
        maxval = int(f.readline())
        assert magic == b'P5' and maxval == 255, img_path
        img = np.frombuffer(f.read(w * h), np.uint8).reshape(h, w)
    occ = img < 255 * (1 - meta['occupied_thresh'])
    free = img > 255 * (1 - meta['free_thresh'])
    if meta.get('negate', 0):
        occ, free = free, occ
    # row 0 is the top of the image -> flip so that index [y, x] grows with y
    return np.flipud(occ), np.flipud(free), meta['resolution'], meta['origin']


def cells_to_world(mask, res, origin, pose=(0.0, 0.0, 0.0)):
    ys, xs = np.nonzero(mask)
    px = origin[0] + (xs + 0.5) * res
    py = origin[1] + (ys + 0.5) * res
    x0, y0, th = pose
    c, s = math.cos(th), math.sin(th)
    return x0 + c * px - s * py, y0 + s * px + c * py


def world_to_cells(x, y, res, origin, shape):
    i = np.floor((x - origin[0]) / res).astype(int)
    j = np.floor((y - origin[1]) / res).astype(int)
    ok = (i >= 0) & (j >= 0) & (i < shape[1]) & (j < shape[0])
    return i[ok], j[ok], ok


def dilate(mask):
    """8-neighbour binary dilation by one cell."""
    out = mask.copy()
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            out |= np.roll(np.roll(mask, dy, 0), dx, 1)
    return out


def flood_fill(passable, seed):
    region = np.zeros_like(passable)
    region[seed] = True
    while True:
        grown = dilate(region) & passable
        if (grown == region).all():
            return region
        region = grown


def nearest_dist(px, py, qx, qy, chunk=2000):
    """Distance from each point p to the nearest point q (brute force)."""
    q = np.stack([qx, qy], 1)
    out = np.empty(len(px))
    for k in range(0, len(px), chunk):
        p = np.stack([px[k:k + chunk], py[k:k + chunk]], 1)
        out[k:k + chunk] = np.sqrt(((p[:, None, :] - q[None]) ** 2).sum(-1)).min(1)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('map')
    ap.add_argument('--pose-log')
    ap.add_argument('--plot', help='write an overlay figure (needs matplotlib)')
    ap.add_argument('--layout', default='layout',
                    help='layout module with the spawn pose (layout, layout_2floor)')
    ap.add_argument('--gt', default=os.path.join(HERE, '..', 'maps', 'ground_truth.yaml'),
                    help='ground-truth map yaml (world frame)')
    ap.add_argument('--floor', type=int, help='only use pose-log rows from this floor')
    ap.add_argument('--tol', type=float, default=0.10, help='match tolerance [m]')
    a = ap.parse_args()
    global START
    START = importlib.import_module(a.layout).START

    gt_occ, gt_free, gt_res, gt_org = load_map(a.gt)
    occ, free, res, org = load_map(a.map)

    # Only the inside of the room is observable: flood fill free space from
    # the spawn point, and keep the walls bordering it.
    si, sj, _ = world_to_cells(np.array([START[0]]), np.array([START[1]]),
                               gt_res, gt_org, gt_occ.shape)
    inside = flood_fill(~gt_occ, (sj[0], si[0]))
    visible_walls = gt_occ & dilate(inside)

    # SLAM occupied cells in the world frame vs true obstacle cell centres
    x, y = cells_to_world(occ, res, org, START)
    gx, gy = cells_to_world(gt_occ, gt_res, gt_org)
    vx, vy = cells_to_world(visible_walls, gt_res, gt_org)
    d = nearest_dist(x, y, gx, gy)
    precision = float(np.mean(d <= a.tol)) if len(d) else 0.0
    recall = float(np.mean(nearest_dist(vx, vy, x, y) <= a.tol)) if len(x) else 0.0

    # free space coverage: rasterise the SLAM free cells into the true grid
    xf, yf = cells_to_world(free, res, org, START)
    fi, fj, _ = world_to_cells(xf, yf, gt_res, gt_org, gt_occ.shape)
    slam_free = np.zeros_like(gt_occ)
    slam_free[fj, fi] = True
    coverage = float(np.mean(slam_free[inside]))

    print(f'map: {a.map}')
    print(f'occupied cells: SLAM {occ.sum()}, ground truth (visible) {visible_walls.sum()}')
    print(f'wall precision  (SLAM wall within {a.tol:.2f} m of a true wall): {precision:6.1%}')
    print(f'wall recall     (true wall within {a.tol:.2f} m of a SLAM wall): {recall:6.1%}')
    print(f'free coverage   (true free space mapped as free):          {coverage:6.1%}')
    print(f'mean wall error (SLAM occupied -> nearest true obstacle):  '
          f'{np.mean(d):.3f} m')

    rows = []
    if a.pose_log:
        with open(a.pose_log) as f:
            rows = list(csv.DictReader(f))
        if a.floor is not None:
            rows = [r for r in rows if int(r.get('floor', 1)) == a.floor]
        exy = np.array([float(r['err_xy']) for r in rows])
        eyaw = np.degrees([float(r['err_yaw']) for r in rows])
        dur = float(rows[-1]['t']) - float(rows[0]['t'])
        print(f'pose log: {len(rows)} samples over {dur:.0f} s sim time')
        print(f'position error: mean {exy.mean():.3f} m, RMSE {np.sqrt((exy ** 2).mean()):.3f} m, '
              f'max {exy.max():.3f} m')
        print(f'heading error:  mean {eyaw.mean():.2f} deg, max {eyaw.max():.2f} deg')

    if a.plot:
        plot(a.plot, gt_occ, gt_res, gt_org, x, y, rows)


def plot(path, gt_occ, gt_res, gt_org, x, y, rows):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 6.6), dpi=150)
    gx, gy = cells_to_world(gt_occ, gt_res, gt_org)
    ax.scatter(gx, gy, s=4, marker='s', c='#c8c8c8', linewidths=0,
               label='ground-truth walls')
    ax.scatter(x, y, s=1.5, marker='s', c='#1f5fbf', linewidths=0,
               label='Hector map (occupied)')
    if rows:
        def world(px, py):
            c, s = math.cos(START[2]), math.sin(START[2])
            px, py = np.asarray(px, float), np.asarray(py, float)
            return START[0] + c * px - s * py, START[1] + s * px + c * py
        tx, ty = world([r['gt_x'] for r in rows], [r['gt_y'] for r in rows])
        ex, ey = world([r['est_x'] for r in rows], [r['est_y'] for r in rows])
        ax.plot(tx, ty, c='#222222', lw=1.6, label='true trajectory')
        ax.plot(ex, ey, c='#e0761c', lw=1.2, ls='--', label='Hector trajectory')
        ax.plot(*world([0], [0]), 'o', c='#222222', ms=5)
    ax.set_aspect('equal')
    ax.set_xlabel('x [m] (world)')
    ax.set_ylabel('y [m] (world)')
    h, w = gt_occ.shape
    ax.set_xlim(gt_org[0], gt_org[0] + w * gt_res)
    ax.set_ylim(gt_org[1], gt_org[1] + h * gt_res)
    ax.grid(alpha=0.3, lw=0.5)
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.1), ncol=2,
              frameon=False, markerscale=4)
    fig.tight_layout()
    fig.savefig(path)
    print(f'figure: {path}')


if __name__ == '__main__':
    main()
