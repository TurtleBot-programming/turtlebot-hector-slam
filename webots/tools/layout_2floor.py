"""Two-floor test building (metres, world frame). Single source of truth for
generate_world_2floor.py (the .wbt, per-floor ground-truth maps and the ramp
config) and the bridge controller (patrol waypoints).

Building 20 m x 14 m. Floor 1 is the ground (z = 0); floor 2 is a slab whose
top is at FLOOR_HEIGHT. Three ramps connect them, each running along an outer
wall in a 1.4 m wide corridor. On floor 1 the ramp corridors are walled on
the inner side; on floor 2 the ramp openings are railed on the inner side and
at the low end, so the only way in is from the top.

Walls are axis-aligned segments ((x0, y0), (x1, y1)); gaps are doorways.
"""

FLOOR_HEIGHT = 0.6          # top of the floor-2 slab
SLAB_T = 0.05               # slab thickness
WALL_T = 0.1
WALL1_H = FLOOR_HEIGHT - SLAB_T   # floor-1 walls reach the slab underside
WALL2_H = 0.5               # floor-2 walls (on the slab)
RAIL_H = 0.4                # railings around the ramp openings
X0, X1, Y0, Y1 = -10.0, 10.0, -7.0, 7.0

START = (0.0, 0.0, 0.0)     # x, y, yaw on floor 1 (centre of the main hall)

# Ramps: bottom end (x, y) on floor 1, unit direction of climb, run length,
# width. The ramp occupies [bottom, bottom + length * dir] x width.
RAMP_LEN = 6.0
RAMP_W = 1.4
RAMP_EASE = 0.6             # bottom/top transition sections at half the slope
RAMPS = {
    'north': {'bottom': (-4.0, 6.3), 'dir': (1, 0)},    # climbs east,  y in [5.6, 7]
    'south': {'bottom': (5.0, -6.3), 'dir': (-1, 0)},   # climbs west,  y in [-7, -5.6]
    'east':  {'bottom': (9.3, -3.0), 'dir': (0, 1)},    # climbs north, x in [8.6, 10]
}


def ramp_profile():
    """Ramp surface as (u, z) points, u along the climb: eased at both ends
    so the robot's casters and bumper don't catch on a sharp crease."""
    import math
    # z(L) = 2 * e * tan(a / 2) + (L - 2e) * tan(a) = H, solved for a
    lo, hi = 0.0, 0.5
    for _ in range(60):
        a = (lo + hi) / 2
        h = 2 * RAMP_EASE * math.tan(a / 2) + (RAMP_LEN - 2 * RAMP_EASE) * math.tan(a)
        lo, hi = (a, hi) if h < FLOOR_HEIGHT else (lo, a)
    z1 = RAMP_EASE * math.tan(a / 2)
    return [(0.0, 0.0), (RAMP_EASE, z1), (RAMP_LEN - RAMP_EASE, FLOOR_HEIGHT - z1),
            (RAMP_LEN, FLOOR_HEIGHT)]


def ramp_rect(r):
    """Axis-aligned footprint (xmin, xmax, ymin, ymax) of a ramp."""
    (bx, by), (dx, dy) = r['bottom'], r['dir']
    tx, ty = bx + dx * RAMP_LEN, by + dy * RAMP_LEN
    if dx:
        return min(bx, tx), max(bx, tx), by - RAMP_W / 2, by + RAMP_W / 2
    return bx - RAMP_W / 2, bx + RAMP_W / 2, min(by, ty), max(by, ty)


# Outer walls span both floors.
OUTER = [((X0, Y0), (X1, Y0)), ((X0, Y1), (X1, Y1)),
         ((X0, Y0), (X0, Y1)), ((X1, Y0), (X1, Y1))]

# ---- floor 1 ---------------------------------------------------------------
# Main hall y in [-1.2, 1.2] runs west-east through the middle; rooms north
# and south of it. Ramp bottoms: north from the lobby (x = -4), south from
# the south-east band (x = 5), east from the south (y = -3).
WALLS_1 = [
    # ramp corridors, inner sides
    ((-4.0, 5.6), (2.0, 5.6)),
    ((-1.0, -5.6), (5.0, -5.6)),
    ((8.6, -3.0), (8.6, 3.0)),
    # north side of the hall
    ((-10.0, 1.2), (-8.5, 1.2)), ((-7.3, 1.2), (-6.0, 1.2)),   # room N1 door
    ((-6.0, 1.2), (-6.0, 5.0)), ((-6.0, 6.2), (-6.0, 7.0)),    # N1 | lobby, door
    ((-4.0, 1.2), (-4.0, 3.8)), ((-4.0, 4.8), (-4.0, 5.6)),    # lobby | N2, door
    ((-4.0, 1.2), (-2.0, 1.2)), ((-0.8, 1.2), (2.0, 1.2)),     # room N2 door
    ((2.0, 1.2), (2.0, 3.5)), ((2.0, 4.6), (2.0, 5.6)),        # N2 | N3, door
    ((2.0, 3.0), (6.0, 3.0)), ((7.2, 3.0), (8.6, 3.0)),        # N3 | alcove, door
    ((2.0, 1.2), (5.0, 1.2)),                                  # alcove, open east
    # south side of the hall
    ((-10.0, -1.2), (-7.5, -1.2)), ((-6.3, -1.2), (-5.0, -1.2)),  # room S1 door
    ((-5.0, -1.2), (-5.0, -4.3)), ((-5.0, -5.5), (-5.0, -7.0)),   # S1 | corridor
    ((-3.0, -1.2), (0.5, -1.2)), ((1.7, -1.2), (6.0, -1.2)),      # room S2 doors
    ((7.2, -1.2), (8.6, -1.2)),
    ((-3.0, -1.2), (-3.0, -2.8)), ((-3.0, -4.0), (-3.0, -5.6)),   # corridor | S2
    ((-3.0, -5.6), (-1.0, -5.6)),                                 # S2 | pocket
    ((3.0, -5.6), (3.0, -3.0)),                                   # S2 divider
    ((8.6, -1.2), (8.6, -3.0)),                                   # S2 | east ramp
]
OBSTACLES_1 = [  # (cx, cy, sx, sy)
    (-1.0, 4.0, 0.8, 0.8),
    (5.0, 5.2, 1.0, 1.0),
    (-8.0, -4.5, 1.0, 0.6),
    (1.0, -4.0, 0.8, 0.8),
    (6.0, -4.2, 0.6, 1.2),
    (-8.0, 4.5, 0.6, 0.6),
]

# ---- floor 2 ---------------------------------------------------------------
# Corridor y in [-1, 1] west-east; rooms U1..U5 around it. Ramp tops: north
# ends at x = 2 (into the north-east hall), south at x = -1 (into U4), east
# at y = 3 (into the north-east hall).
WALLS_2 = [
    ((-10.0, 1.0), (-7.0, 1.0)), ((-5.8, 1.0), (-4.0, 1.0)),   # U1 door
    ((-4.0, 1.0), (-4.0, 2.0)), ((-4.0, 3.2), (-4.0, 5.6)),    # U1 | U2, door
    ((-4.0, 1.0), (-0.5, 1.0)), ((0.7, 1.0), (4.0, 1.0)),      # U2 door
    ((4.0, 1.0), (4.0, 3.0)),                                  # U2 | U3
    ((4.0, 1.0), (5.5, 1.0)), ((6.7, 1.0), (8.6, 1.0)),        # U3 door
    ((4.0, 3.0), (6.0, 3.0)), ((7.2, 3.0), (8.6, 3.0)),        # U3 | NE hall, door
    ((7.0, 4.5), (7.0, 7.0)),                                  # NE hall divider
    ((-10.0, -1.0), (-8.0, -1.0)), ((-6.8, -1.0), (-3.5, -1.0)),  # U4 doors
    ((-2.3, -1.0), (-1.0, -1.0)),
    ((-5.5, -7.0), (-5.5, -3.5)),                                 # U4 divider
    ((-1.0, -1.0), (-1.0, -5.6)),                                 # U4 | U5
    ((-1.0, -1.0), (2.0, -1.0)), ((3.2, -1.0), (8.6, -1.0)),      # U5 door
]
OBSTACLES_2 = [
    (0.0, 3.3, 0.6, 0.6),
    (-2.5, 2.2, 0.5, 0.5),
    (5.5, 5.0, 0.8, 0.8),
    (-8.0, 4.0, 1.0, 1.0),
    (2.0, -3.5, 1.0, 0.6),
    (6.0, -3.0, 0.6, 0.6),
    (-8.0, -4.0, 0.6, 0.6),
]


def _railings():
    """Floor-2 railings: inner long side and low end of every ramp opening."""
    out = []
    for r in RAMPS.values():
        xmin, xmax, ymin, ymax = ramp_rect(r)
        (bx, by), (dx, dy) = r['bottom'], r['dir']
        if dx:  # runs along x; the inner side is the one away from the outer wall
            inner_y = ymin if ymax >= Y1 - 1e-6 else ymax
            out.append(((xmin, inner_y), (xmax, inner_y)))
            out.append(((bx, ymin), (bx, ymax)))
        else:
            inner_x = xmin if xmax >= X1 - 1e-6 else xmax
            out.append(((inner_x, ymin), (inner_x, ymax)))
            out.append(((xmin, by), (xmax, by)))
    return out


RAILINGS = _railings()

# Patrol: start hall -> floor 1 rooms -> north ramp up -> floor 2 -> south
# ramp down -> floor 1 -> east ramp up -> floor 2 -> north ramp down -> start.
# Waypoints just before a ramp are aligned with it so the robot climbs straight.
PATROL = [
    (-1.4, 0.0), (-1.4, 1.8), (-1.4, 3.0), (-1.4, 0.0), (-5.0, 0.0),
    (-5.0, 5.0), (-5.0, 6.3), (-4.6, 6.3),                     # north ramp bottom
    (2.6, 6.3), (3.5, 5.0), (3.5, 2.0), (0.1, 1.6), (0.1, 0.0),  # up, U2, corridor
    (-7.4, 0.0), (-7.4, -2.0), (-3.0, -2.0), (-2.0, -6.3),
    (-1.6, -6.3),                                               # south ramp top
    (5.6, -6.3), (7.5, -6.3),                                   # down
    (9.3, -5.2), (9.3, -3.6),                                   # east ramp bottom
    (9.3, 3.6), (9.3, 5.0), (6.6, 3.6), (6.6, 2.0), (6.1, 1.6),  # up, NE hall, U3
    (6.1, 0.0), (0.1, 0.0), (0.1, 1.6), (1.0, 4.5), (2.6, 4.8),
    (2.6, 6.3),                                                 # north ramp top
    (2.4, 6.3), (-4.6, 6.3), (-5.0, 5.0), (-5.0, 0.0), (0.0, 0.0),
]
