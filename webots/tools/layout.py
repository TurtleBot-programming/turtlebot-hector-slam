"""Single source of truth for the test environment (metres, world frame).

Used by generate_world.py (the .wbt + ground-truth map) and by the bridge
controller (patrol waypoints).
"""

# Axis-aligned boxes: (centre_x, centre_y, size_x, size_y)
WALL_T = 0.1
WALLS = [
    # outer room, 10 m x 8 m
    (0.0, 4.0 + WALL_T / 2, 10.0 + 2 * WALL_T, WALL_T),
    (0.0, -4.0 - WALL_T / 2, 10.0 + 2 * WALL_T, WALL_T),
    (-5.0 - WALL_T / 2, 0.0, WALL_T, 8.0),
    (5.0 + WALL_T / 2, 0.0, WALL_T, 8.0),
    # left room divider at x=-1.5 with a door at y in [1.0, 2.5]
    (-1.5, -1.5, WALL_T, 5.0),
    (-1.5, 3.25, WALL_T, 1.5),
    # right side dividers
    (3.25, 0.5, 3.5, WALL_T),
    (1.5, -2.75, WALL_T, 2.5),
]
OBSTACLES = [
    (-3.2, -1.5, 0.8, 0.8),
    (-3.0, 2.5, 0.5, 0.5),
    (0.0, -0.5, 0.5, 0.5),
    (3.2, 2.5, 0.6, 0.6),
    (3.2, -2.5, 0.6, 0.6),
]
WALL_HEIGHT = 0.5

START = (-4.0, 3.0, 0.0)  # x, y, yaw

# Patrol covering every room and revisiting the start (loop closure).
PATROL = [
    (-4.0, 0.5), (-4.0, -3.0), (-2.2, -3.0), (-2.2, 1.7), (-3.8, 1.7),
    (-3.8, 3.4), (-2.3, 3.4), (-2.3, 1.75), (0.8, 1.75), (0.8, 3.4),
    (-0.8, 3.4), (-0.8, -3.2), (0.8, -3.2), (0.8, -1.0), (2.5, -0.5),
    (4.2, -0.5), (4.2, -3.2), (2.4, -3.2), (2.4, -1.0), (0.8, -1.0),
    (0.8, 1.75), (2.5, 1.75), (4.2, 1.75), (4.2, 3.4), (2.2, 3.4),
    (2.2, 1.75), (0.8, 1.75), (-2.3, 1.75), (-4.0, 3.0),
]
