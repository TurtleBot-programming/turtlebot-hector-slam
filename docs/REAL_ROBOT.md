# Running on a physical TurtleBot 4

**Status: not yet tested on hardware.** Everything here is based on the
simulation results and on how the stock TurtleBot 4 (ROS 2 Humble) is set up.
Expect to adjust parameters; this guide says which ones and how.

The same launch files are used as in simulation. The differences are:
no Webots, no `use_sim_time`, and a network between the laptop and the robot.

```
TurtleBot 4 (Raspberry Pi + Create 3)           Laptop
  RPLIDAR A1  ── /scan  (best-effort) ───────►  hector_mapping ─► /map, map→odom
  Create 3    ── /odom, TF odom→base_link ───►  Nav2 (planner, controller, costmaps)
              ◄─ /cmd_vel_unstamped ───────────────────   frontier_explorer (optional), RViz
```

Hector, Nav2 and RViz run on the **laptop**. The robot only publishes sensors
and executes `/cmd_vel_unstamped`.

---

## 1. Before you start

### Hardware / safety

- Robot charged, lidar spinning, in an area with walls (Hector needs structure).
- Clear floor; no stairs or drops nearby (the Create 3 cliff sensors help, but do not rely on them).
- Keep a person next to the robot with a way to stop it (see [Emergency stop](#emergency-stop)).
- The first runs should be in **one room**, not a long corridor.

### Software on the laptop

Same as the simulation setup (Ubuntu 22.04, ROS 2 Humble). You do **not** need
Webots. You need:

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
                 ros-humble-turtlebot4-description ros-humble-turtlebot4-msgs \
                 ros-humble-irobot-create-msgs ros-humble-teleop-twist-keyboard
```

Then build Hector (with its two patches) and this package exactly as in the
top-level README, steps 1 and 2. Without the patches Hector will either get no
scans (QoS) or build a skewed map (laser transform).

### Robot software

Use the TurtleBot 4 image for Humble and update it:

```bash
# on the robot (ssh ubuntu@<robot-ip>)
sudo apt update && sudo apt upgrade
```

The Create 3 firmware must be a Humble-compatible release (H.x). Check it in
the Create 3 web page at `http://<robot-ip>:8080` (or the USB-C/Wi-Fi address
shown in the TurtleBot 4 user manual).

---

## 2. Connect the laptop to the robot

Both machines must use the same `ROS_DOMAIN_ID` and the same RMW.

```bash
# on BOTH the robot and the laptop, in every terminal
export ROS_DOMAIN_ID=0                    # any number, but identical
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp   # or whatever the robot uses
source /opt/ros/humble/setup.bash
source ~/hector_ws/install/setup.bash
source ~/tb4_slam_ws/install/setup.bash   # laptop only
```

Find out what the robot uses and copy it:

```bash
# on the robot
echo $ROS_DOMAIN_ID $RMW_IMPLEMENTATION
cat /etc/turtlebot4/setup.bash 2>/dev/null     # robot-wide settings, if present
```

Check that the laptop sees the robot:

```bash
ros2 topic list                  # must show /scan /odom /tf /tf_static /cmd_vel_unstamped
ros2 topic hz /scan              # expect ~10 Hz, steady
ros2 topic hz /odom              # expect ~20 Hz
ros2 run tf2_tools view_frames   # odom -> base_link must exist
```

**If the list is empty or `/scan` is slow or jittery**: this is almost always
Wi-Fi discovery or bandwidth, not this project.

- Use the same router for both machines, 5 GHz, and keep the laptop close.
- Try the Fast DDS **discovery server** (recommended by the TurtleBot 4 manual): run it on the robot and set `ROS_DISCOVERY_SERVER=<robot-ip>:11811` on the laptop (and `ROS_SUPER_CLIENT=true` in terminals that run `ros2 topic list`).
- As a last resort connect the laptop with a cable (USB-C Ethernet to the Pi), or run Hector and Nav2 **on the Pi** (slower, see [Running on the robot](#running-everything-on-the-robot)).
- `ros2 daemon stop && ros2 daemon start` after changing any of these settings.

### Namespaced robot

If topics look like `/robot1/scan`, the robot is namespaced. Add
`namespace:=/robot1` to the launch commands and read
[Namespaces](#namespaces), because one part of the config needs editing.

---

## 3. Prepare the robot

### 3.1 Undock

The Create 3 will not drive while docked.

```bash
ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"
# namespaced: /robot1/undock
```

### 3.2 Stop anything else publishing `map → odom`

Exactly one node may publish `map → odom`. The stock `turtlebot4_navigation`
SLAM or localisation would fight with Hector.

```bash
ros2 node list | grep -Ei "slam|amcl|localiz|nav2|map_server"   # expect nothing
ros2 topic info /tf -v | grep "Node name"                       # see who publishes TF
```

If something shows up, stop that launch (Ctrl+C in the terminal that started it).
Do not stop the `turtlebot4_node` or the Create 3 nodes: they publish
`odom → base_link` and the lidar.

### 3.3 Check the lidar frame and mounting

```bash
ros2 topic echo /scan --once | head -30      # frame_id (usually rplidar_link), range_min/max
ros2 run tf2_ros tf2_echo base_link rplidar_link
```

`base_link → rplidar_link` must exist. It is yawed by about 90°; the patched
Hector reads it from TF for every scan, so no manual angle is needed.

### 3.4 Check `use_sim_time`

`config/nav2.yaml` has `use_sim_time: True` written in many places, but the
launch files rewrite it from the `use_sim_time` launch argument (default
`false`), so no edit is needed. Confirm after launching:

```bash
ros2 param get /controller_server use_sim_time    # must say: Boolean value is: False
ros2 param get /hector_mapping use_sim_time        # must say: Boolean value is: False
```

If either says `True`, TF lookups will time out and nothing moves. Check that
you did not pass `use_sim_time:=true`.

---

## 4. Run: step by step

Do it in this order. Each step needs the previous one to look good before you
continue. All commands run on the laptop with the environment from section 2.

### Step A: mapping only, drive by hand

```bash
# terminal 1
ros2 launch tb4_hector_slam hector_slam_launch.py
# terminal 2
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

In RViz the map should grow and the laser scan should line up with the walls.

Drive slowly: **≤ 0.15 m/s and ≤ 0.5 rad/s**. In teleop press `x` / `z` to lower
the speeds. Hector has no wheel odometry in the matcher and no loop closure, so
fast turns smear the map.

What good looks like: straight walls, doors in the right places, the trajectory
(green) does not jump. What bad looks like: doubled walls, a map that rotates
while the robot stands still, the robot "sliding" along a corridor.

### Step B: add Nav2, send goals by hand

Stop step A (Ctrl+C), then:

```bash
ros2 launch tb4_hector_slam slam_nav_launch.py
```

In RViz use **Nav2 Goal** to click a point about 1 m away in known free space.
Check that the global plan (path) and local costmap look sensible and the robot
stops 0.25 m from the goal. Try a goal behind a chair to see it route around.

### Step C: autonomous exploration

Only once step B is reliable:

```bash
ros2 launch tb4_hector_slam slam_nav_launch.py explore:=true
```

The explorer logs `Exploring towards frontier (x, y)`. It ends with
`No reachable frontiers left: exploration complete`. Watch the robot the whole
time. On real floors expect this to take several times longer than the ~3 min
in simulation.

Do not run teleop at the same time as Nav2 (both publish `/cmd_vel_unstamped`).

### Save the map

```bash
ros2 run nav2_map_server map_saver_cli -f ~/real_map
```

This writes `real_map.pgm` and `real_map.yaml`. There is no ground truth on a
real robot, so measure two or three distances in the room with a tape measure
and compare them with the map (in RViz use the **Measure** tool, or compute from
the PGM: distance = pixels × 0.05 m).

### Record localisation data (optional)

`webots/tools/pose_logger.py` needs the simulator's ground truth and does not
work on hardware. Record the topics instead and replay later:

```bash
ros2 bag record -o real_run /scan /odom /tf /tf_static /map /slam_out_pose
```

---

## 5. What to adjust, and how

Edit the files in `~/turtlebot-hector-slam/tb4_hector_slam/config/`. Because the
package is built with `--symlink-install`, edits take effect on the next launch
without rebuilding. Restart the launch after every change.

Change **one thing at a time**, and re-run the same route to compare.

### 5.1 Symptoms → fixes

| Symptom on the real robot | Likely cause | What to change |
| --- | --- | --- |
| `/map` empty, Hector silent | QoS patch missing or `/scan` not arriving | `ros2 topic info /scan -v`: Hector subscriber must be `BEST_EFFORT`; check `ros2 topic hz /scan` on the laptop |
| Map is skewed, robot slides sideways | Laser-transform patch missing | Rebuild Hector with the second patch (README step 1) |
| `Transform data too old` / `extrapolation into the future` in Nav2 | Network latency, or clocks differ between robot and laptop | Sync clocks (below), then raise `transform_tolerance` in `nav2.yaml` (controller, both costmaps) from `0.5` to `0.8`–`1.0` |
| Robot stops often, `Failed to make progress` | Costmap too tight, or goal tolerance too small | Raise `inflation_radius` (`0.45` → `0.55`) or `xy_goal_tolerance` (`0.25` → `0.35`) |
| Map smears on turns | Rotating too fast | Lower `max_vel_theta` in `nav2.yaml` (`1.0` → `0.6`); in teleop keep turns slow |
| Map gets noisy / fills with speckles | Lidar noise, glass, shiny floors | In `hector_slam.yaml` lower `update_factor_occupied` (`0.9` → `0.8`), raise `laser_min_dist` (`0.15` → `0.2`) |
| Map updates too slowly or the robot out-runs it | Hector update thresholds | In `hector_slam.yaml` lower `map_update_distance_thresh` (`0.4` → `0.2`) and `map_update_angle_thresh` (`0.06` → `0.03`) |
| Robot sees its own body as an obstacle | Lidar shares view with robot parts | Raise `laser_min_dist` in `hector_slam.yaml`; and `obstacle_min_range` in both costmaps in `nav2.yaml` (`0.0` → `0.2`) |
| Phantom walls from far away | Lidar max range noise | Lower `laser_max_dist` (`12.0` → `8.0`) |
| Robot too fast for the room | Default speed limits | `max_vel_x` and `max_speed_xy` in `nav2.yaml` (`0.26` → `0.15` for the first runs) |
| Explorer stops almost immediately | Few reachable frontiers, or every goal blacklisted | Lower `min_frontier_cells` (`8` → `5`) and `goal_clearance` (`0.3` → `0.25`); see 5.3 |
| Explorer keeps chasing the same spot | Frontier in noisy area, goal unreachable | Raise `blacklist_radius` (`0.5` → `0.8`), lower `goal_timeout` (`60` → `40`) |
| Map has a seam after a loop | No loop closure in Hector | Not fixable in this setup; plan routes that avoid big loops, or use a different SLAM for loops |
| Robot slips along a long corridor | Featureless corridor, scan matcher has no constraint | Do not map long empty corridors with Hector; add something to the walls, or use odometry-based SLAM there |

### 5.2 Speed limits (do this first)

`config/nav2.yaml`, in `controller_server → FollowPath`:

```yaml
max_vel_x: 0.15        # was 0.26
max_speed_xy: 0.15     # was 0.26
max_vel_theta: 0.6     # was 1.0
```

and in `velocity_smoother`:

```yaml
max_velocity: [0.15, 0.0, 0.6]    # was [0.26, 0.0, 1.0]
```

Raise them again once the map is stable.

### 5.3 Explorer parameters

The explorer takes ROS parameters. Pass them without editing code:

```bash
ros2 run tb4_hector_slam frontier_explorer --ros-args \
    -p min_frontier_cells:=5 -p goal_clearance:=0.25 \
    -p blacklist_radius:=0.8 -p goal_timeout:=40.0
```

To set them through the launch file, add them under `parameters=[...]` in
`launch/slam_nav_launch.py`, in the `explorer` node, next to `use_sim_time`.
That file is symlinked, so no rebuild is needed (only C++ changes need
`colcon build`).

| Parameter | Default | Meaning |
| --- | --- | --- |
| `min_frontier_cells` | 8 | Smallest frontier (in cells) worth visiting. Lower = explores small gaps, but chases noise. |
| `goal_clearance` | 0.3 m | Free space needed around a goal. Keep at least the robot radius (0.175). |
| `blacklist_radius` | 0.5 m | After a failure, goals this close to the failed one are skipped. |
| `min_goal_distance` | 0.5 m | Ignore frontiers closer than this to the robot. |
| `goal_timeout` | 60 s | Cancel and blacklist a goal after this long. Real floors are slower; raise it if goals time out while still moving. |
| `empty_checks_to_finish` | 5 | Consecutive "no frontier" checks before declaring done. |

(`climb_speed`, `ramp_*` are only for the two-floor simulation world.)

### 5.4 Hector map size and start

`config/hector_slam.yaml`:

```yaml
map_resolution: 0.05
map_size: 512          # cells; 512 x 0.05 = 25.6 m
```

If the area is larger than about 20 m across, raise `map_size` (1024 =
51 m). Very large maps made RViz freeze in simulation, so go up in steps.
The map origin is the robot's start position, so start roughly in the middle of
the area.

### 5.5 Robot size

`robot_radius: 0.175` in both costmaps is correct for the TurtleBot 4 (Create 3
base is about 0.34 m in diameter). Do not lower it. Raise it a little (`0.2`)
if the robot clips furniture.

### 5.6 Clock sync

TF and scan timestamps are compared across machines. If the robot's clock and
the laptop's clock differ by more than ~0.1 s you will see TF extrapolation
errors.

```bash
# on both machines
timedatectl status | grep -E "synchronized|NTP"
sudo apt install chrony && sudo systemctl enable --now chrony
```

For a robot with no internet, point both machines at the same NTP source
(e.g. make the laptop an NTP server for the robot).

---

## 6. Namespaces

If the robot uses a namespace (`/robot1/...`) launch with it:

```bash
ros2 launch tb4_hector_slam slam_nav_launch.py namespace:=/robot1
```

Hector, the trajectory server, the Nav2 nodes and the explorer are all started
inside the namespace, `/tf` and `/tf_static` are remapped, and the costmaps
use relative topic names (`scan`, `map`), so they resolve to `/robot1/scan` and
`/robot1/map`. Check with `ros2 node list` that everything is under `/robot1`.

To use your own Nav2 parameters, pass them with
`nav2_params_file:=/path/to/nav2.yaml` (accepted by both `slam_nav_launch.py`
and `navigation_launch.py`).

---

## 7. Running everything on the robot

Only if the Wi-Fi link is unreliable. The Raspberry Pi 4 is slow: Hector at
10 Hz is fine, but Nav2 + RViz are not. Run Hector and Nav2 on the Pi and RViz
on the laptop.

```bash
# on the robot: install the same packages and build both workspaces,
# then
ros2 launch tb4_hector_slam slam_nav_launch.py use_rviz:=false
# on the laptop
ros2 run rviz2 rviz2 -d ~/turtlebot-hector-slam/tb4_hector_slam/rviz/hector_slam.rviz
```

Watch CPU with `top`; if Nav2 falls behind, lower the costmap
`update_frequency` and the controller `controller_frequency` in `nav2.yaml`.

---

## 8. Emergency stop

Fastest first:

1. `Ctrl+C` the launch terminal. Nav2 stops commanding, and the Create 3 stops after its command timeout.
2. Press the **robot's power / bumper** or lift it: the Create 3 stops on bump, cliff and wheel drop.
3. Power the robot off (hold the power button).

Keep a second terminal open with the teleop node ready, but remember teleop and
Nav2 both publish `/cmd_vel_unstamped` and must not run together.

---

## 9. Checklist

- [ ] Same `ROS_DOMAIN_ID` / RMW on laptop and robot; `ros2 topic hz /scan` ≈ 10 Hz
- [ ] Clocks synced
- [ ] Robot undocked; nothing else publishes `map → odom`
- [ ] `use_sim_time` is `False` in `hector_mapping` and `controller_server`
- [ ] `ros2 topic info /scan -v`: Hector subscriber is `BEST_EFFORT`
- [ ] Speed limits lowered for the first run
- [ ] Step A (teleop mapping) gives a clean map
- [ ] Step B (clicked goals) works
- [ ] Step C (`explore:=true`) with someone watching the robot
- [ ] Map saved with `map_saver_cli` and checked with a tape measure

## 10. What to report in the case study

Real-robot results will not match the simulation numbers (99 % precision,
~3 cm error). Report honestly:

- Measured wall distances vs the map (3 or more).
- How well the start and end poses agree after a loop (drift, since there is no loop closure).
- Which parameters you changed and why (use the table in section 5).
- Where Hector failed (corridors, glass, fast turns) and why that follows from scan-matching-only SLAM.
