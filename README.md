# TurtleBot Hector SLAM

Case study for **ICTE4001** (Curtin Colombo, Year 3). The project explores
2D mapping and localisation for a TurtleBot using Hector SLAM, with the robot
simulated in [Webots](https://cyberbotics.com/).

## Repository structure

```
.
├── docs/      Project documentation (reports, notes, diagrams)
├── tb4_hector_slam/  ROS 2 package: Hector SLAM on the TurtleBot 4
├── webots/    Webots simulation files (worlds, controllers, protos)
└── README.md
```

| Folder | Purpose |
| ------ | ------- |
| [`docs/`](docs/) | Written documentation for the case study |
| [`tb4_hector_slam/`](tb4_hector_slam/) | ROS 2 package launching Hector SLAM on the TurtleBot 4 |
| [`webots/`](webots/) | Webots worlds, robot controllers and related assets |

## Getting started

### Prerequisites

- [Webots](https://cyberbotics.com/#download) (R2023b or newer recommended)
- ROS with the `hector_slam` package, if running SLAM outside Webots

### Clone

```bash
git clone https://github.com/TurtleBot-programming/turtlebot-hector-slam.git
cd turtlebot-hector-slam
```

## Runbook

All commands assume ROS 2 Humble. Each numbered block is a separate terminal.
Status: build + sim + Hector `/map` were seen working; Nav2 + explorer
end-to-end is **still untested** (see "Known issues").

### 1. Build (once, and after any change to config/launch/source)

Hector SLAM (`hector_slam_humble`, with the best-effort `/scan` QoS patch in
`HectorMappingRos.cpp`) must be built first, e.g. in `~/hector_ws`:

```bash
source /opt/ros/humble/setup.bash
source ~/hector_ws/install/setup.bash          # patched hector_mapping
mkdir -p ~/tb4_slam_ws/src && cd ~/tb4_slam_ws
ln -sfn ~/turtlebot-hector-slam/tb4_hector_slam src/tb4_hector_slam
colcon build --symlink-install --packages-select tb4_hector_slam
source install/setup.bash
```

`--symlink-install` matters: without it the installed `nav2.yaml` is a stale
copy and config edits are silently ignored.

### 2. Start the Webots simulation (terminal 1)

```bash
cd ~/turtlebot-hector-slam
chmod +x webots/run_sim.sh webots/protos/setup_meshes.sh   # first time only
./webots/run_sim.sh               # GUI, drive via /cmd_vel
./webots/run_sim.sh --headless    # no GUI
./webots/run_sim.sh --patrol      # built-in waypoint loop (do NOT combine with explore:=true)
```

The first run copies the TurtleBot 4 meshes from `/opt/ros/humble/share`. The
Webots snap cannot read `/media` or `/mnt`, so the script stages a copy in
`~/tb4_hector_webots`. Check it works: `ros2 topic list` shows `/clock`,
`/scan`, `/odom`, `/tf`, `/cmd_vel`, `/ground_truth/pose`.

### 3. Pick one mode (terminal 2)

Source the environment in every new terminal first:

```bash
source /opt/ros/humble/setup.bash
source ~/hector_ws/install/setup.bash
source ~/tb4_slam_ws/install/setup.bash
```

| Mode | Command |
| ---- | ------- |
| SLAM only (+ RViz) | `ros2 launch tb4_hector_slam hector_slam_launch.py use_sim_time:=true` |
| Nav2 only | `ros2 launch tb4_hector_slam navigation_launch.py use_sim_time:=true` |
| SLAM + Nav2 + RViz (click **2D Goal Pose**) | `ros2 launch tb4_hector_slam slam_nav_launch.py use_sim_time:=true` |
| SLAM + Nav2 + autonomous exploration | `ros2 launch tb4_hector_slam slam_nav_launch.py use_sim_time:=true explore:=true` |
| Physical TurtleBot 4 | `ros2 launch tb4_hector_slam slam_nav_launch.py` (`use_sim_time` stays `false`; add `namespace:=/<ns>` if needed) |

Other launch args: `use_rviz`, `rviz_config`, `params_file`, `log_level`.

Manual driving (SLAM only mode):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 4. Check things are alive

```bash
ros2 topic info /scan -v | grep -B1 -A3 hector     # hector_mapping subscriber must be BEST_EFFORT
ros2 run tf2_ros tf2_echo map odom                 # Hector's map -> odom
ros2 topic hz /map
ros2 lifecycle get /bt_navigator                   # expect: active [3]
ros2 node info /global_costmap/global_costmap | grep -A6 Subscribers   # must list /map and /scan (not /global_costmap/map)
```

### 5. Save the map and compare with ground truth

```bash
ros2 run nav2_map_server map_saver_cli -f ~/turtlebot-hector-slam/tb4_hector_slam/map/my_map
# reference map: webots/maps/ground_truth.{pgm,yaml}; true robot pose on /ground_truth/pose
```

### 6. Stop everything

`Ctrl+C` the launch terminals, then close Webots (or in headless mode kill
`webots-bin`). Stray nodes from earlier runs can linger
(`pgrep -fa "nav2|hector|rviz2"`), kill them before relaunching.

### Known issues / things to verify

- Nav2 costmaps previously subscribed to `/global_costmap/map` and
  `/global_costmap/scan`, so the global costmap never got the map and every
  explorer goal timed out. `config/nav2.yaml` now uses absolute `/map` and
  `/scan`; **this fix is not yet verified**. This also means a namespaced
  robot needs topic remaps.
- Explorer thresholds (`min_frontier_cells`, `goal_clearance`, `blacklist_radius`,
  `min_goal_distance`, `goal_timeout`) are untuned guesses.
- Hector `map_size: 2048` at 0.05 m is a ~4 MB message; shrink it if the map is slow to reach Nav2.
- Not tested on the physical robot. See `HANDOVER.md` for the checklist.

## Contributing

Create a branch, commit your changes with a clear message, and open a pull
request against `main`.
