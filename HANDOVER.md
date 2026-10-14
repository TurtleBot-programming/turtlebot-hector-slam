# Handover: Hector SLAM + Nav2 on TurtleBot 4

> **Update 2026-10-14:** the full pipeline now runs end-to-end in Webots (SLAM +
> Nav2 + frontier explorer, ~3 cm localisation error, 99 % wall precision). The
> fixes are listed in the top-level README (Setup, Known limitations). The
> "not tested" items below are done for simulation; only the physical-robot
> checklist is still open. Hector needs a second patch on top of the QoS one:
> the `base_link -> laser` transform is now applied to each scan.

State of the project when moved to a Webots-enabled machine (2026-10-13).
Goal: map and navigate with Hector SLAM + Nav2 in Webots, then run the same
launch files on the physical TurtleBot 4 (ROS 2 Humble).

## What exists

| Path | What | Status |
| ---- | ---- | ------ |
| `tb4_hector_slam/` | ROS 2 package (this work) | builds, launches; **not yet run against Webots** |
| `webots/` | world, TurtleBot 4 PROTO, Python ROS 2 bridge, `run_sim.sh` | written in another session; untracked in git |
| `../hector_slam_humble/` | Hector SLAM ROS 2 port (sibling of this repo) | **locally patched, see below** |

### `tb4_hector_slam` contents

- `launch/hector_slam_launch.py` – `hector_mapping` + `hector_trajectory_server` + RViz (practical-style launch: `namespace`, `use_sim_time`, `params_file`, `log_level`, `RewrittenYaml`).
- `launch/navigation_launch.py` – Nav2 navigation only (no map_server/AMCL; Hector provides `/map` and `map -> odom`).
- `launch/slam_nav_launch.py` – everything in one: SLAM + Nav2 + RViz, plus `explore:=true` for the explorer. `use_sim_time` defaults to `false` (hardware); pass `true` for Webots.
- `config/hector_slam.yaml` – Hector parameters. **`use_sim_time` is overridden by the launch argument.**
- `config/nav2.yaml` – copy of `turtlebot4_navigation/config/nav2.yaml`, changed so the global costmap uses Hector's `/map` (`subscribe_to_updates: False`, transient local) and the local costmap is scan-only (static layer removed).
- `src/frontier_explorer.cpp` – C++ node: frontier-based autonomous exploration. Reads `/map`, BFS from the robot through free cells, clusters frontiers, sends the nearest valid one to `navigate_to_pose`, blacklists failures, stops when none left.
- `rviz/hector_slam.rviz` – map, scan, TF, trajectory, global/local costmap, plan, 2D Goal Pose tool.

## Important: Hector patch

`hector_mapping` subscribed to `/scan` with default **reliable** QoS. The
TurtleBot 4 RPLIDAR and the Webots bridge (`qos_profile_sensor_data`) publish
**best-effort**, so the two never match and Hector gets no scans.

Fixed in `HectorMappingRos.cpp` (~line 236):
`create_subscription<LaserScan>(p_scan_topic_, rclcpp::SensorDataQoS().keep_last(p_scan_subscriber_queue_size_), ...)`.

Applied to `ICTE4001/hector_slam_humble` and `~/hector_ws/src`. **The patch is
not committed anywhere** (uncommitted change in the `hector_slam_humble` repo).
On the new machine, re-apply it (or copy the repo with the change) and rebuild
`hector_mapping`. If `/map` stays empty, check this first:
`ros2 topic info /scan -v` should show the hector_mapping subscriber as BEST_EFFORT.

## Build on the new machine

```bash
source /opt/ros/humble/setup.bash
# workspace with hector_slam_humble (patched) + this package in src/
colcon build --packages-up-to tb4_hector_slam
source install/setup.bash
```

Needs: `nav2_bringup`, `nav2_common`, `nav2_map_server`, `rviz2`, `tf2_ros`,
`rclcpp_action`, `nav2_msgs`. (Previous machine: `~/hector_ws` for Hector, a
scratch workspace symlinking this package.)

## Run in Webots

Terminal 1 (see `webots/README.md`, `webots/run_sim.sh`; `--patrol` drives a
built-in loop, default is teleop via `/cmd_vel`):

```bash
./webots/run_sim.sh
```

Terminal 2:

```bash
ros2 launch tb4_hector_slam slam_nav_launch.py use_sim_time:=true             # then 2D Goal Pose in RViz
ros2 launch tb4_hector_slam slam_nav_launch.py use_sim_time:=true explore:=true   # autonomous mapping
```

The bridge publishes `/clock`, `/scan` (frame `rplidar_link`), `/odom`, TF
`odom -> base_link` and static `base_link -> rplidar_link`, and subscribes to
`/cmd_vel` (`geometry_msgs/Twist`). It also publishes `ground_truth/pose`
(for map/pose evaluation) and `webots/maps/ground_truth.{pgm,yaml}` exists for
comparison.

## What has and has not been tested

Tested (without Webots):
- Package builds with colcon; all launch files load; YAML parses.
- 25 s launch of SLAM + Nav2 + explorer with no simulator: no errors, only the expected waiting for TF/scan/action server.
- `hector_mapping` `/scan` subscription is BEST_EFFORT after the patch.

**Not tested – do these first:**
1. Hector actually produces `/map` and `map -> odom` from the bridge's scan/odom.
2. Nav2 lifecycle comes up (all nodes active) and `navigate_to_pose` works on the live map.
3. `frontier_explorer`: frontier selection and goal handling have never run on a real map; thresholds (`min_frontier_cells=8`, `goal_clearance=0.3`, `blacklist_radius=0.5`, `min_goal_distance=0.5`, `goal_timeout=60`) are guesses.
4. Anything on the physical robot.

## Likely things to tune / watch for

- Sim time: everything must use `use_sim_time:=true` in Webots, otherwise TF lookups time out. The Nav2 yaml has `use_sim_time: True` baked in, but the launch rewrites it from the argument.
- Robot footprint/speed in `nav2.yaml` (`robot_radius 0.175`, `max_vel_x 0.26`, `max_vel_theta 1.0`) come from the real TB4 config; check they suit the Webots PROTO.
- Hector `laser_min_dist/laser_max_dist` (0.15 / 12.0) and update thresholds in `hector_slam.yaml`; Hector is scan-matching only, so fast rotation smears the map.
- The Hector map origin/size (`map_size: 2048`, `map_resolution: 0.05`, `map_start_x/y: 0.5`) is for a large area; shrink for speed if needed.
- Webots snap cannot read `/media` or `/mnt`; `run_sim.sh` stages a copy under `~/tb4_hector_webots`.
- `use_composition` is `False` for Nav2; fine for TB4 but slower to start.

## Physical TurtleBot 4 checklist

Same launch files, `use_sim_time` left at `false`.
1. Laptop and robot on the same `ROS_DOMAIN_ID` and RMW (`rmw_fastrtps_cpp` or `rmw_cyclonedds_cpp`); `ros2 topic list` shows `/scan`, `/odom`, `/tf`.
2. Undock; make sure nothing else publishes `map -> odom` (stock `turtlebot4_navigation` `slam.launch.py`/`localization.launch.py`).
3. `ros2 launch tb4_hector_slam slam_nav_launch.py` (add `namespace:=/<ns>` if the robot is namespaced).
4. Confirm `/cmd_vel` type on the robot is `geometry_msgs/Twist` (Humble default, assumed but not verified).
5. Drive slowly first; send goals once the map has structure.

## Save the map

```bash
ros2 run nav2_map_server map_saver_cli -f src/tb4_hector_slam/map/my_map
```

## Case-study deliverables still open (from the brief)

- Report (due 25 Oct, 23:59) and presentation (Week 13/14) per `ICTE4001 Case Study(2).pdf`: theory, architecture diagram + pseudocode of Hector's modules, parameters, Nav2 + SLAM integration, performance evaluation (map quality, localisation accuracy vs `ground_truth`), screenshots/videos.
- README already documents launch/config; add screenshots and known limitations after real runs.
- Optional: custom `iar_hector` node implementing Hector's core (occupancy grid, bilinear map gradient, Gauss-Newton scan matcher) to compare against `hector_mapping`. Not started; not required by the brief.

## Git state

Branch `main`. Changes from this session in `tb4_hector_slam/` (new `config/nav2.yaml`, `launch/navigation_launch.py`, `launch/slam_nav_launch.py`, `src/frontier_explorer.cpp`; edited `CMakeLists.txt`, `package.xml`, `README.md`, `rviz/hector_slam.rviz`) and this file are **uncommitted**. `webots/*`, `.vscode/` were already untracked.
