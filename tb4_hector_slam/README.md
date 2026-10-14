# tb4_hector_slam

ROS 2 (ament_cmake) package that runs **Hector SLAM** on the TurtleBot 4. It
follows the layout of the earlier practical packages (`tb4_cpp_prac5/6`):
launch file, YAML parameters, and a rewritten-params launch pattern with
`namespace` and `use_sim_time` support. ICTE4001 case study, Curtin Colombo.

Hector SLAM needs only the 2D LiDAR (no wheel odometry in the scan matcher),
so it uses the TurtleBot 4 `/scan` topic and the `odom -> base_link` transform
the robot already publishes, and adds `map -> odom`.

## Package layout

- `launch/hector_slam_launch.py` — starts `hector_mapping_node`, `hector_trajectory_server` and (optionally) RViz.
- `src/frontier_explorer.cpp` — C++ node: autonomous frontier exploration. Reads Hector's `/map`, picks the nearest reachable frontier, sends it to Nav2 (`navigate_to_pose`), blacklists unreachable goals, stops when the map is closed.
- `launch/navigation_launch.py` — Nav2 navigation stack only (no map_server/AMCL).
- `launch/slam_nav_launch.py` — Hector SLAM + Nav2 + RViz in one command.
- `config/hector_slam.yaml` — Hector mapping and trajectory server parameters.
- `config/nav2.yaml` — Nav2 parameters (from `turtlebot4_navigation`, global costmap fed by Hector's `/map`).
- `rviz/hector_slam.rviz` — RViz config (map, scan, TF, trajectory).
- `map/` — saved maps; `hector_map.*` + `pose_error.csv` are the reference autonomous-exploration run (see the top-level README for results).

## Dependencies

- ROS 2 Humble, TurtleBot 4 packages (real robot or simulator)
- [`hector_slam_humble`](../../hector_slam_humble) (`hector_mapping`, `hector_trajectory_server`, `hector_nav_msgs`, ...) built in the same workspace
- `nav2_common`, `nav2_map_server`, `rviz2`

## Build

Put `hector_slam_humble` packages and this package in the workspace `src/`:

```bash
colcon build --packages-up-to tb4_hector_slam
source install/setup.bash
```

## Run

Start the TurtleBot 4 (or simulation) first, then:

```bash
ros2 launch tb4_hector_slam hector_slam_launch.py
# simulation:
ros2 launch tb4_hector_slam hector_slam_launch.py use_sim_time:=true
```

Launch arguments: `namespace`, `use_sim_time`, `params_file`, `log_level`, `use_rviz`, `rviz_config`.

Drive the robot (e.g. `ros2 run teleop_twist_keyboard teleop_twist_keyboard`),
then save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f src/tb4_hector_slam/map/my_map
```

## Navigate while mapping

```bash
ros2 launch tb4_hector_slam slam_nav_launch.py use_sim_time:=true   # Webots
ros2 launch tb4_hector_slam slam_nav_launch.py                      # physical TurtleBot 4
```

Hector provides `/map` and `map -> odom`; Nav2's global costmap uses `/map`
(unknown space is traversable, so goals can be set in unexplored areas) and
the local costmap uses `/scan`. In RViz use **2D Goal Pose** to send a goal.
Nav2 publishes `/cmd_vel`, which the Webots bridge subscribes to.
`use_sim_time` defaults to `false` (physical robot), as in the practical packages; pass `true` for Webots.
Run SLAM and Nav2 separately with `hector_slam_launch.py` / `navigation_launch.py`.

## Autonomous exploration

```bash
ros2 launch tb4_hector_slam slam_nav_launch.py explore:=true use_sim_time:=true
```

`frontier_explorer` parameters: `min_frontier_cells`, `goal_clearance`,
`blacklist_radius`, `min_goal_distance`, `goal_timeout`. When it logs
"exploration complete", save the map with `map_saver_cli`.

## Running on the physical TurtleBot 4

The same launch files are used on hardware; only `use_sim_time` differs.

1. Undock the robot and make sure the Create 3 and the Raspberry Pi are on the same `ROS_DOMAIN_ID` / RMW (`rmw_fastrtps_cpp` or `rmw_cyclonedds_cpp`) as your laptop. Check `ros2 topic list` shows `/scan`, `/odom`, `/tf`.
2. Stop anything else publishing `map -> odom` on the robot (the stock `turtlebot4_navigation` `slam.launch.py` / `localization.launch.py`).
3. On the laptop (or the Pi): `ros2 launch tb4_hector_slam slam_nav_launch.py`.
4. If the robot uses a namespace: `namespace:=/robot_ns`.
5. Drive slowly at first (Hector is scan-matching only, so fast rotation smears the map). Send goals with **2D Goal Pose** once the map has a few metres of structure.

`hector_mapping` must include the sensor-data QoS fix for `/scan`: the RPLIDAR
(and the Webots bridge) publish best-effort, which a default reliable
subscription never matches. This is applied in `hector_slam_humble`
(`HectorMappingRos.cpp`, `rclcpp::SensorDataQoS()`); rebuild it after pulling.

## Notes

- Disable other nodes publishing `map -> odom` (e.g. the TurtleBot 4 `slam_toolbox`/`nav2` localisation) while Hector runs.
- Tune `laser_min_dist`/`laser_max_dist`, `map_resolution` and the update thresholds in `config/hector_slam.yaml`.
- Tested end-to-end in Webots (SLAM + Nav2 + explorer, 2026-10-14); not yet tested on hardware.
- `hector_mapping` must also include the laser-transform fix (applies `base_link -> laser` to each scan); without it the rotated TB4 lidar makes the pose drift sideways.
- Full run guide, results and troubleshooting: [top-level README](../README.md).
