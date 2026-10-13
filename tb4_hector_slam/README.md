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
- `config/hector_slam.yaml` — Hector mapping and trajectory server parameters.
- `rviz/hector_slam.rviz` — RViz config (map, scan, TF, trajectory).
- `map/` — place saved maps here.

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

## Notes

- Disable other nodes publishing `map -> odom` (e.g. the TurtleBot 4 `slam_toolbox`/`nav2` localisation) while Hector runs.
- Tune `laser_min_dist`/`laser_max_dist`, `map_resolution` and the update thresholds in `config/hector_slam.yaml`.
- Not yet tested on hardware or in simulation.
