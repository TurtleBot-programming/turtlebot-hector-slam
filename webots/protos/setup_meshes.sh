#!/usr/bin/env bash
# Copy the TurtleBot 4 / Create 3 CAD meshes from the ROS 2 Jazzy install
# next to the proto (Webots resolves the proto's relative mesh URLs here).
set -euo pipefail
ROS_ROOT="${ROS_ROOT:-/opt/ros/jazzy}"
DEST="$(cd "$(dirname "$0")" && pwd)/meshes"
mkdir -p "$DEST"
for f in \
  irobot_create_description/meshes/body_visual.dae \
  irobot_create_description/meshes/bumper_collision.dae \
  irobot_create_description/meshes/bumper_visual.dae \
  turtlebot4_description/meshes/camera_bracket.dae \
  turtlebot4_description/meshes/oakd_pro.dae \
  turtlebot4_description/meshes/rplidar.dae \
  turtlebot4_description/meshes/shell_collision.dae \
  turtlebot4_description/meshes/shell.dae \
  turtlebot4_description/meshes/tower_sensor_plate.dae \
  turtlebot4_description/meshes/tower_standoff.dae \
  turtlebot4_description/meshes/weight_block.dae; do
  cp -u "$ROS_ROOT/share/$f" "$DEST/"
done
echo "Meshes copied to $DEST"
