#!/usr/bin/env bash
# Start Webots with the SLAM test world and the ROS 2 bridge controller.
#   ./run_sim.sh            # drive with teleop (/cmd_vel_unstamped_unstamped)
#   ./run_sim.sh --patrol   # drive the built-in waypoint loop
#   ./run_sim.sh --headless # no GUI (xvfb, fast mode)
#   ./run_sim.sh --world two_floor   # two-floor building with ramps
set -eo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
source "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"

export WEBOTS_HOME="${WEBOTS_HOME:-/snap/webots/current/usr/share/webots}"
export PYTHONPATH="$WEBOTS_HOME/lib/controller/python:${PYTHONPATH:-}"
export WEBOTS_CONTROLLER_URL="tcp://localhost:1234/Turtlebot4"

BRIDGE_ARGS=()
HEADLESS=0
WORLD=hector_slam_test
while [ $# -gt 0 ]; do
  case "$1" in
    --headless) HEADLESS=1 ;;
    --world) WORLD="$2"; shift ;;
    --world=*) WORLD="${1#--world=}" ;;
    *) BRIDGE_ARGS+=("$1") ;;
  esac
  shift
done
case "$WORLD" in
  hector_slam_test) LAYOUT=layout ;;
  two_floor) LAYOUT=layout_2floor ;;
  *) echo "unknown world '$WORLD' (hector_slam_test, two_floor)" >&2; exit 1 ;;
esac
BRIDGE_ARGS+=(--layout "$LAYOUT")

[ -d "$HERE/protos/meshes" ] || "$HERE/protos/setup_meshes.sh"
[ -f "$HERE/worlds/hector_slam_test.wbt" ] || (cd "$HERE/tools" && python3 generate_world.py)
[ -f "$HERE/worlds/two_floor.wbt" ] || (cd "$HERE/tools" && python3 generate_world_2floor.py)

# The Webots snap cannot read removable drives (/media, /mnt) unless
# `sudo snap connect webots:removable-media` was run. Stage a copy under $HOME.
WORLD_DIR="$HERE"
if command -v snap >/dev/null && snap list webots >/dev/null 2>&1 \
   && [[ "$HERE" != "$HOME"/* ]]; then
  WORLD_DIR="$HOME/tb4_hector_webots"
  mkdir -p "$WORLD_DIR"
  cp -ru "$HERE/worlds" "$HERE/protos" "$WORLD_DIR/"
  echo "Staged world in $WORLD_DIR (snap cannot read $HERE)"
fi

WEBOTS_CMD=(webots --stdout --stderr "$WORLD_DIR/worlds/$WORLD.wbt")
if [ "$HEADLESS" = 1 ]; then
  WEBOTS_CMD=(webots --mode=realtime --no-rendering --minimize --batch --stdout --stderr "$WORLD_DIR/worlds/$WORLD.wbt")
fi
"${WEBOTS_CMD[@]}" &
WEBOTS_PID=$!
trap 'kill $WEBOTS_PID 2>/dev/null || true' EXIT
sleep 6
python3 "$HERE/controllers/tb4_ros2_bridge/tb4_ros2_bridge.py" "${BRIDGE_ARGS[@]}"
