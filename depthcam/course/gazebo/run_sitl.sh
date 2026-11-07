#!/usr/bin/env bash
# Start Gazebo Harmonic on the course world, spawn the drone, then PX4 SITL attached to both
# (WSL2 / Ubuntu 22.04).
#
#   bash course/gazebo/run_sitl.sh                       # GUI + PX4 shell (pxh>) in this terminal
#   HEADLESS=1 bash course/gazebo/run_sitl.sh            # no GUI
#   DRONE=x500_course_lidar_2d bash course/gazebo/run_sitl.sh   # with the 2D lidar (needs GPU rendering)
#   DRONE=gz_x500 bash course/gazebo/run_sitl.sh         # PX4's stock model, spawned by PX4
#   WORLD=course_v3 POSE="-2.5146,0.9144,0.05,0,0,1.5708" PX4_DIR=~/PX4-Autopilot bash ...
#
# DRONE=x500_course (default) / x500_course_lidar_2d are PX4's x500 without propeller collisions
# (make_sim_drone.py explains why); this script spawns them and PX4 attaches with the matching
# airframe.  DRONE=gz_<model> lets PX4 spawn its own model as usual.
#
# PX4 normally starts Gazebo itself on a world inside its own tree.  If a world is already running
# it attaches to it instead (px4-rc.gzsim), so this script starts Gazebo on the repo's world with the
# environment PX4 would have set (its models, plugins and server.config).  POSE is
# x,y,z,roll,pitch,yaw in the course frame; the default is the launch mark, nose north, which is the
# heading the planner assumes by default (14_plan_course.py prints the matching POSE).
#
# Then fly a plan from a second terminal:
#   python3 course/15_fly_plan.py course/out/window_yaw90_px4_ned.csv --gcs-heartbeat
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
WORLD="${WORLD:-course_v3}"
DRONE="${DRONE:-x500_course}"
POSE="${POSE:--2.5146,0.9144,0.05,0,0,1.5708}"
WORLD_FILE="$REPO/course/gazebo/worlds/$WORLD.sdf"

[ -f "$WORLD_FILE" ] || { echo "no $WORLD_FILE: run python course/13_make_gazebo_world.py first" >&2; exit 2; }
[ -d "$PX4_DIR" ] || { echo "no PX4 checkout at $PX4_DIR (set PX4_DIR)" >&2; exit 2; }
if gz topic -l 2>/dev/null | grep -q '^/world/.*/clock'; then
  echo "a Gazebo world is already running; stop it first (pkill -f 'gz sim')" >&2
  exit 1
fi
case "$DRONE" in
  x500_course) AIRFRAME=gz_x500 ;;
  x500_course_lidar_2d) AIRFRAME=gz_x500_lidar_2d ;;
  gz_*) AIRFRAME="$DRONE" ;;
  *) echo "unknown DRONE=$DRONE (x500_course, x500_course_lidar_2d or a PX4 gz_<model> target)" >&2; exit 2 ;;
esac
if [ "${DRONE#gz_}" = "$DRONE" ]; then
  python3 "$REPO/course/gazebo/make_sim_drone.py" --px4 "$PX4_DIR"
fi

export PX4_GZ_MODELS="$PX4_DIR/Tools/simulation/gz/models"
export PX4_GZ_WORLDS="$PX4_DIR/Tools/simulation/gz/worlds"
export GZ_SIM_RESOURCE_PATH="$REPO/course/gazebo/models:$PX4_GZ_MODELS:$PX4_GZ_WORLDS${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PX4_DIR/build/px4_sitl_default/src/modules/simulation/gz_plugins${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_SIM_SERVER_CONFIG_PATH="$PX4_DIR/src/modules/simulation/gz_bridge/server.config"

gz sim --verbose=1 -r -s "$WORLD_FILE" &
GZ_PID=$!
trap 'kill $GZ_PID 2>/dev/null || true; pkill -f "gz sim -g" 2>/dev/null || true' EXIT
for _ in $(seq 1 60); do
  gz topic -l 2>/dev/null | grep -q "^/world/$WORLD/clock" && break
  sleep 0.5
done
if [ -z "${HEADLESS:-}" ]; then
  gz sim -g >/dev/null 2>&1 &
fi

if [ "${DRONE#gz_}" = "$DRONE" ]; then
  IFS=, read -r PX PY PZ PR PP PYAW <<< "$POSE"
  NAME="${DRONE}_0"
  gz service -s "/world/$WORLD/create" --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 5000 \
    --req "sdf: '<sdf version=\"1.9\"><include><uri>model://$DRONE</uri><name>$NAME</name><pose>$PX $PY $PZ $PR $PP $PYAW</pose></include></sdf>', allow_renaming: false" \
    >/dev/null
  echo "spawned $NAME at $POSE"
  cd "$PX4_DIR"
  PX4_GZ_WORLD="$WORLD" PX4_GZ_MODEL_NAME="$NAME" make px4_sitl "$AIRFRAME"
else
  cd "$PX4_DIR"
  PX4_GZ_WORLD="$WORLD" PX4_GZ_MODEL_POSE="$POSE" make px4_sitl "$AIRFRAME"
fi
