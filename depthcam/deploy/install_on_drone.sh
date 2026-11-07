#!/usr/bin/env bash
# Install the ROS 2 package dronehacks_lidar into drone 1's team workspace.
#
# Run it on the drone, in the host shell, from a checkout of this repo:
#     bash depthcam/deploy/install_on_drone.sh
#
# It refuses to run anywhere but drone 1 (hostname skybeta-01). Logging into or deploying to another
# team's drone gets a team disqualified (AGENTS.md §1). It:
#   1. copies what colcon builds from depthcam/ (the package root) to ~/team_ws/src/dronehacks_lidar:
#      package.xml, setup.py, setup.cfg, resource/, launch/, routes/, the Python packages setup.py
#      installs (dronehacks_lidar, pose_fusion, follower, course and its subpackages), the map and
#      rules JSON, and the parameter file. It leaves out the research code, recordings, CAD files
#      and the STEP model.
#   2. adds pyserial to the team image's Dockerfile (~/team_ws/Dockerfile), if missing
# then prints the remaining steps. It does not edit /etc/skybeta/drone.env (that needs sudo, and
# choosing the run's lidar policy is yours). It does not restart the stack either. Supply the
# measured course->odom transform (odom_yaw_deg / odom_origin_x / odom_origin_y) in the params
# file for the run, and verify the drone's placement against the follower's 0.25 m / 10 deg
# start check. See methods/lidar-vo-fusion/docs/deploy.md.
set -euo pipefail

EXPECTED_HOST="skybeta-01"
host="$(hostname)"
if [ "$host" != "$EXPECTED_HOST" ]; then
  echo "install_on_drone: this is '$host', not drone 1 ($EXPECTED_HOST). Nothing was changed." >&2
  exit 1
fi

SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="$HOME/team_ws/src/dronehacks_lidar"
[ -f "$SRC/package.xml" ] || { echo "install_on_drone: $SRC has no package.xml" >&2; exit 2; }

echo "[1/2] $SRC -> $DEST (the package's build inputs only)"
STAGE="$(mktemp -d)"
trap 'rm -rf -- "$STAGE"' EXIT
(
  cd "$SRC"
  printf '%s\n' package.xml setup.py setup.cfg methods/lidar-vo-fusion/config/pose_fusion.example.yaml \
    course/maps/course_v3_map.json course/maps/course_v3_rules.json
  find resource launch routes -type f -not -path '*/__pycache__/*'
  # the Python packages setup.py installs: each folder with an __init__.py, its .py files only
  find dronehacks_lidar pose_fusion follower course -name __init__.py -not -path '*/__pycache__/*' |
    while read -r init; do find "$(dirname "$init")" -maxdepth 1 -name '*.py'; done
) > "$STAGE/files.txt"
mkdir -p "$STAGE/pkg" "$DEST"
rsync -a --files-from="$STAGE/files.txt" "$SRC/" "$STAGE/pkg/"
rsync -a --delete "$STAGE/pkg/" "$DEST/"
echo "  $(wc -l < "$STAGE/files.txt") files"

DOCKERFILE="$HOME/team_ws/Dockerfile"
echo "[2/2] pyserial in $DOCKERFILE"
if [ ! -f "$DOCKERFILE" ]; then
  echo "  no team Dockerfile yet: run  ~/skybeta/host/run.sh build --init  then this script again" >&2
  exit 3
fi
if grep -q "pyserial" "$DOCKERFILE"; then
  echo "  already there"
else
  echo 'RUN pip3 install pyserial==3.5' >> "$DOCKERFILE"
  echo "  added: RUN pip3 install pyserial==3.5"
fi

cat <<'NEXT'

Done. Remaining steps (methods/lidar-vo-fusion/docs/deploy.md):
  ~/skybeta/host/run.sh build                      # the team image, with pyserial
  sudoedit /etc/skybeta/drone.env                  # COMPETITOR_PKG=dronehacks_lidar
                                                   # COMPETITOR_LAUNCH=one of, for this run:
                                                   #   lidar_nav_allowed.launch.py          lidar allowed,   window route
                                                   #   lidar_nav_allowed_gate_b.launch.py   lidar allowed,   Gate B route
                                                   #   lidar_nav_forbidden.launch.py        lidar forbidden, window route
                                                   #   lidar_nav_forbidden_gate_b.launch.py lidar forbidden, Gate B route
  # measure the course->odom transform for this run (see deploy.md step 2), fill it into the
  # params file, put the drone on the launch mark, then restart the stack:
  docker rm -f skybeta-stack
NEXT
