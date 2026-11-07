#!/usr/bin/env bash
# Install the ROS 2 package dronehacks_lidar into drone 1's team workspace.
#
# Run it on the drone, in the host shell, from a checkout of this repo:
#     bash depthcam/deploy/install_on_drone.sh
#
# It refuses to run anywhere but drone 1 (hostname skybeta-01). Logging into or deploying to another
# team's drone gets a team disqualified (AGENTS.md §1). It:
#   1. copies depthcam/ (the package root) to ~/team_ws/src/dronehacks_lidar
#   2. adds pyserial to the team image's Dockerfile (~/team_ws/Dockerfile), if missing
# then prints the remaining steps. It does not edit /etc/skybeta/drone.env (that needs sudo, and
# choosing the run's lidar policy is yours). It does not restart the stack either: a restart
# re-zeroes the VIO origin, so do it yourself with the drone on the launch mark, facing north.
# See methods/lidar-vo-fusion/docs/deploy.md.
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

echo "[1/2] $SRC -> $DEST"
mkdir -p "$DEST"
rsync -a --delete --exclude recordings/ --exclude snapshots/ --exclude __pycache__/ --exclude .pytest_cache/ \
      "$SRC/" "$DEST/"

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
  # put the drone on the launch mark, facing north, then restart the stack (re-zeroes the VIO origin):
  docker rm -f skybeta-stack
NEXT
