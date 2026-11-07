#!/usr/bin/env bash
# Install dronehacks_planner + dronehacks_traj into drone 1's team workspace, build them in the running
# skybeta-stack container, and dry-run the whole mission on the map the competitor window will fly.
#
# Run on the drone's HOST shell (not in a container), from a checkout of this repo on branch
# navigation/vo-traj-follower (or main, once merged):
#
#   V_MAX=1.0 REST_HEIGHT=0.07 nav_traj/scripts/install_on_drone.sh
#   nav_traj/scripts/install_on_drone.sh                     # keeps the values the last install set
#   GUARD_SRC=~/DroneHacks-vo/depthcam nav_traj/scripts/install_on_drone.sh    # also install dronehacks_vo (the Guard)
#
# V_MAX (the live MPC_XY_VEL_MAX) and REST_HEIGHT (measured) are written into the installed
# config/traj_nav.example.yaml, so the dry run and the launched mission use the same values. If one
# is not given, the value the last install wrote is kept; the first install uses the checkout's.
# Change other parameters in the checkout's nav_traj/config/traj_nav.example.yaml: the copy in
# $TEAM_WS/src is replaced on every run.
#
# It writes only under $TEAM_WS/src/{dronehacks_planner,dronehacks_traj[,dronehacks_vo]}. It does not
# touch /etc/skybeta/drone.env, PX4 parameters or the stack service: it prints what to set.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TEAM_WS="${TEAM_WS:-$HOME/team_ws}"
CONTAINER="${CONTAINER:-skybeta-stack}"
CONFIG_REL=config/traj_nav.example.yaml
INSTALLED_CONFIG="$TEAM_WS/src/dronehacks_traj/$CONFIG_REL"

say() { printf '\n[install] %s\n' "$*"; }
die() { printf '\n[install] ERROR: %s\n' "$*" >&2; exit 1; }
param() {  # param FILE KEY: KEY's value in a traj_nav config, or nothing
  [[ -f "$1" ]] || return 0
  sed -n -E "s/^[[:space:]]*$2:[[:space:]]*([^[:space:]#]+).*/\1/p" "$1" | head -n 1
}
set_param() {  # set_param FILE KEY VALUE
  sed -i -E "s/^([[:space:]]*$2:[[:space:]]*)[^[:space:]#]+/\1$3/" "$1"
  [[ "$(param "$1" "$2")" == "$3" ]] || die "could not set $2 in $1"
}

[[ -f "$REPO/planner/package.xml" && -f "$REPO/nav_traj/package.xml" ]] || die "run this from a checkout of the repo"
[[ -d "$TEAM_WS" ]] || die "$TEAM_WS does not exist: is this the drone? (set TEAM_WS)"
command -v docker >/dev/null || die "docker not found: run this on the drone's host shell"
docker ps --format '{{.Names}}' | grep -qx "$CONTAINER" || die "container $CONTAINER is not running (sudo systemctl start skybeta-stack)"

# Before the copy replaces it: what the last install set, unless given; then the checkout's.
V_MAX="${V_MAX:-$(param "$INSTALLED_CONFIG" v_max_mps)}"
REST_HEIGHT="${REST_HEIGHT:-$(param "$INSTALLED_CONFIG" rest_height_m)}"
V_MAX="${V_MAX:-$(param "$REPO/nav_traj/$CONFIG_REL" v_max_mps)}"
REST_HEIGHT="${REST_HEIGHT:-$(param "$REPO/nav_traj/$CONFIG_REL" rest_height_m)}"
FLOAT='^[0-9]+\.[0-9]+$'   # ROS 2 refuses an int for a float parameter: 1.0, not 1
[[ "$V_MAX" =~ $FLOAT ]] || die "V_MAX='$V_MAX': give metres per second with a decimal point, e.g. 1.0"
[[ "$REST_HEIGHT" =~ $FLOAT ]] || die "REST_HEIGHT='$REST_HEIGHT': give metres with a decimal point, e.g. 0.07"

copy() {  # copy SRC DST: replace DST with SRC, without build artefacts
  local src="$1" dst="$2"
  mkdir -p "$dst"
  if command -v rsync >/dev/null; then
    rsync -a --delete --exclude '__pycache__' --exclude '*.pyc' "$src/" "$dst/"
  else
    find "$dst" -mindepth 1 -delete
    cp -r "$src/." "$dst/"
    find "$dst" -name '__pycache__' -prune -exec rm -r {} +
  fi
}

say "copying packages into $TEAM_WS/src"
mkdir -p "$TEAM_WS/src"
copy "$REPO/planner" "$TEAM_WS/src/dronehacks_planner"
copy "$REPO/nav_traj" "$TEAM_WS/src/dronehacks_traj"
set_param "$INSTALLED_CONFIG" v_max_mps "$V_MAX"
set_param "$INSTALLED_CONFIG" rest_height_m "$REST_HEIGHT"
echo "[install] $INSTALLED_CONFIG: v_max_mps $V_MAX, rest_height_m $REST_HEIGHT"
PACKAGES="dronehacks_planner dronehacks_traj"
if [[ -n "${GUARD_SRC:-}" ]]; then
  [[ -f "$GUARD_SRC/package.xml" ]] || die "GUARD_SRC=$GUARD_SRC has no package.xml"
  copy "$GUARD_SRC" "$TEAM_WS/src/dronehacks_vo"
  PACKAGES="$PACKAGES dronehacks_vo"
fi
for p in $(find "$TEAM_WS/src" -name package.xml -not -path '*/dronehacks_*' 2>/dev/null); do
  echo "[install] note: other package in the team workspace: $p (team_build builds it too)"
done

say "building $PACKAGES in $CONTAINER"
docker exec "$CONTAINER" bash -c "source skybeta_env.sh && team_build.sh --packages-select $PACKAGES" \
  || die "build failed (see above)"

say "checking the image has what the packages import"
docker exec "$CONTAINER" bash -c "source skybeta_env.sh && python3 -c 'import numpy, yaml, dronehacks_planner, dronehacks_traj; print(\"imports ok\")'" \
  || die "a Python import failed inside the container"

if [[ -f "$TEAM_WS/course_map.yaml" ]]; then
  MAP=/root/team_ws/course_map.yaml
else
  MAP="\$(ros2 pkg prefix dronehacks_traj)/share/dronehacks_traj/maps/course_v3_step.yaml"
fi
say "dry run on the competitor window's map ($MAP), v_max $V_MAX m/s, rest height $REST_HEIGHT m"
docker exec "$CONTAINER" bash -c "source skybeta_env.sh && ros2 run dronehacks_traj dry_run $MAP --rest-height $REST_HEIGHT --v-max $V_MAX" \
  || die "the dry run failed: do not fly"

cat <<EOF

[install] done. Next, by hand:

  1. The installed config and the dry run both used v_max_mps $V_MAX and rest_height_m $REST_HEIGHT.
     If they are not the live MPC_XY_VEL_MAX and the measured rest height, rerun with
     V_MAX=... REST_HEIGHT=... (don't edit the copy in $TEAM_WS/src: the next run replaces it).
  2. In /etc/skybeta/drone.env (sudo):
        COMPETITOR_PKG=dronehacks_traj
        COMPETITOR_LAUNCH=traj_nav_competitor.launch.py       # or traj_nav_competitor_raw.launch.py (no Guard)
  3. Drone still on the launch mark, facing north: docker rm -f $CONTAINER   (the service restarts it)
  4. Props off: docker exec -it $CONTAINER bash -c 'source skybeta_env.sh && ros2 topic echo /mission/status'
     and follow nav_traj/docs/deploy.md section 6 before the first flight.
EOF
