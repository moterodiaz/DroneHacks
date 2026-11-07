#!/usr/bin/env bash
# On the drone's Jetson HOST (not inside a container): record one flight into one folder.
#
#   record_flight.sh start [--profile light|full] [--label TEXT] [--zstd|--no-zstd]
#   record_flight.sh stop
#   record_flight.sh status
#
# `start` (before arming) makes ${TEAM_WS}/flights/<run_id>/, starts `ros2 bag record` inside
# the running skybeta container, detached, so it survives this SSH session, then collects the
# metadata in the background. `stop` (after landing and disarm) sends the recorder SIGINT, its
# clean shutdown, waits for the bag to close, and records what changed. Then fetch PX4's .ulg
# into <run_id>/px4/ (README) and run the analyzer.
#
# Profiles (rosbag2 -a with an exclude regex, so topics that appear later are still caught):
#   light  every topic except images, point clouds and paths: a few MB per flight. Default.
#   full   adds the raw stereo images (and depth if published) so cuVSLAM can be replayed;
#          on the order of a GB per 30 s. File-compressed with zstd unless --no-zstd.
#
# Environment: SKYBETA_CONTAINER (default: the running skybeta-stack, else skybeta),
# TEAM_WS (default: drone.env's, else ~/team_ws), COURSE_MAP (container path; default the
# mission's course_map_yaml parameter, else the competitor package's packaged map).
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DRONE_ENV="${SKYBETA_DRONE_ENV:-/etc/skybeta/drone.env}"

# Images, clouds and paths: what made the lab's bag too big to move
LIGHT_EXCLUDE='image|compressed|/depth|points|cloud|/visual_slam/vis/|_path$'
# The full profile keeps the raw images cuVSLAM consumes; the mono and rgb copies are derived
FULL_EXCLUDE='image_mono|image_rect|/oak/rgb/|compressed|points|cloud|/visual_slam/vis/|_path$'

log()  { printf '[flight] %s\n' "$*"; }
die()  { printf '[flight] ERROR: %s\n' "$*" >&2; exit 1; }
usage() { sed -n '2,21p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; }

# drone.env's value for a key, taken literally with `~/` expanded, as host/drone_env.sh reads it
drone_env_value() {
  [[ -r "${DRONE_ENV}" ]] || return 0
  sed -n "s/^$1=//p" "${DRONE_ENV}" | tail -n 1 | sed "s|^~/|${HOME}/|"
}

TEAM_WS="${TEAM_WS:-$(drone_env_value TEAM_WS)}"
TEAM_WS="${TEAM_WS:-${HOME}/team_ws}"
FLIGHTS="${TEAM_WS}/flights"
CURRENT="${FLIGHTS}/.current"

# The container the stack runs in; recording joins its DDS domain and sees its /root/team_ws
container() {
  if [[ -n "${SKYBETA_CONTAINER:-}" ]]; then
    echo "${SKYBETA_CONTAINER}"
    return 0
  fi

  local name
  for name in skybeta-stack skybeta; do
    if docker ps --format '{{.Names}}' | grep -qx "${name}"; then
      echo "${name}"
      return 0
    fi
  done
  return 1
}

# Sourced first in every command run inside the container: ROS, the skybeta underlay, the team overlay
ENV_SETUP='if ! source skybeta_env.sh 2>/dev/null; then set +u; source /opt/ros/humble/setup.bash;
  for overlay in /root/skybeta_ws/install/local_setup.bash /root/team_ws/install/local_setup.bash; do
    if [[ -f "${overlay}" ]]; then source "${overlay}"; fi; done; fi'

# One `key: value` line in the manifest, newlines flattened
manifest() {
  printf '%s: %s\n' "$1" "$(printf '%s' "$2" | tr '\n' ' ')" >> "${DIR}/meta/manifest.txt"
}

# The team repo this script is in: its commit, branch, dirty flag and uncommitted diff
record_team_git() {
  local top
  if top="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel 2>/dev/null)"; then
    manifest team_git_sha "$(git -C "${top}" rev-parse HEAD)"
    manifest team_git_branch "$(git -C "${top}" rev-parse --abbrev-ref HEAD)"
    if [[ -n "$(git -C "${top}" status --porcelain --untracked-files=no)" ]]; then
      manifest team_git_dirty yes
      git -C "${top}" diff HEAD > "${DIR}/meta/team_git.diff" || true
    else
      manifest team_git_dirty no
    fi
  else
    manifest team_git_sha "unknown (${SCRIPT_DIR} is not in a git checkout)"
  fi

  # A content hash of the deployed sources, for code copied onto the drone without git
  manifest team_src_sha256 "$(find "${TEAM_WS}/src" -type f \( -name '*.py' -o -name '*.yaml' \
    -o -name '*.xml' -o -name '*.sh' -o -name '*.cfg' \) -not -path '*/.git/*' -print0 2>/dev/null \
    | sort -z | xargs -0 -r sha256sum | sha256sum | cut -d' ' -f1)"
}

# Host facts: clock sync, disk, identity and the organiser stack's version
record_host() {
  manifest hostname "$(hostname)"
  manifest drone_id "$(drone_env_value DRONE_ID)"
  manifest ros_domain_id "$(drone_env_value ROS_DOMAIN_ID)"
  [[ -r "${DRONE_ENV}" ]] && cp "${DRONE_ENV}" "${DIR}/meta/drone.env"
  { timedatectl status; echo; timedatectl show-timesync --all 2>/dev/null || true; echo
    command -v chronyc >/dev/null && chronyc tracking; } > "${DIR}/meta/timedatectl.txt" 2>&1 || true
  df -h "${FLIGHTS}" / > "${DIR}/meta/df.txt" 2>&1 || true
  manifest disk_free "$(df -h --output=avail "${FLIGHTS}" | tail -n 1 | tr -d ' ')"
  manifest skybeta_version "$(cat "${HOME}/skybeta/VERSION" 2>/dev/null || echo unknown)"
  manifest skybeta_git_sha "$(git -C "${HOME}/skybeta" rev-parse HEAD 2>/dev/null || echo unknown)"
  manifest container "${CONTAINER}"
  manifest image "$(docker inspect --format '{{.Config.Image}}' "${CONTAINER}" 2>/dev/null || echo unknown)"
}

# Inside the container: the ROS graph, every node's parameters, the launch command lines and the
# course map the mission read, copied with its hash
record_graph() {
  docker exec -e COURSE_MAP="${COURSE_MAP:-}" "${CONTAINER}" bash -c "${ENV_SETUP}"'
    out="$1/meta"
    ros2 topic list -t > "${out}/topics_start.txt" 2>&1 || true
    ros2 node list > "${out}/nodes.txt" 2>&1 || true
    mkdir -p "${out}/params"
    while read -r node; do
      [[ "${node}" == /* && "${node}" != /_* ]] || continue
      safe="$(echo "${node#/}" | tr / _)"
      timeout 20 ros2 param dump "${node}" > "${out}/params/${safe}.yaml" 2>&1 || true
    done < "${out}/nodes.txt"
    ps -eo pid,lstart,args | grep -E "ros2 (launch|run)|launch\.py|--ros-args" | grep -v grep \
      > "${out}/processes.txt" || true

    map="${COURSE_MAP}"; why="COURSE_MAP"
    if [[ -z "${map}" ]]; then
      map="$(grep -h -E "^\s*course_map_yaml:" "${out}"/params/*.yaml 2>/dev/null | head -n 1 \
             | sed -E "s/^\s*course_map_yaml:\s*//" | tr -d "\"\047 ")"
      why="the mission parameter course_map_yaml"
    fi
    if [[ -z "${map}" ]]; then
      map="$(ls /root/team_ws/install/${COMPETITOR_PKG:-_}/share/*/maps/*.yaml \
                /root/team_ws/install/*/share/*/maps/*.yaml 2>/dev/null | head -n 1)"
      why="the packaged map (course_map_yaml empty)"
    fi
    if [[ -n "${map}" && -f "${map}" ]]; then
      cp "${map}" "${out}/course_map.yaml"
      printf "course_map: %s\ncourse_map_source: %s\ncourse_map_sha256: %s\n" "${map}" "${why}" \
        "$(sha256sum "${map}" | cut -d" " -f1)" >> "${out}/manifest.txt"
    else
      printf "course_map: none found\n" >> "${out}/manifest.txt"
    fi
  ' _ "${CDIR}"
}

collect_metadata() {
  record_team_git
  record_host
  record_graph || log "graph capture failed: see ${DIR}/meta/collect.log"
  log "metadata collected"
}

start() {
  local profile=light label="" zstd=""
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --profile) profile="${2:-}"; shift 2 ;;
      --label)   label="${2:-}"; shift 2 ;;
      --zstd)    zstd=1; shift ;;
      --no-zstd) zstd=0; shift ;;
      *) usage >&2; die "unknown argument '$1'" ;;
    esac
  done

  local exclude
  case "${profile}" in
    light) exclude="${LIGHT_EXCLUDE}"; zstd="${zstd:-0}" ;;
    full)  exclude="${FULL_EXCLUDE}"; zstd="${zstd:-1}" ;;
    *) die "--profile is light or full, not '${profile}'" ;;
  esac
  [[ "${label}" =~ ^[A-Za-z0-9_.-]*$ ]] || die "--label takes letters, digits, '.', '_' and '-' only"

  if [[ -f "${CURRENT}" ]]; then
    die "a recording is already running ($(cat "${CURRENT}")): record_flight.sh stop first"
  fi

  CONTAINER="$(container)" || die "no running skybeta container: start the stack (host/run.sh stack) first"
  docker exec "${CONTAINER}" test -w /root/team_ws \
    || die "${CONTAINER} has no writable /root/team_ws: the flight folder would not reach the host"

  RUN_ID="$(date -u +%Y%m%d-%H%M%S)-${profile}${label:+-${label}}"
  DIR="${FLIGHTS}/${RUN_ID}"
  CDIR="/root/team_ws/flights/${RUN_ID}"
  mkdir -p "${DIR}/meta" "${DIR}/px4"

  manifest run_id "${RUN_ID}"
  manifest profile "${profile}"
  manifest label "${label}"
  manifest started_utc "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  manifest exclude_regex "${exclude}"
  manifest zstd "${zstd}"

  # The recorder, detached inside the container. `exec` keeps the pid it writes; nothing puts it
  # in the background of a non-interactive shell, so SIGINT is not ignored and stops it cleanly
  local compression=""
  [[ "${zstd}" == 1 ]] && compression="--compression-mode file --compression-format zstd"
  docker exec -d "${CONTAINER}" bash -c "${ENV_SETUP}"'
    echo $$ > "$1/meta/recorder.pid"
    exec ros2 bag record -s mcap -o "$1/bag" -a -x "$2" $3 > "$1/meta/recorder.log" 2>&1
  ' _ "${CDIR}" "${exclude}" "${compression}"

  echo "${RUN_ID}" > "${CURRENT}"
  echo "${CONTAINER}" > "${DIR}/meta/container"

  # Metadata in the background, out of this session's process group so a closed SSH keeps it
  setsid nohup bash "${BASH_SOURCE[0]}" _collect "${DIR}" "${CDIR}" "${CONTAINER}" \
    > "${DIR}/meta/collect.log" 2>&1 < /dev/null &
  echo $! > "${DIR}/meta/collect.pid"

  sleep 3
  if ! docker exec "${CONTAINER}" bash -c 'kill -0 "$(cat "$1/meta/recorder.pid")"' _ "${CDIR}" 2>/dev/null; then
    rm -f "${CURRENT}"
    cat "${DIR}/meta/recorder.log" >&2 || true
    die "the recorder exited at once: its log is above"
  fi

  log "recording ${profile} into ${DIR}"
  log "arm when ready; after landing and disarm: $0 stop"
}

stop() {
  [[ -f "${CURRENT}" ]] || die "no recording is running (no ${CURRENT})"
  RUN_ID="$(cat "${CURRENT}")"
  DIR="${FLIGHTS}/${RUN_ID}"
  CDIR="/root/team_ws/flights/${RUN_ID}"
  CONTAINER="$(cat "${DIR}/meta/container")"

  # SIGINT is rosbag2's clean shutdown: it flushes, writes metadata.yaml and compresses
  log "stopping the recorder (zstd compression of a full bag can take a minute)"
  docker exec "${CONTAINER}" bash -c '
    pid="$(cat "$1/meta/recorder.pid")"
    kill -INT "${pid}" 2>/dev/null || exit 0
    for _ in $(seq 1 3000); do kill -0 "${pid}" 2>/dev/null || exit 0; sleep 0.1; done
    exit 1
  ' _ "${CDIR}" || log "WARNING: the recorder did not exit within 300 s"

  # The metadata job must finish before its files are handed back
  local collector
  collector="$(cat "${DIR}/meta/collect.pid" 2>/dev/null || true)"
  if [[ -n "${collector}" ]]; then
    for _ in $(seq 1 120); do kill -0 "${collector}" 2>/dev/null || break; sleep 0.5; done
  fi

  manifest stopped_utc "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  docker exec "${CONTAINER}" bash -c "${ENV_SETUP}"'; ros2 topic list -t > "$1/meta/topics_end.txt" 2>&1' \
    _ "${CDIR}" || true
  if [[ -f "${DIR}/bag/metadata.yaml" ]]; then
    manifest bag_closed_cleanly yes
  else
    manifest bag_closed_cleanly "no (no metadata.yaml: run 'ros2 bag reindex ${CDIR}/bag' in the container)"
  fi
  manifest bag_size "$(du -sh "${DIR}/bag" 2>/dev/null | cut -f1)"

  # Files the container wrote are root's; hand them to the host user
  docker exec "${CONTAINER}" chown -R "$(id -u):$(id -g)" "${CDIR}" 2>/dev/null || true
  rm -f "${CURRENT}"

  log "wrote ${DIR} ($(du -sh "${DIR}" | cut -f1))"
  log "next: fetch PX4's log into ${DIR}/px4/ (README), then analyze it"
}

status() {
  if [[ ! -f "${CURRENT}" ]]; then
    log "not recording"
    return 0
  fi

  RUN_ID="$(cat "${CURRENT}")"
  DIR="${FLIGHTS}/${RUN_ID}"
  log "recording ${RUN_ID}: $(du -sh "${DIR}/bag" 2>/dev/null | cut -f1) so far"
  tail -n 3 "${DIR}/meta/recorder.log" 2>/dev/null || true
}

case "${1:-}" in
  start)  shift; start "$@" ;;
  stop)   stop ;;
  status) status ;;
  _collect) DIR="$2"; CDIR="$3"; CONTAINER="$4"; collect_metadata ;;
  -h|--help|"") usage ;;
  *) usage >&2; die "unknown command '$1'" ;;
esac
