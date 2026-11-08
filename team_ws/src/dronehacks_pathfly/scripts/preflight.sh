#!/usr/bin/env bash
# Read-only preflight for dronehacks_pathfly on drone 11. Run on the drone
# host (not in the container), after power-on with the drone on START:
#
#   bash ~/team_ws/src/dronehacks_pathfly/scripts/preflight.sh                  (on the drone)
#   ssh skybeta11 'bash ~/team_ws/src/dronehacks_pathfly/scripts/preflight.sh'  (from the laptop)
#
# It changes nothing: no setpoints, no parameters, no files. It ends with
# READY or NOT READY and the list of what failed.
set -u
fails=()
pass() { printf '  [ ok ] %s\n' "$1"; }
fail() { printf '  [FAIL] %s\n' "$1"; fails+=("$1"); }
info() { printf '  [info] %s\n' "$1"; }

echo "== identity / config"
h=$(hostname)
[ "$h" = "skybeta-11" ] && pass "hostname $h" || { fail "hostname is $h, not skybeta-11 - LOG OUT"; }
pkg=$(grep -E '^COMPETITOR_PKG=' /etc/skybeta/drone.env | cut -d= -f2)
lch=$(grep -E '^COMPETITOR_LAUNCH=' /etc/skybeta/drone.env | cut -d= -f2)
[ "$pkg" = "dronehacks_pathfly" ] && pass "COMPETITOR_PKG=$pkg" || fail "COMPETITOR_PKG=$pkg (expected dronehacks_pathfly)"
case "$lch" in pathfly.launch.py|stage[123].launch.py) pass "COMPETITOR_LAUNCH=$lch";; *) fail "COMPETITOR_LAUNCH=$lch";; esac
stage=$(cat ~/team_ws/pathfly_stage 2>/dev/null || echo "(no file = full)")
info "stage file: $stage"
speed=$(cat ~/team_ws/pathfly_speed 2>/dev/null || echo "safe (no file)")
info "speed file: $speed  (scripts/speed.sh fast|safe; takes effect at the next stack start)"
timedatectl 2>/dev/null | grep -q 'synchronized: yes' && pass "clock synchronized" || fail "clock not synchronized (stack may still restart once)"

echo "== stack"
up=$(docker ps --filter name=skybeta-stack --format '{{.Status}}')
[ -n "$up" ] && pass "skybeta-stack $up" || fail "skybeta-stack not running"
boot_s=$(cut -d. -f1 /proc/uptime)
started=$(docker inspect -f '{{.State.StartedAt}}' skybeta-stack 2>/dev/null)
age=$(( $(date +%s) - $(date -d "$started" +%s 2>/dev/null || echo 0) ))
info "container up ${age}s, Jetson up ${boot_s}s"
[ "$age" -ge 60 ] && pass "stack up >= 60 s" || fail "stack up only ${age}s - keep the drone still and wait"

pane=$(docker exec skybeta-stack tmux capture-pane -p -J -t skybeta:competitor -S -2000 2>/dev/null)
echo "$pane" | grep -q '\[pathfly\] \(FULL MISSION\|STAGED TEST\)' \
  && pass "competitor: $(echo "$pane" | grep -o '\[pathfly\] \(FULL MISSION\|STAGED TEST [0-9]\)' | tail -1)" \
  || fail "competitor window is not running pathfly.launch.py (old stack? power-cycle)"
loaded=$(echo "$pane" | grep -o '\[pathfly\] SPEED [A-Z]*' | tail -1)
want=$(echo "${speed%% *}" | tr a-z A-Z)
if [ -z "$loaded" ]; then
  fail "competitor window shows no [pathfly] SPEED line (pathfly older than the fast profile? redeploy)"
elif [ "$loaded" = "[pathfly] SPEED $want" ]; then
  pass "running speed profile: ${loaded#\[pathfly\] SPEED }"
else
  fail "running ${loaded#\[pathfly\] }, but the speed file says $want - power-cycle on START to load it"
fi
flat=$(echo "$pane" | tr -d '\n')   # tmux wraps long log lines: search them joined
echo "$flat" | grep -q 'NOSE AT LIFT-OFF: *0.0 deg' && pass "node summary: nose east" || fail "node summary line not found (scroll the competitor window)"
echo "$pane" | grep -q 'WARNING: the path was planned' && fail "heading WARNING in summary (start_yaw_deg / hold_heading changed)"
echo "$pane" | grep -q 'build failed' && fail "team_ws build failed (see the competitor window)"
last_div=$(echo "$pane" | grep 'camera and PX4 positions disagree' | tail -1)
[ -n "$last_div" ] && info "seen in log: ${last_div##*: }"

echo "== live checks (inside the container)"
docker exec -i skybeta-stack bash -l > /tmp/pathfly_preflight.out 2>&1 <<'IN'
source /root/team_ws/install/setup.bash 2>/dev/null
echo "STATE=$(timeout 6 ros2 topic echo /skybeta/state --once --field data 2>/dev/null | head -1)"
echo "VO=$(timeout 6 ros2 topic echo /visual_slam/status --once --field vo_state 2>/dev/null | head -1)"
echo "FRAME=$(timeout 6 ros2 topic echo /fmu/in/vehicle_visual_odometry --once --field pose_frame 2>/dev/null | head -1)"
echo "TWIST=$(timeout 4 ros2 topic echo /skybeta/setpoint/twist --once --field twist.linear.z 2>/dev/null | head -1)"
echo "OCM=$(timeout 4 ros2 topic echo /fmu/in/offboard_control_mode --once --field timestamp 2>/dev/null | head -1)"
IN
get() { grep "^$1=" /tmp/pathfly_preflight.out | head -1 | cut -d= -f2-; }
state=$(get STATE)
if [ -z "$state" ]; then
  fail "no /skybeta/state (flight_interface not running?)"
else
  python3 - "$state" <<'PY' > /tmp/pathfly_preflight.state
import json, math, sys
s = json.loads(sys.argv[1].strip().strip("'"))
o, l = s.get('odom_enu'), s.get('local_enu')
print('LINK', s.get('px4_link'))
print('ARMED', s.get('armed'))
print('LANDED', s.get('landed'))
print('OFFBOARD', s.get('offboard_active'))
print('NAV', s.get('nav_state_name'))
if o and l:
    d = [a - b for a, b in zip(o, l)]
    print('DIV %.3f %.3f %.3f %.3f' % (math.sqrt(sum(c * c for c in d)), d[0], d[1], d[2]))
else:
    print('DIV none')
PY
  sv() { grep "^$1 " /tmp/pathfly_preflight.state | cut -d' ' -f2-; }
  [ "$(sv LINK)" = "True" ] && pass "px4_link" || fail "px4_link false (power-cycle the drone)"
  [ "$(sv ARMED)" = "False" ] && pass "disarmed" || fail "armed now - preflight is for a disarmed drone"
  [ "$(sv LANDED)" = "True" ] && pass "PX4 reports landed" || fail "PX4 does not report landed"
  [ "$(sv OFFBOARD)" = "False" ] && pass "not in Offboard" || fail "already in Offboard"
  info "nav_state $(sv NAV)"
  div=$(sv DIV)
  if [ "$div" = "none" ]; then
    fail "no PX4 local position (local_enu null)"
  else
    set -- $div
    if awk "BEGIN{exit !($1 <= 0.30)}"; then
      pass "camera vs PX4 position gap $1 m (limit 0.30)"
    else
      diverged=1
      fail "camera vs PX4 position gap $1 m (dx $2, dy $3, dz $4) > 0.30 - drone upright on START, power-cycle"
    fi
  fi
fi
[ "$(get VO)" = "1" ] && pass "camera tracking vo_state 1" || fail "vo_state '$(get VO)' (1 = tracking)"
[ "$(get FRAME)" = "2" ] && pass "pose_frame 2" || fail "pose_frame '$(get FRAME)' (must be 2 - tell the organisers)"
if [ -n "$(get TWIST)" ]; then
  pass "node is publishing (zero) twists"
elif [ "${diverged:-0}" = 1 ]; then
  fail "node is holding back on purpose (position gap above) - Offboard would be refused"
else
  fail "node is not publishing - Offboard would be refused (see the competitor window)"
fi
[ -n "$(get OCM)" ] && pass "flight_interface heartbeat running" || fail "no offboard_control_mode heartbeat"

echo
if [ ${#fails[@]} -eq 0 ]; then
  echo "READY: safety pilot may arm in Altitude, then switch to Position."
else
  echo "NOT READY (${#fails[@]}):"; for f in "${fails[@]}"; do echo "  - $f"; done
fi
