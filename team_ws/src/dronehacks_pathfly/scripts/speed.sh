#!/usr/bin/env bash
# Pick the pathfly speed profile for the NEXT run. Run on the drone host:
#
#   bash ~/team_ws/src/dronehacks_pathfly/scripts/speed.sh          # show
#   bash ~/team_ws/src/dronehacks_pathfly/scripts/speed.sh fast     # same path, faster
#   bash ~/team_ws/src/dronehacks_pathfly/scripts/speed.sh safe     # the margin-first speeds
#
# From the laptop: ssh skybeta11 'bash ~/team_ws/src/dronehacks_pathfly/scripts/speed.sh fast'
#
# Both fly the same planned path (path-planning 11db65f, nose east). The
# choice is the file ~/team_ws/pathfly_speed, read once when pathfly.launch.py
# starts, so it takes effect at the next stack start: power-cycle the drone
# sitting on START (as between every flight). The competitor window then
# prints "[pathfly] SPEED FAST" or "[pathfly] SPEED SAFE". Nothing else is
# changed: no drone.env edit, no sudo, no rebuild.
set -eu
f="$HOME/team_ws/pathfly_speed"
case "${1:-}" in
  "")
    if [ -f "$f" ]; then echo "next run: $(cat "$f") (from $f)"
    else echo "next run: safe (no $f)"; fi ;;
  fast|safe)
    tmp="$f.tmp.$$"
    printf '%s\n' "$1" > "$tmp" && mv -f "$tmp" "$f"
    echo "next run: $1 (wrote $f)"
    echo "Takes effect at the next stack start: power-cycle the drone on START,"
    echo "then check the competitor window says [pathfly] SPEED $(echo "$1" | tr a-z A-Z)." ;;
  *)
    echo "usage: $0 [fast|safe]" >&2; exit 2 ;;
esac
