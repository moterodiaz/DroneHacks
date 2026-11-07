#!/usr/bin/env bash
# Start the bridge server (methods/cuvslam-vio/10_cuvslam_bridge_server.py) inside WSL2 or on the Jetson.
# methods/cuvslam-vio/09_cuvslam_live_view.py runs this through wsl.exe; by hand:
#   bash methods/cuvslam-vio/tools/run_bridge_server.sh [PORT] [VENV] [server flags...]
set -euo pipefail
PORT="${1:-47800}"
VENV="${2:-$HOME/venvs/cuvslam}"
VENV="${VENV/#\~/$HOME}"
shift $(( $# > 2 ? 2 : $# ))
if [ ! -f "$VENV/bin/activate" ]; then
  echo "no Python environment at $VENV (run methods/cuvslam-vio/tools/setup_wsl2.sh or methods/cuvslam-vio/tools/setup_jetson.sh first)" >&2
  exit 3
fi
# shellcheck disable=SC1091
source "$VENV/bin/activate"
cd "$(dirname "$0")/.."
exec python -u 10_cuvslam_bridge_server.py --port "$PORT" "$@"
