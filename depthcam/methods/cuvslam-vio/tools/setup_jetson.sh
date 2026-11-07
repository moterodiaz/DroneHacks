#!/usr/bin/env bash
# methods/cuvslam-vio/tools/setup_jetson.sh - re-runnable setup of PyCuVSLAM v17.0.0 (cu12, cp310, aarch64) plus
# DepthAI 3.10.0 on a Jetson Orin Nano running JetPack 6.2.x (Jetson Linux 36.4.x, Ubuntu
# 22.04, Python 3.10, CUDA 12.6).
#
# What it does (each step is skipped when already done, so re-running is safe):
#   1. sanity checks: aarch64, /etc/nv_tegra_release (L4T R36), Ubuntu 22.04, Python 3.10
#   2. apt: python3-venv, python3-pip, git, curl
#   3. JetPack components: `sudo apt install nvidia-jetpack` if not present. JetPack 6.2.x
#      "includes CUDA 12.6" (https://developer.nvidia.com/embedded/jetpack-sdk-621) and the
#      r36.4 apt index shows nvidia-jetpack -> nvidia-jetpack-runtime -> nvidia-cuda ->
#      cuda-runtime-12-6 -> cuda-libraries-12-6 -> libcublas-12-6, libcusolver-12-6,
#      libcusparse-12-6, cuda-cudart-12-6 (https://repo.download.nvidia.com/jetson/common/
#      dists/r36.4/main/binary-arm64/Packages). Those are exactly the CUDA libraries the
#      aarch64 wheel's libcuvslam.so links (libcublas.so.12, libcusolver.so.11,
#      libcusparse.so.12; libcudart 12.6 is bundled in the wheel), so NO separate CUDA
#      toolkit install is needed. nvidia-jetpack-dev also brings cuda-toolkit-12-6 (nvcc).
#   4. Luxonis udev rule for the OAK (vendor 03e7), same as camera/docs/jetson-orin-nano-setup.md
#   5. a venv (default <repo>/.venv, no --system-site-packages so JetPack's numpy 1.x and
#      OpenCV 4.5 never leak in)
#   6. the aarch64 wheel from the v17.0.0 GitHub release, checked against its sha256 digest
#      (digest from the GitHub release API, re-verified by downloading the file on 2026-09-19)
#   7. depthai==3.10.0 (manylinux_2_28_aarch64 wheel on PyPI, cp39-cp314), the NVIDIA example
#      dependencies (pins from examples/requirements.txt at tag v17.0.0, numpy raised to
#      2.2.6 = the last numpy with a cp310 aarch64 wheel and what this repo pins for
#      Python < 3.12), evo; rerun-sdk only with WITH_RERUN=1
#   8. jtop (jetson-stats) for load monitoring, installed system-wide with sudo pip3 as its
#      README says (https://github.com/rbonghi/jetson_stats); WITH_JTOP=0 skips it
#   9. a smoke test (import cuvslam, get_version, warm_up_gpu) and methods/cuvslam-vio/tools/check_env.py
#
# Run on the Jetson from the repo checkout:
#   bash ~/DroneDepthCamTesting/methods/cuvslam-vio/tools/setup_jetson.sh
#
# Environment overrides (all optional):
#   CUVSLAM_VENV=<repo>/.venv            where the venv lives
#   CUVSLAM_WHEEL_DIR=~/Downloads/cuvslam where the wheel is cached
#   WITH_RERUN=0                          1 installs rerun-sdk==0.33.1 (135 MB aarch64 wheel)
#   WITH_JTOP=1                           0 skips the jetson-stats install
#   SKIP_APT=0                            1 skips every apt/udev step (no sudo needed then)
#
# Sources:
#   - wheel table ("Jetson Orin | 22.04 (JetPack 6.x) | cp310 | cu12 | aarch64"), "JetPack only
#     ships CUDA runtime libraries by default" (dev packages are needed only to BUILD cuVSLAM):
#     https://github.com/nvidia-isaac/cuVSLAM/blob/v17.0.0/README.md
#   - release assets + sha256 digests: https://github.com/nvidia-isaac/cuVSLAM/releases/tag/v17.0.0
#   - JetPack 6.2.1 (CUDA 12.6, "apt install nvidia-jetpack"): https://developer.nvidia.com/embedded/jetpack-sdk-621
#   - JetPack apt install: https://docs.nvidia.com/jetson/jetpack/install-setup/index.html
#   - depthai 3.10.0 wheels: https://pypi.org/project/depthai/3.10.0/#files
#   - numpy 2.2.6 wheels (cp310 aarch64): https://pypi.org/project/numpy/2.2.6/#files
#   - Luxonis udev rule: https://docs.luxonis.com/hardware/platform/deploy/usb-deployment-guide
#   - jetson-stats: https://github.com/rbonghi/jetson_stats
set -euo pipefail

# --------------------------------------------------------------------------- constants
CUVSLAM_VERSION="17.0.0"
WHEEL_NAME="cuvslam-17.0.0+cu12-cp310-cp310-manylinux_2_35_aarch64.whl"
# The GitHub URL encodes the '+' of the local version tag as %2B.
WHEEL_URL="https://github.com/nvidia-isaac/cuVSLAM/releases/download/v17.0.0/cuvslam-17.0.0%2Bcu12-cp310-cp310-manylinux_2_35_aarch64.whl"
WHEEL_SHA256="658250d97c96c5db0d61ed68d8c0e99514c130574339367f0d917ad60f05d315"
WHEEL_SIZE=113375996
CUDA_MAJOR_MINOR="12.6"
UDEV_RULE_FILE="/etc/udev/rules.d/80-movidius.rules"
UDEV_RULE='SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"'

# Pinned Python packages (see header). opencv 4.11 is what NVIDIA validated the examples on;
# the repo's requirements.txt pin (5.0.0.93) also runs the repo scripts, but keep one version.
PY_PINS=(
  "depthai==3.10.0"
  "numpy==2.2.6"
  "opencv-python~=4.11.0"
  "scipy==1.14.1"
  "pyyaml==6.0.2"
  "pillow==12.2.0"
  "evo"
)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"
VENV="${CUVSLAM_VENV:-$REPO_DIR/.venv}"
WHEEL_DIR="${CUVSLAM_WHEEL_DIR:-$HOME/Downloads/cuvslam}"
WITH_RERUN="${WITH_RERUN:-0}"
WITH_JTOP="${WITH_JTOP:-1}"
SKIP_APT="${SKIP_APT:-0}"

log()  { printf '\n==> %s\n' "$*"; }
warn() { printf 'WARNING: %s\n' "$*" >&2; }
die()  { printf 'ERROR: %s\n' "$*" >&2; exit 1; }

# --------------------------------------------------------------------------- 1. sanity
log "Checking the host"
[ "$(uname -s)" = "Linux" ] || die "this script is for Linux on a Jetson"
[ "$(uname -m)" = "aarch64" ] || die "expected aarch64, got $(uname -m); use methods/cuvslam-vio/tools/setup_wsl2.sh on the laptop"
[ -f /etc/nv_tegra_release ] || die "/etc/nv_tegra_release missing: this is not a Jetson (JetPack) image"
L4T_LINE="$(head -n1 /etc/nv_tegra_release)"
echo "L4T: $L4T_LINE"
case "$L4T_LINE" in
  *"R36 "*) ;;
  *) warn "expected Jetson Linux R36.x (JetPack 6.x); the cu12/cp310 wheel targets JetPack 6 only" ;;
esac
if [ -r /proc/device-tree/model ]; then
  echo "model: $(tr -d '\0' < /proc/device-tree/model)"
fi

# shellcheck disable=SC1091
. /etc/os-release
if [ "${ID:-}" != "ubuntu" ] || [ "${VERSION_ID:-}" != "22.04" ]; then
  die "expected Ubuntu 22.04 (JetPack 6.x), got ${PRETTY_NAME:-unknown}. JetPack 7 (24.04, Python 3.12) has no Orin cuvslam wheel."
fi
PY_SYS="$(command -v python3 || true)"
[ -n "$PY_SYS" ] || die "python3 not found"
PY_VER="$("$PY_SYS" -c 'import sys; print("%d.%d" % sys.version_info[:2])')"
[ "$PY_VER" = "3.10" ] || die "the wheel is cp310 only; system python3 is $PY_VER"
echo "python3: $PY_SYS ($("$PY_SYS" --version 2>&1))"

# --------------------------------------------------------------------------- 2. apt basics
if [ "$SKIP_APT" != "1" ]; then
  log "Installing base packages (sudo)"
  export DEBIAN_FRONTEND=noninteractive
  sudo apt-get update -qq
  sudo apt-get install -y -qq python3-venv python3-pip git curl ca-certificates
fi

# --------------------------------------------------------------------------- 3. JetPack / CUDA
log "JetPack components (CUDA ${CUDA_MAJOR_MINOR} runtime)"
if dpkg -s nvidia-jetpack >/dev/null 2>&1; then
  echo "nvidia-jetpack $(dpkg-query -W -f='${Version}' nvidia-jetpack) already installed"
elif [ "$SKIP_APT" = "1" ]; then
  warn "nvidia-jetpack is not installed and SKIP_APT=1; cuvslam needs its CUDA libraries"
else
  echo "installing nvidia-jetpack (several GB; this is the documented 'apt install nvidia-jetpack' step)"
  sudo apt-get install -y nvidia-jetpack
fi
# The three toolkit libraries libcuvslam.so links against (libcudart is bundled in the wheel).
MISSING_LIBS=0
for lib in libcublas.so.12 libcusolver.so.11 libcusparse.so.12; do
  if ldconfig -p | grep -q "$lib "; then
    echo "found $lib: $(ldconfig -p | grep "$lib " | head -n1 | awk '{print $NF}')"
  else
    warn "$lib not visible to ldconfig"
    MISSING_LIBS=1
  fi
done
if [ "$MISSING_LIBS" = "1" ] && [ "$SKIP_APT" != "1" ]; then
  echo "installing cuda-runtime-12-6 (cuda-libraries-12-6) explicitly"
  sudo apt-get install -y -qq cuda-runtime-12-6
  sudo ldconfig
fi
CUDA_BIN="/usr/local/cuda-${CUDA_MAJOR_MINOR}/bin"
if [ -x "$CUDA_BIN/nvcc" ]; then
  echo "nvcc: $("$CUDA_BIN/nvcc" --version | grep -o 'V[0-9.]*')"
  case ":$PATH:" in *":$CUDA_BIN:"*) ;; *) export PATH="$CUDA_BIN:$PATH" ;; esac
  if ! grep -qs "cuda-${CUDA_MAJOR_MINOR}/bin" "$HOME/.bashrc"; then
    printf '\n# added by DroneDepthCamTesting/methods/cuvslam-vio/tools/setup_jetson.sh\nexport PATH=%s${PATH:+:${PATH}}\n' "$CUDA_BIN" >> "$HOME/.bashrc"
    echo "PATH line added to ~/.bashrc"
  fi
else
  echo "nvcc not present (fine: the wheel needs only the runtime libraries; nvidia-jetpack-dev brings cuda-toolkit-12-6 if you ever build cuVSLAM)"
fi

# --------------------------------------------------------------------------- 4. udev rule
if [ "$SKIP_APT" != "1" ]; then
  log "Luxonis udev rule ($UDEV_RULE_FILE)"
  if [ -f "$UDEV_RULE_FILE" ] && grep -qF "$UDEV_RULE" "$UDEV_RULE_FILE"; then
    echo "already present"
  else
    echo "$UDEV_RULE" | sudo tee "$UDEV_RULE_FILE" >/dev/null
    sudo udevadm control --reload-rules && sudo udevadm trigger
    echo "rule written; unplug and re-plug the OAK before using it"
  fi
fi

# --------------------------------------------------------------------------- 5. venv
log "Python venv at $VENV"
if [ ! -x "$VENV/bin/python" ]; then
  "$PY_SYS" -m venv "$VENV"      # deliberately NOT --system-site-packages
fi
PY="$VENV/bin/python"
"$PY" -m pip install --quiet --upgrade pip

# --------------------------------------------------------------------------- 6. wheel
log "PyCuVSLAM wheel"
mkdir -p "$WHEEL_DIR"
WHEEL_PATH="$WHEEL_DIR/$WHEEL_NAME"
if [ -f "$WHEEL_PATH" ] && echo "$WHEEL_SHA256  $WHEEL_PATH" | sha256sum -c --quiet - 2>/dev/null; then
  echo "cached wheel verified: $WHEEL_PATH"
else
  echo "downloading $WHEEL_URL ($WHEEL_SIZE bytes)"
  curl -fL --retry 3 -o "$WHEEL_PATH" "$WHEEL_URL"
  if ! echo "$WHEEL_SHA256  $WHEEL_PATH" | sha256sum -c --quiet -; then
    rm -f "$WHEEL_PATH"
    die "sha256 mismatch for $WHEEL_NAME (expected $WHEEL_SHA256); file removed"
  fi
  echo "sha256 verified"
fi
INSTALLED="$("$PY" -m pip show cuvslam 2>/dev/null | awk '/^Version:/{print $2}' || true)"
if [ "$INSTALLED" = "${CUVSLAM_VERSION}+cu12" ]; then
  echo "cuvslam ${INSTALLED} already installed"
else
  "$PY" -m pip install --quiet "$WHEEL_PATH"
fi

# --------------------------------------------------------------------------- 7. deps
log "depthai, example dependencies and evo"
"$PY" -m pip install --quiet "${PY_PINS[@]}"
if [ "$WITH_RERUN" = "1" ]; then
  "$PY" -m pip install --quiet "rerun-sdk==0.33.1"
fi

# --------------------------------------------------------------------------- 8. jtop
if [ "$WITH_JTOP" = "1" ] && [ "$SKIP_APT" != "1" ]; then
  log "jtop (jetson-stats)"
  if command -v jtop >/dev/null 2>&1; then
    echo "jtop already installed: $(jtop --version 2>/dev/null || echo present)"
  else
    sudo pip3 install -U jetson-stats
    echo "jtop installed; if 'jtop' complains the service is not running, log out and back in (or reboot) once"
  fi
fi

# --------------------------------------------------------------------------- 9. smoke test
log "Smoke test"
"$PY" - <<'PY'
import cuvslam
ver = cuvslam.get_version()          # (semver string, major, minor, patch) in v17.0.0
print("cuvslam.get_version():", ver)
print("cuvslam.__version__:", cuvslam.__version__)
cuvslam.warm_up_gpu()                # raises if CUDA / cuSOLVER / cuBLAS cannot initialise
print("warm_up_gpu(): ok")
import depthai as dai
print("depthai:", dai.__version__)
PY

if [ -f "$REPO_DIR/methods/cuvslam-vio/tools/check_env.py" ]; then
  log "methods/cuvslam-vio/tools/check_env.py"
  "$PY" "$REPO_DIR/methods/cuvslam-vio/tools/check_env.py"
fi

log "Done"
if command -v nvpmodel >/dev/null 2>&1; then
  echo "power mode: $(sudo nvpmodel -q 2>/dev/null | tr '\n' ' ')"
  echo "  (for benchmarking: sudo nvpmodel -m <MAXN SUPER id from /etc/nvpmodel.conf> && sudo jetson_clocks)"
fi
cat <<EOF
Activate:   source "$VENV/bin/activate"
Camera:     re-plug the OAK, then  timeout 90 python "$REPO_DIR/camera/00_device_info.py"
Record:     timeout 120 python "$REPO_DIR/datasets/07_record_dataset.py" --duration 30 --headless
Track:      python "$REPO_DIR/methods/cuvslam-vio/08_cuvslam_track.py" recordings/<name> --mode stereo
Live:       python "$REPO_DIR/methods/cuvslam-vio/08_cuvslam_track.py" --live --mode stereo --headless
EOF
