#!/usr/bin/env bash
# methods/cuvslam-vio/tools/setup_wsl2.sh - re-runnable setup of PyCuVSLAM v17.0.0 (cu12, cp310) inside WSL2
# Ubuntu 22.04 on the Windows laptop (RTX 1000 Ada, Windows driver 581.95).
#
# What it does (each step is skipped when already done, so re-running is safe):
#   1. sanity checks: Linux x86_64, Ubuntu 22.04, Python 3.10, the GPU is visible through
#      nvidia-smi (the Windows driver, mapped in by WSL2; this script NEVER installs a driver)
#   2. apt: python3-venv, python3-pip, git, curl
#   3. NVIDIA's WSL-Ubuntu apt repo (cuda-keyring) and the `cuda-toolkit-12-6` metapackage ONLY
#      ("DO NOT choose the cuda, cuda-12-x, or cuda-drivers meta-packages under WSL 2 as these
#      packages will result in an attempt to install the Linux NVIDIA driver under WSL 2.
#      Install the cuda-toolkit-12-x metapackage only." - NVIDIA CUDA on WSL User Guide,
#      https://docs.nvidia.com/cuda/wsl-user-guide/index.html)
#   4. a venv on the Linux file system (default ~/venvs/cuvslam; /mnt/c is slow and the
#      Windows .venv in the repo cannot be reused)
#   5. the x86_64 wheel from the v17.0.0 GitHub release, checked against its sha256 digest
#      (digest from the GitHub release API, re-verified by downloading the file on 2026-09-19)
#   6. the NVIDIA example dependencies (pins from examples/requirements.txt at tag v17.0.0,
#      numpy raised to 2.2.6 = what this repo pins for Python < 3.12), depthai 3.10.0, evo;
#      rerun-sdk only with WITH_RERUN=1
#   7. a smoke test (import cuvslam, get_version, warm_up_gpu) and methods/cuvslam-vio/tools/check_env.py
#
# Run inside WSL (Ubuntu-22.04), from any directory:
#   bash "/mnt/c/<path to the repo checkout>/methods/cuvslam-vio/tools/setup_wsl2.sh"
#
# Environment overrides (all optional):
#   CUVSLAM_VENV=~/venvs/cuvslam     where the venv lives (Linux file system recommended)
#   CUVSLAM_WHEEL_DIR=~/Downloads/cuvslam   where the wheel is cached
#   WITH_DEPTHAI=1                   install depthai==3.10.0 too (needed for the later usbipd
#                                    live-camera path and for the repo's tests); 0 to skip
#   WITH_RERUN=0                     1 installs rerun-sdk==0.33.1 (140 MB; viewer needs WSLg)
#   SKIP_APT=0                       1 skips every apt step (no sudo needed then)
#
# Sources:
#   - wheel table, CUDA prerequisite ("CUDA Toolkit 12 or 13 must be installed separately
#     ... Its major version must match the wheel's cu12 or cu13 tag"):
#     https://github.com/nvidia-isaac/cuVSLAM/blob/v17.0.0/README.md  (NVlabs/PyCuVSLAM
#     redirects here with HTTP 301)
#   - release assets + sha256 digests: https://github.com/nvidia-isaac/cuVSLAM/releases/tag/v17.0.0
#   - examples/requirements.txt at v17.0.0: numpy==2.2.4 opencv-python~=4.11.0 pillow==12.2.0
#     pyyaml==6.0.2 rerun-sdk==0.33.1 scipy==1.14.1
#   - NVIDIA CUDA on WSL User Guide: https://docs.nvidia.com/cuda/wsl-user-guide/index.html
#   - WSL-Ubuntu apt repo (cuda-keyring, cuda-toolkit-12-6 up to 12.6.3-1):
#     https://developer.download.nvidia.com/compute/cuda/repos/wsl-ubuntu/x86_64/
#   - CUDA 12.x needs driver >= 525 (CUDA Toolkit release notes, minor version compatibility):
#     https://docs.nvidia.com/cuda/cuda-toolkit-release-notes/index.html
#   - WSL file-system performance: https://learn.microsoft.com/en-us/windows/wsl/filesystems
set -euo pipefail

# --------------------------------------------------------------------------- constants
CUVSLAM_VERSION="17.0.0"
WHEEL_NAME="cuvslam-17.0.0+cu12-cp310-cp310-manylinux_2_35_x86_64.whl"
# The GitHub URL encodes the '+' of the local version tag as %2B.
WHEEL_URL="https://github.com/nvidia-isaac/cuVSLAM/releases/download/v17.0.0/cuvslam-17.0.0%2Bcu12-cp310-cp310-manylinux_2_35_x86_64.whl"
WHEEL_SHA256="0ae3113e43902e0c15423992450510aef241f84207718ed77a1d2d39f28744c1"
WHEEL_SIZE=116412244
CUDA_MAJOR_MINOR="12.6"
CUDA_APT_PKG="cuda-toolkit-12-6"
CUDA_REPO="https://developer.download.nvidia.com/compute/cuda/repos/wsl-ubuntu/x86_64"
CUDA_KEYRING_DEB="cuda-keyring_1.1-1_all.deb"
MIN_DRIVER_MAJOR=525          # CUDA 12.x minor-version compatibility floor (release notes)

# Pinned Python packages (see header). opencv 4.11 is what NVIDIA validated the examples on.
PY_PINS=(
  "numpy==2.2.6"
  "opencv-python~=4.11.0"
  "scipy==1.14.1"
  "pyyaml==6.0.2"
  "pillow==12.2.0"
  "evo"
)

VENV="${CUVSLAM_VENV:-$HOME/venvs/cuvslam}"
WHEEL_DIR="${CUVSLAM_WHEEL_DIR:-$HOME/Downloads/cuvslam}"
WITH_DEPTHAI="${WITH_DEPTHAI:-1}"
WITH_RERUN="${WITH_RERUN:-0}"
SKIP_APT="${SKIP_APT:-0}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"

log()  { printf '\n==> %s\n' "$*"; }
warn() { printf 'WARNING: %s\n' "$*" >&2; }
die()  { printf 'ERROR: %s\n' "$*" >&2; exit 1; }

# --------------------------------------------------------------------------- 1. sanity
log "Checking the host"
[ "$(uname -s)" = "Linux" ] || die "this script is for Linux (WSL2); run it inside Ubuntu-22.04"
[ "$(uname -m)" = "x86_64" ] || die "expected x86_64, got $(uname -m); use methods/cuvslam-vio/tools/setup_jetson.sh on the Jetson"

# shellcheck disable=SC1091
. /etc/os-release
if [ "${ID:-}" != "ubuntu" ] || [ "${VERSION_ID:-}" != "22.04" ]; then
  die "expected Ubuntu 22.04 (the cp310 / manylinux_2_35 wheel target), got ${PRETTY_NAME:-unknown}"
fi

if grep -qi microsoft /proc/version 2>/dev/null; then
  echo "WSL2 kernel detected: $(uname -r)"
else
  warn "this does not look like WSL2 ($(uname -r)). The script still works on a bare-metal"
  warn "Ubuntu 22.04 box, but then YOU must have installed the NVIDIA Linux driver first."
fi

PY_SYS="$(command -v python3 || true)"
[ -n "$PY_SYS" ] || die "python3 not found; on Ubuntu 22.04 it is preinstalled (3.10)"
PY_VER="$("$PY_SYS" -c 'import sys; print("%d.%d" % sys.version_info[:2])')"
[ "$PY_VER" = "3.10" ] || die "the wheel is cp310 only; system python3 is $PY_VER"
echo "python3: $PY_SYS ($("$PY_SYS" --version 2>&1))"

# The Windows driver is the only driver. WSL maps its tools/libraries in under /usr/lib/wsl/lib.
SMI="$(command -v nvidia-smi || true)"
[ -n "$SMI" ] || [ ! -x /usr/lib/wsl/lib/nvidia-smi ] || SMI=/usr/lib/wsl/lib/nvidia-smi
if [ -z "$SMI" ]; then
  die "nvidia-smi not found. In WSL2 that means the Windows NVIDIA driver is missing or too old:
  install/update it from https://www.nvidia.com/drivers (R495 or later; this laptop has 581.95),
  then in PowerShell run: wsl --update ; wsl --shutdown   and re-open Ubuntu. Never install a
  Linux NVIDIA driver inside WSL (NVIDIA CUDA on WSL User Guide)."
fi
DRIVER="$("$SMI" --query-gpu=driver_version --format=csv,noheader 2>/dev/null | head -n1 | tr -d ' ' || true)"
GPU_NAME="$("$SMI" --query-gpu=name --format=csv,noheader 2>/dev/null | head -n1 || true)"
echo "nvidia-smi: $SMI  driver=$DRIVER  gpu=$GPU_NAME"
if [ -n "$DRIVER" ]; then
  DRIVER_MAJOR="${DRIVER%%.*}"
  if [ "$DRIVER_MAJOR" -lt "$MIN_DRIVER_MAJOR" ] 2>/dev/null; then
    die "Windows driver $DRIVER is older than $MIN_DRIVER_MAJOR; CUDA 12.x needs >= $MIN_DRIVER_MAJOR"
  fi
fi
if [ -e /usr/lib/wsl/lib/libcuda.so.1 ]; then
  echo "libcuda.so.1: /usr/lib/wsl/lib/libcuda.so.1 (Windows driver stub, as expected)"
fi

# --------------------------------------------------------------------------- 2. apt basics
if [ "$SKIP_APT" != "1" ]; then
  log "Installing base packages (sudo)"
  export DEBIAN_FRONTEND=noninteractive
  sudo apt-get update -qq
  sudo apt-get install -y -qq python3-venv python3-pip git curl ca-certificates
fi

# --------------------------------------------------------------------------- 3. CUDA toolkit
if [ -x "/usr/local/cuda-${CUDA_MAJOR_MINOR}/bin/nvcc" ]; then
  log "CUDA toolkit ${CUDA_MAJOR_MINOR} already installed: $(/usr/local/cuda-${CUDA_MAJOR_MINOR}/bin/nvcc --version | grep -o 'V[0-9.]*')"
elif [ "$SKIP_APT" = "1" ]; then
  die "CUDA toolkit ${CUDA_MAJOR_MINOR} is missing and SKIP_APT=1; run once without SKIP_APT"
else
  log "Installing NVIDIA's WSL-Ubuntu apt repo and ${CUDA_APT_PKG} (toolkit only, no driver)"
  if ! dpkg -s cuda-keyring >/dev/null 2>&1; then
    TMP_DEB="$(mktemp --suffix=.deb)"
    curl -fsSL "${CUDA_REPO}/${CUDA_KEYRING_DEB}" -o "$TMP_DEB"
    sudo dpkg -i "$TMP_DEB"
    rm -f "$TMP_DEB"
  fi
  sudo apt-get update -qq
  # cuda-toolkit-12-6 pulls cuda-compiler, cuda-libraries (libcublas/libcusolver/libcusparse:
  # the three the wheel needs), cuda-libraries-dev, cuda-tools, documentation and nvml-dev.
  sudo apt-get install -y -qq "$CUDA_APT_PKG"
fi

# Make nvcc reachable in this shell and in future ones (the deb does not touch PATH).
CUDA_BIN="/usr/local/cuda-${CUDA_MAJOR_MINOR}/bin"
case ":$PATH:" in *":$CUDA_BIN:"*) ;; *) export PATH="$CUDA_BIN:$PATH" ;; esac
if ! grep -qs "cuda-${CUDA_MAJOR_MINOR}/bin" "$HOME/.bashrc"; then
  printf '\n# added by DroneDepthCamTesting/methods/cuvslam-vio/tools/setup_wsl2.sh\nexport PATH=%s${PATH:+:${PATH}}\n' "$CUDA_BIN" >> "$HOME/.bashrc"
  echo "PATH line added to ~/.bashrc"
fi
# The runtime libraries the wheel links against must be visible to the dynamic loader
# (the .deb drops /etc/ld.so.conf.d/000_cuda.conf, so ldconfig finds them without LD_LIBRARY_PATH).
for lib in libcublas.so.12 libcusolver.so.11 libcusparse.so.12; do
  if ldconfig -p | grep -q "$lib "; then
    echo "found $lib: $(ldconfig -p | grep "$lib " | head -n1 | awk '{print $NF}')"
  else
    warn "$lib not visible to ldconfig; cuvslam will fail to import. Check ${CUDA_APT_PKG} installed and run: sudo ldconfig"
  fi
done

# --------------------------------------------------------------------------- 4. venv
log "Python venv at $VENV"
case "$VENV" in
  /mnt/*) warn "venv on /mnt/* (Windows drive) is slow; prefer the Linux file system, e.g. ~/venvs/cuvslam" ;;
esac
if [ ! -x "$VENV/bin/python" ]; then
  mkdir -p "$(dirname "$VENV")"
  "$PY_SYS" -m venv "$VENV"
fi
PY="$VENV/bin/python"
"$PY" -m pip install --quiet --upgrade pip

# --------------------------------------------------------------------------- 5. wheel
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

# --------------------------------------------------------------------------- 6. deps
log "Example dependencies and evo"
"$PY" -m pip install --quiet "${PY_PINS[@]}"
if [ "$WITH_DEPTHAI" = "1" ]; then
  "$PY" -m pip install --quiet "depthai==3.10.0"
fi
if [ "$WITH_RERUN" = "1" ]; then
  "$PY" -m pip install --quiet "rerun-sdk==0.33.1"
fi

# --------------------------------------------------------------------------- 7. smoke test
log "Smoke test"
"$PY" - <<'PY'
import cuvslam
ver = cuvslam.get_version()          # (semver string, major, minor, patch) in v17.0.0
print("cuvslam.get_version():", ver)
print("cuvslam.__version__:", cuvslam.__version__)
cuvslam.warm_up_gpu()                # raises if CUDA / cuSOLVER / cuBLAS cannot initialise
print("warm_up_gpu(): ok")
PY

if [ -f "$REPO_DIR/methods/cuvslam-vio/tools/check_env.py" ]; then
  log "methods/cuvslam-vio/tools/check_env.py"
  "$PY" "$REPO_DIR/methods/cuvslam-vio/tools/check_env.py"
fi

log "Done"
cat <<EOF
Activate:      source "$VENV/bin/activate"
Repo (slow):   "$REPO_DIR"
Datasets:      copy recordings to the Linux file system before tracking, e.g.
               mkdir -p ~/datasets && cp -r "$REPO_DIR/recordings/<name>" ~/datasets/
Track:         python "$REPO_DIR/methods/cuvslam-vio/08_cuvslam_track.py" ~/datasets/<name> --mode stereo
EOF
