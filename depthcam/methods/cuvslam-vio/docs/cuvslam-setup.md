# cuVSLAM (PyCuVSLAM v17.0.0) setup: WSL2 on the laptop and the Jetson Orin Nano

How to get NVIDIA's cuVSLAM Python bindings running on the two Linux targets this repo cares about,
and how to prove the install works before any of the repo's own cuVSLAM code is involved.

Researched 2026-09-19 against the GitHub release API, the v17.0.0 tag of the source, the NVIDIA,
Microsoft and Luxonis docs, PyPI and the two NVIDIA apt repositories; every version, size and
digest below is what those sources returned that day. **WSL is not installed on this laptop yet**,
so no WSL command was executed and neither setup script has run on its target; both were
syntax-checked (`bash -n`) and `methods/cuvslam-vio/tools/check_env.py` was run on Windows only. Section 6 lists what
could not be verified.

| Item | Value |
|---|---|
| Library | cuVSLAM 17.0.0, Python package `cuvslam` 17.0.0+cu12, released 2026-07-23 |
| Repository | `NVlabs/PyCuVSLAM` now redirects (HTTP 301) to **`nvidia-isaac/cuVSLAM`**; every file cited below was read at tag `v17.0.0` |
| Laptop target | WSL2 **Ubuntu-22.04** (Python 3.10) on Windows 11 build 26200, RTX 1000 Ada (compute 8.9), Windows driver **581.95** (= `32.0.15.8195` in Device Manager) |
| Laptop wheel | `cuvslam-17.0.0+cu12-cp310-cp310-manylinux_2_35_x86_64.whl`, 116,412,244 bytes, sha256 `0ae3113e43902e0c15423992450510aef241f84207718ed77a1d2d39f28744c1` |
| Laptop CUDA | toolkit **12.6** (`cuda-toolkit-12-6`, 12.6.3-1) from NVIDIA's `wsl-ubuntu` apt repo; the Windows driver is the only driver |
| Drone target | Jetson Orin Nano, JetPack **6.2.x** (Jetson Linux 36.4.x, Ubuntu 22.04, Python 3.10, **CUDA 12.6 already included**) |
| Drone wheel | `cuvslam-17.0.0+cu12-cp310-cp310-manylinux_2_35_aarch64.whl`, 113,375,996 bytes, sha256 `658250d97c96c5db0d61ed68d8c0e99514c130574339367f0d917ad60f05d315` |
| Windows | no `cuvslam` wheel exists (Linux-only); the laptop records datasets with `datasets/07_record_dataset.py` and runs the pytest suite (`tests/`) and `methods/cuvslam-vio/08_cuvslam_track.py --fake-cuvslam` dry runs against `cuvslam_oak/fake_cuvslam.py` (flags re-checked against the scripts, see section 6) |
| Scripts | `methods/cuvslam-vio/tools/setup_wsl2.sh`, `methods/cuvslam-vio/tools/setup_jetson.sh` (bash, idempotent), `methods/cuvslam-vio/tools/check_env.py` (Python, exits 0 always) |
| Independent check | NVIDIA's own `examples/euroc/track_euroc.py` (tag v17.0.0) on one of our EuRoC-layout recordings |

---

## 1. Facts that drive the choices

### 1.1 The release and its wheels

The GitHub release API for tag `v17.0.0` (queried through `gh api repos/NVlabs/PyCuVSLAM/releases/tags/v17.0.0`;
the API answers from the renamed repo) lists these assets. Both `cu12 cp310` wheels were downloaded
to the scratchpad and their sha256 recomputed locally: they match the API's `digest` field.

| Asset | Size (bytes) | sha256 |
|---|---|---|
| `cuvslam-17.0.0+cu12-cp310-cp310-manylinux_2_35_x86_64.whl` (WSL2 / Ubuntu 22.04 desktop) | 116,412,244 | `0ae3113e43902e0c15423992450510aef241f84207718ed77a1d2d39f28744c1` |
| `cuvslam-17.0.0+cu12-cp310-cp310-manylinux_2_35_aarch64.whl` (Jetson Orin, JetPack 6) | 113,375,996 | `658250d97c96c5db0d61ed68d8c0e99514c130574339367f0d917ad60f05d315` |
| `cuvslam-17.0.0+cu12-cp312-abi3-manylinux_2_39_x86_64.whl` (Ubuntu 24.04, not used) | 116,479,970 | `39a6adfedbdf01f8e8e746f93095b1ba04c71e3efe258c1efc0251bab1310018` |
| `cuvslam-17.0.0+cu13-*` (x86_64 cp310/cp312, aarch64 cp312 = Jetson Thor; not used) | 84-90 MB | see release page |
| `cuvslam-cpp-17.0.0-orin-cuda12.6.3-ubuntu22.04.tar.gz`, `...-x86_64-cuda12.6.3-ubuntu22.04.tar.gz` (C++ SDK, not needed) | 113-117 MB | see release page |
| `cuvslam-docs-17.0.0.tar.gz` (offline API docs, `python/index.html`) | 8,054,388 | `d180656ce63e7f6a0397f86fa9297f863f919666f2018de5b87b5b4ce13b80ec` |

Download URLs (the `+` in the version is URL-encoded as `%2B`; the old `NVlabs/PyCuVSLAM` form of
the same URL answers HTTP 301 to these):

- `https://github.com/nvidia-isaac/cuVSLAM/releases/download/v17.0.0/cuvslam-17.0.0%2Bcu12-cp310-cp310-manylinux_2_35_x86_64.whl`
- `https://github.com/nvidia-isaac/cuVSLAM/releases/download/v17.0.0/cuvslam-17.0.0%2Bcu12-cp310-cp310-manylinux_2_35_aarch64.whl`

The README's wheel table at v17.0.0: "Desktop/server | 22.04 | cp310 (Python 3.10) | cu12, cu13 |
x86_64" and "Jetson Orin | 22.04 (JetPack 6.x) | cp310 (Python 3.10) | cu12 | aarch64"; "Only the
combinations listed above are provided as pre-built wheels"; "**Prerequisite**: CUDA Toolkit 12 or
13 must be installed separately (not included in the wheels). Its major version must match the
wheel's cu12 or cu13 tag." The Orin has no cu13 wheel, so **cu12 on both hosts** keeps the laptop
and the drone on the same build.

### 1.2 What is inside the wheel (unpacked, not guessed)

Both `cu12 cp310` wheels contain the same 12 files (16 zip entries, 4 of them directories):
`cuvslam/__init__.py`, `libcuvslam.so` (170.9 MB x86_64 / 161.5 MB aarch64),
`pycuvslam.cpython-310-*-linux-gnu.so`, `pycuvslam.pyi`, `tracker.py`, `utils.py`, an empty
`py.typed` marker, one bundled runtime `cuvslam.libs/libcudart-*.so.12.6.77` (x86_64) /
`libcudart-*.so.12.6.68` (aarch64), and `cuvslam-17.0.0+cu12.dist-info/` holding `METADATA`,
`WHEEL`, `RECORD` and `sboms/auditwheel.cdx.json`. `METADATA` declares `Requires-Python: >=3.9` and exactly one
dependency, `pyyaml>=5.3.1`; numpy is **not** pinned by the wheel. Wheel tag
`cp310-cp310-manylinux_2_35_*` means glibc >= 2.35 (Ubuntu 22.04 has 2.35).

Library names embedded in `libcuvslam.so` (both architectures): `libcublas.so.12`,
`libcusolver.so.11`, `libcusparse.so.12`, `libcudart.so.12`, `libstdc++.so.6`, `libgcc_s.so.1`,
`libc.so.6`, `libm.so.6`; the bundled `libcudart` needs `libcuda.so.1` (the driver). So the CUDA
toolkit only has to supply **cuBLAS 12, cuSOLVER 11 and cuSPARSE 12**, i.e. any CUDA 12.x
`cuda-libraries` set; `nvcc` is never used at run time.

### 1.3 Which CUDA 12.x, and why 12.6

- cuVSLAM's own prebuilt C++ archives are `cuda12.6.3`, the bundled cudart is 12.6.77 and JetPack
  6.2.x ships CUDA 12.6, so 12.6 puts the laptop and the drone on the same runtime.
- NVIDIA's `wsl-ubuntu` apt index (fetched 2026-09-19, 2.9 MB) offers `cuda-toolkit-12-0` ...
  `12-6`, `12-8`, `12-9`, `13-0` ... `13-3`; the newest `cuda-toolkit-12-6` is **12.6.3-1**. Its
  `Depends` are `cuda-compiler-12-6, cuda-libraries-12-6, cuda-libraries-dev-12-6, cuda-tools-12-6,
  cuda-documentation-12-6, cuda-nvml-dev-12-6`, and `cuda-libraries-12-6` pulls `libcublas-12-6`,
  `libcusolver-12-6`, `libcusparse-12-6`, `cuda-cudart-12-6` (plus cuFFT, cuRAND, NPP, nvJPEG,
  nvJitLink, NVRTC, OpenCL). The metapackage `.deb` also ships
  `/usr/local/cuda-12.6/version.json` (`{"cuda": {"name": "CUDA SDK", "version": "12.6.3"}, ...}`;
  its component list has `cuda_nvcc` 12.6.85 and `cuda_cudart` 12.6.77), which is the file
  `methods/cuvslam-vio/tools/check_env.py` reads when `nvcc` is not on `PATH`.
- The repo carries **no** `nvidia-driver-*` / `cuda-drivers` packages at all (only
  `nvidia-driver-assistant`), which is the point of the WSL-Ubuntu repo.

### 1.4 The Windows driver: `32.0.15.8195` is `581.95`, and it supports CUDA 12.x

Measured on this laptop (`nvidia-smi` and WMI, read-only): `Win32_VideoController.DriverVersion =
32.0.15.8195` and `nvidia-smi` reports `NVIDIA-SMI 581.95, Driver Version: 581.95, CUDA Version:
13.0` for `NVIDIA RTX 1000 Ada Generation Laptop GPU`, compute capability 8.9. (The mapping rule:
drop the leading `32.0.1`, the remaining digits `5.8195` read as `581.95`.)

- "CUDA Version: 13.0" in the `nvidia-smi` header is the **newest** toolkit this driver supports.
  Table 3 of NVIDIA's CUDA Toolkit release notes lists 12.x minor-version compatibility for drivers
  **>= 525 and < 580** and 13.x for >= 580; a newer driver always runs older toolkits (CUDA driver
  backward compatibility), and Table 4 of the same notes puts CUDA 12.6 Update 3's own minimum at
  Windows driver >= 561.17, so 581.95 runs the `cu12` wheel and the 12.6 toolkit.
- NVIDIA's CUDA on WSL User Guide: "R495 and later windows will have CUDA support for WSL 2."
- Nothing is installed on Windows for this: no CUDA toolkit is needed on the Windows side, and
  `nvcc` is absent there (see the `check_env.py` output in section 4.1).

### 1.5 Python package pins used on both Linux hosts

| Package | Pin | Why |
|---|---|---|
| `numpy` | **2.2.6** | `examples/requirements.txt` at tag v17.0.0 says `numpy==2.2.4`, `main` says `2.2.6`; the wheel itself pins nothing; `scipy==1.14.1` needs `numpy<2.3`; 2.2.6 is the last numpy with a cp310 aarch64 wheel and what this repo's `requirements.txt` pins for Python < 3.12, so one number works everywhere |
| `opencv-python` | `~=4.11.0` (-> 4.11.0.86, `cp37-abi3` wheels for x86_64 and aarch64) | the v17.0.0 examples pin `~=4.11.0`; this repo's Windows venv has 5.0.0.93, which also runs the repo scripts, but the cuVSLAM venvs stay on the version NVIDIA validated |
| `scipy` | 1.14.1 (cp310 wheels for both arches) | v17.0.0 pin; used by `examples/euroc/dataset_utils.py` (`Rotation.from_matrix`) |
| `pyyaml` | 6.0.2 | v17.0.0 pin; also the wheel's only declared dependency |
| `pillow` | 12.2.0 (requires Python >= 3.10) | v17.0.0 pin; NVIDIA's EuRoC loader reads PNGs with PIL |
| `rerun-sdk` | 0.33.1, **optional** (`WITH_RERUN=1`), 140 MB x86_64 / 135 MB aarch64 `cp310-abi3-manylinux_2_28` wheels | NVIDIA's examples visualise with it; needs a display |
| `evo` | latest (1.37.1, 2026-08-31, requires Python >= 3.10, pure Python) | trajectory evaluation of the TUM files our tracker writes |
| `depthai` | 3.10.0 (`manylinux_2_28_aarch64` and `_x86_64`, cp39-cp314, `Requires-Dist: numpy<3.0.0`) | the camera on the Jetson; on WSL only for the later usbipd path and the repo's tests (`WITH_DEPTHAI=0` skips it) |

---

## 2. WSL2 on this laptop (record on Windows, track recordings in WSL2)

### 2.1 Install WSL2 and Ubuntu 22.04 (PowerShell as Administrator, once)

Microsoft's prerequisite is "Windows 10 version 2004 and higher (Build 19041 and higher) or
Windows 11"; this laptop is Windows 11 Enterprise 10.0.26200. `C:\WINDOWS\system32\wsl.exe` is
present (the stub always is) but WSL itself is not set up here yet.

```powershell
wsl --install -d Ubuntu-22.04
```

- `Ubuntu-22.04` ("Ubuntu 22.04 LTS") is a current name in Microsoft's online distribution
  manifest (`distributions/DistributionInfo.json` in the `microsoft/WSL` repo, which is what
  `wsl --list --online` prints; it also lists `Ubuntu-24.04` and `Ubuntu-26.04`, and plain
  `Ubuntu` is the default). Do **not** take the default `Ubuntu`: the cuvslam wheel is cp310 and
  only Ubuntu 22.04 ships Python 3.10.
- "The above command only works if WSL is not installed at all. If you run `wsl --install` and see
  the WSL help text, please try running `wsl --list --online` to see a list of available distros
  and run `wsl --install -d <DistroName>` to install a distro. If the install process hangs at
  0.0%, run `wsl --install --web-download -d <DistroName>`."
- Restart Windows when asked. On first launch Ubuntu asks for a Unix username and password (this is
  the `sudo` password the setup script needs).
- New installs default to WSL 2; confirm with `wsl -l -v` (the VERSION column must say 2; CUDA and
  GUI apps do not work on WSL 1). `wsl --update` fetches the newest kernel; NVIDIA recommends
  5.10.16.3 or later, and usbipd (section 2.7) needs 5.10.60.1 or later.

### 2.2 GPU in WSL2: the Windows driver is the only driver

From NVIDIA's CUDA on WSL User Guide:

- "This is the only driver you need to install. Do not install any Linux display driver in WSL."
- "Once a Windows NVIDIA GPU driver is installed on the system, CUDA becomes available within WSL 2.
  The CUDA driver installed on Windows host will be stubbed inside the WSL 2 as `libcuda.so`", and
  "Root user on bare metal (not containers) will not find nvidia-smi at the expected location. Use
  `/usr/lib/wsl/lib/nvidia-smi` or manually add `/usr/lib/wsl/lib/` to the PATH".
- Known limitations listed there: "Maxwell GPU is not supported", unified/managed memory is
  limited, "Concurrent CPU/GPU access is not supported", "OpenGL-CUDA Interop is not yet supported".
  None of these affects cuVSLAM.

So inside Ubuntu the check is simply:

```bash
nvidia-smi            # same 581.95 driver, "CUDA Version: 13.0", RTX 1000 Ada
ls /usr/lib/wsl/lib   # libcuda.so, libcuda.so.1, nvidia-smi, ... mapped in from Windows
```

If `nvidia-smi` is missing inside WSL, update the Windows driver (nvidia.com/drivers; R495 or
later, 581.95 is far newer), then `wsl --update` and `wsl --shutdown` from PowerShell. Never
`apt install nvidia-driver-*` inside WSL; `methods/cuvslam-vio/tools/check_env.py` flags a `libcuda.so.1` that does not
come from `/usr/lib/wsl/lib`.

### 2.3 CUDA toolkit 12.6 inside WSL (what `methods/cuvslam-vio/tools/setup_wsl2.sh` step 3 runs)

The guide's rule: "DO NOT choose the `cuda`, `cuda-12-x`, or `cuda-drivers` meta-packages under
WSL 2 as these packages will result in an attempt to install the Linux NVIDIA driver under WSL 2.
Install the `cuda-toolkit-12-x` metapackage only." The WSL-Ubuntu package "will not overwrite the
NVIDIA driver that was already mapped into the WSL 2 environment".

```bash
wget https://developer.download.nvidia.com/compute/cuda/repos/wsl-ubuntu/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb          # adds the repo + signing key (4,328-byte .deb, HTTP 200 on 2026-09-19)
sudo apt-get update
sudo apt-get -y install cuda-toolkit-12-6        # 12.6.3-1; toolkit + libraries, NO driver
export PATH=/usr/local/cuda-12.6/bin${PATH:+:${PATH}}   # nvcc; the .deb does not touch PATH
```

The `.deb`s install under `/usr/local/cuda-12.6` (with the `/usr/local/cuda` symlink, per the CUDA
Linux installation guide) and drop `/etc/ld.so.conf.d/000_cuda.conf`, so `ldconfig -p | grep
libcublas.so.12` finds the libraries without `LD_LIBRARY_PATH`. Smaller alternative if disk is
tight: `cuda-libraries-12-6` alone provides the three libraries the wheel needs (no `nvcc`, no
docs); the script installs the full toolkit because the NVIDIA build instructions and
`nvcc --version` checks assume it.

### 2.4 Python venv, the wheel, example dependencies, evo (steps 4-7 of the script)

Ubuntu 22.04's `python3` is 3.10 (3.10.12 after updates). The repo's Windows `.venv` holds Windows
binaries and cannot be used from Linux, so the script makes a separate venv on the **Linux** file
system (`~/venvs/cuvslam` by default, `CUVSLAM_VENV` to change).

```bash
bash "/mnt/c/<path to the repo checkout>/methods/cuvslam-vio/tools/setup_wsl2.sh"
# optional: WITH_RERUN=1 (rerun viewer), WITH_DEPTHAI=0 (skip depthai), SKIP_APT=1 (re-run without sudo)
```

What it does, in order (each step is skipped when already satisfied, so re-running is safe):
host checks (x86_64, Ubuntu 22.04, Python 3.10, `nvidia-smi` reachable, driver >= 525) -> apt
basics -> keyring + `cuda-toolkit-12-6` (only if `/usr/local/cuda-12.6/bin/nvcc` is absent) ->
`PATH` line in `~/.bashrc` -> `ldconfig` check for `libcublas.so.12`, `libcusolver.so.11`,
`libcusparse.so.12` -> venv + `pip -U pip` -> download the wheel to `~/Downloads/cuvslam/`
(`curl -fL`), verify `sha256sum -c` against the digest above (a mismatch deletes the file and
aborts), `pip install <file>` -> `pip install numpy==2.2.6 "opencv-python~=4.11.0" scipy==1.14.1
pyyaml==6.0.2 pillow==12.2.0 evo` (+ `depthai==3.10.0`, + `rerun-sdk==0.33.1` on request) ->
smoke test (`cuvslam.get_version()`, `cuvslam.warm_up_gpu()`) -> `methods/cuvslam-vio/tools/check_env.py`.

Installing the wheel straight from its GitHub URL (`pip install "<x86_64 wheel URL from section 1.1>"`)
should also work, but the script downloads first so the digest can be checked; that direct form was
not exercised here.

`evo` usage on the TUM files the tracker writes (`timestamp_s tx ty tz qx qy qz qw`, which evo
documents as "Every row has 8 entries containing timestamp (in seconds), position and orientation
(as quaternion) with each value separated by a space"):

```bash
evo_traj tum traj_stereo.tum traj_inertial.tum --plot          # plot needs a display (WSLg)
evo_traj tum traj_stereo.tum                                   # stats only, headless
evo_ape  tum ref.tum traj_stereo.tum -a --plot                 # once a reference exists
```

### 2.5 Reaching the Windows repo from WSL, and where datasets should live

The Windows drive is mounted at `/mnt/c`; the repo path contains spaces, so quote it every time:

```bash
cd "/mnt/c/<path to the repo checkout>"
ln -sfn "/mnt/c/<path to the repo checkout>" ~/DroneDepthCamTesting   # convenience symlink
```

Microsoft: "For the fastest performance speed, store your files in the WSL file system if you are
working in a Linux command line ... It is possible to store your project files on a mounted drive,
but your performance speed will improve if you store them directly on the `\\wsl$` drive." Running
the Python scripts from `/mnt/c` is fine; reading an EuRoC dataset (thousands of PNGs) from
`/mnt/c` is the slow case, so copy recordings to the Linux side before tracking:

```bash
mkdir -p ~/datasets
cp -r ~/DroneDepthCamTesting/recordings/<name> ~/datasets/
python ~/DroneDepthCamTesting/methods/cuvslam-vio/08_cuvslam_track.py ~/datasets/<name> --mode stereo   # see its --help
```

Linux-side files are visible from Windows Explorer at `\\wsl$\Ubuntu-22.04\home\<user>\` (`explorer.exe .`
from a WSL shell opens the current directory). Two more practical notes: the repo folder is inside
OneDrive, so if Files On-Demand keeps `recordings/` online-only, the first read through `/mnt/c`
downloads them (mark the folder "Always keep on this device"); and the Windows Python 3.13 and the
WSL Python 3.10 write different `__pycache__/*.cpython-3xx.pyc` names, so sharing the checkout does
not corrupt either side.

### 2.6 Rerun viewer under WSLg (optional)

NVIDIA's examples call `rr.init("cuVSLAM Visualizer", spawn=True)`, which launches the rerun
viewer window. Windows 11 runs Linux GUI apps through WSLg out of the box ("You will need to be on
Windows 10 Build 19044+ or Windows 11"; an existing install just needs `wsl --update` then
`wsl --shutdown`; the vendor GPU driver on Windows provides the vGPU). Rerun's troubleshooting page
says WSL needs current Mesa Vulkan drivers and offers `rerun --renderer=gl` / `WGPU_BACKEND=gl` and
`unset WAYLAND_DISPLAY` as fallbacks. Not tried here; `08_cuvslam_track.py` does not depend on
rerun, so treat the viewer as a nice-to-have.

### 2.7 Later: the live camera in WSL2 through usbipd-win

Not needed for the plan (Windows records, WSL tracks recordings, the Jetson tracks live), kept
here because it is the only way to run `cuvslam` against the S2 on the laptop.

- Requirements (Microsoft "Connect USB devices"): Windows 11 build 22000+, x64, WSL 2, kernel
  5.10.60.1+ (`wsl --update`). Install usbipd-win with `winget install --interactive --exact
  dorssel.usbipd-win` (latest release v5.3.0, 2025-10-11, `usbipd-win_5.3.0_x64.msi`); it adds a
  `usbipd` service, CLI and firewall rule.
- Attach (PowerShell): `usbipd list` -> `usbipd bind --busid <id>` (administrator, once per
  device) -> `usbipd attach --wsl --busid <id>` (no elevation; keep a WSL shell open) -> `lsusb`
  inside WSL shows `03e7:2485 Intel Movidius MyriadX` -> `usbipd detach --busid <id>` when done.
  "As long as the USB device is attached to WSL, it cannot be used by Windows."
- udev inside WSL: the same rule as the Jetson (`SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7",
  MODE="0666"` in `/etc/udev/rules.d/80-movidius.rules`), then `sudo udevadm control --reload`;
  the usbipd wiki notes that if reload fails with "Failed to send reload request", run
  `sudo service udev restart`. Rules must exist **before** attaching.
- The OAK-specific problem: the camera enumerates as `03e7:2485` (High-Speed) before boot and
  re-enumerates as `03e7:f63b` ("Luxonis Device", SuperSpeed) after the firmware upload, then falls
  back when the app exits (Luxonis issue #723). Over USB/IP that is a detach + re-attach that
  takes ~5 s and a **new bus id**, so a plain attach gives `X_LINK_DEVICE_NOT_FOUND`. usbipd-win
  has `usbipd attach --wsl --busid <id> --auto-attach` ("Automatically re-attach when the device
  is detached or unplugged"); bind both ids, and Luxonis's own workaround is a PowerShell/Python
  loop that re-attaches any `Movidius MyriadX` / `Luxonis Device` row whose state is `Shared`
  (depthai-python issue #938 has the current-syntax version). Luxonis staff also suggested
  `DEPTHAI_BOOTUP_TIMEOUT=15000`, which the reporter found insufficient on its own.
- Limitations: USB/IP tunnels the device over the WSL virtual network, so do not expect USB3
  throughput or use it for timing/latency measurements (no throughput figure was found or
  measured); the device is unavailable to Windows while attached; every boot of the camera is a
  re-attach. Good for functional tests of the live path, not for the bench protocol.

---

## 3. Jetson Orin Nano (JetPack 6.2.x): live tracking

`camera/docs/jetson-orin-nano-setup.md` already covers flashing JetPack 6.2.1 (SD image, Jetson Linux
36.4.4), the firmware caveat for factory boards, the 03e7 udev rule, USB3/`usb=SUPER`, power modes
and the DepthAI scripts. This section adds only what cuVSLAM changes.

### 3.1 CUDA 12.6 is already on the board: no toolkit install

- NVIDIA's JetPack pages: JetPack 6.2 / 6.2.1 / 6.2.3 each "includes CUDA 12.6" (6.2.1: "Jetson
  Linux 36.4.4, featuring the Linux Kernel 5.15 and an Ubuntu 22.04-based root file system ...
  CUDA 12.6, TensorRT 10.3, cuDNN 9.3"; the CUDA paragraph says "includes CUDA 12.6.10"), and
  "To install Jetpack on top of Jetson Linux 36.4.x, all you need to do is `apt install nvidia-jetpack`".
- NVIDIA's Jetson apt index for r36.4 (`repo.download.nvidia.com/jetson/common/dists/r36.4/main/binary-arm64/Packages`,
  fetched 2026-09-19) resolves it: `nvidia-jetpack 6.2.1+b38` -> `nvidia-jetpack-runtime` ->
  `nvidia-cuda 6.2.1+b38` -> `cuda-runtime-12-6 (= 12.6.11-1)` -> `cuda-libraries-12-6` ->
  `libcublas-12-6 12.6.1.4-1`, `libcusolver-12-6 11.6.4.69-1`, `libcusparse-12-6 12.5.3.3-1`,
  `cuda-cudart-12-6 12.6.68`. Those are exactly the three toolkit libraries `libcuvslam.so` links
  (section 1.2); `libcudart` is bundled in the wheel. `nvidia-jetpack-dev` -> `nvidia-cuda-dev`
  -> `cuda-toolkit-12-6 (= 12.6.11-1)` adds `nvcc` under `/usr/local/cuda-12.6/bin` (not on `PATH`
  by default; the setup script adds it).
- The cuVSLAM README agrees: "JetPack only ships CUDA runtime libraries by default. Install the dev
  packages on the Jetson before building: `sudo apt-get install libcublas-dev-12-6
  libcusolver-dev-12-6`" - that is for **building** cuVSLAM, which the wheel makes unnecessary.
  (If you ever build: `./build_release.sh --cuda_arch=87`, "87 for Orin Nano/NX/AGX".)
- Whether the 6.2.1 SD-card image pre-installs these packages was not stated on the pages read
  (the quick-start page now documents the JetPack 7.2 USB-ISO flow), so `methods/cuvslam-vio/tools/setup_jetson.sh`
  checks `dpkg -s nvidia-jetpack` and runs `sudo apt-get install -y nvidia-jetpack` only when it is
  missing (several GB; do this on a good network before the event, not at the venue).
- `nvidia-smi` does not exist on Jetson (integrated GPU); `jtop` is the monitor
  (`sudo pip3 install -U jetson-stats`, then `jtop`; jetson-stats 7.2.1, 2026-08-09; the README
  shows no reboot step, but if `jtop` reports its service is not running, log out and in once).

### 3.2 15-minute bring-up checklist (unboxing to a trajectory file)

Do steps 0-1 at home; the rest is what happens at the venue with a flashed board.

| # | Minutes | Step | Done when |
|---|---|---|---|
| 0 | at home | Flash JetPack 6.2.1 to a 64 GB+ microSD with Balena Etcher (`camera/docs/jetson-orin-nano-setup.md` step 1); pre-download the aarch64 wheel (113 MB) and a copy of this repo (without `.venv/`, `recordings/`, `snapshots/`) onto a USB stick, in case venue Wi-Fi is bad | SD in the board, stick in your pocket |
| 1 | 2 | Boot; accept the EULA, pick language/keyboard/time zone, join the network, create the user (Orin Nano quick start) | desktop or SSH prompt |
| 2 | 1 | `sudo nvpmodel -q` (note the mode; `camera/docs/jetson-orin-nano-setup.md` step 2 for MAXN SUPER); skip `apt full-upgrade` at the venue | mode noted |
| 3 | 1 | Copy the repo: `rsync -av --exclude .venv --exclude recordings --exclude snapshots /media/$USER/<stick>/DroneDepthCamTesting/ ~/DroneDepthCamTesting/` (or `git clone` with credentials; the repo is private) | `ls ~/DroneDepthCamTesting/tools` |
| 4 | 5-8 | `bash ~/DroneDepthCamTesting/methods/cuvslam-vio/tools/setup_jetson.sh` (`CUVSLAM_WHEEL_DIR=/media/$USER/<stick>` if the wheel is on the stick). It verifies L4T R36 / Ubuntu 22.04 / Python 3.10, installs `nvidia-jetpack` only if absent, checks the three CUDA libraries, writes the udev rule, creates `~/DroneDepthCamTesting/.venv`, installs the wheel (sha256-checked), `depthai==3.10.0` and the pins from section 1.5, `jtop`, then runs the smoke test and `check_env.py` | `warm_up_gpu(): ok` and `cuvslam: importable version=17.0.0 dist=17.0.0+cu12` |
| 5 | 1 | Unplug/re-plug the OAK-D S2 into a USB Type-A port, `source ~/DroneDepthCamTesting/.venv/bin/activate`, `timeout 90 python camera/00_device_info.py` | `Device: OAK-D-S2 ... usb=SUPER` |
| 6 | 1 | `timeout 120 python datasets/07_record_dataset.py --duration 30 --headless` while walking the camera in a slow loop (see the script's `--help` for options) | a new `recordings/<name>/mav0/` with `cam0/`, `cam1/`, `imu0/` |
| 7 | 1 | `python methods/cuvslam-vio/08_cuvslam_track.py recordings/<name> --mode stereo` (then `--mode inertial`) | a `.tum` trajectory file and the script's end-point summary |
| 8 | 1 | `evo_traj tum <file>.tum` for the stats; `jtop` in a second terminal while re-running step 7 to see GPU load | numbers written down |

Total: about 15 minutes when `nvidia-jetpack` is already on the image; add the apt download time
if it is not.

### 3.3 What `methods/cuvslam-vio/tools/setup_jetson.sh` does, and the knobs

Idempotent, `set -euo pipefail`, run as the normal user (it calls `sudo` where needed). Steps:
host checks -> apt basics -> `nvidia-jetpack` if missing -> `ldconfig` check for `libcublas.so.12`,
`libcusolver.so.11`, `libcusparse.so.12` (installs `cuda-runtime-12-6` explicitly if any is
missing) -> `nvcc` on `PATH` when present -> udev rule (written only if the exact line is absent,
then `udevadm control --reload-rules && udevadm trigger`) -> venv at `<repo>/.venv` **without**
`--system-site-packages` (JetPack's system `python3-numpy` 1.x and OpenCV 4.5 must not leak in,
see the Jetson doc) -> wheel download + `sha256sum -c` + `pip install` -> `depthai==3.10.0
numpy==2.2.6 opencv-python~=4.11.0 scipy==1.14.1 pyyaml==6.0.2 pillow==12.2.0 evo` ->
`rerun-sdk==0.33.1` with `WITH_RERUN=1` -> `sudo pip3 install -U jetson-stats` unless `WITH_JTOP=0`
-> smoke test -> `check_env.py` -> prints `nvpmodel -q` and the next commands.
`SKIP_APT=1` re-runs only the Python side (no sudo). `CUVSLAM_VENV` / `CUVSLAM_WHEEL_DIR` relocate
the venv / wheel cache.

The OpenCV pin differs from the repo's `requirements.txt` (5.0.0.93): both are `cp37-abi3`
aarch64 wheels and the repo scripts use only APIs present in 4.11; do not `pip install -r
requirements.txt` into this venv afterwards or it will swap OpenCV back to 5.0 (harmless for the
scripts, but then the venv no longer matches NVIDIA's validated set).

---

## 4. Verify the install

### 4.1 `methods/cuvslam-vio/tools/check_env.py`

Standard library only, exits 0 always, `--json` for machine-readable output, `--warm-up` to also
call `cuvslam.warm_up_gpu()` (which "Throws an exception if CUDA, cusolver or cublas initialization
fails", per the v17.0.0 pybind source). It reports OS/WSL/Jetson detection (JetPack version from
`dpkg-query -W nvidia-jetpack`, L4T from `/etc/nv_tegra_release`), Python, `nvidia-smi` (driver,
the driver's "CUDA Version" ceiling, GPU, compute capability), the toolkit from `nvcc --version` or
`/usr/local/cuda/version.json` (`cuda.version`) or `version.txt`, the `/usr/local/cuda-*` dirs,
where `libcuda.so.1` and `libcublas.so.12` come from, `cuvslam` (`get_version()` returns
`(semver string, major, minor, patch)` in v17.0.0; `__version__` strips the `+cu12`), `depthai`,
and the package versions.

Measured on this laptop on 2026-09-19 (`.venv\Scripts\python.exe methods\cuvslam-vio\tools\check_env.py`):

```
os: Windows-11-10.0.26200-SP0 (AMD64)
os.wsl: no
os.jetson: no
python: 3.13.3 (CPython) venv=yes exe=C:\Users\<user>\...\DepthCamTesting\.venv\Scripts\python.exe
nvidia-smi: present (C:\WINDOWS\system32\nvidia-smi.EXE) driver=581.95 driver_cuda_version=13.0 gpu=NVIDIA RTX 1000 Ada Generation Laptop GPU (compute 8.9)
cuda.nvcc: not found
cuda.version_json: not found
cuda.dirs: none
cuda.libcuda: not found
cuda.libcublas: not found
cuvslam: NOT importable (ModuleNotFoundError: No module named 'cuvslam')
depthai: importable version=3.10.0
numpy: 2.5.3
opencv: 5.0.0
scipy: not installed
pyyaml: not installed
pillow: not installed
rerun-sdk: not installed
evo: not installed
summary: 7 note(s)
  - CUDA toolkit not installed on Windows (not needed: cuVSLAM cannot run here)
  - cuvslam not importable: expected on Windows (no Windows wheel exists); tests use cuvslam_oak.fake_cuvslam
  ...
```

Expected after the setup scripts (not yet observed; these are the lines the scripts' checks
correspond to):

| Line | WSL2 Ubuntu-22.04 | Jetson Orin Nano, JetPack 6.2.x |
|---|---|---|
| `os.wsl` / `os.jetson` | `yes (Ubuntu-22.04)` / `no` | `no` / `yes model=NVIDIA Jetson Orin Nano ... L4T=36.4.x jetpack=6.2.x+b..` |
| `python` | `3.10.12 ... venv=yes exe=~/venvs/cuvslam/bin/python` | `3.10.12 ... venv=yes exe=~/DroneDepthCamTesting/.venv/bin/python` |
| `nvidia-smi` | `present (/usr/lib/wsl/lib/nvidia-smi or /usr/bin/nvidia-smi) driver=581.95 driver_cuda_version=13.0` | `not found` (normal, use `jtop`) |
| `cuda.nvcc` | `/usr/local/cuda-12.6/bin/nvcc version=12.6.85` (12.6.3 toolkit; nvcc component 12.6.85, cudart 12.6.77: `cuda-compiler-12-6` 12.6.3-1 depends on `cuda-nvcc-12-6 >= 12.6.85`) | `/usr/local/cuda-12.6/bin/nvcc version=12.6.68` if `nvidia-jetpack-dev` is installed, else `not found` |
| `cuda.version_json` | `/usr/local/cuda/version.json version=12.6.3` | `.../version.json version=12.6.x` (from `cuda-toolkit-12-6 12.6.11-1`) |
| `cuda.libcuda` | `/usr/lib/wsl/lib/libcuda.so.1` (anything else means a Linux driver was installed in WSL) | `/usr/lib/aarch64-linux-gnu/tegra/libcuda.so.1` or similar |
| `cuda.libcublas` | `/usr/local/cuda-12.6/targets/x86_64-linux/lib/libcublas.so.12` | `/usr/local/cuda-12.6/targets/aarch64-linux/lib/libcublas.so.12` |
| `cuvslam` | `importable version=17.0.0 dist=17.0.0+cu12 get_version=['17.0.0...', 17, 0, 0]` (`__version__` strips the `+cu12`; `dist` is the pip-installed version) | same |
| `depthai` | `3.10.0` (unless `WITH_DEPTHAI=0`) | `3.10.0` |
| `summary` | `all expected pieces present` (or only the rerun/evo "optional" notes) | same |

### 4.2 One-liner smoke test (what both scripts run)

```bash
python - <<'PY'
import cuvslam
print(cuvslam.get_version())   # ('17.0.0...', 17, 0, 0)
cuvslam.warm_up_gpu()          # raises if CUDA / cuSOLVER / cuBLAS cannot start
print("ok")
PY
```

`ImportError: libcublas.so.12: cannot open shared object file` here means the toolkit libraries are
missing or `ldconfig` has not seen them; a CUDA "driver version is insufficient" error means the
Windows driver is older than 525 (it is 581.95 on this laptop, so that would be a different
machine).

### 4.3 The independent check: NVIDIA's own EuRoC example on one of our recordings

`datasets/07_record_dataset.py` writes the EuRoC ASL layout precisely so that
`examples/euroc/track_euroc.py` from tag v17.0.0 can load it **unmodified**. What that loader
(`examples/euroc/dataset_utils.py` at v17.0.0) requires of the folder:

- `mav0/cam0/data.csv` and `mav0/cam1/data.csv` with a header line then `timestamp_ns,filename`
  rows; the two files must have the **same number of rows and identical timestamps row by row**
  (it raises otherwise); images at `mav0/cam0/data/<filename>` loaded with PIL and must be mode
  `L` (mono8), `RGB` or `I;16`.
- `mav0/cam0/sensor.yaml`, `mav0/cam1/sensor.yaml`: a key starting with `T_` whose `data` is 16
  numbers (4x4, cam0's is inverted so cam0 becomes the rig origin), `intrinsics: [fu, fv, cu, cv]`,
  `resolution: [w, h]`, `distortion_coefficients`. With plain `sensor.yaml` files it builds
  `Distortion.Model.Brown` from `[k1, k2, 0, p1, p2]`, so write exactly four radtan coefficients
  `[k1, k2, p1, p2]` and **do not** create `sensor_recalibrated.yaml` (that path switches to the
  4-coefficient Fisheye model).
- `mav0/imu0/data.csv` rows `timestamp_ns, gx, gy, gz, ax, ay, az` and `mav0/imu0/sensor.yaml` with
  `T_BS`, `rate_hz`, `gyroscope_noise_density`, `gyroscope_random_walk`,
  `accelerometer_noise_density`, `accelerometer_random_walk` (the example's default mode is
  `OdometryMode.Inertial`, which reads these; set `euroc_tracking_mode =
  cuvslam.Tracker.OdometryMode.Multicamera` for stereo-only).

Run it (WSL2 or Jetson, venv active, `rerun-sdk` installed because the example imports it):

```bash
git clone --depth 1 --branch v17.0.0 https://github.com/nvidia-isaac/cuVSLAM.git ~/cuvslam-src
cd ~/cuvslam-src/examples/euroc
ln -sfn ~/datasets/<recording> dataset        # the script hard-codes dataset/mav0 next to itself
python track_euroc.py
```

Expected: `cuVSLAM Tracker initilized with odometry mode: OdometryMode.Inertial` (sic), a rerun
window with the two camera views, the trajectory and, in inertial mode, a red gravity arrow that
points down (NVIDIA: "If the gravity vector is consistently misaligned (not pointing downward),
please double-check and update your IMU extrinsics matrix"). `Warning: Failed to track frame N`
should be rare; `Warning: No IMU measurements between timestamps ...` means the IMU CSV has gaps.
Without a display, change the two visualisation lines to `rr.init("cuVSLAM Visualizer",
spawn=False)` and add `rr.save("run.rrd")` (the only edit needed). The example keeps its
`odom_trajectory` in memory and writes no file; use `methods/cuvslam-vio/08_cuvslam_track.py` for the TUM
output and compare its end point with what the rerun view shows.

Why this is the independent check: it uses NVIDIA's loader, NVIDIA's rig construction and
NVIDIA's mode selection, none of our code. If it tracks and ours does not, the bug is in
`cuvslam_oak/`; if neither tracks, the dataset (calibration, sync, image content) is the problem
(NVIDIA's `TROUBLESHOOTING.md`: "When tracking is inaccurate, the root cause is almost always in
the inputs, not the algorithm parameters").

---

## 5. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `wsl --install` prints the help text | WSL already present | `wsl --list --online`, then `wsl --install -d Ubuntu-22.04` |
| `nvidia-smi: command not found` inside WSL | Windows driver missing/old, or the distro runs as WSL 1 | update the Windows driver, `wsl --update`, `wsl --shutdown`; `wsl -l -v` must show VERSION 2; try `/usr/lib/wsl/lib/nvidia-smi` |
| `E: Unable to locate package cuda-toolkit-12-6` | keyring/repo step skipped | re-run the keyring `dpkg -i` and `apt-get update` |
| `apt` wants to install `nvidia-driver-*` | you asked for `cuda` / `cuda-12-6` / `cuda-drivers` | abort; install `cuda-toolkit-12-6` only |
| `cuvslam-...whl is not a supported wheel on this platform` | wrong interpreter (not 3.10), glibc < 2.35 (Ubuntu 20.04), or the x86_64 wheel on aarch64 | Ubuntu 22.04's `python3`, correct wheel for `uname -m` |
| `ImportError: libcublas.so.12` (or `libcusolver.so.11`, `libcusparse.so.12`) | toolkit libraries missing or not in the loader cache | WSL: `sudo apt-get install cuda-toolkit-12-6`; Jetson: `sudo apt-get install nvidia-jetpack` (or `cuda-runtime-12-6`); then `sudo ldconfig` |
| `warm_up_gpu()` raises a CUDA error | GPU not visible to the VM/driver | `nvidia-smi -L`; in WSL confirm `libcuda.so.1` is `/usr/lib/wsl/lib/libcuda.so.1`; on Jetson check `jtop` shows the GPU |
| `bash: $'\r': command not found` running a `methods/cuvslam-vio/tools/*.sh` | CRLF line endings after a Windows checkout with autocrlf | `sed -i 's/\r$//' methods/cuvslam-vio/tools/setup_wsl2.sh` (the files in this repo were written with LF) |
| `Permission denied` running `methods/cuvslam-vio/tools/setup_*.sh` | executable bit not preserved on the Windows file system | run them as `bash methods/cuvslam-vio/tools/setup_wsl2.sh`, no `chmod` needed |
| Tracking a dataset is far slower than the camera frame rate | dataset read through `/mnt/c` | copy it to `~/datasets` first (section 2.5) |
| pip resolves `numpy` to 2.5.x | Python 3.13 (Windows venv) not 3.10 | you are in the wrong venv; the Linux venvs pin 2.2.6 |
| `[warning] Insufficient permissions to communicate with X_LINK_UNBOOTED device` (Jetson) | udev rule missing or camera not re-plugged | `methods/cuvslam-vio/tools/setup_jetson.sh` writes it; re-plug |
| Camera works on Windows, `X_LINK_DEVICE_NOT_FOUND` through usbipd | the OAK re-enumerates after firmware boot (new bus id) | `--auto-attach`, bind both ids, the re-attach loop from depthai-python#938, `DEPTHAI_BOOTUP_TIMEOUT=15000` (section 2.7) |
| rerun viewer fails to open / crashes under WSLg | Vulkan on WSL | `WGPU_BACKEND=gl` or `rerun --renderer=gl`, `unset WAYLAND_DISPLAY`; or run the example with `spawn=False` |
| `evo_traj ... --plot` says no display | headless shell | drop `--plot` (stats print anyway) or use WSLg / the Jetson desktop |

---

## 6. Not verified (and why)

- **WSL2 path: verified on 2026-09-20** (`methods/cuvslam-vio/tools/setup_wsl2.sh` ran on this laptop; the Jetson path
  below is still unverified). Lessons from that first install:
  - `wsl --install` **without** `-d Ubuntu-22.04` only installed the WSL platform on this Windows
    build (26200); after the reboot `wsl -l -v` said "no installed distributions". `wsl --install -d
    Ubuntu-22.04 --no-launch` then installed the distro without the first-run prompt, and the name
    matched `wsl --list --online`.
  - **Check free space on C: first** (`Get-PSDrive C`): the toolkit occupies 5.0 GB under
    `/usr/local/cuda-12.6` plus about 3 GB of `.deb` files in the apt cache until `apt-get clean`,
    and the WSL virtual disk grew from 1.5 GB to 13 GB. The C: drive filled during the download,
    the ext4 root remounted read-only (`Read-only file system` on every write), and a dataset copy
    made minutes earlier turned out truncated (219 zero-byte files, an empty `dataset.json`). After
    freeing space: `wsl --shutdown`, `dpkg --configure -a`, `apt-get -f install`, re-run the script.
    Verify every copied dataset with `find <copy> -type f -size 0` before tracking it.
  - `SKIP_APT=1` works as intended once the apt steps have been done as root (`wsl -d Ubuntu-22.04
    -u root`), so the user-level run needs no password.
  - Install time: about 15 minutes for the toolkit on this connection, 3 minutes for the venv, wheel
    and pins.
- `cuvslam_oak/` (incl. `fake_cuvslam.py`), `datasets/07_record_dataset.py`, `methods/cuvslam-vio/08_cuvslam_track.py`
  and `tests/` landed after this doc was first written; on 2026-09-20 every flag quoted for them
  was re-checked against the scripts' `--help` on Windows: `07 --duration 30 --headless` exist,
  `08 --mode stereo` is listed and `--mode inertial` is accepted as an alias of the listed
  `--mode vio` (`cuvslam_oak.tracking.MODE_ALIASES`), `08` writes the TUM file (a `--fake-cuvslam`
  dry run on a real recording), imports `rerun` only under `--rerun`, and `check_env.py` prints the
  `tests use cuvslam_oak.fake_cuvslam` note. Running them with the real wheel remains unverified
  (no Windows wheel; see [cuvslam.md](cuvslam.md) section 9).
- `methods/cuvslam-vio/tools/setup_jetson.sh` has not run on a Jetson (none on hand). `methods/cuvslam-vio/tools/check_env.py` has run on
  Windows and on WSL2, where it matched the WSL column of the section 4.1 table line for line
  (`cuda.nvcc ... version=12.6.85`, `cuda.version_json ... version=12.6.3`, `cuda.libcuda:
  /usr/lib/wsl/lib/libcuda.so.1`, `cuvslam: importable version=17.0.0 dist=17.0.0+cu12
  get_version=['17.0.0+57f42cc', 17, 0, 0]`, `summary: all expected pieces present`). The Jetson
  column is still derived from package data, not observed.
- Whether the JetPack 6.2.1 SD-card image pre-installs `nvidia-jetpack` (CUDA etc.) is not stated
  on the pages read; the script handles both cases.
- `pip install <URL with %2B>` directly from GitHub was not exercised (the scripts download the
  file first for the sha256 check).
- The `cuda-toolkit-12-6` install on WSL was measured on 2026-09-20 (5.0 GB installed, about 15
  minutes); the `nvidia-jetpack` size was not.
- usbipd-win throughput for an OAK over USB/IP and the rerun viewer under WSLg were not tested; the
  Luxonis forum thread on usbipd (discuss.luxonis.com/d/5088) and the Softpedia driver page both
  returned HTTP 403 to the fetcher, so the OAK-in-WSL details come from the two GitHub issues cited
  instead, and the `32.0.15.8195` = `581.95` mapping is from this laptop's own `nvidia-smi`/WMI.
- `main` of the cuVSLAM repo pins `numpy==2.2.6`, `pillow==12.3.0`, `scipy==1.15.3` while tag
  v17.0.0 pins `2.2.4` / `12.2.0` / `1.14.1`; the tag's pins were kept except numpy (section 1.5).
  The brief's note that `main` changed the `Tracker` constructor was not re-checked; v17.0.0's
  `python/tracker.py` is `Tracker(rig, odom_config=None, slam_config=None)`.
- Every URL in section 7 answered HTTP 200 to `curl -sI` on 2026-09-19, with three expected
  exceptions: the two wheel URLs answer 302 to GitHub's `release-assets.githubusercontent.com`
  CDN (the download worked and the digests matched), and `https://github.com/nvidia-isaac/cuVSLAM.git`
  (the `git clone` form) answers 301 to the repository page.

---

## 7. Sources

cuVSLAM / PyCuVSLAM (all file reads at tag `v17.0.0`; `NVlabs/PyCuVSLAM` -> 301 -> `nvidia-isaac/cuVSLAM`):

- Release v17.0.0 (assets, digests, KPIs): https://github.com/nvidia-isaac/cuVSLAM/releases/tag/v17.0.0 ; API: `gh api repos/NVlabs/PyCuVSLAM/releases/tags/v17.0.0`
- README at v17.0.0 (wheel table, CUDA prerequisite, Jetson build notes, FAQ): https://raw.githubusercontent.com/NVlabs/PyCuVSLAM/v17.0.0/README.md
- `examples/requirements.txt`: https://raw.githubusercontent.com/NVlabs/PyCuVSLAM/v17.0.0/examples/requirements.txt ; on `main`: https://raw.githubusercontent.com/NVlabs/PyCuVSLAM/main/examples/requirements.txt
- EuRoC example: https://raw.githubusercontent.com/NVlabs/PyCuVSLAM/v17.0.0/examples/euroc/track_euroc.py , https://raw.githubusercontent.com/NVlabs/PyCuVSLAM/v17.0.0/examples/euroc/dataset_utils.py , https://raw.githubusercontent.com/NVlabs/PyCuVSLAM/v17.0.0/examples/euroc/README.md , sensor templates `examples/euroc/sensor_cam0.yaml`, `sensor_cam1.yaml`, `sensor_imu0.yaml`
- OAK-D example: https://raw.githubusercontent.com/NVlabs/PyCuVSLAM/v17.0.0/examples/oak-d/run_stereo.py , https://raw.githubusercontent.com/NVlabs/PyCuVSLAM/v17.0.0/examples/oak-d/README.md
- pybind source (`get_version`, `warm_up_gpu`): https://raw.githubusercontent.com/NVlabs/PyCuVSLAM/v17.0.0/python/cuvslam2.cpp ; package init and Tracker wrapper: https://raw.githubusercontent.com/NVlabs/PyCuVSLAM/v17.0.0/python/__init__.py , https://raw.githubusercontent.com/NVlabs/PyCuVSLAM/v17.0.0/python/tracker.py ; API doc index: https://raw.githubusercontent.com/NVlabs/PyCuVSLAM/v17.0.0/python/docs/api.rst
- Troubleshooting guide: https://raw.githubusercontent.com/NVlabs/PyCuVSLAM/v17.0.0/TROUBLESHOOTING.md
- Online API docs: https://nvidia-isaac.github.io/cuVSLAM/python/

WSL2 and CUDA on WSL:

- Install WSL (`wsl --install -d`, prerequisites, `--web-download`, WSL 1 vs 2): https://learn.microsoft.com/en-us/windows/wsl/install
- Distribution manifest behind `wsl --list --online`: https://raw.githubusercontent.com/microsoft/WSL/master/distributions/DistributionInfo.json
- File systems and performance (`/mnt/c`, `\\wsl$`): https://learn.microsoft.com/en-us/windows/wsl/filesystems
- Linux GUI apps / WSLg: https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps
- Connect USB devices (usbipd-win): https://learn.microsoft.com/en-us/windows/wsl/connect-usb ; usbipd-win repo and releases (v5.3.0): https://github.com/dorssel/usbipd-win , https://github.com/dorssel/usbipd-win/releases/tag/v5.3.0 ; wiki WSL support (udev, kernel): https://github.com/dorssel/usbipd-win/wiki/WSL-support ; `--auto-attach` option: `Usbipd/Program.cs` in that repo
- NVIDIA CUDA on WSL User Guide (driver rule, `libcuda.so`, `/usr/lib/wsl/lib`, `cuda-toolkit-12-x` only, limitations): https://docs.nvidia.com/cuda/wsl-user-guide/index.html
- WSL-Ubuntu apt repo (keyring, `Packages` index): https://developer.download.nvidia.com/compute/cuda/repos/wsl-ubuntu/x86_64/ ; keyring: https://developer.download.nvidia.com/compute/cuda/repos/wsl-ubuntu/x86_64/cuda-keyring_1.1-1_all.deb ; CUDA 12.6.3 WSL-Ubuntu download page: https://developer.nvidia.com/cuda-12-6-3-download-archive?target_os=Linux&target_arch=x86_64&Distribution=WSL-Ubuntu&target_version=2.0&target_type=deb_network
- CUDA Toolkit release notes (Table 3: minor-version compatibility 12.x >= 525 and < 580, 13.x >= 580; Table 4: CUDA 12.6 Update 3 needs Windows driver >= 561.17): https://docs.nvidia.com/cuda/cuda-toolkit-release-notes/index.html
- CUDA Linux installation guide (`/usr/local/cuda` symlink, PATH, Ubuntu 22.04 row): https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html
- Rerun troubleshooting (`--renderer=gl`, `WGPU_BACKEND`, Wayland, WSL Mesa): https://rerun.io/docs/getting-started/install-rerun/troubleshooting (the older `/docs/getting-started/troubleshooting` path answers 308 to it)
- OAK under WSL2/usbipd: https://github.com/luxonis/depthai/issues/723 (re-enumeration, `DEPTHAI_BOOTUP_TIMEOUT`, auto-attach loop) , https://github.com/luxonis/depthai-python/issues/938 (current `usbipd attach --wsl` syntax, `Shared` state)

Jetson:

- JetPack 6.2 / 6.2.1 / 6.2.3 pages (CUDA 12.6, `apt install nvidia-jetpack`): https://developer.nvidia.com/embedded/jetpack-sdk-62 , https://developer.nvidia.com/embedded/jetpack-sdk-621 , https://developer.nvidia.com/embedded/jetpack-sdk-623
- JetPack install via apt: https://docs.nvidia.com/jetson/jetpack/install-setup/index.html
- Orin Nano quick start (first boot, Etcher, JetPack 7.2 notes): https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/quick_start.html
- Jetson apt index r36.4 (package dependency chain): https://repo.download.nvidia.com/jetson/common/dists/r36.4/main/binary-arm64/Packages
- jetson-stats / jtop: https://github.com/rbonghi/jetson_stats , https://pypi.org/project/jetson-stats/
- Luxonis udev rule / USB guide: https://docs.luxonis.com/hardware/platform/deploy/usb-deployment-guide
- This repo's Jetson guide: `camera/docs/jetson-orin-nano-setup.md`

PyPI (wheel availability and pins):

- https://pypi.org/project/depthai/3.10.0/#files , https://pypi.org/project/numpy/2.2.6/#files , https://pypi.org/project/opencv-python/4.11.0.86/#files , https://pypi.org/project/scipy/1.14.1/#files , https://pypi.org/project/pillow/12.2.0/#files , https://pypi.org/project/PyYAML/6.0.2/#files , https://pypi.org/project/rerun-sdk/0.33.1/#files , https://pypi.org/project/evo/
- evo README (tools, formats) and TUM format definition: https://github.com/MichaelGrupp/evo , https://github.com/MichaelGrupp/evo/wiki/Formats
