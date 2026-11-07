# Running the DroneDepthCamTesting scripts on a Jetson Orin Nano

How to run this repo's DepthAI v3 Python scripts (`camera/00_device_info.py` ... `camera/06_replay.py`,
all built on `camera/common.py`) on an NVIDIA Jetson Orin Nano (the target compute board) with the
OAK-D S2 plugged in over USB3.

Researched 2026-09-18 against the live Luxonis docs, PyPI, GitHub and NVIDIA docs. Version numbers are
what those sources showed on that date; the camera was **not** opened while writing this. The scripts
were written and tested on Windows 11 with `depthai==3.10.0`; the identical PyPI release installs on
the Jetson (see the wheel evidence below), the scripts use `pathlib`, no Windows-only APIs and no
syntax newer than Python 3.10, so the code does not change between the two hosts.

At a glance:

| Item | Value |
|---|---|
| Recommended JetPack | **6.2.x** (Jetson Linux 36.4.x-36.5.x; newest 6.2.3 = L4T 36.5.2; Ubuntu 22.04, Python 3.10, glibc 2.35) |
| Also works | JetPack 7.2 / 7.2.1 (Jetson Linux 39.2.x, Ubuntu 24.04, kernel 6.8, Python 3.12, glibc 2.39); USB-ISO install only |
| Does **not** work | JetPack 5.x (Ubuntu 20.04, Python 3.8): `depthai` 3.10.0 needs Python >= 3.9, `open3d` 0.20.0 needs >= 3.10 and glibc 2.35 |
| `depthai` on aarch64 | 3.10.0 ships one `manylinux_2_28_aarch64` wheel tagged cp39 + cp310 + cp311 + cp312 + cp313 + cp314 (82.6 MB, no sdist), uploaded 2026-09-02 |
| Camera | OAK-D S2 (RVC2 / Myriad X), USB vendor id `03e7`, expected link `UsbSpeed.SUPER` (5 Gbps) |
| Shared CLI flags | every script: `--device-id <id>`, `--duration <seconds>`, `--headless` (from `camera/common.py`) |
| Headless smoke test | `timeout 90 python camera/01_rgb_preview.py --headless --duration 10` |

---

## 1. Setup checklist

Do these in order. Steps 1-4 are done once per board, step 5 once per checkout, steps 6-7 each time
the camera is (re)connected; step 8 is optional.

### 1. Flash JetPack 6.2.x (Ubuntu 22.04 / Python 3.10)

- Use the **JetPack 6.2.1 SD card image for the Jetson Orin Nano Developer Kit** (Jetson Linux
  36.4.4, "Ubuntu 22.04-based root file system"), or SDK Manager onto NVMe:
  https://developer.nvidia.com/embedded/jetpack-sdk-621
- The newest 6.x is JetPack 6.2.3 (L4T 36.5.2) and NVIDIA did not cut a new SD image for it; its page
  says "Use the SD Card image of JetPack 6.2.1/Jetson Linux 36.4.4 and APT upgrade to JetPack
  6.2.3/Jetson Linux 36.5.2" (https://developer.nvidia.com/embedded/jetpack-sdk-623). The
  `apt full-upgrade` in step 2 is that upgrade. Release list:
  https://developer.nvidia.com/embedded/jetpack-archive
- A devkit still on its factory (JetPack 5-era) firmware must have its QSPI/UEFI firmware updated
  first: "The factory-installed firmware on the Jetson Orin Developer Kit supports JetPack 5.x and
  requires an update to ensure compatibility with JetPack 6.x" - follow the Initial Setup Guide linked
  from the 6.2.1 page above.
- If you go to **JetPack 7.2 / 7.2.1** instead (Ubuntu 24.04, Python 3.12): NVIDIA's Orin Nano quick
  start now recommends 7.2.1, but "Starting with JetPack 7.2, SD Card images are no longer supported"
  - you write the unified Jetson ISO to a USB stick and install to microSD/NVMe from it, and
  "JetPack 7.2 and later installations require JetPack 6.x-generation Jetson UEFI/QSPI firmware"
  (https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/quick_start.html). The 7.2
  download page lists "Jetson Orin Family" as supported and says "There is no more SD Card image for
  Jetson Orin Nano Developer Kit" (https://developer.nvidia.com/embedded/jetpack/downloads/archive-7.2).
- Python is the stock Ubuntu interpreter: 3.10 on 22.04 (3.10.12 after updates,
  https://launchpad.net/ubuntu/jammy/+source/python3.10), 3.12.3 on 24.04. Both are inside the
  cp39-cp314 range of the `depthai` wheel. This repo targets **3.10**, so JetPack 6.2.x is the
  primary choice.

### 2. First boot housekeeping and power mode

```bash
sudo apt update && sudo apt full-upgrade -y && sudo reboot
```

JetPack 6.2 added "MAXN SUPER" to the Orin Nano 8 GB (modes `7W | 15W | 25W | MAXN SUPER`); the
blog's command for the Orin Nano is `sudo nvpmodel -m 2`
(https://developer.nvidia.com/blog/nvidia-jetpack-6-2-brings-super-mode-to-nvidia-jetson-orin-nano-and-jetson-orin-nx-modules/).
The RVC2 does the depth work on the camera, but a faster host keeps OpenCV/Open3D display and any
host-side processing from becoming the bottleneck:

```bash
sudo nvpmodel -q            # show the current mode
sudo nvpmodel -m 2          # MAXN SUPER on the Orin Nano (ids depend on the flashing config; check -q)
sudo jetson_clocks          # pin clocks to max while benchmarking (not persistent)
```

Mode ids come from `/etc/nvpmodel.conf`, so confirm with `-q` rather than hardcoding `2` in a script.

### 3. Install the few system packages the venv needs

```bash
sudo apt install -y python3-venv python3-pip git
```

Nothing else is required for `depthai`. The aarch64 wheel was unpacked while writing this: it
contains one extension module per CPython version (`depthai.cpython-3XX-aarch64-linux-gnu.so`) plus a
`depthai.libs/` directory with `libdepthaicore` (104 MB), `libdynamic_calibration`, `libusb1`,
`libgomp` and FFmpeg 7 (`libavcodec.so.61`, `libavformat.so.61`, `libavutil.so.59`, `libswscale`,
`libswresample`, `libavfilter`, `libavdevice`). Its system-level references (`DT_NEEDED` in the
ELF objects) are glibc >= 2.28 (`libc`, `libm`, `libpthread`, `libdl`, `librt`, `ld-linux-aarch64`;
highest symbol version `GLIBC_2.28`, in `libdepthaicore` and the FFmpeg libs), `libstdc++.so.6`
(`GLIBCXX_3.4.22` / `CXXABI_1.3.11`) and `libgcc_s.so.1`, all present on any JetPack 6/7 image.
`libdepthaicore` also carries optional `dlopen` names (`libtbbmalloc.so.2`, `libtbbbind*.so.3`,
`libiomp5.so`) that need not exist. Do not install `libusb` or FFmpeg for it.

The Luxonis `install_dependencies.sh` (https://docs.luxonis.com/install_dependencies.sh, which
redirects to https://raw.githubusercontent.com/luxonis/depthai-python/refs/heads/main/docs/install_dependencies.sh)
is the v2-era helper: it installs OpenCV/Qt build packages and writes the same udev rule as step 4.
You do not need it for the pip wheel; the udev rule is the only part that matters.

### 4. Install the Luxonis udev rule (required, once)

```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Then **unplug and re-plug the camera** (or reboot). The rule and file name are exactly what
Luxonis' `install_dependencies.sh` writes; the same command is in the Luxonis USB deployment guide
(https://docs.luxonis.com/hardware/platform/deploy/usb-deployment-guide). (It also used to be on the
v2 troubleshooting page, `docs.luxonis.com/en/latest/pages/troubleshooting/#udev-rules-on-linux`,
but that URL now just redirects to the docs home page.)

Why it is needed: the OAK-D S2 is a raw USB device (vendor id `03e7` = Intel Movidius / Myriad X)
that DepthAI drives from user space through the `libusb` bundled in the wheel, not through a kernel
video driver. Without the rule the device node under `/dev/bus/usb/...` is root-only: the device is
reported with a permissions warning and cannot be opened. The installed 3.10.0 library
(`libdepthaicore`) holds that warning as a format template,
`Insufficient permissions to communicate with {} device with name "{}". Make sure udev rules are set`
(plus a `... device having name "{}" ...` variant and one ending `... Make sure udev rules are set. Error: {}`);
the XLink state (`X_LINK_UNBOOTED`) and the `bus.port` name fill the two slots at runtime, and those
slots suggest it is emitted while enumerating. The filled-in line is the title of
https://github.com/luxonis/depthai/issues/765, a closed 2022 "posting this for visibility" issue
whose body just points at the udev rule fix. (Not reproduced while writing this: Windows has no
udev.) `MODE="0666"` makes every
`03e7` device world-read/write, which covers both the unbooted id (`03e7:2485`, "Intel Movidius
MyriadX") and the booted id (`03e7:f63b`, "Intel Myriad VPU") because it matches on vendor only.

Verify:

```bash
lsusb -d 03e7:
# Bus 001 Device 005: ID 03e7:2485 Intel Movidius MyriadX      <- unbooted, waiting for firmware
ls -l /dev/bus/usb/001/005                                       # should be crw-rw-rw-
```

### 5. Copy the repo and create the venv

The GitHub repo (https://github.com/jsun4231/DroneDepthCamTesting, the laptop checkout's `origin`) is
**private**, so an anonymous `git clone` on the Jetson fails with an authentication error. Either
give the Jetson GitHub credentials (an SSH key on the account, or `gh auth login` / a personal
access token over HTTPS) and push the laptop's latest commit first, or copy the folder from the
laptop:

```bash
# Option A: copy the folder from the laptop, leaving out .venv/, recordings/ and snapshots/
#           (rsync from WSL or a Linux/macOS shell; from a plain Windows prompt use scp -r)
rsync -av --exclude .venv --exclude recordings --exclude snapshots DroneDepthCamTesting/ jetson:~/DroneDepthCamTesting/
# Option B: clone with credentials (private repo) - SSH key shown; HTTPS works after gh auth login / PAT
git clone git@github.com:jsun4231/DroneDepthCamTesting.git ~/DroneDepthCamTesting
cd ~/DroneDepthCamTesting
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -U pip
python -m pip install -r requirements.txt     # depthai 3.10.0, opencv-python, numpy, open3d
python -m pip install matplotlib              # optional: only 04_imu.py --plot uses it
```

Do **not** create the venv with `--system-site-packages`: anything not pinned in `requirements.txt`
(or pulled in transitively) would resolve to JetPack's system `python3-numpy` / `python3-opencv`
(numpy 1.x, OpenCV 4.5 without contrib modules such as `ximgproc`,
https://github.com/luxonis/depthai-ros/issues/497), and pip may treat such a requirement as already
satisfied by the system package and skip it.

`requirements.txt` already carries Python-version markers, so the same file resolves on both hosts.
What pip picks on JetPack 6 (Python 3.10) vs the Windows dev venv (Python 3.13.3):

| Package | Windows dev venv | Jetson, JetPack 6 (cp310) | Wheel evidence |
|---|---|---|---|
| depthai | 3.10.0 | 3.10.0 | `manylinux_2_28_aarch64`, cp39-cp314, `Requires-Python >=3.9`, `Requires-Dist: numpy<3.0.0` (https://pypi.org/project/depthai/3.10.0/#files) |
| numpy | 2.5.3 | **2.2.6** | 2.5.3 requires Python >= 3.12 and 2.3.0 requires >= 3.11; 2.2.6 is the last release with a cp310 aarch64 wheel (https://pypi.org/project/numpy/2.2.6/#files) |
| opencv-python | 5.0.0.93 | 5.0.0.93 | `cp37-abi3-manylinux_2_28_aarch64` (stable ABI, any CPython >= 3.7) (https://pypi.org/project/opencv-python/#files) |
| open3d | 0.20.0 | 0.20.0 | `cp310-cp310-manylinux_2_35_aarch64` (also cp311-cp314), `Requires-Python >=3.10` (https://pypi.org/project/open3d/#files) |
| matplotlib (optional) | not installed | 3.10.9 | 3.11.x (3.11.0-3.11.2) requires Python >= 3.11; 3.10.9 (2026-04-24) is the newest 3.10.x and has a cp310 aarch64 wheel (https://pypi.org/project/matplotlib/3.10.9/#files) |

pip needs to understand PEP 600 `manylinux_2_28` tags (pip >= 20.3); Ubuntu 22.04 ships pip
22.0.2 (https://launchpad.net/ubuntu/jammy/+source/python-pip), so the `-U pip` above is belt and
braces, not a requirement. The numpy difference means the scripts must not rely on APIs added after
numpy 2.2; nothing in this repo does.

Verify the install without booting the camera (enumeration only; safe even while another process
holds the device):

```bash
python - <<'PY'
import depthai as dai
print("depthai", dai.__version__)
for d in dai.Device.getAllAvailableDevices():
    print(d.deviceId, d.name, d.state, d.protocol, d.platform)
PY
```

Expected line for this camera (identical to what the Windows laptop shows):
`OAKDS2EXAMPLEID000 <bus.port> XLinkDeviceState.X_LINK_UNBOOTED XLinkProtocol.X_LINK_USB_VSC XLinkPlatform.X_LINK_MYRIAD_X`.
An empty list means USB enumeration failed (cable/port) or the camera is held by another process;
see Troubleshooting.

### 6. Confirm the link is USB3 (SUPER), not USB2 (HIGH)

Plug the OAK-D S2 into one of the devkit's **USB Type-A** ports with a short USB 3 cable (< 1 m
ideal, 2 m max). The devkit has "four USB Type-A ports" ("USB 3.2", "host mode only") in "two
dual-stacked Type-A connectors. Each stack has VBUS limited to 3A output current", plus one USB 3.2
Type-C that does host/device/recovery duty and "does not output a display signal"
(https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/hardware_layout.html). Keep the
camera on Type-A so the Type-C stays free for flashing.

Two checks, host side and DepthAI side:

```bash
# Host side: sysfs 'speed' is 480 (USB2), 5000 (USB3 Gen1) or 10000 (Gen2)
for d in /sys/bus/usb/devices/*; do
  [ -f "$d/idVendor" ] && [ "$(cat "$d/idVendor")" = "03e7" ] && \
    echo "$d pid=$(cat "$d/idProduct") speed=$(cat "$d/speed")M"
done
lsusb -t      # the Movidius entry shows 480M before boot and 5000M once booted
```

Before any pipeline runs the camera sits at **480M** even on a USB3 port: the Luxonis guide says
`lsusb -v` shows "480 Mb/s when the device is waiting for connection" and "5 Gb/s (if USB3 is
available)" only after the firmware is uploaded, when it re-enumerates as `03e7:f63b`
(https://docs.luxonis.com/hardware/platform/deploy/usb-deployment-guide). So either run the sysfs
loop a second time while a script is running, or use the repo's own check, which boots the camera
(~0.5 s enumeration + ~3 s firmware upload), prints one `Device:` line with the negotiated speed and
exits:

```bash
timeout 90 python camera/00_device_info.py
# Device: OAK-D-S2 (OAK-D-S2-AF) id=OAKDS2EXAMPLEID000 platform=RVC2 usb=SUPER
```

Every script that opens the camera prints that line at start-up (`common.print_device_summary`;
`06_replay.py` only in `--mode holistic`, since its default `--mode video` never boots the OAK) and writes a
`!!! WARNING: USB speed is HIGH instead of SUPER (USB 3) ...` line to stderr when the link is USB2.
`dai.UsbSpeed` in 3.10.0 has `LOW, FULL, HIGH, SUPER, SUPER_PLUS, UNKNOWN` (introspected); `SUPER`
(5 Gbps) is what the OAK-D S2 negotiates, `HIGH` means 480 Mb/s and the RGB + depth + preview
streams will drop frames or fail. `camera/00_device_info.py --duration 10` additionally logs the USB
speed, chip temperature and memory once per second, which is a cheap link-stability soak.

Seen once in five back-to-back boots on the laptop with a known-good cable and port (the
`04_imu.py` hardware run, 2026-09-18): the link came up `usb=HIGH` on one boot and `SUPER` on the
other four. So if `usb=HIGH` appears once, re-run the script (or re-plug) before swapping anything;
only a repeatable `HIGH` points at the cable, hub or port.

### 7. Run a script headless, then with a display

Every script accepts `--device-id`, `--duration SECONDS` (0 = until `q`/Esc/Ctrl-C) and
`--headless` (no OpenCV windows or key handling; periodic stats on stdout instead), defined once in
`camera/common.py` (`add_device_arg`, `add_run_args`). Over SSH:

```bash
cd ~/DroneDepthCamTesting && source .venv/bin/activate
timeout 90 python camera/00_device_info.py                                   # report, no window
timeout 90 python camera/01_rgb_preview.py   --headless --duration 10 --snapshot-on-exit
timeout 90 python camera/02_stereo_depth.py  --headless --duration 10 --snapshot-on-exit
timeout 90 python camera/03_point_cloud.py   --headless --duration 10 --save-on-exit
timeout 90 python camera/04_imu.py           --headless --duration 10
timeout 90 python camera/05_record.py        --headless --duration 10
timeout 90 python camera/06_replay.py recordings --headless                   # no camera needed (video mode)
```

`timeout 90` guarantees you never leave a process holding the camera even if SSH drops. Exit code 2
from the camera scripts means "no device / wrong `--device-id` / device busy" (`common.EXIT_NO_DEVICE`;
one line on stderr, no traceback); `06_replay.py` also exits 2 for a missing or unreadable recording
(its `EXIT_BAD_INPUT`, same one-line-on-stderr style); 0 is a normal end including `--duration` elapsed.

With a monitor on the devkit's DisplayPort (the "only display output"; HDMI needs a DP-to-HDMI
adapter, per the hardware layout page above), open a terminal on the desktop, or from SSH point at
the local X display:

```bash
DISPLAY=:0 python camera/02_stereo_depth.py --duration 30
```

`ssh -X` also works but ships every frame over the network; use `--headless` for throughput
numbers. See "OpenCV GUI on Jetson" and "Open3D on aarch64" below for what each script needs.

### 8. Optional: Open3D viewer, ROS 2, PX4

All covered in the notes below. None of them is needed for the seven test scripts.

---

## 2. Notes

### Which JetPack, and why Python 3.10 is the target

JetPack 6.2.x is the last JetPack that gives you Ubuntu 22.04 + Python 3.10 on the Orin Nano and is
the combination this repo is written against. JetPack 7.2.x (Ubuntu 24.04, Python 3.12) is now what
NVIDIA's Orin Nano quick start recommends, and every wheel above also has a cp312 aarch64 build, so
it is a valid second choice; the practical differences for this repo are the install path (USB ISO,
no SD image, JetPack 6-generation firmware prerequisite) and that ROS 2 would be Jazzy instead of
Humble. JetPack 5.x is out: Python 3.8 is below `depthai`'s `>=3.9`, and Ubuntu 20.04's glibc 2.31
is below the `manylinux_2_35` floor of the Open3D wheel. glibc references:
https://launchpad.net/ubuntu/jammy/+source/glibc (2.35), https://launchpad.net/ubuntu/noble/+source/glibc (2.39).

The Luxonis "Deploy with NVIDIA's Jetson" page
(https://docs.luxonis.com/hardware/platform/deploy/to-jetson) is still the v2-era text (clone
`depthai-python`, `install_requirements.py`, `OPENBLAS_CORETYPE=ARMV8`, disable zram and add a 4 GB
swap file). For DepthAI v3 the pip wheel is the whole install; the swap advice only matters if you
compile depthai-core or depthai-ros from source on the 8 GB board. Its one still-useful line is that
Jetson boards generally power OAK cameras directly over USB.

### What the wheel is (and is not)

- `depthai-3.10.0-cp39.cp310.cp311.cp312.cp313.cp314-...-manylinux_2_28_aarch64.whl`, uploaded
  2026-09-02, 82,579,808 bytes, 46 entries: one `.so` per CPython version plus `depthai.libs/` with
  `libdepthaicore`, `libdynamic_calibration`, `libusb1`, `libgomp` and FFmpeg 7. No sdist is
  published, so pip must find a matching wheel: CPython 3.9-3.14, glibc >= 2.28, aarch64.
  Files: https://pypi.org/project/depthai/3.10.0/#files ; release notes:
  https://github.com/luxonis/depthai-core/releases/tag/v3.10.0
- Only `numpy<3.0.0` is declared as a dependency; OpenCV is not pulled in by `depthai`.
- Pre-release builds live on Luxonis' own index (`--extra-index-url
  https://artifacts.luxonis.com/artifactory/luxonis-python-release-local/ --pre`), per the
  depthai-core README (https://github.com/luxonis/depthai-core). Not needed here; pin `==3.10.0`.
- Building depthai-core from source on Linux needs `libudev-dev` (same README); the wheel does not.
- The README also says Android is not supported on v3; that is the only ARM caveat it lists, and it
  does not apply to Linux/aarch64.

### What each script needs on the Jetson

| Script | Camera | Display | Optional dependency |
|---|---|---|---|
| `00_device_info.py` | boots it (report + optional `--duration` monitor) | never opens a window; `--headless` accepted, no-op | - |
| `01_rgb_preview.py` | yes | OpenCV window unless `--headless` | - |
| `02_stereo_depth.py` | yes | OpenCV window unless `--headless` | - |
| `03_point_cloud.py` | yes | Open3D + OpenCV windows unless `--headless` | `open3d` (lazy import; if missing or it cannot create a window it prints the reason and runs like `--headless`) |
| `04_imu.py` | yes | only with `--plot` (`--headless` suppresses the plot, text output unchanged) | `matplotlib` for `--plot` |
| `05_record.py` | yes | previews unless `--headless` | - |
| `06_replay.py` | **no** in `--mode video` (host-only `ReplayVideo` pipelines, the OAK stays unbooted); yes in `--mode holistic` | OpenCV windows unless `--headless` | - |

`06_replay.py --mode video` is therefore the one script you can run on the Jetson before the camera
is even plugged in, to prove the venv (depthai host nodes + OpenCV) works: copy a
`recordings/video_<stamp>/` folder from the laptop and replay it with `--headless`.

### OpenCV GUI on Jetson

- The non-headless Linux `opencv-python` wheels bundle Qt 5 for `highgui`
  (https://github.com/opencv/opencv-python). Qt's `xcb` platform plugin needs an X server. On the
  JetPack desktop image, log in locally or use `DISPLAY=:0` from SSH.
- Over plain SSH without a display, `cv2.imshow` fails with `qt.qpa.plugin: Could not load the Qt
  platform plugin "xcb"` (seen on Jetson in https://github.com/luxonis/depthai/issues/1089). The
  scripts catch the display failure and tell you to re-run with `--headless`; that, `ssh -X`, or
  `DISPLAY=:0` are the fixes.
- If you only ever run headless, `opencv-python-headless==5.0.0.93` is a drop-in that skips Qt
  entirely (same `cv2` API, same aarch64 `cp37-abi3` wheels,
  https://pypi.org/project/opencv-python-headless/#files). Install one or the other, not both.

### Open3D on aarch64

- Open3D 0.20.0 (released 2026-09-16) publishes `manylinux_2_35_aarch64` wheels for cp310-cp314, so
  `pip install open3d` works on JetPack 6 and 7 without building
  (https://pypi.org/project/open3d/#files). Its declared dependencies are `numpy`, `dash`, `werkzeug`,
  `flask`, `nbformat`, `configargparse` and `ipywidgets` (Requires-Dist in
  https://pypi.org/pypi/open3d/0.20.0/json), which is why the install is much bigger than the 48 MB wheel.
- Open3D's ARM page: "Starting from Open3D 0.14, we provide pre-compiled ARM64 wheels for Linux and
  macOS"; "Open3D installed via `pip install open3d` will not contain CUDA support on ARM64
  platforms"; "Full OpenGL (not OpenGL ES) is needed for Open3D GUI. Open3D GUI is supported on
  Nvidia Jetson platforms" (https://www.open3d.org/docs/release/arm.html). So the `03_point_cloud.py`
  viewer works on the Jetson's local desktop, and CPU-only Open3D is what you get from pip (fine for
  this repo; it makes no CUDA calls).
- Without a display the script prints `Open3D could not create a window (no display?); running like
  --headless.` and keeps streaming; `--save-on-exit` still writes the PLY.

### USB3 bandwidth, cables, hubs and power

From https://docs.luxonis.com/hardware/platform/deploy/usb-deployment-guide unless noted:

- "Use a USB3 USB-C cable for maximum bandwidth"; "USB3 cables are usually blue inside the USB-A
  connector". "Connecting via a long (>2m) USB cable might cause issues" - for long runs "make sure
  it's an active USB3 cable".
- Cable-related failures show up as `X_LINK_COMMUNICATION_NOT_OPEN` or `X_LINK_ERROR`, which the
  guide attributes to "an USB2 cable/USB2 port, ... a bad USB3 cable, or ... longer cable".
- Hubs: only powered **USB3** hubs. A USB2 hub silently drops you to `HIGH`.
- Forcing USB2 (for a known-bad cable or a USB2-only hub) is a constructor argument:
  `dai.Device(maxUsbSpeed=dai.UsbSpeed.HIGH)` (overload 2 of `dai.Device.__init__` in 3.10.0, "Maximum
  allowed USB speed"); it uploads the USB2 firmware and caps the link. Diagnostic only; the scripts'
  stream set is sized for USB3.
- Power: "OAK cameras draw between 500mA and 900mA", and "OAK have current spikes (especially when
  using Video Encoder and running AI inference) of up to 2W, which can lead to brownout"; USB3 ports
  supply 900 mA, USB2 500 mA. The guide's wording is "We always recommend externally powering your
  OAK camera", via a barrel jack, a powered USB3 hub or a Y-adapter. The OAK-D S2 has no barrel jack
  (USB-C only, see `camera/docs/oak-d-s2-specs.md`), so on the Jetson the options are a powered USB3 hub or
  a Y-adapter; running straight off a Type-A port is what the Luxonis Jetson page says generally
  works, but it is below the guide's recommendation, so try external power first if the camera
  resets under load. The OAK-D S2 datasheet (see `camera/docs/oak-d-s2-specs.md`) puts consumption at 4 W min /
  6 W typ / 7.5 W max with a 2 A maximum input current; the devkit's Type-A stacks are limited to 3 A
  each, so one camera per stack is comfortable, but do not share that stack with other
  power-hungry USB devices (LTE modem, telemetry radio) on a drone.
- The devkit itself is powered from its 5.5 mm x 2.5 mm DC jack (hardware layout page above);
  budget the Jetson's 7-25 W power mode plus ~5-7.5 W for the camera when sizing the drone's supply.

### Random disconnects on the Orin Nano (USB autosuspend)

An NVIDIA forum thread reports an OAK-D disconnecting after hours on the Orin Nano Super devkit
with `tegra-xusb` errors in `dmesg`; NVIDIA's reply was that "the latest Jetpack version for Orin is
6.2.1 and we don't see USB issues being reported", asked for the JetPack version and logs, and no
fix was posted
(https://forums.developer.nvidia.com/t/oak-d-disconnects-in-jetson-orin-nano-super-developer-kit-due-issues-with-xusb-tegra/349466).
If the camera vanishes mid-run:

```bash
# Immediate (until reboot): stop the host from autosuspending USB devices
echo -1 | sudo tee /sys/module/usbcore/parameters/autosuspend
# Persistent: append usbcore.autosuspend=-1 to the APPEND line in /boot/extlinux/extlinux.conf
```

and watch `dmesg -w` while it runs. On the software side, `DEPTHAI_WATCHDOG`,
`DEPTHAI_WATCHDOG_INITIAL_DELAY`, `DEPTHAI_RECONNECT_TIMEOUT` and `DEPTHAI_CONNECT_TIMEOUT` are all
present in the installed 3.10.0 library, and depthai-core ships a reconnect example
(`examples/python/Misc/AutoReconnect/reconnect_callback.py`,
https://github.com/luxonis/depthai-core/tree/main/examples/python/Misc/AutoReconnect).

The only v3 bug report found that involves an Orin Nano is https://github.com/luxonis/depthai/issues/1252
(OAK-D **Lite**: a CAM_B / stereo stream crashes the device firmware, `PlgSrcMipi` / `X_LINK_ERROR`;
the reporter's OAK-D Pro was fine). It is neither 3.8.0-specific nor Jetson-specific: opened
2026-08-19 on an Orin Nano Super with 3.8.0; a 2026-09-12 comment bisects it to a regression after
`depthai==3.6.1` (3.7.1, 3.8.0, 3.9.0 and 3.10.0 all fail intermittently on the same board); a
2026-09-16 comment reproduces the same crash dump on a Raspberry Pi 5 with 3.8.0 and 3.10.0; still
open as of 2026-09-18. It is scoped to the Lite's OV7251 sensors, not the S2's OV9282, but if the S2
ever dies with a `PlgSrcMipi` crash dump on the Jetson, compare against that issue and against a run
on the laptop.

### One process per camera

Only one process can hold the OAK-D S2, and the failure is quieter than an error code (measured on
the laptop, `camera/docs/depthai-v3-cheatsheet.md` section 1): while another process streams, the camera
**vanishes from enumeration** (`dai.Device.getAllAvailableDevices()` returns `[]`, with a
`[warning] skipping X_LINK_BOOTED device having name "..." (status: X_LINK_ERROR)` log line) and a
direct `dai.Device()` raises a plain `RuntimeError('No available devices')` (or
`'Cannot find any device with given deviceInfo'` when an id is given) after a ~10-14 s search. The
scripts turn that into one stderr line and exit 2: `No OAK device found. Check the USB-C cable and
that no other program has the camera open.` after three enumeration attempts 3 s apart, or
`Device <id> is in use by another process.` when the open itself fails. `X_LINK_DEVICE_ALREADY_IN_USE`
exists in the library as an XLink error code but was never raised on this camera (Linux/libusb
behaviour not verified), so do not grep for it. A device that crashed can stay `X_LINK_BOOTED` until
it is re-plugged or the watchdog resets it. Consequences on the Jetson:

- Never run a test script while the ROS 2 driver (or a stray previous script) is up:
  `pgrep -af python` and `pgrep -af depthai` before starting.
- Wrap runs in `timeout`, and pass `--duration`, so a hung SSH session cannot leave the camera
  locked. `common.open_device` retries an empty enumeration for a while (a camera rebooting after a
  previous run) only when no `--device-id` is given; an explicit `--device-id` never retries and
  fails fast with one stderr line, camera present or not.
- `dai.Device.getAllAvailableDevices()` is always safe; it does not boot the device.

### Environment variables worth knowing

All of these strings are present in the installed 3.10.0 library (grepped from the binary);
`DEPTHAI_LEVEL`, `XLINK_LEVEL`, the timeouts and the device filters are documented at
https://docs.luxonis.com/software-v3/depthai/depthai-components/device/ ; `DEPTHAI_TELEMETRY` in
the depthai-ros README (https://github.com/luxonis/depthai-ros).

| Variable | Use |
|---|---|
| `DEPTHAI_LEVEL=trace\|debug\|info\|warn\|error\|off` | library log level; `debug` shows the USB boot sequence and negotiated speed |
| `XLINK_LEVEL=debug\|info\|warn\|error\|fatal\|off` | XLink (USB transport) log level |
| `DEPTHAI_TELEMETRY=0` | disable anonymised usage telemetry; set it on an offline drone so nothing tries to reach the network |
| `DEPTHAI_DEVICE_ID_LIST=OAKDS2EXAMPLEID000` | restrict the search to this camera if a second OAK is ever attached |
| `DEPTHAI_PLATFORM=rvc2`, `DEPTHAI_PROTOCOL=usb` | restrict the search to RVC2 / USB devices |
| `DEPTHAI_SEARCH_TIMEOUT`, `DEPTHAI_CONNECT_TIMEOUT`, `DEPTHAI_BOOTUP_TIMEOUT`, `DEPTHAI_RECONNECT_TIMEOUT` (ms) | lengthen if a slow USB path times out during boot or reconnect |
| `DEPTHAI_CRASHDUMP=0` | skip crash-dump retrieval after a device crash (faster restarts in the field) |

### Optional: depthai-ros for ROS 2 Humble (pointers only)

- Luxonis' v3 ROS driver supports Humble, Jazzy and Kilted
  (https://docs.luxonis.com/software-v3/depthai/ros/). On **Humble/Jazzy the v3 packages carry a
  `_v3` suffix**: `sudo apt install ros-$ROS_DISTRO-depthai-ros-v3` (the page says to enable the
  `ros2-testing` apt source until the binaries sync to the stable ROS repo). On Kilted and newer the
  suffix is dropped: `ros-$ROS_DISTRO-depthai-ros`. Humble is the distro for Ubuntu 22.04 /
  JetPack 6; on JetPack 7 (24.04) you would use Jazzy. Launch:
  `ros2 launch depthai_ros_driver_v3 driver.launch.py` (driver docs:
  https://docs.luxonis.com/software-v3/depthai/ros/driver/).
- The `_v3` packages vendor their own depthai-core (`depthai_ros_driver_v3` 3.3.0 on the `v3_humble`
  branch depends on `depthai_v3 >= 3.7.1`), independent of the 3.10.0 pip wheel in the venv. Both can
  be installed; they just cannot open the camera at the same time.
- From source (https://docs.luxonis.com/software-v3/depthai/ros/build-from-source/): the page
  spells the branches `v3-humble` / `v3-jazzy` for both `depthai-core` and `depthai-ros`, but as of
  2026-09-18 the Humble branch in both repos is `v3_humble` (underscore; depthai-ros also has
  `humble_v3`), while Jazzy has both `v3-jazzy` and `v3_jazzy`. The plain `humble` branch of
  depthai-ros is still the v2 driver. For a pinned build use the release **tags**
  (`v3.3.0-humble`, `v3.2.1-jazzy`, `v3.3.0-kilted`, https://github.com/luxonis/depthai-ros/tags).
  Build with `MAKEFLAGS="-j1 -l1" colcon build` or the repo's `build.sh`; the page warns a full
  parallel build "may take a lot of RAM", which is where Luxonis' Jetson swap-file advice comes in.
- Known Jetson build snag: `depthai_filters` needs `opencv2/ximgproc`, which JetPack's
  `nvidia-opencv-dev` does not ship (https://github.com/luxonis/depthai-ros/issues/497). The apt
  binaries avoid this; from source, `colcon build --packages-skip depthai_filters` or install a
  contrib-enabled OpenCV. A JetPack 6 + Jazzy attempt is in
  https://github.com/luxonis/depthai-ros/issues/739.
- The same udev rule from step 4 is required for the ROS driver.

### Optional: PX4 / Pixhawk 6C alongside the camera (pointers only)

- PX4's ROS 2 guide recommends "ROS 2 Jazzy LTS on Ubuntu 24.04" with "ROS 2 Humble LTS on Ubuntu
  22.04" as the alternative, and uses uXRCE-DDS (a client on PX4, `MicroXRCEAgent` on the companion)
  over serial/UDP (https://docs.px4.io/main/en/ros2/user_guide.html). That lines up with the JetPack
  choice above: JetPack 6 -> Humble, JetPack 7 -> Jazzy.
- Companion wiring: PX4 expects companions on `TELEM2`; for ROS 2 you switch that port from MAVLink
  to the uXRCE-DDS client (https://docs.px4.io/main/en/companion_computer/pixhawk_companion.html).
  Pixhawk 6C ports: https://docs.px4.io/main/en/flight_controller/pixhawk6c.html. The Jetson side is
  a 40-pin-header UART (`/dev/ttyTHS*`) or a USB-serial adapter; check logic levels before wiring a
  bare UART.
- Agent and messages: https://github.com/eProsima/Micro-XRCE-DDS-Agent, https://github.com/PX4/px4_msgs,
  https://github.com/PX4/px4_ros_com. Overview: https://docs.px4.io/main/en/companion_computer/
- USB budget: the OAK-D S2 and any USB telemetry/serial adapters share the same two Type-A stacks;
  put the camera on its own stack if you can.

---

## 3. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `getAllAvailableDevices()` returns `[]`, `lsusb -d 03e7:` shows nothing | No enumeration: cable, port, or camera not powered | Different cable (USB3, < 2 m), different Type-A port, `dmesg -w` while plugging in |
| `[warning] Insufficient permissions to communicate with X_LINK_UNBOOTED device ...`, script exits 2 | udev rule missing, or camera not re-plugged after adding it | Step 4; re-plug; `ls -l /dev/bus/usb/<bus>/<dev>` must show `rw-rw-rw-` |
| `Device: ... usb=HIGH (WARNING: not USB3)` | A 5 Gbps link that failed to train on this boot (seen once in five back-to-back boots on the laptop with a good cable and port), or a USB2 cable / USB2 hub if it repeats | Re-run the script (or re-plug) first; only if `HIGH` repeats swap cable/port and remove the hub; compare sysfs `speed` before and after boot; as a diagnostic `dai.Device(maxUsbSpeed=dai.UsbSpeed.HIGH)` proves the pipeline itself is fine |
| `X_LINK_COMMUNICATION_NOT_OPEN` / `X_LINK_ERROR` during a run | Marginal USB3 link (long/bad cable), brown-out on a hub | Shorter or active cable, powered USB3 hub or Y-adapter, `DEPTHAI_LEVEL=debug` for the failing transfer |
| Script exits 2 with `No OAK device found. Check the USB-C cable and that no other program has the camera open.` (at once with `--device-id`, after three retries without it) or `Device <id> is in use by another process.`, yet `lsusb -d 03e7:` still lists the camera; a bare `dai.Device()` raises `RuntimeError('No available devices')` after ~10-14 s | Another process (previous script, ROS driver) holds the camera, so it vanishes from enumeration (no `X_LINK_DEVICE_ALREADY_IN_USE` is raised; see "One process per camera") | `pgrep -af python`, kill it; wait ~10 s and retry; re-plug if it stays `X_LINK_BOOTED` |
| Camera disappears after minutes/hours, `dmesg` shows `tegra-xusb` errors | Host USB autosuspend / xusb quirk on the Orin Nano | Disable autosuspend (notes above), shorter cable, `apt full-upgrade` to 6.2.3; no confirmed fix upstream |
| `qt.qpa.plugin: Could not load the Qt platform plugin "xcb"` or "Cannot open an OpenCV window" | No X display in this shell (SSH) | `--headless`, or `DISPLAY=:0` with a local desktop session, or `ssh -X` |
| `open3d is not importable ...` / `Open3D could not create a window` from `03_point_cloud.py` | Open3D missing, or no display | `pip install open3d==0.20.0` (needs Python >= 3.10, glibc >= 2.35); the script keeps running headless either way |
| `pip install depthai==3.10.0` says "No matching distribution" | Python < 3.9 (JetPack 5), a non-CPython interpreter, or pip too old for `manylinux_2_28` tags | Use JetPack 6/7; `python -m pip install -U pip`; `python -c "import sysconfig; print(sysconfig.get_platform())"` must print `linux-aarch64` |
| `pip install open3d` fails to resolve | Python 3.9 or older, or glibc < 2.35 | Needs JetPack 6+ (Python 3.10, glibc 2.35); Open3D is optional, only `03_point_cloud.py` uses it |
| numpy resolves to 2.2.6 rather than 2.5.x | Expected on Python 3.10 | Nothing to fix; `requirements.txt` pins it that way on < 3.12 |
| Frames slow / dropped although `usb=SUPER` | Jetson in a low power mode, or the display path (X over SSH) is the bottleneck | `sudo nvpmodel -m 2 && sudo jetson_clocks` (confirm with `nvpmodel -q`); use `--headless` to measure camera-side throughput |
| `ImportError: ... GLIBCXX_...` | Old `libstdc++` (should not happen on JetPack 6+) or a conda env shadowing it | `sudo apt install --only-upgrade libstdc++6`; use the plain `python3 -m venv` from step 5 |

Debug recipe for anything USB-related, run over SSH with nothing else using the camera:

```bash
DEPTHAI_LEVEL=debug timeout 90 python camera/01_rgb_preview.py --headless --duration 5 2>&1 | tee /tmp/dai.log
dmesg | tail -n 50
```

---

## 4. Sources

- DepthAI 3.10.0 files on PyPI (wheel tags, sizes, upload date): https://pypi.org/project/depthai/3.10.0/#files ; release: https://github.com/luxonis/depthai-core/releases/tag/v3.10.0
- depthai-core README (pip index, `libudev-dev` for source builds, Android note), examples: https://github.com/luxonis/depthai-core ; https://github.com/luxonis/depthai-core/tree/main/examples/python ; AutoReconnect: https://github.com/luxonis/depthai-core/tree/main/examples/python/Misc/AutoReconnect
- Luxonis Device docs (`DEPTHAI_LEVEL`, `XLINK_LEVEL`, timeouts, device filters, `maxUsbSpeed`): https://docs.luxonis.com/software-v3/depthai/depthai-components/device/
- Luxonis USB deployment guide (udev rule, 480 Mb/s vs 5 Gb/s, cables, hubs, power, `maxUsbSpeed`): https://docs.luxonis.com/hardware/platform/deploy/usb-deployment-guide
- Luxonis "Deploy with NVIDIA's Jetson" (v2-era): https://docs.luxonis.com/hardware/platform/deploy/to-jetson
- Luxonis `install_dependencies.sh` (writes `/etc/udev/rules.d/80-movidius.rules`): https://docs.luxonis.com/install_dependencies.sh -> https://raw.githubusercontent.com/luxonis/depthai-python/refs/heads/main/docs/install_dependencies.sh
- udev permissions warning (issue title): https://github.com/luxonis/depthai/issues/765 ; the v2 troubleshooting page its body links (https://docs.luxonis.com/en/latest/pages/troubleshooting/#udev-rules-on-linux) now redirects to https://docs.luxonis.com/ and no longer serves the rule
- Qt xcb error on Jetson: https://github.com/luxonis/depthai/issues/1089 ; OAK-D Lite `PlgSrcMipi` regression after 3.6.1 (Orin Nano Super and Raspberry Pi 5; open, comments to 2026-09-16): https://github.com/luxonis/depthai/issues/1252
- OAK-D S2 hardware page: https://docs.luxonis.com/hardware/products/OAK-D%20S2 (power figures summarised in `camera/docs/oak-d-s2-specs.md`)
- Open3D 0.20.0 files: https://pypi.org/project/open3d/#files ; ARM notes: https://www.open3d.org/docs/release/arm.html
- opencv-python files: https://pypi.org/project/opencv-python/#files ; headless variant: https://pypi.org/project/opencv-python-headless/#files ; README (Qt in Linux wheels): https://github.com/opencv/opencv-python
- numpy 2.2.6 files: https://pypi.org/project/numpy/2.2.6/#files ; matplotlib 3.10.9 files: https://pypi.org/project/matplotlib/3.10.9/#files
- depthai-ros v3 docs: https://docs.luxonis.com/software-v3/depthai/ros/ ; driver: https://docs.luxonis.com/software-v3/depthai/ros/driver/ ; build from source: https://docs.luxonis.com/software-v3/depthai/ros/build-from-source/ ; repo, branches and tags: https://github.com/luxonis/depthai-ros ; Jetson build issues: https://github.com/luxonis/depthai-ros/issues/497 , https://github.com/luxonis/depthai-ros/issues/739
- NVIDIA JetPack 6.2.1 (SD image, firmware note): https://developer.nvidia.com/embedded/jetpack-sdk-621 ; 6.2.3 (latest 6.x, apt-upgrade note): https://developer.nvidia.com/embedded/jetpack-sdk-623 ; archive: https://developer.nvidia.com/embedded/jetpack-archive ; JetPack 7.2 downloads (Orin Family, no SD image): https://developer.nvidia.com/embedded/jetpack/downloads/archive-7.2 ; JetPack page (Ubuntu 24.04, kernel 6.8): https://developer.nvidia.com/embedded/jetpack
- Orin Nano devkit quick start (JetPack 7.2.1, USB ISO, firmware prerequisite): https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/quick_start.html ; hardware layout (USB ports, 3 A per stack, DisplayPort, DC jack): https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/hardware_layout.html
- JetPack 6.2 super-mode blog (`nvpmodel -m 2`, mode list): https://developer.nvidia.com/blog/nvidia-jetpack-6-2-brings-super-mode-to-nvidia-jetson-orin-nano-and-jetson-orin-nx-modules/
- OAK-D disconnects on the Orin Nano (tegra-xusb): https://forums.developer.nvidia.com/t/oak-d-disconnects-in-jetson-orin-nano-super-developer-kit-due-issues-with-xusb-tegra/349466
- Ubuntu package versions: Python 3.10 on jammy https://launchpad.net/ubuntu/jammy/+source/python3.10 ; pip 22.0.2 on jammy https://launchpad.net/ubuntu/jammy/+source/python-pip ; glibc 2.35 https://launchpad.net/ubuntu/jammy/+source/glibc ; glibc 2.39 https://launchpad.net/ubuntu/noble/+source/glibc
- PX4: https://docs.px4.io/main/en/ros2/user_guide.html ; https://docs.px4.io/main/en/companion_computer/ ; https://docs.px4.io/main/en/companion_computer/pixhawk_companion.html ; https://docs.px4.io/main/en/flight_controller/pixhawk6c.html
