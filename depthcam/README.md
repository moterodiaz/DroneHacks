# Depth-camera drone localization testing

Experiments in GPS-denied UAV localization with an OAK-D S2 depth camera, aimed at a 7-inch quad with an
NVIDIA Jetson Orin Nano and a Pixhawk 6C. The repo is organised so that several localization methods
can be researched side by side on the same camera, the same recordings and the same evaluation
protocol: each method lives in its own folder under [methods/](methods/) with its own README.

## Layout

| Folder | What is in it |
|---|---|
| [camera/](camera/) | OAK-D S2 + DepthAI v3 basics: the `00`-`06` ladder of self-checking scripts (device info, RGB, stereo depth, point cloud, IMU, record, replay), `common.py`, the camera and Jetson reference docs |
| [datasets/](datasets/) | the dataset recorder (`07`, EuRoC layout: stereo pair + IMU), the format doc, IMU calibration fit and stored corrections |
| [methods/](methods/) | one folder per localization method, each with a README: [cuvslam-vio](methods/cuvslam-vio/) (working), [cuvslam-vslam](methods/cuvslam-vslam/) (tried), [lidar-vo-fusion](methods/lidar-vo-fusion/) (Guard + lidar localiser + drift correction applied to the flight commands + route follower, packaged as `dronehacks_lidar`; written, not flown), [obstacle-detection](methods/obstacle-detection/), [_template](methods/_template/) for a new one |
| [course/](course/) | the competition course: STEP → map (`12`), Gazebo world + PX4 SITL launcher (`13`), fastest safe trajectory through the checkpoints (`14`), flight-log analysis (`16`), MS200K 2D lidar: smear/tilt study (`17`), serial reader (`ms200.py`), scan-to-map matcher (`lidar_localize.py`), bench map (`20`); [hardware setup](course/docs/hardware-setup.md), [bench results](course/docs/bench-results.md). Ported from DroneDepthCamTesting `course-planner-lidar`; the tools that command PX4 directly were not ported ([provenance](methods/lidar-vo-fusion/README.md#provenance)) |
| `cuvslam_oak/` | the shared Python package: camera pipeline (`oak_source.py`), EuRoC writer / reader (`dataset.py`), EEPROM calibration maths (`rig.py`), IMU correction (`imu_calib.py`), and the cuVSLAM tracking loop, bridge and live view |
| `pose_fusion/` | the drone's lidar correction: the Guard on cuVSLAM, the lidar localiser around the matcher, the drift filter, the setpoint correction (`C⁻¹ ∘ setpoint`), course facts from the map files, the course-to-tracker frame transform (contracts/frames.md), the per-run `lidar_policy`, ROS message conversion (no rclpy) |
| `follower/` | the route follower (ROS-free): `make_route.py` plans the route from the STEP-model map through the ground-truth checkpoints; `mission.py` flies it (pre-takeoff checks, take off, follow at the planned heading, hold, land) |
| `routes/` | the planned routes, `course_v3_window.json` and `course_v3_gate_b.json` (format `dronehacks-lidar-route/1`) |
| `dronehacks_lidar/`, `launch/`, `package.xml`, `setup.py` | the ROS 2 package `dronehacks_lidar` (this folder is its root): the four nodes and the launch files |
| `deploy/` | `install_on_drone.sh`: installs the package on drone 11 (refuses any other host); see [methods/lidar-vo-fusion/docs/deploy.md](methods/lidar-vo-fusion/docs/deploy.md) |
| `tests/` | `python -m pytest tests -q`: 300 pass, 9 skip, no camera, no GPU, no lidar, no PX4 (a fake cuVSLAM stands in for the engine; synthetic scans for the lidar; a stand-in PX4 for the closed loops). `tests/test_ros_nodes.py` needs ROS 2 Humble and px4_msgs and skips without them |
| `recordings/`, `snapshots/` | outputs, git-ignored |

## Hardware

Measured from the unit itself on 2026-09-18 (`camera/00_device_info.py`):

- **OAK-D-S2-AF** (autofocus colour camera), device id `OAKDS2EXAMPLEID000`, board `DM9098` rev `R9M2E9`, platform **RVC2** (Myriad X), USB-C, normally links at **USB3 SUPER** (5 Gbps). No IR projector (that is the OAK-D Pro).
- **CAM_A**: IMX378 4056x3040 colour, autofocus (fx = 3065.9 px at full resolution).
- **CAM_B / CAM_C**: OV9282 1280x800 mono global shutter; stereo left = CAM_B, right = CAM_C, **baseline 7.5 cm** (74.97 mm calibrated).
- **IMU**: BNO086, firmware 3.9.9 (has sensor fusion, so a rotation vector is available).
- Depth from this unit's calibration: near limit ~63 cm at 800P / ~31 cm at 400P or with extended disparity; Luxonis' accuracy claim is < 2 % below 4 m (measured on an OAK-D Pro, not an S2).

Full spec sheet with sources, docs-vs-measured cross-checks and the drone implications: [camera/docs/oak-d-s2-specs.md](camera/docs/oak-d-s2-specs.md).

## Setup

### Windows (this laptop)

```bat
py -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
```

- On this machine use `py` (the Windows Python launcher), not `python`, to create the venv. Once the venv is activated, `python` resolves to `.venv\Scripts\python.exe` and the `python scripts/...` commands below work as written.
- `requirements.txt` pins `depthai==3.10.0`, `opencv-python`, `numpy` and `open3d==0.20.0` (Open3D is only used by `03_point_cloud.py`; the script falls back to headless without it).
- Plug the camera into a **USB3 port with the supplied cable**. Every script prints a `Device: ... usb=SUPER` line after boot; `usb=HIGH` means USB2 and comes with a `!!! WARNING` on stderr (see Troubleshooting).
- Optional: `pip install matplotlib` for the live plot in `04_imu.py --plot`. It is not installed in the venv; without it the script prints a one-line notice and streams normally.
- `pip install -r requirements-dev.txt` adds `pytest` (the only development dependency); `python -m pytest tests -q` then runs the cuVSLAM suite (61 tests, ~6 s) against the built-in fake cuVSLAM. It never opens the camera: `tests/conftest.py` replaces `dai.Device`, `dai.DeviceBase` and `dai.Pipeline` with functions that fail the test, and the CLI smoke tests only enumerate.

### Jetson Orin Nano

[camera/docs/jetson-orin-nano-setup.md](camera/docs/jetson-orin-nano-setup.md) is the step-by-step: flash JetPack 6.2.x (Ubuntu 22.04, Python 3.10), install the Luxonis `03e7` udev rule, create a venv and install the same `requirements.txt` (numpy resolves to 2.2.6 on Python 3.10, which is intended), confirm the link is `usb=SUPER` and not `HIGH`, then run `timeout 90 python camera/01_rgb_preview.py --headless --duration 10` as the smoke test. It also covers USB autosuspend disconnects on the Orin Nano, OpenCV/Open3D GUI on aarch64, and pointers for depthai-ros and PX4. The camera was not opened while that doc was researched; it has not been run on a Jetson yet.

Linux hosts for cuVSLAM (WSL2 on the laptop, the Jetson): [methods/cuvslam-vio/README.md](methods/cuvslam-vio/README.md).

## Working with the camera

Keep sessions short: the OAK-D S2 runs hot after a few minutes. Every script prints when the device is
released. One process at a time can hold the camera. Commands and flags: [camera/README.md](camera/README.md).

## Evaluation protocol

Record a walk that starts and ends on a floor mark, facing the same way (`datasets/07_record_dataset.py`,
protocol in [methods/cuvslam-vio/docs/cuvslam.md](methods/cuvslam-vio/docs/cuvslam.md) section 3.2),
track it, then judge by end-point offset / path length and the heading error at the end; repeat runs
before comparing settings. Results per method are in the method READMEs.

## Status (2026-10-01)

- cuVSLAM stereo-inertial odometry works on the laptop (engine in WSL2) with a live view; 2.3 % return-
  to-start drift on a 57 s full-resolution walk, 3-30 % on bare-walled rooms; details in
  [methods/cuvslam-vio/README.md](methods/cuvslam-vio/README.md).
- cuVSLAM's loop-closure layer gave no correction on our recordings: [methods/cuvslam-vslam/README.md](methods/cuvslam-vslam/README.md).
- Not yet done: the Jetson bring-up (no board on hand yet), the PX4 odometry bridge, any other method.

## Docs

- Camera: [camera/docs/depthai-v3-cheatsheet.md](camera/docs/depthai-v3-cheatsheet.md), [camera/docs/oak-d-s2-specs.md](camera/docs/oak-d-s2-specs.md), [camera/docs/jetson-orin-nano-setup.md](camera/docs/jetson-orin-nano-setup.md)
- Datasets: [datasets/dataset-format.md](datasets/dataset-format.md)
- cuVSLAM: [methods/cuvslam-vio/docs/cuvslam.md](methods/cuvslam-vio/docs/cuvslam.md) (user guide), [cuvslam-setup.md](methods/cuvslam-vio/docs/cuvslam-setup.md), [cuvslam-api-v17.md](methods/cuvslam-vio/docs/cuvslam-api-v17.md), [oak-to-cuvslam-calibration.md](methods/cuvslam-vio/docs/oak-to-cuvslam-calibration.md)
