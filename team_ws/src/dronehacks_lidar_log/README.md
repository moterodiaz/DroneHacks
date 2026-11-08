# dronehacks_lidar_log

Records the MS200K 2D lidar during a flight, to rebuild the course map from real scans. It runs
beside a team's normal launch and changes nothing about how the drone flies:

- it only **reads** the kit's `/skybeta/state` (`armed`) and the lidar's serial port;
- it **publishes nothing**, never talks to PX4, and a lidar fault only logs a warning.

## What it records

From arming until 5 s after disarm (the follower's bag timing), into
`~/team_ws/flights/<UTC>-lidar/` next to the flight's own recording:

| File | Contents |
|---|---|
| `lidar.bin` | `b"DHLIDAR1\n"`, then records of `struct "<dI"` (host wall-clock time, length) + the raw serial bytes |
| `meta.json` | device, baud, arm and close wall-clock times, bytes written |

The bytes are MS200 frames: parse them with `depthcam/course/ms200.py` (`FrameParser`,
`Assembler`). The wall-clock times are the clock the flight bags use, so each scan pairs with the
bag's pose and attitude. Every write is flushed, so a crash or power cut keeps everything up to it.
A re-arm within the 5 s tail keeps one file; a later arm starts a new folder.

The device is the `device` parameter (default `/dev/ms200`), else the first of
`/dev/serial/by-id/*CP210*`, `*Silicon_Labs*`, `/dev/ttyUSB*`, at 230400 baud. With none of
those, a CH340 adapter (`1a86:7523`) is read straight through usbfs (`ch340.py`): skybeta-11's
kernel (5.15.185-tegra) is built without `CONFIG_USB_SERIAL_CH341`, so the MS200K's own CH340
adapter gets no `/dev/ttyUSB*`. `ch340.py` sets it up as Linux's `ch341.c` does (baud divisor,
8N1, DTR/RTS) using the standard library only. The stack container runs privileged as root with
`/dev` mounted, so it can open either.

Ubuntu's `brltty-udev.service` (braille displays) claims every CH340 first, and the read then fails
with `Errno 16 Device or resource busy`. On skybeta-11 it was stopped and masked on 2026-11-08 (undo
with `systemctl unmask`):

```bash
sudo systemctl stop brltty-udev.service brltty.service
sudo systemctl mask brltty-udev.service brltty.service   # then replug the lidar
```

Check the port before a flight, in the container:
`ros2 run dronehacks_lidar_log lidar_probe` reads 3 s and prints the valid frame count, the
rotation rate and a few ranges, then PASS or FAIL. On skybeta-11, 2026-11-08: 377 valid frames/s
(about 4,500 points/s), rotation 10.0 Hz, PASS.

## Running it

The stack starts one launch from `/etc/skybeta/drone.env`. Each wrapper here includes a team launch
unchanged, with no arguments as the stack starts it, plus the logger:

| `COMPETITOR_LAUNCH` (with `COMPETITOR_PKG=dronehacks_lidar_log`) | Includes |
|---|---|
| `traj_window_0p5mps.launch.py` | `dronehacks_traj` `traj_nav_competitor_window_0p5mps.launch.py` |
| `follower.launch.py` | `dronehacks_course_follower` `competitor.launch.py` |

```bash
# on the drone: copy this folder to ~/team_ws/src, then
docker exec skybeta-stack bash -c 'source skybeta_env.sh && team_build.sh --packages-select dronehacks_lidar_log'
sudo sed -i -e 's/^COMPETITOR_PKG=.*/COMPETITOR_PKG=dronehacks_lidar_log/' \
  -e 's/^COMPETITOR_LAUNCH=.*/COMPETITOR_LAUNCH=traj_window_0p5mps.launch.py/' /etc/skybeta/drone.env
# drone on the launch mark, power-cycle; the competitor pane then shows
#   lidar on /dev/... at 230400 baud; recording only while armed
```

The included team package must be installed in the same workspace.

## Tests

```bash
python3 -m pytest team_ws/src/dronehacks_lidar_log/test -q
```

The recorder's arm/disarm windows and file format, the CH340 register values against the Linux
driver's (230400, 115200 and its 9600 default), the usbfs ioctl numbers, and the probe's frame
parsing; all without ROS, a serial port or the device.

Checked on the drone: the CH340 read (`lidar_probe` PASS above). Not checked: a flight with the
wrapped launches. The MS200K's eligibility
for judged runs is unresolved (`DRONEHACKS.md`, evidence boundaries): ask the organisers before
flying it in one.
