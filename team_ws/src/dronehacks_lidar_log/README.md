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
`/dev/serial/by-id/*CP210*`, `*Silicon_Labs*`, `/dev/ttyUSB*`, at 230400 baud. The stack container
runs privileged with `/dev` mounted, so it can open the port.

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

The recorder's arm/disarm windows and file format, without ROS or a serial port.

Not checked: any run on the drone (the real port, the wrapped launches). The MS200K's eligibility
for judged runs is unresolved (`DRONEHACKS.md`, evidence boundaries): ask the organisers before
flying it in one.
