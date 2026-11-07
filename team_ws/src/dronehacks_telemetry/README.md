# dronehacks_telemetry

One self-describing folder per flight, and one report read from it. The same report runs on a
real-drone flight, a PX4 SITL bag and a FlyX lab bag, so the three compare like for like; a
simulator's truth log adds estimate-versus-truth.

- `scripts/record_flight.sh` runs on the Jetson **host** and records inside the running skybeta
  container (`ros2 bag record -s mcap`), detached, so it survives a dropped SSH session.
- `dronehacks_telemetry.analyze` reads the folder off-drone and writes `report.md` and
  `legs.csv` into it.
- `dronehacks_telemetry.ulog_triage` reads PX4's own `.ulg` (vision fusion, innovations,
  `EKF2_EV_DELAY` from the yaw-rate cross-correlation, vibration, failsafes, rangefinder).

Nothing here publishes a topic, writes a PX4 parameter or touches the organisers' stack: it only
subscribes (rosbag2) and reads.

## On the drone (drone 1, `skybeta-01`)

The stack must be up (`host/run.sh stack`, or the boot service). The script is wherever this
package sits under `~/team_ws/src`; below, `REC=~/team_ws/src/<repo>/team_ws/src/dronehacks_telemetry/scripts/record_flight.sh`
(adjust to where the team repo is checked out).

```bash
# 1. Before arming (every practice and scored flight):
bash $REC start                          # light profile: no images, a few MB
bash $REC start --label run3             # optional label in the folder name
bash $REC start --profile full           # chosen flights: + raw stereo images, zstd-compressed

# check it is running (prints the bag's size so far)
bash $REC status

# 2. After landing AND disarm:
bash $REC stop
#   -> ~/team_ws/flights/<UTC time>-<profile>[-label]/   (bag/, meta/, px4/)
```

`start` refuses to run twice; `stop` waits for rosbag2's clean shutdown (SIGINT), which writes
`bag/metadata.yaml`. If the manifest says `bag_closed_cleanly: no`, run
`ros2 bag reindex /root/team_ws/flights/<run>/bag -s mcap` inside the container.

Environment overrides: `SKYBETA_CONTAINER` (default the running `skybeta-stack`, else
`skybeta`), `TEAM_WS` (default `drone.env`'s, else `~/team_ws`), `COURSE_MAP` (a path inside the
container, when the mission's own `course_map_yaml` parameter is not the map flown).

### 3. Fetch PX4's log (ULog) into `<run>/px4/`

PX4 logs to its SD card from arming to disarm: the organisers' `px4/params/00_baseline.params`
sets `SDLOG_MODE 0` (arm to disarm) and `SDLOG_PROFILE 1` (PX4's default logged set only). Do not
change either; they are the organisers' settings (contracts/px4.md allowlist). The Jetson cannot
fetch it: TELEM2 carries uXRCE-DDS, not MAVLink (`MAV_1_CONFIG 0`). So, from a laptop:

- **QGroundControl**, FC on USB (or the telemetry radio): *Analyze Tools → Log Download →
  Refresh*, pick the newest entry (without GPS time PX4 names logs by session, so go by order and
  size), *Download*, and copy the `.ulg` into `<run>/px4/`.
- or the SD card: `log/<date or sessNNN>/*.ulg`.

The report prints the log's armed time beside the bag's: a mismatch means the wrong log.

### 4. Copy the folder off and analyze it

```bash
scp -r dronehacks@skybeta-01:~/team_ws/flights/<run> .
pip install numpy pyyaml mcap mcap-ros2-support pyulog   # zstandard too, for a full bag
PYTHONPATH=<repo>/team_ws/src/dronehacks_telemetry python3 -m dronehacks_telemetry.analyze <run>
```

The skybeta image has no Python `mcap`, so the analyzer runs on a laptop. With the package built
in the team workspace, `ros2 run dronehacks_telemetry analyze <run>` is the same command.

Options: `--course-to-odom YAW X Y` (the measured transform; default the mission's own
`course_to_odom_*` parameters from `meta/params/`, else the launch heading, reported as ASSUMED),
`--px4-frame odom|launch|auto`, `--arrival-radius-m` (default 0.12, the mission's), `--ulog`,
`--truth`, `--out`.

## What the folder holds

| path | what |
|---|---|
| `bag/` | rosbag2 mcap, every topic but the profile's excluded ones (`-a -x <regex>`, so topics that appear later are caught) |
| `meta/manifest.txt` | run id, profile, UTC start/stop, team git SHA + dirty flag (+ `team_git.diff`), a hash of `~/team_ws/src`, skybeta version/SHA, image, disk free, course map path + source + SHA-256, `bag_closed_cleanly` |
| `meta/params/<node>.yaml` | `ros2 param dump` of every node: the launch arguments as the nodes received them |
| `meta/processes.txt` | the `ros2 launch`/`run` command lines |
| `meta/topics_start.txt`, `topics_end.txt`, `nodes.txt` | `ros2 topic list -t` at start and stop, `ros2 node list` |
| `meta/timedatectl.txt`, `df.txt`, `drone.env` | clock sync state, free disk, the drone's identity file |
| `meta/course_map.yaml` | the map the mission read |
| `px4/` | put the `.ulg` here |
| `report.md`, `legs.csv`, `px4_params.params` | written by `analyze`; the params file is the ULog's parameters in QGC format, for `~/skybeta/tools/param_diff.py --dump` |

**Profiles.** `light` excludes `image|compressed|/depth|points|cloud|/visual_slam/vis/|_path$`:
every `/fmu/out`, `/fmu/in`, `/visual_slam/*` (status, odometry), `/skybeta/*`, `/tf`,
`/tf_static`, `/clock`, `/rosout`, the IMU and `camera_info` stay. `camera_info` is kept on
purpose: it carries every frame's header stamp, so the camera's rate and gaps are measurable
without images. `full` also keeps the raw stereo images (and depth if published) for an offline
cuVSLAM replay; it is zstd file-compressed (`--no-zstd` to skip) and the analyzer decompresses it
with `zstandard`. The lab measured about 1.6 GB per 30 s with images and a few MB without.

## What the report measures

Every time is seconds from the bag's first message, on the recorder's receive clock.

- **Timeline**: arming, modes, failsafe flags, cuVSLAM `vo_state` changes, vision gaps, EKF2
  fusion-flag dropouts (where `/fmu/out/estimator_status_flags` is published), the kit's
  `clamped`/`frozen`/link-lost flags, `/rosout` warnings and the team nodes' lines.
- **Setpoint tracking per leg** (`legs.csv`): the kit's `/fmu/in/trajectory_setpoint` against
  PX4's `vehicle_local_position`, both NED, in Offboard. A leg is a run of setpoints with no step
  over 5 cm, so a waypoint mission's legs are its waypoints. Per leg: time to reach the arrival
  radius, final and max error, overshoot past the target, cross-track, height error, heading swing,
  vision gaps during it.
- **Vision health**: rate and gaps (over 0.3 s) from header stamps for the camera, IMU, cuVSLAM
  odometry and the bridge's `timestamp_sample` (skybeta's `docs/debugging.md`: header stamps,
  never `ros2 topic hz`); time not tracking; bridge `reset_counter` changes.
- **Timing**: image stamp to cuVSLAM odometry received, image stamp to bridge message received,
  and the bridge's own `timestamp - timestamp_sample`. The serial hop to the FC is not in these.
- **Height**: the rangefinder (`/fmu/out/distance_sensor` if published, else
  `vehicle_local_position.dist_bottom`) against PX4's height, with the mount offset taken out.
- **Course**: PX4's estimate placed on the map through contracts/frames.md's transform; the
  `odom` and `launch` readings of PX4's frame are each compared with cuVSLAM and the closer one
  is used. Nearest wall face, spans within the 0.28 m prop reach, nearest boulder (passed over,
  so not a touch), closest pass per goal. This is the estimate's own view: drift is invisible to
  it on a real flight.
- **Truth** (`--truth truth.csv`, SITL or lab; needs `/clock` in the bag): PX4's estimate and
  cuVSLAM against the truth, p50/p95/max, and heading error.
- **ULog**: the full triage, its armed time against the bag's, and `px4_params.params`.

## Tests

```bash
python3 -m pytest team_ws/src/dronehacks_telemetry/tests -q
```

Synthetic data only: no ROS, no drone. The bag tests write a real mcap with `mcap-ros2-support`
and skip without it.

## What has been checked, and what has not (2026-11-07)

- **Recorder, measured off-drone**: `record_flight.sh start`/`status`/`stop` against a stand-in
  container named `skybeta-stack` (`ros:humble-ros-base` plus `ros-humble-rosbag2-storage-mcap`,
  on WSL, publishing `/skybeta/state`, `/oak/left/image_raw` and `/oak/left/camera_info`). Light
  left the image topic out and kept `camera_info`; full kept it and wrote `bag_0.mcap.zstd`;
  SIGINT closed both cleanly (`metadata.yaml` written, "Recording stopped" in `recorder.log`); a
  second `start` was refused; the files came back owned by the host user. Humble's
  `ros2 bag record --help` lists `-x/--exclude` and `--compression-format zstd`.
- **Analyzer, measured**: on a FlyX lab bag with the lab's truth log (`--truth`), it picked the
  `launch` frame by its agreement with cuVSLAM and showed the vision gap, EV-fusion dropout and
  failsafe sequence; its PX4-versus-truth error agreed with the team's earlier lab comparison.
- **Not checked without the drone**: the recorder inside the real skybeta image (its
  `skybeta_env.sh`, the uXRCE `/fmu` QoS as rosbag2 adapts to it, the bag rate with real images),
  `ros2 param dump` of the kit's nodes, whether PX4 v1.16 publishes `distance_sensor` and
  `estimator_status_flags` over DDS on the kit (the report falls back to `dist_bottom` and the
  ULog), the ULog fetch, and which PX4 frame reading holds on the real EKF2 (the report prints
  both against cuVSLAM).

## Provenance

`analyze`'s course scoring, the lab comparison and `ulog_triage.py` are adapted from the team's
own analysis tools (Michael Bar On's `DroneDepthCamTesting`: `sim/score_run.py`,
`sim/log_triage.py`). The recorder wraps the organisers' container environment
(`skybeta_env.sh`) but copies none of their code; it does not reuse their `record_bag.sh`
because that script's topic list is fixed (always images) and it runs `ros2 bag` in the
foreground of a shell that traps SIGINT, so a signal sent to the detached shell would not reach
the recorder.
