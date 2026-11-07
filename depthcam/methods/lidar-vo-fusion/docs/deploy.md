# Deploying `dronehacks_lidar` on drone 1

These steps take the package from this repo to drone 1's team workspace (hostname `skybeta-01`),
following the kit's procedure (`drone-snapshot/home/dronehacks/skybeta/docs/competitor_guide.md`
§5). Never log into or deploy to another drone: that gets a team disqualified (AGENTS.md §1).

## Once: install

On the drone, in the host shell, from a checkout of this repo (a Linux checkout keeps the
scripts' LF line endings):

```bash
~/skybeta/host/run.sh build --init          # first time only: writes ~/team_ws/Dockerfile
bash depthcam/deploy/install_on_drone.sh    # copies the package's build inputs to ~/team_ws/src/dronehacks_lidar, adds pyserial
~/skybeta/host/run.sh build                 # the team image, with pyserial
```

The script refuses to run on any host but `skybeta-01`. `depthcam/` is the package root. The
script copies only what colcon builds from it: `package.xml`, `setup.py`, `setup.cfg`,
`resource/`, `launch/`, `routes/`, the Python packages, the map and rules JSON, and the config.
It also copies the ROS tests, because the kit's container sees only `~/team_ws`. The research
code, recordings, CAD files and the STEP model stay off the drone.

### Run the ROS tests (once, after the first build)

In the kit's container, with the drone disarmed and the props off:

```bash
~/skybeta/host/run.sh shell
source /root/team_ws/install/setup.bash
cd /root/team_ws/src/dronehacks_lidar
ROS_DOMAIN_ID=101 python3 -m pytest tests/test_ros_nodes.py -q
```

The tests publish setpoints on `/skybeta/setpoint/pose`. `ROS_DOMAIN_ID=101` keeps them away
from the running stack (domain 1), and the kit's loopback-only DDS keeps them on this Jetson. The
tests refuse to run in the kit's container if the domain is the drone's own, or DDS isn't
loopback-only.
The stack's `competitor` window builds the workspace with `team_build.sh` when the stack starts.
Don't put `px4_msgs` in the workspace, because the image carries the matching one.

## Once: the lidar

- Give the MS200K's USB adapter a fixed name, `/dev/ms200`, with a udev rule on the host
  (course/README.md, *Wiring*).
- The stack container runs `--privileged` with `/dev/bus/usb` mounted. Check the name inside
  it: `~/skybeta/host/run.sh shell` then `ls -l /dev/ms200`.
- The mount values in `config/pose_fusion.example.yaml` are the ones from past tests: yaw offset
  87.4°, clockwise, scan plane 0.035 m above the body centre. If the lidar is ever moved,
  re-measure them, because every degree is about 1.7 cm of error per metre.

## The route

Two routes ship with the package, both planned from the STEP-model map
(`course/maps/course_v3_map.json`, extracted from `dronehacks-course-v3.step`) through the
ground-truth checkpoints (`DRONEHACKS.md` §4, `course/maps/course_v3_rules.json`):

| Route | CP4 | Length | Tightest room beyond the 5 cm margin |
|---|---|---|---|
| `window` (default) | through C2a's window (+5 bonus) | 15.5 m | +3 mm |
| `gate_b` | through Gate B | 16.2 m | +3 mm |

To replan after a change to the map or the rules, run `python -m follower.make_route` from
`depthcam/` and redeploy.

## Every run

1. **Choose the run's lidar policy and route.** In `/etc/skybeta/drone.env` (`sudoedit`), set
   `COMPETITOR_PKG=dronehacks_lidar` and `COMPETITOR_LAUNCH` to one of these:

   | `COMPETITOR_LAUNCH` | Lidar | Route |
   |---|---|---|
   | `lidar_nav_allowed.launch.py` | allowed | window |
   | `lidar_nav_allowed_gate_b.launch.py` | allowed | Gate B |
   | `lidar_nav_forbidden.launch.py` | forbidden | window |
   | `lidar_nav_forbidden_gate_b.launch.py` | forbidden | Gate B |

   The kit passes only these two keys into the container, so the choice lives in the file name.
2. **Put the drone on the launch mark, facing north, and restart the stack:**
   `docker rm -f skybeta-stack` (it is back in about 10 s). The restart re-zeroes cuVSLAM's
   origin there, which is the frame the route is flown in. If you skip it, the origin is
   wherever the stack started (at power-on). The follower's pre-takeoff check then keeps the
   drone on the ground, unless the lidar is allowed and corrects the offset.
3. **Wait for ready, before arming.** `/pose_fusion/ready` must be `true`; it always is when the
   lidar is forbidden. `/follower/status` must say `WAIT_FOR_OFFBOARD`, with the reason "on the
   start, sending the takeoff pose". That stream is what lets the kit enter Offboard: it asks once
   per arming, at the switch to Position, and only if a team setpoint is under 1 s old. If the
   reason says "sending nothing", fix what it names first. `/pose_fusion/lidar_policy` shows the
   policy (latched).
4. **The pilot arms, then switches to Position, within 10 s** (PX4 disarms a drone that hasn't
   taken off 10 s after arming, `COM_DISARM_PRFLT`). `offboard_active` goes true, and the follower
   takes off and flies.
   - **If the follower wasn't sending,** the kit logs `auto offboard: staying out of Offboard: no
     team setpoint received` and the vehicle stays on the ground under the pilot. That attempt is
     used up: disarm, fix the cause, and arm again.
   - **If something changed in the second before the switch,** Offboard starts, the follower's
     second check fails, and after 20 s it blocks and sends nothing. The interface holds the
     last takeoff pose, and the pilot takes over.
5. **The run:** CP1 → CP4, hold over the pad, land by velocity setpoint. PX4 disarms 2 s after
   touchdown (`COM_DISARM_LAND`). `/follower/status` lists the checkpoints crossed.
   - **If cuVSLAM loses tracking,** the follower holds still (zero velocity) from the first frame
     the Guard withholds (`/vo_guard/tracking`), and while no fresh pose arrives. If cuVSLAM
     re-initialises in a jumped frame, `/pose_fusion/aligned` stays false until the lidar has
     realigned the correction. Then the follower carries on. If the hold lasts 5 s
     (`realign_timeout_s`), it lands straight down where it is. Under the forbidden policy a frame
     jump can't be measured: it holds through the break, then flies on in the new frame.

By hand instead of the `competitor` window: stop what the window runs (attach, select it,
Ctrl-C; two flight interfaces fight), then
`ros2 launch dronehacks_lidar lidar_nav.launch.py lidar_policy:=allowed route:=window`.

## Before the first flight with the props on

- Clocks: `/vo_guard/status` must show `stamp_lag_s` between 0 and 0.5 s, and nothing
  withheld as `clock`. If every pose is withheld as `clock`, cuVSLAM's stamps aren't on the
  ROS clock, and nothing downstream will fly.
- Carry test: carry the drone by hand from the launch mark. `/pose_fusion/odometry` must follow
  it in the tracker frame (x = north, y = west).
- Props off, on the mark and ready: before arming, `/skybeta/setpoint/pose` must already carry the
  takeoff pose (z 0.73 m). Then arm and switch to Position: the follower must leave
  `WAIT_FOR_OFFBOARD`.
- Props off, 0.5 m off the mark: `/follower/status` must say "sending nothing", and the switch to
  Position must leave the vehicle in Position (the kit logs "no team setpoint received").
- First hover, then the first leg, with the pilot ready on the mode switch.
