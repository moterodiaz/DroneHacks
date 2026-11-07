# Setting up the drone: MS200K lidar + Jetson Orin Nano + Pixhawk 6C, flying the course plan

The drone flies the planned trajectory with PX4 offboard control. Its position comes from the MS200K 2D lidar matched against the course map. There's no GPS indoors, and no magnetometer: the lidar also gives the heading. Height comes from PX4's barometer, helped by the downward rangefinder.

Each step says what "working" looks like, so you can tell where it breaks. Do the stages in order. The bench stages (no frame or motors) have now run on the real parts: results, the setup fixes they needed, and what is still open are in [bench-results.md](bench-results.md).

## 1. What runs where

| Part | Runs | Talks to |
|---|---|---|
| MS200K lidar | spins at 10 Hz and streams points | Jetson, over its USB-serial adapter (`/dev/ms200`) |
| Jetson Orin Nano | `mavlink-routerd`: shares the one Pixhawk serial link with several programs | Pixhawk TELEM2 (serial, 921600 baud) |
| | `course/19_lidar_px4_bridge.py`: lidar → position and heading → PX4 | the lidar; PX4 via mavlink-router, UDP 14541 |
| | `course/15_fly_plan.py`: the plan → PX4 setpoints, arm, land, disarm | PX4 via mavlink-router, UDP 14540 |
| Pixhawk 6C (PX4 v1.17) | flight control; EKF2 fuses the lidar pose as "external vision" | Jetson (TELEM2); rangefinder (I2C); RC receiver |
| Laptop | QGroundControl over Wi-Fi (optional); planning (`14_plan_course.py`) and analysis (`16_analyze_flight.py`) | Jetson, UDP 14550 |

## 2. Wiring

### 2.1 Lidar to the Jetson

- Plug the MS200K's own USB-serial adapter into a Jetson USB port. Not the Pixhawk: PX4 has no driver for it, and the matching runs on the Jetson.
- Give it a fixed device name with a udev rule. The steps are in [course/README.md](../README.md), under *Wiring*. The guide below assumes `/dev/ms200`.
- Strain-relieve the cable. A yanked USB cable mid-run means no position.

### 2.2 Pixhawk TELEM2 to the Jetson

TELEM2 is PX4's companion-computer port. On the 6C it is `/dev/ttyS3` inside PX4, a 6-pin JST-GH connector.

The pinout follows the Pixhawk connector standard: 1 = +5 V, 2 = TX (out), 3 = RX (in), 4 = CTS, 5 = RTS, 6 = GND.

Option A, the Jetson's 40-pin header UART (3.3 V, the same as the Pixhawk):

| Pixhawk TELEM2 | Jetson 40-pin header |
|---|---|
| pin 2 TX | pin 10 UART1 RXD |
| pin 3 RX | pin 8 UART1 TXD |
| pin 6 GND | pin 6 GND |
| pin 1 +5 V | **not connected** |

- Find the port name. It's one of `/dev/ttyTHS0` or `/dev/ttyTHS1` (`/dev/ttyTCU0` is the Jetson's own console, so don't use that).
- To identify it, jumper pin 8 to pin 10 with the Pixhawk unplugged. Run `python3 -m serial.tools.miniterm /dev/ttyTHS0 921600` (needs `pip install pyserial`, section 4): what you type should echo back.
- If a login console holds the port, `sudo systemctl disable --now nvgetty` frees it.

Option B is a 3.3 V USB-TTL cable (FTDI) on TELEM2. It shows up as `/dev/ttyUSB*`, so give it its own udev name (e.g. `/dev/pixhawk`), distinct from the lidar's adapter.

Both need your user in the `dialout` group: `sudo usermod -aG dialout $USER`, then log out and back in.

Don't use the Pixhawk's USB port for flight. PX4 starts its USB link in a mode meant for a ground station, and a USB plug is the easiest connector on the drone to knock loose. TELEM2 is the companion port.

### 2.3 Rangefinder (VL53L1X) to the Pixhawk

- Mount it on the Pixhawk's I2C port, facing straight down.
- PX4's driver reads it 5 times a second (`vl53l1x.cpp`, 200 ms per measurement), with up to 4 m range in long mode.

### 2.4 Mounting the lidar

- **Height:** on top, with the scan plane above the propellers and nothing tall around it (GPS mast, antennas).
- **Measure the scan plane's height above the body centre.** Pass it as `--mount-z` (default 0.10 m).
- **Level:** the scan should be level with the frame. Tilt compensation uses PX4's attitude, which assumes the lidar is square to the Pixhawk.
- **Yaw:** any yaw works. `18_lidar_live.py check` measures `--yaw-offset` (stage 1).

## 3. PX4 parameters

Set these in QGroundControl (*Vehicle Setup → Parameters*, or the *MAVLink Console* with `param set NAME VALUE`). Reboot the Pixhawk afterwards. `python3 course/19_lidar_px4_bridge.py --print-params` prints the vision ones.

**Companion link**

| Parameter | Value | Why |
|---|---|---|
| `MAV_1_CONFIG` | TELEM 2 | a MAVLink instance on TELEM2 |
| `MAV_1_MODE` | Onboard (the default for instance 1) | onboard links send TIMESYNC, which the bridge needs to stamp its measurements in PX4's clock |
| `SER_TEL2_BAUD` | 921600 | matches mavlink-router |
| `UXRCE_DDS_CFG` | not TELEM 2 (Disabled, or another port) | the kit may ship with ROS 2 on TELEM2; our tools use MAVLink, and a port runs one or the other |

**Position and heading from the lidar ("external vision")**

| Parameter | Value | Why |
|---|---|---|
| `EKF2_EV_CTRL` | 9 | fuse horizontal position (bit 0) and yaw (bit 3) |
| `EKF2_GPS_CTRL` | 0 | no GPS indoors |
| `EKF2_MAG_TYPE` | 5 (None) | the heading comes from the lidar; a magnetometer near the course's metal is wrong anyway |
| `EKF2_EVP_NOISE` | 0.02 | default 0.1 m is a floor on the vision noise: PX4 would trust our 1 cm as 10 cm |
| `EKF2_EVA_NOISE` | 0.02 | same for the heading (rad) |
| `EKF2_EV_DELAY` | 0 | the bridge's timestamps are synced to PX4's clock |
| `EKF2_GPS_DELAY` | 0 | GPS is off, but its 110 ms default still pushes EKF2's fusion horizon 156 ms into the past; the forward projection cost about 1 cm in SITL |
| `EKF2_DELAY_MAX` | 60 (PX4 may raise it to 100 itself) | must stay above the bridge's delay. Check `age_ms` p99 in `course/out/bridge/*/summary.json`: 42 ms in SITL; the Jetson may be slower |
| `EKF2_EV_POS_X/Y/Z` | 0 | the bridge already reports the body centre, not the lidar |
| `EKF2_TAU_POS`, `EKF2_TAU_VEL` | 0.1 (optional) | PX4's output follows the fused estimate faster. In SITL: 4.4 → 4.0 cm RMS, max 10.0 → 9.3 cm. Try it after the first hovers: it also passes more noise to the controller |

**Height**

| Parameter | Value | Why |
|---|---|---|
| `EKF2_HGT_REF` | 0 (barometer) | default 1 is GPS. Don't use 2 (range) as the reference: the return flight crosses the 1.73 m barrier, and a rangefinder reference would see the floor "jump" 1.7 m. With the barometer, PX4's height zero is not the floor (+0.69 m on one SITL boot); the player and the bridge measure the floor while PX4 says "landed", so start them with the drone on the floor |
| `SENS_EN_VL53L1X` | 1 | start the rangefinder driver |
| `EKF2_RNG_CTRL` | 1 (conditional) | rangefinder helps the barometer near the ground |
| `EKF2_RNG_POS_X/Y/Z` | its offset from the body centre (FRD, m) | |

**Flight**

| Parameter | Value | Why |
|---|---|---|
| `MPC_TKO_RAMP_T` | 0.5 (after the first flights) | the default 3 s takeoff ramp cost ~3.5 s in SITL; 0.5 saved ~2.6 s |
| `COM_OF_LOSS_T`, `COM_OBL_RC_ACT` | 1 s; *Land* | what PX4 does if the player stops sending setpoints |
| `NAV_DLL_ACT` | 0 if QGC is not connected during runs | otherwise a "ground station lost" failsafe can trigger |

Check after the reboot, in the MAVLink Console. It only shows external vision as fused once stage 3 is running:

```
listener estimator_status_flags     # cs_ev_pos: True, cs_ev_yaw: True, cs_gps: False, cs_mag_hdg: False
listener vehicle_local_position     # xy_valid: True
```

## 4. Jetson software

1. **JetPack, repo and Python environment.** Follow [camera/docs/jetson-orin-nano-setup.md](../../camera/docs/jetson-orin-nano-setup.md), sections 1–5.
2. **The two packages the flight tools add:** `python3 -m pip install pyserial pymavlink`.
3. **Plans.** Copy them (`course/out/<route>_*_px4_ned.csv` and `_summary.json`) from the laptop, or make them on the Jetson with `14_plan_course.py`. `course/out/` is not in git.
4. **mavlink-router** (one serial link, several programs):

   ```
   sudo apt install git meson ninja-build pkg-config gcc g++ systemd
   git clone https://github.com/mavlink-router/mavlink-router.git
   cd mavlink-router && git submodule update --init --recursive
   meson setup build . && ninja -C build && sudo ninja -C build install
   ```

   Then write `/etc/mavlink-router/main.conf`. Put in your real serial device; the laptop connects to the Jetson's IP on 14550.

   ```
   [General]
   TcpServerPort = 0
   ReportStats = false

   [UartEndpoint pixhawk]
   Device = /dev/ttyTHS0
   Baud = 921600

   [UdpEndpoint player]
   Mode = Normal
   Address = 127.0.0.1
   Port = 14540

   [UdpEndpoint bridge]
   Mode = Normal
   Address = 127.0.0.1
   Port = 14541

   [UdpEndpoint qgc]
   Mode = Server
   Address = 0.0.0.0
   Port = 14550
   ```

   Start it with `sudo systemctl enable --now mavlink-router` if the install added the service. Otherwise run `mavlink-routerd` in its own terminal.

   Check it with `python3 -c "from pymavlink import mavutil; c = mavutil.mavlink_connection('udpin:0.0.0.0:14540'); print(c.wait_heartbeat(timeout=10))"`, which should print a HEARTBEAT.

## 5. Bring-up, one stage at a time

**Stage 1: the lidar on the bench** (Jetson or laptop, no props):

```
python3 course/18_lidar_live.py check --port /dev/ms200
```

- **What working looks like:** about 10 Hz, about 450 points per turn, 0 CRC errors, few lost frames.
- **Yaw offset:** put a box in front of where the drone's nose will be. The tool prints the `--yaw-offset` to use.
- **Sweep direction:** a box on the left must read about +90°. If it reads −90°, add `--counter-clockwise`.
- **Record a recording** for later timing checks: `record --seconds 30 -o bench.ms200`.

**Stage 2: the lidar on the drone, props off**

- Run `check` again. Nothing should come back closer than `--range-min` (0.25 m). If the arms or legs show up, raise it.
- Note the returns percentage.

**Stage 3: lidar → PX4, props off.** This needs the course, or a mock of part of it built to the drawing. The matcher only localises against the course map.

1. Put the drone on the launch mark, nose north, and power it on.
2. Start the bridge:

   ```
   python3 course/19_lidar_px4_bridge.py --port /dev/ms200 --mavlink udpin:0.0.0.0:14541 \
       --summary course/out/window_yaw90_summary.json --yaw-offset <deg> --mount-z <m>
   ```

What working looks like:
- The status line shows `sent` counting up. `lost` stays near 0. The pose matches the launch mark: x −2.51, y 0.91, yaw 90.
- After about 50 s, the line says `(PX4 sync converged)`. That's 500 clock exchanges with PX4.
- The MAVLink Console checks from section 3 pass.
- Carry the drone along the first corridor: QGC's position follows it, and the heading follows the nose.

Logs go to `course/out/bridge/<stamp>_lidar/`.

**Stage 4: first hover** (safety pilot on the RC, bridge running):

- Take off in *Position* mode. It needs the lidar position: if PX4 refuses Position mode, stage 3 isn't working.
- Hover at about 0.8 m over the launch mark for a minute. It should hold within a few cm.
- Then stop the bridge (Ctrl-C) while hovering low, and see what PX4 does. That's the lidar-loss behaviour you'll rely on.

**Stage 5: the plan, slow first.** Bridge running, safety pilot ready.

```
python3 course/15_fly_plan.py course/out/window_yaw90_px4_ned.csv --connect udpin:0.0.0.0:14540 --slow 3 --no-return
python3 course/16_analyze_flight.py course/out/flights/<log folder>
```

- Repeat at `--slow 2`, then at full speed.
- Set the planner's `--margin` from the off-path p95 that `16` reports for your drone. The SITL x500 strayed 0.20 m at full speed.
- Then try `--disarm force` (disarms as soon as it's on the floor) and `MPC_TKO_RAMP_T` 0.5.

**Stage 6: the return flight.** The return crosses the barrier at 2.3 m, where the lidar sees only the perimeter nets.

- In simulation, localisation holds if the nets return at least 10% of beams. It fails below 5%.
- Hold the lidar at about 2.3 m inside the course and run `check`. Returns of 10% or more mean the return can fly. Otherwise fly `--no-return`.

## 6. Run sheet at the event

```
# 10 min before the block: drone on the launch mark, nose north, powered; mavlink-router running
python3 course/19_lidar_px4_bridge.py --port /dev/ms200 --mavlink udpin:0.0.0.0:14541 \
    --summary course/out/<plan>_summary.json --yaw-offset <deg> --mount-z <m>
# wait for "PX4 sync converged" and sent counting; QGC shows the drone on the mark
python3 course/15_fly_plan.py course/out/<plan>_px4_ned.csv --connect udpin:0.0.0.0:14540 [--disarm force] [--no-return]
# between runs, back on the mark: the bridge keeps running; start the player again
python3 course/16_analyze_flight.py course/out/flights/<log folder>
```

## 7. When things go wrong

- **The lidar loses the match.** The bridge stops sending (`lost` counts up), and PX4 dead-reckons on its IMU. If the lidar doesn't come back, PX4 declares the position invalid and its failsafe takes over. Check what that does in stage 4, before relying on it.
- **The drone strays more than 0.5 m from the plan.** The player commands Land (`--abort-error`).
- **The bridge never starts sending.** It only trusts the lidar after 3 good matches in a row, starting from the launch mark facing north. Put the drone on the mark, or pass `--start-yaw`.
- **The safety pilot can always take over** by switching out of Offboard on the RC.

## 8. Still to measure on the real hardware

- `--yaw-offset`, sweep direction, `--mount-z`, `--range-min` (stages 1–2).
- **The lidar's timing.** Which point a frame's timestamp belongs to, and the fixed delay (`--latency-ms`, assumed 5 ms). A wrong value shows up as a lag between the lidar pose and PX4's in fast flight.
- **Net returns at 2.3 m** (stage 6).
- **The real drone's off-path error**, which sets `--margin`, and its height, which the planner assumes is 0.20 m (`--height`).
- **The course as built vs the drawing.** Pieces placed 4 cm off cost 2.5–3 cm of lidar error. Measure the walls on arrival and fix the map if they're off.

## Simulation result this is based on

PX4 v1.17 SITL in Gazebo, with these parameters and GPS and magnetometer off. The lidar was synthetic MS200K scans cast from the simulated drone's true pose, fed through `19_lidar_px4_bridge.py --sim`.

**What worked:** the gentle plan's run and return flew to completion three times on the lidar alone.

| | Measured against the simulator's truth |
|---|---|
| Lidar pose | ~1 cm RMS, heading p95 0.7° |
| Pose sent | 18–19 ms after the measurement (p99 37–42 ms) |
| PX4's position | 4.0–4.4 cm RMS, worst 9–10 cm |
| Off the planned path | 0.2 m (p95) |

**Where it fell short:**
- The drone still clipped the north net after CP1 by 2–3 cm. That's the simulated x500's control tracking, the same as on GPS. Your drone's tuning and the planner's margin decide this, which is why stage 5 measures it.

**Not covered by the simulation:**
- The real sensor's timing and noise;
- the drone's own frame blocking beams;
- the nets' real reflectivity;
- the course as built.

Details: [course/README.md](../README.md), *Lidar → PX4*.
