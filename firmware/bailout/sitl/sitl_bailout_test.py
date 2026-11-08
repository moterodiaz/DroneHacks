#!/usr/bin/env python3
"""SITL test of the DH Bailout flight mode on PX4 v1.16.2 (SIH quadcopter, headless).

Runs the px4_sitl_default build that firmware/bailout/build.sh makes, with the
same dh_bailout module as the Pixhawk image. It checks that:

  1. dh_bailout registers as External Mode 1, and its name reaches the ground station
     (AVAILABLE_MODES);
  2. arming is refused while DH Bailout is selected;
  3. in Offboard flight at --speed, switching to DH Bailout stops the drone, lands it
     and disarms it, without leaving the mode.

With --fi, dh_fi_emulator first streams trajectory_setpoint at 10 Hz with the
forward velocity, as the kit's flight_interface does in every mode. That is the
competing stream DH Bailout has to override.

Usage (WSL, from anywhere):
  python3 sitl_bailout_test.py --build ~/dh-bailout-fw/px4/build/px4_sitl_default [--speed 1] [--fi]

SIH is a generic 1 kg quad, not drone 11. The SITL params are drone 11's where they
shape this test: COM_DISARM_LAND 5, MPC_TILTMAX_AIR 45, MPC_XY_P 0.95,
MPC_XY_VEL_P_ACC 1.8, LNDMC_Z_VEL_MAX 0.25.
"""
import argparse
import math
import os
import signal
import struct
import subprocess
import sys
import tempfile
import time

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil  # noqa: E402

M = mavutil.mavlink
EXT1_CUSTOM = (4 << 16) | (11 << 24)      # PX4_CUSTOM_MAIN_MODE_AUTO, SUB_MODE_EXTERNAL1
OFFBOARD_CUSTOM = 6 << 16
LAND_CUSTOM = (4 << 16) | (6 << 24)       # stock AUTO_LAND, for comparison runs
LAND_NAV_STATE = 18
LANDED_ON_GROUND = 1

# Set at boot through PX4_PARAM_<name> (init.d-posix/rcS). Drone 11's values where they
# shape the test (contracts/px4-baselines/skybeta-11-2026-11-07.params); no RC in SITL.
# The estimator stays SIH's default (GPS + baro): giving it drone 11's rangefinder height
# reference with position-only GPS made SIH's EKF lose its height (2.2 m estimated, 5 m
# true), which tests the simulator, not the mode. So the stop is judged on SIH's ground
# truth, and the landing on PX4's own land detector and disarm.
SITL_PARAMS = {
    "COM_RC_IN_MODE": 4, "NAV_DLL_ACT": 0, "NAV_RCL_ACT": 0,
    "COM_DISARM_LAND": 5.0, "COM_DISARM_PRFLT": 10.0, "MPC_TILTMAX_AIR": 45.0,
    "MPC_XY_P": 0.95, "MPC_XY_VEL_P_ACC": 1.8, "LNDMC_Z_VEL_MAX": 0.25, "LNDMC_TRIG_TIME": 1.0,
    "MPC_XY_VEL_MAX": 12.0, "COM_OF_LOSS_T": 1.0, "COM_FLTMODE1": 100,
}


def log(*a):
    print(*a, flush=True)


class Px4:
    def __init__(self, build, workdir, logfile):
        self.build = build
        env = dict(os.environ, PX4_SIM_MODEL="sihsim_quadx", PX4_SIMULATOR="sihsim", HEADLESS="1")
        env.update({f"PX4_PARAM_{k}": str(v) for k, v in SITL_PARAMS.items()})
        self.log = open(logfile, "w")
        self.proc = subprocess.Popen([os.path.join(build, "bin", "px4"), "-d", os.path.join(build, "etc")],
                                     cwd=workdir, env=env, stdout=self.log, stderr=subprocess.STDOUT,
                                     preexec_fn=os.setsid)

    def cmd(self, *args, check=False):
        r = subprocess.run([os.path.join(self.build, "bin", "px4-" + args[0]), *args[1:]],
                           capture_output=True, text=True, timeout=10)
        out = (r.stdout + r.stderr).strip()
        if check and r.returncode != 0:
            raise RuntimeError(f"{' '.join(args)} failed: {out}")
        return out

    def stop(self):
        try:
            self.cmd("shutdown")
            self.proc.wait(timeout=10)
        except Exception:
            os.killpg(self.proc.pid, signal.SIGKILL)
        self.log.close()


class Link:
    def __init__(self):
        self.m = mavutil.mavlink_connection("udpin:127.0.0.1:14550", source_system=255,
                                            source_component=190, dialect="development")
        hb = None
        t0 = time.time()
        while time.time() - t0 < 60:
            hb = self.m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if hb and hb.get_srcComponent() == 1 and hb.type != M.MAV_TYPE_GCS:
                break
        if not hb:
            raise RuntimeError("no heartbeat from SITL")
        self.sys = hb.get_srcSystem()
        self.hb = hb
        self.lpos = None
        self.landed_state = None
        self.samples = []
        self.acks = {}

    def pump(self, timeout=0.0):
        end = time.time() + timeout
        while True:
            msg = self.m.recv_match(blocking=timeout > 0, timeout=max(0.0, end - time.time()))
            if msg is None:
                if time.time() >= end:
                    return
                continue
            t = msg.get_type()
            if t == "HEARTBEAT" and msg.get_srcComponent() == 1 and msg.type != M.MAV_TYPE_GCS:
                self.hb = msg
            elif t == "LOCAL_POSITION_NED":
                self.lpos = msg
                self.samples.append((msg.time_boot_ms * 1e-3, msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz,
                                     self.armed(), self.hb.custom_mode, self.landed_state))
            elif t == "EXTENDED_SYS_STATE":
                self.landed_state = msg.landed_state
            elif t == "COMMAND_ACK":
                self.acks[msg.command] = msg.result
            elif t == "STATUSTEXT":
                log("   [px4]", msg.text)
            if time.time() >= end:
                return

    def armed(self):
        return bool(self.hb.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED)

    def command(self, cmd, *p, wait=3.0):
        p = list(p) + [0] * (7 - len(p))
        self.acks.pop(cmd, None)
        self.m.mav.command_long_send(self.sys, 1, cmd, 0, *p)
        end = time.time() + wait
        while time.time() < end and cmd not in self.acks:
            self.pump(0.05)
        return self.acks.get(cmd)

    def set_mode(self, custom):
        main = (custom >> 16) & 0xFF
        sub = (custom >> 24) & 0xFF
        return self.command(M.MAV_CMD_DO_SET_MODE, M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, main, sub)

    def param(self, name, value):
        if isinstance(value, int):
            enc = struct.unpack("<f", struct.pack("<i", value))[0]
            ptype = M.MAV_PARAM_TYPE_INT32
        else:
            enc, ptype = float(value), M.MAV_PARAM_TYPE_REAL32
        for _ in range(5):
            self.m.mav.param_set_send(self.sys, 1, name.encode(), enc, ptype)
            end = time.time() + 1.0
            while time.time() < end:
                msg = self.m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.2)
                if msg and msg.param_id.rstrip("\x00") == name:
                    return
        raise RuntimeError(f"param {name} not acknowledged")

    def interval(self, msg_id, hz):
        self.command(M.MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, 1e6 / hz)

    def offboard_sp(self, x=None, y=None, z=None, vx=None, vy=None, yaw=0.0):
        mask = (M.POSITION_TARGET_TYPEMASK_AX_IGNORE | M.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                M.POSITION_TARGET_TYPEMASK_AZ_IGNORE | M.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE |
                M.POSITION_TARGET_TYPEMASK_VZ_IGNORE)
        if x is None:
            mask |= M.POSITION_TARGET_TYPEMASK_X_IGNORE | M.POSITION_TARGET_TYPEMASK_Y_IGNORE
        if vx is None:
            mask |= M.POSITION_TARGET_TYPEMASK_VX_IGNORE | M.POSITION_TARGET_TYPEMASK_VY_IGNORE
        self.m.mav.set_position_target_local_ned_send(0, self.sys, 1, M.MAV_FRAME_LOCAL_NED, mask,
                                                      x or 0, y or 0, z, vx or 0, vy or 0, 0, 0, 0, 0, yaw, 0)


def available_mode_names(link):
    names = {}
    link.m.mav.command_long_send(link.sys, 1, M.MAV_CMD_REQUEST_MESSAGE, 0, 435, 0, 0, 0, 0, 0, 0)
    end = time.time() + 3
    while time.time() < end:
        msg = link.m.recv_match(type="AVAILABLE_MODES", blocking=True, timeout=0.3)
        if msg:
            names[msg.custom_mode] = msg.mode_name.rstrip("\x00")
            if len(names) >= msg.number_modes:
                break
    return names


def ulog_metrics(work, nav_state=23):
    """Stop and landing metrics from the run's ULog, on vehicle_local_position_groundtruth."""
    import bisect
    import glob
    try:
        from pyulog import ULog
    except ImportError:
        log("   pyulog missing: no ground-truth metrics")
        return None
    files = sorted(glob.glob(os.path.join(work, "log", "**", "*.ulg"), recursive=True))
    if not files:
        return None
    u = ULog(files[-1], ["vehicle_status", "vehicle_local_position_groundtruth", "vehicle_land_detected"])
    d = {x.name: x.data for x in u.data_list if x.multi_id == 0}
    vs, gt, ld = d["vehicle_status"], d["vehicle_local_position_groundtruth"], d["vehicle_land_detected"]
    t_sw = None
    for i in range(1, len(vs["timestamp"])):
        now = vs["nav_state"][i] == nav_state and vs["arming_state"][i] == 2
        before = vs["nav_state"][i - 1] == nav_state and vs["arming_state"][i - 1] == 2
        if now and not before:
            t_sw = int(vs["timestamp"][i])
    if t_sw is None:
        return None
    t_dis = next((int(vs["timestamp"][i]) for i in range(len(vs["timestamp"]))
                  if vs["timestamp"][i] > t_sw and vs["arming_state"][i] != 2), None)
    ts = gt["timestamp"]
    i0 = bisect.bisect_left(ts, t_sw)
    x0, y0, z0, vx0, vy0 = (float(gt[k][i0]) for k in ("x", "y", "z", "vx", "vy"))
    n = max(1e-6, math.hypot(vx0, vy0))
    rows = [(float(ts[i] - t_sw) * 1e-6, float(gt["x"][i]), float(gt["y"][i]), float(gt["z"][i]),
             float(gt["vx"][i]), float(gt["vy"][i]), float(gt["vz"][i])) for i in range(i0, len(ts))]
    z_ground = max(r[3] for r in rows)  # NED: the lowest point reached
    touchdown_t = next((r[0] for r in rows if r[3] > z_ground - 0.02 and abs(r[6]) < 0.05), float("nan"))
    air = [r for r in rows if r[0] < touchdown_t]
    along = [((r[1] - x0) * vx0 + (r[2] - y0) * vy0) / n for r in air]
    stop_i = next((i for i, r in enumerate(air) if math.hypot(r[4], r[5]) < 0.1), None)
    brake_i = next((i for i, r in enumerate(air) if math.hypot(r[4], r[5]) < 0.3), None)
    landed_t, unlands, prev = float("nan"), 0, None
    for i in range(len(ld["timestamp"])):
        if ld["timestamp"][i] < t_sw:
            continue
        landed = bool(ld["landed"][i])
        if landed and math.isnan(landed_t):
            landed_t = float(ld["timestamp"][i] - t_sw) * 1e-6
        if prev is True and not landed:
            unlands += 1
        prev = landed
    return {
        "stop_t": air[stop_i][0] if stop_i is not None else float("nan"),
        "travel_at_stop": along[stop_i] if stop_i is not None else float("nan"),
        "brake_t": air[brake_i][0] if brake_i is not None else float("nan"),
        "brake_m": along[brake_i] if brake_i is not None else float("nan"),
        "travel": max(along) if along else float("nan"),
        "reversal": max(max(along[:i + 1]) - along[i] for i in range(len(along))) if along else float("nan"),
        "lateral": max(abs(((r[1] - x0) * -vy0 + (r[2] - y0) * vx0) / n) for r in air) if air else float("nan"),
        "climb": max(z0 - r[3] for r in air) if air else float("nan"),
        "touchdown_t": touchdown_t,
        "landed_t": landed_t,
        "disarm_t": float(t_dis - t_sw) * 1e-6 if t_dis else float("nan"),
        "unlands": unlands,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--build", required=True)
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--alt", type=float, default=1.5)
    ap.add_argument("--fi", action="store_true", help="emulate flight_interface's 10 Hz stream")
    ap.add_argument("--target", choices=("bailout", "land"), default="bailout",
                    help="land: switch to PX4's stock Land instead, as a control run")
    ap.add_argument("--runs", default="~/dh-bailout-fw/sitl-runs",
                    help="parent directory for this run's PX4 working dir, log and samples.csv")
    a = ap.parse_args()
    build = os.path.abspath(os.path.expanduser(a.build))
    runs = os.path.expanduser(a.runs)
    os.makedirs(runs, exist_ok=True)
    work = tempfile.mkdtemp(prefix=time.strftime("%Y%m%d-%H%M%S-") + a.target + ("-fi-" if a.fi else "-"), dir=runs)
    target_custom = EXT1_CUSTOM if a.target == "bailout" else LAND_CUSTOM
    target_nav = 23 if a.target == "bailout" else LAND_NAV_STATE
    px4 = Px4(build, work, os.path.join(work, "px4.log"))
    checks = []

    def check(ok, what):
        checks.append((bool(ok), what))
        log(("  PASS " if ok else "  FAIL ") + what)

    try:
        link = Link()
        log(f"SITL up (work dir {work}); speed {a.speed} m/s, alt {a.alt} m, flight_interface emulated: {a.fi}")
        link.interval(M.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 50)
        link.interval(M.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 10)

        # 1. registration
        log(px4.cmd("dh_bailout", "start"))
        status = ""
        for _ in range(50):
            status = px4.cmd("dh_bailout", "status")
            if "mode 1 (nav 23)" in status:
                break
            link.pump(0.2)
        log("   " + status.replace("\n", "\n   "))
        check("mode 1 (nav 23)" in status, "registered as External Mode 1 (COM_FLTMODEx = 100)")
        link.pump(1.5)  # commander lists the mode as available once it has answered an arming check
        names = available_mode_names(link)
        check(names.get(EXT1_CUSTOM) == "DH Bailout", f"AVAILABLE_MODES names it: {names.get(EXT1_CUSTOM)!r}")

        # wait for the estimator
        t0 = time.time()
        while time.time() - t0 < 60:
            link.pump(0.2)
            if link.lpos is not None and abs(link.lpos.vz) < 0.05 and time.time() - t0 > 8:
                break

        # 2. arming refused in DH Bailout
        check(link.set_mode(EXT1_CUSTOM) == M.MAV_RESULT_ACCEPTED, "DH Bailout selectable on the ground")
        link.pump(0.5)
        r = link.command(M.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        link.pump(0.5)
        check(r != M.MAV_RESULT_ACCEPTED and not link.armed(), f"arming refused in DH Bailout (ack {r})")

        # 3. Offboard flight, then bail out
        x0, y0 = link.lpos.x, link.lpos.y
        for _ in range(20):
            link.offboard_sp(x=x0, y=y0, z=-a.alt)
            link.pump(0.05)
        check(link.set_mode(OFFBOARD_CUSTOM) == M.MAV_RESULT_ACCEPTED, "Offboard accepted")
        r = None
        for _ in range(30):
            link.offboard_sp(x=x0, y=y0, z=-a.alt)
            r = link.command(M.MAV_CMD_COMPONENT_ARM_DISARM, 1, wait=0.3)
            if r == M.MAV_RESULT_ACCEPTED:
                break
        check(r == M.MAV_RESULT_ACCEPTED, "armed in Offboard")
        t0 = time.time()
        while time.time() - t0 < 15:
            link.offboard_sp(x=x0, y=y0, z=-a.alt)
            link.pump(0.05)
            if abs(link.lpos.z + a.alt) < 0.1 and abs(link.lpos.vz) < 0.1 and time.time() - t0 > 4:
                break
        t0 = time.time()
        while time.time() - t0 < 12:
            link.offboard_sp(z=-a.alt, vx=a.speed, vy=0.0)
            link.pump(0.05)
            if time.time() - t0 > 2.5 and abs(link.lpos.vx - a.speed) < 0.05 * max(1.0, a.speed):
                break
        if a.fi:
            log(px4.cmd("dh_fi_emulator", "start", "-x", str(a.speed)))
            for _ in range(10):  # its stream runs while Offboard still flies
                link.offboard_sp(z=-a.alt, vx=a.speed, vy=0.0)
                link.pump(0.05)
        sw = link.lpos
        n_before = len(link.samples)
        r = link.set_mode(target_custom)
        check(r == M.MAV_RESULT_ACCEPTED, f"switched to {a.target} at {math.hypot(sw.vx, sw.vy):.2f} m/s")
        t_sw = sw.time_boot_ms * 1e-3
        t0 = time.time()
        while time.time() - t0 < 30:
            link.offboard_sp(z=-a.alt, vx=a.speed, vy=0.0)  # autonomy keeps streaming, as on the drone
            link.pump(0.05)
            if not link.armed():
                link.pump(1.0)
                break
        bail = px4.cmd("dh_bailout", "status")
        fi = px4.cmd("dh_fi_emulator", "status") if a.fi else ""

        # metrics
        with open(os.path.join(work, "samples.csv"), "w") as f:
            f.write("t_s,x,y,z,vx,vy,vz,armed,custom_mode,landed_state\n")
            for x in link.samples:
                f.write(",".join(str(v) for v in (round(x[0] - t_sw, 3),) + x[1:]) + "\n")
        s = [x for x in link.samples[n_before:] if x[0] >= t_sw]
        dir_x, dir_y = sw.vx, sw.vy
        n = max(1e-6, math.hypot(dir_x, dir_y))
        touch_t = next((x[0] - t_sw for x in s if x[9] == LANDED_ON_GROUND), float("nan"))
        disarm_t = next((x[0] - t_sw for x in s if not x[7]), float("nan"))
        # The stop is judged in the air: from the switch until touchdown.
        air = [x for x in s if not (x[0] - t_sw >= touch_t)]
        along = [((x[1] - sw.x) * dir_x + (x[2] - sw.y) * dir_y) / n for x in air]
        travel = max(along) if along else float("nan")
        reversal = max((max(along[:i + 1]) - along[i]) for i in range(len(along))) if along else float("nan")
        stop_t = next((x[0] - t_sw for x in s if math.hypot(x[4], x[5]) < 0.1), float("nan"))
        travel_at_stop = next((a_ for x, a_ in zip(air, along) if math.hypot(x[4], x[5]) < 0.1), float("nan"))
        climb = max((sw.z - x[3]) for x in air) if air else float("nan")
        lateral = max(abs(((x[1] - sw.x) * -dir_y + (x[2] - sw.y) * dir_x) / n) for x in air) if air else float("nan")
        # Heartbeats come at 1 Hz: judge the mode from the first one that shows the switch.
        first = next((i for i, x in enumerate(s) if x[8] == target_custom), None)
        in_mode = first is not None and all(x[8] == target_custom for x in s[first:] if x[7])
        modes_seen = sorted({hex(x[8]) for x in s[first or 0:] if x[7]})
        log(f"   stop {stop_t:.2f} s after {travel_at_stop:.3f} m; in the air: forward travel {travel:.3f} m, "
            f"pulled back {reversal:.3f} m, lateral {lateral:.3f} m, climb {climb:.3f} m; "
            f"landed {touch_t:.2f} s, disarmed {disarm_t:.2f} s; modes while armed {modes_seen}")
        log("   " + bail.replace("\n", "\n   "))
        if fi:
            log("   " + fi)
        check(in_mode, f"stayed in {a.target} until disarmed")
        if a.fi and a.target == "bailout":
            n_foreign = int(bail.split("foreign ")[1].split()[0])
            check(n_foreign > 0, f"emulated flight_interface samples were overridden ({n_foreign})")
    finally:
        try:
            px4.stop()
        except Exception as e:  # noqa: BLE001
            log("shutdown:", e)

    # The stop, judged on SIH's ground truth (what the airframe did), not on the estimate.
    gt = ulog_metrics(work, target_nav)
    if gt:
        v = a.speed
        log(f"   ground truth: under 0.3 m/s at {gt['brake_t']:.2f} s after {gt['brake_m']:.3f} m; under 0.1 m/s "
            f"at {gt['stop_t']:.2f} s; peak forward {gt['travel']:.3f} m, pulled back {gt['reversal']:.3f} m, "
            f"lateral {gt['lateral']:.3f} m, climb {gt['climb']:.3f} m; touchdown {gt['touchdown_t']:.2f} s, "
            f"landed (detector) {gt['landed_t']:.2f} s, disarmed {gt['disarm_t']:.2f} s, "
            f"landed->not-landed flips {gt['unlands']}")
        # SIH's GPS/baro estimator makes the drone wander after the stop, in stock Land
        # as much as here (README, SITL results), so the stop is judged on the brake
        # itself: true speed under 0.3 m/s. Absolute travel numbers are reported only.
        v = a.speed
        check(gt["brake_t"] < 0.6 + 1.0 * v, f"true speed under 0.3 m/s within {0.6 + 1.0 * v:.1f} s")
        check(gt["touchdown_t"] < 12, "touched down within 12 s")
        check(gt["disarm_t"] > gt["landed_t"], "PX4 disarmed after its land detector fired (COM_DISARM_LAND)")
    else:
        check(False, "ULog ground truth available")
    failed = [w for ok, w in checks if not ok]
    log(f"{len(checks) - len(failed)}/{len(checks)} checks passed; PX4 log: {os.path.join(work, 'px4.log')}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
