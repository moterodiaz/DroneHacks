#!/usr/bin/env python3
"""Side-by-side table of SITL runs (sitl_bailout_test.py), from each run's ULog ground truth.

  python3 compare_runs.py [~/dh-bailout-fw/sitl-runs]

For every run it reports how hard the mode braked: the time and forward distance until
the airframe's true horizontal speed first fell below 0.3 and 0.2 m/s, and the peak
forward travel before touchdown. It also reports the touchdown, the land detector, the
disarm, and how often the land detector dropped "landed" again. SIH's GPS/baro estimator
makes the drone wander after it has stopped, in stock Land as much as in DH Bailout.
So compare the modes with each other, not against drone-11 numbers.
"""
import bisect
import glob
import math
import os
import sys

from pyulog import ULog

NAV = {"bailout": 23, "land": 18}


def analyse(run):
    name = os.path.basename(run.rstrip("/"))
    target = "land" if "-land-" in name else "bailout"
    files = sorted(glob.glob(os.path.join(run, "log", "**", "*.ulg"), recursive=True))
    if not files:
        return None
    u = ULog(files[-1], ["vehicle_status", "vehicle_local_position_groundtruth", "vehicle_land_detected"])
    d = {x.name: x.data for x in u.data_list if x.multi_id == 0}
    vs, gt, ld = d["vehicle_status"], d["vehicle_local_position_groundtruth"], d["vehicle_land_detected"]
    t_sw = None
    for i in range(1, len(vs["timestamp"])):
        on = vs["nav_state"][i] == NAV[target] and vs["arming_state"][i] == 2
        was = vs["nav_state"][i - 1] == NAV[target] and vs["arming_state"][i - 1] == 2
        if on and not was:
            t_sw = int(vs["timestamp"][i])
    if t_sw is None:
        return None
    t_dis = next((int(vs["timestamp"][i]) for i in range(len(vs["timestamp"]))
                  if vs["timestamp"][i] > t_sw and vs["arming_state"][i] != 2), None)
    ts = gt["timestamp"]
    i0 = bisect.bisect_left(ts, t_sw)
    x0, y0, z0, vx0, vy0 = (float(gt[k][i0]) for k in ("x", "y", "z", "vx", "vy"))
    v0 = math.hypot(vx0, vy0)
    rows = [((ts[i] - t_sw) * 1e-6, float(gt["x"][i]), float(gt["y"][i]), float(gt["z"][i]),
             math.hypot(float(gt["vx"][i]), float(gt["vy"][i])), float(gt["vz"][i])) for i in range(i0, len(ts))]
    z_ground = max(r[3] for r in rows)
    touch = next((r[0] for r in rows if r[3] > z_ground - 0.02 and abs(r[5]) < 0.05), float("nan"))
    air = [r for r in rows if r[0] < touch]

    def along(r):
        return ((r[1] - x0) * vx0 + (r[2] - y0) * vy0) / max(v0, 1e-6)

    def first_below(v):
        r = next((r for r in air if r[4] < v), None)
        return (r[0], along(r)) if r else (float("nan"), float("nan"))

    landed_t, flips, prev = float("nan"), 0, None
    for i in range(len(ld["timestamp"])):
        if ld["timestamp"][i] < t_sw:
            continue
        landed = bool(ld["landed"][i])
        if landed and math.isnan(landed_t):
            landed_t = (ld["timestamp"][i] - t_sw) * 1e-6
        flips += prev is True and not landed
        prev = landed
    return {
        "run": name, "target": target, "fi": "-fi-" in name, "v0": v0,
        "t03": first_below(0.3), "t02": first_below(0.2),
        "peak": max((along(r) for r in air), default=float("nan")),
        "climb": max((z0 - r[3] for r in air), default=float("nan")),
        "touch": touch, "landed": landed_t, "disarm": (t_dis - t_sw) * 1e-6 if t_dis else float("nan"),
        "flips": flips,
    }


def main():
    root = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else "~/dh-bailout-fw/sitl-runs")
    print(f"{'run':44} {'mode':7} {'fi':3} {'v0':>5} | {'<0.3 m/s':>14} | {'<0.2 m/s':>14} | {'peak':>5} "
          f"{'climb':>5} | {'touch':>5} {'landed':>6} {'disarm':>6} {'flips':>5}")
    for run in sorted(glob.glob(os.path.join(root, "*/"))):
        r = analyse(run)
        if not r:
            continue
        print(f"{r['run']:44} {r['target']:7} {'yes' if r['fi'] else 'no':3} {r['v0']:5.2f} | "
              f"{r['t03'][0]:5.2f} s {r['t03'][1]:5.2f} m | {r['t02'][0]:5.2f} s {r['t02'][1]:5.2f} m | "
              f"{r['peak']:5.2f} {r['climb']:5.2f} | {r['touch']:5.1f} {r['landed']:6.1f} {r['disarm']:6.1f} {r['flips']:5d}")


if __name__ == "__main__":
    main()
