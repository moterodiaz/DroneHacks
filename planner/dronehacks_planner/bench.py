"""Benchmark: compare ways of flying the planned route, offline, in the
``sim.py`` stand-in for PX4. Every number here is a model output (see
``sim.py`` for what is guessed), not a flight measurement.

    python -m dronehacks_planner.bench MAP.yaml [--out runs/bench] [--speeds 1.0 2.0] [--no-plots]

Methods (each adds one thing to the one before):

* **A, waypoints**: the kit example's approach. The planner's polyline, one
  pose per waypoint, advance at 0.3 m.
* **B, smooth + one speed**: the smoothed path at a single speed (the lowest
  cap anywhere), lead poses.
* **C, smooth + speed profile**: per-point speed limits, forward-backward
  acceleration and jerk limits, lead poses. This is the Follower as built.
* **C0, C with plain poses**: no lead. Shows what the lead is worth.

Plus planned-only variants of C: with only acceleration limits (no jerk
limit), and with ``j_max`` = 8 (``MPC_JERK_MAX``).

With ``--lidar-follower DIR`` (the lidar stream's ``depthcam/`` directory,
``../DroneHacks-depthcam/depthcam``), its Follower (``follower/mission.py``) is
flown too, unmodified. It reads this route's JSON through its own loader
(``follower.route.route_from_dict``) and works in its tracker frame:
* **L**: its defaults (carrot 0.4 m ahead, clamped at each segment's end; waypoints reached at 0.15 m);
* **L, carrot 1.0 m**: the same with a longer carrot. That is our tuning, not the lidar stream's.

The results end with a verdict: of the followers that finish with no touch
and with every opening crossed, the fastest.

Writes ``results.json``, ``results.md`` and (with matplotlib) PNG figures
to ``--out``. ``runs/`` is git-ignored.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

from .course_map import load_map
from .follower import Follower, FollowerConfig, WaypointFollower
from .limits import VehicleLimits
from .pipeline import build, fastest
from .route import PlannerConfig, make_field
from .sim import SimConfig, flown_metrics, landed_on, openings_crossed, simulate
from .speed import ProfileConfig

# Reference palette (dataviz skill, references/palette.md), light mode, fixed slot order.
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4"]
INK, INK2, MUTED, GRID, AXIS, SURFACE = "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7", "#fcfcfb"


class LidarStreamFollower:
    """Adapter: the lidar stream's ``follower.mission.Follower``, unmodified, in ``sim.py``.

    The route goes in as JSON through its own ``route_from_dict``, into its
    tracker frame (x = north, y = west, origin at the launch mark;
    contracts/frames.md). The simulator's course-frame odometry is converted
    in, and its pose commands converted back out. It is done (for timing)
    when its state machine has reached HOLD over the pad within
    ``done_radius_m``: the same rule as the other followers. Its landing
    stage is not simulated.
    """

    def __init__(self, depthcam_dir: str, route, cmap, rest_height_m: float = 0.07,
                 done_radius_m: float = 0.10, **config):
        if depthcam_dir not in sys.path:
            sys.path.insert(0, depthcam_dir)
        from follower.mission import FOLLOW, HOLD, LAND, Follower as Mission, FollowerConfig as MissionConfig, Pose
        from follower.route import route_from_dict
        from pose_fusion.frames import LaunchMark

        self._Pose, self._states_done, self._follow = Pose, (HOLD, LAND), FOLLOW
        launch = cmap.launch.position if cmap.launch is not None else route.waypoints[0].position
        self.mark = LaunchMark(east=launch[0], north=launch[1])
        try:
            self.route = route_from_dict(route.to_dict(), self.mark)
        except ValueError as exc:
            raise RuntimeError(
                "the lidar stream's follower did not accept this planner's route ({}). This adapter "
                "matches its commit 6dc1263, which read dronehacks-route/0; later versions read their "
                "own route format.".format(exc)) from exc
        self.mission = Mission(self.route, MissionConfig(rest_height_m=rest_height_m, land=False, **config))
        self.end = np.asarray(route.waypoints[-1].position, dtype=float)
        self.done_radius = done_radius_m
        self._last = np.asarray(route.waypoints[0].position, dtype=float)
        # Check the frame conversion both ways on the route's own points
        # (to within the route JSON's 0.1 mm rounding).
        for w, p in zip(route.waypoints, self.route.points):
            back = self.to_course(p.x, p.y)
            if np.hypot(*(back - np.asarray(w.position))) > 1e-4:
                raise ValueError("tracker <-> course conversion does not round-trip")

    def to_tracker(self, e: float, n: float):
        return n - self.mark.north, -(e - self.mark.east)

    def to_course(self, x: float, y: float) -> np.ndarray:
        return np.array([self.mark.east - y, self.mark.north + x])

    def update(self, now: float, est_pos, est_vel=None):
        from .follower import Setpoint

        x, y = self.to_tracker(float(est_pos[0]), float(est_pos[1]))
        pose = self._Pose(t=now, x=x, y=y, z=self.mission.z_fly, yaw=0.0)
        cmd = self.mission.update(now, True, pose, landed=False)
        if cmd is not None and cmd.kind == "pose":
            self._last = self.to_course(cmd.x, cmd.y)
        done = (self.mission.state in self._states_done
                and float(np.hypot(*(np.asarray(est_pos, dtype=float) - self.end))) <= self.done_radius)
        return Setpoint(position=self._last.copy(), yaw=0.0, reference=self._last.copy(), ref_time=now,
                        lead_m=0.0, lead_scale=1.0, lag_m=0.0, cross_track_m=0.0, clock_rate=1.0, done=done)


def _densify(points, step: float = 0.01) -> np.ndarray:
    pts = [np.asarray(p, dtype=float) for p in points]
    out = [pts[0][None, :]]
    for a, b in zip(pts, pts[1:]):
        n = max(1, int(np.ceil(np.hypot(*(b - a)) / step)))
        t = np.linspace(0, 1, n + 1)[1:, None]
        out.append(a * (1 - t) + b * t)
    return np.vstack(out)


def run(map_path: str, speeds: List[float], out: Path, plots: bool = True,
        lidar_follower: Optional[str] = None) -> Dict:
    cmap = load_map(map_path)
    planner = PlannerConfig()
    field = make_field(cmap, planner)
    extent = planner.drone_extent_m
    pad_name = next((g.name for g in cmap.goals if g.shape == "disc"), None)
    out.mkdir(parents=True, exist_ok=True)
    results: Dict = {"map": map_path, "map_sha256": cmap.sha256, "sim": dataclasses.asdict(SimConfig()),
                     "settings": []}
    for v_max in speeds:
        limits = dataclasses.replace(VehicleLimits(), v_max_mps=v_max)
        t0 = time.perf_counter()
        best, alternatives = fastest(cmap, planner, limits)
        choose_s = time.perf_counter() - t0
        route = best.route
        const = build(cmap, planner, limits, profile=ProfileConfig(constant_speed=True), route=route)
        variants = {
            "C, acceleration limits only (no jerk limit)": build(
                cmap, planner, limits, profile=ProfileConfig(longitudinal_jerk_smoothing=False,
                                                              lateral_jerk_limit=False), route=route),
            "C, j_max 8 (MPC_JERK_MAX)": build(
                cmap, planner, dataclasses.replace(limits, j_max_mps3=8.0), route=route),
        }
        start = route.waypoints[0].position
        runs = {
            "A waypoints": (WaypointFollower([w.position for w in route.waypoints]), None),
            "B smooth + one speed": (Follower(const.trajectory, field, limits), const),
            "C smooth + speed profile": (Follower(best.trajectory, field, limits), best),
            "C0 C with plain poses": (Follower(best.trajectory, field, limits, FollowerConfig(lead=False)), best),
        }
        if lidar_follower:
            runs["L lidar-stream follower (its defaults)"] = (LidarStreamFollower(lidar_follower, route, cmap), None)
            runs["L lidar-stream follower, carrot 1.0 m (our tuning)"] = (
                LidarStreamFollower(lidar_follower, route, cmap, lookahead_m=1.0), None)
        polyline = _densify([w.position for w in route.waypoints])
        rows, flights, refs = [], {}, {}
        for name, (follower, fp) in runs.items():
            refs[name] = best.path.xy if fp is not None else polyline
            res = simulate(follower, start, limits)
            # Off-path error against the path each follower is meant to fly.
            m = flown_metrics(res, field, extent, best.path.xy if fp is not None else polyline)
            crossed = openings_crossed(res.pos, route, cmap, extent)
            row = {
                "method": name,
                "planned_s": fp.duration if fp else None,
                "flown_s": m["time_s"] if m["finished"] else None,
                "finished": m["finished"],
                "planned_length_m": fp.path.length_m if fp else route.length_m,
                "flown_length_m": m["flown_length_m"],
                "max_speed_mps": m["max_speed_mps"],
                "mean_speed_mps": m["mean_speed_mps"],
                "max_accel_flown_mps2": m["max_accel_mps2"],
                "ref_max_jerk_long_mps3": fp.report["max_jerk_long_mps3"] if fp else None,
                "ref_max_jerk_lat_mps3": fp.report["max_jerk_lat_mps3"] if fp else None,
                "ref_max_curvature": fp.report["max_curvature"] if fp else None,
                "planned_min_clearance_m": fp.path.min_clearance_m if fp else route.min_clearance_m,
                "flown_min_wall_clearance_m": m["min_wall_clearance_m"],
                "wall_touches": m["wall_touches"],
                "openings_ok": all(crossed.values()),
                "landed_on_pad": landed_on(res.pos[-1], cmap, pad_name, extent) if pad_name else None,
                "max_cross_track_m": m["max_cross_track_m"],
                "reference_held_fraction": float(np.mean(res.clock_rate < 1.0)),
                "lead_clipped_fraction": float(np.mean(res.lead_scale < 1.0)),
                "violations": list(fp.report["violations"]) if fp else [],
                "compute_s": fp.compute_s if fp else None,
            }
            rows.append(row)
            flights[name] = res
        setting_verdict = _verdict(rows)
        planned_only = [{"method": n, "planned_s": fp.duration, "violations": fp.report["violations"],
                         "ref_max_jerk_long_mps3": fp.report["max_jerk_long_mps3"],
                         "ref_max_jerk_lat_mps3": fp.report["max_jerk_lat_mps3"]} for n, fp in variants.items()]
        setting = {
            "v_max_mps": v_max,
            "limits": dataclasses.asdict(limits),
            "route": route.goals,
            "alternatives": [{"choices": c, "outcome": o} for c, o in alternatives],
            "choose_fastest_s": choose_s,
            "rows": rows,
            "planned_only": planned_only,
            "verdict": setting_verdict,
        }
        results["settings"].append(setting)
        if plots:
            _plots(out, cmap, field, best, flights, setting, planner, refs)
    (out / "results.json").write_text(json.dumps(results, indent=2, default=_json), encoding="utf-8")
    (out / "results.md").write_text(_markdown(results), encoding="utf-8")
    return results


def _safe(r: Dict) -> bool:
    return bool(r["finished"] and r["wall_touches"] == 0 and r["openings_ok"] and r["landed_on_pad"] is not False)


def _verdict(rows: List[Dict]) -> Dict:
    """The fastest follower that finished with no touch, every opening crossed and on the pad;
    and, if the lidar stream's follower ran, the head-to-head against ours (C)."""
    safe = [r for r in rows if _safe(r)]
    out: Dict = {"fastest_safe": min(safe, key=lambda r: r["flown_s"])["method"] if safe else None,
                 "unsafe_or_unfinished": [r["method"] for r in rows if not _safe(r)]}
    ours = next((r for r in rows if r["method"].startswith("C smooth")), None)
    theirs = [r for r in rows if r["method"].startswith("L ")]
    if ours is not None and theirs:
        contenders = [r for r in [ours] + theirs if _safe(r)]
        better = min(contenders, key=lambda r: r["flown_s"]) if contenders else None
        out["head_to_head"] = {
            "better": better["method"] if better else None,
            "times_s": {r["method"]: r["flown_s"] for r in [ours] + theirs},
            "closest_wall_m": {r["method"]: r["flown_min_wall_clearance_m"] for r in [ours] + theirs},
        }
    return out


def _json(o):
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, tuple):
        return list(o)
    raise TypeError(type(o))


def _fmt(v, nd=2):
    if v is None:
        return "-"
    if isinstance(v, bool):
        return "yes" if v else "NO"
    return f"{v:.{nd}f}" if isinstance(v, float) else str(v)


def _markdown(results: Dict) -> str:
    lines = ["# Route-following benchmark (simulated)", "",
             f"Map `{results['map']}` (sha256 `{results['map_sha256'][:12]}`). Model outputs from "
             "`dronehacks_planner/sim.py`, not flight data. Its acceleration lag is a guess.", ""]
    for st in results["settings"]:
        lines += [f"## v_max = {st['v_max_mps']} m/s", "",
                  f"Route: {' > '.join(st['route'])}. Alternatives by planned flight time: "
                  + "; ".join(f"{a['choices']}: {a['outcome']}" for a in st["alternatives"])
                  + f" (chosen in {st['choose_fastest_s']:.2f} s).", "",
                  "| method | planned s | flown s | touches | flown min wall clearance m | openings | pad "
                  "| max cross-track m | max speed m/s | mean speed m/s | ref jerk along / lateral m/s^3 "
                  "| reference held | lead clipped | violations |",
                  "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
        for r in st["rows"]:
            jerk = "-" if r["ref_max_jerk_long_mps3"] is None else \
                f"{r['ref_max_jerk_long_mps3']:.1f} / {r['ref_max_jerk_lat_mps3']:.1f}"
            lines.append(
                f"| {r['method']} | {_fmt(r['planned_s'])} | {_fmt(r['flown_s'])} | {r['wall_touches']} "
                f"| {_fmt(r['flown_min_wall_clearance_m'], 3)} | {_fmt(r['openings_ok'])} | {_fmt(r['landed_on_pad'])} "
                f"| {_fmt(r['max_cross_track_m'], 3)} | {_fmt(r['max_speed_mps'])} | {_fmt(r['mean_speed_mps'])} "
                f"| {jerk} | {r['reference_held_fraction']:.0%} | {r['lead_clipped_fraction']:.0%} "
                f"| {'; '.join(r['violations']) or 'none'} |")
        v = st["verdict"]
        lines += ["", f"**Fastest safe follower: {v['fastest_safe']}** (finished, no wall touch, every opening "
                  "crossed whole-drone, on the pad)."]
        if "head_to_head" in v:
            h = v["head_to_head"]
            lines.append(f"**Flagged as better, ours vs the lidar stream's: {h['better']}.** "
                         + "; ".join(f"{m}: {_fmt(t)} s, closest wall {_fmt(h['closest_wall_m'][m], 3)} m"
                                     for m, t in h["times_s"].items()) + ".")
        lines += ["", "Planned-only variants of C:", "",
                  "| variant | planned s | jerk along / lateral m/s^3 | violations |", "|---|---|---|---|"]
        for p in st["planned_only"]:
            lines.append(f"| {p['method']} | {p['planned_s']:.2f} | {p['ref_max_jerk_long_mps3']:.1f} / "
                         f"{p['ref_max_jerk_lat_mps3']:.1f} | {'; '.join(p['violations']) or 'none'} |")
        lines.append("")
    lines += ["A wall touch means the prop tips (0.28 m from centre) came inside a wall or the net. "
              "\"Openings\": the drone went through each opening inside its width less the drone's half-span, "
              "and its whole body got past the far face.", ""]
    return "\n".join(lines)


def _plots(out: Path, cmap, field, best, flights, setting, planner, refs) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed: skipping plots", file=sys.stderr)
        return
    plt.rcParams.update({"font.family": "sans-serif", "font.size": 9, "axes.edgecolor": AXIS,
                         "axes.labelcolor": INK2, "xtick.color": MUTED, "ytick.color": MUTED,
                         "axes.titlecolor": INK, "figure.facecolor": SURFACE, "axes.facecolor": SURFACE})
    tag = f"vmax{setting['v_max_mps']:g}"

    # 1. Small multiples: planned path vs each simulated flight.
    names = list(flights)
    ncols = min(len(names), 3 if len(names) > 4 else len(names))
    nrows = int(np.ceil(len(names) / ncols))
    fig, axes = plt.subplots(nrows, ncols, figsize=(4.4 * ncols, 4.7 * nrows), squeeze=False)
    for ax in axes.ravel()[len(names):]:
        ax.set_visible(False)
    for ax, name in zip(axes.ravel(), names):
        _draw_map(ax, cmap, planner)
        ref = refs[name]
        ax.plot(ref[:, 0], ref[:, 1], color=MUTED, lw=1, ls="--",
                label="path this follower flies (smoothed path, or the planner's polyline)")
        res = flights[name]
        ax.plot(res.pos[:, 0], res.pos[:, 1], color=SERIES[0], lw=2, label="simulated flight")
        row = next(r for r in setting["rows"] if r["method"] == name)
        flown = "did not finish" if row["flown_s"] is None else f"{row['flown_s']:.1f} s"
        ax.set_title(f"{name}\n{flown}, {row['wall_touches']} touches, closest wall "
                     f"{row['flown_min_wall_clearance_m']:.2f} m", fontsize=9, loc="left")
    handles, labels = axes.ravel()[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper right", fontsize=8, frameon=False, ncol=2)
    fig.suptitle(f"Planned path and simulated flights, v_max {setting['v_max_mps']:g} m/s (model, not flight data)",
                 x=0.01, ha="left", color=INK)
    fig.tight_layout(rect=(0, 0, 1, 1 - 0.08 / nrows))
    fig.savefig(out / f"flights_{tag}.png", dpi=130)
    plt.close(fig)

    # 2. Speed profile with its caps; curvature and acceleration as separate panels (no dual axes).
    prof, path = best.profile, best.path
    fig, (a1, a2, a3) = plt.subplots(3, 1, figsize=(10, 7.2), sharex=True,
                                     gridspec_kw={"height_ratios": [2.2, 1, 1]})
    cap_names = [k for k in ("speed", "lateral", "lateral_jerk", "braking", "opening") if k in prof.caps]
    for i, k in enumerate(cap_names):
        c = np.where(np.isfinite(prof.caps[k]), prof.caps[k], np.nan)
        a1.plot(prof.s, c, color=SERIES[(i + 1) % len(SERIES)], lw=1, ls=":", label=f"cap: {k.replace('_', ' ')}")
    a1.plot(prof.s, prof.v, color=SERIES[0], lw=2, label="planned speed")
    a1.set_ylim(0, setting["v_max_mps"] * 1.25)
    a1.set_ylabel("speed, m/s")
    a1.legend(ncol=6, fontsize=8, frameon=False, loc="lower left", bbox_to_anchor=(0, 1.08))
    a1.set_title(f"Speed along the route, v_max {setting['v_max_mps']:g} m/s: planned {best.duration:.1f} s",
                 loc="left")
    a2.plot(path.s, np.abs(path.curvature), color=SERIES[0], lw=1.5)
    a2.set_ylabel("|curvature|, 1/m")
    a3.plot(prof.s, prof.a_long, color=SERIES[0], lw=1.5, label="along the path")
    a3.plot(prof.s, prof.a_lat, color=SERIES[1], lw=1.5, label="lateral")
    a3.set_ylabel("acceleration, m/s^2")
    a3.set_xlabel("distance along the route, m")
    a3.legend(fontsize=8, frameon=False, loc="upper right")
    for ax in (a1, a2, a3):
        ax.grid(True, color=GRID, lw=0.6)
        ax.spines[["top", "right"]].set_visible(False)
        for name, s0, s1 in _opening_spans(path):
            ax.axvspan(s0, s1, color=GRID, alpha=0.6, lw=0)
            if ax is a1:
                ax.text((s0 + s1) / 2, 0.04 * setting["v_max_mps"], name, ha="center", fontsize=8, color=INK2)
    fig.tight_layout()
    fig.savefig(out / f"speed_{tag}.png", dpi=130)
    plt.close(fig)

    # 3. Planned vs simulated time per method.
    rows = setting["rows"]
    fig, ax = plt.subplots(figsize=(8.5, 1.2 + 0.55 * len(rows)))
    y = np.arange(len(rows))
    planned = [r["planned_s"] or 0 for r in rows]
    flown = [r["flown_s"] or 0 for r in rows]
    ax.barh(y - 0.19, planned, height=0.36, color=SERIES[0], label="planned")
    ax.barh(y + 0.19, flown, height=0.36, color=SERIES[1], label="simulated")
    for yi, p, f in zip(y, planned, flown):
        if p:
            ax.text(p + 0.4, yi - 0.19, f"{p:.1f}", va="center", fontsize=8, color=INK2)
        ax.text(f + 0.4, yi + 0.19, f"{f:.1f}" if f else "did not finish", va="center", fontsize=8, color=INK2)
    ax.set_yticks(y, [r["method"] for r in rows])
    ax.invert_yaxis()
    ax.set_xlabel("seconds, launch to pad")
    ax.set_title(f"Flight time by method, v_max {setting['v_max_mps']:g} m/s (simulated)", loc="left")
    ax.legend(frameon=False, fontsize=8, loc="lower right")
    ax.set_axisbelow(True)
    ax.grid(True, axis="x", color=GRID, lw=0.6)
    ax.spines[["top", "right"]].set_visible(False)
    fig.tight_layout()
    fig.savefig(out / f"times_{tag}.png", dpi=130)
    plt.close(fig)


def _draw_map(ax, cmap, planner) -> None:
    from matplotlib.patches import Circle

    for s in cmap.segments:
        ax.plot([s.start[0], s.end[0]], [s.start[1], s.end[1]], color=INK, lw=1.2, solid_capstyle="round")
    for b in cmap.boulders:
        ax.add_patch(Circle(b.centre, b.radius, color=MUTED, lw=0))
    for g in cmap.goals:
        if g.shape == "disc":
            ax.add_patch(Circle(g.centre, g.params["radius"], fill=False, color=MUTED, lw=1.0, ls="--"))
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    for side in ax.spines.values():
        side.set_visible(False)


def _opening_spans(path):
    spans, cur, start = [], "", 0.0
    for s, name in zip(path.s, path.pass_name + [""]):
        if name != cur:
            if cur:
                spans.append((cur, start, s))
            cur, start = name, s
    if cur:
        spans.append((cur, start, float(path.s[-1])))
    return spans


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python -m dronehacks_planner.bench", description=__doc__.splitlines()[0])
    ap.add_argument("map")
    ap.add_argument("--out", default="runs/bench")
    ap.add_argument("--speeds", nargs="+", type=float, default=[1.0, 2.0],
                    help="v_max settings: 1.0 = SN0001 params; anything higher is unvalidated")
    ap.add_argument("--no-plots", action="store_true")
    ap.add_argument("--lidar-follower", metavar="DEPTHCAM_DIR",
                    help="also fly the lidar stream's follower from this directory "
                         "(e.g. ../DroneHacks-depthcam/depthcam)")
    args = ap.parse_args(argv)
    t0 = time.perf_counter()
    lidar = str(Path(args.lidar_follower).resolve()) if args.lidar_follower else None
    run(args.map, args.speeds, Path(args.out), plots=not args.no_plots, lidar_follower=lidar)
    print(f"wrote {args.out}/results.md in {time.perf_counter() - t0:.1f} s")
    print(Path(args.out, "results.md").read_text(encoding="utf-8"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
