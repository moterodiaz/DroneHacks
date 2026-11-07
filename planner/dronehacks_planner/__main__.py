"""Plan a route from a map file.

    python -m dronehacks_planner MAP.yaml [--out route.json] [--svg route.svg] [--prefer NAME ...]
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import sys

from .course_map import MapError, load_map
from .route import PlannerConfig, PlanningError, plan
from .svg import render_svg


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python -m dronehacks_planner", description=__doc__.splitlines()[0])
    ap.add_argument("map", help="course map file (contracts/course.md format)")
    ap.add_argument("--out", help="write the route as JSON here")
    ap.add_argument("--svg", help="write a picture of the map and route here")
    ap.add_argument("--prefer", nargs="*", default=[], metavar="NAME",
                    help="goal names to take when a group offers them (e.g. window)")
    defaults = PlannerConfig()
    for f in dataclasses.fields(PlannerConfig):
        if f.name == "prefer":
            continue
        ap.add_argument("--" + f.name.replace("_", "-"), type=type(getattr(defaults, f.name) or 0.0),
                        default=None, help=f"default {getattr(defaults, f.name)}")
    args = ap.parse_args(argv)

    overrides = {f.name: getattr(args, f.name) for f in dataclasses.fields(PlannerConfig)
                 if f.name != "prefer" and getattr(args, f.name, None) is not None}
    config = dataclasses.replace(defaults, prefer=tuple(args.prefer), **overrides)
    try:
        cmap = load_map(args.map)
        route = plan(cmap, config)
    except (MapError, PlanningError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    print(f"route: {route.length_m:.2f} m, {len(route.waypoints)} waypoints, goals {' > '.join(route.goals)}")
    if route.choices:
        print("chose: " + ", ".join(f"{k} = {v}" for k, v in route.choices.items()))
    for name, why in route.rejected.items():
        print(f"rejected {name}: {why}")
    for name, p in route.passes.items():
        print(f"pass {name}: heading ({p.direction[0]:+.3f}, {p.direction[1]:+.3f}), tilt {p.tilt_deg:+.0f} deg, "
              f"run-in {p.approach_m:.3f} m, min margin {p.min_margin_m:+.3f} m")
    print(f"nearest obstacle: {route.min_clearance_m:.3f} m at "
          f"({route.min_clearance_at[0]:+.3f}, {route.min_clearance_at[1]:+.3f}); "
          f"margin over inscribed radius {route.min_margin_m:+.3f} m")
    print(f"planned in {route.planning_time_s:.2f} s")
    if args.out:
        with open(args.out, "w", encoding="utf-8", newline="\n") as fh:
            json.dump(route.to_dict(), fh, indent=2)
            fh.write("\n")
    if args.svg:
        with open(args.svg, "w", encoding="utf-8", newline="\n") as fh:
            fh.write(render_svg(cmap, route))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
