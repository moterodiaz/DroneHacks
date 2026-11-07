"""Plan a route through a course map's goals, in order.

The pipeline (planner/README.md has the reasoning):

1. Exact distance field and Nav2-style cost grid (``clearance``).
2. Each goal becomes fixed points. A pass-through goal (tunnel, window,
   gate) gives an *entry* and an *exit* on its axis, each far enough out
   that the whole drone is clear of the opening, joined by a straight pass.
   A disc (the pad) gives its centre. Both directions through an opening are
   options, and so is each goal in a group of alternatives.
3. A* from the current point to every option's first point. Options are
   compared on cost to enter, cost of the pass, and cost from their exit on
   to the next stage, and the cheapest is taken, unless
   ``PlannerConfig.prefer`` names one.
4. Each A* leg is shortcut: a straight line replaces a run of grid steps
   only when it is non-lethal, costs no more under the same cost model, and
   comes no closer to an obstacle than ``min(comfort, what the grid path did)``.
5. The finished polyline is re-checked with exact segment-to-obstacle
   distances (no sampling). A route that fails is an error, never a warning.

The output is in course-frame ``(east, north)`` metres at the map's
``slice_height``. Timing, yaw, corner arcs and the frames.md transform are
the Follower's job.
"""

from __future__ import annotations

import dataclasses
import math
import time
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

from .clearance import CostModel, ObstacleField, build_grid, segment_cost
from .course_map import CourseMap, Goal, Point
from .grid_search import SearchResult, search


class PlanningError(RuntimeError):
    """No acceptable route exists, or the inputs make one impossible to check."""


@dataclasses.dataclass(frozen=True)
class PlannerConfig:
    """Tunables. None of them describes a course; all of them describe the
    drone, the safety margins or the search."""

    #: Grid cell size. contracts/ros-graph.md: a 5 cm grid.
    resolution_m: float = 0.05
    #: Closest the drone's centre may come to a wall or the net.
    #: contracts/ros-graph.md: impassable within 0.32 m (0.28 m prop reach + 0.04 m).
    inscribed_radius_m: float = 0.32
    #: Same, for every boulder, overriding the rule below. None: use the rule.
    boulder_inscribed_radius_m: Optional[float] = None
    #: The rule. A boulder whose top sits at least ``boulder_overhead_m``
    #: below the drone's underside can pass under the props; what matters
    #: then is the downward rangefinder reading it as a floor step. So its
    #: radius is the rangefinder's footprint on the floor at flight height
    #: (``slice_height * tan(half FOV)``) plus ``rangefinder_margin_m``. A
    #: boulder of unknown or greater height gets ``inscribed_radius_m``.
    boulder_overhead_m: float = 0.15
    #: VL53L1X: 27 degree field of view (DRONEHACKS.md section 2).
    rangefinder_half_fov_deg: float = 13.5
    rangefinder_margin_m: float = 0.06
    #: Distance from an obstacle beyond which a cell costs nothing extra.
    inflation_radius_m: float = 1.0
    #: Nav2's cost_scaling_factor: how fast cost decays away from the inscribed radius.
    cost_scaling_per_m: float = 3.0
    #: A cell at the inscribed radius costs (1 + cost_weight) times a free one, per metre.
    cost_weight: float = 4.0
    #: Drone centre to prop tip (contracts/course.md derived error budget).
    drone_extent_m: float = 0.28
    #: How far past an opening's far face the whole drone must get before the
    #: route may turn: the checkpoint needs the whole drone across.
    crossing_margin_m: float = 0.10
    #: Straight, aligned run-in before any part of the drone enters an
    #: opening. Shortened (down to 0) where obstacles leave no room for it.
    approach_m: float = 0.30
    #: A ``gate`` (an unframed gap between wall ends) may be crossed up to
    #: this far off its nominal axis, if that gives more room. Windows and
    #: tunnels are always flown on their axis.
    gate_max_tilt_deg: float = 35.0
    gate_tilt_step_deg: float = 5.0
    #: Half the drone's height, for the window's vertical band.
    drone_half_height_m: float = 0.10
    #: Shortcuts may come this close to an obstacle even where the grid path
    #: stayed further away. Closer than this only where the grid path did too.
    comfort_clearance_m: float = 0.50
    #: Sample spacing when testing a shortcut.
    shortcut_step_m: float = 0.02
    #: How many failed shortcut candidates to scan past before committing.
    shortcut_lookahead: int = 8
    #: Sample spacing for the final whole-route check.
    verify_step_m: float = 0.01
    #: Goal names to take whenever a group of alternatives offers one.
    prefer: Tuple[str, ...] = ()
    #: Fly a window (a ``rect`` goal). Off by default: ``contracts/course.md``
    #: leaves the window's ``clearance_radius`` unreviewed, so the route goes
    #: via the reviewed alternative (e.g. Gate B) unless this is set. A stage
    #: whose only option is a window keeps it; there is no alternative.
    allow_windows: bool = False

    def cost_model(self) -> CostModel:
        return CostModel(
            inscribed_radius_m=self.inscribed_radius_m,
            inflation_radius_m=self.inflation_radius_m,
            cost_scaling_per_m=self.cost_scaling_per_m,
            cost_weight=self.cost_weight,
        )

    def boulder_radii(self, cmap: CourseMap) -> List[float]:
        """Inscribed radius of each of the map's boulders, in map order."""
        if self.boulder_inscribed_radius_m is not None:
            return [self.boulder_inscribed_radius_m] * len(cmap.boulders)
        low = (cmap.slice_height * math.tan(math.radians(self.rangefinder_half_fov_deg))
               + self.rangefinder_margin_m)
        underside = cmap.slice_height - self.drone_half_height_m
        return [low if b.height is not None and b.height + self.boulder_overhead_m <= underside
                else self.inscribed_radius_m
                for b in cmap.boulders]


@dataclasses.dataclass(frozen=True)
class Waypoint:
    position: Point
    #: start | via | entry | exit | goal
    kind: str
    #: Goal this point belongs to (entry, exit, goal), else None.
    goal: Optional[str]
    #: Distance to the nearest obstacle surface here.
    clearance_m: float
    #: The segment to the next waypoint is a straight pass through an opening:
    #: fly it straight, with the nose on the axis and no corner arc at either end.
    straight_to_next: bool = False


@dataclasses.dataclass(frozen=True)
class Pass:
    """How the route flies straight through one opening."""

    goal: str
    #: Unit direction of travel through the opening.
    direction: Point
    #: Angle off the goal's nominal axis (gates only; else 0).
    tilt_deg: float
    #: The aligned run-in actually used, after any shortening.
    approach_m: float
    #: Smallest margin over the inscribed radius along the straight pass.
    min_margin_m: float


@dataclasses.dataclass
class Route:
    waypoints: List[Waypoint]
    goals: List[str]
    choices: Dict[str, str]
    #: Alternatives that could not be flown, with the reason.
    rejected: Dict[str, str]
    passes: Dict[str, Pass]
    length_m: float
    cost: float
    min_clearance_m: float
    min_clearance_at: Point
    min_margin_m: float
    slice_height: float
    map_sha256: str
    config: PlannerConfig
    planning_time_s: float

    @property
    def points(self) -> np.ndarray:
        return np.array([w.position for w in self.waypoints], dtype=float)

    def to_dict(self) -> Dict[str, Any]:
        r = lambda v: round(float(v), 4)  # noqa: E731
        return {
            "format": "dronehacks-route/0",
            "frame": "course (east, north), metres; see contracts/frames.md for PX4",
            "height_m": r(self.slice_height),
            "map_sha256": self.map_sha256,
            "goals": list(self.goals),
            "choices": dict(self.choices),
            "rejected": dict(self.rejected),
            "passes": {
                name: {
                    "direction": [r(p.direction[0]), r(p.direction[1])],
                    "tilt_deg": r(p.tilt_deg),
                    "approach_m": r(p.approach_m),
                    "min_margin_m": r(p.min_margin_m),
                }
                for name, p in self.passes.items()
            },
            "length_m": r(self.length_m),
            "min_clearance_m": r(self.min_clearance_m),
            "min_clearance_at": [r(self.min_clearance_at[0]), r(self.min_clearance_at[1])],
            "min_margin_m": r(self.min_margin_m),
            "planning_time_s": round(self.planning_time_s, 3),
            "config": {k: (list(v) if isinstance(v, tuple) else v)
                       for k, v in dataclasses.asdict(self.config).items()},
            "waypoints": [
                {
                    "position": [r(w.position[0]), r(w.position[1])],
                    "kind": w.kind,
                    "goal": w.goal,
                    "clearance_m": r(w.clearance_m),
                    "straight_to_next": w.straight_to_next,
                }
                for w in self.waypoints
            ],
        }


@dataclasses.dataclass(frozen=True)
class _Option:
    goal: Goal
    points: Tuple[Point, ...]  # (entry, exit) for a pass, (centre,) for a disc
    pass_cost: float = 0.0
    info: Optional[Pass] = None


def plan(cmap: CourseMap, config: PlannerConfig = PlannerConfig(),
         start: Optional[Point] = None) -> Route:
    """Plan from ``start`` (default: the map's launch position) through every goal stage."""
    return _Planner(cmap, config).run(start)


def make_field(cmap: CourseMap, config: PlannerConfig = PlannerConfig()) -> ObstacleField:
    """The obstacle field the planner checks routes against, for later stages to share."""
    return ObstacleField(cmap, config.cost_model(), config.boulder_radii(cmap))


class _Planner:
    def __init__(self, cmap: CourseMap, config: PlannerConfig):
        t0 = time.perf_counter()
        self.cmap = cmap
        self.config = config
        self.model = config.cost_model()
        self.field = make_field(cmap, config)
        self.grid = build_grid(cmap, self.field, config.resolution_m)
        # A grid edge is up to res*sqrt(2) long, so a cell centre with margin
        # under half that could have a lethal point on the way to a neighbour.
        # Treat those cells as lethal for the search; the exact checks are
        # unaffected.
        pad = config.resolution_m * math.sqrt(2.0) / 2.0
        mult = np.where(self.grid.margin < pad, np.inf, self.grid.mult)
        self._mult: List[float] = mult.ravel().tolist()
        self._finite_cap = 1.0 + config.cost_weight
        self._cache: Dict[Tuple[int, Tuple[int, ...]], SearchResult] = {}
        self._t0 = t0

    # -- queries -------------------------------------------------------------

    def _margin(self, p: Point) -> float:
        return float(self.field.margin(np.asarray(p, dtype=float))[0])

    def _surface(self, p: Point) -> float:
        return float(self.field.query(np.asarray(p, dtype=float))[0][0])

    def _cell(self, p: Point, what: str) -> int:
        c = self.grid.cell(p)
        if c is None:
            raise PlanningError(f"{what} {_fmt(p)} is outside the map")
        return c

    def _search(self, src: Point, targets: Sequence[Point]) -> Tuple[SearchResult, List[int]]:
        src_cell = self._cell(src, "point")
        cells = [self._cell(t, "point") for t in targets]
        key = (src_cell, tuple(sorted(set(cells))))
        if key not in self._cache:
            mult = self._mult
            # The exact points were checked free; let the search start and end
            # in their cells even if the padded grid calls those cells lethal.
            fix = [c for c in [src_cell] + cells if mult[c] == math.inf]
            if fix:
                mult = list(mult)
                for c in fix:
                    mult[c] = self._finite_cap
            self._cache[key] = search(mult, self.grid.nx, self.grid.ny, self.grid.res, src_cell, cells)
        return self._cache[key], cells

    def _stage(self, stage: Tuple[Goal, ...]) -> Tuple[Goal, ...]:
        """Drop window (``rect``) goals unless the config allows them or
        explicitly prefers one by name. A stage with no other goal keeps
        its window; the caller falls back to it if nothing else flies."""
        cfg = self.config
        if cfg.allow_windows or any(g.name in cfg.prefer for g in stage):
            return stage
        rest = tuple(g for g in stage if g.shape != "rect")
        return rest or stage

    # -- goals ---------------------------------------------------------------

    def _options(self, stage: Tuple[Goal, ...]) -> Tuple[List[_Option], Dict[str, str]]:
        """Every flyable way to visit this stage, and why any goal in it can't be flown."""
        cfg = self.config
        names = [g.name for g in stage]
        preferred = [n for n in cfg.prefer if n in names]
        candidates = [g for g in stage if g.name == preferred[0]] if preferred else list(stage)
        options: List[_Option] = []
        rejected: Dict[str, str] = {}
        for goal in candidates:
            if goal.passes_through:
                found, reason = self._pass_options(goal)
            elif self._margin(goal.centre) < 0:
                found, reason = [], f"centre {_fmt(goal.centre)} is within the inscribed radius"
            else:
                found, reason = [_Option(goal, (goal.centre,))], ""
            options += found
            if not found:
                rejected[goal.name] = reason
        if not options:
            which = f"preferred goal {preferred[0]!r}" if preferred else f"goal(s) {', '.join(names)}"
            raise PlanningError(f"{which} cannot be flown: "
                                + "; ".join(f"{k}: {v}" for k, v in rejected.items()))
        return options, rejected

    def _pass_options(self, goal: Goal) -> Tuple[List[_Option], str]:
        """Straight passes through an opening: along ``axis``, and against it
        too if the goal is ``bidirectional``.

        The exit is far enough out that the whole drone is ``crossing_margin_m``
        past the far face. The entry adds an aligned run-in of up to
        ``approach_m``, halved until it fits. A gate also tries axes tilted up
        to ``gate_max_tilt_deg`` and keeps the one with the most room.
        """
        cfg = self.config
        if goal.shape == "rect" and "bottom" in goal.params:
            lo = goal.params["bottom"] + cfg.drone_half_height_m
            hi = goal.params["top"] - cfg.drone_half_height_m
            h = self.cmap.slice_height
            if not lo <= h <= hi:
                return [], f"flight height {h} m is outside its usable band {lo:.3f}-{hi:.3f} m"
        tilts = [0.0]
        if goal.shape == "gate" and cfg.gate_max_tilt_deg > 0 and cfg.gate_tilt_step_deg > 0:
            k = int(cfg.gate_max_tilt_deg / cfg.gate_tilt_step_deg + 1e-9)
            tilts = sorted((i * cfg.gate_tilt_step_deg for i in range(-k, k + 1)), key=abs)
        half = goal.depth / 2.0 + cfg.drone_extent_m
        cx, cy = goal.centre
        options: List[_Option] = []
        closest = -math.inf
        for sign in ((1.0, -1.0) if goal.bidirectional else (1.0,)):
            best: Optional[_Option] = None
            for tilt in tilts:
                t = math.radians(tilt)
                ax, ay = goal.axis
                dx = sign * (ax * math.cos(t) - ay * math.sin(t))
                dy = sign * (ax * math.sin(t) + ay * math.cos(t))
                cos_t = math.cos(t)
                out_r = (half + cfg.crossing_margin_m) / cos_t
                exit_ = (cx + dx * out_r, cy + dy * out_r)
                if self._margin(exit_) < 0:
                    closest = max(closest, self._margin(exit_))
                    continue
                approach = cfg.approach_m
                while True:
                    in_r = (half + approach) / cos_t
                    entry = (cx - dx * in_r, cy - dy * in_r)
                    cost, margin = segment_cost(self.field, entry, exit_, cfg.verify_step_m)
                    closest = max(closest, margin)
                    if math.isfinite(cost):
                        if best is None or margin > best.info.min_margin_m + 1e-9:
                            info = Pass(goal.name, (dx, dy), tilt, approach, margin)
                            best = _Option(goal, (entry, exit_), cost, info)
                        break
                    if approach <= 0.0:
                        break
                    approach = approach / 2.0 if approach > 0.02 else 0.0
            if best is not None:
                options.append(best)
        reason = "" if options else (f"no straight pass along its axis fits "
                                     f"(best margin {closest:+.3f} m over the inscribed radius)")
        return options, reason

    # -- legs ----------------------------------------------------------------

    def _shortcut(self, pts: List[Point]) -> List[Point]:
        n = len(pts)
        if n <= 2:
            return pts
        cfg = self.config
        step = cfg.shortcut_step_m
        seg = [segment_cost(self.field, pts[i], pts[i + 1], step) for i in range(n - 1)]
        prefix = [0.0]
        for c, _ in seg:
            prefix.append(prefix[-1] + c)
        seg_margin = [m for _, m in seg]
        comfort = cfg.comfort_clearance_m - cfg.inscribed_radius_m
        out = [pts[0]]
        i = 0
        while i < n - 1:
            best = i + 1
            misses = 0
            orig_min = seg_margin[i]
            j = i + 2
            while j < n and misses <= cfg.shortcut_lookahead:
                orig_min = min(orig_min, seg_margin[j - 1])
                cost, margin = segment_cost(self.field, pts[i], pts[j], step)
                orig_cost = prefix[j] - prefix[i]
                # The relative slack absorbs sampling differences, so collinear
                # runs always merge.
                if (math.isfinite(cost) and cost <= orig_cost * (1.0 + 1e-3) + 1e-6
                        and margin >= min(comfort, orig_min) - 1e-6):
                    best = j
                    misses = 0
                else:
                    misses += 1
                j += 1
            out.append(pts[best])
            i = best
        return out

    # -- main ----------------------------------------------------------------

    def run(self, start: Optional[Point]) -> Route:
        if start is None:
            if self.cmap.launch is None:
                raise PlanningError("no start: pass start=, or give the map a launch position")
            start = self.cmap.launch.position
        start = (float(start[0]), float(start[1]))
        if self._margin(start) < 0:
            raise PlanningError(f"start {_fmt(start)} is within the inscribed radius "
                                f"(margin {self._margin(start):+.3f} m)")

        stage_options: List[List[_Option]] = []
        rejected: Dict[str, str] = {}
        for stage in self.cmap.stages():
            use = self._stage(stage)
            try:
                options, why = self._options(use)
            except PlanningError:
                if use is stage:
                    raise
                # No reviewed alternative in this stage is flyable: fall back
                # to the windows rather than refusing to plan at all.
                options, why = self._options(stage)
            else:
                if use is not stage:
                    for g in stage:
                        if g.shape == "rect":
                            why[g.name] = why.get(g.name, "skipped: windows are opt-in "
                                                               "(PlannerConfig.allow_windows)")
            stage_options.append(options)
            rejected.update(why)

        cur = start
        waypoints = [Waypoint(start, "start", None, self._surface(start))]
        visited: List[str] = []
        choices: Dict[str, str] = {}
        passes: Dict[str, Pass] = {}
        for k, options in enumerate(stage_options):
            res, cells = self._search(cur, [o.points[0] for o in options])
            nxt = stage_options[k + 1] if k + 1 < len(stage_options) else None
            best: Optional[Tuple[float, int]] = None
            for idx, (opt, cell) in enumerate(zip(options, cells)):
                total = res.costs[cell] + opt.pass_cost
                if not math.isfinite(total):
                    continue
                if nxt is not None and len(options) > 1:
                    ahead, ahead_cells = self._search(opt.points[-1], [o.points[0] for o in nxt])
                    total += min(ahead.costs[c] for c in ahead_cells)
                if math.isfinite(total) and (best is None or total < best[0]):
                    best = (total, idx)
            if best is None:
                names = sorted({o.goal.name for o in options})
                raise PlanningError(f"cannot reach {' or '.join(names)} from {_fmt(cur)}")
            opt = options[best[1]]
            path = res.path(cells[best[1]])
            leg = [cur] + [self.grid.centre(c) for c in path[1:-1]] + [opt.points[0]]
            leg = self._shortcut(leg)
            for p in leg[1:-1]:
                waypoints.append(Waypoint(p, "via", None, self._surface(p)))
            goal = opt.goal
            if goal.passes_through:
                waypoints.append(Waypoint(opt.points[0], "entry", goal.name,
                                          self._surface(opt.points[0]), straight_to_next=True))
                waypoints.append(Waypoint(opt.points[1], "exit", goal.name, self._surface(opt.points[1])))
                passes[goal.name] = opt.info
            else:
                waypoints.append(Waypoint(opt.points[0], "goal", goal.name, self._surface(opt.points[0])))
            visited.append(goal.name)
            if goal.group is not None:
                choices[goal.group] = goal.name
            cur = opt.points[-1]

        return self._finish(waypoints, visited, choices, rejected, passes)

    def _finish(self, waypoints: List[Waypoint], visited: List[str], choices: Dict[str, str],
                rejected: Dict[str, str], passes: Dict[str, Pass]) -> Route:
        cfg = self.config
        pts = [w.position for w in waypoints]
        # Exact per-segment distances: nothing between samples can be missed.
        if len(pts) > 1:
            checks = [self.field.segment_query(pts[i], pts[i + 1]) for i in range(len(pts) - 1)]
        else:
            s, m = self.field.query(np.asarray(pts[0], dtype=float))
            checks = [(float(s[0]), float(m[0]), pts[0])]
        min_margin = min(c[1] for c in checks)
        if min_margin < 0:
            bad = min(checks, key=lambda c: c[1])
            raise PlanningError(f"route check failed near {_fmt(bad[2])}: within the "
                                f"inscribed radius (margin {min_margin:+.3f} m)")
        nearest = min(checks, key=lambda c: c[0])
        length = sum(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))
        cost = sum(segment_cost(self.field, pts[i], pts[i + 1], cfg.verify_step_m)[0]
                   for i in range(len(pts) - 1))
        return Route(
            waypoints=waypoints,
            goals=visited,
            choices=choices,
            rejected=rejected,
            passes=passes,
            length_m=length,
            cost=cost,
            min_clearance_m=float(nearest[0]),
            min_clearance_at=(float(nearest[2][0]), float(nearest[2][1])),
            min_margin_m=float(min_margin),
            slice_height=self.cmap.slice_height,
            map_sha256=self.cmap.sha256,
            config=cfg,
            planning_time_s=time.perf_counter() - self._t0,
        )


def _fmt(p: Point) -> str:
    return f"({p[0]:+.3f}, {p[1]:+.3f})"
