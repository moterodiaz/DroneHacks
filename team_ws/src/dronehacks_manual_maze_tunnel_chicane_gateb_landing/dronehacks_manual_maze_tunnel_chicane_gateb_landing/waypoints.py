"""Map-driven waypoint derivation for the manual full-course mission.

Rule 9.5 (AGENTS.md §3.9): no course coordinate appears in this module or in
``mission.py``. Every waypoint is computed at run time from a course map in
the ``contracts/course.md`` format — a wall/boulder/goal slice of the course
at flight height (``contracts/course_v3_layout.yaml``, generated from the STEP
CAD by ``course/step_to_map.py``; an identical copy ships in this package's
``maps/`` for on-drone use, and ``course_map_yaml`` overrides it with the
surveyed map). The margins below are drone/safety tuning constants, not
course geometry.

Frames: waypoints are returned in COURSE frame ``(east, north)`` metres — the
map's frame, floor-centred origin. ``course_to_odom_xy`` applies the measured
course→odom transform of ``contracts/frames.md``; the mission never assumes an
identity rotation or that arming resets the vision origin.

Route structure (each stage derived from map features):

* **Maze** — the vertical interior walls ("dividers") spanning the open band
  between the long horizontal interior wall ("barrier") and the north net are
  passed in x-order, each at its free end (bottom-attached dividers are passed
  north of their top; top-hung dividers south of their bottom). Legs between
  passes run along the centres of the corridors between adjacent divider
  faces, matching the map's corridor widths.
* **Tunnel** — enter along the tunnel goal's axis: hold short of the
  entrance, fly the bore on the axis centre line, overshoot past the exit.
* **Chicane** — contracts/course.md fails the window closed
  (``clearance_radius: null`` until a reviewed budget exists), so the default
  route goes through Gate B: over the north end of the chicane line that
  contains the window wall, then south through the gap between that line and
  the vertical wall nearest the pad, and around that wall's free end into the
  landing area. A window route (approach/exit on the window goal's axis) is
  derived only when the map supplies a reviewed ``clearance_radius`` at least
  the turn margin, and refused otherwise — never substituted with a default
  (decisions #23 and #25 point 2).
* **Pad** — hold over the pad goal's centre; the pilot lands (contracts/px4.md
  blocks autonomous landing pending an organizer-supported procedure; decision
  #25 point 4 chose ``land: false``).

Safety model, all derived from contracts (not course coordinates):

* ``PROP_REACH_M`` — 0.28 m prop reach from centre (contracts/course.md).
* ``WALL_BUFFER_M`` — extra 0.02 m so checks are not knife-edge.
* Every planned leg must stay >= ``PROP_REACH_M`` from every wall; every
  waypoint where the heading changes must stay >= ``arrival_radius_m +
  PROP_REACH_M`` from every wall, because the drone can accept a waypoint up
  to ``arrival_radius_m`` early. ``_check_route`` enforces this exactly
  (segment-to-rectangle distances, no sampling) and raises ``RouteError`` —
  a route that fails is an error, never a warning.

Boulders are excluded from the aerial clearance check: they are 0.533 m tall
(contracts/course.md) and the route flies the 0.8 m slice, so they pass under
the airframe; they matter for landing, which this mission leaves to the pilot.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

try:
    import yaml
except ImportError as exc:  # pragma: no cover - environment without PyYAML
    raise ImportError(
        'PyYAML is required to load the course map (python3-yaml)') from exc

# ── Tuning constants (drone/safety margins, not course geometry) ────────────
PROP_REACH_M = 0.28       # prop reach from centre, contracts/course.md
WALL_BUFFER_M = 0.02      # extra margin so checks are not knife-edge
DEFAULT_ARRIVAL_RADIUS_M = 0.12  # <= course.md ±0.18 m error budget, minus tracking
NET_THICKNESS_M = 0.01    # segments thinner than this are the net, not walls
ATTACH_TOL_M = 0.03       # wall ends this close to another wall are "attached"
DEFAULT_WALL_THICKNESS_M = 0.102  # contracts/course.md geometry table default

REQUIRED_GOALS = ('tunnel', 'window', 'pad', 'launch')


class RouteError(RuntimeError):
    """The map cannot yield a safe route, or the inputs are unusable."""


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    label: str
    note: str = ''


@dataclass
class Route:
    waypoints: list                    # list[Waypoint], course frame
    via_window: bool
    refusal_reason: str = ''
    checks: dict = field(default_factory=dict)


# ── Map loading ──────────────────────────────────────────────────────────────

def _validate(data, source):
    if not isinstance(data, dict):
        raise RouteError(f'{source}: map is not a YAML mapping')
    for key in ('origin', 'slice_height', 'segments', 'goals'):
        if key not in data:
            raise RouteError(f'{source}: missing "{key}" (contracts/course.md schema)')
    goals = {}
    for goal in data.get('goals') or []:
        if not isinstance(goal, dict) or 'name' not in goal:
            raise RouteError(f'{source}: unnamed goal entry')
        goals[goal['name']] = goal
    for name in REQUIRED_GOALS:
        if name not in goals:
            # contracts/course.md: hard parse error, never substitute a default
            raise RouteError(f'{source}: missing required goal "{name}"')
    return data, goals


def load_course_map(path):
    """Parse and validate a contracts/course.md map. Fails closed."""
    with open(path, 'r', encoding='utf-8') as handle:
        data = yaml.safe_load(handle)
    return _validate(data, path)


def load_course_map_from_data(data):
    """Validate an already-parsed map (same rules as ``load_course_map``)."""
    return _validate(data, '<data>')


# ── Geometry helpers (exact, no sampling) ──────────────────────────────────

def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def _norm(a):
    return math.hypot(a[0], a[1])


def _unit(a):
    n = _norm(a)
    if n < 1e-9:
        raise RouteError('zero-length direction in wall geometry')
    return (a[0] / n, a[1] / n)


def _cross(u, v):
    return u[0] * v[1] - u[1] * v[0]


def _rect_of_segment(start, end, thickness):
    """Corners of a wall footprint: the segment swept by thickness/2.

    Corner order: (start+n·h, end+n·h, end−n·h, start−n·h) — edges (0,1) and
    (3,2) are the long edges, (1,2) and (3,0) the end caps.
    """
    dx, dy = _sub(end, start)
    length = math.hypot(dx, dy)
    if length < 1e-9:
        raise RouteError('zero-length wall segment')
    nx, ny = -dy / length, dx / length
    h = thickness / 2.0
    return (
        (start[0] + nx * h, start[1] + ny * h),
        (end[0] + nx * h, end[1] + ny * h),
        (end[0] - nx * h, end[1] - ny * h),
        (start[0] - nx * h, start[1] - ny * h),
    )


def _rect_bounds(rect):
    xs = [p[0] for p in rect]
    ys = [p[1] for p in rect]
    return min(xs), min(ys), max(xs), max(ys)


def _rect_direction(rect):
    """Unit direction of the segment the rect was swept from (long edge 0→1)."""
    return _unit(_sub(rect[1], rect[0]))


def _point_segment_distance(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    length2 = dx * dx + dy * dy
    if length2 < 1e-18:
        return math.hypot(p[0] - a[0], p[1] - a[1])
    t = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / length2))
    return math.hypot(p[0] - (a[0] + t * dx), p[1] - (a[1] + t * dy))


def _point_in_rect(p, rect):
    for i in range(len(rect)):
        ax, ay = rect[i]
        bx, by = rect[(i + 1) % len(rect)]
        if (bx - ax) * (p[1] - ay) - (by - ay) * (p[0] - ax) < 0.0:
            return False
    return True


def point_rect_distance(p, rect):
    if _point_in_rect(p, rect):
        return 0.0
    return min(
        _point_segment_distance(p, rect[i], rect[(i + 1) % len(rect)])
        for i in range(len(rect))
    )


def _segment_segment_distance(a, b, c, d):
    r = _sub(b, a)
    s = _sub(d, c)
    denom = _cross(r, s)
    if abs(denom) > 1e-12:
        t = _cross(_sub(c, a), s) / denom
        u = _cross(_sub(c, a), r) / denom
        if 0.0 <= t <= 1.0 and 0.0 <= u <= 1.0:
            return 0.0
    return min(_point_segment_distance(a, c, d),
               _point_segment_distance(b, c, d),
               _point_segment_distance(c, a, b),
               _point_segment_distance(d, a, b))


def segment_rect_distance(a, b, rect):
    if _point_in_rect(a, rect) or _point_in_rect(b, rect):
        return 0.0
    return min(
        _segment_segment_distance(a, b, rect[i], rect[(i + 1) % len(rect)])
        for i in range(len(rect))
    )


def rect_rect_distance(rect_a, rect_b):
    return min(
        segment_rect_distance(rect_a[i], rect_a[(i + 1) % len(rect_a)], rect_b)
        for i in range(len(rect_a))
    )


def _closest_points_segment_point(a, b, p):
    """Closest point on segment [a,b] to point p (for gate midpoint)."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    length2 = dx * dx + dy * dy
    if length2 < 1e-18:
        return a
    t = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / length2))
    return (a[0] + t * dx, a[1] + t * dy)


# ── Feature identification ──────────────────────────────────────────────────

def _wall_rects(data):
    """(walls, nets): footprints of interior walls and of the net."""
    walls, nets = [], []
    for seg in data['segments']:
        thickness = float(seg.get('thickness', DEFAULT_WALL_THICKNESS_M))
        rect = _rect_of_segment(tuple(seg['start']), tuple(seg['end']), thickness)
        (nets if thickness < NET_THICKNESS_M else walls).append(rect)
    if not nets:
        raise RouteError('map has no net segments')
    return walls, nets


def _merge_horizontal(wall_rects):
    """Merge collinear, contiguous horizontal wall pieces into full walls.

    Returns rows as ``(y_centre, x_start, x_end, thickness)``.
    """
    rows = []
    for rect in wall_rects:
        x0, y0, x1, y1 = _rect_bounds(rect)
        yc = (y0 + y1) / 2.0
        tol = (y1 - y0) / 2.0 + NET_THICKNESS_M
        for row in rows:
            if abs(yc - row[0]) <= tol and not (
                    x0 > row[2] + 1e-3 or x1 < row[1] - 1e-3):
                row[1] = min(row[1], x0)
                row[2] = max(row[2], x1)
                break
        else:
            rows.append([yc, x0, x1, y1 - y0])
    return rows


def _features(data, goals):
    walls, nets = _wall_rects(data)
    net_x0, net_y0, net_x1, net_y1 = (math.inf, math.inf, -math.inf, -math.inf)
    for rect in nets:
        a, b, c, d = _rect_bounds(rect)
        net_x0, net_y0 = min(net_x0, a), min(net_y0, b)
        net_x1, net_y1 = max(net_x1, c), max(net_y1, d)

    def is_vertical(rect):
        return abs(_rect_direction(rect)[0]) < 1e-6

    def is_horizontal(rect):
        return abs(_rect_direction(rect)[1]) < 1e-6

    verticals = [r for r in walls if is_vertical(r)]
    horizontals = [r for r in walls if is_horizontal(r)]
    diagonals = [r for r in walls if not is_vertical(r) and not is_horizontal(r)]

    launch = tuple(map(float, goals['launch']['centre']))

    # Barrier: the horizontal interior wall nearest the launch — the wall the
    # maze dividers stand on.
    barrier_rows = _merge_horizontal(horizontals)
    if not barrier_rows:
        raise RouteError('no horizontal interior wall (unsupported layout)')
    barrier = min(barrier_rows, key=lambda row: abs(row[0] - launch[1]))

    return {
        'walls': walls,
        'net': (net_x0, net_y0, net_x1, net_y1),
        'verticals': verticals,
        'diagonals': diagonals,
        'launch': launch,
        'barrier': barrier,  # (y_centre, x_start, x_end, thickness)
    }


def _merge_chicane_line(diagonal_rects, seed_rect):
    """All diagonal walls collinear with and contiguous to ``seed_rect``."""
    seed_dir = _rect_direction(seed_rect)
    group = [seed_rect]
    changed = True
    while changed:
        changed = False
        for rect in diagonal_rects:
            if any(rect is g for g in group):
                continue
            if abs(_cross(seed_dir, _rect_direction(rect))) > 0.02:
                continue  # not parallel
            if min(rect_rect_distance(rect, g) for g in group) <= 1e-3:
                group.append(rect)
                changed = True
    return group


def _line_face_x_at(rects, y, side):
    """x of a wall line's ``side`` face at height ``y`` (long-edge interpolation)."""
    xs = []
    for rect in rects:
        y_lo = min(p[1] for p in rect)
        y_hi = max(p[1] for p in rect)
        if not (y_lo - 1e-6 <= y <= y_hi + 1e-6):
            continue
        for a, b in ((rect[0], rect[1]), (rect[3], rect[2])):
            if abs(b[1] - a[1]) < 1e-9:
                xs.extend([a[0], b[0]])
                continue
            t = (y - a[1]) / (b[1] - a[1])
            xs.append(a[0] + (b[0] - a[0]) * t)
    if not xs:
        raise RouteError('chicane line does not span the required height')
    return max(xs) if side == 'east' else min(xs)


def _push_clear_of(point, rects, margin, step=0.005, limit=4.0):
    """Shift ``point`` along +x until it is ``margin`` from every rect.

    Face interpolation places a point a margin from a wall's face at one
    height, but the true distance is to the nearest corner/edge; this makes
    the clearance exact. Fails closed if no clear position exists.
    """
    x, y = point
    start = x
    while any(point_rect_distance((x, y), rect) < margin - 1e-9 for rect in rects):
        x += step
        if x - start > limit:
            raise RouteError(f'no clear position east of ({start:.3f}, {y:.3f}) '
                             f'within {limit:.1f} m')
    return x, y


def _gate_centre(rect_west, rects_east):
    """Midpoint of the closest pair between the two gate walls."""
    best_d, best_a, best_b = math.inf, None, None
    for other in rects_east:
        for i in range(4):
            a = rect_west[i]
            b = rect_west[(i + 1) % 4]
            for j in range(4):
                c = other[j]
                d = other[(j + 1) % 4]
                dist = _segment_segment_distance(a, b, c, d)
                if dist < best_d:
                    best_d = dist
                    if _point_segment_distance(a, c, d) <= _point_segment_distance(b, c, d):
                        best_a, best_b = _closest_points_segment_point(c, d, a), a
                    else:
                        best_a, best_b = _closest_points_segment_point(c, d, b), b
    if best_a is None:
        raise RouteError('no gate gap between the walls')
    return ((best_a[0] + best_b[0]) / 2.0, (best_a[1] + best_b[1]) / 2.0), best_d


# ── Route derivation ────────────────────────────────────────────────────────

def derive_route(data, goals, arrival_radius_m=DEFAULT_ARRIVAL_RADIUS_M):
    """Derive the full-course waypoint list (course frame). Fails closed."""
    turn_margin = arrival_radius_m + PROP_REACH_M + WALL_BUFFER_M
    pass_margin = PROP_REACH_M + WALL_BUFFER_M

    feats = _features(data, goals)
    net_x0, _, _, net_y1 = feats['net']
    launch = feats['launch']
    pad = tuple(map(float, goals['pad']['centre']))
    axis_a = tuple(map(float, goals['tunnel']['params']['axis_start']))
    axis_b = tuple(map(float, goals['tunnel']['params']['axis_end']))
    axis_dir = _unit(_sub(axis_b, axis_a))

    b_y, b_x0, b_x1, b_t = feats['barrier']
    barrier_y_max = b_y + b_t / 2.0
    barrier_y_min = b_y - b_t / 2.0

    # Maze dividers: vertical interior walls spanning the open band between
    # the barrier and the north net (the band the launch mark sits in).
    dividers = [
        rect for rect in feats['verticals']
        if _rect_bounds(rect)[3] > barrier_y_max + 1e-6
        and _rect_bounds(rect)[2] < net_y1 - 1e-6
    ]
    dividers.sort(key=lambda r: (_rect_bounds(r)[0] + _rect_bounds(r)[2]) / 2.0)
    if not dividers:
        raise RouteError('no maze dividers found (unsupported layout)')
    feats['dividers'] = dividers

    waypoints: list[Waypoint] = [
        Waypoint(launch[0], launch[1], 'launch',
                 'hover over the launch mark; climb before navigating'),
    ]

    prev_east_face = None  # corridor west boundary; None means the west net
    for index, rect in enumerate(dividers):
        dx0, dy0, dx1, dy1 = _rect_bounds(rect)
        west_face, east_face = dx0, dx1
        if west_face <= launch[0] + 1e-6 or west_face >= min(axis_a[0], axis_b[0]):
            raise RouteError('divider outside the launch→tunnel span '
                             '(unsupported layout)')
        if abs(dy0 - barrier_y_max) <= ATTACH_TOL_M:
            free_y, heading = dy1, +1.0   # bottom-attached: pass north of top
        elif abs(dy1 - net_y1) <= ATTACH_TOL_M:
            free_y, heading = dy0, -1.0   # top-hung: pass south of bottom
        else:
            raise RouteError('divider attached to neither barrier nor net '
                             '(unsupported layout)')
        y_pass = free_y + heading * turn_margin
        if heading > 0 and net_y1 - y_pass < PROP_REACH_M:
            raise RouteError(
                f'arrival_radius_m={arrival_radius_m:.3f} leaves under '
                f'{PROP_REACH_M:.2f} m of prop clearance below the north net '
                f'at divider {index}')
        if heading < 0 and y_pass - barrier_y_max < PROP_REACH_M:
            raise RouteError(
                f'arrival_radius_m={arrival_radius_m:.3f} leaves under '
                f'{PROP_REACH_M:.2f} m of prop clearance above the barrier '
                f'at divider {index}')

        west_bound = prev_east_face if prev_east_face is not None else net_x0
        corridor_west = (west_bound + west_face) / 2.0
        if index + 1 < len(dividers):
            corridor_east = (east_face + _rect_bounds(dividers[index + 1])[0]) / 2.0
        else:
            corridor_east = east_face + turn_margin  # exit past the last divider
        waypoints.append(Waypoint(corridor_west, y_pass,
                                  f'maze divider {index} pass',
                                  'pass the divider free end'))
        waypoints.append(Waypoint(corridor_east, y_pass,
                                  f'maze corridor {index} cross',
                                  'cross to the next corridor'))
        prev_east_face = east_face

    # ── Tunnel: along the goal axis; standoff before, overshoot after ──────
    back = (-axis_dir[0], -axis_dir[1])
    waypoints.append(Waypoint(axis_a[0] + back[0] * turn_margin,
                              axis_a[1] + back[1] * turn_margin,
                              'tunnel entry',
                              'turn onto the tunnel axis, short of the bore'))
    tunnel_exit = (axis_b[0] + axis_dir[0] * turn_margin,
                   axis_b[1] + axis_dir[1] * turn_margin)
    waypoints.append(Waypoint(*tunnel_exit, 'tunnel exit',
                              'through the bore on the axis centre, past the exit'))

    # ── Chicane: Gate B by default; window only with a reviewed budget ─────
    via_window, refusal_reason = False, ''
    clearance = goals['window']['params'].get('clearance_radius')
    if clearance is not None and float(clearance) >= turn_margin - 1e-9:
        _append_window_route(waypoints, goals, feats, turn_margin)
        via_window = True
    else:
        if clearance is None:
            refusal_reason = ('window clearance_radius is null (unreviewed) — '
                               'traversal refused per contracts/course.md; '
                               'routing via Gate B')
        else:
            refusal_reason = (
                f'window clearance_radius {float(clearance):.3f} m is below the '
                f'required {turn_margin:.3f} m — routing via Gate B')
        _append_gate_b_route(waypoints, data, goals, feats, tunnel_exit,
                             turn_margin)

    waypoints.append(Waypoint(pad[0], pad[1], 'pad',
                              'hold over the pad for the pilot to land'))

    _check_route(waypoints, data, goals, arrival_radius_m)
    return Route(waypoints=waypoints, via_window=via_window,
                 refusal_reason=refusal_reason,
                 checks={'turn_margin_m': turn_margin,
                         'pass_margin_m': pass_margin})


def _append_gate_b_route(waypoints, data, goals, feats, tunnel_exit,
                         turn_margin):
    """From the tunnel exit through the Gate B gap, to the landing area.

    Features, all identified from the map: the first chicane diagonal (the
    diagonal wall nearest the tunnel exit), the chicane line containing the
    window wall (its north end is the gate's east side), and the vertical wall
    nearest the pad that is not a maze divider (the gate's west side).
    """
    diagonals = feats['diagonals']
    if len(diagonals) < 2:
        raise RouteError('fewer than two diagonal walls (unsupported layout)')
    c1 = min(diagonals, key=lambda r: point_rect_distance(tunnel_exit, r))
    c1_corners = [p for p in c1]
    x_c1_east = max(p[0] for p in c1_corners)
    y_c1_south = min(p[1] for p in c1_corners)

    window_rect = _window_wall_rect(data, goals)
    c2_group = _merge_chicane_line(diagonals, window_rect)
    if any(rect is c1 for rect in c2_group):
        raise RouteError('window wall and first diagonal are the same wall '
                         '(unsupported layout)')
    c2_corners = [p for rect in c2_group for p in rect]

    pad = tuple(map(float, goals['pad']['centre']))
    others = [r for r in feats['verticals']
              if not any(r is d for d in feats['dividers'])]
    if not others:
        raise RouteError('no pad-side vertical wall (unsupported layout)')
    pad_wall = min(others, key=lambda r: point_rect_distance(pad, r))
    a_x0, a_y0, a_x1, a_y1 = _rect_bounds(pad_wall)
    a_centre_x = (a_x0 + a_x1) / 2.0

    n_corner = max(c2_corners, key=lambda p: (p[1], p[0]))  # north end, east-most
    gate, gate_width = _gate_centre(pad_wall, c2_group)
    if gate_width < 2 * PROP_REACH_M:
        raise RouteError(f'Gate B gap is {gate_width:.3f} m, under the '
                         f'{2 * PROP_REACH_M:.2f} m the drone needs')

    b_y_min = feats['barrier'][0] - feats['barrier'][3] / 2.0

    y_exit = tunnel_exit[1]
    y_c1_pass = y_c1_south - turn_margin
    face_east = _line_face_x_at(c2_group, y_c1_pass, 'east')
    # One shared x for the northbound corridor leg beside the chicane line:
    # clear of the line's face at the corner-turn height and of its north
    # end corner, so the whole vertical leg keeps the leg margin.
    x_corridor = max(_push_clear_of((face_east + turn_margin, y_c1_pass),
                                    c2_group + [c1], turn_margin)[0],
                     n_corner[0] + turn_margin)
    y_pocket = b_y_min - turn_margin

    waypoints.extend([
        Waypoint(x_c1_east + turn_margin, y_exit, 'chicane east of first diagonal',
                 'turn south, clear of the first diagonal'),
        Waypoint(x_c1_east + turn_margin, y_c1_pass,
                 'chicane south of first diagonal',
                 'pass the first diagonal free end'),
        Waypoint(x_corridor, y_c1_pass, 'chicane corner turn',
                 'turn north beside the chicane line'),
        Waypoint(x_corridor, y_pocket, 'gate pocket east',
                 'cross west over the chicane line north end, under the barrier'),
        Waypoint(n_corner[0] - turn_margin, y_pocket,
                 'gate pocket west', 'turn south into the Gate B gap'),
        Waypoint(gate[0], gate[1], 'Gate B centre',
                 'pass through the gap between the chicane line and the pad wall'),
        Waypoint(a_centre_x, a_y0 - turn_margin, 'south of pad wall end',
                 'cross past the free end of the pad-side wall'),
    ])


def _append_window_route(waypoints, goals, feats, turn_margin):
    window = goals['window']
    params = window['params']
    axis_a = tuple(map(float, params['axis_start']))
    axis_b = tuple(map(float, params['axis_end']))
    u = _unit(_sub(axis_b, axis_a))
    pad_wall_rect = _pad_wall_rect(goals, feats)
    a_x0, a_y0, a_x1, _ = _rect_bounds(pad_wall_rect)
    waypoints.append(Waypoint(axis_a[0] - u[0] * turn_margin,
                              axis_a[1] - u[1] * turn_margin,
                              'window approach',
                              'turn onto the window axis on the approach side'))
    waypoints.append(Waypoint(axis_b[0] + u[0] * turn_margin,
                              axis_b[1] + u[1] * turn_margin,
                              'window exit',
                              'straight through the opening on the axis'))
    waypoints.append(Waypoint((a_x0 + a_x1) / 2.0, a_y0 - turn_margin,
                              'south of pad wall end',
                              'cross past the free end of the pad-side wall'))


def _pad_wall_rect(goals, feats):
    pad = tuple(map(float, goals['pad']['centre']))
    dividers = feats['dividers']
    others = [r for r in feats['verticals']
              if not any(r is d for d in dividers)]
    if not others:
        raise RouteError('no pad-side vertical wall (unsupported layout)')
    return min(others, key=lambda r: point_rect_distance(pad, r))


def _window_wall_rect(data, goals):
    """Footprint of the window wall segment (the portal is handled in checks)."""
    index = int(goals['window']['params']['wall_segment_index'])
    if not 0 <= index < len(data['segments']):
        raise RouteError(f'window wall_segment_index {index} out of range')
    seg = data['segments'][index]
    return _rect_of_segment(
        tuple(seg['start']), tuple(seg['end']),
        float(seg.get('thickness', DEFAULT_WALL_THICKNESS_M)))


def _check_route(waypoints, data, goals, arrival_radius_m):
    """Exact clearance self-check. Raises RouteError on any violation."""
    obstacles = _obstacle_rects(data, goals)
    hard_leg = PROP_REACH_M
    hard_turn = arrival_radius_m + PROP_REACH_M

    def turn_at(i):
        if i <= 0 or i >= len(waypoints) - 1:
            return True  # first/last points are holds (turns)
        a, b, c = waypoints[i - 1], waypoints[i], waypoints[i + 1]
        u = _unit(_sub((b.x, b.y), (a.x, a.y)))
        v = _unit(_sub((c.x, c.y), (b.x, b.y)))
        return abs(_cross(u, v)) > 0.017  # heading change beyond ~1 degree

    for i, wp in enumerate(waypoints):
        need = hard_turn if turn_at(i) else hard_leg
        for rect in obstacles:
            dist = point_rect_distance((wp.x, wp.y), rect)
            if dist < need - 1e-9:
                raise RouteError(
                    f'waypoint {i} ({wp.label}) is {dist:.3f} m from a wall, '
                    f'needs {need:.3f} m (arrival radius {arrival_radius_m:.3f})')
    for i in range(len(waypoints) - 1):
        a = (waypoints[i].x, waypoints[i].y)
        b = (waypoints[i + 1].x, waypoints[i + 1].y)
        for rect in obstacles:
            dist = segment_rect_distance(a, b, rect)
            if dist < hard_leg - 1e-9:
                raise RouteError(
                    f'leg {i}→{i + 1} ({waypoints[i].label} → '
                    f'{waypoints[i + 1].label}) is {dist:.3f} m from a wall, '
                    f'needs {hard_leg:.3f} m')
    return True


def _obstacle_rects(data, goals):
    """All wall footprints; the window wall is split around its opening and
    the tunnel contributes its two side walls (the bore between them is free)."""
    rects = []
    window = goals['window']
    win_index = int(window['params']['wall_segment_index'])
    for i, seg in enumerate(data['segments']):
        if i == win_index:
            rects.extend(_split_window_wall(seg, window))
            continue
        rects.append(_rect_of_segment(
            tuple(seg['start']), tuple(seg['end']),
            float(seg.get('thickness', DEFAULT_WALL_THICKNESS_M))))
    rects.extend(_tunnel_side_walls(goals['tunnel']))
    return rects


def _split_window_wall(seg, window):
    """The window wall as two rects, excluding the opening span (a portal)."""
    start = tuple(map(float, seg['start']))
    end = tuple(map(float, seg['end']))
    thickness = float(seg.get('thickness', DEFAULT_WALL_THICKNESS_M))
    u = _unit(_sub(end, start))
    centre = tuple(map(float, window['centre']))
    side = float(window['params']['side'])
    length = _norm(_sub(end, start))
    t_c = (centre[0] - start[0]) * u[0] + (centre[1] - start[1]) * u[1]
    lo = max(0.0, t_c - side / 2.0)
    hi = min(length, t_c + side / 2.0)
    parts = []
    if lo > 1e-6:
        parts.append(_rect_of_segment(start, (start[0] + u[0] * lo,
                                               start[1] + u[1] * lo), thickness))
    if hi < length - 1e-6:
        parts.append(_rect_of_segment((start[0] + u[0] * hi,
                                        start[1] + u[1] * hi), end, thickness))
    return parts


def _tunnel_side_walls(tunnel):
    """The tube shell's side walls at the slice; the bore between them is free."""
    params = tunnel['params']
    a = tuple(map(float, params['axis_start']))
    b = tuple(map(float, params['axis_end']))
    width = float(params['width'])
    u = _unit(_sub(b, a))
    n = (-u[1], u[0])
    thickness = DEFAULT_WALL_THICKNESS_M  # conservative shell thickness
    out = []
    for sign in (+1.0, -1.0):
        offset = sign * (width / 2.0 - thickness / 2.0)
        s = (a[0] + n[0] * offset, a[1] + n[1] * offset)
        e = (b[0] + n[0] * offset, b[1] + n[1] * offset)
        out.append(_rect_of_segment(s, e, thickness))
    return out


# ── Course → odom transform (contracts/frames.md) ───────────────────────────

def course_to_odom_xy(point, o_course, o_odom, yaw_rad):
    """``p_odom = o_odom + R_odom_course @ (p_course - o_course)`` (xy only).

    ``yaw_rad`` is the measured course→odom rotation about +z (frames.md);
    the vertical reference is the operator's measured rest height, never
    assumed from the launch heading.
    """
    cos_y, sin_y = math.cos(yaw_rad), math.sin(yaw_rad)
    dx = point[0] - o_course[0]
    dy = point[1] - o_course[1]
    return (o_odom[0] + cos_y * dx - sin_y * dy,
            o_odom[1] + sin_y * dx + cos_y * dy)


# ── Pure tracking monitor (frozen/stale VIO must hold, not advance) ─────────

class TrackingMonitor:
    """Detects stale and frozen visual odometry.

    ``'stale'``  — no odometry message within ``stale_after_s``.
    ``'frozen'`` — position within ``frozen_eps_m`` across a window of at
    least ``frozen_window_s`` (cuVSLAM can freeze inside the dark tunnel).
    """

    def __init__(self, stale_after_s=0.5, frozen_window_s=1.0,
                 frozen_eps_m=1e-3, max_samples=400):
        self.stale_after_s = float(stale_after_s)
        self.frozen_window_s = float(frozen_window_s)
        self.frozen_eps_m = float(frozen_eps_m)
        self.max_samples = int(max_samples)
        self._samples: list = []
        self._last_t = None

    def update(self, now, position):
        self._last_t = now
        self._samples.append((now, *position))
        if len(self._samples) > self.max_samples:
            self._samples = self._samples[-self.max_samples // 2:]

    def status(self, now):
        if self._last_t is None or now - self._last_t > self.stale_after_s:
            return 'stale'
        window = [s for s in self._samples if now - s[0] <= self.frozen_window_s]
        if (window and now - window[0][0] >= self.frozen_window_s - 1e-9
                and len(window) >= 2):
            for axis in (1, 2, 3):
                values = [s[axis] for s in window]
                if max(values) - min(values) > self.frozen_eps_m:
                    return 'ok'
            return 'frozen'
        return 'ok'


# ── Pure mission engine (state machine; the reviewed fixes live here) ───────

class MissionEngine:
    """WAIT_FOR_OFFBOARD → CLIMB → NAVIGATE → HOLD_OVER_PAD, with HOLD_FAULT.

    Reviewed-finding fixes, all testable without ROS:

    * **Per-waypoint timeout** — exceeding it holds the last setpoint and
      stops autonomous advance; the old tunnel-only force-advance published
      stale targets forever once the pose froze, and pushed a healthy-but-slow
      drone out of the tunnel mid-flight.
    * **Tracking loss** (stale or frozen VIO) holds the last setpoint with a
      clear abort log instead of navigating on a frozen pose. The mission
      resumes only while the loss count stays under ``max_tracking_losses``.
    * **Waypoint timeout faults hold permanently** — the safety pilot takes
      over; re-entering offboard after a disarm/rearm restarts the mission.
    * **No landing descent** — heights are absolute in the odom frame, so a
      pilot who takes off in Position mode first cannot turn an
      odometry-at-Offboard-start "landing floor" airborne. The final state
      holds over the pad for the pilot (contracts/px4.md blocks autonomous
      landing; decision #25 point 4 set ``land: false``).
    """

    def __init__(self, waypoints_xy, hover_z, pad_xy, arrival_radius_m,
                 waypoint_timeout_s=45.0, max_tracking_losses=3):
        self.waypoints = [tuple(w) for w in waypoints_xy]
        self.hover_z = float(hover_z)
        self.pad_xy = tuple(pad_xy)
        self.arrival_radius_m = float(arrival_radius_m)
        self.waypoint_timeout_s = float(waypoint_timeout_s)
        self.max_tracking_losses = int(max_tracking_losses)
        self.state = 'WAIT_FOR_OFFBOARD'
        self.idx = 0
        self.deadline = None
        self.tracking_losses = 0
        self.fault_kind = ''
        self.last_target = None
        self.log_once: set = set()

    def tick(self, now, offboard_active, position, tracking_status):
        """Advance the state machine; returns (setpoint-or-None, events)."""
        events = []
        if position is None:
            self._once(events, 'no_odom', 'waiting for odometry ...')
            return None, events
        if not offboard_active and self.state != 'WAIT_FOR_OFFBOARD':
            self._once(events, 'offboard_lost',
                       'Offboard inactive — holding position.')
            return tuple(position), events

        if self.state == 'WAIT_FOR_OFFBOARD':
            if not offboard_active:
                self._once(events, 'waiting',
                           'waiting for safety pilot — arm and enter Position mode.')
                return tuple(position), events
            if tracking_status != 'ok':
                self._once(events, 'wait_tracking',
                           f'Offboard active but tracking is "{tracking_status}" '
                           '— holding position.')
                return tuple(position), events
            self.state = 'CLIMB'
            self.last_target = (self.waypoints[0][0], self.waypoints[0][1],
                                self.hover_z)
            events.append('Offboard active and tracking — climbing to '
                          f'{self.hover_z:.2f} m before navigating.')
            return self.last_target, events

        if self.state == 'CLIMB':
            target = (self.waypoints[0][0], self.waypoints[0][1], self.hover_z)
            self.last_target = target
            if tracking_status != 'ok':
                self._fault(events, f'tracking {tracking_status} during climb')
                return target, events
            if position[2] >= self.hover_z - 0.10:
                self.state = 'NAVIGATE'
                self.idx = 1
                self.deadline = now + self.waypoint_timeout_s
                events.append('Climb complete — entering NAVIGATE.')
            return target, events

        if self.state == 'NAVIGATE':
            if tracking_status != 'ok':
                self._fault(events, f'tracking {tracking_status}')
                return self.last_target, events
            if now > self.deadline:
                self._fault(events, f'waypoint timeout ({self.waypoint_timeout_s:.0f} s) '
                            f'at waypoint {self.idx}', kind='timeout')
                return self.last_target, events
            target = (self.waypoints[self.idx][0], self.waypoints[self.idx][1],
                      self.hover_z)
            self.last_target = target
            if math.hypot(position[0] - target[0],
                          position[1] - target[1]) <= self.arrival_radius_m:
                self.idx += 1
                if self.idx >= len(self.waypoints):
                    self.state = 'HOLD_OVER_PAD'
                    events.append('All waypoints reached — holding over the pad '
                                  'for the pilot to land.')
                else:
                    self.deadline = now + self.waypoint_timeout_s
                    events.append(f'reached waypoint {self.idx - 1}; '
                                  f'next is {self.idx}')
            return target, events

        if self.state == 'HOLD_FAULT':
            if tracking_status == 'ok' and self.fault_kind == 'tracking':
                if self.tracking_losses <= self.max_tracking_losses:
                    self.state = 'NAVIGATE'
                    self.deadline = now + self.waypoint_timeout_s
                    events.append('Tracking recovered — resuming NAVIGATE at '
                                  f'waypoint {self.idx}.')
                else:
                    self._once(events, 'abort_final',
                               'Too many tracking losses — pilot take over.')
            return self.last_target, events

        if self.state == 'HOLD_OVER_PAD':
            self.last_target = (self.pad_xy[0], self.pad_xy[1], self.hover_z)
            return self.last_target, events

        raise RuntimeError(f'unknown state {self.state}')

    def _fault(self, events, reason, kind='tracking'):
        if kind == 'tracking':
            self.tracking_losses += 1
        self.fault_kind = kind
        self.state = 'HOLD_FAULT'
        events.append(f'ABORT: {reason} — holding the last setpoint '
                      f'(loss #{self.tracking_losses}/{self.max_tracking_losses}); '
                      'safety pilot take over.')

    def _once(self, events, key, message):
        if key not in self.log_once:
            self.log_once.add(key)
            events.append(message)
