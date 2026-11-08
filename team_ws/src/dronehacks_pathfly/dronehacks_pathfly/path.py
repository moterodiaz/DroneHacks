"""The planned path: loading, arena -> odom transform, speed profile.

No ROS and no NumPy here, so it runs anywhere (the drone container, a laptop,
the unit tests).

Arena frame (the path-planning repo's output frame): top-down metres, origin at
the inside corner of the arena, x = CAD X, y = -CAD Z. It is right-handed with
z up, so it maps to the ENU ``odom`` frame by a rotation and a translation only.
"""
import csv
import math
from dataclasses import dataclass


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def load_path_csv(filename):
    """Read ``x_m, y_m[, margin_m]`` rows (path-planning's CSV format).

    Returns (points, margins); margins is None when the column is absent.
    """
    points, margins = [], []
    with open(filename, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            x, y = float(row['x_m']), float(row['y_m'])
            if not (math.isfinite(x) and math.isfinite(y)):
                raise ValueError(f'{filename}: non-finite point {row}')
            points.append((x, y))
            m = row.get('margin_m')
            margins.append(float(m) if m not in (None, '') else None)
    if len(points) < 2:
        raise ValueError(f'{filename}: need at least 2 points, got {len(points)}')
    if any(m is None for m in margins):
        margins = None
    return points, margins


def truncate_path(points, margins, limit):
    """The first ``limit`` metres of a polyline (and its margins). A limit
    that is not finite, <= 0, or past the end returns the path unchanged."""
    if not math.isfinite(limit) or limit <= 0.0:
        return list(points), margins
    out, out_m, run = [points[0]], [margins[0]] if margins else None, 0.0
    for i in range(1, len(points)):
        seg = math.dist(points[i - 1], points[i])
        if run + seg >= limit:
            f = (limit - run) / seg if seg > 0 else 0.0
            a, b = points[i - 1], points[i]
            out.append((a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1])))
            if margins:
                out_m.append(margins[i - 1] + f * (margins[i] - margins[i - 1]))
            return out, out_m
        run += seg
        out.append(points[i])
        if margins:
            out_m.append(margins[i])
    return list(points), margins


def rotate_xy(vx, vy, angle):
    """Rotate a horizontal vector counter-clockwise by ``angle`` (rad)."""
    c, s = math.cos(angle), math.sin(angle)
    return vx * c - vy * s, vx * s + vy * c


def twist_for_interface(twist, odom_to_ned_yaw_offset_rad):
    """Pre-rotate an ENU velocity so it survives a vision-vs-EKF2 heading
    offset.

    skybeta_flight_interface turns /skybeta/setpoint/twist into PX4's NED with
    the plain REP-103 swap; unlike pose setpoints, it does not apply its
    ``odom_to_ned_yaw_offset_rad`` to velocities. That parameter rotates NED
    vectors by +offset about Down (North towards East), i.e. -offset about
    ENU Up, so we apply the same rotation here, using the same measured
    value. The yaw rate is a rate about the vertical axis and is unchanged.
    """
    if not odom_to_ned_yaw_offset_rad:
        return twist
    vx, vy = rotate_xy(twist[0], twist[1], -odom_to_ned_yaw_offset_rad)
    return (vx, vy, twist[2], twist[3])


@dataclass(frozen=True)
class ArenaToOdom:
    """Rigid 2-D transform from the arena frame into the vision ``odom`` frame.

    Built from one matched pose: the vehicle sits at ``arena_start`` with its
    nose at ``arena_yaw`` (arena frame), and odometry reads ``odom_start`` /
    ``odom_yaw`` at that same instant.
    """

    arena_start: tuple
    arena_yaw: float
    odom_start: tuple
    odom_yaw: float

    @property
    def rotation(self):
        return wrap_pi(self.odom_yaw - self.arena_yaw)

    def point(self, p):
        c, s = math.cos(self.rotation), math.sin(self.rotation)
        dx, dy = p[0] - self.arena_start[0], p[1] - self.arena_start[1]
        return (self.odom_start[0] + c * dx - s * dy,
                self.odom_start[1] + s * dx + c * dy)

    def inverse_point(self, q):
        c, s = math.cos(self.rotation), math.sin(self.rotation)
        dx, dy = q[0] - self.odom_start[0], q[1] - self.odom_start[1]
        return (self.arena_start[0] + c * dx + s * dy,
                self.arena_start[1] - s * dx + c * dy)

    def yaw_to_arena(self, odom_yaw):
        return wrap_pi(odom_yaw - self.rotation)


class Polyline:
    """A 2-D polyline resampled at a fixed arc-length step."""

    def __init__(self, points, margins=None, step=0.02):
        if len(points) < 2:
            raise ValueError('a path needs at least 2 points')
        cum = [0.0]
        for a, b in zip(points[:-1], points[1:]):
            cum.append(cum[-1] + math.dist(a, b))
        if cum[-1] <= 0.0:
            raise ValueError('path has zero length')
        self.length = cum[-1]
        n = max(2, int(math.ceil(self.length / step)) + 1)
        self.step = self.length / (n - 1)
        self.s = [i * self.step for i in range(n)]
        self.xy = [_interp_xy(points, cum, si) for si in self.s]
        self.margin = ([_interp_scalar(margins, cum, si) for si in self.s]
                       if margins is not None else None)

    def _index(self, s):
        s = min(max(s, 0.0), self.length)
        i = min(int(s / self.step), len(self.s) - 2)
        return i, (s - self.s[i]) / self.step

    def at(self, s):
        i, f = self._index(s)
        a, b = self.xy[i], self.xy[i + 1]
        return (a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]))

    def tangent(self, s, half_window=0.05):
        a = self.at(s - half_window)
        b = self.at(s + half_window)
        d = math.dist(a, b)
        if d < 1e-9:
            return (1.0, 0.0)
        return ((b[0] - a[0]) / d, (b[1] - a[1]) / d)

    def curvature(self, s, half_window=0.15):
        """Heading change per metre over a +/- half_window chord."""
        t0 = self.tangent(s - half_window)
        t1 = self.tangent(s + half_window)
        dpsi = abs(wrap_pi(math.atan2(t1[1], t1[0]) - math.atan2(t0[1], t0[0])))
        return dpsi / (2.0 * half_window)

    def margin_at(self, s):
        if self.margin is None:
            return None
        i, f = self._index(s)
        return self.margin[i] + f * (self.margin[i + 1] - self.margin[i])

    def project(self, p, s_hint, window=1.0):
        """Arc length of the point nearest p, searched near s_hint."""
        lo = max(0, int((s_hint - window) / self.step))
        hi = min(len(self.s) - 1, int((s_hint + window) / self.step) + 1)
        best = min(range(lo, hi + 1), key=lambda i: math.dist(self.xy[i], p))
        return self.s[best]


def _interp_xy(points, cum, s):
    for i in range(len(cum) - 1):
        if s <= cum[i + 1] or i == len(cum) - 2:
            seg = cum[i + 1] - cum[i]
            f = 0.0 if seg <= 0 else min(max((s - cum[i]) / seg, 0.0), 1.0)
            a, b = points[i], points[i + 1]
            return (a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]))
    return points[-1]


def _interp_scalar(values, cum, s):
    for i in range(len(cum) - 1):
        if s <= cum[i + 1] or i == len(cum) - 2:
            seg = cum[i + 1] - cum[i]
            f = 0.0 if seg <= 0 else min(max((s - cum[i]) / seg, 0.0), 1.0)
            return values[i] + f * (values[i + 1] - values[i])
    return values[-1]


def speed_profile(line, v_max, v_min, full_speed_margin, accel, lateral_accel):
    """Speed limit at every sample of ``line``, starting and ending at rest.

    Slows where the planner's wall margin is thin and where the path bends,
    then applies accel/decel limits forwards and backwards.
    """
    v = []
    for s in line.s:
        lim = v_max
        m = line.margin_at(s)
        if m is not None and full_speed_margin > 0:
            frac = min(max(m / full_speed_margin, 0.0), 1.0)
            lim = min(lim, v_min + (v_max - v_min) * frac)
        k = line.curvature(s)
        if k > 1e-6:
            lim = min(lim, max(math.sqrt(lateral_accel / k), v_min))
        v.append(lim)
    v[0] = 0.0
    v[-1] = 0.0
    ds = line.step
    for i in range(len(v) - 2, -1, -1):
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2.0 * accel * ds))
    for i in range(1, len(v)):
        v[i] = min(v[i], math.sqrt(v[i - 1] ** 2 + 2.0 * accel * ds))
    return v


def profile_at(line, profile, s):
    i, f = line._index(s)
    return profile[i] + f * (profile[i + 1] - profile[i])


def profile_duration(line, profile):
    t = 0.0
    for a, b in zip(profile[:-1], profile[1:]):
        vm = 0.5 * (a + b)
        if vm > 1e-6:
            t += line.step / vm
    return t
