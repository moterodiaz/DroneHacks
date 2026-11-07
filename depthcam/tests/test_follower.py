"""follower: the route from the STEP map and the ground-truth checkpoints, and the mission flown on it. No ROS.

* The shipped routes (``routes/course_v3_*.json``) must trace to the map and rules files: start on
  the launch mark, end on the pad, cross CP1-CP4 in order, carry their hashes, and come out the
  same when planned again.
* Flown in closed loop by a PX4-like vehicle (first-order lag, capped at 1.0 m/s like the
  organisers' ``MPC_XY_VEL_MAX``), the window route must stay within the planner's margin.
* The mission's pre-takeoff checks, Offboard handling and landing. The vehicle model includes the
  kit's Offboard rule: at the pilot's switch to Position, Offboard starts only if a team setpoint is
  under 1 s old (``auto_offboard.py``), once per arming.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import numpy as np
import pytest

from follower.mission import (BLOCKED, FOLLOW, HOLD, LAND, LAND_HERE, LANDED, PRECHECK, STOPPED, TAKEOFF,
                              WAIT_FOR_OFFBOARD, Follower, FollowerConfig, Pose)
from follower.route import ROUTE_FORMAT, load_route, plan_to_dict, route_from_dict
from pose_fusion.course_files import launch_mark_from_files, load_course_map, load_rules
from pose_fusion.frames import CourseToTracker, LaunchMark, course_pose_to_tracker

ROOT = Path(__file__).resolve().parents[1]
MAP = ROOT / "course" / "maps" / "course_v3_map.json"
RULES = ROOT / "course" / "maps" / "course_v3_rules.json"
ROUTES = {name: ROOT / "routes" / "course_v3_{}.json".format(name) for name in ("window", "gate_b")}
REST = 0.07
DT = 1.0 / 50.0
MARK = launch_mark_from_files(MAP, RULES)
#: the measured identity for the drone on the launch mark facing north, passed explicitly
#: (issue #29, recommendation B: the measured transform has no default, not even this one).
TF = CourseToTracker(MARK, 0.5 * math.pi, 0.0, 0.0)
KIT_SETPOINT_TIMEOUT_S = 1.0                    # the flight interface's setpoint_timeout_s default


def lf_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes().replace(b"\r\n", b"\n")).hexdigest()


class Vehicle:
    """PX4 + the kit's interface, crudely: poses are servoed (tau 0.4 s, horizontal speed capped at 1.0 m/s),
    twists integrated, the floor stops it, and PX4 disarms 2 s after landing, which ends Offboard.
    Setpoints sent before Offboard are stored, not flown."""

    def __init__(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0) -> None:
        self.x, self.y, self.z, self.yaw = x, y, 0.0, yaw
        self.offboard = False
        self.refused = False
        self.last_cmd_t = None
        self.landed_since = None
        self.mode = None
        self.target = None
        self.twist = None

    def switch_to_position(self, t: float) -> None:
        """The pilot armed, then switched to Position: the kit's one Offboard attempt for this arming."""
        if self.last_cmd_t is not None and t - self.last_cmd_t <= KIT_SETPOINT_TIMEOUT_S:
            self.offboard = True
        else:
            self.refused = True                                        # "no team setpoint received"

    def apply(self, cmd, t: float) -> None:
        if cmd is not None:
            self.last_cmd_t = t
            if cmd.kind == "pose":
                self.mode, self.target = "pose", cmd
            else:
                self.mode, self.twist = "twist", cmd
        if self.offboard and self.mode is None:
            self.z += (0.3 - REST - self.z) * DT / 0.4                # the interface's own hold at its 0.3 m floor
        if self.mode == "pose" and self.offboard:
            vx, vy = (self.target.x - self.x) / 0.4, (self.target.y - self.y) / 0.4
            speed = math.hypot(vx, vy)
            if speed > 1.0:
                vx, vy = vx / speed, vy / speed
            self.x += vx * DT
            self.y += vy * DT
            self.z += (max(self.target.z, 0.3 - REST) - self.z) * DT / 0.4
            if self.target.yaw is not None:
                self.yaw += math.remainder(self.target.yaw - self.yaw, 2 * math.pi) * DT / 0.4
        elif self.mode == "twist" and self.offboard:
            self.x += self.twist.vx * DT
            self.y += self.twist.vy * DT
            self.z = max(0.0, self.z + self.twist.vz * DT)
        if self.z <= 0.005 and self.mode == "twist":
            self.landed_since = t if self.landed_since is None else self.landed_since
            if t - self.landed_since >= 2.0:
                self.offboard = False                                  # disarmed
        else:
            self.landed_since = None

    @property
    def landed(self) -> bool:
        return self.landed_since is not None

    def pose(self, t: float) -> Pose:
        return Pose(t, self.x, self.y, self.z, self.yaw)


class StandOriginVehicle(Vehicle):
    """cuVSLAM started on a stand and the drone was then placed on the mark: its z origin is the
    stand, 0.30 m above the floor-rest pose, so every pose it reports is the true rise minus 0.30,
    while PX4 still holds height on its rangefinder (EKF2_HGT_REF 2), on its own z reference."""

    def pose(self, t: float) -> Pose:
        p = Vehicle.pose(self, t)
        return Pose(p.t, p.x, p.y, p.z - 0.30, p.yaw)


def fly(follower: Follower, vehicle: Vehicle, seconds: float, offboard_at: float = 0.5, ready=lambda t: True):
    log = []
    for i in range(int(seconds / DT)):
        t = i * DT
        if abs(t - offboard_at) < DT / 2:
            vehicle.switch_to_position(t)                              # the pilot armed, then switched to Position
        cmd = follower.update(t, vehicle.offboard, vehicle.pose(t), vehicle.landed, ready(t))
        log.append((t, follower.state, cmd, vehicle.pose(t)))
        vehicle.apply(cmd, t)
        if follower.state == STOPPED and len(log) > 1 and log[-2][1] == STOPPED:
            break
    return log


def distance_to_path(path: np.ndarray, x: float, y: float) -> float:
    a, b = path[:-1], path[1:]
    ab = b - a
    t = np.clip(((x - a[:, 0]) * ab[:, 0] + (y - a[:, 1]) * ab[:, 1]) / np.maximum((ab ** 2).sum(1), 1e-12), 0, 1)
    px, py = a[:, 0] + t * ab[:, 0], a[:, 1] + t * ab[:, 1]
    return float(np.min(np.hypot(px - x, py - y)))


# ---------------------------------------------------------------------------------------------- the shipped routes

@pytest.mark.parametrize("name", ["window", "gate_b"])
def test_shipped_route_traces_to_the_map_and_the_checkpoints(name):
    data = json.loads(ROUTES[name].read_text(encoding="utf-8"))
    assert data["format"] == ROUTE_FORMAT and data["route"] == name
    assert data["map_sha256"] == lf_sha256(MAP) and data["rules_sha256"] == lf_sha256(RULES)
    cmap, rules = load_course_map(MAP), load_rules(RULES)
    pad = cmap.feature(rules["finish"]["marking"])
    assert data["start"] == pytest.approx([MARK.east, MARK.north], abs=1e-4)
    assert data["goal"] == pytest.approx(pad["centre"][:2], abs=1e-4)
    gates = [c["name"] for c in rules["checkpoints"] if not c.get("land")]
    assert [g["name"] for g in data["gates"]] == gates                  # CP1-CP4, in order
    idx = [g["index"] for g in data["gates"]]
    assert idx == sorted(idx) and len(set(idx)) == len(idx)
    route = load_route(ROUTES[name], TF)
    assert route.start == pytest.approx((0.0, 0.0), abs=1e-3)           # the tracker origin is the launch mark
    assert route.yaw == pytest.approx(0.0)                              # planned nose north = tracker yaw 0
    assert route.cruise_z_m == pytest.approx(0.8)
    assert data["min_room_m"] >= 0.0                                    # the planner kept its margin everywhere


def test_shipped_window_route_is_what_the_planner_makes_today():
    from course.planner.plan import PlanConfig, plan_course

    plan = plan_course(load_course_map(MAP), load_rules(RULES), "window", PlanConfig())
    fresh = plan_to_dict(plan, "window", lf_sha256(MAP), lf_sha256(RULES), "test")
    shipped = json.loads(ROUTES["window"].read_text(encoding="utf-8"))
    assert np.allclose(fresh["path"], shipped["path"], atol=1e-4)
    assert fresh["gates"] == shipped["gates"]


# ---------------------------------------------------------------------------------------------- the real route, flown

def test_window_route_flown_stays_inside_the_planners_margin():
    route = load_route(ROUTES["window"], TF)
    follower = Follower(route, FollowerConfig(rest_height_m=REST, land=True))   # the full mission, landing on
    log = fly(follower, Vehicle(), 240.0)
    states = [s for _, s, _, _ in log]
    order = [s for i, s in enumerate(states) if i == 0 or s != states[i - 1]]
    order = [s for s in order if s != PRECHECK]                        # zero ticks long when the checks pass at once
    assert order == [WAIT_FOR_OFFBOARD, TAKEOFF, FOLLOW, HOLD, LAND, LANDED, STOPPED], order
    assert [name for name, _ in follower.crossed] == [g.name for g in route.gates]
    worst = max(distance_to_path(route.path, p.x, p.y) for _, s, _, p in log if s == FOLLOW)
    assert worst < route.margin_m, "left the path by {:.3f} m (margin {:.3f} m)".format(worst, route.margin_m)
    assert worst < 0.03
    final = log[-1][3]
    assert math.hypot(final.x - route.goal[0], final.y - route.goal[1]) < 0.05 and final.z <= 0.005
    assert max(abs(math.remainder(p.yaw - route.yaw, 2 * math.pi)) for _, s, _, p in log if s == FOLLOW) < 1e-3


# ---------------------------------------------------------------------------------------------- the mission

def synthetic_route(corner: bool = True):
    """3 m north of a fixture launch mark, then (if corner) 2 m east, as a 4 cm polyline."""
    mark = LaunchMark(east=1.25, north=-0.5)
    tf = CourseToTracker(mark, 0.5 * math.pi, 0.0, 0.0)   # measured identity, passed explicitly
    pts = [(mark.east, mark.north + s) for s in np.arange(0.0, 3.0 + 1e-9, 0.04)]
    if corner:
        pts += [(mark.east + s, mark.north + 3.0) for s in np.arange(0.04, 2.0 + 1e-9, 0.04)]
    data = {"format": ROUTE_FORMAT, "route": "synthetic", "yaw_deg": 90.0, "cruise_z_m": 0.8,
            "path": [list(p) for p in pts], "gates": [{"index": 40, "name": "G1"}],
            "drone": {"margin_m": 0.05}, "min_room_m": 0.0}
    return route_from_dict(data, tf)


def test_route_converts_to_the_tracker_frame():
    route = synthetic_route()
    assert tuple(route.path[0]) == pytest.approx((0.0, 0.0))
    assert tuple(route.path[75]) == pytest.approx((3.0, 0.0))           # north = tracker +x
    assert tuple(route.path[-1]) == pytest.approx((3.0, -2.0))          # east = tracker -y
    assert route.yaw == pytest.approx(0.0)


def test_route_rejects_other_formats():
    with pytest.raises(ValueError, match=ROUTE_FORMAT):
        route_from_dict({"format": "dronehacks-route/0"}, TF)


def test_every_carrot_keeps_the_path_within_max_cut():
    route = synthetic_route()
    cfg = FollowerConfig(rest_height_m=REST)
    follower = Follower(route, cfg)
    vehicle = Vehicle()
    for i in range(int(60.0 / DT)):
        t = i * DT
        vehicle.offboard = vehicle.offboard or t >= 0.5
        cmd = follower.update(t, vehicle.offboard, vehicle.pose(t), vehicle.landed)
        if follower.state == FOLLOW and cmd is not None:
            j = int(np.argmin(np.hypot(route.path[:, 0] - cmd.x, route.path[:, 1] - cmd.y)))
            p0 = np.array([vehicle.x, vehicle.y])
            chord = route.path[j] - p0
            length = float(np.hypot(*chord))
            if length > 1e-9 and j > follower.index + 1:
                between = route.path[follower.index:j + 1]
                off = np.abs((between[:, 0] - p0[0]) * chord[1] - (between[:, 1] - p0[1]) * chord[0]) / length
                assert off.max() <= cfg.max_cut_m + 1e-9
        vehicle.apply(cmd, t)


def test_on_the_start_it_streams_the_takeoff_pose_before_offboard():
    follower = Follower(synthetic_route(), FollowerConfig(rest_height_m=REST))
    assert follower.update(0.0, False, None) is None and "no fresh pose" in follower.reason
    vehicle = Vehicle(x=0.1, y=-0.05, yaw=math.radians(4.0))           # inside the start tolerances
    before = [follower.update(i * DT, False, vehicle.pose(i * DT)) for i in range(25)]
    assert all(c is not None and c.kind == "pose" for c in before)     # every tick, so it is never stale for the kit
    c = before[-1]
    assert (c.x, c.y, c.z, c.yaw) == pytest.approx((0.1, -0.05, follower.z_fly, math.radians(4.0)))
    assert follower.state == WAIT_FOR_OFFBOARD and "sending the takeoff pose" in follower.reason
    first = follower.update(25 * DT, True, vehicle.pose(25 * DT))      # Offboard starts: the same pose, no jump
    assert follower.state == TAKEOFF
    assert first.kind == "pose" and (first.x, first.y, first.z, first.yaw) == pytest.approx((c.x, c.y, c.z, c.yaw))


def test_a_drone_carried_off_the_mark_sends_nothing_so_the_kit_refuses_offboard():
    route = synthetic_route()
    follower = Follower(route, FollowerConfig(rest_height_m=REST, start_timeout_s=5.0))
    vehicle = Vehicle(x=0.6, y=0.0)                                     # 0.6 m from the start
    log = fly(follower, vehicle, 10.0)
    assert vehicle.refused and not vehicle.offboard and vehicle.z == 0.0
    assert follower.state == WAIT_FOR_OFFBOARD and "measured course->odom transform" in follower.reason
    assert all(c is None for _, _, c, _ in log)


def test_a_wrong_heading_is_refused_offboard():
    follower = Follower(synthetic_route(), FollowerConfig(rest_height_m=REST, start_timeout_s=2.0))
    vehicle = Vehicle(yaw=math.radians(30.0))
    fly(follower, vehicle, 5.0)
    assert vehicle.refused and follower.state == WAIT_FOR_OFFBOARD and "heading" in follower.reason


def test_checks_that_fail_in_the_second_before_the_switch_still_block():
    follower = Follower(synthetic_route(), FollowerConfig(rest_height_m=REST, start_timeout_s=2.0))
    ticks = int(1.0 / DT)
    assert all(follower.update(i * DT, False, Pose(i * DT, 0.0, 0.0, 0.0, 0.0)) is not None for i in range(ticks))
    # carried 0.6 m just before the pilot's switch, so Offboard starts on the last streamed takeoff pose
    after = [follower.update((ticks + i) * DT, True, Pose((ticks + i) * DT, 0.6, 0.0, 0.0, 0.0))
             for i in range(int(3.0 / DT))]
    assert all(c is None for c in after)
    assert follower.state == BLOCKED and "measured course->odom transform" in follower.reason


def test_a_switch_before_ready_is_refused_and_a_switch_after_flies():
    early = Follower(synthetic_route(corner=False), FollowerConfig(rest_height_m=REST))
    vehicle = Vehicle()
    log = fly(early, vehicle, 10.0, offboard_at=0.5, ready=lambda t: t >= 3.0)
    assert next(t for t, _, c, _ in log if c is not None) >= 3.0      # nothing streamed before ready
    assert vehicle.refused and not vehicle.offboard and vehicle.z == 0.0   # one attempt per arming: used up
    late = Follower(synthetic_route(corner=False), FollowerConfig(rest_height_m=REST, land=True))
    log = fly(late, Vehicle(), 40.0, offboard_at=4.0, ready=lambda t: t >= 3.0)   # re-armed, switched once ready
    assert log[-1][1] == STOPPED


def test_takeoff_finishes_when_cuvslams_z_origin_is_not_the_floor_rest_pose():
    """PR #27 review, P1: the start checks cross-check only x, y and yaw against the measured
    transform, so a stand start passes them all with a resting tracker z of -0.30. PX4 still
    climbs to the requested height on its rangefinder, so the mission must measure the climb as
    a rise above the resting tracker z it recorded when the checks passed - not against z_fly
    bare, or it never leaves TAKEOFF and the route never starts."""
    follower = Follower(synthetic_route(corner=False), FollowerConfig(rest_height_m=REST))
    log = fly(follower, StandOriginVehicle(), 30.0)
    assert any(s == FOLLOW for _, s, _, _ in log), "takeoff never finished: " + follower.reason


def test_losing_offboard_stops_for_good():
    follower = Follower(synthetic_route(), FollowerConfig(rest_height_m=REST))
    vehicle = Vehicle()
    fly(follower, vehicle, 4.0)
    assert follower.state in (TAKEOFF, FOLLOW)
    assert follower.update(4.0, False, vehicle.pose(4.0)) is None
    assert follower.state == STOPPED
    assert follower.update(4.1, True, vehicle.pose(4.1)) is None


def test_a_stale_pose_in_flight_holds_still_by_velocity():
    # A held pose setpoint would point PX4 at the wrong place if cuVSLAM's frame jumps meanwhile;
    # a zero velocity setpoint doesn't depend on any frame
    follower = Follower(synthetic_route(), FollowerConfig(rest_height_m=REST))
    vehicle = Vehicle()
    fly(follower, vehicle, 4.0)
    cmd = follower.update(4.0, True, Pose(0.0, vehicle.x, vehicle.y, vehicle.z, vehicle.yaw))
    assert cmd.kind == "twist" and (cmd.vx, cmd.vy, cmd.vz, cmd.yaw_rate) == (0.0, 0.0, 0.0, 0.0)
    assert "no fresh pose" in follower.reason


def test_a_stale_pose_before_offboard_still_sends_nothing():
    follower = Follower(synthetic_route(), FollowerConfig(rest_height_m=REST))
    assert follower.update(0.0, False, None) is None and "no fresh pose" in follower.reason


def test_a_stale_pose_while_landing_holds_then_lands_straight_down():
    """The author's in-flight hold (PR #27, 44d320e): a stale pose in LAND holds by zero twist like
    any flight state; only once the hold outlasts realign_timeout_s does it give up the route and
    descend straight down (LAND_HERE), which needs no pose."""
    follower = Follower(synthetic_route(corner=False), FollowerConfig(rest_height_m=REST))
    z = follower.z_fly
    for i in range(20):                                   # offboard, past PRECHECK, then cut to LAND
        follower.update(i * DT, True, Pose(i * DT, 0.0, 0.0, z, 0.0), None, True, True)
    assert follower.state in (TAKEOFF, FOLLOW)
    follower.state = LAND
    stale = Pose(0.0, 0.0, 0.0, z, 0.0)                   # pose.t = 0: stale at t = 21 s
    cmd = follower.update(21.0, True, stale, None, True, True)
    assert cmd.kind == "twist" and (cmd.vx, cmd.vy, cmd.vz) == (0.0, 0.0, 0.0)   # holds, no descent
    assert "no fresh pose" in follower.reason
    cmd = follower.update(21.0 + follower.config.realign_timeout_s + DT, True, stale, None, True, True)
    assert follower.state == LAND_HERE and cmd.kind == "twist"
    assert (cmd.vx, cmd.vy) == (0.0, 0.0) and cmd.vz < 0.0                  # gives up: straight down


def test_the_default_holds_over_the_pad_for_the_pilot():
    """The default (issue #25 point 4): land is false, so it holds over the pad with a pose
    setpoint and the pilot takes over the landing, until a landing API is contracted."""
    follower = Follower(synthetic_route(corner=False), FollowerConfig(rest_height_m=REST))
    log = fly(follower, Vehicle(), 40.0)
    assert log[-1][1] == HOLD and log[-1][2].kind == "pose"
    assert all(s not in (LAND, LANDED) for _, s, _, _ in log)


def test_landing_is_a_downward_twist():
    follower = Follower(synthetic_route(corner=False), FollowerConfig(rest_height_m=REST, land=True))
    log = fly(follower, Vehicle(), 40.0)
    assert all(c.kind == "twist" and c.vz < 0.0 for _, s, c, _ in log if s in (LAND, LANDED) and c is not None)


def test_cruise_must_be_above_the_rest_height():
    data = json.loads(ROUTES["window"].read_text(encoding="utf-8"))
    data["cruise_z_m"] = 0.05
    with pytest.raises(ValueError, match="rest height"):
        Follower(route_from_dict(data, TF), FollowerConfig(rest_height_m=REST))


def test_course_points_use_the_fusions_transform():
    p = course_pose_to_tracker(MARK.east, MARK.north + 1.0, 0.0, TF)
    assert (p.x, p.y) == pytest.approx((1.0, 0.0))


# ---------------------------------------------------------------------------------------------- route provenance

def test_a_route_is_refused_with_other_map_or_rules(tmp_path):
    route = load_route(ROUTES["window"], TF, MAP, RULES)            # the shipped files: accepted
    assert len(route.path) > 2
    other_map = tmp_path / "map.json"
    other_map.write_bytes(MAP.read_bytes() + b" ")
    with pytest.raises(ValueError, match="different map file"):
        load_route(ROUTES["window"], TF, other_map, RULES)
    other_rules = tmp_path / "rules.json"
    other_rules.write_bytes(RULES.read_bytes().replace(b"\n", b"\n\n", 1))
    with pytest.raises(ValueError, match="different rules file"):
        load_route(ROUTES["window"], TF, MAP, other_rules)


def test_a_route_without_hashes_is_refused_when_checked(tmp_path):
    data = json.loads(ROUTES["window"].read_text(encoding="utf-8"))
    del data["map_sha256"]
    stripped = tmp_path / "route.json"
    stripped.write_text(json.dumps(data), encoding="utf-8")
    with pytest.raises(ValueError, match="records map_sha256 None"):
        load_route(stripped, TF, MAP, RULES)
    assert len(load_route(stripped, TF).path) > 2                    # unchecked when no files are given
