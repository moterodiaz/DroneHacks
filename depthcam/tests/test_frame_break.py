"""A cuVSLAM frame break mid-route: the Guard's ``tracking``, pose_fusion's ``aligned``, the follower's hold. No ROS.

cuVSLAM loses tracking for 0.5 s, then re-initialises in a frame jumped 0.6 m / -0.4 m / 15 deg.
PX4 gets the resumed poses at once through the organisers' bridge, so its estimate jumps then.
Our chain sees them only after the Guard's 0.2 s of settling. Until pose_fusion re-learns its
correction, the true pose the follower flies on and the setpoints sent to PX4 are both wrong by
the jump. Here a PX4-like vehicle flies a route in its own odom frame through the whole chain
(Follower -> C⁻¹ -> vehicle; vehicle -> VO -> PoseFusion -> Follower), with lidar fixes of the
truth.

* **With the hold** (the default): the drone holds still by velocity from the first withheld
  frame. So it doesn't move when PX4's frame jumps, and it carries on once the correction is
  back, within the planner's margin of the path. The pose stream going quiet triggers the same
  hold without the Guard's signal.
* **Without the realign hold** (``require_ready=False``): it flies on the old correction and ends
  up far off the path.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np
import pytest

from follower.mission import FOLLOW, HOLD, LAND, LAND_HERE, LANDED, TAKEOFF, Command, Follower, FollowerConfig, Pose
from follower.route import ROUTE_FORMAT, route_from_dict
from pose_fusion.frames import CourseToTracker, LaunchMark
from pose_fusion.fusion import FusionConfig, LidarFix, PoseFusion, VoSample
from pose_fusion.policy import LidarPolicy
from pose_fusion.se2 import IDENTITY, Pose2, quat_about_z, quat_yaw, wrap_angle
from pose_fusion.setpoints import PoseSetpoint, TwistSetpoint, to_odom_pose, to_odom_twist

DT = 1.0 / 50.0
REST = 0.07
FIXTURE_MARK = LaunchMark(east=1.25, north=-0.5)       # synthetic, not the course's launch mark
#: the measured identity for a launch-mark placement facing north, passed explicitly (issue #29,
#: recommendation B: the measured transform has no default, not even this one).
FIXTURE_TF = CourseToTracker(FIXTURE_MARK, 0.5 * math.pi, 0.0, 0.0)
JUMP = Pose2(0.6, -0.4, math.radians(15.0))           # new odom = JUMP ∘ old odom, at the same place


def fixture_route():
    """3 m north of the fixture mark, then 2 m east: a 4 cm polyline, nose north."""
    m = FIXTURE_MARK
    pts = [(m.east, m.north + s) for s in np.arange(0.0, 3.0 + 1e-9, 0.04)]
    pts += [(m.east + s, m.north + 3.0) for s in np.arange(0.04, 2.0 + 1e-9, 0.04)]
    data = {"format": ROUTE_FORMAT, "route": "fixture", "yaw_deg": 90.0, "cruise_z_m": 0.8,
            "path": [list(p) for p in pts], "gates": [], "drone": {"margin_m": 0.05}, "min_room_m": 0.0}
    return route_from_dict(data, FIXTURE_TF)


def distance_to_path(path: np.ndarray, x: float, y: float) -> float:
    a, b = path[:-1], path[1:]
    ab = b - a
    t = np.clip(((x - a[:, 0]) * ab[:, 0] + (y - a[:, 1]) * ab[:, 1]) / np.maximum((ab ** 2).sum(1), 1e-12), 0, 1)
    return float(np.min(np.hypot(a[:, 0] + t * ab[:, 0] - x, a[:, 1] + t * ab[:, 1] - y)))


def lidar_fix(t: float, pose: Pose2) -> LidarFix:
    return LidarFix(t, pose, np.diag([0.02 ** 2, 0.02 ** 2, math.radians(0.5) ** 2]))


def is_still(cmd: Optional[Command]) -> bool:
    return (cmd is not None and cmd.kind == "twist"
            and (cmd.vx, cmd.vy, cmd.vz, cmd.yaw_rate) == (0.0, 0.0, 0.0, 0.0))


class OdomVehicle:
    """PX4 flying in its odom frame: poses servoed (tau 0.4 s, at most 1 m/s), twists integrated."""

    def __init__(self) -> None:
        self.odom = IDENTITY
        self.z = 0.0
        self.target: Optional[PoseSetpoint] = None
        self.twist: Optional[TwistSetpoint] = None

    def apply(self, sp, flying: bool) -> None:
        if isinstance(sp, PoseSetpoint):
            self.target, self.twist = sp, None
        elif isinstance(sp, TwistSetpoint):
            self.target, self.twist = None, sp
        if not flying:
            return
        o = self.odom
        if self.target is not None:
            vx, vy = (self.target.x - o.x) / 0.4, (self.target.y - o.y) / 0.4
            speed = math.hypot(vx, vy)
            if speed > 1.0:
                vx, vy = vx / speed, vy / speed
            yaw = o.yaw
            if any(self.target.q):
                yaw += wrap_angle(quat_yaw(self.target.q) - o.yaw) * DT / 0.4
            self.odom = Pose2(o.x + vx * DT, o.y + vy * DT, wrap_angle(yaw))
            self.z += (max(self.target.z, 0.3 - REST) - self.z) * DT / 0.4
        elif self.twist is not None:
            self.odom = Pose2(o.x + self.twist.vx * DT, o.y + self.twist.vy * DT,
                              wrap_angle(o.yaw + self.twist.yaw_rate * DT))
            self.z = max(0.0, self.z + self.twist.vz * DT)


SETTLE_S = 0.2       # the Guard's settle time: PX4 has the resumed poses this much before our chain


@dataclass(frozen=True)
class Row:
    t: float
    state: str
    truth: Pose2         # where the drone really is
    cmd: Optional[Command]
    aligned: bool
    blind: bool          # our chain gets no pose (tracking lost, or the Guard settling)
    jumped: bool         # PX4's frame has jumped


def fly(hold: bool, jump_after_follow_s: float = 2.0, gap_s: float = 0.5, lidar_after_jump: bool = True,
        tracking_signal: bool = True, seconds: float = 60.0, seed: int = 3):
    """Fly the fixture route through the whole chain; returns (rows, follower, fusion).

    Tracking is lost for *gap_s*, from *jump_after_follow_s* into FOLLOW. Then PX4's frame jumps
    at once, and our chain gets poses again after another ``SETTLE_S``. *tracking_signal*: the
    Guard's ``~/tracking`` reaches the follower (else only the quiet pose stream does).
    pose_fusion's node logic is reproduced: snap the correction while the follower's last command
    is a zero twist and the frames don't agree.
    """
    rng = np.random.default_rng(seed)
    route = fixture_route()
    # land=True is passed explicitly: the closed loop exercises the landing path, though the
    # shipped default (issue #25 point 4) holds over the pad for the pilot
    follower = Follower(route, FollowerConfig(rest_height_m=REST, require_ready=hold, land=True))
    fusion = PoseFusion(LidarPolicy.ALLOWED, FusionConfig())
    vehicle = OdomVehicle()
    drift = IDENTITY                     # truth = drift ∘ odom
    pending: List[Tuple[float, LidarFix]] = []
    pose: Optional[Pose] = None
    follow_since = gap_start = None
    jumped = False
    last_cmd: Optional[Command] = None
    rows: List[Row] = []
    for i in range(int(seconds / DT)):
        t = i * DT
        drift = Pose2(drift.x + 0.004 * DT, drift.y - 0.003 * DT, drift.yaw + math.radians(0.05) * DT)
        if follower.state == FOLLOW and follow_since is None:
            follow_since = t
        if follow_since is not None and gap_start is None and t - follow_since >= jump_after_follow_s:
            gap_start = t
        jump_t = None if gap_start is None else gap_start + gap_s
        blind = gap_start is not None and gap_start <= t < jump_t + SETTLE_S
        if jump_t is not None and t >= jump_t and not jumped:
            # cuVSLAM re-initialised: same place, new frame. PX4 gets it now; our Guard settles first
            jumped = True
            vehicle.odom = JUMP.compose(vehicle.odom)
            drift = drift.compose(JUMP.inverse())
        truth = drift.compose(vehicle.odom)
        if i % 5 == 0 and (lidar_after_jump or not jumped):
            noisy = Pose2(truth.x + rng.normal(0, 0.01), truth.y + rng.normal(0, 0.01),
                          truth.yaw + rng.normal(0, math.radians(0.3)))
            pending.append((t + 0.05, lidar_fix(t, noisy)))
        while pending and pending[0][0] <= t:
            fusion.add_lidar(pending.pop(0)[1])
        if not blind:
            if not fusion.aligned() and is_still(last_cmd):
                fusion.snap_to_estimate()
            out = fusion.add_vo(VoSample(t, vehicle.odom.x, vehicle.odom.y, vehicle.z, quat_about_z(vehicle.odom.yaw)))
            pose = Pose(t, out.x, out.y, out.z, quat_yaw(out.q), received=t)
        offboard = t >= 3.0
        landed = vehicle.z <= 0.005 and follower.state in (LAND, LANDED, LAND_HERE)
        tracking = not blind if tracking_signal else True
        cmd = follower.update(t, offboard, pose, landed, fusion.settled(), fusion.aligned(), tracking)
        sp = None
        if cmd is not None:
            c = fusion.correction_applied
            if cmd.kind == "pose":
                q = quat_about_z(cmd.yaw) if cmd.yaw is not None else (0.0, 0.0, 0.0, 0.0)
                sp = to_odom_pose(PoseSetpoint(cmd.x, cmd.y, cmd.z, q), c)
            else:
                sp = to_odom_twist(TwistSetpoint(cmd.vx, cmd.vy, cmd.vz, cmd.yaw_rate), c)
            last_cmd = cmd
        vehicle.apply(sp, offboard)
        rows.append(Row(t, follower.state, drift.compose(vehicle.odom), cmd, fusion.aligned(), blind, jumped))
        if follower.state in (LANDED, LAND_HERE) and vehicle.z <= 0.005:
            break
    return rows, follower, fusion


def worst_off_path(rows, route) -> float:
    return max(distance_to_path(route.path, r.truth.x, r.truth.y) for r in rows if r.state in (FOLLOW, HOLD))


def moved_while_blind_after_the_jump(rows) -> float:
    """How far the drone really moved between PX4's frame jump and our chain seeing poses again."""
    after = [r.truth for r in rows if r.blind and r.jumped]
    return max(math.hypot(p.x - after[0].x, p.y - after[0].y) for p in after)


# ---------------------------------------------------------------------------------------------- closed loop

def test_without_a_break_the_route_never_holds():
    rows, follower, _ = fly(hold=True, jump_after_follow_s=1e9)
    assert follower.holds == 0
    assert all(r.aligned for r in rows if r.state in (TAKEOFF, FOLLOW, HOLD))
    assert worst_off_path(rows, fixture_route()) < 0.03


def test_a_frame_break_mid_route_holds_then_carries_on_within_the_margin():
    rows, follower, fusion = fly(hold=True)
    route = fixture_route()
    assert fusion.stats.vo_gaps == 1 and fusion.stats.reacquired == 1
    first_blind = next(r for r in rows if r.blind)
    assert is_still(first_blind.cmd), "the hold starts at the first withheld frame"
    assert moved_while_blind_after_the_jump(rows) < 0.005             # PX4's jump moves nothing
    held = [r.t for r in rows if is_still(r.cmd) and r.state in (FOLLOW, HOLD)]
    assert held and max(held) - first_blind.t < 2.0, "held for {:.1f} s".format(max(held) - first_blind.t)
    worst = worst_off_path(rows, route)
    assert worst < route.margin_m, "left the path by {:.3f} m (margin {:.3f})".format(worst, route.margin_m)
    final = rows[-1].truth
    assert rows[-1].state == LANDED
    assert math.hypot(final.x - route.goal[0], final.y - route.goal[1]) < 0.06


def test_a_quiet_pose_stream_alone_holds_before_the_jump():
    rows, follower, _ = fly(hold=True, tracking_signal=False)
    route = fixture_route()
    first_blind = next(r for r in rows if r.blind)
    first_still = next(r for r in rows if r.t >= first_blind.t and is_still(r.cmd))
    assert first_still.t - first_blind.t <= follower.config.pose_gap_s + 2 * DT
    assert not first_still.jumped                                      # held before PX4's frame jumped
    assert moved_while_blind_after_the_jump(rows) < 0.005
    assert worst_off_path(rows, route) < route.margin_m


def test_without_the_realign_hold_the_same_break_leaves_the_path():
    rows, _, _ = fly(hold=False)
    route = fixture_route()
    worst = worst_off_path(rows, route)
    assert worst > 3.0 * route.margin_m, "left the path by only {:.3f} m".format(worst)


def test_a_break_with_the_lidar_gone_lands_where_it_is():
    rows, follower, _ = fly(hold=True, lidar_after_jump=False)
    states = [r.state for r in rows]
    assert LAND_HERE in states and LANDED not in states
    landing = [r.cmd for r in rows if r.state == LAND_HERE]
    assert all(c.kind == "twist" and c.vx == 0.0 and c.vy == 0.0 and c.vz < 0.0 for c in landing)


# The stale-pose regression (a gap past pose_timeout_s holds by zero twist, not silence) is the
# author's, on their refactored Row-based harness: test_a_quiet_stream_is_stale_within_the_pose_gap,
# test_the_hold_is_a_zero_twist_and_makes_no_progress and
# test_a_pose_delayed_after_the_guard_is_stale_although_it_just_arrived below cover it.


# ---------------------------------------------------------------------------------------------- pose_fusion.aligned

def test_aligned_through_normal_lidar_updates_and_false_after_a_gap():
    fusion = PoseFusion(LidarPolicy.ALLOWED)
    truth = Pose2(0.5, 0.2, 0.1)
    for i in range(90):
        t = i * DT
        if i % 5 == 0 and i:
            fusion.add_lidar(lidar_fix(t - DT, truth))
        fusion.add_vo(VoSample(t, truth.x, truth.y, 0.8, quat_about_z(truth.yaw)))
        assert fusion.aligned()
    fusion.add_vo(VoSample(90 * DT + 0.5, truth.x, truth.y, 0.8, quat_about_z(truth.yaw)))
    assert fusion.stats.vo_gaps == 1 and not fusion.aligned() and not fusion.settled()


def test_one_outlier_keeps_aligned_two_in_a_row_do_not():
    fusion = PoseFusion(LidarPolicy.ALLOWED)
    truth = IDENTITY
    t = 0.0
    for i in range(60):
        t = i * DT
        if i % 5 == 0 and i:
            fusion.add_lidar(lidar_fix(t - DT, truth))
        fusion.add_vo(VoSample(t, 0.0, 0.0, 0.8, quat_about_z(0.0)))
    far = Pose2(2.0, 0.0, 0.0)
    fusion.add_lidar(lidar_fix(t - 0.5 * DT, far))
    assert fusion.stats.consecutive_gated == 1 and fusion.aligned()
    fusion.add_vo(VoSample(t + DT, 0.0, 0.0, 0.8, quat_about_z(0.0)))
    fusion.add_lidar(lidar_fix(t + 0.5 * DT, far))
    assert fusion.stats.consecutive_gated == 2 and not fusion.aligned()
    fusion.add_vo(VoSample(t + 2 * DT, 0.0, 0.0, 0.8, quat_about_z(0.0)))
    fusion.add_lidar(lidar_fix(t + 1.5 * DT, truth))
    assert fusion.aligned()


def test_snap_applies_the_estimate_at_once_and_forbidden_is_always_aligned():
    fusion = PoseFusion(LidarPolicy.ALLOWED)
    fusion._c = np.array([0.4, -0.2, 0.1])          # a re-learnt correction, not yet applied
    fusion.add_vo(VoSample(0.0, 1.0, 0.0, 0.8, quat_about_z(0.0)))
    assert not fusion.aligned()
    fusion.snap_to_estimate()
    assert fusion.correction_applied == fusion.correction_estimate and fusion.aligned()
    forbidden = PoseFusion(LidarPolicy.FORBIDDEN)
    forbidden.add_vo(VoSample(0.0, 1.0, 0.0, 0.8, quat_about_z(0.0)))
    forbidden.add_vo(VoSample(5.0, 9.0, 0.0, 0.8, quat_about_z(0.0)))
    forbidden.snap_to_estimate()
    assert forbidden.aligned() and forbidden.correction_applied == IDENTITY


# ---------------------------------------------------------------------------------------------- the follower's hold

def _following(**cfg):
    follower = Follower(fixture_route(), FollowerConfig(rest_height_m=REST, **cfg))
    z = follower.z_fly
    # Offboard starts with the drone resting on the floor: PRECHECK runs there and records the
    # resting tracker z the climb is measured over (PR #27 review), then the drone is at z_fly.
    follower.update(0.0, True, Pose(0.0, 0.0, 0.0, 0.0, 0.0), None, True, True)
    for i in range(1, 20):                                # take off and start following
        follower.update(i * DT, True, Pose(i * DT, 0.0, 0.0, z, 0.0), None, True, True)
    assert follower.state == FOLLOW
    return follower, z


def test_the_hold_is_a_zero_twist_and_makes_no_progress():
    follower, z = _following()
    index = follower.index
    for i in range(20, 60):
        cmd = follower.update(i * DT, True, Pose(i * DT, 0.5, 0.0, z, 0.0), None, True, False)
        assert is_still(cmd) and follower.state == FOLLOW
    assert follower.index == index and "realigning" in follower.reason
    cmd = follower.update(60 * DT, True, Pose(60 * DT, 0.5, 0.0, z, 0.0), None, True, True)
    assert cmd.kind == "pose" and follower.holds == 1 and follower.reason == ""


def test_the_hold_times_out_into_landing_straight_down_even_without_a_pose():
    follower, z = _following(realign_timeout_s=1.0)
    for i in range(20, 20 + int(1.0 / DT) + 2):
        cmd = follower.update(i * DT, True, Pose(i * DT, 0.5, 0.0, z, 0.0), None, True, False)
    assert follower.state == LAND_HERE and (cmd.vx, cmd.vy) == (0.0, 0.0) and cmd.vz < 0.0
    cmd = follower.update(10.0, True, None, None, True, True)              # no pose at all: still descending
    assert cmd.kind == "twist" and cmd.vz < 0.0 and follower.state == LAND_HERE
    assert follower.update(11.0, False, None) is None and follower.reason.startswith("Offboard ended")


def test_require_ready_false_ignores_aligned():
    follower, z = _following(require_ready=False)
    cmd = follower.update(1.0, True, Pose(1.0, 0.5, 0.0, z, 0.0), None, True, False)
    assert cmd.kind == "pose" and follower.holds == 0


@pytest.mark.parametrize("aligned", [True, False])
def test_landed_keeps_pressing_down_whatever_aligned_says(aligned):
    follower, z = _following()
    follower.state = LANDED
    cmd = follower.update(1.0, True, Pose(1.0, 0.5, 0.0, 0.0, 0.0), True, True, aligned)
    assert cmd.kind == "twist" and cmd.vz < 0.0


# ---------------------------------------------------------------------------------------------- what counts as a fresh pose

def test_a_pose_delayed_after_the_guard_is_stale_although_it_just_arrived():
    follower, z = _following()
    late = Pose(1.0 - 0.6, 0.5, 0.0, z, 0.0, received=1.0)              # captured 0.6 s ago, arrived now
    cmd = follower.update(1.0, True, late, None, True, True)
    assert is_still(cmd) and "no fresh pose" in follower.reason
    assert not follower.fresh(1.0, late) and follower.fresh(1.0, Pose(0.95, 0.5, 0.0, z, 0.0, received=1.0))


def test_a_quiet_stream_is_stale_within_the_pose_gap():
    follower, z = _following()
    last = Pose(1.0, 0.5, 0.0, z, 0.0, received=1.0)
    assert follower.fresh(1.0 + follower.config.pose_gap_s, last)
    assert not follower.fresh(1.0 + follower.config.pose_gap_s + 0.01, last)
    assert is_still(follower.update(1.2, True, last, None, True, True))


def test_the_guard_withholding_holds_from_its_first_frame():
    follower, z = _following()
    cmd = follower.update(1.0, True, Pose(1.0, 0.5, 0.0, z, 0.0, received=1.0), None, True, True, False)
    assert is_still(cmd) and "Guard" in follower.reason
    cmd = follower.update(1.02, True, Pose(1.02, 0.5, 0.0, z, 0.0, received=1.02), None, True, True, True)
    assert cmd.kind == "pose"


def test_before_offboard_a_stale_pose_still_sends_nothing():
    follower = Follower(fixture_route(), FollowerConfig(rest_height_m=REST))
    assert follower.update(1.0, False, Pose(0.3, 0.0, 0.0, 0.0, 0.0, received=1.0), None, True, True) is None
    assert "no fresh pose" in follower.reason
