"""Correcting the flight commands: pose_fusion.setpoints, the fusion's re-acquire, and a closed loop.

The closed loop stands in for the issued drone. PX4 flies to setpoints in its own odom frame
(raw cuVSLAM), and that frame drifts away from the true one. With the corrected commands, the
drone's true position reaches a target given in the true frame. No ROS.
"""

from __future__ import annotations

import math
from types import SimpleNamespace

import numpy as np
import pytest

from pose_fusion.fusion import FusionConfig, LidarFix, PoseFusion, VoSample
from pose_fusion.policy import LidarPolicy
from pose_fusion.ros_io import (fill_pose_setpoint, fill_twist_setpoint, pose_setpoint_from_msg,
                                twist_setpoint_from_msg)
from pose_fusion.se2 import IDENTITY, Pose2, quat_about_z, quat_yaw, wrap_angle
from pose_fusion.setpoints import (PoseSetpoint, TwistSetpoint, correction_moved, refresh_due, to_odom_pose,
                                   to_odom_twist)

DT = 1.0 / 30.0


def fix(t, pose, sigma=0.02, sigma_yaw=math.radians(0.5)):
    return LidarFix(t, pose, np.diag([sigma ** 2, sigma ** 2, sigma_yaw ** 2]))


def vo(t, pose):
    return VoSample(t, pose.x, pose.y, 0.8, quat_about_z(pose.yaw))


# ---------------------------------------------------------------------------------------------- the maths

def test_odom_setpoint_lands_on_the_true_target():
    c = Pose2(0.3, -0.2, math.radians(12.0))
    target = PoseSetpoint(2.0, 1.0, 0.8, quat_about_z(0.4))
    odom = to_odom_pose(target, c)
    back = c.compose(Pose2(odom.x, odom.y, quat_yaw(odom.q)))
    assert (back.x, back.y) == pytest.approx((2.0, 1.0))
    assert wrap_angle(back.yaw - 0.4) == pytest.approx(0.0, abs=1e-12)
    assert odom.z == 0.8


def test_all_zero_quaternion_still_means_yaw_not_commanded():
    out = to_odom_pose(PoseSetpoint(1.0, 1.0, 1.0, (0.0, 0.0, 0.0, 0.0)), Pose2(0.1, 0.1, 0.5))
    assert out.q == (0.0, 0.0, 0.0, 0.0)


def test_non_finite_setpoints_pass_unchanged():
    bad = PoseSetpoint(float("nan"), 0.0, 1.0, (0.0, 0.0, 0.0, 1.0))
    assert to_odom_pose(bad, Pose2(1.0, 1.0, 1.0)) is bad
    bad_twist = TwistSetpoint(float("inf"), 0.0, 0.0, 0.0)
    assert to_odom_twist(bad_twist, Pose2(0.0, 0.0, 1.0)) is bad_twist


def test_twist_is_turned_into_the_odom_frame():
    out = to_odom_twist(TwistSetpoint(1.0, 0.0, 0.2, 0.3), Pose2(5.0, 5.0, math.pi / 2))
    assert (out.vx, out.vy) == pytest.approx((0.0, -1.0), abs=1e-12)
    assert (out.vz, out.yaw_rate) == (0.2, 0.3)


def test_identity_correction_changes_nothing():
    sp = PoseSetpoint(1.5, -2.5, 0.8, quat_about_z(0.7))
    out = to_odom_pose(sp, IDENTITY)
    assert (out.x, out.y, out.z) == pytest.approx((sp.x, sp.y, sp.z))
    assert out.q == pytest.approx(sp.q)


def test_correction_moved():
    assert not correction_moved(Pose2(0.0, 0.0, 0.0), Pose2(0.001, 0.0, 0.0))
    assert correction_moved(Pose2(0.0, 0.0, 0.0), Pose2(0.003, 0.0, 0.0))
    assert correction_moved(Pose2(0.0, 0.0, 0.0), Pose2(0.0, 0.0, math.radians(0.2)))


def test_a_silent_follower_setpoint_is_never_refreshed():
    moved, sent = Pose2(0.01, 0.0, 0.0), IDENTITY
    assert refresh_due(moved, sent, 0.1, 0.5)                          # the follower is streaming
    assert not refresh_due(moved, sent, 0.6, 0.5)                      # silent: don't make it look live to the kit
    assert not refresh_due(moved, sent, None, 0.5)                     # nothing received yet
    assert not refresh_due(moved, sent, float("nan"), 0.5)
    assert not refresh_due(IDENTITY, sent, 0.1, 0.5)                   # streaming, but the correction hasn't moved


def test_setpoint_messages_round_trip():
    q = quat_about_z(0.3)
    pose_msg = SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=1.0, y=2.0, z=0.8),
                                                    orientation=SimpleNamespace(x=q[0], y=q[1], z=q[2], w=q[3])))
    sp = pose_setpoint_from_msg(pose_msg)
    out = SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(), orientation=SimpleNamespace()))
    fill_pose_setpoint(out, sp)
    assert (out.pose.position.x, out.pose.position.y, out.pose.position.z) == (1.0, 2.0, 0.8)
    assert out.pose.orientation.w == pytest.approx(q[3])
    twist_msg = SimpleNamespace(twist=SimpleNamespace(linear=SimpleNamespace(x=0.5, y=0.0, z=0.1),
                                                      angular=SimpleNamespace(x=0.0, y=0.0, z=0.2)))
    tw = twist_setpoint_from_msg(twist_msg)
    out_t = SimpleNamespace(twist=SimpleNamespace(linear=SimpleNamespace(), angular=SimpleNamespace()))
    fill_twist_setpoint(out_t, tw, twist_msg)
    assert (out_t.twist.linear.x, out_t.twist.linear.z, out_t.twist.angular.z) == (0.5, 0.1, 0.2)


# ---------------------------------------------------------------------------------------------- re-acquiring

def test_reacquires_after_cuvslam_jumps_its_frame_in_a_gap():
    """cuVSLAM re-initialises during a Guard break: its frame jumps 1.0 m / -0.5 m / 20 deg."""
    fusion = PoseFusion(LidarPolicy.ALLOWED)
    truth = Pose2(1.0, 0.5, 0.3)
    jump = Pose2(1.0, -0.5, math.radians(20.0))                   # new odom = jump ∘ old odom, same place
    t = 0.0
    for i in range(90):                                           # 3 s hovering, C ≈ identity
        t = i * DT
        if i % 3 == 0 and i:
            fusion.add_lidar(fix(t - DT, truth))
        fusion.add_vo(vo(t, truth))
    odom_after = jump.compose(truth)
    t += 0.5                                                      # the Guard's break
    outs = []
    for i in range(900):                                          # 30 s
        t += DT
        if i % 3 == 0 and i:
            fusion.add_lidar(fix(t - DT, truth))
        outs.append(fusion.add_vo(vo(t, odom_after)))
    assert fusion.stats.vo_gaps == 1 and fusion.stats.reacquired == 1
    first = outs[0]
    assert math.hypot(first.x - odom_after.x, first.y - odom_after.y) < 1e-9   # C kept across the gap: no jump
    last = outs[-1]
    assert math.hypot(last.x - truth.x, last.y - truth.y) < 0.03
    assert abs(wrap_angle(quat_yaw(last.q) - truth.yaw)) < math.radians(1.0)


def test_a_wide_placement_error_is_reacquired_not_ignored_forever():
    fusion = PoseFusion(LidarPolicy.ALLOWED)
    placed_wrong = Pose2(0.35, -0.25, math.radians(10.0))         # well past the prior's ~3 sigma
    for i in range(600):
        t = i * DT
        if i % 3 == 0 and i:
            fusion.add_lidar(fix(t - DT, placed_wrong))
        out = fusion.add_vo(vo(t, IDENTITY))
    assert fusion.stats.reacquired == 1
    assert math.hypot(out.x - placed_wrong.x, out.y - placed_wrong.y) < 0.02


def test_scattered_outliers_never_reacquire():
    fusion = PoseFusion(LidarPolicy.ALLOWED)
    rng = np.random.default_rng(1)
    truth = Pose2(0.0, 0.0, 0.0)
    for i in range(300):
        t = i * DT
        if i % 3 == 0 and i:
            bad = Pose2(float(rng.uniform(-3, 3)), float(rng.uniform(-3, 3)), float(rng.uniform(-3, 3)))
            fusion.add_lidar(fix(t - DT, bad))
        out = fusion.add_vo(vo(t, truth))
    assert fusion.stats.reacquired == 0
    assert math.hypot(out.x, out.y) < 0.05


# ---------------------------------------------------------------------------------------------- closed loop

def fly_closed_loop(corrected: bool, seconds: float = 60.0, seed: int = 4):
    """PX4 servoes its odom pose onto the setpoint (tau 0.5 s); its odom frame drifts from the true one."""
    rng = np.random.default_rng(seed)
    fusion = PoseFusion(LidarPolicy.ALLOWED)
    target = PoseSetpoint(2.0, 1.0, 0.8, quat_about_z(0.5))       # where the Follower wants the drone (true frame)
    odom = IDENTITY
    tau = 0.5
    truth_log = []
    pending = []
    for i in range(int(seconds / DT)):
        t = i * DT
        drift = Pose2(0.006 * t, -0.004 * t, 0.002 * t)           # truth = drift ∘ odom: 0.43 m and 6.9 deg by 60 s
        truth = drift.compose(odom)
        if i % 3 == 0:
            noisy = Pose2(truth.x + rng.normal(0, 0.02), truth.y + rng.normal(0, 0.02),
                          truth.yaw + rng.normal(0, math.radians(0.5)))
            pending.append((t + 0.05, fix(t, noisy)))               # the localiser's latency
        while pending and pending[0][0] <= t:
            fusion.add_lidar(pending.pop(0)[1])
        fusion.add_vo(vo(t, odom))
        sp = to_odom_pose(target, fusion.correction_applied) if corrected else target
        sp_yaw = quat_yaw(sp.q)
        a = DT / tau
        odom = Pose2(odom.x + a * (sp.x - odom.x), odom.y + a * (sp.y - odom.y),
                     wrap_angle(odom.yaw + a * wrap_angle(sp_yaw - odom.yaw)))
        truth_log.append(truth)
    return truth_log, target, fusion


def test_corrected_commands_hold_the_true_target_while_px4_drifts():
    truth_c, target, fusion = fly_closed_loop(corrected=True)
    truth_u, _, _ = fly_closed_loop(corrected=False)
    tail = slice(-int(10.0 / DT), None)                           # the last 10 s
    err_c = max(math.hypot(p.x - target.x, p.y - target.y) for p in truth_c[tail])
    err_u = min(math.hypot(p.x - target.x, p.y - target.y) for p in truth_u[tail])
    yaw_c = max(abs(wrap_angle(p.yaw - quat_yaw(target.q))) for p in truth_c[tail])
    assert err_u > 0.15, "uncorrected PX4 should be off by the drift ({:.3f} m)".format(err_u)
    assert err_c < 0.05, "corrected: {:.3f} m off the target (uncorrected {:.3f} m)".format(err_c, err_u)
    assert math.degrees(yaw_c) < 1.0
    assert fusion.stats.lidar_fused > 500


def test_forbidden_policy_commands_are_the_followers_own():
    fusion = PoseFusion(LidarPolicy.FORBIDDEN)
    assert fusion.correction_applied == IDENTITY
    sp = PoseSetpoint(2.0, 1.0, 0.8, quat_about_z(0.5))
    out = to_odom_pose(sp, fusion.correction_applied)
    assert (out.x, out.y) == pytest.approx((2.0, 1.0)) and out.q == pytest.approx(sp.q)


def test_the_gap_threshold_is_shorter_than_the_guards_settle_time():
    from pose_fusion.guard import DEFAULT_SETTLE_S
    assert FusionConfig().vo_gap_s < DEFAULT_SETTLE_S              # every Guard break is a gap to the fusion
