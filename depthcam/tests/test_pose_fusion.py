"""pose_fusion: lidar-corrected visual odometry. Pure numpy: no ROS, no camera, no GPU.

Trajectories are synthetic. The launch marks below are made-up fixtures, not course values
(AGENTS.md §3.7 keeps course geometry out of code).
"""

from __future__ import annotations

import math
from types import SimpleNamespace
from typing import Callable, List, Optional, Tuple

import numpy as np
import pytest

from pose_fusion.frames import (LaunchMark, course_cov_to_tracker, course_pose_to_tracker, course_to_frd_xy,
                                frd_to_flu_xy)
from pose_fusion.fusion import (FusedPose, FusionConfig, LidarFix, PolicyViolation, PoseFusion, VoSample,
                                apply_correction, floor_covariance)
from pose_fusion.policy import LidarPolicy, parse_lidar_policy
from pose_fusion.ros_io import LidarInputError, fill_pose, lidar_pose_to_fix, odometry_to_vo_sample
from pose_fusion.se2 import IDENTITY, Pose2, interpolate, quat_about_z, quat_multiply, quat_yaw, wrap_angle

VO_DT = 1.0 / 30.0
LIDAR_DT = 0.1
HEIGHT = 0.73
FIXTURE_MARK = LaunchMark(east=1.25, north=-0.5)     # synthetic, not the course's launch mark


# ---------------------------------------------------------------------------------------------- helpers

def vo_sample(t: float, pose: Pose2, z: float = HEIGHT) -> VoSample:
    return VoSample(t, pose.x, pose.y, z, quat_about_z(pose.yaw))


def lidar_fix(t: float, pose: Pose2, sigma_pos: float = 0.03, sigma_yaw: float = math.radians(1.0)) -> LidarFix:
    return LidarFix(t, pose, np.diag([sigma_pos ** 2, sigma_pos ** 2, sigma_yaw ** 2]))


def figure_eight(t: float) -> Pose2:
    """A smooth loop (about 0.3-0.5 m/s), heading along the velocity. Speed never reaches zero."""
    w = 0.12
    x, y = 3.0 * math.sin(w * t), 1.5 * math.sin(2.0 * w * t)
    dx, dy = 3.0 * w * math.cos(w * t), 3.0 * w * math.cos(2.0 * w * t)
    return Pose2(x, y, math.atan2(dy, dx))


def drifting_vo(truth: List[Pose2], scale_error: float, yaw_bias_per_s: float) -> List[Pose2]:
    """VO built from truth's body-frame increments, with a scale error and a gyro-like yaw bias."""
    out = [truth[0]]
    for prev, cur in zip(truth, truth[1:]):
        inc = prev.inverse().compose(cur)
        inc_err = Pose2(inc.x * (1.0 + scale_error), inc.y * (1.0 + scale_error), inc.yaw + yaw_bias_per_s * VO_DT)
        out.append(out[-1].compose(inc_err))
    return out


def position_error(a, b) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def run_with_lidar(fusion: PoseFusion, times: List[float], vo: List[Pose2], truth_at: Callable[[float], Pose2],
                   latency_s: float = 0.05, noise_pos: float = 0.03, noise_yaw: float = math.radians(1.0),
                   seed: int = 7) -> List[FusedPose]:
    """Feed VO at 30 Hz and lidar fixes at 10 Hz (truth plus noise), each delivered *latency_s* late."""
    rng = np.random.default_rng(seed)
    fix_times = list(np.arange(times[0] + 0.013, times[-1], LIDAR_DT))
    outputs: List[FusedPose] = []
    j = 0
    for t, pose in zip(times, vo):
        while j < len(fix_times) and fix_times[j] + latency_s <= t:
            tf = float(fix_times[j])
            truth = truth_at(tf)
            noisy = Pose2(truth.x + rng.normal(0.0, noise_pos), truth.y + rng.normal(0.0, noise_pos),
                          wrap_angle(truth.yaw + rng.normal(0.0, noise_yaw)))
            fusion.add_lidar(lidar_fix(tf, noisy, noise_pos, noise_yaw))
            j += 1
        out = fusion.add_vo(vo_sample(t, pose))
        assert out is not None
        outputs.append(out)
    return outputs


def euler_roll_pitch(q) -> Tuple[float, float]:
    x, y, z, w = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    return roll, pitch


# ---------------------------------------------------------------------------------------------- se2

def test_compose_inverse_round_trip():
    a = Pose2(1.2, -0.7, 0.9)
    b = Pose2(-0.3, 2.0, -2.5)
    ab = a.compose(b)
    back = a.inverse().compose(ab)
    assert back.x == pytest.approx(b.x) and back.y == pytest.approx(b.y)
    assert wrap_angle(back.yaw - b.yaw) == pytest.approx(0.0, abs=1e-12)
    ident = a.compose(a.inverse())
    assert ident.as_tuple() == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)


def test_interpolate_takes_the_short_way_round():
    a, b = Pose2(0.0, 0.0, math.radians(170)), Pose2(1.0, 2.0, math.radians(-170))
    mid = interpolate(a, b, 0.5)
    assert (mid.x, mid.y) == pytest.approx((0.5, 1.0))
    assert abs(abs(mid.yaw) - math.pi) < 1e-9


def test_quat_yaw_round_trip():
    for yaw in (-3.0, -1.0, 0.0, 0.5, 3.1):
        assert quat_yaw(quat_about_z(yaw)) == pytest.approx(yaw)


# ---------------------------------------------------------------------------------------------- policy

@pytest.mark.parametrize("value", ["allowed", "forbidden"])
def test_policy_accepts_exact_values(value):
    assert parse_lidar_policy(value).value == value


@pytest.mark.parametrize("value", ["", None, "Allowed", " allowed", "yes", "true", 1, "disallowed"])
def test_policy_has_no_default_and_no_guessing(value):
    with pytest.raises(ValueError, match="no default"):
        parse_lidar_policy(value)


# ---------------------------------------------------------------------------------------------- frames

def test_frd_follows_the_contract_axis_map():
    # contracts/frames.md step 2: world north -> PX4 +x (forward), world east -> PX4 +y (right)
    forward, right = course_to_frd_xy(FIXTURE_MARK.east + 2.0, FIXTURE_MARK.north + 3.0, FIXTURE_MARK)
    assert (forward, right) == pytest.approx((3.0, 2.0))
    assert frd_to_flu_xy(forward, right) == pytest.approx((3.0, -2.0))


@pytest.mark.parametrize("d_east, d_north, heading, expected", [
    (0.0, 3.0, math.pi / 2, (3.0, 0.0, 0.0)),            # 3 m north of the mark, facing north
    (2.0, 0.0, 0.0, (0.0, -2.0, -math.pi / 2)),          # 2 m east, facing east: right of start
    (-1.0, 0.0, math.pi, (0.0, 1.0, math.pi / 2)),       # 1 m west, facing west: left of start
])
def test_course_pose_to_tracker(d_east, d_north, heading, expected):
    pose = course_pose_to_tracker(FIXTURE_MARK.east + d_east, FIXTURE_MARK.north + d_north, heading, FIXTURE_MARK)
    assert (pose.x, pose.y) == pytest.approx(expected[:2])
    assert wrap_angle(pose.yaw - expected[2]) == pytest.approx(0.0, abs=1e-12)


def test_course_covariance_swaps_axes():
    cov = np.diag([0.1 ** 2, 0.3 ** 2, 0.02 ** 2])          # east, north, heading
    out = course_cov_to_tracker(cov)
    assert np.sqrt(np.diag(out)) == pytest.approx([0.3, 0.1, 0.02])   # x = north, y = west


def test_launch_mark_must_be_given():
    with pytest.raises(ValueError, match="contracts/course.md"):
        LaunchMark(float("nan"), 0.0)


# ---------------------------------------------------------------------------------------------- forbidden policy

def test_forbidden_is_an_exact_pass_through_and_refuses_lidar():
    fusion = PoseFusion(LidarPolicy.FORBIDDEN)
    q = (0.01, -0.02, 0.3, math.sqrt(1.0 - 0.01 ** 2 - 0.02 ** 2 - 0.3 ** 2))
    sample = VoSample(12.5, 1.234567, -7.654321, 0.73, q)
    out = fusion.add_vo(sample)
    assert (out.t, out.x, out.y, out.z, out.q) == (sample.t, sample.x, sample.y, sample.z, sample.q)
    with pytest.raises(PolicyViolation):
        fusion.add_lidar(lidar_fix(12.5, Pose2(0.0, 0.0, 0.0)))
    assert fusion.stats.lidar_refused == 1 and fusion.stats.lidar_received == 0


def test_allowed_without_any_lidar_is_also_a_pass_through():
    fusion = PoseFusion(LidarPolicy.ALLOWED)
    times = [i * VO_DT for i in range(300)]
    for t in times:
        pose = figure_eight(t)
        out = fusion.add_vo(vo_sample(t, pose))
        assert (out.x, out.y) == (pose.x, pose.y)


def test_policy_must_be_the_enum():
    with pytest.raises(TypeError):
        PoseFusion("allowed")  # type: ignore[arg-type]


# ---------------------------------------------------------------------------------------------- drift correction

def test_lidar_removes_vo_drift_on_a_figure_eight():
    times = [i * VO_DT for i in range(int(60.0 / VO_DT))]
    truth = [figure_eight(t) for t in times]
    vo = drifting_vo(truth, scale_error=0.03, yaw_bias_per_s=0.004)
    fusion = PoseFusion(LidarPolicy.ALLOWED)
    outputs = run_with_lidar(fusion, times, vo, figure_eight)

    tail = range(len(times) - int(20.0 / VO_DT), len(times))          # the last 20 s
    vo_rms = math.sqrt(np.mean([position_error(vo[i], truth[i]) ** 2 for i in tail]))
    fused_rms = math.sqrt(np.mean([position_error(outputs[i], truth[i]) ** 2 for i in tail]))
    yaw_rms = math.sqrt(np.mean([wrap_angle(quat_yaw(outputs[i].q) - truth[i].yaw) ** 2 for i in tail]))

    assert vo_rms > 0.4, "the synthetic VO should drift visibly ({:.3f} m)".format(vo_rms)
    assert fused_rms < 0.08, "fused rms {:.3f} m (VO {:.3f} m)".format(fused_rms, vo_rms)
    assert fused_rms < 0.2 * vo_rms
    assert math.degrees(yaw_rms) < 1.5
    assert fusion.stats.lidar_fused > 500 and fusion.stats.lidar_gated < 10


def test_correction_never_moves_the_output_faster_than_the_limits():
    # The drone sits on the launch mark before take-off (VO at its origin); the lidar says it was
    # placed 0.2 m forward, 0.1 m right and turned 8 deg. The output must walk there, not jump.
    cfg = FusionConfig()
    still = Pose2(0.0, 0.0, 0.0)
    lidar_says = Pose2(0.2, -0.1, math.radians(8.0))
    fusion = PoseFusion(LidarPolicy.ALLOWED, cfg)
    previous: Optional[FusedPose] = None
    for i in range(int(15.0 / VO_DT)):
        t = i * VO_DT
        if i % 3 == 0 and i > 0:
            fusion.add_lidar(lidar_fix(t - VO_DT, lidar_says))
        out = fusion.add_vo(vo_sample(t, still))
        if previous is not None:
            step = math.hypot(out.x - previous.x, out.y - previous.y)
            turn = abs(wrap_angle(quat_yaw(out.q) - quat_yaw(previous.q)))
            assert step <= cfg.max_correction_speed_mps * VO_DT + 1e-9
            assert turn <= cfg.max_correction_yaw_rate_rps * VO_DT + 1e-9
        previous = out
    assert position_error(previous, lidar_says) < 0.02
    assert abs(math.degrees(wrap_angle(quat_yaw(previous.q) - lidar_says.yaw))) < 0.5


def test_outlier_fix_is_gated_and_changes_nothing():
    fusion = PoseFusion(LidarPolicy.ALLOWED)
    still = Pose2(0.5, -0.2, 0.0)
    for i in range(90):
        t = i * VO_DT
        if i % 3 == 0 and i > 0:
            fusion.add_lidar(lidar_fix(t - VO_DT, still))
        fusion.add_vo(vo_sample(t, still))
    before = fusion.correction_estimate
    t = 90 * VO_DT
    fusion.add_lidar(lidar_fix(t - VO_DT, Pose2(3.5, -0.2, 0.0)))     # 3 m off
    out = fusion.add_vo(vo_sample(t, still))
    assert fusion.stats.lidar_gated == 1 and fusion.stats.consecutive_gated == 1
    assert fusion.correction_estimate == before
    assert position_error(out, still) < 1e-6


def test_fix_between_vo_samples_is_compared_with_interpolated_vo():
    fusion = PoseFusion(LidarPolicy.ALLOWED)
    speed = 1.0
    for i in range(10):
        t = i * VO_DT
        fusion.add_vo(vo_sample(t, Pose2(speed * t, 0.0, 0.0)))
    tf = 7.5 * VO_DT                                                   # halfway between two VO samples
    fusion.add_lidar(lidar_fix(tf, Pose2(speed * tf, 0.0, 0.0)))
    assert fusion.stats.lidar_fused == 1
    assert fusion.status()["last_innovation"] == pytest.approx([0.0, 0.0, 0.0], abs=1e-9)


def test_fix_slightly_ahead_waits_for_vo_and_far_ahead_is_refused():
    fusion = PoseFusion(LidarPolicy.ALLOWED)
    for i in range(5):
        fusion.add_vo(vo_sample(i * VO_DT, Pose2(0.0, 0.0, 0.0)))
    newest = 4 * VO_DT
    fusion.add_lidar(lidar_fix(newest + 0.02, Pose2(0.0, 0.0, 0.0)))   # within max_lidar_wait_s
    assert fusion.stats.lidar_fused == 0 and fusion.status()["pending_fixes"] == 1
    fusion.add_vo(vo_sample(5 * VO_DT, Pose2(0.0, 0.0, 0.0)))
    assert fusion.stats.lidar_fused == 1 and fusion.status()["pending_fixes"] == 0
    fusion.add_lidar(lidar_fix(5 * VO_DT + 1.0, Pose2(0.0, 0.0, 0.0)))  # a clock mismatch
    assert fusion.stats.lidar_future == 1


def test_fix_older_than_the_vo_buffer_is_stale():
    cfg = FusionConfig(vo_buffer_s=0.5)
    fusion = PoseFusion(LidarPolicy.ALLOWED, cfg)
    for i in range(60):                                                # 2 s of VO
        fusion.add_vo(vo_sample(i * VO_DT, Pose2(0.0, 0.0, 0.0)))
    fusion.add_lidar(lidar_fix(0.1, Pose2(0.0, 0.0, 0.0)))
    assert fusion.stats.lidar_stale == 1 and fusion.stats.lidar_fused == 0


def test_out_of_order_vo_is_not_published():
    fusion = PoseFusion(LidarPolicy.ALLOWED)
    fusion.add_vo(vo_sample(1.0, IDENTITY))
    assert fusion.add_vo(vo_sample(1.0, IDENTITY)) is None
    assert fusion.add_vo(vo_sample(0.9, IDENTITY)) is None
    assert fusion.stats.vo_out_of_order == 2


# ---------------------------------------------------------------------------------------------- VO gaps

def _converge_then_gap(reanchor: bool):
    """VO is 0.2 m to the left of the truth; the lidar is the truth. Converge, go blind for 1 s, resume.

    After the gap the Guard resumes from PX4's estimate, which is this stage's output (about the
    truth), so the resumed VO is the truth.
    """
    cfg = FusionConfig(reanchor_on_gap=reanchor)
    fusion = PoseFusion(LidarPolicy.ALLOWED, cfg)
    truth = lambda t: Pose2(0.3 * t, 0.0, 0.0)                         # noqa: E731
    vo_offset = Pose2(0.0, 0.2, 0.0)
    times = [i * VO_DT for i in range(int(10.0 / VO_DT))]
    vo = [vo_offset.compose(truth(t)) for t in times]
    before = run_with_lidar(fusion, times, vo, truth, noise_pos=0.0, noise_yaw=0.0)
    gap_start = times[-1]
    resumed_times = [gap_start + 1.0 + i * VO_DT for i in range(int(5.0 / VO_DT))]
    after = [fusion.add_vo(vo_sample(t, truth(t))) for t in resumed_times]
    return fusion, truth, before, after, resumed_times


def test_gap_reanchor_resumes_without_a_jump():
    fusion, truth, before, after, times = _converge_then_gap(reanchor=True)
    assert position_error(before[-1], truth(before[-1].t)) < 0.02      # converged before the gap
    assert fusion.stats.vo_gaps == 1
    assert after[0].correction == IDENTITY                              # reset at the gap
    assert position_error(after[0], truth(times[0])) < 1e-9            # so the first output is the resumed VO
    worst = max(position_error(out, truth(t)) for out, t in zip(after, times))
    assert worst < 0.03, "worst error after the gap {:.3f} m".format(worst)


def test_gap_without_reanchor_reapplies_the_old_correction():
    # Documents why reanchor_on_gap matters: if the Guard really resumed from PX4's estimate,
    # keeping the correction would shift the output by the whole old correction.
    _, truth, _, after, times = _converge_then_gap(reanchor=False)
    assert position_error(after[0], truth(times[0])) == pytest.approx(0.2, abs=0.03)


# ---------------------------------------------------------------------------------------------- 3D pass-through

def test_correction_keeps_height_roll_and_pitch():
    roll, pitch, yaw = math.radians(5.0), math.radians(-3.0), math.radians(20.0)
    q_roll = (math.sin(roll / 2), 0.0, 0.0, math.cos(roll / 2))
    q_pitch = (0.0, math.sin(pitch / 2), 0.0, math.cos(pitch / 2))
    q = quat_multiply(quat_about_z(yaw), quat_multiply(q_pitch, q_roll))  # ZYX
    sample = VoSample(3.0, 1.0, 2.0, 0.73, q)
    out = apply_correction(sample, Pose2(0.1, -0.2, math.radians(10.0)))
    assert out.z == sample.z
    assert euler_roll_pitch(out.q) == pytest.approx((roll, pitch), abs=1e-9)
    assert math.degrees(quat_yaw(out.q)) == pytest.approx(30.0)
    c, s = math.cos(math.radians(10.0)), math.sin(math.radians(10.0))
    assert (out.x, out.y) == pytest.approx((c * 1.0 - s * 2.0 + 0.1, s * 1.0 + c * 2.0 - 0.2))


def test_identity_correction_is_bit_exact():
    q = (0.1, 0.2, 0.3, math.sqrt(1.0 - 0.14))
    sample = VoSample(1.0, -3.3, 4.4, 0.5, q)
    out = apply_correction(sample, IDENTITY)
    assert (out.x, out.y, out.z, out.q) == (sample.x, sample.y, sample.z, sample.q)


def test_floor_covariance_raises_only_small_variances():
    cov = np.diag([0.001 ** 2, 0.5 ** 2, 1e-8])
    out = floor_covariance(cov, 0.02, math.radians(0.5))
    assert np.sqrt(np.diag(out)) == pytest.approx([0.02, 0.5, math.radians(0.5)])


def test_config_rejects_non_positive_values():
    with pytest.raises(ValueError, match="max_correction_speed_mps"):
        FusionConfig(max_correction_speed_mps=0.0)


# ---------------------------------------------------------------------------------------------- ROS message conversion

def _pose_msg(frame_id: str, sec: int, nanosec: int, x: float, y: float, heading: float, diag=(0.01, 0.04, 1, 1, 1, 4e-4)):
    q = quat_about_z(heading)
    cov = [0.0] * 36
    for i, v in enumerate(diag):
        cov[i * 7] = v
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame_id, stamp=SimpleNamespace(sec=sec, nanosec=nanosec)),
        pose=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y, z=0.0),
                                                  orientation=SimpleNamespace(x=q[0], y=q[1], z=q[2], w=q[3])),
                             covariance=cov))


def test_lidar_message_becomes_a_tracker_frame_fix():
    msg = _pose_msg("course", 12, 500_000_000, FIXTURE_MARK.east, FIXTURE_MARK.north + 2.0, math.pi / 2)
    fix = lidar_pose_to_fix(msg, "course", FIXTURE_MARK)
    assert fix.t == pytest.approx(12.5)
    assert fix.pose.as_tuple() == pytest.approx((2.0, 0.0, 0.0), abs=1e-12)
    assert np.diag(fix.cov) == pytest.approx([0.04, 0.01, 4e-4])        # north variance -> x


def test_lidar_message_in_the_wrong_frame_is_refused():
    with pytest.raises(LidarInputError, match="frame"):
        lidar_pose_to_fix(_pose_msg("odom", 1, 0, 0.0, 0.0, 0.0), "course", FIXTURE_MARK)


def test_lidar_message_with_unknown_covariance_is_refused():
    with pytest.raises(LidarInputError, match="covariance"):
        lidar_pose_to_fix(_pose_msg("course", 1, 0, 0.0, 0.0, 0.0, diag=(0.01, 0.01, 1, 1, 1, 0.0)), "course",
                          FIXTURE_MARK)


def test_odometry_round_trip():
    q = quat_about_z(0.4)
    odom = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=3, nanosec=250_000_000)),
                           pose=SimpleNamespace(pose=SimpleNamespace(
                               position=SimpleNamespace(x=1.0, y=2.0, z=0.7),
                               orientation=SimpleNamespace(x=q[0], y=q[1], z=q[2], w=q[3]))))
    sample = odometry_to_vo_sample(odom)
    assert (sample.t, sample.x, sample.y, sample.z) == pytest.approx((3.25, 1.0, 2.0, 0.7))
    target = SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                             orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))
    fill_pose(target, apply_correction(sample, IDENTITY))
    assert (target.position.x, target.position.y, target.position.z) == (1.0, 2.0, 0.7)
    assert (target.orientation.z, target.orientation.w) == pytest.approx((q[2], q[3]))
