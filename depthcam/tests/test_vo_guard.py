"""pose_fusion.guard: the rules that withhold cuVSLAM poses. Synthetic poses and gyro; no camera images, no ROS."""

from __future__ import annotations

import math

import numpy as np
import pytest

from pose_fusion.guard import (DEFAULT_ORIENTATION_VARIANCE_RAD2, DEFAULT_POSITION_VARIANCE_M2, VO_STATE_FAILED,
                               Guard, GuardConfig, fixed_pose_covariance)

DT = 1.0 / 30.0
LEVEL = (0.0, 0.0, 0.0, 1.0)


def yaw_q(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def walk(guard: Guard, n: int, speed: float = 0.5, yaw_rate: float = 0.0, gyro: bool = False):
    """n poses at 30 Hz along x, turning at *yaw_rate*; optionally the matching gyro. Returns the verdicts."""
    out = []
    for i in range(n):
        t = i * DT
        if gyro:
            guard.on_gyro(t, (0.0, 0.0, yaw_rate))
        out.append(guard.judge(t, (speed * t, 0.0, 0.8), yaw_q(yaw_rate * t)))
    return out


def test_a_steady_walk_passes():
    guard = Guard()
    verdicts = walk(guard, 60)
    assert all(v.publish for v in verdicts)
    assert guard.published == 60 and not guard.withheld


def test_tracker_failure_withholds_then_settles():
    guard = Guard()
    walk(guard, 10)
    guard.on_status(VO_STATE_FAILED)
    reasons = [guard.judge(i * DT, (0.5 * i * DT, 0.0, 0.8), LEVEL).reason for i in range(10, 15)]
    assert reasons == ["failed"] * 5
    guard.on_status(1)
    verdicts = [guard.judge(i * DT, (0.5 * i * DT, 0.0, 0.8), LEVEL) for i in range(15, 30)]
    settling = [v for v in verdicts if v.reason == "settling"]
    assert len(settling) in (6, 7)                                         # 0.2 s at 30 Hz (6 x 1/30 = 0.2 - eps)
    first_back = verdicts[len(settling)]
    assert first_back.publish and first_back.after_break
    assert all(v.publish and not v.after_break for v in verdicts[len(settling) + 1:])


def test_jump_and_spin():
    guard = Guard()
    walk(guard, 10)
    t = 10 * DT
    assert guard.judge(t, (0.5 * t + 0.5, 0.0, 0.8), LEVEL).reason == "jump"       # 0.5 m in one frame: 15 m/s
    # judged from the jumped pose, so carrying on from it is no jump, only settling
    assert guard.judge(t + DT, (0.5 * (t + DT) + 0.5, 0.0, 0.8), LEVEL).reason == "settling"
    guard2 = Guard()
    walk(guard2, 10)
    assert guard2.judge(10 * DT, (0.5 * 10 * DT, 0.0, 0.8), yaw_q(math.radians(30.0))).reason == "spin"


def test_gyro_disagreement_withholds_a_slow_runaway():
    """A blank view: cuVSLAM turns 20 deg/s while PX4's gyro says the drone holds its heading."""
    guard = Guard()
    verdicts = []
    for i in range(40):
        t = i * DT
        guard.on_gyro(t, (0.0, 0.0, 0.0))
        verdicts.append(guard.judge(t, (0.5 * t, 0.0, 0.8), yaw_q(math.radians(20.0) * t)))
    first = next(i for i, v in enumerate(verdicts) if v.reason == "gyro")
    assert 4 <= first <= 12                                                  # > 3 deg after about 0.15-0.4 s


def test_gyro_agreement_passes_a_real_turn():
    guard = Guard()
    assert all(v.publish for v in walk(guard, 60, yaw_rate=math.radians(60.0), gyro=True))


def test_no_gyro_no_gyro_verdict():
    guard = Guard()
    verdicts = [guard.judge(i * DT, (0.0, 0.0, 0.8), yaw_q(math.radians(20.0) * i * DT)) for i in range(40)]
    assert all(v.publish for v in verdicts)


def test_non_finite_pose_is_withheld():
    guard = Guard()
    assert guard.judge(0.0, (float("nan"), 0.0, 0.0), LEVEL).reason == "jump"


def test_the_guard_has_no_image_rule():
    assert not hasattr(Guard(), "on_image")
    assert "min_texture" not in {f for f in GuardConfig.__dataclass_fields__}


def test_fixed_covariance():
    cov = np.array(fixed_pose_covariance()).reshape(6, 6)
    assert np.diag(cov) == pytest.approx(list(DEFAULT_POSITION_VARIANCE_M2) + list(DEFAULT_ORIENTATION_VARIANCE_RAD2))
    assert np.count_nonzero(cov - np.diag(np.diag(cov))) == 0


def test_config_rejects_negative_values():
    with pytest.raises(ValueError, match="settle_s"):
        GuardConfig(settle_s=-1.0)
