"""pose_fusion.localizer: MS200K revolutions -> course-frame body pose with a covariance.

Synthetic scans from course/lidar_sim.py on the real course map (course/maps/course_v3_map.json).
PX4 is a stand-in built from the same truth, with a deliberately arbitrary heading reference.
Course positions come from the map and rules files, never typed in (AGENTS.md §3.7). No ROS,
no lidar, no PX4.
"""

from __future__ import annotations

import math
from pathlib import Path
from types import SimpleNamespace
from typing import List, Tuple

import numpy as np
import pytest

from course.lidar_sim import LidarSpec, Scene, Trajectory, simulate
from pose_fusion.course_files import (START_HEADING_NORTH, launch_mark_from_files, load_course_map, load_rules,
                                      start_marking_centre, wall_and_net_tops)
from pose_fusion.frames import CourseToTracker
from pose_fusion.localizer import (LidarLocalizer, LocalizerConfig, MapSlices, Px4Buffer, Px4Sample,
                                   course_velocity, match_covariance, px4_sample_from_odometry)
from pose_fusion.ros_io import fill_pose_with_covariance, lidar_pose_to_fix, s_to_stamp
from pose_fusion.se2 import wrap_angle

REPO_ROOT = Path(__file__).resolve().parents[1]
MAP = REPO_ROOT / "course" / "maps" / "course_v3_map.json"
RULES = REPO_ROOT / "course" / "maps" / "course_v3_rules.json"
MOUNT = (0.0, 0.0, 0.10)          # scan plane 10 cm above the body centre (the hardware guide's default)
REST = 0.07                       # body centre above the floor when landed (synthetic fixture)
PX4_HEADING_REF = 0.7             # PX4's yaw = this - course yaw: an arbitrary reference, as with pose_frame FRD


@pytest.fixture(scope="module")
def cmap():
    return load_course_map(MAP)


@pytest.fixture(scope="module")
def start(cmap) -> Tuple[float, float]:
    return start_marking_centre(cmap, load_rules(RULES))


@pytest.fixture(scope="module")
def slices(cmap) -> MapSlices:
    wall_top, net_top = wall_and_net_tops(cmap)
    return MapSlices(cmap, (0.2, 0.9), wall_top, net_top)       # on the ground, and at flight height


def config(**kw) -> LocalizerConfig:
    return LocalizerConfig(mount_xyz=MOUNT, rest_height_m=REST, slice_heights=(0.2, 0.9), **kw)


# ---------------------------------------------------------------------------------------------- PX4 stand-in

def quat_wxyz_from_frd(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """ZYX Euler (FRD body in NED) -> PX4 quaternion [w, x, y, z]."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)


def px4_from_truth(t: float, pos: np.ndarray, rpy_flu: np.ndarray, vel_course: np.ndarray,
                   yaw_rate_course: float) -> Px4Sample:
    """What PX4 would report for the course-frame truth, in its own NED frame with an arbitrary heading."""
    roll, pitch_flu, yaw_c = (float(v) for v in rpy_flu)
    yaw_ned = PX4_HEADING_REF - yaw_c
    fwd = math.cos(yaw_c) * vel_course[0] + math.sin(yaw_c) * vel_course[1]
    left = -math.sin(yaw_c) * vel_course[0] + math.cos(yaw_c) * vel_course[1]
    right = -left
    c, s = math.cos(yaw_ned), math.sin(yaw_ned)
    vel_ned = (c * fwd - s * right, s * fwd + c * right, 0.0)
    q = quat_wxyz_from_frd(roll, -pitch_flu, yaw_ned)
    z_ned = -(float(pos[2]) - REST)                       # EKF origin: the body resting on the floor
    return px4_sample_from_odometry(t, q, (0.0, 0.0, z_ned), vel_ned, (0.0, 0.0, -yaw_rate_course))


def corridor_flight(start: Tuple[float, float]) -> Trajectory:
    """Rest on the launch mark facing north for 1 s, climb to 0.8 m, then fly 1.2 m up the first corridor."""
    t = np.arange(0.0, 6.0, 0.005)
    climb = np.clip((t - 1.0) / 1.0, 0.0, 1.0)
    north = np.clip((t - 2.5) / 2.4, 0.0, 1.0) * 1.2
    pos = np.stack([np.full_like(t, start[0]), start[1] + north, REST + climb * (0.80 - REST)], axis=1)
    pitch = np.where((t > 2.5) & (t < 2.9), -math.radians(3.0), 0.0)   # nose down while it accelerates
    rpy = np.stack([np.zeros_like(t), pitch, np.full_like(t, START_HEADING_NORTH)], axis=1)
    return Trajectory(t, pos, rpy)


def fly(localizer: LidarLocalizer, traj: Trajectory, cmap, seconds: float, with_px4: bool = True, seed: int = 3):
    scans = simulate(traj, Scene.from_map(cmap), LidarSpec(mount=MOUNT), 0.0, seconds, seed=seed)
    buf = Px4Buffer()
    px4_t = np.arange(0.0, seconds + 0.5, 0.02)
    pos, rpy = traj.at(px4_t)
    vel = np.gradient(pos[:, :2], px4_t, axis=0)
    rate = np.gradient(np.unwrap(rpy[:, 2]), px4_t)
    out = []
    k = 0
    for scan in scans:
        while with_px4 and k < len(px4_t) and px4_t[k] <= scan.t_end:
            buf.add(px4_from_truth(float(px4_t[k]), pos[k], rpy[k], vel[k], float(rate[k])))
            k += 1
        out.append((localizer.update(scan.t, scan.range, scan.angle, buf), scan))
    return out


# ---------------------------------------------------------------------------------------------- course files

def test_launch_mark_comes_from_the_rules_start_marking(cmap):
    rules = load_rules(RULES)
    mark = launch_mark_from_files(MAP, RULES)
    feature = cmap.feature(rules["start"]["marking"])
    assert feature["kind"] == "marking"
    assert (mark.east, mark.north) == pytest.approx(tuple(feature["centre"][:2]))


def test_wall_tops_sit_below_net_tops(cmap):
    wall_top, net_top = wall_and_net_tops(cmap)
    assert 0.0 < wall_top < net_top


# ---------------------------------------------------------------------------------------------- PX4 conversions

def test_px4_attitude_becomes_flu_roll_and_pitch():
    roll, pitch, yaw = math.radians(5.0), math.radians(3.0), math.radians(30.0)   # FRD: pitch up is positive
    s = px4_sample_from_odometry(1.0, quat_wxyz_from_frd(roll, pitch, yaw), (0.0, 0.0, -0.5), (1.0, 0.0, 0.0),
                                 (0.0, 0.0, 0.2))
    assert (s.roll_flu, s.pitch_flu, s.yaw_ned) == pytest.approx((roll, -pitch, yaw))
    assert s.yaw_rate_flu == pytest.approx(-0.2)
    assert s.z_ned == -0.5


@pytest.mark.parametrize("ref", [0.0, 0.7, -2.5, math.pi])
def test_course_velocity_does_not_depend_on_px4s_heading_reference(ref):
    global PX4_HEADING_REF
    saved, PX4_HEADING_REF = PX4_HEADING_REF, ref
    try:
        yaw_c = math.radians(100.0)
        v_course = np.array([0.3, -0.8])
        s = px4_from_truth(0.0, np.array([0.0, 0.0, REST]), np.array([0.0, 0.0, yaw_c]), v_course, 0.0)
        assert course_velocity(s, yaw_c) == pytest.approx(v_course)
    finally:
        PX4_HEADING_REF = saved


def test_no_px4_velocity_means_the_tracker_uses_its_own():
    s = px4_sample_from_odometry(0.0, (1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0), None, (0.0, 0.0, 0.0))
    assert course_velocity(s, 0.3) is None


def test_body_height_is_rest_height_plus_the_climb():
    buf = Px4Buffer()
    level = (1.0, 0.0, 0.0, 0.0)
    buf.add(px4_sample_from_odometry(0.0, level, (0.0, 0.0, 0.02), (0, 0, 0), (0, 0, 0)))   # resting: the floor
    buf.add(px4_sample_from_odometry(1.0, level, (0.0, 0.0, -0.71), (0, 0, 0), (0, 0, 0)))
    assert buf.body_height(REST) == pytest.approx(REST + 0.73)


# ---------------------------------------------------------------------------------------------- covariance

def test_covariance_is_large_where_the_scan_sees_nothing():
    info = np.diag([1.0e4, 1.0e-9, 1.0e3])                    # y: a featureless corridor axis
    cov = match_covariance({"information": info, "sigma2_m2": 1.0e-4}, scale=1.0, max_variance=50.0)
    assert np.diag(cov) == pytest.approx([1.0e-8, 50.0, 1.0e-7])


def test_covariance_follows_a_rotated_degenerate_axis():
    a = math.radians(30.0)
    r = np.array([[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]])
    info = r @ np.diag([1.0e4, 1.0e-9, 1.0e3]) @ r.T
    cov = match_covariance({"information": info, "sigma2_m2": 1.0e-4}, scale=1.0, max_variance=50.0)
    eigval, eigvec = np.linalg.eigh(cov)
    weak = eigvec[:, int(np.argmax(eigval))]
    assert abs(abs(weak @ r[:, 1]) - 1.0) < 1e-6                 # the big variance lies along the weak axis


def test_no_information_no_covariance():
    assert match_covariance({}, 1.0, 1.0) is None
    assert match_covariance({"information": np.full((3, 3), np.nan), "sigma2_m2": 1.0}, 1.0, 1.0) is None


# ---------------------------------------------------------------------------------------------- the localiser

def test_localizer_follows_a_corridor_flight(cmap, start, slices):
    traj = corridor_flight(start)
    loc = LidarLocalizer(cmap, start, config(), START_HEADING_NORTH, slices=slices)
    results = fly(loc, traj, cmap, 5.5)

    assert [r.publish for r, _ in results[:2]] == [False, False]           # 3 good matches before the first pose
    assert results[2][0].publish
    errors, yaw_errors, sigmas = [], [], []
    for r, scan in results:
        if not r.publish:
            continue
        true_pos, true_rpy = traj.at(np.array([r.t]))
        errors.append(math.hypot(r.x - true_pos[0, 0], r.y - true_pos[0, 1]))
        yaw_errors.append(abs(wrap_angle(r.yaw - true_rpy[0, 2])))
        assert np.all(np.isfinite(r.cov)) and np.all(np.linalg.eigvalsh(r.cov) > 0.0)
        sigmas.append(math.sqrt(max(r.cov[0, 0], r.cov[1, 1])))
    assert loc.counts["lost"] == 0
    assert len(errors) >= 45
    assert max(errors) < 0.05, "worst position error {:.3f} m".format(max(errors))
    assert math.degrees(max(yaw_errors)) < 1.5
    assert 0.001 < float(np.median(sigmas)) < 0.5


def test_no_px4_attitude_no_pose(cmap, start, slices):
    loc = LidarLocalizer(cmap, start, config(), START_HEADING_NORTH, slices=slices)
    results = fly(loc, corridor_flight(start), cmap, 1.0, with_px4=False)
    assert not any(r.publish for r, _ in results)
    assert all(r.reason == "no fresh PX4 attitude" for r, _ in results)


def test_a_blank_revolution_is_lost_not_published(cmap, start, slices):
    loc = LidarLocalizer(cmap, start, config(), START_HEADING_NORTH, slices=slices)
    results = fly(loc, corridor_flight(start), cmap, 0.8)
    assert results[-1][0].publish
    last_scan = results[-1][1]
    buf = Px4Buffer()
    buf.add(px4_from_truth(float(last_scan.t_end), np.array([start[0], start[1], REST]),
                           np.array([0.0, 0.0, START_HEADING_NORTH]), np.zeros(2), 0.0))
    blank = loc.update(last_scan.t + 0.1, np.full(len(last_scan.t), np.nan), last_scan.angle, buf)
    assert not blank.publish and blank.reason.startswith("lost")
    assert loc.counts["lost"] == 1


def test_resting_pose_reaches_the_fusion_as_the_tracker_origin(cmap, start, slices):
    """Localiser -> PoseWithCovarianceStamped -> pose_fusion's course-to-tracker conversion.

    On the launch mark, facing north, the drone is the tracker frame's origin (contracts/frames.md).
    """
    loc = LidarLocalizer(cmap, start, config(), START_HEADING_NORTH, slices=slices)
    results = fly(loc, corridor_flight(start), cmap, 0.8)
    r = results[-1][0]
    assert r.publish
    msg = SimpleNamespace(header=SimpleNamespace(frame_id="course", stamp=SimpleNamespace()),
                          pose=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(),
                                                                    orientation=SimpleNamespace()), covariance=None))
    msg.header.stamp.sec, msg.header.stamp.nanosec = s_to_stamp(r.t)
    fill_pose_with_covariance(msg, r.x, r.y, r.z_body, r.yaw, r.cov)
    tf = CourseToTracker(launch_mark_from_files(MAP, RULES), START_HEADING_NORTH, 0.0, 0.0)  # measured identity
    fix = lidar_pose_to_fix(msg, "course", tf)
    assert fix.t == pytest.approx(r.t, abs=1e-6)
    assert math.hypot(fix.pose.x, fix.pose.y) < 0.03
    assert abs(math.degrees(fix.pose.yaw)) < 1.0
    assert np.all(np.diag(fix.cov) > 0.0)


def test_stamp_rounding_carries_into_seconds():
    assert s_to_stamp(12.9999999999) == (13, 0)
    assert s_to_stamp(1.25) == (1, 250_000_000)


def test_mount_and_rest_height_must_be_measured():
    with pytest.raises(ValueError, match="mount"):
        LocalizerConfig(mount_xyz=(0.0, 0.0, float("nan")), rest_height_m=REST)
    with pytest.raises(ValueError, match="rest_height_m"):
        LocalizerConfig(mount_xyz=MOUNT, rest_height_m=float("nan"))
