"""The course-to-odom transform and the headings the flight interface reads."""
from __future__ import annotations

import math

import pytest

from dronehacks_course_follower.frames import CourseToOdom, quaternion_yaw, yaw_quaternion

# An arbitrary measured transform: odom turned a third of a right angle from the course, its
# launch reading off zero, its origin a little above the floor
_TRANSFORM = CourseToOdom(yaw_rad=math.pi / 6, odom_origin=(0.3, -0.2), course_origin=(-1.0, 2.0),
                          origin_height_m=0.1)


def test_the_launch_reference_lands_on_its_measured_odom_reading() -> None:
    assert _TRANSFORM.point_to_odom(-1.0, 2.0) == pytest.approx((0.3, -0.2))


def test_a_course_point_goes_to_odom_and_back_unchanged() -> None:
    odom = _TRANSFORM.point_to_odom(1.7, -0.4)
    assert _TRANSFORM.point_to_course(*odom) == pytest.approx((1.7, -0.4))


def test_course_east_becomes_odom_north_under_a_quarter_turn() -> None:
    quarter = CourseToOdom(yaw_rad=math.pi / 2, odom_origin=(0.0, 0.0), course_origin=(0.0, 0.0),
                           origin_height_m=0.0)
    assert quarter.vector_to_odom(1.0, 0.0) == pytest.approx((0.0, 1.0))
    assert quarter.point_to_odom(2.0, 0.0) == pytest.approx((0.0, 2.0))
    assert quarter.yaw_to_odom(0.0) == pytest.approx(math.pi / 2)


def test_headings_and_heights_go_to_odom_and_back() -> None:
    assert _TRANSFORM.yaw_to_course(_TRANSFORM.yaw_to_odom(2.5)) == pytest.approx(2.5)
    assert _TRANSFORM.height_to_odom(0.8) == pytest.approx(0.7)
    assert _TRANSFORM.height_to_course(0.7) == pytest.approx(0.8)


def test_a_missing_heading_stays_missing_and_becomes_the_zero_quaternion() -> None:
    assert yaw_quaternion(_TRANSFORM.yaw_to_odom(math.nan)) == (0.0, 0.0, 0.0, 0.0)


def test_a_heading_survives_the_quaternion_round_trip() -> None:
    for yaw_rad in (-3.0, -1.0, 0.0, 0.5, 2.9):
        assert quaternion_yaw(*yaw_quaternion(yaw_rad)) == pytest.approx(yaw_rad)


def test_an_unmeasured_transform_is_refused() -> None:
    with pytest.raises(ValueError, match="measured"):
        CourseToOdom(yaw_rad=math.nan, odom_origin=(0.0, 0.0), course_origin=(0.0, 0.0), origin_height_m=0.0)
