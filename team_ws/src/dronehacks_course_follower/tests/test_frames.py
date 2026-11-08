"""The course-to-odom transform and the headings the flight interface reads."""
from __future__ import annotations

import math

import pytest

from dronehacks_course_follower.frames import (REST_WINDOW_S, CourseToOdom, NotAtRest, RestOnMark, quaternion_yaw,
                                               yaw_quaternion)

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


# The published launch mark, facing north (a quarter turn counter-clockwise from east)
_MARK = (-2.5146, 0.9144)
_NORTH_RAD = math.pi / 2


def _rest(samples: list[tuple[float, float, float, float]]) -> RestOnMark:
    """A rest on the published mark fed `(time_s, x, y, yaw_rad)` odometry samples."""
    rest = RestOnMark(launch=_MARK, launch_heading_rad=_NORTH_RAD, origin_height_m=0.0)
    for sample in samples:
        rest.add(*sample)

    return rest


def test_a_drone_resting_on_the_mark_places_the_course_by_its_odometry() -> None:
    # The tracker started with the drone turned 3 degrees, so its nose reads 3 degrees in odom
    odom_yaw_rad = math.radians(3.0)
    transform = _rest([(index * 0.05, 0.1, -0.05, odom_yaw_rad) for index in range(40)]).transform()

    assert transform.point_to_odom(*_MARK) == pytest.approx((0.1, -0.05))
    assert transform.yaw_to_odom(_NORTH_RAD) == pytest.approx(odom_yaw_rad)

    # One metre north on the course is one metre along the drone's own nose in odom
    north = transform.point_to_odom(_MARK[0], _MARK[1] + 1.0)
    assert north == pytest.approx((0.1 + math.cos(odom_yaw_rad), -0.05 + math.sin(odom_yaw_rad)))


def test_a_drone_moving_or_turning_on_the_mark_is_not_at_rest() -> None:
    sliding = _rest([(index * 0.05, 0.001 * index, 0.0, 0.0) for index in range(40)])
    with pytest.raises(NotAtRest, match="moved"):
        sliding.transform()

    turning = _rest([(index * 0.05, 0.0, 0.0, math.radians(0.1 * index)) for index in range(40)])
    with pytest.raises(NotAtRest, match="turned"):
        turning.transform()


def test_too_little_odometry_is_not_a_rest_and_old_samples_are_forgotten() -> None:
    with pytest.raises(NotAtRest, match="samples"):
        _rest([(0.0, 0.0, 0.0, 0.0)]).transform()

    # A drone carried to the mark then left still: only the still window counts
    carried = [(index * 0.05, 3.0 - 0.05 * index, 0.0, 0.0) for index in range(60)]
    still = [(3.0 + index * 0.05, 0.0, 0.0, 0.0) for index in range(int(REST_WINDOW_S / 0.05) + 5)]
    assert _rest(carried + still).transform().point_to_odom(*_MARK) == pytest.approx((0.0, 0.0))
