"""The mission's phases: waiting for Offboard, the climb, the pilot taking over, and the landing trigger."""
from __future__ import annotations

import math

import numpy as np
import pytest

from dronehacks_course_follower import dry_run
from dronehacks_course_follower.follower import Drone, HeadingMode
from dronehacks_course_follower.mission import (LANDING_CORRECTION_MPS, Mission, Phase, PoseCommand, TwistCommand,
                                                landing_twist)

# PX4 counts ground contact only while the setpoint descends at 1.1 x LNDMC_Z_VEL_MAX, 0.25 m/s in
# the organisers' parameter dumps
_LAND_DETECTOR_MPS = 1.1 * 0.25

# The descent the run plan asks for, well clear of that threshold
_LANDING_MIN_MPS = 0.5

# A short route in open space from a launch mark at the origin, flown at this height
_ROUTE = [(2.0, 0.0)]
_HEIGHT_M = 0.8
_PERIOD_S = 0.02


def _mission() -> Mission:
    """A mission over the short route with no walls."""
    return Mission(_ROUTE, np.empty((0, 4)), _HEIGHT_M, (0.0, 0.0), top_speed_mps=1.0)


def _on_the_ground(east: float = 0.0) -> Drone:
    """The drone resting on the floor at `east` on the route's line, facing along it."""
    return Drone(east, 0.0, 0.0, 0.0)


def _airborne(mission: Mission) -> Drone:
    """The drone at the flight height on the follower's reference."""
    east, north = mission.follower.reference()
    return Drone(east, north, _HEIGHT_M, mission.follower.yaw_rad)


def test_before_offboard_it_streams_a_pose_over_the_drone_at_the_flight_height() -> None:
    mission = _mission()
    command = mission.tick(0.0, offboard=False, armed=True, drone=_on_the_ground())
    assert command == PoseCommand(0.0, 0.0, _HEIGHT_M, command.yaw_rad)
    assert math.isnan(command.yaw_rad)
    assert mission.tick(_PERIOD_S, offboard=False, armed=True, drone=None) is None


def test_offboard_climbs_then_flies_once_at_height() -> None:
    mission = _mission()
    mission.tick(0.0, offboard=True, armed=True, drone=_on_the_ground())
    assert mission.phase is Phase.CLIMB

    mission.tick(_PERIOD_S, offboard=True, armed=True, drone=Drone(0.0, 0.0, _HEIGHT_M - 0.05, 0.0))
    assert mission.phase is Phase.FLY


def _climb_holding_at(height_m: float, seconds: float) -> Mission:
    """A mission past Offboard whose drone then holds still at `height_m` for `seconds`."""
    mission = _mission()
    mission.tick(0.0, offboard=True, armed=True, drone=_on_the_ground())
    for step in range(1, int(seconds / _PERIOD_S)):
        mission.tick(step * _PERIOD_S, offboard=True, armed=True, drone=Drone(0.0, 0.0, height_m, 0.0))

    return mission


def test_a_climb_that_settles_short_of_the_flight_height_still_starts_the_route() -> None:
    # In SITL PX4 held 0.80 m by its own height estimate while the tracker read 0.63-0.66 m, and
    # the drone hovered on the mark for minutes waiting to be within 0.1 m
    assert _climb_holding_at(0.65, 2.0).phase is Phase.FLY


def test_a_drone_still_on_the_ground_or_still_rising_is_not_settled() -> None:
    assert _climb_holding_at(0.0, 5.0).phase is Phase.CLIMB

    mission = _mission()
    mission.tick(0.0, offboard=True, armed=True, drone=_on_the_ground())
    now_s, height_m = _PERIOD_S, 0.4
    while height_m < _HEIGHT_M - 0.15:
        mission.tick(now_s, offboard=True, armed=True, drone=Drone(0.0, 0.0, height_m, 0.0))
        now_s, height_m = now_s + _PERIOD_S, height_m + 0.3 * _PERIOD_S
        assert mission.phase is Phase.CLIMB


def test_a_held_climb_hovers_over_the_start_until_released() -> None:
    # The node holds the climb while the push compensator measures the hover
    mission = _mission()
    mission.tick(0.0, offboard=True, armed=True, drone=_on_the_ground())
    for step in range(1, 200):
        command = mission.tick(step * _PERIOD_S, offboard=True, armed=True, drone=Drone(0.0, 0.0, _HEIGHT_M, 0.0),
                               hold_climb=True)
        assert mission.phase is Phase.CLIMB and mission.hovering
        assert command[:3] == (0.0, 0.0, _HEIGHT_M)

    mission.tick(200 * _PERIOD_S, offboard=True, armed=True, drone=Drone(0.0, 0.0, _HEIGHT_M, 0.0))
    assert mission.phase is Phase.FLY


def test_offboard_away_from_the_launch_mark_refuses_the_route() -> None:
    mission = _mission()
    assert mission.tick(0.0, offboard=True, armed=True, drone=_on_the_ground(east=1.0)) is None
    assert mission.phase is Phase.ABORTED


def test_the_pilot_taking_over_stops_the_mission_until_a_disarm() -> None:
    mission = _mission()
    mission.tick(0.0, offboard=True, armed=True, drone=_on_the_ground())
    assert mission.tick(_PERIOD_S, offboard=False, armed=True, drone=_on_the_ground()) is None
    assert mission.phase is Phase.ABORTED

    mission.tick(2 * _PERIOD_S, offboard=False, armed=False, drone=_on_the_ground())
    assert mission.phase is Phase.WAIT


def test_at_the_routes_end_it_descends_fast_enough_to_land_and_stops_at_the_disarm() -> None:
    mission = _mission()
    mission.tick(0.0, offboard=True, armed=True, drone=_on_the_ground())
    mission.tick(_PERIOD_S, offboard=True, armed=True, drone=Drone(0.0, 0.0, _HEIGHT_M, 0.0))
    now_s, command = 2 * _PERIOD_S, None
    while mission.phase is Phase.FLY and now_s < 20.0:
        command = mission.tick(now_s, offboard=True, armed=True, drone=_airborne(mission))
        now_s += _PERIOD_S

    assert mission.phase is Phase.DESCEND
    assert isinstance(command, TwistCommand)
    assert -command.up_mps >= _LANDING_MIN_MPS > _LAND_DETECTOR_MPS

    # PX4 disarms on its own: the mission is done and sends nothing more
    assert mission.tick(now_s, offboard=True, armed=False, drone=Drone(*_ROUTE[-1], 0.0, 0.0)) is None
    assert mission.phase is Phase.DONE
    assert mission.flight_over(armed=False)


def test_the_flight_is_over_only_once_offboard_was_flown_and_px4_has_disarmed() -> None:
    mission = _mission()

    # Arming and disarming on the ground, or a route refused away from the mark, is no flight
    mission.tick(0.0, offboard=False, armed=True, drone=_on_the_ground())
    mission.tick(_PERIOD_S, offboard=False, armed=False, drone=_on_the_ground())
    mission.tick(2 * _PERIOD_S, offboard=True, armed=True, drone=_on_the_ground(east=1.0))
    mission.tick(3 * _PERIOD_S, offboard=False, armed=False, drone=_on_the_ground(east=1.0))
    assert not mission.flight_over(armed=False)

    # A climb that the pilot takes over is a flight, over once PX4 disarms
    mission.tick(4 * _PERIOD_S, offboard=True, armed=True, drone=_on_the_ground())
    mission.tick(5 * _PERIOD_S, offboard=False, armed=True, drone=_on_the_ground())
    assert not mission.flight_over(armed=True)
    assert mission.flight_over(armed=False)


def test_with_landing_off_it_holds_over_the_routes_end() -> None:
    mission = Mission(_ROUTE, np.empty((0, 4)), _HEIGHT_M, (0.0, 0.0), top_speed_mps=1.0, land=False)
    mission.tick(0.0, offboard=True, armed=True, drone=_on_the_ground())
    mission.tick(_PERIOD_S, offboard=True, armed=True, drone=Drone(0.0, 0.0, _HEIGHT_M, 0.0))
    command = None
    for step in range(2, 1000):
        command = mission.tick(step * _PERIOD_S, offboard=True, armed=True, drone=_airborne(mission))

    assert mission.phase is Phase.FLY
    assert isinstance(command, PoseCommand) and command.height_m == _HEIGHT_M
    assert math.dist(command[:2], _ROUTE[-1]) < 0.05


def test_the_landing_twist_steers_back_over_the_pad_no_faster_than_its_cap() -> None:
    twist = landing_twist((0.0, 0.0), Drone(3.0, -4.0, 0.5, 0.0))
    assert math.hypot(twist.east_mps, twist.north_mps) <= LANDING_CORRECTION_MPS + 1e-9
    assert twist.east_mps < 0.0 < twist.north_mps
    assert landing_twist((0.0, 0.0), None)[:2] == (0.0, 0.0)


def test_a_fixed_heading_commands_the_heading_at_offboard_from_the_climb_to_the_landing() -> None:
    # Not the route's bearing, east, so a look-ahead heading would turn; the drone has wandered off
    # it a little by the time it reaches the height, and the route still holds the Offboard heading
    offboard_yaw_rad, wandered_yaw_rad = 1.0, 1.05
    mission = Mission(_ROUTE, np.empty((0, 4)), _HEIGHT_M, (0.0, 0.0), top_speed_mps=1.0,
                      heading_mode=HeadingMode.FIXED)
    mission.tick(0.0, offboard=True, armed=True, drone=Drone(0.0, 0.0, 0.0, offboard_yaw_rad))
    climb = mission.tick(_PERIOD_S, offboard=True, armed=True, drone=Drone(0.0, 0.0, 0.3, wandered_yaw_rad))
    assert climb.yaw_rad == offboard_yaw_rad

    mission.tick(2 * _PERIOD_S, offboard=True, armed=True, drone=Drone(0.0, 0.0, _HEIGHT_M, wandered_yaw_rad))
    now_s, commands = 3 * _PERIOD_S, []
    while mission.phase is Phase.FLY and now_s < 20.0:
        commands.append(mission.tick(now_s, offboard=True, armed=True, drone=_airborne(mission)))
        now_s += _PERIOD_S

    assert mission.phase is Phase.DESCEND
    poses = [command for command in commands if isinstance(command, PoseCommand)]
    assert poses and all(math.isclose(pose.yaw_rad, offboard_yaw_rad) for pose in poses)
    assert isinstance(commands[-1], TwistCommand) and commands[-1].yaw_rate_rad_s == 0.0


def test_a_turn_rate_that_is_not_positive_is_refused() -> None:
    with pytest.raises(ValueError, match="turn rate"):
        Mission(_ROUTE, np.empty((0, 4)), _HEIGHT_M, (0.0, 0.0), top_speed_mps=1.0, turn_rate_rad_s=0.0)


def test_the_dry_run_flies_the_packaged_map_to_a_landing() -> None:
    assert dry_run.main(["--steps", "0"]) == 0


def test_the_dry_run_flies_the_packaged_map_to_a_landing_with_a_fixed_heading() -> None:
    assert dry_run.main(["--steps", "0", "--heading-mode", "fixed"]) == 0
