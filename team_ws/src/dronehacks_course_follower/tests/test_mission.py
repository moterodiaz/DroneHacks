"""The mission's phases: waiting for Offboard, the climb, the pilot taking over, and the landing trigger."""
from __future__ import annotations

import math

import numpy as np

from dronehacks_course_follower import dry_run
from dronehacks_course_follower.follower import Drone
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


def test_the_dry_run_flies_the_packaged_map_to_a_landing() -> None:
    assert dry_run.main(["--steps", "0"]) == 0
