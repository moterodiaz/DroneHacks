"""The mission's phases: waiting for Offboard, the climb, the pilot taking over, and the landing trigger."""
from __future__ import annotations

import math

import numpy as np
import pytest

from dronehacks_course_follower import dry_run
from dronehacks_course_follower.follower import Drone, HeadingMode
from dronehacks_course_follower.mission import (LANDING_CORRECTION_MPS, LANDING_DESCENT_MPS, RETURN_CLIMBED_M,
                                                RETURN_PRESS_MPS, RETURN_STALE_LAND_S, TOUCHDOWN_DWELL_S, Mission,
                                                Phase, PoseCommand, ReturnPlan, TwistCommand, landing_twist)

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


def _flying_for(seconds: float) -> tuple[Mission, float]:
    """A mission flown along its route for `seconds` with the drone on its reference, and the time."""
    mission = _mission()
    mission.tick(0.0, offboard=True, armed=True, drone=_on_the_ground())
    mission.tick(_PERIOD_S, offboard=True, armed=True, drone=Drone(0.0, 0.0, _HEIGHT_M, 0.0))
    now_s = 2 * _PERIOD_S
    while now_s < seconds:
        mission.tick(now_s, offboard=True, armed=True, drone=_airborne(mission))
        now_s += _PERIOD_S

    return mission, now_s


def test_a_tracker_jump_in_flight_descends_straight_down_until_the_disarm() -> None:
    # cuVSLAM jumped about 8 m at once on skybeta-11; steering onto the jumped pose crosses walls
    mission, now_s = _flying_for(1.0)
    east, north = mission.follower.reference()
    command = mission.tick(now_s, offboard=True, armed=True, drone=Drone(east - 8.0, north, _HEIGHT_M, 0.0))
    assert mission.phase is Phase.LOST
    assert isinstance(command, TwistCommand)
    assert (command.east_mps, command.north_mps) == (0.0, 0.0) and -command.up_mps > _LAND_DETECTOR_MPS

    # Back on a sane pose, it still goes straight down, and PX4's disarm ends it off the pad
    command = mission.tick(now_s + _PERIOD_S, offboard=True, armed=True, drone=Drone(east, north + 0.5, 0.5, 0.0))
    assert (command.east_mps, command.north_mps) == (0.0, 0.0)
    mission.tick(now_s + 2 * _PERIOD_S, offboard=True, armed=False, drone=None)
    assert mission.phase is Phase.DONE and not mission.landed_on_pad


def test_odometry_back_after_a_gap_is_not_a_jump() -> None:
    # Half a second without odometry at 1 m/s moves the drone 0.5 m: over the jump distance, not its speed
    mission, now_s = _flying_for(1.0)
    east, north = mission.follower.reference()
    mission.tick(now_s, offboard=True, armed=True, drone=Drone(east, north, _HEIGHT_M, 0.0))
    for step in range(1, 25):
        mission.tick(now_s + step * _PERIOD_S, offboard=True, armed=True, drone=None)

    mission.tick(now_s + 25 * _PERIOD_S, offboard=True, armed=True, drone=Drone(east + 0.5, north, _HEIGHT_M, 0.0))
    assert mission.phase is Phase.FLY


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


def test_the_dry_run_flies_the_return_to_the_launch_mark() -> None:
    assert dry_run.main(["--steps", "0", "--return-home"]) == 0


# The return as the launch defaults fly it, from the pad at the short route's end to the mark at the origin
_RETURN = ReturnPlan(height_m=2.3, rest_s=2.0, speed_mps=0.5)

# skybeta-11's COM_DISARM_LAND, raised from the organisers' 2 s for the return
_DISARM_AFTER_S = 5.0


class _Tick:
    """One tick of a flown mission: its phase after the tick, the command and the drone it saw."""

    def __init__(self, now_s: float, phase: Phase, command: PoseCommand | TwistCommand | None, drone: Drone,
                 frozen: bool) -> None:
        """Keeps the tick's record."""
        self.now_s, self.phase, self.command, self.drone, self.frozen = now_s, phase, command, drone, frozen


def _returning_mission() -> Mission:
    """A mission over the short route that returns to the launch mark."""
    return Mission(_ROUTE, np.empty((0, 4)), _HEIGHT_M, (0.0, 0.0), top_speed_mps=1.0, return_plan=_RETURN)


def _fly_ideally(mission: Mission, disarm_after_s: float = _DISARM_AFTER_S, stop_at: Phase | None = None,
                 until_s: float = 120.0) -> list[_Tick]:
    """Flies the mission against a drone that sits on every pose, follows every twist and lands at
    the floor, PX4 disarming it `disarm_after_s` after it lands; stops at `stop_at` or the run's end."""
    drone, flown, landed_since_s, now_s, ticks = _on_the_ground(), False, math.nan, 0.0, []
    while now_s < until_s and mission.phase not in (Phase.DONE, Phase.ABORTED, stop_at):
        landed = flown and drone.height_m <= 0.0
        landed_since_s = math.nan if not landed else (now_s if math.isnan(landed_since_s) else landed_since_s)
        armed = not landed or now_s - landed_since_s < disarm_after_s
        command = mission.tick(now_s, offboard=True, armed=armed, drone=drone, landed=landed)
        ticks.append(_Tick(now_s, mission.phase, command, drone, mission.offset_frozen))
        drone = _ideal_next(mission, command, drone)
        flown = flown or drone.height_m > 0.0
        now_s += _PERIOD_S

    return ticks


def _ideal_next(mission: Mission, command: PoseCommand | TwistCommand | None, drone: Drone) -> Drone:
    """The ideal drone one tick on: on the follower's reference while it flies the route, else on the
    pose, or moved by the twist and stopped at the floor."""
    if isinstance(command, TwistCommand):
        return Drone(drone.east + command.east_mps * _PERIOD_S, drone.north + command.north_mps * _PERIOD_S,
                     max(drone.height_m + command.up_mps * _PERIOD_S, 0.0), drone.yaw_rad)

    if isinstance(command, PoseCommand) and mission.phase is Phase.FLY:
        return _airborne(mission)

    if isinstance(command, PoseCommand):
        return Drone(command.east, command.north, command.height_m, drone.yaw_rad)

    return drone


def _phases(ticks: list[_Tick]) -> list[Phase]:
    """The phases the ticks passed through, in order, each once."""
    return [tick.phase for index, tick in enumerate(ticks) if index == 0 or tick.phase is not ticks[index - 1].phase]


def test_the_return_rests_on_the_pad_climbs_slides_and_lands_on_the_launch_mark() -> None:
    mission = _returning_mission()
    ticks = _fly_ideally(mission)
    assert _phases(ticks) == [Phase.CLIMB, Phase.FLY, Phase.DESCEND, Phase.RETURN_REST, Phase.RETURN_CLIMB,
                              Phase.RETURN_FLY, Phase.RETURN_DESCEND, Phase.DONE]
    assert mission.landed_on_pad
    assert math.dist((ticks[-1].drone.east, ticks[-1].drone.north), (0.0, 0.0)) < 0.05

    # The rest presses down for its whole length while PX4 reports landed
    rest = [tick for tick in ticks if tick.phase is Phase.RETURN_REST]
    assert rest[-1].now_s - rest[0].now_s == pytest.approx(_RETURN.rest_s, abs=2 * _PERIOD_S)
    assert all(isinstance(tick.command, TwistCommand) and tick.command.up_mps < 0.0 for tick in rest)


def test_without_the_return_a_landed_pad_descent_waits_for_the_disarm() -> None:
    ticks = _fly_ideally(_mission())
    assert _phases(ticks)[-2:] == [Phase.DESCEND, Phase.DONE]


def test_a_rest_longer_than_three_seconds_is_refused() -> None:
    # COM_DISARM_LAND is 5 s and the motors need about 1 s to lift the drone off the pad
    with pytest.raises(ValueError, match="rest"):
        Mission(_ROUTE, np.empty((0, 4)), _HEIGHT_M, (0.0, 0.0), top_speed_mps=1.0,
                return_plan=ReturnPlan(height_m=2.3, rest_s=3.5, speed_mps=0.5))


def test_the_return_climbs_straight_over_the_pad_before_it_slides() -> None:
    ticks = _fly_ideally(_returning_mission())
    climb = [tick.command for tick in ticks if tick.phase is Phase.RETURN_CLIMB]
    assert climb and all(command[:3] == (*_ROUTE[-1], _RETURN.height_m) for command in climb)

    slide = [tick for tick in ticks if tick.phase is Phase.RETURN_FLY]
    assert slide[0].drone.height_m >= _RETURN.height_m - RETURN_CLIMBED_M
    easts = [tick.command.east for tick in slide]
    assert all(tick.command.height_m == _RETURN.height_m for tick in slide)
    assert all(later <= earlier for earlier, later in zip(easts, easts[1:], strict=False))


def test_the_returns_descent_aims_at_the_launch_mark() -> None:
    mission = _returning_mission()
    _fly_ideally(mission, stop_at=Phase.RETURN_DESCEND)
    twist = mission.tick(100.0, offboard=True, armed=True, drone=Drone(0.4, -0.2, 1.0, 0.0))
    assert isinstance(twist, TwistCommand) and twist.east_mps < 0.0 < twist.north_mps and twist.up_mps < 0.0


def test_a_disarm_during_the_rest_ends_the_run_landed_on_the_pad() -> None:
    # A drone still at the organisers' COM_DISARM_LAND of 1-2 s disarms before the rest ends
    mission = _returning_mission()
    ticks = _fly_ideally(mission, disarm_after_s=1.0)
    assert _phases(ticks)[-2:] == [Phase.RETURN_REST, Phase.DONE]
    assert mission.landed_on_pad and "rest" in mission.events[-1]


def test_px4s_offset_is_frozen_from_the_pad_descent_on() -> None:
    ticks = _fly_ideally(_returning_mission())
    for tick in ticks:
        assert tick.frozen == (tick.phase not in (Phase.CLIMB, Phase.FLY)), tick.phase


def test_lost_odometry_on_the_return_holds_then_descends_straight_down() -> None:
    mission = _returning_mission()
    ticks = _fly_ideally(mission, stop_at=Phase.RETURN_FLY)
    now_s = ticks[-1].now_s
    commands = []
    for step in range(1, int((RETURN_STALE_LAND_S + 0.5) / _PERIOD_S)):
        commands.append(mission.tick(now_s + step * _PERIOD_S, offboard=True, armed=True, drone=None))

    held = commands[:int((RETURN_STALE_LAND_S - 0.1) / _PERIOD_S)]
    assert all(isinstance(command, PoseCommand) and command[:3] == held[0][:3] for command in held)
    assert mission.phase is Phase.RETURN_DESCEND
    assert commands[-1] == TwistCommand(0.0, 0.0, commands[-1].up_mps, 0.0) and commands[-1].up_mps < 0.0

    # Odometry back mid-descent: it still goes straight down rather than steering blind across walls
    twist = mission.tick(now_s + 10.0, offboard=True, armed=True, drone=Drone(1.0, 0.0, 1.0, 0.0))
    assert (twist.east_mps, twist.north_mps) == (0.0, 0.0)


def test_the_pilot_taking_over_on_the_return_aborts_it() -> None:
    mission = _returning_mission()
    ticks = _fly_ideally(mission, stop_at=Phase.RETURN_FLY)
    assert mission.tick(ticks[-1].now_s + _PERIOD_S, offboard=False, armed=True, drone=ticks[-1].drone) is None
    assert mission.phase is Phase.ABORTED


# A 1 m/s error through PX4's MPC_Z_VEL_P_ACC of 4 asks 4 m/s² down, past the land detector's
# low-throttle threshold at once; the descent's 0.5 m/s waits on the integrator
_FIRM_PRESS_MPS = 1.0


def _sink(mission: Mission, now_s: float, drone: Drone, seconds: float, floor_m: float = 0.0,
          landed: bool = False) -> list[tuple[float, Drone, TwistCommand]]:
    """Ticks the mission for `seconds` against a drone that follows every twist down to `floor_m`,
    PX4 reporting `landed` throughout; returns each tick's time, drone and command."""
    record = []
    for step in range(1, int(seconds / _PERIOD_S) + 1):
        tick_s = now_s + step * _PERIOD_S
        command = mission.tick(tick_s, offboard=True, armed=True, drone=drone, landed=landed)
        record.append((tick_s, drone, command))
        if isinstance(command, TwistCommand):
            drone = Drone(drone.east, drone.north, max(drone.height_m + command.up_mps * _PERIOD_S, floor_m),
                          drone.yaw_rad)

    return record


def test_on_the_floor_the_descent_presses_firmly_until_px4_lands() -> None:
    mission = _mission()
    ticks = _fly_ideally(mission, stop_at=Phase.DESCEND)
    record = _sink(mission, ticks[-1].now_s, ticks[-1].drone, 4.0)
    assert mission.phase is Phase.DESCEND

    # All the way down at the descent's speed; the firm press only after a dwell on the floor
    floor_s = next(tick_s for tick_s, drone, _ in record if drone.height_m <= 0.0)
    firm = [tick_s for tick_s, _, command in record if command.up_mps <= -_FIRM_PRESS_MPS]
    assert all(command.up_mps == -LANDING_DESCENT_MPS for tick_s, _, command in record if tick_s < floor_s)
    assert firm and abs(firm[0] - (floor_s + TOUCHDOWN_DWELL_S)) <= 0.1
    assert firm == [tick_s for tick_s, _, _ in record if tick_s >= firm[0]]


def test_a_descent_held_up_above_the_floor_keeps_its_speed() -> None:
    # A stall 0.5 m above where the drone stood at Offboard is not trusted as the floor
    mission = _mission()
    ticks = _fly_ideally(mission, stop_at=Phase.DESCEND)
    record = _sink(mission, ticks[-1].now_s, ticks[-1].drone, 3.0, floor_m=0.5)
    assert all(command.up_mps == -LANDING_DESCENT_MPS for _, _, command in record)


def test_the_returns_rest_and_climb_follow_a_firm_press_on_the_pad() -> None:
    mission = _returning_mission()
    ticks = _fly_ideally(mission, stop_at=Phase.DESCEND)
    pressed = _sink(mission, ticks[-1].now_s, ticks[-1].drone, 3.0)
    assert pressed[-1][2].up_mps == -_FIRM_PRESS_MPS

    # PX4 reports landed: the rest presses gently, then the climb lifts off the pad
    rest = _sink(mission, pressed[-1][0], pressed[-1][1], _RETURN.rest_s + 0.2, landed=True)
    twists = [command for _, _, command in rest if isinstance(command, TwistCommand)]
    assert all(command == TwistCommand(0.0, 0.0, -RETURN_PRESS_MPS, 0.0) for command in twists)
    assert len(twists) * _PERIOD_S == pytest.approx(_RETURN.rest_s, abs=2 * _PERIOD_S)
    assert mission.phase is Phase.RETURN_CLIMB
    assert rest[-1][2][:3] == (*_ROUTE[-1], _RETURN.height_m)


def test_on_the_launch_mark_the_returns_descent_presses_firmly() -> None:
    mission = _returning_mission()
    ticks = _fly_ideally(mission, stop_at=Phase.RETURN_DESCEND)
    record = _sink(mission, ticks[-1].now_s, ticks[-1].drone, 8.0)
    assert record[0][2].up_mps == -LANDING_DESCENT_MPS
    assert record[-1][2].up_mps == -_FIRM_PRESS_MPS
