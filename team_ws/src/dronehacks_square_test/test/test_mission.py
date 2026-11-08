"""Fly the mission against a crude drone model, including PX4's auto-disarm after landing."""
import math

from dronehacks_square_test.mission import DONE, STOPPED, Config, Mission, _wrap, frames_disagree

DT = 0.05
START = (2.0, -1.0, 0.0)
COM_DISARM_LAND = 2.0
START_YAW = 0.4                  # rad: not zero, so the test catches a missing offset


def fly(mission, drop_pose=None, takeover_at=None, max_t=150.0, heading=True, height_bias=0.0):
    """Moves toward a pose setpoint at <= 0.6 m/s (z floored 0.3 m, as the kit does), follows
    twists, sits on the ground at z 0. Disarms after COM_DISARM_LAND landed, which ends Offboard.
    `height_bias` is how far below a commanded height the drone settles in the tracker's z: PX4
    holds height by its rangefinder, which reads a few centimetres apart from cuVSLAM."""
    x, y, z = START
    yaw = START_YAW
    armed = True
    landed_since = None
    track, cmds = [], []
    t = 0.0
    while t < max_t:
        t += DT
        offboard = armed and t > 1.0 and (takeover_at is None or t < takeover_at)
        landed = z <= 1e-6
        if landed and t > 1.0:
            landed_since = t if landed_since is None else landed_since
            if t - landed_since >= COM_DISARM_LAND:
                armed = offboard = False
        else:
            landed_since = None
        stale = drop_pose and drop_pose[0] <= t < drop_pose[1]
        cmd = mission.step(t, None if stale else (x, y, z), None if stale else t, offboard, landed, armed,
                           yaw if heading and not stale else None)
        cmds.append((t, cmd))
        if not offboard:
            pass                              # PX4 ignores setpoints outside Offboard
        elif cmd.kind == 'pose':
            d = [cmd.x - x, cmd.y - y, max(cmd.z, 0.3) - height_bias - z]
            n = math.sqrt(sum(v * v for v in d))
            f = min(1.0, 0.6 * DT / n) if n else 0.0
            x, y, z = x + d[0] * f, y + d[1] * f, z + d[2] * f
            if cmd.yaw is not None:           # PX4 turns toward it at up to 60 deg/s
                e = _wrap(cmd.yaw - yaw)
                yaw = _wrap(yaw + max(-math.radians(60) * DT, min(math.radians(60) * DT, e)))
        elif cmd.kind == 'twist':
            x, y, z = x + cmd.x * DT, y + cmd.y * DT, max(0.0, z + cmd.z * DT)
        track.append((t, x, y, z, armed, yaw))
        if mission.state in (DONE, STOPPED):
            break
    return track, cmds


def held(track, p, tol=0.12):
    """Longest stretch spent within tol of p, in seconds."""
    best = run = 0.0
    for _, x, y, z, _, _ in track:
        run = run + DT if math.dist((x, y, z), p) <= tol else 0.0
        best = max(best, run)
    return best


def test_full_sequence():
    m = Mission(Config())
    track, _ = fly(m)
    assert m.state == DONE and m.reason == 'disarmed on the ground', m.reason
    s, h = 0.5, 0.73
    x0, y0, _ = START
    for dx, dy in [(0, 0), (s, 0), (s, s), (0, s)]:          # x forward, y left
        assert held(track, (x0 + dx, y0 + dy, h)) >= 2.9, (dx, dy)
    touchdowns = sum(1 for a, b in zip(track, track[1:]) if a[3] > 1e-6 and b[3] <= 1e-6)
    assert touchdowns == 2
    first_down = next(i for i, p in enumerate(track) if i and p[3] <= 1e-6 and track[i - 1][3] > 1e-6)
    assert all(p[4] for p in track[:first_down + 20])        # still armed on the ground: touch-and-go
    assert max(p[3] for p in track[first_down:]) > 0.7       # climbed back to height
    _, x, y, z, armed, yaw = track[-1]
    assert not armed and z == 0.0 and math.hypot(x - x0, y - y0) < 0.05
    assert abs(_wrap(yaw - START_YAW)) < math.radians(5)         # faces its take-off heading again
    assert all(x0 - 0.1 <= p[1] <= x0 + s + 0.1 and y0 - 0.1 <= p[2] <= y0 + s + 0.1 for p in track)


def test_ground_wait_longer_than_auto_disarm_ends_after_first_landing():
    m = Mission(Config(ground_s=2.5))
    track, _ = fly(m)
    assert m.state == DONE and 'COM_DISARM_LAND' in m.reason
    assert max(p[3] for p in track[-30:]) < 0.05


def test_pilot_takeover_stops_for_good():
    m = Mission(Config())
    _, cmds = fly(m, takeover_at=8.0, max_t=12.0)
    assert m.state == STOPPED
    assert all(c.kind == 'none' for t, c in cmds if t >= 8.0)


def test_stale_pose_holds_then_descends():
    m = Mission(Config())
    _, cmds = fly(m, drop_pose=(10.0, 30.0), max_t=14.5)
    during = [c for t, c in cmds if 10.0 <= t < 12.9]
    assert during and all(c.kind == 'twist' and (c.x, c.y, c.z) == (0.0, 0.0, 0.0) for c in during)
    late = [c for t, c in cmds if 13.1 <= t < 14.5]
    assert late and all(c.kind == 'twist' and c.z < 0 and c.x == c.y == 0.0 for c in late)


def test_no_pose_no_stream():
    m = Mission(Config())
    assert m.step(0.0, None, None, False, True, False).kind == 'none'
    assert m.step(1.0, (0, 0, 0), 1.0, False, True, False).kind == 'pose'


def test_turns_slowly_to_face_each_leg():
    m = Mission(Config())
    track, cmds = fly(m)
    s, h = 0.5, 0.73
    x0, y0, _ = START
    # at each corner, the heading it settles on before moving on: left turns of 90 deg
    for (dx, dy), turn in [((s, 0), 90), ((s, s), 180), ((0, s), 270)]:
        here = [p for p in track if math.dist(p[1:4], (x0 + dx, y0 + dy, h)) < 0.03]
        assert abs(_wrap(here[-1][5] - START_YAW - math.radians(turn))) < math.radians(5), (dx, dy)
    yaws = [(t, c.yaw) for t, c in cmds if c.kind == 'pose' and c.yaw is not None and t > 1.1]
    for (t0, a), (t1, b) in zip(yaws, yaws[1:]):
        assert abs(_wrap(b - a)) <= math.radians(20) * (t1 - t0) + 1e-9   # never faster than 20 deg/s
    assert max(abs(_wrap(b - a)) for (_, a), (_, b) in zip(yaws, yaws[1:])) > 0   # it did turn


def test_a_height_a_little_off_the_trackers_still_flies_the_box():
    # The rangefinder PX4 holds height by and cuVSLAM's z differ by a few percent of the height,
    # so the drone settles 12 cm under the tracker's target: the box must still be flown
    m = Mission(Config())
    track, _ = fly(m, height_bias=0.12)
    assert m.state == DONE and m.reason == 'disarmed on the ground', m.reason
    x0, y0, _ = START
    assert max(p[1] for p in track) > x0 + 0.45 and max(p[2] for p in track) > y0 + 0.45


def test_frames_that_disagree_are_named_and_agreeing_ones_pass():
    # PX4 NED is the tracker's ENU with x and y swapped and z negated, when both started together
    odom = (-0.04, -0.01, 0.01)
    assert frames_disagree(odom, (-0.01, -0.04, -0.03), limit_m=0.15) is None

    # Skybeta-11 on 2026-11-07: PX4's height origin 0.73 m above the floor, the tracker's on it
    assert 'height' in frames_disagree(odom, (-0.01, -0.04, 0.72), limit_m=0.15)

    # The tracker started 3.3 m away from where PX4's position says the drone is
    assert 'position' in frames_disagree((-3.30, 0.64, 0.0), (0.0, 0.0, 0.0), limit_m=0.15)


def test_without_heading_it_flies_without_commanding_yaw():
    m = Mission(Config())
    _, cmds = fly(m, heading=False)
    assert m.state == DONE
    assert all(c.yaw is None for _, c in cmds)
