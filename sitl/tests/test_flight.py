"""End-to-end flights: the organizer procedure on the real kit code and team code.

These run the full simulation (tens of seconds of CPU each).  The traj and
landing flights need the git refs their versions name in versions.yaml
(navigation/vo-traj-follower, core/vo-nav-no-lidar); each is skipped if its
ref is missing.
"""
import os
import subprocess
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
SITL = os.path.dirname(HERE)
REPO = os.path.dirname(SITL)
sys.path.insert(0, SITL)


def _have_ref(ref):
    # the catalog's own rule: the ref itself, else origin/<ref> (a fresh clone)
    return any(subprocess.run(['git', '-C', REPO, 'rev-parse', '--verify', candidate],
                              capture_output=True).returncode == 0 for candidate in (ref, 'origin/' + ref))


def _run(version, until, **kw):
    code = (
        'import sys, json; sys.path.insert(0, %r)\n'
        'from dronesim.sim import Scenario, Simulation\n'
        'sc = Scenario(version=%r, duration_s=%r, **%r)\n'
        'sim = Simulation(sc)\n'
        'sim.run()\n'
        'r = sim.result(); st = sim.snapshot()\n'
        'tr = sim.trajectory; hi = [i for i, row in enumerate(tr) if row[3] > 0.6]\n'
        'td = next((row[0] for row in tr[hi[-1]:] if row[3] < 0.08), None) if hi else None\n'
        'print("RESULT" + json.dumps({"events": r["events"], "ref": r["referee"], "procs": r["processes"],'
        ' "nav": st["px4"]["nav_name"], "z": st["true"]["z"], "msgs": r["px4_messages"], "touchdown": td}))\n'
        'sim.close()\n' % (SITL, version, until, kw))
    out = subprocess.run([sys.executable, '-B', '-c', code], capture_output=True, text=True, timeout=1500)
    for line in out.stdout.splitlines():
        if line.startswith('RESULT'):
            import json
            return json.loads(line[6:])
    raise AssertionError(out.stdout[-3000:] + out.stderr[-3000:])


class KitProcedureTest(unittest.TestCase):
    def test_arm_then_position_enters_offboard_and_takes_off(self):
        r = _run('kit_square', 26.0)
        msgs = ' | '.join(m[2] for m in r['msgs'])
        self.assertIn('Armed by RC switch', msgs)
        self.assertEqual(r['nav'], 'OFFBOARD')
        self.assertGreater(r['z'], 1.2)               # the example hovers at 1.5 m above its start
        self.assertTrue(all(p['alive'] for p in r['procs']))


@unittest.skipUnless(_have_ref('navigation/vo-traj-follower'), 'traj ref not present')
class TrajFlightTest(unittest.TestCase):
    def test_traj_reaches_the_tunnel_exit_when_the_tunnel_is_lit(self):
        # with the default dark tunnel cuVSLAM drops out inside the tube (see README "Known results")
        r = _run('traj', 70.0, vo={'tunnel_lighting': 'lit'})
        rows = {c['id']: c for c in r['ref']['checkpoints']}
        self.assertIsNotNone(rows[2]['whole_drone_s'])     # Gate A
        self.assertIsNotNone(rows[3]['whole_drone_s'])     # tunnel south end
        self.assertEqual(r['ref']['touches'], 0)


@unittest.skipUnless(_have_ref('core/vo-nav-no-lidar'), 'vo_nolidar ref not present')
class LandingTest(unittest.TestCase):
    def test_px4_disarms_within_a_few_seconds_of_touchdown_on_the_pad(self):
        # mc_pos_control's flying-but-ground-contact thrust cut lets the land detector finish within
        # LNDMC_TRIG_TIME of touchdown; then COM_DISARM_LAND (2 s). Without it the vehicle sat on the
        # pad armed for about 21 s while the velocity integrator wound the thrust down.
        r = _run('vo_nolidar', 130.0, vo={'tunnel_lighting': 'lit'})
        disarm = next(m[0] for m in r['msgs'] if 'Disarmed by auto disarm on land' in m[2])
        self.assertIsNotNone(r['touchdown'])
        self.assertLess(disarm - r['touchdown'], 4.0)
        rows = {c['id']: c for c in r['ref']['checkpoints']}
        self.assertGreater(rows[5]['earned'], 0.0)             # landed and disarmed on the pad
        self.assertEqual(r['ref']['touches'], 0)


if __name__ == '__main__':
    unittest.main()
