"""End-to-end flights: the organizer procedure on the real kit code and team code.

These run the full simulation (tens of seconds of CPU each).  The traj flight
needs the git ref origin/navigation/vo-traj-follower in the local repository;
it is skipped if the ref is missing.
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
    r = subprocess.run(['git', '-C', REPO, 'rev-parse', '--verify', ref], capture_output=True)
    return r.returncode == 0


def _run(version, until, **kw):
    code = (
        'import sys, json; sys.path.insert(0, %r)\n'
        'from dronesim.sim import Scenario, Simulation\n'
        'sc = Scenario(version=%r, duration_s=%r, **%r)\n'
        'sim = Simulation(sc)\n'
        'sim.run()\n'
        'r = sim.result(); st = sim.snapshot()\n'
        'print("RESULT" + json.dumps({"events": r["events"], "ref": r["referee"], "procs": r["processes"],'
        ' "nav": st["px4"]["nav_name"], "z": st["true"]["z"], "msgs": r["px4_messages"]}))\n'
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


@unittest.skipUnless(_have_ref('origin/navigation/vo-traj-follower'), 'traj ref not present')
class TrajFlightTest(unittest.TestCase):
    def test_traj_reaches_the_tunnel_exit_when_the_tunnel_is_lit(self):
        # with the default dark tunnel cuVSLAM drops out inside the tube (see README "Known results")
        r = _run('traj', 70.0, vo={'tunnel_lighting': 'lit'})
        rows = {c['id']: c for c in r['ref']['checkpoints']}
        self.assertIsNotNone(rows[2]['whole_drone_s'])     # Gate A
        self.assertIsNotNone(rows[3]['whole_drone_s'])     # tunnel south end
        self.assertEqual(r['ref']['touches'], 0)


if __name__ == '__main__':
    unittest.main()
