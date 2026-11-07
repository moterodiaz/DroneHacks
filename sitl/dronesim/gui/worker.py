"""The simulation process behind the GUI.

The engine patches ``time``, ``threading`` and stdio for the team's nodes, so it
lives in its own process.  It talks to the GUI over a ``multiprocessing`` pipe:

GUI -> worker   ('pause',) ('resume',) ('speed', x|None) ('command', dict)
                ('echo', topic) ('save', dir) ('stop',)
worker -> GUI   ('ready', info) ('tel', snapshot, [log records]) ('topics', list)
                ('echo', topic, text) ('done', result) ('saved', dir) ('error', text)
"""
from __future__ import annotations

import os
import sys
import traceback


def worker_main(conn, scenario_dict, speed):
    here = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    if here not in sys.path:
        sys.path.insert(0, here)
    logs = []
    try:
        from dronesim.rosfake.runtime import REAL_PERF, REAL_SLEEP
        from dronesim.sim import Scenario, Simulation

        def sink(record):
            logs.append(record)
            if len(logs) > 4000:
                del logs[:2000]
        sim = Simulation(Scenario.from_dict(scenario_dict), log_sink=sink)
    except Exception:  # noqa: BLE001 - shown to the user
        conn.send(('error', traceback.format_exc()))
        conn.close()
        return
    info = {
        'version': sim.version['id'], 'label': sim.version['label'], 'provenance': sim.provenance,
        'notes': list(sim.run_notes), 'measured': dict(sim.subs),
        'install_config': [{'file': f, 'node': n, 'values': v} for f, n, v in sim.config_written],
        'params_profile': sim.params.profile,
        'param_sources': {k: (sim.params.get(k), sim.params.sources.get(k)) for k in (
            'MPC_XY_VEL_MAX', 'NAV_RCL_ACT', 'COM_OBL_RC_ACT', 'COM_OF_LOSS_T', 'COM_FAIL_ACT_T',
            'COM_DISARM_PRFLT', 'COM_DISARM_LAND', 'COM_RC_OVERRIDE', 'COM_FLTMODE6', 'EKF2_EV_DELAY',
            'EKF2_EV_CTRL', 'EKF2_HGT_REF', 'EKF2_RNG_CTRL', 'GF_MAX_VER_DIST', 'MPC_THR_HOVER',
            'MAV_SYS_ID', 'EKF2_NOAID_TOUT')},
        'team_packages': list(sim.team_packages),
    }
    conn.send(('ready', info))
    paused = False
    done_sent = False
    wall_last = REAL_PERF()
    tel_last = 0.0
    topics_last = 0.0
    try:
        while True:
            while conn.poll():
                msg = conn.recv()
                kind = msg[0]
                if kind == 'stop':
                    raise SystemExit
                if kind == 'pause':
                    paused = True
                elif kind == 'resume':
                    paused = False
                    wall_last = REAL_PERF()
                elif kind == 'speed':
                    speed = msg[1]
                    wall_last = REAL_PERF()
                elif kind == 'command':
                    try:
                        sim.command(msg[1])
                    except Exception as exc:  # noqa: BLE001
                        logs.append({'t': sim.t, 'process': None, 'logger': 'dronesim', 'level': 'ERROR',
                                     'source': 'sim', 'msg': f'command failed: {exc}'})
                elif kind == 'echo':
                    conn.send(('echo', msg[1], sim.echo(msg[1]) or '(no message yet)'))
                elif kind == 'save':
                    sim.save(msg[1])
                    conn.send(('saved', msg[1]))
            now = REAL_PERF()
            running = not paused and not sim.done()
            if running:
                if speed is None:
                    sim.step(0.05)
                else:
                    budget = (now - wall_last) * speed
                    budget = min(budget, 0.25)
                    if budget >= 0.01:
                        sim.step(budget)
                        wall_last = now
                    else:
                        REAL_SLEEP(0.004)
            else:
                REAL_SLEEP(0.02)
                wall_last = REAL_PERF()
            if now - tel_last > 0.05:
                tel_last = now
                snap = sim.snapshot()
                snap['est'] = sim.estimates_in_course()
                snap['paused'] = paused
                snap['done'] = sim.done()
                batch = logs[:]
                logs.clear()
                conn.send(('tel', snap, batch))
            if now - topics_last > 1.0:
                topics_last = now
                conn.send(('topics', sim.topics()))
            if sim.done() and not done_sent:
                done_sent = True
                conn.send(('done', sim.result()))
    except (SystemExit, EOFError, BrokenPipeError):
        pass
    except Exception:  # noqa: BLE001
        try:
            conn.send(('error', traceback.format_exc()))
        except Exception:  # noqa: BLE001
            pass
    finally:
        try:
            sim.close()
        except Exception:  # noqa: BLE001
            pass
        try:
            conn.close()
        except Exception:  # noqa: BLE001
            pass
