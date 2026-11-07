"""Command line: ``python -m dronesim <command>`` (run from ``sitl/``).

    list                                   autonomy versions, followers, options, PX4 profiles
    run --version traj [--follower F] [--option route=window] [--duration 120]
        [--speed 1.0|max] [--speed-cap 1.0] [--out runs/x] [--set key=value ...] [--fault t:action[:arg]]
    gui                                    the graphical test environment
"""
from __future__ import annotations

import argparse
import json
import os
import sys


def _parse_value(text):
    import yaml
    try:
        return yaml.safe_load(text)
    except Exception:  # noqa: BLE001
        return text


def cmd_list(args):
    from .autonomy.catalog import load_catalog
    from .px4.params import PROFILES
    from .sim import SITL_ROOT
    cat = load_catalog(os.path.join(SITL_ROOT, 'versions.yaml'))
    for vid, v in cat.items():
        print(f'{vid:16s} {v["label"]}')
        src = v.get('source', {})
        print(f'{"":16s}   source: {src}  launch: {v["launch"]["package"]} {v["launch"]["file"]}')
        for name, opt in v.get('options', {}).items():
            print(f'{"":16s}   option {name}: {", ".join(opt["choices"])} (default {opt.get("default")})')
        fol = v['followers']
        for fid, f in fol['choices'].items():
            mark = '*' if fid == fol.get('default') else ' '
            print(f'{"":16s}   follower {mark}{fid}: {f.get("label", fid)}')
    print('\nPX4 parameter profiles:')
    for name, (desc, _files, _o) in PROFILES.items():
        print(f'  {name:24s} {desc}')


def cmd_run(args):
    from .sim import Scenario, Simulation, format_log
    sc = Scenario(version=args.version, follower=args.follower, duration_s=args.duration, seed=args.seed,
                  param_profile=args.profile, speed_cap_mps=args.speed_cap)
    for opt in args.option or []:
        k, _, v = opt.partition('=')
        sc.options[k] = v
    for kv in args.set or []:
        k, _, v = kv.partition('=')
        if '.' in k:
            head, sub = k.split('.', 1)
            getattr(sc, head)[sub] = _parse_value(v)
        else:
            setattr(sc, k, _parse_value(v))
    for f in args.fault or []:
        parts = f.split(':')
        fault = {'t': float(parts[0]), 'action': parts[1]}
        if parts[1] == 'pilot':
            fault['what'] = parts[2]
            if len(parts) > 3:
                fault['value'] = _parse_value(parts[3])
        elif parts[1] in ('vo_lost',) and len(parts) > 2:
            fault['duration'] = float(parts[2])
        elif parts[1] == 'kill_node':
            fault['name'] = parts[2]
        sc.faults.append(fault)
    quiet = args.quiet

    def sink(r):
        if not quiet and r.get('level') in ('WARN', 'ERROR', 'CRITICAL', 'FATAL', 'INFO'):
            if r.get('source') == 'stdout' and not args.verbose:
                return
            if r.get('level') == 'INFO' and not args.verbose and r.get('source') not in ('px4', 'sim'):
                return
            sys.__stdout__.write(format_log(r) + '\n')
    sim = Simulation(sc, log_sink=sink)
    try:
        speed = None if args.speed in ('max', '0') else float(args.speed)
        result = sim.run(realtime=speed)
        if args.out:
            sim.save(args.out)
        ref = result['referee']
        sys.__stdout__.write('\n' + json.dumps({'version': result['version'], 'sim_time_s': result['sim_time_s'],
                                                'referee': ref, 'events': result['events'][-25:]}, indent=1) + '\n')
    finally:
        sim.close()


def cmd_gui(args):
    from .gui.app import main
    main()


def main(argv=None):
    p = argparse.ArgumentParser(prog='dronesim', description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest='cmd')
    sub.add_parser('list')
    r = sub.add_parser('run')
    r.add_argument('--version', default='traj')
    r.add_argument('--follower')
    r.add_argument('--option', action='append')
    r.add_argument('--profile', default='drone1-fleet')
    r.add_argument('--speed-cap', type=float, dest='speed_cap',
                   help="PX4's horizontal speed cap MPC_XY_VEL_MAX, m/s (default: the profile's)")
    r.add_argument('--duration', type=float, default=150.0)
    r.add_argument('--seed', type=int, default=1)
    r.add_argument('--speed', default='max')
    r.add_argument('--out')
    r.add_argument('--set', action='append', help='scenario field, e.g. start_heading_deg=90 or vo.tunnel_lighting=lit')
    r.add_argument('--fault', action='append', help='t:action[:arg], e.g. 40:vo_lost:3  50:pilot:takeover')
    r.add_argument('--quiet', action='store_true')
    r.add_argument('--verbose', action='store_true')
    sub.add_parser('gui')
    args = p.parse_args(argv)
    if args.cmd == 'list':
        cmd_list(args)
    elif args.cmd == 'run':
        cmd_run(args)
    elif args.cmd == 'gui':
        cmd_gui(args)
    else:
        p.print_help()


if __name__ == '__main__':
    main()
