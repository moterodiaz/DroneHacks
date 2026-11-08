#!/usr/bin/env python3
"""Compare a PX4 flight controller's live parameters with a saved backup. Read only.

It never writes a parameter (AGENTS.md §3.8). It reads every parameter over MAVLink
(USB), twice, and compares them with a backup, printing every difference. Flight
counters that change on every flight are listed separately. The backup is either:
  - a raw-word JSON, compared bit for bit
    (fc-backups/2026-11-07-fc-sysid11/fc-sysid11-params-2026-11-07.json); or
  - a QGC .params file, with REAL32 compared to 1e-6 relative, because QGC prints ~8
    digits (contracts/px4-baselines/skybeta-11-2026-11-07.params).

  python fc_params_compare.py COM3 BACKUP [--expect NAME=VALUE ...]

--expect marks a difference as intended, for example the mode-switch value
(COM_FLTMODE1=100) or the hash PX4 writes when DH Bailout first registers
(COM_MODE0_HASH). The exit status is 0 only if every difference is expected.
Needs pymavlink and pyserial.
"""
import argparse
import json
import struct
import sys
import time

from pymavlink import mavutil

M = mavutil.mavlink
COUNTERS = {"COM_FLIGHT_UUID", "LND_FLIGHT_T_HI", "LND_FLIGHT_T_LO", "_HASH_CHECK"}


def connect(port):
    m = mavutil.mavlink_connection(port, baud=115200, source_system=255, source_component=190)
    t0 = time.time()
    while time.time() - t0 < 15:
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb and hb.get_srcComponent() == 1 and hb.type != M.MAV_TYPE_GCS:
            m.target_system = hb.get_srcSystem()
            return m
    raise SystemExit(f"no autopilot heartbeat on {port}")


def fetch(m):
    params, count, last = {}, None, time.time()
    m.mav.param_request_list_send(m.target_system, 1)
    while True:
        p = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if p is not None and p.get_srcComponent() == 1:
            name = p.param_id.rstrip("\x00") if isinstance(p.param_id, str) else p.param_id.decode().rstrip("\x00")
            raw = struct.unpack("<I", struct.pack("<f", p.param_value))[0]
            params[name] = {"index": p.param_index, "type": p.param_type, "raw": raw}
            count, last = p.param_count, time.time()
            continue
        if count is not None and len([n for n in params if n != "_HASH_CHECK"]) >= count:
            return params
        if time.time() - last > 2.0:
            if count is None:
                m.mav.param_request_list_send(m.target_system, 1)
            else:
                have = {v["index"] for v in params.values()}
                for i in range(count):
                    if i not in have:
                        m.mav.param_request_read_send(m.target_system, 1, b"", i)
            last = time.time()


def value(entry):
    raw = struct.pack("<I", entry["raw"])
    return struct.unpack("<f", raw)[0] if entry["type"] == M.MAV_PARAM_TYPE_REAL32 else struct.unpack("<i", raw)[0]


def load_backup(path):
    """Raw-word JSON (exact), or a QGC .params text file (REAL32 to 1e-6 relative)."""
    if path.endswith(".json"):
        return json.load(open(path))["params"], True
    params = {}
    for line in open(path):
        if line.startswith("#") or not line.strip():
            continue
        _, _, name, text, ptype = line.rstrip("\n").split("\t")
        ptype = int(ptype)
        if ptype == M.MAV_PARAM_TYPE_REAL32:
            raw = struct.unpack("<I", struct.pack("<f", float(text)))[0]
        else:
            raw = struct.unpack("<I", struct.pack("<i", int(text)))[0]
        params[name] = {"type": ptype, "raw": raw}
    return params, False


def same(b, cur, exact):
    if b["type"] != cur["type"]:
        return False
    if exact or b["type"] != M.MAV_PARAM_TYPE_REAL32:
        return b["raw"] == cur["raw"]
    x, y = value(b), value(cur)
    return abs(x - y) <= 1e-6 * max(1.0, abs(x))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port")
    ap.add_argument("backup", help="raw-word .json (exact) or QGC .params")
    ap.add_argument("--expect", action="append", default=[], metavar="NAME[=VALUE]")
    a = ap.parse_args()
    expected = {}
    for e in a.expect:
        name, _, v = e.partition("=")
        expected[name] = v or None
    backup, exact = load_backup(a.backup)
    backup.pop("_HASH_CHECK", None)
    m = connect(a.port)
    live, live2 = fetch(m), fetch(m)
    live.pop("_HASH_CHECK", None)
    unstable = sorted(n for n in live if live2.get(n, {}).get("raw") != live[n]["raw"])
    unexpected, ok_diffs, counters = [], [], []
    for name in sorted(set(backup) | set(live)):
        b, cur = backup.get(name), live.get(name)
        if b is not None and cur is not None and same(b, cur, exact):
            continue
        line = f"{name}: backup {value(b) if b else 'MISSING'} -> live {value(cur) if cur else 'MISSING'}"
        if name in COUNTERS:
            counters.append(line)
        elif name in expected and (expected[name] is None or (cur and str(value(cur)) == expected[name])):
            ok_diffs.append(line)
        else:
            unexpected.append(line)
    print(f"live {len(live)} params (sysid {m.target_system}), backup {len(backup)}")
    for title, lines in (("expected differences", ok_diffs), ("flight counters", counters),
                         ("UNEXPECTED differences", unexpected), ("changed between two reads", unstable)):
        print(f"{title}: {len(lines)}")
        for line in lines:
            print("  " + line)
    missing = [n for n in expected if not any(line.startswith(n + ":") for line in ok_diffs)]
    if missing:
        print("expected but not found: " + ", ".join(missing))
    return 0 if not unexpected and not unstable and not missing else 1


if __name__ == "__main__":
    sys.exit(main())
