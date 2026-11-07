#!/usr/bin/env python3
"""Bridge server: runs cuVSLAM on frames that arrive over TCP and sends the poses back.

Runs where the ``cuvslam`` wheel is (WSL2 Ubuntu-22.04 on the laptop, or the Jetson).  The camera
side is ``methods/cuvslam-vio/09_cuvslam_live_view.py``, which starts this script inside WSL2 by itself
(``methods/cuvslam-vio/tools/run_bridge_server.sh``); start it by hand only to serve another machine::

    source ~/venvs/cuvslam/bin/activate
    python methods/cuvslam-vio/10_cuvslam_bridge_server.py                  # one client on port 47800, then exit
    python methods/cuvslam-vio/10_cuvslam_bridge_server.py --keep-serving   # until Ctrl-C

The tracking loop, the rig and the guard are the ones of ``methods/cuvslam-vio/08_cuvslam_track.py``
(``cuvslam_oak.tracking``); the protocol is described in ``cuvslam_oak/bridge.py``.

Exit codes: 0 served (or nobody came within ``--accept-timeout``), 1 the port cannot be opened,
3 ``cuvslam`` is not importable (``python methods/cuvslam-vio/tools/check_env.py`` says why).
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import List, Optional

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import cuvslam_oak  # noqa: E402
from cuvslam_oak import bridge  # noqa: E402

EXIT_OK, EXIT_ERROR, EXIT_NO_CUVSLAM = 0, 1, 3


def log(message: str) -> None:
    print(message, flush=True)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run cuVSLAM on frames streamed by methods/cuvslam-vio/09_cuvslam_live_view.py.")
    parser.add_argument("--host", default="0.0.0.0", help="address to listen on (default %(default)s)")
    parser.add_argument("--port", type=int, default=bridge.DEFAULT_PORT, help="TCP port (default %(default)s; 0 = any free port)")
    parser.add_argument("--keep-serving", action="store_true", help="serve one client after another until Ctrl-C")
    parser.add_argument("--accept-timeout", type=float, default=120.0, metavar="SECONDS",
                        help="give up when no client connects within this time (default %(default)s; 0 = wait forever)")
    parser.add_argument("--fake-cuvslam", action="store_true",
                        help="use cuvslam_oak.fake_cuvslam (synthetic poses): a dry run of the bridge without a GPU")
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        vslam = cuvslam_oak.import_cuvslam(prefer_fake=bool(args.fake_cuvslam))
    except ImportError as exc:
        print("cuvslam is not importable: {}".format(str(exc).splitlines()[0]), file=sys.stderr, flush=True)
        return EXIT_NO_CUVSLAM
    if cuvslam_oak.is_fake(vslam):
        log("FAKE cuVSLAM in use: the poses are synthetic, NOT tracking results")
    try:
        bridge.run_server(vslam, args.host, args.port, once=not args.keep_serving, log=log,
                          accept_timeout_s=args.accept_timeout if args.accept_timeout > 0 else None)
    except OSError as exc:
        print("cannot listen on {}:{}: {}".format(args.host, args.port, exc), file=sys.stderr, flush=True)
        return EXIT_ERROR
    except KeyboardInterrupt:
        log("interrupted")
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())
