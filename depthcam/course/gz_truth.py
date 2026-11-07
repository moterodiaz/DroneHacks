"""The drone's true pose from Gazebo, on the simulation clock (SITL only; needs gz-transport's Python
bindings, which come with Gazebo Harmonic).

Subscribes to ``/world/<world>/pose/info``, the topic PX4's own gz bridge reads for its ground
truth, and to ``/world/<world>/clock``.  :meth:`GzTruth.trajectory` feeds ``lidar_sim``, so the
synthetic MS200K sees the course from where the simulated drone really is.  :meth:`GzTruth.now`
is the simulation clock.  PX4 SITL runs in lockstep with it, so using it for every timestamp (the
beams, PX4's messages, the TIMESYNC replies) keeps all of them consistent even when the simulation
runs slower than real time.

The Gazebo world is the course frame (ENU, metres), and the model pose is the body centre.
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from typing import Optional

import numpy as np

from course.lidar_sim import Trajectory


def _rpy(w: float, x: float, y: float, z: float):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


class GzTruth:
    def __init__(self, world: str, model: str, keep_s: float = 3.0) -> None:
        from gz.msgs10.clock_pb2 import Clock
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node
        self.model = model
        self.lock = threading.Lock()
        self.buf: deque = deque()       # sim t, x, y, z, roll, pitch, yaw
        self.keep_s = keep_s
        self._clock: Optional[tuple] = None  # (sim t, host monotonic t)
        self.node = Node()
        if not self.node.subscribe(Pose_V, "/world/{}/pose/info".format(world), self._on_pose):
            raise RuntimeError("cannot subscribe to /world/{}/pose/info".format(world))
        if not self.node.subscribe(Clock, "/world/{}/clock".format(world), self._on_clock):
            raise RuntimeError("cannot subscribe to /world/{}/clock".format(world))

    def _on_pose(self, msg) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        for p in msg.pose:
            if p.name == self.model:
                o = p.orientation
                row = (t, p.position.x, p.position.y, p.position.z) + _rpy(o.w, o.x, o.y, o.z)
                with self.lock:
                    if not self.buf or t > self.buf[-1][0]:
                        self.buf.append(row)
                    while self.buf and self.buf[0][0] < t - self.keep_s:
                        self.buf.popleft()
                return

    def _on_clock(self, msg) -> None:
        self._clock = (msg.sim.sec + msg.sim.nsec * 1e-9, time.monotonic())

    def wait(self, timeout_s: float = 30.0) -> bool:
        """True once the clock and the drone's pose are both arriving."""
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            if self._clock is not None and self.latest_t() is not None:
                return True
            time.sleep(0.05)
        return False

    def now(self) -> float:
        """Simulation time now: the last clock tick plus the wall time since it (capped at 50 ms)."""
        sim, host = self._clock
        return sim + min(time.monotonic() - host, 0.05)

    def latest_t(self) -> Optional[float]:
        with self.lock:
            return self.buf[-1][0] if self.buf else None

    def trajectory(self) -> Trajectory:
        with self.lock:
            a = np.array(self.buf)
        return Trajectory(a[:, 0], a[:, 1:4], a[:, 4:7])
