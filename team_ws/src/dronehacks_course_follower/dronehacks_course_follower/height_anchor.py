"""PX4's height paired with the odometry's at the moment the camera saw it: the setpoint's height offset.

PX4 holds height by its own estimate, whose reference is the downward rangefinder with no terrain
model: a boulder, a window sill or a wall top under the beam reads as the drone dropping, and PX4
climbs, or it drifts on the barometer once EKF2 drops the rangefinder. The tracker's z sees none of
it. So the pose setpoint's height offset, PX4's height minus the tracker's, is taken from PX4's own
local position at its full rate and followed fast: the setpoint moves with PX4's error and the
drone stays at the tracker's height.

The odometry runs behind PX4 by the tracker's latency, so in a climb PX4's height now minus the
last odometry would read the climb as offset. Each odometry sample is paired instead with PX4's
height when its image was taken. Its stamp is on the node's clock on the drone (the camera driver's
ROS time) and on PX4's in the simulator (sim time is PX4's clock); PX4's heights are kept by both,
their arrival on the node's clock and PX4's own sample stamp. Times are seconds, heights metres up.
"""
from __future__ import annotations

from collections import deque
from enum import Enum
from typing import NamedTuple

import numpy as np

# PX4 heights kept for pairing: longer than any tracker latency the pairing accepts
HISTORY_S = 1.0

# An odometry stamp further behind than this is not on that clock: no tracker runs this late
LAG_MAX_S = 0.5

# How far a stamp may sit past PX4's newest height and still be paired with it: PX4's estimate
# runs ahead of the camera, so this only covers the gap between PX4's samples at the bridge
AHEAD_S = 0.1

# The anchor is off once PX4's height or the last pairing is older than this, as the odometry is
# stale at the same age: the offset then falls back to /skybeta/state
STALE_S = 0.5

# The local position topic on the PX4 v1.16 bridge; a message at version N > 0 adds `_v<N>`
LOCAL_POSITION_TOPIC = "/fmu/out/vehicle_local_position"


class Alignment(str, Enum):
    """Which clock an odometry stamp is on, so which of PX4's times it is paired by."""

    NODE = "the node's clock"        # the drone: paired with PX4's height as it arrived
    PX4 = "PX4's clock"              # the simulator: paired by PX4's own sample stamp
    ARRIVAL = "neither clock"        # paired on arrival, so a climb reads as offset by the latency


class Px4Height(NamedTuple):
    """One of PX4's height estimates."""

    arrival_s: float          # when it reached the node, on the node's clock
    sample_s: float           # PX4's sample stamp, on PX4's clock
    up_m: float               # PX4's local z, up


class HeightAnchor:
    """PX4's height offset from the odometry, as of the last odometry PX4 has a height for."""

    def __init__(self) -> None:
        """Starts with no heights and no offset."""
        self.heights: deque[Px4Height] = deque()
        self.offset_m: float | None = None
        self.paired_at_s = -float("inf")
        self.alignment: Alignment | None = None
        self.lag_s = float("nan")

    def add_px4(self, arrival_s: float, sample_s: float, up_m: float) -> None:
        """Keeps one PX4 height, forgetting those older than the history."""
        self.heights.append(Px4Height(arrival_s, sample_s, up_m))
        while arrival_s - self.heights[0].arrival_s > HISTORY_S:
            self.heights.popleft()

    def pair(self, arrival_s: float, stamp_s: float, odom_up_m: float) -> float | None:
        """PX4's height minus the odometry's `odom_up_m` when its image was taken at `stamp_s`, kept
        as the offset; None, keeping the last, while PX4 has no height for that moment."""
        if not self.heights:
            return None

        # The capture on whichever clock the stamp is on, and PX4's heights on that clock
        alignment, times = self._placed(arrival_s, stamp_s)
        capture_s = arrival_s if alignment is Alignment.ARRIVAL else stamp_s
        if not times[0] <= capture_s <= times[-1] + AHEAD_S:
            return None

        px4_up_m = float(np.interp(capture_s, times, [height.up_m for height in self.heights]))
        self.offset_m = px4_up_m - odom_up_m
        self.paired_at_s = arrival_s
        self.alignment = alignment
        self.lag_s = (times[-1] if alignment is Alignment.PX4 else arrival_s) - capture_s
        return self.offset_m

    def _placed(self, arrival_s: float, stamp_s: float) -> tuple[Alignment, list[float]]:
        """Which clock `stamp_s` is on, judged by how far behind each clock's now it sits, and PX4's
        height times on that clock."""
        newest = self.heights[-1]
        if -AHEAD_S <= arrival_s - stamp_s <= LAG_MAX_S:
            return Alignment.NODE, [height.arrival_s for height in self.heights]

        if -AHEAD_S <= newest.sample_s - stamp_s <= LAG_MAX_S:
            return Alignment.PX4, [height.sample_s for height in self.heights]

        return Alignment.ARRIVAL, [height.arrival_s for height in self.heights]

    def offset(self, now_s: float) -> float | None:
        """The offset to follow now, or None while PX4's height or the pairing is stale."""
        fresh = (self.heights and now_s - self.heights[-1].arrival_s <= STALE_S
                 and now_s - self.paired_at_s <= STALE_S)
        return self.offset_m if fresh else None


def local_position_topic() -> tuple[type, str] | None:
    """PX4's local position message and its topic, resolved as the kit's interface resolves its
    own from the installed px4_msgs; None when px4_msgs cannot be imported."""
    try:
        from px4_msgs.msg import VehicleLocalPosition
    except ImportError:
        return None

    version = int(getattr(VehicleLocalPosition, "MESSAGE_VERSION", 0))
    return VehicleLocalPosition, LOCAL_POSITION_TOPIC + (f"_v{version}" if version > 0 else "")
