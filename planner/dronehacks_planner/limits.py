"""Vehicle and controller limits for speed profiling and execution.

Every default records where it came from and whether a flight has confirmed
it. **None has**: there has been no flight on this airframe yet. Where sources
disagree, the most restrictive value is the default.

Sources:

* ``SN0001``: ``drone-snapshot/.../px4/params/fleet/SN0001-2026-10-15.params``,
  most likely our drone 1. ``MPC_XY_VEL_MAX`` = 1.0, ``MPC_TILTMAX_AIR`` = 25.
* ``baseline``: ``.../px4/params/00_baseline.params``. ``MPC_XY_VEL_MAX`` = 12,
  ``MPC_ACC_HOR`` = 3, ``MPC_JERK_AUTO`` = 4, ``MPC_XY_P`` = 0.95,
  ``MPC_XY_VEL_P_ACC`` = 1.8. The ``MPC_ACC_*`` and ``MPC_JERK_*`` values shape
  PX4's own trajectories in its auto modes. Offboard position setpoints
  bypass them, so here they are only a sensible reference.
* The organisers' measured median vision latency: 68.7 ms
  (``docs/field-guide.html`` on PR #5).

The organisers' PX4 parameters must not be changed (rule 8.5), so these
limits describe what the drone already does; they configure nothing on it.
Read the live ``MPC_XY_VEL_MAX`` off the drone before trusting ``v_max_mps``.
"""

from __future__ import annotations

import dataclasses
from typing import Dict, List, Tuple

_G = 9.80665


@dataclasses.dataclass(frozen=True)
class VehicleLimits:
    #: Horizontal speed cap. SN0001 MPC_XY_VEL_MAX (baseline file: 12).
    v_max_mps: float = 1.0
    #: Lateral (centripetal) acceleration on curves. Unvalidated. 1.0 rather than 2.0:
    #: in simulation it was as fast overall (less governor hold-back) and plan matched flight.
    a_lat_mps2: float = 1.0
    #: Speeding up along the path. Unvalidated; under MPC_ACC_HOR.
    a_acc_mps2: float = 2.0
    #: Braking along the path. Unvalidated; under MPC_ACC_HOR.
    a_brake_mps2: float = 2.0
    #: Combined horizontal acceleration, sqrt(a_long^2 + a_lat^2). MPC_ACC_HOR.
    #: SN0001's 25 degree tilt limit allows g*tan(25) = 4.6 m/s^2, so this binds first.
    a_horizontal_mps2: float = 3.0
    #: Jerk. MPC_JERK_AUTO (auto modes). Unvalidated for offboard.
    j_max_mps3: float = 4.0
    #: Delay from deciding to brake to braking: vision latency (0.07 s) plus
    #: up to one 10 Hz flight-interface tick (0.10 s).
    latency_s: float = 0.17
    #: Extra stopping room kept in front of the drone.
    stop_margin_m: float = 0.05
    #: PX4 position gain, MPC_XY_P (1/s).
    kp_pos: float = 0.95
    #: PX4 velocity gain, MPC_XY_VEL_P_ACC (1/s).
    kp_vel: float = 1.8
    #: MPC_TILTMAX_AIR in degrees (SN0001).
    tilt_max_deg: float = 25.0

    def stopping_distance(self, v: float) -> float:
        """v^2 / (2 a_brake) + v * latency + margin."""
        return v * v / (2.0 * self.a_brake_mps2) + v * self.latency_s + self.stop_margin_m

    def describe(self) -> List[Tuple[str, float, str]]:
        """``(name, value, source)`` for every limit, for reports."""
        return [(f.name, getattr(self, f.name), SOURCES.get(f.name, "")) for f in dataclasses.fields(self)]


SOURCES: Dict[str, str] = {
    "v_max_mps": "SN0001 MPC_XY_VEL_MAX = 1.0 (baseline file: 12); not validated, read the live value",
    "a_lat_mps2": "chosen from simulation (2.0 was no faster); not validated",
    "a_acc_mps2": "chosen under MPC_ACC_HOR = 3; not validated",
    "a_brake_mps2": "chosen under MPC_ACC_HOR = 3; not validated",
    "a_horizontal_mps2": "MPC_ACC_HOR = 3 (auto modes); not validated for offboard",
    "j_max_mps3": "MPC_JERK_AUTO = 4 (auto modes); not validated for offboard",
    "latency_s": "68.7 ms measured vision latency (organisers) + 0.1 s interface tick",
    "stop_margin_m": "chosen; not validated",
    "kp_pos": "MPC_XY_P = 0.95 (SN0001 and baseline)",
    "kp_vel": "MPC_XY_VEL_P_ACC = 1.8 (SN0001 and baseline)",
    "tilt_max_deg": "SN0001 MPC_TILTMAX_AIR = 25 (baseline file: 45)",
}


def tilt_accel_limit(limits: VehicleLimits) -> float:
    """The most horizontal acceleration PX4 will command at its tilt limit."""
    import math

    return _G * math.tan(math.radians(limits.tilt_max_deg))
