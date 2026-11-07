"""The per-run lidar policy.

The 360 lidar is allowed in some scored runs and not in others, so each run states its policy
explicitly. There is no default. A missing or misspelt value is an error, never a silent choice:

* ``allowed``   - lidar fixes correct the visual odometry;
* ``forbidden`` - the lidar is never subscribed, and the odometry passes through unchanged.
"""

from __future__ import annotations

import enum


class LidarPolicy(enum.Enum):
    ALLOWED = "allowed"
    FORBIDDEN = "forbidden"


VALID_VALUES = tuple(member.value for member in LidarPolicy)


def parse_lidar_policy(value: object) -> LidarPolicy:
    """The :class:`LidarPolicy` named by *value*, which must be exactly ``"allowed"`` or ``"forbidden"``.

    Matching is strict (lower case, no surrounding spaces), so a typo can't quietly pick a mode.
    """
    if not isinstance(value, str) or value not in VALID_VALUES:
        raise ValueError(
            "lidar_policy must be set explicitly for every run to one of {} (got {!r}); "
            "there is no default".format(", ".join(repr(v) for v in VALID_VALUES), value))
    return LidarPolicy(value)
