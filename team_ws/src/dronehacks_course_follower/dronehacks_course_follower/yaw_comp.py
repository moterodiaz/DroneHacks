"""Feed-forward against a push fixed to the airframe, which PX4 cancels in world axes.

PX4's velocity integrator learns a steady push as a world-frame acceleration. A push fixed to the
airframe turns with the nose, so after a yaw the learned correction points the old way and the
drone drifts until the integrator relearns it. This models the integrator as a first-order lag L
of the world push R(heading) d, and moves the pose setpoint by o = -(R d - L) / K, so PX4's
proportional path supplies the part not yet relearned.

Frames: the world is odom ENU (east = x, north = y), headings counter-clockwise from +x in radians;
the push d is in the nose's yaw-only frame (forward, left), m/s². PX4's accelerometer
(/fmu/out/sensor_combined accelerometer_m_s2) is FRD, so forward = ax and left = -ay; in a steady
hover its horizontal reading, times `push_gain`, is d.
"""
from __future__ import annotations

import math
from typing import NamedTuple

# Defaults fitted on flights 20261108-013018 and 003042 against stock PX4 gains: the integrator's
# lag MPC_XY_VEL_P_ACC 1.8 / MPC_XY_VEL_I_ACC 0.4, the pose-to-acceleration gain MPC_XY_P 0.95 x
# MPC_XY_VEL_P_ACC 1.8, the push per unit of hover accelerometer reading, and the offset cap
DEFAULT_TAU_S = 4.5
DEFAULT_GAIN_PER_S2 = 1.71
DEFAULT_PUSH_GAIN = 1.25
DEFAULT_MAX_M = 0.20

# The first estimate averages this much steady flight
MEASURE_S = 2.5

# Once estimated, steady flight refines it with this time constant
REFINE_TAU_S = 30.0

# Steady: slow enough that rotor drag stays small next to the push, and not turning
STEADY_SPEED_MPS = 0.2
STEADY_YAW_RATE_DPS = 5.0

# An accelerometer reading older than this counts as no IMU
IMU_STALE_S = 0.5

# The horizontal speed the steady gate reads is low-passed over this, since odometry arrives
# slower than the ticks and repeats
SPEED_TAU_S = 0.3


class Vector(NamedTuple):
    """A horizontal vector: (x, y) in whichever frame its name gives."""

    x: float
    y: float


def rotated(yaw_rad: float, forward: float, left: float) -> Vector:
    """The body vector (forward, left) in the world frame (east, north) at heading `yaw_rad`."""
    cos, sin = math.cos(yaw_rad), math.sin(yaw_rad)
    return Vector(cos * forward - sin * left, sin * forward + cos * left)


def _wrap(angle_rad: float) -> float:
    """The angle wrapped into [-pi, pi)."""
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


class YawCompensator:
    """Estimates the airframe's push in steady flight, then gives the setpoint offset each tick."""

    def __init__(self, tau_s: float = DEFAULT_TAU_S, gain_per_s2: float = DEFAULT_GAIN_PER_S2,
                 max_m: float = DEFAULT_MAX_M, push_gain: float = DEFAULT_PUSH_GAIN) -> None:
        """Keeps PX4's integrator lag `tau_s`, its gain, the offset cap and the push gain; no push yet."""
        if tau_s <= 0.0 or gain_per_s2 <= 0.0 or max_m < 0.0:
            raise ValueError(f"yaw_comp tau and gain must be positive and the cap not negative, not {tau_s}, "
                             f"{gain_per_s2}, {max_m}")

        self.tau_s = tau_s
        self.gain_per_s2 = gain_per_s2
        self.max_m = max_m
        self.push_gain = push_gain
        self.push_body: Vector | None = None
        self.learned = Vector(0.0, 0.0)
        self.offset = Vector(0.0, 0.0)
        self._reading = Vector(0.0, 0.0)
        self._measure_sum = Vector(0.0, 0.0)
        self._measured_s = 0.0
        self._accel_sum = Vector(0.0, 0.0)
        self._accel_count = 0
        self._imu_at_s = -math.inf
        self._previous_s: float | None = None
        self._previous_yaw_rad: float | None = None

    @property
    def ready(self) -> bool:
        """Whether the push has been estimated."""
        return self.push_body is not None

    def imu_fresh(self, now_s: float) -> bool:
        """Whether an accelerometer reading arrived within `IMU_STALE_S` of the last step."""
        return now_s - self._imu_at_s <= IMU_STALE_S

    def add_accel(self, ax_frd: float, ay_frd: float) -> None:
        """Adds one accelerometer sample, FRD m/s², to the mean the next step reads."""
        self._accel_sum = Vector(self._accel_sum.x + ax_frd, self._accel_sum.y - ay_frd)
        self._accel_count += 1

    def step(self, now_s: float, yaw_rad: float, speed_mps: float, airborne: bool = True) -> Vector:
        """The setpoint offset to add now, odom ENU metres; zero until the push is estimated.

        `yaw_rad` is the measured heading and `speed_mps` the horizontal speed; the push is learned
        only `airborne` while steady (slow and not turning), when the reading is the push alone."""
        elapsed_s = 0.0 if self._previous_s is None else max(0.0, now_s - self._previous_s)
        yaw_rate_dps = 0.0
        if elapsed_s > 0.0 and self._previous_yaw_rad is not None:
            yaw_rate_dps = math.degrees(_wrap(yaw_rad - self._previous_yaw_rad)) / elapsed_s
        self._previous_s, self._previous_yaw_rad = now_s, yaw_rad
        reading = self._take_reading(now_s)

        # Learn the push from steady flight only: anywhere else the reading holds the drone's own acceleration
        steady = airborne and speed_mps < STEADY_SPEED_MPS and abs(yaw_rate_dps) < STEADY_YAW_RATE_DPS
        if reading is not None and steady:
            self._learn(reading, elapsed_s, yaw_rad)

        if self.push_body is None:
            return self.offset

        # PX4's integrator follows the world push with its lag; the unlearned part is what drifts the drone
        push = rotated(yaw_rad, *self.push_body)
        share = min(1.0, elapsed_s / self.tau_s)
        self.learned = Vector(self.learned.x + share * (push.x - self.learned.x),
                              self.learned.y + share * (push.y - self.learned.y))
        self.offset = _capped(Vector(-(push.x - self.learned.x) / self.gain_per_s2,
                                     -(push.y - self.learned.y) / self.gain_per_s2), self.max_m)
        return self.offset

    def _take_reading(self, now_s: float) -> Vector | None:
        """The mean body (forward, left) accelerometer reading since the last step, or None."""
        if self._accel_count == 0:
            return None

        reading = Vector(self._accel_sum.x / self._accel_count, self._accel_sum.y / self._accel_count)
        self._accel_sum, self._accel_count, self._imu_at_s = Vector(0.0, 0.0), 0, now_s
        return reading

    def _learn(self, reading: Vector, elapsed_s: float, yaw_rad: float) -> None:
        """Averages the first `MEASURE_S` of steady reading, then low-passes it; at the first estimate
        PX4 has had the whole hover to learn the push, so nothing is left unlearned."""
        if self.push_body is None:
            self._measure_sum = Vector(self._measure_sum.x + reading.x * elapsed_s,
                                       self._measure_sum.y + reading.y * elapsed_s)
            self._measured_s += elapsed_s
            if self._measured_s >= MEASURE_S:
                self._reading = Vector(self._measure_sum.x / self._measured_s, self._measure_sum.y / self._measured_s)
                self.push_body = Vector(self.push_gain * self._reading.x, self.push_gain * self._reading.y)
                self.learned = rotated(yaw_rad, *self.push_body)
            return

        share = min(1.0, elapsed_s / REFINE_TAU_S)
        self._reading = Vector(self._reading.x + share * (reading.x - self._reading.x),
                               self._reading.y + share * (reading.y - self._reading.y))
        self.push_body = Vector(self.push_gain * self._reading.x, self.push_gain * self._reading.y)


class HorizontalSpeed:
    """The drone's horizontal speed from successive positions, low-passed over `SPEED_TAU_S`."""

    def __init__(self) -> None:
        """No position yet: the speed reads infinite, never steady, until two have arrived."""
        self.speed_mps = math.inf
        self._previous: tuple[float, float, float] | None = None

    def update(self, now_s: float, x: float, y: float) -> float:
        """The speed after the position (`x`, `y`) metres at `now_s`."""
        if self._previous is not None and now_s > self._previous[0]:
            elapsed_s = now_s - self._previous[0]
            instant_mps = math.hypot(x - self._previous[1], y - self._previous[2]) / elapsed_s
            share = min(1.0, elapsed_s / SPEED_TAU_S)
            self.speed_mps = instant_mps if math.isinf(self.speed_mps) else \
                self.speed_mps + share * (instant_mps - self.speed_mps)
        self._previous = (now_s, x, y)
        return self.speed_mps


def _capped(offset: Vector, max_m: float) -> Vector:
    """`offset` shortened to `max_m` where it is longer."""
    length_m = math.hypot(*offset)
    if length_m <= max_m:
        return offset

    return Vector(offset.x * max_m / length_m, offset.y * max_m / length_m)
