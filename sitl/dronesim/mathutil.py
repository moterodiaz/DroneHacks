"""Small vector / quaternion helpers on plain floats (fast in the 500 Hz loop).

Quaternions are Hamilton ``(w, x, y, z)`` and rotate body vectors into the
world: ``v_world = q * v_body * q^-1`` -- the PX4 ``matrix::Quatf`` convention.
Frames inside the simulator core are PX4's: world NED, body FRD.
"""
import math

G = 9.80665          # CONSTANTS_ONE_G in PX4


def add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def scale(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def norm(a):
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def normalized(a):
    n = norm(a)
    return (a[0] / n, a[1] / n, a[2] / n) if n > 1e-12 else (0.0, 0.0, 0.0)


def finite3(a):
    return math.isfinite(a[0]) and math.isfinite(a[1]) and math.isfinite(a[2])


def constrain(v, lo, hi):
    # PX4 math::constrain: (val < min) ? min : ((val > max) ? max : val)
    return lo if v < lo else (hi if v > hi else v)


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


# -- quaternions ---------------------------------------------------------------
def q_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def q_conj(q):
    return (q[0], -q[1], -q[2], -q[3])


def q_norm(q):
    n = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


def q_canonical(q):
    if q[0] < 0 or (q[0] == 0 and (q[1] < 0 or (q[1] == 0 and (q[2] < 0 or (q[2] == 0 and q[3] < 0))))):
        return (-q[0], -q[1], -q[2], -q[3])
    return q


def q_rotate(q, v):
    """Rotate vector v (body) by q into the world."""
    w, x, y, z = q
    # t = 2 * cross(q.xyz, v)
    tx = 2.0 * (y * v[2] - z * v[1])
    ty = 2.0 * (z * v[0] - x * v[2])
    tz = 2.0 * (x * v[1] - y * v[0])
    return (v[0] + w * tx + (y * tz - z * ty),
            v[1] + w * ty + (z * tx - x * tz),
            v[2] + w * tz + (x * ty - y * tx))


def q_rotate_inv(q, v):
    return q_rotate(q_conj(q), v)


def q_dcm_z(q):
    """Third column of the rotation matrix: body z axis in the world."""
    w, x, y, z = q
    return (2.0 * (w * y + x * z), 2.0 * (y * z - w * x), w * w - x * x - y * y + z * z)


def q_from_euler(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy)


def q_to_euler(q):
    w, x, y, z = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    s = 2.0 * (w * y - z * x)
    pitch = math.asin(constrain(s, -1.0, 1.0))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def q_yaw(q):
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def q_from_two_vectors(src, dst):
    """Shortest-arc rotation taking unit vector src onto dst (matrix::Quaternion(src, dst))."""
    eps = 1e-5
    cr = cross(src, dst)
    d = dot(src, dst)
    if norm(cr) < eps and d < 0:
        # opposite vectors: rotate 180 deg about any orthogonal axis
        ax = (abs(src[0]), abs(src[1]), abs(src[2]))
        if ax[0] < ax[1]:
            if ax[0] < ax[2]:
                other = (1.0, 0.0, 0.0)
            else:
                other = (0.0, 0.0, 1.0)
        else:
            if ax[1] < ax[2]:
                other = (0.0, 1.0, 0.0)
            else:
                other = (0.0, 0.0, 1.0)
        axis = normalized(cross(src, other))
        return (0.0, axis[0], axis[1], axis[2])
    q = (d + math.sqrt(dot(src, src) * dot(dst, dst)), cr[0], cr[1], cr[2])
    return q_norm(q)


def q_from_dcm_columns(bx, by, bz):
    """Quaternion of the rotation whose matrix columns are bx, by, bz."""
    m00, m10, m20 = bx
    m01, m11, m21 = by
    m02, m12, m22 = bz
    tr = m00 + m11 + m22
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return q_norm((0.25 * s, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s))
    if m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2
        return q_norm(((m21 - m12) / s, 0.25 * s, (m01 + m10) / s, (m02 + m20) / s))
    if m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2
        return q_norm(((m02 - m20) / s, (m01 + m10) / s, 0.25 * s, (m12 + m21) / s))
    s = math.sqrt(1.0 + m22 - m00 - m11) * 2
    return q_norm(((m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, 0.25 * s))


def q_integrate(q, w, dt):
    """q(t+dt) for body rate w (rad/s), exact for constant w."""
    wn = norm(w)
    if wn * dt < 1e-9:
        return q
    half = 0.5 * wn * dt
    s = math.sin(half) / wn
    dq = (math.cos(half), w[0] * s, w[1] * s, w[2] * s)
    return q_norm(q_mul(q, dq))


# -- frame conversions -----------------------------------------------------------
def ned_to_enu(v):
    return (v[1], v[0], -v[2])


enu_to_ned = ned_to_enu


def frd_to_flu(v):
    return (v[0], -v[1], -v[2])


flu_to_frd = frd_to_flu

_NED_ENU_Q = (0.0, math.sqrt(0.5), math.sqrt(0.5), 0.0)
_AIRCRAFT_BASELINK_Q = (0.0, 1.0, 0.0, 0.0)


def q_ned_frd_to_enu_flu(q):
    """PX4 attitude -> ROS attitude (same composition as px4_ros_com)."""
    return q_mul(q_mul(_NED_ENU_Q, q), _AIRCRAFT_BASELINK_Q)


q_enu_flu_to_ned_frd = q_ned_frd_to_enu_flu


class LowPass1:
    """First-order low-pass filter (PX4 AlphaFilter with a cutoff frequency)."""

    __slots__ = ('cutoff_hz', 'state', 'initialized')

    def __init__(self, cutoff_hz):
        self.cutoff_hz = cutoff_hz
        self.state = None
        self.initialized = False

    def reset(self, value):
        self.state = value
        self.initialized = True

    def update(self, value, dt):
        if not self.initialized or self.cutoff_hz <= 0:
            self.reset(value)
            return value
        rc = 1.0 / (2.0 * math.pi * self.cutoff_hz)
        a = dt / (rc + dt)
        if isinstance(value, tuple):
            s = self.state
            self.state = tuple(s[i] + a * (value[i] - s[i]) for i in range(len(value)))
        else:
            self.state = self.state + a * (value - self.state)
        return self.state


class LowPass2:
    """Second-order Butterworth low-pass (PX4 LowPassFilter2p), per axis."""

    def __init__(self, sample_hz, cutoff_hz):
        self.set(sample_hz, cutoff_hz)

    def set(self, sample_hz, cutoff_hz):
        self.cutoff = cutoff_hz
        if cutoff_hz <= 0 or cutoff_hz >= sample_hz / 2:
            self.b0, self.b1, self.b2, self.a1, self.a2 = 1.0, 0.0, 0.0, 0.0, 0.0
        else:
            fr = sample_hz / cutoff_hz
            ohm = math.tan(math.pi / fr)
            c = 1.0 + 2.0 * math.cos(math.pi / 4.0) * ohm + ohm * ohm
            self.b0 = ohm * ohm / c
            self.b1 = 2.0 * self.b0
            self.b2 = self.b0
            self.a1 = 2.0 * (ohm * ohm - 1.0) / c
            self.a2 = (1.0 - 2.0 * math.cos(math.pi / 4.0) * ohm + ohm * ohm) / c
        self.d1 = None
        self.d2 = None

    def reset(self, value):
        dval = tuple(v / (self.b0 + self.b1 + self.b2) for v in value)
        self.d1 = dval
        self.d2 = dval

    def apply(self, value):
        if self.d1 is None:
            self.reset(value)
        out = []
        d1, d2 = list(self.d1), list(self.d2)
        for i, v in enumerate(value):
            d0 = v - d1[i] * self.a1 - d2[i] * self.a2
            out.append(d0 * self.b0 + d1[i] * self.b1 + d2[i] * self.b2)
            d2[i] = d1[i]
            d1[i] = d0
        self.d1, self.d2 = tuple(d1), tuple(d2)
        return tuple(out)
