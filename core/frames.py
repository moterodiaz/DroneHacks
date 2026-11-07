"""Coordinate-frame conversions defined by :mod:`contracts/frames.md`."""

from __future__ import annotations

import numpy as np


def enu_to_ned(x: float, y: float, z: float) -> tuple[float, float, float]:
    """Apply the contract's proper 3-D ENU-to-NED axis rotation."""
    return y, x, -z


def ned_to_enu(x: float, y: float, z: float) -> tuple[float, float, float]:
    """Apply the inverse NED-to-ENU axis rotation."""
    return y, x, -z


def flu_to_frd(x: float, y: float, z: float) -> tuple[float, float, float]:
    """Map body FLU coordinates to body FRD coordinates."""
    return x, -y, -z


def frd_to_flu(x: float, y: float, z: float) -> tuple[float, float, float]:
    """Map body FRD coordinates to body FLU coordinates."""
    return x, -y, -z


def _transform_inputs(point, origin_course, origin_odom, rotation):
    p = np.asarray(point, dtype=float)
    oc = np.asarray(origin_course, dtype=float)
    oo = np.asarray(origin_odom, dtype=float)
    r = np.asarray(rotation, dtype=float)
    if p.shape != (3,) or oc.shape != (3,) or oo.shape != (3,) or r.shape != (3, 3):
        raise ValueError("points must be length 3 and R_odom_course must be 3x3")
    if not all(np.all(np.isfinite(v)) for v in (p, oc, oo, r)):
        raise ValueError("transform inputs must be finite")
    if not np.allclose(r.T @ r, np.eye(3), atol=1e-7) or not np.isclose(
        np.linalg.det(r), 1.0, atol=1e-7
    ):
        raise ValueError("R_odom_course must be a proper rotation matrix")
    return p, oc, oo, r


def course_to_odom(p_course, o_course, o_odom, R_odom_course) -> np.ndarray:
    """Return ``o_odom + R_odom_course @ (p_course - o_course)``."""
    p, oc, oo, r = _transform_inputs(p_course, o_course, o_odom, R_odom_course)
    return oo + r @ (p - oc)


def odom_to_course(p_odom, o_course, o_odom, R_odom_course) -> np.ndarray:
    """Invert the measured course-to-odom rigid transform."""
    p, oc, oo, r = _transform_inputs(p_odom, o_course, o_odom, R_odom_course)
    return oc + r.T @ (p - oo)
