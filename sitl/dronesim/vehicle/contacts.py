"""Contacts between the airframe and the course (penalty springs, NED forces).

* **Landing gear**: four feet on the layered heightmap of upward surfaces
  (floor, pad, markings, boulder tops...), spring-damper normal force and
  regularised Coulomb friction.  This is what the land detector and the
  pilot's/team's landing feel.
* **Props and body**: spheres around every propeller disc rim and hub, the
  body and the camera, tested against the triangles near the vehicle (only
  when the distance field says something is within reach).  Every contact is
  reported with the solid's name and kind so scoring can count touches.
"""
from __future__ import annotations

import math

import numpy as np

from ..mathutil import cross, q_conj, q_rotate
from ..world.course import closest_points_on_triangles, course_to_ned, ned_to_course


class Contact:
    __slots__ = ('kind', 'name', 'part', 'point', 'normal', 'speed_into', 'depth')

    def __init__(self, kind, name, part, point, normal, speed_into, depth):
        self.kind, self.name, self.part = kind, name, part
        self.point, self.normal, self.speed_into, self.depth = point, normal, speed_into, depth


class ContactModel:
    BODY_K = 4000.0
    BODY_C = 45.0
    BODY_MU = 0.4

    def __init__(self, course, cfg):
        self.course = course
        self.cfg = cfg
        self.feet = [(x, y, cfg.rest_height_m) for x, y in cfg.gear_points]
        spheres = []
        r_rim = 0.014
        for idx, (px, py, _s) in enumerate(cfg.rotors):
            spheres.append(((px, py, cfg.rotor_z_m), 0.02, f'prop{idx}'))
            for k in range(10):
                a = 2 * math.pi * k / 10
                spheres.append(((px + (cfg.prop_radius_m - r_rim) * math.cos(a),
                                 py + (cfg.prop_radius_m - r_rim) * math.sin(a), cfg.rotor_z_m), r_rim, f'prop{idx}'))
        for bx in (-0.08, 0.06):
            for by in (-0.06, 0.06):
                spheres.append(((bx, by, -0.02), 0.045, 'body'))
        spheres.append(((cfg.camera_xyz_flu[0] + 0.01, 0.0, -cfg.camera_xyz_flu[2]), 0.03, 'camera'))
        self.spheres = spheres
        self.sphere_body = np.array([s[0] for s in spheres], dtype=float)
        self.sphere_r = np.array([s[1] for s in spheres], dtype=float)
        self.reach = float(max(math.hypot(p[0], p[1]) + r for p, r, _ in spheres)) + 0.05
        self._local_tris = None
        self._local_centre = None
        self._local_t = -1.0
        self.t = 0.0
        self.contacts = []
        self.feet_contacts = []
        self.max_impact = 0.0

    def height_above_support(self, pos_ned):
        c = ned_to_course(pos_ned)
        h, _ = self.course.support_below(c[0], c[1], c[2])
        return c[2] - h if math.isfinite(h) else 10.0

    def forces(self, s, dt):
        self.t += dt
        cfg = self.cfg
        q, pos, vel, w = s.q, s.pos, s.vel, s.omega
        fx = fy = fz = 0.0
        tx = ty = tz = 0.0
        feet_contacts = []
        # -- landing gear ------------------------------------------------------------------
        for foot in self.feet:
            r_w = q_rotate(q, foot)
            p = (pos[0] + r_w[0], pos[1] + r_w[1], pos[2] + r_w[2])
            c = ned_to_course(p)
            h, sid = self.course.support_below(c[0], c[1], c[2] + 0.03)
            if not math.isfinite(h):
                continue
            pen = h - c[2]
            if pen <= 0:
                continue
            wr = cross(w, foot)
            wr_w = q_rotate(q, wr)
            v = (vel[0] + wr_w[0], vel[1] + wr_w[1], vel[2] + wr_w[2])
            fn = cfg.gear_stiffness * pen + cfg.gear_damping * v[2]
            if fn <= 0:
                continue
            vt = math.hypot(v[0], v[1])
            ft_max = cfg.ground_friction * fn
            # regularised Coulomb friction; the viscous slope keeps the explicit step stable
            ft = min(ft_max, 60.0 * vt)
            fxi = -ft * v[0] / vt if vt > 1e-6 else 0.0
            fyi = -ft * v[1] / vt if vt > 1e-6 else 0.0
            fzi = -fn
            fx += fxi
            fy += fyi
            fz += fzi
            fb = q_rotate(q_conj(q), (fxi, fyi, fzi))
            tq = cross(foot, fb)
            tx += tq[0]
            ty += tq[1]
            tz += tq[2]
            kind = self.course.solid_kind[sid] if sid >= 0 else 'floor'
            name = self.course.index.solid_names[sid] if sid >= 0 else ''
            feet_contacts.append(Contact(kind, name, 'gear', c, (0.0, 0.0, 1.0), v[2], pen))
        self.feet_contacts = feet_contacts
        # -- props / body ----------------------------------------------------------------------
        contacts = []
        centre_c = ned_to_course(pos)
        if self.course.clearance(*centre_c) < self.reach + 0.1:
            if self._local_tris is None or self.t - self._local_t > 0.02 or \
                    math.dist(centre_c, self._local_centre) > 0.03:
                self._local_tris = self.course.triangles_near(centre_c, self.reach + 0.08)
                self._local_centre = centre_c
                self._local_t = self.t
            idx = self._local_tris
            if idx.size:
                cf, ct = self._sphere_contacts(q, pos, vel, w, idx, contacts)
                fx += cf[0]
                fy += cf[1]
                fz += cf[2]
                tx += ct[0]
                ty += ct[1]
                tz += ct[2]
        self.contacts = contacts
        info = {'feet': feet_contacts, 'body': contacts}
        return (fx, fy, fz), (tx, ty, tz), info

    def _sphere_contacts(self, q, pos, vel, w, idx, out):
        cfg = self.cfg
        rot = _rotmat(q)
        centres_w = (rot @ self.sphere_body.T).T + np.asarray(pos)
        # course-frame centres
        cc = centres_w[:, [1, 0, 2]] * np.array([1.0, 1.0, -1.0])
        tris = self.course.index.triangles[idx]
        a, b, c = tris[:, 0, :], tris[:, 1, :], tris[:, 2, :]
        cp = closest_points_on_triangles(cc, a, b, c)               # (P,T,3)
        d_vec = cc[:, None, :] - cp
        dist = np.linalg.norm(d_vec, axis=2)
        j = np.argmin(dist, axis=1)
        pidx = np.arange(cc.shape[0])
        dmin = dist[pidx, j]
        hit = dmin < self.sphere_r
        if not np.any(hit):
            return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
        tri_n = self.course.index.normals[idx]
        fx = fy = fz = 0.0
        tx = ty = tz = 0.0
        for i in np.nonzero(hit)[0]:
            jj = j[i]
            n_tri = tri_n[jj]
            dv = d_vec[i, jj]
            dd = dmin[i]
            if dd > 1e-6:
                n = dv / dd
                if np.dot(n, n_tri) < -0.3:      # centre behind the surface: push out along its normal
                    n = n_tri
                    dd = -dd
            else:
                n = n_tri
            pen = float(self.sphere_r[i] - dd)
            n_ned = course_to_ned(tuple(float(x) for x in n))
            r_body = tuple(self.sphere_body[i])
            r_w = q_rotate(q, r_body)
            wr = q_rotate(q, cross(w, r_body))
            v = (vel[0] + wr[0], vel[1] + wr[1], vel[2] + wr[2])
            vn = v[0] * n_ned[0] + v[1] * n_ned[1] + v[2] * n_ned[2]       # >0 moving away
            fn = self.BODY_K * pen - self.BODY_C * vn
            if fn <= 0:
                fn = 0.0
            vt = (v[0] - vn * n_ned[0], v[1] - vn * n_ned[1], v[2] - vn * n_ned[2])
            vtn = math.sqrt(vt[0] ** 2 + vt[1] ** 2 + vt[2] ** 2)
            ft = min(self.BODY_MU * fn, 200.0 * vtn)
            f = [fn * n_ned[k] - (ft * vt[k] / vtn if vtn > 1e-6 else 0.0) for k in range(3)]
            fx += f[0]
            fy += f[1]
            fz += f[2]
            fb = q_rotate(q_conj(q), tuple(f))
            tq = cross(r_body, fb)
            tx += tq[0]
            ty += tq[1]
            tz += tq[2]
            sid = int(self.course.index.tri_solid[idx[jj]])
            kind = self.course.solid_kind[sid]
            name = self.course.index.solid_names[sid]
            self.max_impact = max(self.max_impact, -vn)
            out.append(Contact(kind, name, self.spheres[i][2], tuple(float(x) for x in cc[i]),
                               tuple(float(x) for x in n), -vn, pen))
        return (fx, fy, fz), (tx, ty, tz)


def _rotmat(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
