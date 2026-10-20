"""Vehicle mass properties (spec/README.md section 4), mirror of src/flight/massprops.js.

The kit is folded once into first/second moments about the body origin; the gas enters linearly. `at(mg)` works
on floats or numpy arrays of gas mass (batched twin) with the same expression order as the JS version.
Symmetric 3x3 tensors are 6-tuples (xx, yy, zz, xy, xz, yz) in body axes about the COM.
"""
from __future__ import annotations

import math

import numpy as np

from .spec import PROPULSION, GENERATED


def _add_point(J, m, x):
    xx, yy, zz = x[0] * x[0], x[1] * x[1], x[2] * x[2]
    J[0] += m * (yy + zz)
    J[1] += m * (xx + zz)
    J[2] += m * (xx + yy)
    J[3] -= m * (x[0] * x[1])
    J[4] -= m * (x[0] * x[2])
    J[5] -= m * (x[1] * x[2])


def _add_cylinder(J, m, b):
    ax = b['axis']
    al = math.sqrt(ax[0] * ax[0] + ax[1] * ax[1] + ax[2] * ax[2])
    a = (ax[0] / al, ax[1] / al, ax[2] / al)
    r = 0.5 * b['diameter']
    Ia = m * (0.5 * r * r)
    It = m * ((3 * r * r + b['length'] * b['length']) / 12)
    d = Ia - It
    J[0] += It + d * a[0] * a[0]
    J[1] += It + d * a[1] * a[1]
    J[2] += It + d * a[2] * a[2]
    J[3] += d * a[0] * a[1]
    J[4] += d * a[0] * a[2]
    J[5] += d * a[1] * a[2]


class MassModel:
    def __init__(self, prop: dict = PROPULSION, mp: dict = GENERATED['massprops']):
        L = mp['lattice']
        Jd, sd = [0.0] * 6, [0.0] * 3
        md = [0.0]

        def point(m, x):
            md[0] += m
            sd[0] += m * x[0]
            sd[1] += m * x[1]
            sd[2] += m * x[2]
            _add_point(Jd, m, x)

        point(L['mass'], L['com'])
        I = L['inertia']
        Jd[0] += I[0][0]
        Jd[1] += I[1][1]
        Jd[2] += I[2][2]
        Jd[3] += I[0][1]
        Jd[4] += I[0][2]
        Jd[5] += I[1][2]
        for b in prop['tank']['bottles']:
            point(b['dry_mass'], b['center'])
            _add_cylinder(Jd, b['dry_mass'], b)
        for z in prop['nozzles']:
            point(z['mass'], z['mount'])
        for h in prop['hardware']:
            point(h['mass'], h['center'])
        V = 0.0
        for b in prop['tank']['bottles']:
            V += b['volume']
        Jg, sg = [0.0] * 6, [0.0] * 3
        for b in prop['tank']['bottles']:
            w = b['volume'] / V
            sg[0] += w * b['center'][0]
            sg[1] += w * b['center'][1]
            sg[2] += w * b['center'][2]
            _add_point(Jg, w, b['center'])
            _add_cylinder(Jg, w, b)
        self.m_dry, self.s_dry, self.J_dry, self.s_gas, self.J_gas = md[0], tuple(sd), tuple(Jd), tuple(sg), tuple(Jg)

    def at(self, mg):
        """-> (M, (cx, cy, cz), I6, Iinv6) for gas mass mg (float or ndarray)."""
        M = self.m_dry + mg
        s, sg, J, Jg = self.s_dry, self.s_gas, self.J_dry, self.J_gas
        cx = (s[0] + mg * sg[0]) / M
        cy = (s[1] + mg * sg[1]) / M
        cz = (s[2] + mg * sg[2]) / M
        I = ((J[0] + mg * Jg[0]) - M * (cy * cy + cz * cz),
             (J[1] + mg * Jg[1]) - M * (cx * cx + cz * cz),
             (J[2] + mg * Jg[2]) - M * (cx * cx + cy * cy),
             (J[3] + mg * Jg[3]) + M * (cx * cy),
             (J[4] + mg * Jg[4]) + M * (cx * cz),
             (J[5] + mg * Jg[5]) + M * (cy * cz))
        return M, (cx, cy, cz), I, inv_sym(I)


def inv_sym(I):
    a, b, c, d, e, f = I
    A = b * c - f * f
    B = e * f - d * c
    C = d * f - e * b
    det = a * A + d * B + e * C
    return (A / det, (a * c - e * e) / det, (a * b - d * d) / det, B / det, C / det, (d * e - a * f) / det)


__all__ = ['MassModel', 'inv_sym', 'np']
