"""Heuristic attitude + descent autopilot, vectorised mirror of src/flight/controllers/pd.js (parity-tested).

Every branch of the JS code is evaluated for all vehicles and selected with np.where, which yields the same IEEE
results lane by lane. Only + - * / sqrt min max are used.
"""
from __future__ import annotations

import numpy as np

from . import DEFAULTS
from ..contract import GROUPS, NU, CONTROL_DT
from ..massprops import MassModel, inv_sym
from ..obs import height_np
from ..propulsion import Propulsion
from ..spec import PROPULSION, WORLD
from ..state import batch_of


class PD:
    name = 'pd'

    def __init__(self, **cfg):
        c = {k: v for k, v in DEFAULTS['pd'].items() if not k.startswith('_')}
        c.update(cfg)
        self.config = c
        self.P = Propulsion(PROPULSION, WORLD['air']['pressure'])
        self.mass = MassModel()
        self.B = GROUPS['bottom']
        self.ex = [PROPULSION['nozzles'][i]['exit'] for i in self.B]
        self.g = WORLD['gravity']
        self.reset_batch(1, None)

    def reset(self, task=None, seed=0):
        self.reset_batch(1, task)

    def reset_batch(self, n, task=None, idx=None):
        """(Re)initialise the per-vehicle memory (hover hold timer, landing flag) for n vehicles or lanes idx."""
        self.task = task
        hover = bool(task and task.get('target'))
        if idx is None:
            self.hold_t = np.zeros(n)
            self.landing = np.full(n, not hover)
        else:
            self.hold_t[idx] = 0.0
            self.landing[idx] = not hover

    def act(self, state, info=None):
        return self.act_batch(batch_of(state))[0]

    def act_batch(self, s, idx=None):
        """s: batch state (coldgas.state); idx: lanes of the controller memory that s refers to (default all)."""
        c, P, g, reg = self.config, self.P, self.g, PROPULSION['regulator']
        sel = slice(None) if idx is None else idx
        hold_t, landing = self.hold_t[sel], self.landing[sel]
        q = s['q']
        qx, qy, qz, qw = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
        ux = 2 * (qx * qy + qz * qw)
        uy = 1 - 2 * (qx * qx + qz * qz)
        uz = 2 * (qy * qz - qx * qw)
        armed = np.asarray(s['armed'], dtype=bool)
        h = height_np(s['p'], q, s['comB'])
        T = self.task
        H = T['target']['height'] if T and T.get('target') else 0.0
        if T and T.get('target'):
            tg = T['target']
            upd = armed & ~landing
            ht = np.where(np.abs(h - tg['height']) <= tg['tolerance'], hold_t + CONTROL_DT, 0.0)
            hold_t = np.where(upd, ht, hold_t)
            landing = landing | (upd & (hold_t >= tg['hold'] - 1e-9))
            self.hold_t[sel], self.landing[sel] = hold_t, landing
        valid = armed & ~(landing & (s['phase'] == 2))
        pc = np.minimum(reg['p_set'], s['tank_p'] - reg['dp_min'])
        pc = np.where(pc > 2 * P.pa, pc, 0.0)
        valid &= pc > 0
        Tc = s['tank_T']
        with np.errstate(all='ignore'):
            F1, Fa = [], np.zeros(len(pc))
            for i in self.B:
                md = P.At[i] * pc * P.choke / np.sqrt(Tc)
                ve = P.Me[i] * np.sqrt(P.g * P.R * Tc * P.TeRatio[i])
                f1 = md * ve + (pc * P.peRatio[i] - P.pa) * P.Ae[i]
                valid &= f1 > 0  # over-expanded at a nearly empty tank: the nozzles no longer push
                F1.append(f1)
                Fa = Fa + f1
            M, _c, I, _Ii = self.mass.at(s['tank_m'])
            # attitude
            sn = np.sqrt(ux * ux + uz * uz)
            kk = np.where(uy < 0, np.where(sn > 1e-6, (2 - sn) / sn, 0.0), 1.0)
            flip = (uy < 0) & (sn <= 1e-6)
            ea = np.where(flip, 0.0, uz * kk)
            eb = np.where(flip, 2.0, -ux * kk)
            w = s['w']
            tx = I[0] * (c['att_kp'] * ea - c['att_kd'] * w[:, 0])
            tz = I[2] * (c['att_kp'] * eb - c['att_kd'] * w[:, 2])
            # collective
            vy = s['v'][:, 1]
            vh = c['hover_kh'] * (H - h)
            vh = np.where(vh > c['hover_vmax'], c['hover_vmax'], np.where(vh < -c['hover_vmax'], -c['hover_vmax'], vh))
            amax = Fa * uy / M - g
            ab = c['brake_frac'] * np.where(amax > 0.5, amax, 0.5)
            hh = h - c['flare_height']
            vl = -np.sqrt(c['td_speed'] * c['td_speed'] + 2 * ab * np.where(hh > 0, hh, 0.0))
            aff = (ab * vy) / vl
            aff = np.where(aff > 2 * ab, 2 * ab, aff)
            aff = np.where(landing & (hh > 0) & (vy < 0), aff, 0.0)
            vref = np.where(landing, vl, vh)
            a = g + aff + c['kv'] * (vref - vy)
            Fc = (M * np.where(a > 0, a, 0.0)) / uy
            Fc = np.where((uy > c['tilt_cut']) & ~(landing & (h <= 0)), Fc, 0.0)
            # allocation
            comB = s['comB']
            nb = len(self.B)
            z0 = np.zeros(len(pc))
            xs, zs = [], []
            sz, szz, sx, sxx, sxz = z0, z0, z0, z0, z0
            for k in range(nb):
                x = self.ex[k][0] - comB[:, 0]
                z = self.ex[k][2] - comB[:, 2]
                xs.append(x)
                zs.append(z)
                sz, szz, sx, sxx, sxz = sz + z, szz + z * z, sx + x, sxx + x * x, sxz + x * z
            Ai = inv_sym((z0 + nb, szz, sxx, -sz, sx, -sxz))
            y0 = Ai[0] * Fc + Ai[3] * tx + Ai[4] * tz
            y1 = Ai[3] * Fc + Ai[1] * tx + Ai[5] * tz
            y2 = Ai[4] * Fc + Ai[5] * tx + Ai[2] * tz
            lam, F = z0, []
            for k in range(nb):
                Fk = y0 - zs[k] * y1 + xs[k] * y2
                wk = Ai[0] - zs[k] * Ai[3] + xs[k] * Ai[4]
                l = -Fk / wk
                lam = np.where((Fk < 0) & (wk > 0) & (l > lam), l, lam)
                F.append(Fk)
            sc = z0 + 1.0
            for k in range(nb):
                F[k] = F[k] + lam * (Ai[0] - zs[k] * Ai[3] + xs[k] * Ai[4])
                r = F1[k] / F[k]
                sc = np.where((F[k] > F1[k]) & (r < sc), r, sc)
            u = np.zeros((len(pc), NU))
            for k in range(nb):
                v = (F[k] * sc) / F1[k]
                u[:, self.B[k]] = np.where(v > 1, 1.0, np.where(v < 0, 0.0, v))
        return np.where(valid[:, None], u, 0.0)
