"""Rigid 6-DOF flight twin, batched over N vehicles (numpy). Mirror of src/flight/twin.js; contract: spec/README.md 5.

Every expression is evaluated in the same order as the JS version and every reduction is a sequential sum in the
documented order, so a vehicle here and the JS twin agree bit for bit (tests/parity). The scalar env is this class
with N = 1, so batched and scalar results are identical by construction. Culled boxes / masked lanes contribute
exact zeros, which leaves sums unchanged.

State layout: S (13, N) rows [p(3), q(4), v(3), w(3)] (row-contiguous for speed); comB C (3, N); mass M (N,);
inertia I / Iinv (6, N) as (xx, yy, zz, xy, xz, yz).
"""
from __future__ import annotations

import math

import numpy as np

from . import fmath
from .contract import NU, SUBSTEPS, DT
from .massprops import MassModel
from .model import CONTACT, PLACEMENT
from .propulsion import BatchPropulsion
from .spec import WORLD, LAUNCH, PROPULSION, VEHICLE, GENERATED
from .world import BOXES, GROUND, KIND, support_level

DECK, KICK = WORLD['deck'], LAUNCH['kick']
PHASE_NAMES = ('deck', 'air', 'ground')
INIT_CODES = {'launch': 0, 'rest': 1, 'state': 2}


def quat_to_mat(qx, qy, qz, qw):
    """Row-major rotation matrix entries (tuple of 9) of q; same expressions as src/core/quat.js."""
    return (1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw),
            2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw),
            2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy))


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def quat_yaw(c: float, s: float):
    """Unit quaternion of a yaw about +Y with cos c, sin s (half angle without trig), as in twin.js quatYaw."""
    if c > -0.5:
        w = math.sqrt((1 + c) / 2)
        y = s / (2 * w)
    else:
        y = math.sqrt((1 - c) / 2)
        if s < 0:
            y = -y
        w = s / (2 * y)
    return (0.0, y, 0.0, w)


def solve3(J, r):
    a, b, c, d, e, f, g, h, i = J
    A = e * i - f * h
    B = f * g - d * i
    C = d * h - e * g
    det = a * A + b * B + c * C
    x0 = (A * r[0] + (c * h - b * i) * r[1] + (b * f - c * e) * r[2]) / det
    x1 = (B * r[0] + (a * i - c * g) * r[1] + (c * d - a * f) * r[2]) / det
    x2 = (C * r[0] + (b * g - a * h) * r[1] + (a * e - b * d) * r[2]) / det
    return x0, x1, x2


class Twin:
    """N rigid twins stepped together. Initial conditions per vehicle (see reset())."""

    def __init__(self, n: int = 1, arm: str | list = 'start', dt: float | None = None):
        self.N = n
        self.dt = DT if dt is None else dt  # dt override: planning models only (MPPI); never for parity
        self.nsub = SUBSTEPS if dt is None else round(PROPULSION['control']['dt'] / dt)
        self.g = WORLD['gravity']
        self.prop = BatchPropulsion(n, PROPULSION, WORLD['air']['pressure'])
        self.mass = MassModel(PROPULSION, GENERATED['massprops'])
        W, H = VEHICLE['wheels'], VEHICLE['hull']
        self.ks, self.cs, self.travel, self.kb = W['spring_k'], W['damper_c'], W['travel'], W['bump_k']
        self.tyre_mu, self.crr = W['tyre_mu'], W['crr']
        self.kh, self.ch, self.hull_mu = H['k'], H['c'], H['mu']
        self.vreg, self.hmin = CONTACT['v_reg'], CONTACT['wheel_min_h']
        self.wheels = [(tuple(map(float, w['centre'])), float(w['radius'])) for w in GENERATED['massprops']['wheels']]
        hp = np.array(H['points'], dtype=np.float64)
        self.hx, self.hy, self.hz = hp[:, 0].copy(), hp[:, 1].copy(), hp[:, 2].copy()
        self.ndir, self.nexit = [], []
        for z in PROPULSION['nozzles']:
            e = z['exhaust']
            l = math.sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2])
            self.ndir.append((-e[0] / l, -e[1] / l, -e[2] / l))
            self.nexit.append(tuple(map(float, z['exit'])))
        rho = WORLD['air']['density']
        self.ka = [0.5 * rho * A for A in VEHICLE['aero']['CdA']]
        wc = np.array([w[0] for w in self.wheels])
        self.wcx, self.wcy, self.wcz = wc[:, 0].copy(), wc[:, 1].copy(), wc[:, 2].copy()
        self.wr = np.array([w[1] for w in self.wheels])
        self._wbits = 1 << np.arange(len(self.wheels))
        nd, ne = np.array(self.ndir), np.array(self.nexit)
        self._ndx, self._ndy, self._ndz = nd[:, 0].copy(), nd[:, 1].copy(), nd[:, 2].copy()
        self._nex, self._ney, self._nez = ne[:, 0].copy(), ne[:, 1].copy(), ne[:, 2].copy()
        self.n_kick = round(KICK['dur'] / self.dt)
        rb = 0.0
        for mg in (self.prop.m0, 0.0):
            c = self.mass.at(mg)[1]
            for x in hp:
                rb = max(rb, math.hypot(x[0] - c[0], x[1] - c[1], x[2] - c[2]))
            for wc, r in self.wheels:
                rb = max(rb, math.hypot(wc[0] - c[0], wc[1] - c[1], wc[2] - c[2]) + r)
        self.r_broad = rb + 0.5
        self._bc = np.array([b['c'] for b in BOXES]).T.copy()  # (3, B)
        self._bext = np.array([[b['rx'], b['e'][1], b['rz']] for b in BOXES]).T.copy()
        # state
        self.S = np.zeros((13, n))
        self.S[6] = 1.0
        self.C = np.zeros((3, n))
        self.M = np.zeros(n)
        self.I = np.zeros((6, n))
        self.Iinv = np.zeros((6, n))
        self.k = np.zeros(n, dtype=np.int64)
        self.arm_start = np.zeros(n, dtype=bool)
        self.set_arm(arm)
        self.kick_state = np.full(n, 2, dtype=np.int8)
        self.kick_left = np.zeros(n, dtype=np.int64)
        self.kick_dw = np.zeros((3, n))
        self.left_deck = np.ones(n, dtype=bool)
        self.armed = np.zeros(n, dtype=bool)
        self.airborne = np.zeros(n, dtype=bool)
        self.init_code = np.zeros(n, dtype=np.int8)
        self.phase = np.zeros(n, dtype=np.int8)
        self.td = np.zeros(n, dtype=bool)
        self.td_t = np.zeros(n)
        self.td_S = np.zeros((13, n))
        self.td_wheels = np.zeros(n, dtype=np.int64)
        self.hs = np.zeros(n, dtype=bool)
        self.hs_t = np.zeros(n)
        self.hs_point = np.zeros(n, dtype=np.int64)
        self.peak_sf = np.zeros(n)
        self.wheel_mask = np.zeros(n, dtype=np.int64)
        self.dropout = np.zeros(n, dtype=bool)
        self.f_any, self.f_street, self.f_deck = np.zeros(n, bool), np.zeros(n, bool), np.zeros(n, bool)
        self.f_other, self.f_hull = np.zeros(n, bool), np.zeros(n, bool)
        self.u = np.zeros((n, NU))
        self.events = [[] for _ in range(n)]
        self.launch = [None] * n
        self.settle_residual = np.zeros((3, n))

    def set_arm(self, arm):
        arms = [arm] * self.N if isinstance(arm, str) else list(arm)
        self.arm_start[:] = [a == 'start' for a in arms]

    # ------------------------------------------------------------------------------------------- reset
    def reset(self, inits, idx=None):
        """inits: list of dicts (one per index in idx, default all vehicles):
        {'type':'launch', speed (km/h), angle (deg), spin [pitch, yaw, roll] (rad/s), lead (s)?}
        {'type':'rest', x, z, heading_deg}
        {'type':'state', p, q, v, w, gas?, valve?, leftDeck?, kick?: {'spin_w': [3]}}"""
        idx = np.arange(self.N) if idx is None else np.asarray(idx, dtype=np.int64)
        assert len(inits) == len(idx)
        P = self.prop
        P.reset(idx)
        M, c, I, Ii = self.mass.at(P.m[idx])
        self.M[idx] = M
        for a in range(3):
            self.C[a, idx] = c[a]
        for a in range(6):
            self.I[a, idx], self.Iinv[a, idx] = I[a], Ii[a]
        self.k[idx] = 0
        self.kick_state[idx], self.kick_left[idx], self.kick_dw[:, idx] = 2, 0, 0.0
        self.left_deck[idx], self.armed[idx], self.airborne[idx], self.td[idx], self.hs[idx] = True, False, False, False, False
        self.peak_sf[idx], self.wheel_mask[idx], self.dropout[idx] = 0.0, 0, False
        for f in (self.f_any, self.f_street, self.f_deck, self.f_other, self.f_hull):
            f[idx] = False
        self.S[:, idx] = 0.0
        self.S[6, idx] = 1.0
        settle = []  # (env, xc, zc, ys, qh)
        for e, init in zip(idx.tolist(), inits):
            self.events[e] = []
            typ = init['type']
            self.init_code[e] = INIT_CODES[typ]
            if typ == 'launch':
                th = (init['angle'] * math.pi) / 180
                s = init['speed'] / 3.6
                lead = init.get('lead', LAUNCH['defaults']['lead'])
                ct, st = fmath.cos(th), fmath.sin(th)
                f, r = (-ct, 0.0, st), (st, 0.0, ct)
                vx, vz = s * f[0], s * f[2]
                xc = (DECK['gap'][0] + DECK['gap'][1]) / 2 + 1.5 - vx * lead
                zc = DECK['halfWidth'] - vz * lead - 0.9
                settle.append((e, xc, zc, support_level(xc, zc), quat_yaw(st, -ct)))
                self.S[7, e], self.S[8, e], self.S[9, e] = vx, 0.0, vz
                sp, sy, sr = init['spin']
                nk = self.n_kick
                self.kick_dw[:, e] = ((sp * r[0] + sr * f[0]) / nk, sy / nk, (sp * r[2] + sr * f[2]) / nk)
                self.kick_state[e], self.left_deck[e] = 0, False
                self.launch[e] = {'speed': s, 'angle': init['angle'], 'f': f, 'r': r, 'x': xc, 'z': zc}
            elif typ == 'rest':
                ps = (init['heading_deg'] * math.pi) / 180
                settle.append((e, init['x'], init['z'], support_level(init['x'], init['z']), quat_yaw(fmath.cos(ps), fmath.sin(ps))))
                self.launch[e] = {'x': init['x'], 'z': init['z']}
            elif typ == 'state':
                if init.get('gas') is not None:
                    m = float(init['gas'])
                    rho = m / P.V
                    P.m[e] = m
                    P.p[e] = P.K * rho ** P.spec['tank']['polytropic_n']
                    P.T[e] = P.p[e] / (rho * P.R) if rho > 0 else P.spec['tank']['T0']
                if init.get('valve') is not None:
                    P.valve[e] = init['valve']
                Me, ce, Ie, Iie = self.mass.at(float(P.m[e]))
                self.M[e] = Me
                self.C[:, e] = ce
                self.I[:, e], self.Iinv[:, e] = Ie, Iie
                self.S[0:3, e], self.S[3:7, e], self.S[7:10, e], self.S[10:13, e] = init['p'], init['q'], init['v'], init['w']
                if init.get('kick'):
                    self.kick_dw[:, e] = [x / self.n_kick for x in init['kick']['spin_w']]
                    self.kick_state[e] = 0
                self.left_deck[e] = init.get('leftDeck', True)
                self.launch[e] = {'x': init['p'][0], 'z': init['p'][2]}
            else:
                raise ValueError(f'unknown init type {typ}')
        if settle:
            self._settle(*map(np.array, zip(*[s[:4] for s in settle])), np.array([s[4] for s in settle]).T)
        self._mgas = P.m.copy()
        self._boundary(idx)

    def _settle(self, env, xc, zc, ys, qh):
        """Static suspension equilibrium (vectorised over the vehicles being placed); see twin.js settle()."""
        env = env.astype(np.int64)
        iters, h = PLACEMENT['iterations'], PLACEMENT['fd_step']
        M, cx, cy, cz = self.M[env], self.C[0, env], self.C[1, env], self.C[2, env]
        Wg = M * self.g
        qh = tuple(qh[i] for i in range(4))

        def res(u0, u1, u2):
            n0 = np.zeros_like(u0)
            q = quat_mul((u1, n0, u2, n0 + 1.0), qh)
            n = np.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
            q = (q[0] / n, q[1] / n, q[2] / n, q[3] / n)
            R = quat_to_mat(*q)
            ax, ay, az = R[0], R[3], R[6]
            hh = np.sqrt(ax * ax + az * az)
            ih = 1 / hh
            dx, dy, dz = ay * ax * ih, -hh, ay * az * ih
            F, Tx, Tz = n0.copy(), n0.copy(), n0.copy()
            for (wc, r) in self.wheels:
                bx, by, bz = wc[0] - cx, wc[1] - cy, wc[2] - cz
                rx = (R[0] * bx + R[1] * by + R[2] * bz) + r * dx
                ry = (R[3] * bx + R[4] * by + R[5] * bz) + r * dy
                rz = (R[6] * bx + R[7] * by + R[8] * bz) + r * dz
                d = ys - (u0 + ry)
                N = self.ks * d
                N = np.where(d > self.travel, N + self.kb * (d - self.travel), N)
                on = d > 0
                F = F + np.where(on, N, 0.0)
                Tx = Tx - np.where(on, rz * N, 0.0)
                Tz = Tz + np.where(on, rx * N, 0.0)
            return (F - Wg, Tx, Tz), q

        u = [ys + cy - Wg / (4 * self.ks), np.zeros_like(ys), np.zeros_like(ys)]
        for _ in range(iters):
            r0, _q = res(*u)
            J = [None] * 9
            for k in range(3):
                uk = list(u)
                uk[k] = uk[k] + h
                r1, _q = res(*uk)
                J[k], J[3 + k], J[6 + k] = (r1[0] - r0[0]) / h, (r1[1] - r0[1]) / h, (r1[2] - r0[2]) / h
            d = solve3(J, r0)
            u = [u[0] - d[0], u[1] - d[1], u[2] - d[2]]
        r0, q = res(*u)
        self.settle_residual[:, env] = np.array(r0)
        self.S[0, env], self.S[1, env], self.S[2, env] = xc, u[0], zc
        for a in range(4):
            self.S[3 + a, env] = q[a]

    # ------------------------------------------------------------------------------------- control step
    def _event(self, mask, typ, t, **extra):
        for e in np.nonzero(mask)[0].tolist():
            ev = {'t': float(t[e]), 'type': typ}
            for k, v in extra.items():
                ev[k] = int(v[e]) if isinstance(v, np.ndarray) else v
            self.events[e].append(ev)

    def _boundary(self, idx=None):
        sel = np.zeros(self.N, bool)
        sel[slice(None) if idx is None else idx] = True
        t = self.k * self.dt
        edge = sel & ~self.left_deck & (self.S[2] > DECK['halfWidth'])
        self.left_deck |= edge
        self._event(edge, 'edge', t)
        air = sel & (self.k > 0) & self.left_deck & ~self.airborne & ~self.f_any
        self.airborne |= air
        self._event(air, 'airborne', t)
        arm = sel & ~self.armed & (self.arm_start | self.left_deck)
        self.armed |= arm
        self._event(arm, 'armed', t)
        k0 = self.k == 0
        ph = np.where(~self.left_deck, 0,
                      np.where(k0, np.where(self.init_code == 1, 2, 1),
                               np.where(~self.f_any, 1, np.where(self.f_deck, 0, 2))))
        self.phase[sel] = ph[sel]

    def step(self, u_cmd: np.ndarray):
        """One control step for every vehicle: u_cmd (N, NU), held for nsub integrator steps (0 while unarmed)."""
        u = np.where(self.armed[:, None], np.asarray(u_cmd, dtype=np.float64).reshape(self.N, NU), 0.0)
        u = np.where(u == u, u, 0.0)
        self.u[:] = u
        for f in (self.f_any, self.f_street, self.f_deck, self.f_other, self.f_hull):
            f[:] = False
        with np.errstate(all='ignore'):
            for _ in range(self.nsub):
                self.substep(u)
        self._boundary()

    # --------------------------------------------------------------------------------- integrator step
    def substep(self, u):
        S, C, dt, N = self.S, self.C, self.dt, self.N
        p0, p1, p2 = S[0], S[1], S[2]
        v0, v1, v2 = S[7], S[8], S[9]
        t = self.k * dt
        R = quat_to_mat(S[3], S[4], S[5], S[6])
        R0, R1, R2, R3, R4, R5, R6, R7, R8 = R
        # 1. kerb kick
        trig = (self.kick_state == 0) & (p2 > DECK['halfWidth'] - KICK['trigger'])
        if trig.any():
            self.kick_state[trig] = 1
            self.kick_left[trig] = self.n_kick
            self._event(trig, 'kick', t)
        act = self.kick_left > 0
        if act.any():
            dx, dy, dz = self.kick_dw
            S[10] = np.where(act, S[10] + (R0 * dx + R3 * dy + R6 * dz), S[10])
            S[11] = np.where(act, S[11] + (R1 * dx + R4 * dy + R7 * dz), S[11])
            S[12] = np.where(act, S[12] + (R2 * dx + R5 * dy + R8 * dz), S[12])
            self.kick_left[act] -= 1
            self.kick_state[act & (self.kick_left == 0)] = 2
        w0, w1, w2 = S[10].copy(), S[11].copy(), S[12].copy()
        # 2. propulsion
        P = self.prop
        p_set = P.spec['regulator']['p_set']
        pc_prev = P.pc.copy()
        T = P.step(u, dt)
        drop = (P.pc < p_set) & (pc_prev >= p_set) & ~self.dropout
        if drop.any():
            self.dropout |= drop
            self._event(drop, 'regulator_dropout', t)
        # 3. forces at the state of the step start
        cx, cy, cz = C[0], C[1], C[2]
        M = self.M
        ox = R0 * w0 + R1 * w1 + R2 * w2
        oy = R3 * w0 + R4 * w1 + R5 * w2
        oz = R6 * w0 + R7 * w1 + R8 * w2
        f_deck, f_other = np.zeros(N, bool), np.zeros(N, bool)
        # broadphase at batch level (a box is processed if it is near any vehicle; exact zeros elsewhere)
        bc, be, rb = self._bc, self._bext, self.r_broad
        near = ((np.abs(p0[:, None] - bc[0]) < be[0] + rb) & (np.abs(p1[:, None] - bc[1]) < be[1] + rb)
                & (np.abs(p2[:, None] - bc[2]) < be[2] + rb))
        active = [BOXES[b] for b in np.nonzero(near.any(axis=0))[0].tolist()]
        Q = [a[:, None] for a in R]
        C0, C1, C2 = cx[:, None], cy[:, None], cz[:, None]
        P0, P1, P2 = p0[:, None], p1[:, None], p2[:, None]
        oX, oY, oZ = ox[:, None], oy[:, None], oz[:, None]
        V0, V1, V2 = v0[:, None], v1[:, None], v2[:, None]
        # wheels, (N, 4) at once
        ax, ay, az = R0, R3, R6
        hh = np.sqrt(ax * ax + az * az)
        wok = (hh > self.hmin)[:, None]
        ih = 1 / hh
        dlx, dly, dlz = (ay * ax * ih)[:, None], (-hh)[:, None], (ay * az * ih)[:, None]
        AX, AZ, IH = ax[:, None], az[:, None], ih[:, None]
        wr = self.wr
        bx, by, bz = self.wcx - C0, self.wcy - C1, self.wcz - C2
        rwx = (Q[0] * bx + Q[1] * by + Q[2] * bz) + wr * dlx
        rwy = (Q[3] * bx + Q[4] * by + Q[5] * bz) + wr * dly
        rwz = (Q[6] * bx + Q[7] * by + Q[8] * bz) + wr * dlz
        xx, xy, xz = P0 + rwx, P1 + rwy, P2 + rwz
        d = GROUND['y'] - xy
        mu = np.full(d.shape, float(GROUND['mu']))
        kind = np.zeros(d.shape, dtype=np.int8)
        for B in active:
            ddx, ddy, ddz = xx - B['c'][0], xy - B['c'][1], xz - B['c'][2]
            lx, lz = B['cy'] * ddx - B['sy'] * ddz, B['sy'] * ddx + B['cy'] * ddz
            db = B['e'][1] - ddy
            ok = (np.abs(lx) < B['e'][0]) & (np.abs(ddy) < B['e'][1]) & (np.abs(lz) < B['e'][2]) & (db <= wr) & (db > d)
            d = np.where(ok, db, d)
            mu = np.where(ok, B['mu'], mu)
            kind = np.where(ok, B['kind'], kind)
        cont = wok & (d > 0)
        wmask = (cont * self._wbits).sum(axis=1)
        anyc = cont.any(axis=1)
        street = (cont & (kind == KIND['street'])).any(axis=1)
        f_deck |= (cont & (kind == KIND['deck'])).any(axis=1)
        f_other |= (cont & (kind == KIND['other'])).any(axis=1)
        vx = V0 + (oY * rwz - oZ * rwy)
        vy = V1 + (oZ * rwx - oX * rwz)
        vz = V2 + (oX * rwy - oY * rwx)
        Nn = self.ks * d - self.cs * vy
        Nn = np.where(d > self.travel, Nn + self.kb * (d - self.travel), Nn)
        on = cont & (Nn > 0)
        vt = (vx * AZ - vz * AX) * IH
        vl = (vx * AX + vz * AZ) * IH
        ft = -(self.crr * Nn) * vt / np.maximum(np.abs(vt), self.vreg)
        fl = -(self.tyre_mu * mu * Nn) * vl / np.maximum(np.abs(vl), self.vreg)
        fwx = np.where(on, (ft * AZ + fl * AX) * IH, 0.0)
        fwy = np.where(on, Nn, 0.0)
        fwz = np.where(on, (fl * AZ - ft * AX) * IH, 0.0)
        twx = np.where(on, rwy * fwz - rwz * fwy, 0.0)
        twy = np.where(on, rwz * fwx - rwx * fwz, 0.0)
        twz = np.where(on, rwx * fwy - rwy * fwx, 0.0)
        self.wheel_mask[:] = wmask
        # hull points, (N, P) at once
        bx, by, bz = self.hx - C0, self.hy - C1, self.hz - C2
        rx = Q[0] * bx + Q[1] * by + Q[2] * bz
        ry = Q[3] * bx + Q[4] * by + Q[5] * bz
        rz = Q[6] * bx + Q[7] * by + Q[8] * bz
        xx, xy, xz = P0 + rx, P1 + ry, P2 + rz
        vx = V0 + (oY * rz - oZ * ry)
        vy = V1 + (oZ * rx - oX * rz)
        vz = V2 + (oX * ry - oY * rx)
        d0 = GROUND['y'] - xy
        hit = d0 > 0
        inside_any = hit.copy()
        street_pts = hit.copy()
        Nn = self.kh * d0 - self.ch * vy
        on = hit & (Nn > 0)
        vt = np.sqrt(vx * vx + vz * vz)
        fr = (self.hull_mu * GROUND['mu'] * Nn) / np.maximum(vt, self.vreg)
        fx = np.where(on, -(fr * vx), 0.0)
        fy = np.where(on, Nn, 0.0)
        fz = np.where(on, -(fr * vz), 0.0)
        for B in active:
            ddx, ddy, ddz = xx - B['c'][0], xy - B['c'][1], xz - B['c'][2]
            lx, lz = B['cy'] * ddx - B['sy'] * ddz, B['sy'] * ddx + B['cy'] * ddz
            alx, aly, alz = np.abs(lx), np.abs(ddy), np.abs(lz)
            ins = (alx < B['e'][0]) & (aly < B['e'][1]) & (alz < B['e'][2])
            if not ins.any():
                continue
            px, py, pz = B['e'][0] - alx, B['e'][1] - aly, B['e'][2] - alz
            fy_ = (py <= px) & (py <= pz)
            fx_ = ~fy_ & (px <= pz)
            sx = np.where(lx >= 0, 1.0, -1.0)
            sy = np.where(ddy >= 0, 1.0, -1.0)
            sz = np.where(lz >= 0, 1.0, -1.0)
            d = np.where(fy_, py, np.where(fx_, px, pz))
            nx = np.where(fy_, 0.0, np.where(fx_, B['cy'] * sx, B['sy'] * sz))
            ny = np.where(fy_, sy, 0.0)
            nz = np.where(fy_, 0.0, np.where(fx_, -B['sy'] * sx, B['cy'] * sz))
            inside_any |= ins
            if B['kind'] == KIND['deck']:
                f_deck |= ins.any(axis=1)
            else:
                hit |= ins
                if B['kind'] == KIND['street']:
                    street_pts |= ins
                else:
                    f_other |= ins.any(axis=1)
            vn = vx * nx + vy * ny + vz * nz
            Nb = self.kh * d - self.ch * vn
            onb = ins & (Nb > 0)
            tx, ty, tz = vx - vn * nx, vy - vn * ny, vz - vn * nz
            frb = (self.hull_mu * B['mu'] * Nb) / np.maximum(np.sqrt(tx * tx + ty * ty + tz * tz), self.vreg)
            fx = fx + np.where(onb, Nb * nx - frb * tx, 0.0)
            fy = fy + np.where(onb, Nb * ny - frb * ty, 0.0)
            fz = fz + np.where(onb, Nb * nz - frb * tz, 0.0)
        # contact sums: 0, wheels 0..3, hull points in order -- np.add.accumulate is strictly sequential
        Z1 = np.zeros((N, 1))
        acc = lambda *cols: np.add.accumulate(np.concatenate((Z1, *cols), axis=1), axis=1)[:, -1]
        Fx, Fy, Fz = acc(fwx, fx), acc(fwy, fy), acc(fwz, fz)
        Tx = acc(twx, ry * fz - rz * fy)
        Ty = acc(twy, rz * fx - rx * fz)
        Tz = acc(twz, rx * fy - ry * fx)
        hull_hit = hit.any(axis=1)
        anyc |= inside_any.any(axis=1)
        street |= street_pts.any(axis=1)
        self.f_any |= anyc
        self.f_deck |= f_deck
        self.f_other |= f_other
        self.f_hull |= hull_hit
        new_hs = hull_hit & ~self.hs
        if new_hs.any():
            pt = np.argmax(hit, axis=1)
            self.hs |= new_hs
            self.hs_t = np.where(new_hs, t, self.hs_t)
            self.hs_point = np.where(new_hs, pt, self.hs_point)
            self._event(new_hs, 'hull_strike', t, point=pt)
        self.f_street |= street
        new_td = street & self.airborne & ~self.td
        if new_td.any():
            self.td |= new_td
            self.td_t = np.where(new_td, t, self.td_t)
            self.td_S[:, new_td] = S[:, new_td]
            self.td_S[10, new_td], self.td_S[11, new_td], self.td_S[12, new_td] = w0[new_td], w1[new_td], w2[new_td]
            self.td_wheels = np.where(new_td, wmask, self.td_wheels)
            self._event(new_td, 'touchdown', t, wheels=wmask)
        # thrust at the nozzle exits about the current COM (body frame), then body-axis drag
        gx, gy, gz = T * self._ndx, T * self._ndy, T * self._ndz
        rx_, ry_, rz_ = self._nex - C0, self._ney - C1, self._nez - C2
        Bx, By, Bz = acc(gx), acc(gy), acc(gz)
        Ux = acc(ry_ * gz - rz_ * gy)
        Uy = acc(rz_ * gx - rx_ * gz)
        Uz = acc(rx_ * gy - ry_ * gx)
        vbx = R0 * v0 + R3 * v1 + R6 * v2
        vby = R1 * v0 + R4 * v1 + R7 * v2
        vbz = R2 * v0 + R5 * v1 + R8 * v2
        ka = self.ka
        Bx = Bx - ka[0] * np.abs(vbx) * vbx
        By = By - ka[1] * np.abs(vby) * vby
        Bz = Bz - ka[2] * np.abs(vbz) * vbz
        Fx = Fx + (R0 * Bx + R1 * By + R2 * Bz)
        Fy = Fy + (R3 * Bx + R4 * By + R5 * Bz)
        Fz = Fz + (R6 * Bx + R7 * By + R8 * Bz)
        tx = Ux + (R0 * Tx + R3 * Ty + R6 * Tz)
        ty = Uy + (R1 * Tx + R4 * Ty + R7 * Tz)
        tz = Uz + (R2 * Tx + R5 * Ty + R8 * Tz)
        sf = np.sqrt(Fx * Fx + Fy * Fy + Fz * Fz) / M
        self.peak_sf = np.maximum(self.peak_sf, sf)
        # 4. integrate
        S[7] = v0 + dt * (Fx / M)
        S[8] = v1 + dt * (Fy / M - self.g)
        S[9] = v2 + dt * (Fz / M)
        I, Ii = self.I, self.Iinv
        wx, wy, wz = w0, w1, w2
        for hdt in (0.5 * dt, dt):
            Lx = I[0] * wx + I[3] * wy + I[4] * wz
            Ly = I[3] * wx + I[1] * wy + I[5] * wz
            Lz = I[4] * wx + I[5] * wy + I[2] * wz
            gx = tx - (wy * Lz - wz * Ly)
            gy = ty - (wz * Lx - wx * Lz)
            gz = tz - (wx * Ly - wy * Lx)
            ex = Ii[0] * gx + Ii[3] * gy + Ii[4] * gz
            ey = Ii[3] * gx + Ii[1] * gy + Ii[5] * gz
            ez = Ii[4] * gx + Ii[5] * gy + Ii[2] * gz
            wx, wy, wz = w0 + hdt * ex, w1 + hdt * ey, w2 + hdt * ez
        S[10], S[11], S[12] = wx, wy, wz
        S[0] = p0 + dt * S[7]
        S[1] = p1 + dt * S[8]
        S[2] = p2 + dt * S[9]
        qx, qy, qz, qw = S[3].copy(), S[4].copy(), S[5].copy(), S[6].copy()
        h = 0.5 * dt
        nx = qx + h * (qw * wx + qy * wz - qz * wy)
        ny = qy + h * (qw * wy + qz * wx - qx * wz)
        nz = qz + h * (qw * wz + qx * wy - qy * wx)
        nw = qw + h * (-qx * wx - qy * wy - qz * wz)
        n = np.sqrt(nx * nx + ny * ny + nz * nz + nw * nw)
        S[3], S[4], S[5], S[6] = nx / n, ny / n, nz / n, nw / n
        # 5. mass properties of the remaining gas with the kinematic COM shift
        Mn, c2, In, Iin = self.mass.at(P.m)
        Q2 = quat_to_mat(S[3], S[4], S[5], S[6])
        dcx, dcy, dcz = c2[0] - cx, c2[1] - cy, c2[2] - cz
        S[0] = S[0] + (Q2[0] * dcx + Q2[1] * dcy + Q2[2] * dcz)
        S[1] = S[1] + (Q2[3] * dcx + Q2[4] * dcy + Q2[5] * dcz)
        S[2] = S[2] + (Q2[6] * dcx + Q2[7] * dcy + Q2[8] * dcz)
        ex, ey, ez = wy * dcz - wz * dcy, wz * dcx - wx * dcz, wx * dcy - wy * dcx
        S[7] = S[7] + (Q2[0] * ex + Q2[1] * ey + Q2[2] * ez)
        S[8] = S[8] + (Q2[3] * ex + Q2[4] * ey + Q2[5] * ez)
        S[9] = S[9] + (Q2[6] * ex + Q2[7] * ey + Q2[8] * ez)
        self.M = Mn
        self.C = np.array(c2)
        self.I = np.array(In)
        self.Iinv = np.array(Iin)
        self.k += 1

    def load_state(self, s: dict, src: int = 0, idx=None):
        """Put lane `src` of a batch FlightState (coldgas.state) into lanes idx (default all): planning from a state
        estimate. Restores what a FlightState carries; tank p/T follow the blowdown polytrope from m, mass properties
        come from the model, the kick is over, a vehicle in phase 'ground' counts as touched down."""
        idx = np.arange(self.N) if idx is None else np.asarray(idx)
        P = self.prop
        for a, (key, n) in enumerate((('p', 3), ('q', 4), ('v', 3), ('w', 3))):
            self.S[[0, 3, 7, 10][a]:[0, 3, 7, 10][a] + n, idx] = np.asarray(s[key][src], dtype=np.float64)[:, None]
        m = float(s['tank_m'][src])
        rho = m / P.V
        P.m[idx] = m
        P.p[idx] = P.K * rho ** P.spec['tank']['polytropic_n']
        P.T[idx] = P.p[idx] / (rho * P.R) if rho > 0 else float(s['tank_T'][src])
        P.valve[idx] = np.asarray(s['valve'][src], dtype=np.float64)
        P.thrust[idx] = 0.0
        reg = P.spec['regulator']
        pc = min(reg['p_set'], float(P.p[idx[0]]) - reg['dp_min'])
        P.pc[idx] = pc if pc > 2 * P.pa else 0.0
        M, c, I, Ii = self.mass.at(m)
        self.M[idx] = M
        self.C[:, idx] = np.array(c)[:, None]
        self.I[:, idx], self.Iinv[:, idx] = np.array(I)[:, None], np.array(Ii)[:, None]
        self.k[idx] = round(float(s['t'][src]) / self.dt)
        self.kick_state[idx], self.kick_left[idx] = 2, 0
        ph = int(s['phase'][src])
        self.left_deck[idx], self.armed[idx] = True, bool(s['armed'][src])
        self.airborne[idx] = ph != 0
        self.td[idx], self.hs[idx] = ph == 2, False
        self.td_S[:, idx] = self.S[:, idx]
        self.peak_sf[idx], self.dropout[idx] = 0.0, P.pc[idx] < reg['p_set']
        for f in (self.f_any, self.f_street, self.f_deck, self.f_other, self.f_hull):
            f[idx] = False
        self.phase[idx] = ph
        self._mgas = P.m.copy()

    # ------------------------------------------------------------------------------------------- views
    def state(self, e: int = 0) -> dict:
        """FlightState of vehicle e (copies; spec/README.md section 7)."""
        S, P = self.S, self.prop
        return {'t': float(self.k[e] * self.dt), 'p': S[0:3, e].copy(), 'q': S[3:7, e].copy(), 'v': S[7:10, e].copy(),
                'w': S[10:13, e].copy(), 'comB': self.C[:, e].copy(),
                'tank': {'m': float(P.m[e]), 'T': float(P.T[e]), 'p': float(P.p[e])}, 'valve': P.valve[e].copy(),
                'armed': bool(self.armed[e]), 'phase': PHASE_NAMES[self.phase[e]], 'pc': float(P.pc[e]),
                'thrust': P.thrust[e].copy(), 'step': int(self.k[e])}

    def flat(self) -> np.ndarray:
        """(N, 13) [p, q, v, w]."""
        return self.S.T.copy()
