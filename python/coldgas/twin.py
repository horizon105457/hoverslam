"""Rigid 6-DOF flight twin, batched over N vehicles (numpy). Mirror of src/flight/twin.js; contract: spec/README.md 5.

Every expression is evaluated in the same order as the JS version and every reduction is a sequential sum in the
documented order, so a vehicle here and the JS twin agree bit for bit (tests/parity). The scalar env is this class
with N = 1, so batched and scalar results are identical by construction. Culled boxes / masked lanes contribute
exact zeros, which leaves sums unchanged.

State layout: S (13, N) rows [p(3), q(4), v(3), w(3)] (row-contiguous for speed); comB C (3, N); mass M (N,);
inertia I / Iinv (6, N) as (xx, yy, zz, xy, xz, yz); wheel state / loads (N, 4); drive state (N,) drive_brake (brake
pressure 0..1), drive_steer (front road-wheel angle, rad), applied drive commands ud (N, 2). Contact parameters come
from the twin's vehicle dict (spec, or spec + override for fitting: model.py); the touchdown extensions of
spec/README.md 5.11 run the v1 expressions unchanged with the spec values, and brakes / steering (5.13) are skipped
while every lane's command and state are 0 (lanes with zero drive add exact selections of the old values).
"""
from __future__ import annotations

import math

import numpy as np

from . import fmath
from .contract import NU, SUBSTEPS, DT, DRIVE_IDX
from .massprops import MassModel
from .model import contact_params, drive_params
from .propulsion import BatchPropulsion
from .spec import WORLD, LAUNCH, PROPULSION, GENERATED
from .world import BOXES, GROUND, KIND, support_level

DECK, KICK = WORLD['deck'], LAUNCH['kick']
PHASE_NAMES = ('deck', 'air', 'ground')
INIT_CODES = {'launch': 0, 'rest': 1, 'state': 2}
OVERLOAD = ('normal', 'lat', 'lat_impulse')  # wheel_overload event quantities (spec/README.md 5.11)
IB, IS = DRIVE_IDX['brake'], DRIVE_IDX['steer']  # action indices of the drive commands (5.13)
BETA_SNAP = 1e-9  # brake pressure snaps onto its command once within this (model constant, 5.13; see twin.js)


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
    """N rigid twins stepped together. Initial conditions per vehicle (see reset()).

    dt: integrator step override for planning models only (MPPI); never for parity. vehicle: partial spec/vehicle.json
    dict deep-merged over the spec (wheels, tyre, wheel_spin, failure, hull, aero, twin.contact / placement) for
    fitting and experiments (spec/README.md 5.12; integrator.dt is not taken from it). Every contact parameter is
    read from it here, never from module constants."""

    def __init__(self, n: int = 1, arm: str | list = 'start', dt: float | None = None, vehicle: dict | None = None):
        self.N = n
        self.dt = DT if dt is None else dt  # dt override: planning models only (MPPI); never for parity
        self.nsub = SUBSTEPS if dt is None else round(PROPULSION['control']['dt'] / dt)
        self.g = WORLD['gravity']
        self.prop = BatchPropulsion(n, PROPULSION, WORLD['air']['pressure'])
        self.mass = MassModel(PROPULSION, GENERATED['massprops'])
        P = contact_params(vehicle)
        self.vehicle, self.params = P['V'], P
        # per-wheel (axle-resolved) parameters as (4,) arrays (v1: all equal) and float tuples (placement)
        self._wp = w = P['wheel']
        A = lambda k: np.array(w[k], dtype=np.float64)
        self.ks, self.cs, self.travel, self.kb = A('spring_k'), A('damper_c'), A('travel'), A('bump_k')
        self.bc, self.kb2, self.tyre_mu, self.crr = A('bump_c'), A('bump_k2'), A('tyre_mu'), A('crr')
        self.c_alpha, self.v_slip, self.v_long = A('c_alpha'), A('v_slip_min'), A('v_long_sat')
        self.kss = A('slip_ratio_sat')  # slip-ratio saturation kappa (0 = off, skipped)
        self.kss_on = bool((self.kss != 0).any())
        self.sld = A('slide')  # sliding blend of the lateral force (0 = off, skipped)
        self.sld_on = bool((self.sld != 0).any())
        self.wA = A('inertia') / self.dt  # wheel inertia over dt (implicit spin update)
        self.toe_c, self.toe_s, self.toe_k = A('toe_cos'), A('toe_sin'), A('toe_compliance')  # static toe, compliance steer
        self.toe_on, self.comply_on = P['toe_on'], P['comply_on']
        # drive (spec/drive.json, 5.13): brake torque limit per wheel, wheel inertia (unloaded braking), pressure lag per
        # step through the portable exp, ABS target factor, steer lock and per-step rate limit (operand order of twin.js)
        D = drive_params()
        self.brake_T, self.w_inertia = A('brake_torque'), A('inertia')
        self.k_brake = 1 - fmath.exp(-self.dt / D['tau'])
        self.abs_keep = 1 - D['abs_slip']
        self.steer_max, self.steer_step = D['steer_max'], D['steer_rate'] * self.dt
        self._front = [j for j, a in enumerate(P['axle']) if a == 0]  # steered wheels
        self.bump_x, self.slip, self.spin_on, self.spin_air = P['bump_x'], P['slip'], P['spin_on'], P['spin_air']
        self.spin_keep = 1 - self.dt * P['spin_decay']  # unloaded wheels: omega *= spin_keep per step
        self.fail_on, self.fail_N, self.fail_L, self.fail_I = P['fail_on'], P['fail_N'], P['fail_L'], P['fail_I']
        self.kh, self.ch, self.hull_mu = P['kh'], P['ch'], P['muh']
        self.vreg, self.hmin, self.placement = P['vreg'], P['hmin'], P['placement']
        self.wheels = [(tuple(map(float, w['centre'])), float(w['radius'])) for w in GENERATED['massprops']['wheels']]
        hp = P['hull']
        self.hx, self.hy, self.hz = hp[:, 0].copy(), hp[:, 1].copy(), hp[:, 2].copy()
        self.ndir, self.nexit = [], []
        for z in PROPULSION['nozzles']:
            e = z['exhaust']
            l = math.sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2])
            self.ndir.append((-e[0] / l, -e[1] / l, -e[2] / l))
            self.nexit.append(tuple(map(float, z['exit'])))
        rho = WORLD['air']['density']
        self.ka = [0.5 * rho * A for A in P['CdA']]
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
        self.f_other, self.f_hull, self.f_kerb = np.zeros(n, bool), np.zeros(n, bool), np.zeros(n, bool)
        self.u = np.zeros((n, NU))
        # drive state (5.13): brake pressure, steer angle, applied [brake, steer]; c0 / s0 (N, 4) = cos / sin of each
        # wheel's heading before compliance steer (static toe, front wheels turned by the steer angle)
        self.drive_brake, self.drive_steer, self.ud = np.zeros(n), np.zeros(n), np.zeros((n, 2))
        self.c0, self.s0 = np.tile(self.toe_c, (n, 1)), np.tile(self.toe_s, (n, 1))
        self._steer_lanes, self._steer_any = np.zeros(n, bool), False
        self._brake_any = self._cmd_b = self._cmd_s = False  # any lane with beta != 0 / brake / steer command
        self.events = [[] for _ in range(n)]
        self.launch = [None] * n
        self.settle_residual = np.zeros((3, n))
        # wheel state and load diagnostics (spec/README.md 5.11), (N, 4): spin (rad/s, + = rolling forward),
        # last-substep normal / lateral / longitudinal force, peaks from the deck edge on, lateral impulse since touchdown
        self.wheel_spin = np.zeros((n, 4))
        self.wheel_N, self.wheel_lat, self.wheel_long = np.zeros((n, 4)), np.zeros((n, 4)), np.zeros((n, 4))
        self.peak_N, self.peak_lat, self.lat_impulse = np.zeros((n, 4)), np.zeros((n, 4)), np.zeros((n, 4))
        self.wo = np.zeros(n, dtype=bool)  # wheel overload happened (first one: t, wheel, quantity code)
        self.wo_t, self.wo_wheel, self.wo_q = np.zeros(n), np.zeros(n, dtype=np.int64), np.zeros(n, dtype=np.int64)
        self.kerb_hit = np.zeros(n, dtype=bool)  # a loaded wheel stood on a kerb off the deck (first one: t, wheel)
        self.kerb_t, self.kerb_wheel = np.zeros(n), np.zeros(n, dtype=np.int64)

    def set_arm(self, arm):
        arms = [arm] * self.N if isinstance(arm, str) else list(arm)
        self.arm_start[:] = [a == 'start' for a in arms]

    # ------------------------------------------------------------------------------------------- reset
    def reset(self, inits, idx=None):
        """inits: list of dicts (one per index in idx, default all vehicles):
        {'type':'launch', speed (km/h), angle (deg), spin [pitch, yaw, roll] (rad/s), lead (s)?}
        {'type':'rest', x, z, heading_deg}
        {'type':'state', p, q, v, w, gas?, valve?, leftDeck?, kick?: {'spin_w': [3]}, wheel_spin?: [4]}"""
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
        for f in (self.f_any, self.f_street, self.f_deck, self.f_other, self.f_hull, self.f_kerb):
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
        self._reset_wheels(idx)
        self._set_drive(idx, 0.0, 0.0)
        if self.spin_on:  # unknown spin: launch / rest roll with the ground; an explicit state is taken as after a flight
            self._match_spin(idx, np.array([self.spin_air if i['type'] == 'state' else 1.0 for i in inits]))
            for e, init in zip(idx.tolist(), inits):
                if init['type'] == 'state' and init.get('wheel_spin') is not None:
                    self.wheel_spin[e] = np.asarray(init['wheel_spin'], dtype=np.float64)
        self._boundary(idx)

    def _reset_wheels(self, idx):
        for a in (self.wheel_spin, self.wheel_N, self.wheel_lat, self.wheel_long, self.peak_N, self.peak_lat, self.lat_impulse):
            a[idx] = 0.0
        self.wo[idx] = False
        self.kerb_hit[idx] = False

    def _set_drive(self, idx, beta: float, delta: float):
        """Drive state of lanes idx (5.13): brake pressure, steer angle (rad); applied commands cleared."""
        idx = np.atleast_1d(np.asarray(idx, dtype=np.int64))
        self.ud[idx] = 0.0
        self.drive_brake[idx] = beta
        self._brake_any = bool(self.drive_brake.any())
        self._cmd_b, self._cmd_s = bool(self.ud[:, 0].any()), bool(self.ud[:, 1].any())
        self._steer_to(idx, np.full(len(idx), float(delta)))

    def _steer_to(self, idx, d):
        """Front road-wheel angle d (rad, per lane of idx): front wheels' static toe heading turned by d (portable
        fmath cos / sin), exactly the static toe where d = 0 (twin.js steerTo)."""
        self.drive_steer[idx] = d
        cd, sd = fmath.cos_np(d), fmath.sin_np(d)
        z = d == 0
        for j in self._front:
            tc, ts = self.toe_c[j], self.toe_s[j]
            self.c0[idx, j] = np.where(z, tc, tc * cd - ts * sd)
            self.s0[idx, j] = np.where(z, ts, ts * cd + tc * sd)
        self._steer_lanes = self.drive_steer != 0
        self._steer_any = bool(self._steer_lanes.any())

    def _match_spin(self, idx, ratio):
        """Wheel spin of lanes idx = ratio (per lane) x rolling-matched at the current state (twin.js matchSpin)."""
        S, C = self.S[:, idx], self.C[:, idx]
        R = quat_to_mat(S[3], S[4], S[5], S[6])
        w0, w1, w2 = S[10], S[11], S[12]
        ox = R[0] * w0 + R[1] * w1 + R[2] * w2
        oy = R[3] * w0 + R[4] * w1 + R[5] * w2
        oz = R[6] * w0 + R[7] * w1 + R[8] * w2
        ax, ay, az = R[0], R[3], R[6]
        with np.errstate(all='ignore'):
            hh = np.sqrt(ax * ax + az * az)
            ih = 1 / hh
            dlx, dly, dlz = (ay * ax * ih)[:, None], (-hh)[:, None], (ay * az * ih)[:, None]
            Q = [a[:, None] for a in R]
            bx, by, bz = self.wcx - C[0][:, None], self.wcy - C[1][:, None], self.wcz - C[2][:, None]
            rx = (Q[0] * bx + Q[1] * by + Q[2] * bz) + self.wr * dlx
            ry = (Q[3] * bx + Q[4] * by + Q[5] * bz) + self.wr * dly
            rz = (Q[6] * bx + Q[7] * by + Q[8] * bz) + self.wr * dlz
            vx = S[7][:, None] + (oy[:, None] * rz - oz[:, None] * ry)
            vz = S[9][:, None] + (ox[:, None] * ry - oy[:, None] * rx)
            vt = (vx * az[:, None] - vz * ax[:, None]) * ih[:, None]
            st = self._steer_lanes[idx]
            if self.toe_on or st.any():  # along the toed / steered heading (unsteered lanes without toe: v1 axes)
                vl = (vx * ax[:, None] + vz * az[:, None]) * ih[:, None]
                vr = self.c0[idx] * vt - self.s0[idx] * vl
                vt = vr if self.toe_on else np.where(st[:, None], vr, vt)
            self.wheel_spin[idx] = np.where((hh > self.hmin)[:, None], np.asarray(ratio, dtype=np.float64)[:, None] * (-vt / self.wr), 0.0)

    def _settle(self, env, xc, zc, ys, qh):
        """Static suspension equilibrium (vectorised over the vehicles being placed); see twin.js settle()."""
        env = env.astype(np.int64)
        iters, h = self.placement['iterations'], self.placement['fd_step']
        wp = self._wp
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
            for j, (wc, r) in enumerate(self.wheels):
                bx, by, bz = wc[0] - cx, wc[1] - cy, wc[2] - cz
                rx = (R[0] * bx + R[1] * by + R[2] * bz) + r * dx
                ry = (R[3] * bx + R[4] * by + R[5] * bz) + r * dy
                rz = (R[6] * bx + R[7] * by + R[8] * bz) + r * dz
                d = ys - (u0 + ry)
                # the static part of the step's wheel force (5.5) at rest: spring + bump stop
                tr = wp['travel'][j]
                e = d - tr
                N = wp['spring_k'][j] * d
                N = np.where(d > tr, N + wp['bump_k'][j] * e, N)
                if self.bump_x:
                    N = np.where(d > tr, N + wp['bump_k2'][j] * e * e, N)
                on = d > 0
                F = F + np.where(on, N, 0.0)
                Tx = Tx - np.where(on, rz * N, 0.0)
                Tz = Tz + np.where(on, rx * N, 0.0)
            return (F - Wg, Tx, Tz), q

        ka = self.params['ks_axle']  # first guess on the springs alone (2 (k_f + k_r) = 4 k_s exactly for equal axles)
        u = [ys + cy - Wg / (2 * (ka[0] + ka[1])), np.zeros_like(ys), np.zeros_like(ys)]
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
        self.peak_N[edge], self.peak_lat[edge] = 0.0, 0.0  # wheel peaks count from the edge (the deck run is the launcher)
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
        """One control step for every vehicle: u_cmd (N, NU) valve commands, or (N, NA) with brake and steer (5.13),
        held for nsub integrator steps (0 while unarmed). Drive: brake clamped to [0, 1], steer to [-1, 1], NaN /
        missing -> 0 (twin.js step)."""
        U = np.asarray(u_cmd, dtype=np.float64).reshape(self.N, -1)
        u = np.where(self.armed[:, None], U[:, :NU], 0.0)
        u = np.where(u == u, u, 0.0)
        self.u[:] = u
        a, nc = self.armed, U.shape[1]
        self.ud[:] = 0.0
        if nc > IB:
            b = U[:, IB]
            self.ud[:, 0] = np.where(a & (b > 0), np.where(b < 1, b, 1.0), 0.0)
            if nc > IS:
                st = U[:, IS]
                self.ud[:, 1] = np.where(a & (st > -1), np.where(st < 1, st, 1.0), np.where(a & (st <= -1), -1.0, 0.0))
        self._cmd_b, self._cmd_s = bool(self.ud[:, 0].any()), bool(self.ud[:, 1].any())
        for f in (self.f_any, self.f_street, self.f_deck, self.f_other, self.f_hull, self.f_kerb):
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
        # 2b. drive (5.13): brake pressure lag, rate-limited steer; skipped while every lane's command and state are 0
        # (a lane at 0 gets exactly 0 back: e = 0 snaps onto b = 0, the steer moves by e = 0)
        if self._cmd_b or self._brake_any:
            b, bb = self.ud[:, 0], self.drive_brake
            e = b - bb
            self.drive_brake = np.where((e < BETA_SNAP) & (e > -BETA_SNAP), b, bb + e * self.k_brake)
            self._brake_any = bool(self.drive_brake.any())
        if self._cmd_s or self._steer_any:
            sc, de = self.ud[:, 1], self.drive_steer
            e = sc * self.steer_max - de
            r = self.steer_step
            dn = de + np.where(e > r, r, np.where(e < -r, -r, e))
            ch = np.nonzero(dn != de)[0]
            if len(ch):
                self._steer_to(ch, dn[ch])
        brake = self.spin_on and self._brake_any  # the brake acts on the wheel spin (no-op without it)
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
        kbw = np.zeros(d.shape, dtype=bool)  # support is a street box (sidewalk / median top: a kerb, 5.13)
        for B in active:
            ddx, ddy, ddz = xx - B['c'][0], xy - B['c'][1], xz - B['c'][2]
            lx, lz = B['cy'] * ddx - B['sy'] * ddz, B['sy'] * ddx + B['cy'] * ddz
            db = B['e'][1] - ddy
            ok = (np.abs(lx) < B['e'][0]) & (np.abs(ddy) < B['e'][1]) & (np.abs(lz) < B['e'][2]) & (db <= wr) & (db > d)
            d = np.where(ok, db, d)
            mu = np.where(ok, B['mu'], mu)
            kind = np.where(ok, B['kind'], kind)
            kbw = (kbw | ok) if B['kind'] == KIND['street'] else (kbw & ~ok)
        cont = wok & (d > 0)
        wmask = (cont * self._wbits).sum(axis=1)
        anyc = cont.any(axis=1)
        street = (cont & (kind == KIND['street'])).any(axis=1)
        f_deck |= (cont & (kind == KIND['deck'])).any(axis=1)
        f_other |= (cont & (kind == KIND['other'])).any(axis=1)
        vx = V0 + (oY * rwz - oZ * rwy)
        vy = V1 + (oZ * rwx - oX * rwz)
        vz = V2 + (oX * rwy - oY * rwx)
        # per-wheel (axle-resolved) parameters broadcast over (N, 4); v2 laws in _tyre (twin.js tyre())
        Nn = self.ks * d - self.cs * vy
        over = d > self.travel
        eb = d - self.travel
        Nn = np.where(over, Nn + self.kb * eb, Nn)
        if self.bump_x:
            Nn = np.where(over, Nn + (self.kb2 * eb * eb - self.bc * vy), Nn)
        on = cont & (Nn > 0)
        vt = (vx * AZ - vz * AX) * IH
        vl = (vx * AX + vz * AZ) * IH
        me = self.tyre_mu * mu  # tyre x surface friction
        if self.slip or self.spin_on or self.toe_on or self._steer_any:
            ft, fl, ftT, flT = self._tyre(Nn, vt, vl, me, on, brake)  # body-axis forces, tyre-axis forces (diagnostics)
        else:
            ft = -(self.crr * Nn) * vt / np.maximum(np.abs(vt), self.vreg)
            fl = -(me * Nn) * vl / np.maximum(np.abs(vl), self.vreg)
            ftT, flT = ft, fl
        # kerb (5.13): a loaded wheel on a street box; first one off the deck -> event (wheel pass: queued with the
        # overload events in twin.js loop order, i.e. by wheel, kerb before overload)
        evq = []
        kerb_w = on & kbw
        if kerb_w.any():
            hit = kerb_w.any(axis=1)
            self.f_kerb |= hit
            new_kb = hit & self.left_deck & ~self.kerb_hit
            if new_kb.any():
                first = np.argmax(kerb_w, axis=1)
                self.kerb_hit |= new_kb
                self.kerb_t = np.where(new_kb, t, self.kerb_t)
                self.kerb_wheel = np.where(new_kb, first, self.kerb_wheel)
                for e in np.nonzero(new_kb)[0].tolist():
                    evq.append((e, int(first[e]), 0, {'t': float(t[e]), 'type': 'kerb', 'wheel': int(first[e])}))
        # load diagnostics (5.11, tyre axes): last substep, peaks, lateral impulse from the step after the touchdown,
        # overload at the first exceedance off the deck (wheel order, then normal / lateral / impulse); unloaded wheels
        # add zeros. wheel_lat is also the compliance-steer input of the next step.
        wN, wL = np.where(on, Nn, 0.0), np.where(on, flT, 0.0)
        aL = np.abs(wL)
        self.wheel_N, self.wheel_lat, self.wheel_long = wN, wL, np.where(on, ftT, 0.0)
        np.maximum(self.peak_N, wN, out=self.peak_N)
        np.maximum(self.peak_lat, aL, out=self.peak_lat)
        if self.td.any():
            self.lat_impulse = np.where(self.td[:, None], self.lat_impulse + aL * dt, self.lat_impulse)
        if self.fail_on:
            ex = np.stack((wN > self.fail_N, aL > self.fail_L, self.lat_impulse > self.fail_I), axis=2).reshape(N, 12)
            new_wo = ex.any(axis=1) & self.left_deck & ~self.wo
            if new_wo.any():
                first = np.argmax(ex, axis=1)
                self.wo |= new_wo
                self.wo_t = np.where(new_wo, t, self.wo_t)
                self.wo_wheel = np.where(new_wo, first // 3, self.wo_wheel)
                self.wo_q = np.where(new_wo, first % 3, self.wo_q)
                for e in np.nonzero(new_wo)[0].tolist():
                    evq.append((e, int(first[e] // 3), 1, {'t': float(t[e]), 'type': 'wheel_overload', 'wheel': int(first[e] // 3),
                                                           'quantity': OVERLOAD[first[e] % 3]}))
        for e, _, _, ev in sorted(evq, key=lambda x: x[:3]):
            self.events[e].append(ev)
        fwx = np.where(on, (ft * AZ + fl * AX) * IH, 0.0)
        fwy = wN
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

    def _tyre(self, Nn, vt, vl, me, on, brake=False):
        """v2 tyre forces (twin.js tyre(), spec/README.md 5.11, 5.13), (N, 4): v_t, v_l turned into the tyre axes by the
        toe (+ steer on the front wheels), force laws there; returns (f_t, f_l) in body axes and in tyre axes, updates the
        spin (brake included). Without toe only the steered lanes rotate (the others keep the v1 axes, as twin.js)."""
        c = s = None
        rot = self.toe_on or self._steer_any
        if rot:  # heading about the vertical (+ = front edge to the right): toe (+ steer) cos / sin x compliance steer
            c, s = self.c0, self.s0
            if self.comply_on:  # tan(delta_c) = -kappa F_lat of the previous step (tyre axes), exact rotation
                dc = -self.toe_k * self.wheel_lat
                cc = 1 / np.sqrt(1 + dc * dc)
                sc = dc * cc
                c, s = c * cc - s * sc, s * cc + c * sc
            vt0, vl0 = vt, vl
            vt, vl = c * vt - s * vl, c * vl + s * vt
            if not self.toe_on:
                m = self._steer_lanes[:, None]
                vt, vl = np.where(m, vt, vt0), np.where(m, vl, vl0)
        sy, D2 = 0.0, 1.0
        if self.slip:  # sigma_y = c_alpha s / mu_eff, |F| = mu_eff N sigma / sqrt(1 + sigma^2)
            sy = self.c_alpha * (vl / np.maximum(np.abs(vt), self.v_slip)) / me
            D2 = 1 + sy * sy
        if self.spin_on:  # longitudinal slip v_t + omega r, secant stiffness, omega and force implicit together
            r, om, vls = self.wr, self.wheel_spin, self.v_long
            if self.kss_on:  # saturation velocity max(v_long_sat, kappa |v_t|) (kappa = 0 wheels: a = 0, unchanged)
                a = self.kss * np.where(vt < 0, -vt, vt)
                vls = np.where(a > vls, a, vls)
            sx = (vt + om * r) / vls
            D2 = D2 + sx * sx
            D = np.sqrt(D2)
            k = (me * Nn) / (vls * D)
            A = self.wA
            wr = om * r
            tau = -(self.crr * Nn) * r * wr / np.maximum(np.abs(wr), self.vreg)  # rolling resistance torque
            den = A + k * r * r
            om2 = (A * om - k * r * vt + tau) / den
            omu = om * self.spin_keep  # unloaded wheels keep their spin (bearing decay)
            if brake:  # implicit Coulomb brake <= T (never reversing, ABS target on loaded wheels), twin.js tyre()
                Tj = self.drive_brake[:, None] * self.brake_T
                dmax = Tj / den
                tgt = self.abs_keep * (-vt / r)
                tp, tn = np.where(tgt > 0, tgt, 0.0), np.where(tgt < 0, tgt, 0.0)
                lo, hi = om2 - dmax, om2 + dmax
                ob = np.where(om2 > 0, np.where(om2 > tp, np.where(lo > tp, lo, tp), om2),
                              np.where(om2 < 0, np.where(om2 < tn, np.where(hi < tn, hi, tn), om2), om2))
                om2 = np.where(Tj > 0, ob, om2)
                du = Tj * self.dt / self.w_inertia  # unloaded: towards 0 by at most T dt / I
                omu = np.where(Tj > 0, np.where(omu > du, omu - du, np.where(omu < -du, omu + du, 0.0)), omu)
            ft = -k * (vt + r * om2)
            self.wheel_spin = np.where(on, om2, omu)
            fl = -(me * Nn) * sy / D if self.slip else -(me * Nn) * vl / np.maximum(np.abs(vl), self.vreg)
            if self.sld_on:  # sliding blend towards isotropic Coulomb along the step-start slip velocity (twin.js)
                vs0 = vt + om * r
                ss = sx * sx
                xi = self.sld * ss / (1 + ss)
                fs = -(me * Nn) * vl / np.maximum(np.sqrt(vs0 * vs0 + vl * vl), self.vreg)
                fl = np.where(self.sld != 0, (1 - xi) * fl + xi * fs, fl)  # slide = 0 wheels: untouched (JS skips)
        else:
            ft = -(self.crr * Nn) * vt / np.maximum(np.abs(vt), self.vreg)
            fl = -(me * Nn) * sy / np.sqrt(D2) if self.slip else -(me * Nn) * vl / np.maximum(np.abs(vl), self.vreg)
        if rot:
            bt, bl = c * ft + s * fl, c * fl - s * ft
            if not self.toe_on:
                bt, bl = np.where(m, bt, ft), np.where(m, bl, fl)
            return bt, bl, ft, fl
        return ft, fl, ft, fl

    def load_state(self, s: dict, src: int = 0, idx=None):
        """Put lane `src` of a batch FlightState (coldgas.state) into lanes idx (default all): planning from a state
        estimate. Restores what a FlightState carries; tank p/T follow the blowdown polytrope from m, mass properties
        come from the model, the kick is over, a vehicle in phase 'ground' counts as touched down. Wheel spin (if
        modelled): s['wheel_spin'][src] if the state carries it, else spin_ratio_air x rolling-matched in the air and
        rolling-matched on the ground / deck (along the current, possibly steered heading); wheel peaks, the overload and
        the kerb hit start afresh. Drive (5.13): s['drive_brake'] / s['drive_steer'] [src] if present, else 0 (twin.js
        loadState)."""
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
        self.td_t[idx], self.td_wheels[idx] = self.k[idx] * self.dt, 0  # as twin.js (snapshot t, no wheel mask)
        self.peak_sf[idx], self.dropout[idx] = 0.0, P.pc[idx] < reg['p_set']
        for f in (self.f_any, self.f_street, self.f_deck, self.f_other, self.f_hull, self.f_kerb):
            f[idx] = False
        self.phase[idx] = ph
        self._mgas = P.m.copy()
        for e in np.atleast_1d(idx).tolist():  # as twin.js: a loaded state starts a fresh event list
            self.events[e] = []
        self._reset_wheels(idx)
        db, ds = s.get('drive_brake'), s.get('drive_steer')
        self._set_drive(idx, float(db[src]) if db is not None else 0.0, float(ds[src]) if ds is not None else 0.0)
        if self.spin_on:
            ws = s.get('wheel_spin')
            if ws is not None:
                self.wheel_spin[idx] = np.asarray(ws[src], dtype=np.float64)
            else:
                self._match_spin(idx, np.full(len(np.atleast_1d(idx)), self.spin_air if ph == 1 else 1.0))

    # ------------------------------------------------------------------------------------------- views
    def state(self, e: int = 0) -> dict:
        """FlightState of vehicle e (copies; spec/README.md section 7) incl. drive {brake, steer} (5.13); twin extras pc,
        thrust, step (+ wheel_spin)."""
        S, P = self.S, self.prop
        s = {'t': float(self.k[e] * self.dt), 'p': S[0:3, e].copy(), 'q': S[3:7, e].copy(), 'v': S[7:10, e].copy(),
             'w': S[10:13, e].copy(), 'comB': self.C[:, e].copy(),
             'tank': {'m': float(P.m[e]), 'T': float(P.T[e]), 'p': float(P.p[e])}, 'valve': P.valve[e].copy(),
             'armed': bool(self.armed[e]), 'phase': PHASE_NAMES[self.phase[e]], 'drive': self.drive(e), 'pc': float(P.pc[e]),
             'thrust': P.thrust[e].copy(), 'step': int(self.k[e])}
        if self.spin_on:
            s['wheel_spin'] = self.wheel_spin[e].copy()
        return s

    def drive(self, e: int = 0) -> dict:
        """Brake pressure (0..1) and front road-wheel steer angle (rad, + = front edge to the right) of vehicle e (5.13)."""
        return {'brake': float(self.drive_brake[e]), 'steer': float(self.drive_steer[e])}

    def kerb(self, e: int = 0):
        """First kerb hit of vehicle e (a loaded wheel on a street box off the deck) as {'t', 'wheel'}, or None."""
        if not self.kerb_hit[e]:
            return None
        return {'t': float(self.kerb_t[e]), 'wheel': int(self.kerb_wheel[e])}

    def wheel_load(self, e: int = 0) -> dict:
        """Normal, lateral and longitudinal (along -forward) tyre force per wheel in the last integrator step."""
        return {'N': self.wheel_N[e].copy(), 'lat': self.wheel_lat[e].copy(), 'long': self.wheel_long[e].copy()}

    def wheel_peak(self, e: int = 0) -> dict:
        """Per-episode peaks (from the deck edge on) of N and |F_lat|, lateral impulse since touchdown (N s)."""
        return {'N': self.peak_N[e].copy(), 'lat': self.peak_lat[e].copy(), 'lat_impulse': self.lat_impulse[e].copy()}

    def wheel_overload(self, e: int = 0):
        """First wheel overload of vehicle e as {'t', 'wheel', 'quantity'} (twin.js wheelOverload), or None."""
        if not self.wo[e]:
            return None
        return {'t': float(self.wo_t[e]), 'wheel': int(self.wo_wheel[e]), 'quantity': OVERLOAD[int(self.wo_q[e])]}

    def flat(self) -> np.ndarray:
        """(N, 13) [p, q, v, w]."""
        return self.S.T.copy()
