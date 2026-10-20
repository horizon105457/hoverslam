"""MPPI (model-predictive path integral control) on the batched twin. Python only.

A Controller like the others: act(FlightState) -> u_cmd. Every `replan` control steps the state is loaded into K
planning lanes (Twin.load_state), K perturbed command sequences are rolled out for `horizon` seconds and the
nominal sequence moves to their cost-weighted mean (softmax, temperature relative to the cost spread).
Commands are piecewise constant over `knot` seconds. With base='pd' the samples are residuals on top of the pd
autopilot evaluated inside each rollout (pd keeps the car upright, MPPI shapes the burn, uses rear/side nozzles to
steer the landing point away from kerbs, saves gas); base='none' plans raw valve commands.

The planning model may use a coarser integrator step (plan_dt) than the real twin: that is model mismatch on
purpose (speed), the executed physics is always the reference twin.
"""
from __future__ import annotations

import numpy as np

from .pd import PD
from ..contract import NU, CONTROL_DT, GROUPS
from ..obs import height_np
from ..propulsion import Propulsion
from ..spec import WORLD, PROPULSION
from ..state import batch_of, from_twin
from ..twin import Twin
from ..world import street_level, GROUND

DEFAULTS = {'samples': 128, 'horizon': 0.8, 'knot': 0.1, 'replan': 2, 'sigma': 0.3, 'temperature': 0.05,
            'plan_dt': 0.002, 'base': 'pd', 'seed': 0,
            # cost weights (planning cost; the benchmark still scores the task's own reward / success)
            'w_crash': 60.0, 'w_vy': 6.0, 'v_ok': 1.0, 'w_td_tilt': 40.0, 'w_td_rate': 3.0, 'w_tilt': 0.3, 'w_rate': 0.01,
            'w_gas': 0.3, 'w_stop': 0.5, 'w_curb': 8.0, 'w_pad': 2.0, 'w_height': 0.5, 'curb_lookahead': 1.0,
            'w_alt': 0.05, 'w_res': 0.5, 'w_deadline': 20.0}


class MPPI:
    name = 'mppi'

    def __init__(self, **cfg):
        c = dict(DEFAULTS)
        c.update(cfg)
        self.config = c
        self.K = int(c['samples'])
        self.cdt = CONTROL_DT
        self.H = max(1, round(c['horizon'] / CONTROL_DT))
        self.knot = max(1, round(c['knot'] / CONTROL_DT))
        self.nk = -(-self.H // self.knot)
        self.task = None

    def reset(self, task=None, seed=None):
        c = self.config
        self.task = task
        self.rng = np.random.default_rng(c['seed'] if seed is None else (int(c['seed']) * 1000003 + int(seed)))
        self.plan = Twin(self.K, arm=(task or {}).get('arm', 'start'), dt=c['plan_dt'])
        self.base = PD() if c['base'] == 'pd' else None
        self.base_real = PD() if c['base'] == 'pd' else None
        if self.base:
            self.base.reset_batch(self.K, task)
            self.base_real.reset(task=task)
        self.U = np.zeros((self.nk, NU))
        self.since = 10 ** 9
        self.g = WORLD['gravity']
        self.plans = 0
        P = Propulsion()  # nominal full-open bottom thrust (regulated feed, full tank) for the stoppability cost
        P.valve = [1.0] * NU
        self.Fb_nom = float(np.array(P.step(np.ones(NU), 1e-9))[GROUPS['bottom']].sum())

    def act(self, state, info=None):
        s = batch_of(state)
        ub = self.base_real.act_batch(s) if self.base else np.zeros((1, NU))  # keeps the real pd memory in step
        if not s['armed'][0]:
            return np.zeros(NU)
        if self.since >= self.config['replan']:
            if self.since < 10 ** 9:  # shift the nominal by the steps executed since the last plan
                sh = self.since // self.knot
                if sh:
                    self.U = np.concatenate([self.U[sh:], np.zeros((min(sh, self.nk), NU))])[: self.nk]
            self._plan(s)
            self.since = 0
        j = min(self.since // self.knot, self.nk - 1)
        self.since += 1
        return np.clip(ub[0] + self.U[j], 0.0, 1.0)

    # -------------------------------------------------------------------------------------------------
    def _plan(self, s):
        c, K, H, tw = self.config, self.K, self.H, self.plan
        noise = self.rng.normal(0.0, c['sigma'], (K, self.nk, NU))
        noise[0] = 0.0  # always evaluate the current nominal
        V = self.U[None] + noise
        tw.load_state(s, 0)
        if self.base:
            self.base.hold_t[:] = self.base_real.hold_t[0]
            self.base.landing[:] = self.base_real.landing[0]
        cost = c['w_res'] * np.mean(V * V, axis=(1, 2))  # residual effort (keeps MPPI close to the base when it can)
        td0, m_prev = tw.td.copy(), tw.prop.m.copy()
        alive = np.ones(K, dtype=bool)
        T = self.task or {}
        for h in range(H):
            bs = from_twin(tw)
            ub = self.base.act_batch(bs) if self.base else 0.0
            u = np.clip(ub + V[:, min(h // self.knot, self.nk - 1)], 0.0, 1.0)
            tw.step(u)
            cost += self._step_cost(tw, m_prev, td0, alive, h, T)
            m_prev = tw.prop.m.copy()
        cost += self._terminal_cost(tw, alive, T)
        # softmax weights with a temperature relative to the spread of the costs
        cmin = cost.min()
        lam = c['temperature'] * max(float(np.mean(cost) - cmin), 1e-6)
        w = np.exp(-(cost - cmin) / lam)
        w /= w.sum()
        self.U = np.clip(np.tensordot(w, V, axes=1), -1.0, 1.0)
        self.plans += 1
        self.last_cost = float(cost[0]), float(cmin)

    def _step_cost(self, tw, m_prev, td0, alive, h, T):
        c, S = self.config, tw.S
        cost = c['w_gas'] * (m_prev - tw.prop.m)
        tilt_c = 2 * (S[3] * S[3] + S[5] * S[5])  # 1 - cos(tilt)
        rate2 = S[10] * S[10] + S[11] * S[11] + S[12] * S[12]
        cost = cost + alive * (c['w_tilt'] * tilt_c + c['w_rate'] * rate2)
        crash = alive & tw.hs
        cost = cost + crash * c['w_crash'] * (2.0 - h / self.H)  # earlier is worse
        new_td = alive & tw.td & ~td0
        if new_td.any():
            d = tw.td_S
            vy = np.abs(d[8])
            tt = 2 * (d[3] * d[3] + d[5] * d[5])
            rr = d[10] * d[10] + d[11] * d[11] + d[12] * d[12]
            ctd = c['w_vy'] * np.maximum(vy - c['v_ok'], 0.0) ** 2 + c['w_td_tilt'] * tt + c['w_td_rate'] * rr
            if T.get('pad'):
                dx, dz = d[0] - T['pad']['center'][0], d[2] - T['pad']['center'][2]
                ctd = ctd + c['w_pad'] * (dx * dx + dz * dz)
            cost = cost + np.where(new_td, ctd, 0.0)
            td0 |= new_td
        hb = height_np(S[0:3].T, S[3:7].T, tw.C.T)
        if T.get('target') and self.base is not None:
            cost = cost + alive * ~self.base.landing * c['w_height'] * np.abs(hb - T['target']['height'])
            cost = cost + alive * self.base.landing * ~tw.td * c['w_alt'] * np.maximum(hb, 0.0)
        else:  # descent progress: being up in the air costs, so hovering out the horizon is never optimal
            cost = cost + alive * ~tw.td * c['w_alt'] * np.maximum(hb, 0.0)
        alive &= ~crash
        return cost

    def _terminal_cost(self, tw, alive, T):
        c, S, g = self.config, tw.S, self.g
        # deadline: still airborne when the task needs the car on its wheels (horizon minus the upright hold)
        t_need = T.get('horizon', 1e9) - T.get('success', {}).get('upright_hold', 0) - 0.3
        late = np.maximum(tw.k * tw.dt - t_need, 0.0)
        hb = height_np(S[0:3].T, S[3:7].T, tw.C.T)
        vy = S[8]
        # can the remaining descent still be stopped with ~70 % of the bottom thrust?
        Fb = self.Fb_nom * np.minimum(tw.prop.pc / PROPULSION['regulator']['p_set'], 1.0)
        cos_t = 1 - 2 * (S[3] * S[3] + S[5] * S[5])
        a_av = 0.7 * Fb * np.maximum(cos_t, 0) / tw.M - g
        a_req = np.where(vy < 0, vy * vy / (2 * np.maximum(hb - 0.3, 0.05)), 0.0)
        cost = np.where(~tw.td, c['w_stop'] * np.maximum(a_req - a_av, 0.0) + c['w_deadline'] * late, 0.0)
        # kerbs ahead: where will the COM be after curb_lookahead seconds of rolling / flying straight on?
        la = c['curb_lookahead']
        x1, z1 = S[0] + S[7] * la, S[2] + S[9] * la
        on_curb = (street_level(x1, z1) > GROUND['y']) | (street_level(S[0] + S[7] * 0.5 * la, S[2] + S[9] * 0.5 * la) > GROUND['y'])
        cost = cost + c['w_curb'] * on_curb
        if T.get('pad') and c['w_pad']:
            # ballistic landing-point estimate for vehicles still in the air
            t_hit = np.where(vy < 0, np.maximum(hb, 0) / np.maximum(-vy, 1.0), 1.0)
            dx, dz = S[0] + S[7] * t_hit - T['pad']['center'][0], S[2] + S[9] * t_hit - T['pad']['center'][2]
            cost = cost + np.where(~tw.td, 0.5 * c['w_pad'] * (dx * dx + dz * dz), 0.0)
        return alive * cost
