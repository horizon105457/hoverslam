"""Cold-gas propulsion model v1 (spec/README.md section 3): Python mirror of src/flight/propulsion.js.

`Propulsion` is the scalar mirror (same API, same operation order, plain floats); `BatchPropulsion` is the same
model on N vehicles at once (numpy, used by the batched twin). Both reproduce the JS reference bit for bit:
pow/sqrt agree between V8 and CPython/numpy; exp goes through coldgas.fmath (the fdlibm algorithm V8 uses).
"""
from __future__ import annotations

import math

import numpy as np

from . import fmath
from .spec import PROPULSION, WORLD


def exit_mach(eps: float, g: float) -> float:
    """Supersonic exit Mach number for area ratio eps (bisection, fixed 80 iterations: deterministic)."""
    k = (g + 1) / (2 * (g - 1))

    def ratio(M):
        return (1 / M) * ((2 / (g + 1)) * (1 + 0.5 * (g - 1) * M * M)) ** k

    lo, hi = 1.0, 50.0
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if ratio(mid) < eps:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


class _Constants:
    """Per-kit constants shared by the scalar and batched models (computed exactly as in the JS constructor)."""

    def __init__(self, spec: dict, ambient: float):
        self.spec = spec
        g = self.g = spec['gas']['gamma']
        R = self.R = spec['gas']['R']
        self.pa = ambient
        self.n = len(spec['nozzles'])
        V = 0.0
        for b in spec['tank']['bottles']:
            V += b['volume']
        self.V = V
        self.rho0 = spec['tank']['p0'] / (R * spec['tank']['T0'])
        self.m0 = self.rho0 * self.V
        self.K = spec['tank']['p0'] / self.rho0 ** spec['tank']['polytropic_n']
        self.At, self.Ae, self.Me, self.peRatio, self.TeRatio = [], [], [], [], []
        for z in spec['nozzles']:
            At = math.pi * 0.25 * z['throat_diameter'] * z['throat_diameter']
            M = exit_mach(z['expansion_ratio'], g)
            f = 1 + 0.5 * (g - 1) * M * M
            self.At.append(At)
            self.Ae.append(At * z['expansion_ratio'])
            self.Me.append(M)
            self.peRatio.append(f ** (-g / (g - 1)))
            self.TeRatio.append(1 / f)
        self.choke = math.sqrt(g / R) * (2 / (g + 1)) ** ((g + 1) / (2 * (g - 1)))

    def feed_pressure(self, p: float) -> float:
        reg = self.spec['regulator']
        pc = min(reg['p_set'], p - reg['dp_min'])
        return pc if pc > 2 * self.pa else 0.0

    def lag(self, dt: float) -> tuple[float, float]:
        """(1 - exp(-dt/tau_open), 1 - exp(-dt/tau_close)), as the JS reference computes it per step."""
        v = self.spec['valve']
        return 1 - fmath.exp(-dt / v['tau_open']), 1 - fmath.exp(-dt / v['tau_close'])


class Propulsion(_Constants):
    """Scalar mirror of the JS class: `Propulsion(spec?, ambient?)`, reset(), step(u, dt) -> thrust,
    attributes valve/thrust/mdot (lists of NU floats), pc, m/T/p, tank {m,T,p}, m0."""

    def __init__(self, spec: dict = PROPULSION, ambient: float = WORLD['air']['pressure']):
        super().__init__(spec, ambient)
        self._lag_dt, self._lag = None, (0.0, 0.0)
        self.reset()

    def reset(self) -> None:
        self.m = self.m0
        self.T = self.spec['tank']['T0']
        self.p = self.spec['tank']['p0']
        self.pc = 0.0
        self.valve = [0.0] * self.n
        self.thrust = [0.0] * self.n
        self.mdot = [0.0] * self.n

    def feedPressure(self) -> float:  # noqa: N802 (JS name)
        return self.feed_pressure(self.p)

    def step(self, u, dt: float) -> list[float]:
        v = self.spec['valve']
        deadband = v['deadband']
        if dt != self._lag_dt:
            self._lag_dt, self._lag = dt, self.lag(dt)
        k_open, k_close = self._lag
        s = self.valve
        for i in range(self.n):
            c = float(u[i])
            c = 1.0 if c > 1 else (0.0 if c < deadband else c)
            s[i] += (c - s[i]) * (k_open if c > s[i] else k_close)
        pc = self.pc = self.feed_pressure(self.p)
        Tc = self.T
        mtot = 0.0
        for i in range(self.n):
            if pc <= 0 or s[i] <= 0:
                self.mdot[i] = 0.0
                self.thrust[i] = 0.0
                continue
            md = s[i] * self.At[i] * pc * self.choke / math.sqrt(Tc)
            ve = self.Me[i] * math.sqrt(self.g * self.R * Tc * self.TeRatio[i])
            F = md * ve + s[i] * (pc * self.peRatio[i] - self.pa) * self.Ae[i]
            self.mdot[i] = md
            self.thrust[i] = F if F > 0 else 0.0
            mtot += md
        m = self.m - mtot * dt
        if m < 0:
            m = 0.0
        self.m = m
        rho = m / self.V
        self.p = self.K * rho ** self.spec['tank']['polytropic_n']
        self.T = self.p / (rho * self.R) if rho > 0 else self.T
        return self.thrust

    @property
    def tank(self) -> dict:
        return {'m': self.m, 'T': self.T, 'p': self.p}


class BatchPropulsion(_Constants):
    """The same model for N vehicles: valve/thrust/mdot (N, NU), m/T/p/pc (N,). Identical per element to
    `Propulsion` (sequential sum over nozzles, same expression order)."""

    def __init__(self, n_envs: int, spec: dict = PROPULSION, ambient: float = WORLD['air']['pressure']):
        super().__init__(spec, ambient)
        self.N = n_envs
        self._At, self._Ae, self._Me = np.array(self.At), np.array(self.Ae), np.array(self.Me)
        self._pe, self._Te = np.array(self.peRatio), np.array(self.TeRatio)
        self.valve = np.zeros((n_envs, self.n))
        self.thrust = np.zeros((n_envs, self.n))
        self.mdot = np.zeros((n_envs, self.n))
        self.m = np.full(n_envs, self.m0)
        self.T = np.full(n_envs, float(spec['tank']['T0']))
        self.p = np.full(n_envs, float(spec['tank']['p0']))
        self.pc = np.zeros(n_envs)
        self._lag_dt, self._lag = None, (0.0, 0.0)

    def reset(self, idx=slice(None)) -> None:
        self.m[idx] = self.m0
        self.T[idx] = self.spec['tank']['T0']
        self.p[idx] = self.spec['tank']['p0']
        self.pc[idx] = 0.0
        self.valve[idx] = 0.0
        self.thrust[idx] = 0.0
        self.mdot[idx] = 0.0

    def step(self, u: np.ndarray, dt: float) -> np.ndarray:
        """u: (N, NU) commanded openings (finite)."""
        v = self.spec['valve']
        if dt != self._lag_dt:
            self._lag_dt, self._lag = dt, self.lag(dt)
        k_open, k_close = self._lag
        c = np.where(u > 1, 1.0, np.where(u < v['deadband'], 0.0, u))
        s = self.valve
        s += (c - s) * np.where(c > s, k_open, k_close)
        reg = self.spec['regulator']
        pc = np.minimum(reg['p_set'], self.p - reg['dp_min'])
        pc = self.pc = np.where(pc > 2 * self.pa, pc, 0.0)
        Tc = self.T
        on = (pc[:, None] > 0) & (s > 0)
        with np.errstate(all='ignore'):
            md = s * self._At * pc[:, None] * self.choke / np.sqrt(Tc)[:, None]
            ve = self._Me * np.sqrt(self.g * self.R * Tc[:, None] * self._Te)
            F = md * ve + s * (pc[:, None] * self._pe - self.pa) * self._Ae
        md = np.where(on, md, 0.0)
        self.mdot[:] = md
        self.thrust[:] = np.where(on & (F > 0), F, 0.0)
        mtot = np.zeros(self.N)
        for i in range(self.n):
            mtot = mtot + md[:, i]
        m = self.m - mtot * dt
        m = np.where(m < 0, 0.0, m)
        self.m[:] = m
        rho = m / self.V
        self.p[:] = self.K * rho ** self.spec['tank']['polytropic_n']
        with np.errstate(all='ignore'):
            self.T[:] = np.where(rho > 0, self.p / (rho * self.R), self.T)
        return self.thrust
