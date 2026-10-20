"""Batched FlightState: dict of arrays with a leading vehicle axis, the common input of observers and controllers.

A single FlightState (spec/README.md section 7: t, p, q, v, w, comB, tank{m,T,p}, valve, armed, phase) becomes a
batch of one with `batch_of(state)`; the twin produces a batch view with `from_twin(twin)`.
"""
from __future__ import annotations

import numpy as np

PHASE_CODE = {'deck': 0, 'air': 1, 'ground': 2}


def batch_of(s: dict) -> dict:
    f = lambda k: np.asarray(s[k], dtype=np.float64).reshape(1, -1)
    return {'t': np.array([float(s['t'])]), 'p': f('p'), 'q': f('q'), 'v': f('v'), 'w': f('w'), 'comB': f('comB'),
            'tank_m': np.array([float(s['tank']['m'])]), 'tank_T': np.array([float(s['tank']['T'])]),
            'tank_p': np.array([float(s['tank']['p'])]), 'valve': f('valve'), 'armed': np.array([bool(s['armed'])]),
            'phase': np.array([PHASE_CODE[s['phase']] if isinstance(s['phase'], str) else int(s['phase'])])}


def from_twin(tw) -> dict:
    S, P = tw.S, tw.prop
    return {'t': tw.k * tw.dt, 'p': S[0:3].T, 'q': S[3:7].T, 'v': S[7:10].T, 'w': S[10:13].T, 'comB': tw.C.T,
            'tank_m': P.m, 'tank_T': P.T, 'tank_p': P.p, 'valve': P.valve, 'armed': tw.armed, 'phase': tw.phase}
