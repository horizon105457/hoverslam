"""Batched FlightState: dict of arrays with a leading vehicle axis, the common input of observers and controllers.

A single FlightState (spec/README.md section 7: t, p, q, v, w, comB, tank{m,T,p}, valve, armed, phase, drive?) becomes
a batch of one with `batch_of(state)`; the twin produces a batch view with `from_twin(twin)`. The optional drive
{brake, steer} (5.13) maps to drive_brake / drive_steer (read by Twin.load_state, 0 if absent).
"""
from __future__ import annotations

import numpy as np

PHASE_CODE = {'deck': 0, 'air': 1, 'ground': 2}


def batch_of(s: dict) -> dict:
    f = lambda k: np.asarray(s[k], dtype=np.float64).reshape(1, -1)
    b = {'t': np.array([float(s['t'])]), 'p': f('p'), 'q': f('q'), 'v': f('v'), 'w': f('w'), 'comB': f('comB'),
         'tank_m': np.array([float(s['tank']['m'])]), 'tank_T': np.array([float(s['tank']['T'])]),
         'tank_p': np.array([float(s['tank']['p'])]), 'valve': f('valve'), 'armed': np.array([bool(s['armed'])]),
         'phase': np.array([PHASE_CODE[s['phase']] if isinstance(s['phase'], str) else int(s['phase'])])}
    if s.get('wheel_spin') is not None:  # twin extra (wheel_spin enabled), read by Twin.load_state
        b['wheel_spin'] = f('wheel_spin')
    if s.get('drive') is not None:  # brake pressure / steer angle (twins, soft-body rig), read by Twin.load_state
        b['drive_brake'] = np.array([float(s['drive']['brake'])])
        b['drive_steer'] = np.array([float(s['drive']['steer'])])
    return b


def from_twin(tw) -> dict:
    S, P = tw.S, tw.prop
    b = {'t': tw.k * tw.dt, 'p': S[0:3].T, 'q': S[3:7].T, 'v': S[7:10].T, 'w': S[10:13].T, 'comB': tw.C.T,
         'tank_m': P.m, 'tank_T': P.T, 'tank_p': P.p, 'valve': P.valve, 'armed': tw.armed, 'phase': tw.phase,
         'drive_brake': tw.drive_brake, 'drive_steer': tw.drive_steer}
    if tw.spin_on:
        b['wheel_spin'] = tw.wheel_spin
    return b
