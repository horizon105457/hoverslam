"""Default observation vector `coldgas-obs@1` (spec/README.md section 8), mirror of src/flight/obs.js, batched."""
from __future__ import annotations

import numpy as np

from .contract import NOZZLE_IDS, CONTROL_DT
from .propulsion import Propulsion
from .spec import PROPULSION, WORLD
from .world import street_level

OBS_FORMAT = 'coldgas-obs@1'
OBS_BASE = ['up_x', 'up_y', 'up_z', 'vb_x', 'vb_y', 'vb_z', 'w_x', 'w_y', 'w_z', 'v_y', 'height', 'gas_frac', 'pc_ratio',
            'armed', *[f'valve_{i}' for i in NOZZLE_IDS]]
_M0 = Propulsion(PROPULSION, WORLD['air']['pressure']).m0
_REG, _PA = PROPULSION['regulator'], WORLD['air']['pressure']


def obs_names(task: dict | None) -> list[str]:
    n = list(OBS_BASE)
    if task and task.get('target'):
        n += ['height_error', 'hold_progress']
    if task and task.get('pad'):
        n += ['pad_dx', 'pad_dz']
    if task and task.get('recover_by_height') is not None:
        n += ['recover_margin']
    return n


def height_np(p, q, comB):
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    r3, r4, r5 = 2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)
    return p[:, 1] - (r3 * comB[:, 0] + r4 * comB[:, 1] + r5 * comB[:, 2]) - street_level(p[:, 0], p[:, 2])


class Observer:
    """Stateful (hold timer) observation builder for N vehicles; observe(bstate) once per control step."""

    def __init__(self, task: dict | None, n: int = 1):
        self.task, self.N = task, n
        self.names = obs_names(task)
        self.size = len(self.names)
        self.hold_t = np.zeros(n)

    def reset(self, idx=slice(None)):
        self.hold_t[idx] = 0.0

    def observe(self, s: dict, idx=None) -> np.ndarray:
        q, v = s['q'], s['v']
        x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
        R0, R1, R2 = 1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)
        R3, R4, R5 = 2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)
        R6, R7, R8 = 2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)
        n = q.shape[0]
        out = np.empty((n, self.size))
        out[:, 0], out[:, 1], out[:, 2] = R3, R4, R5
        out[:, 3] = R0 * v[:, 0] + R3 * v[:, 1] + R6 * v[:, 2]
        out[:, 4] = R1 * v[:, 0] + R4 * v[:, 1] + R7 * v[:, 2]
        out[:, 5] = R2 * v[:, 0] + R5 * v[:, 1] + R8 * v[:, 2]
        out[:, 6:9] = s['w']
        out[:, 9] = v[:, 1]
        h = height_np(s['p'], q, s['comB'])
        out[:, 10] = h
        out[:, 11] = s['tank_m'] / _M0
        pc = np.minimum(_REG['p_set'], s['tank_p'] - _REG['dp_min'])
        pc = np.where(pc > 2 * _PA, pc, 0.0)
        out[:, 12] = pc / _REG['p_set']
        out[:, 13] = np.where(s['armed'], 1.0, 0.0)
        out[:, 14:14 + len(NOZZLE_IDS)] = s['valve']
        k = 14 + len(NOZZLE_IDS)
        T = self.task
        if T and T.get('target'):
            tg = T['target']
            err = h - tg['height']
            ht = self.hold_t if idx is None else self.hold_t[idx]
            ht = np.where(np.abs(err) <= tg['tolerance'], ht + CONTROL_DT, 0.0)
            if idx is None:
                self.hold_t[:] = ht
            else:
                self.hold_t[idx] = ht
            pr = ht / tg['hold']
            out[:, k], out[:, k + 1] = err, np.where(pr > 1, 1.0, pr)
            k += 2
        if T and T.get('pad'):
            out[:, k], out[:, k + 1] = T['pad']['center'][0] - s['p'][:, 0], T['pad']['center'][2] - s['p'][:, 2]
            k += 2
        if T and T.get('recover_by_height') is not None:
            out[:, k] = (s['p'][:, 1] - street_level(s['p'][:, 0], s['p'][:, 2])) - T['recover_by_height']
        return out
