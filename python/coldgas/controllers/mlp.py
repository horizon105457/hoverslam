"""coldgas-policy@1 runner (spec/README.md section 7), mirror of src/flight/controllers/mlp.js.

Forward pass y_j = (sum_i W[j][i] x_i, sequential from 0) + b_j with fmath.tanh: bit-identical to JS. The batched
path (act_batch) keeps the same summation order (loop over inputs, vectorised over outputs and vehicles).
"""
from __future__ import annotations

import json

import numpy as np

from .. import fmath
from ..contract import NU, POLICY_FORMAT
from ..obs import Observer
from ..state import batch_of

_ACT = {'tanh': fmath.tanh_np, 'relu': lambda x: np.where(x > 0, x, 0.0), 'linear': lambda x: x}
_OUT = {'clip01': lambda a: np.where(a > 1, 1.0, np.where(a < 0, 0.0, a)), 'tanh01': lambda a: 0.5 * (fmath.tanh_np(a) + 1)}


def mlp_forward(policy: dict, x: np.ndarray) -> np.ndarray:
    """x: (N, n_in) -> (N, n_out); summation order as in mlp.js."""
    h = x
    for L in policy['layers']:
        W = np.asarray(L['W'], dtype=np.float64)
        b = np.asarray(L['b'], dtype=np.float64)
        s = np.zeros((h.shape[0], W.shape[0]))
        for i in range(W.shape[1]):
            s = s + W[:, i] * h[:, i:i + 1]
        h = _ACT[L.get('act') or 'linear'](s + b)
    return h


class MLP:
    name = 'mlp'

    def __init__(self, policy=None, file=None, **_):
        if policy is None and file is not None:
            with open(file, encoding='utf-8') as f:
                policy = json.load(f)
        if not policy or policy.get('format') != POLICY_FORMAT:
            raise ValueError(f'mlp: need a {POLICY_FORMAT} policy (policy= or file=)')
        self.policy = policy
        O = policy['obs']
        self.mean, self.std, self.clip = np.asarray(O['mean']), np.asarray(O['std']), O.get('clip', 10)
        self.out = _OUT[policy.get('action', {}).get('transform', 'clip01')]
        self.config = {'task': policy.get('task'), 'meta': policy.get('meta')}
        self.obs = None

    def reset(self, task=None, seed=0):
        self.reset_batch(1, task)

    def reset_batch(self, n, task=None, idx=None):
        if idx is None or self.obs is None:
            self.obs = Observer(task, n)
            if self.obs.size != len(self.mean):
                raise ValueError(f'mlp: policy expects {len(self.mean)} observations, task gives {self.obs.size}')
        else:
            self.obs.reset(idx)

    def act(self, state, info=None):
        return self.act_batch(batch_of(state))[0]

    def act_batch(self, s, idx=None):
        o = self.obs.observe(s, idx)
        z = (o - self.mean) / self.std
        x = np.where(z > self.clip, self.clip, np.where(z < -self.clip, -self.clip, z))
        u = self.out(mlp_forward(self.policy, x)[:, :NU])
        return np.where(np.asarray(s['armed'], dtype=bool)[:, None], u, 0.0)
