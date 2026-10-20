import numpy as np

from ..contract import NU


class Zero:
    name = 'zero'

    def __init__(self, **_):
        self.config = {}

    def reset(self, task=None, seed=0):
        pass

    def reset_batch(self, n, task=None):
        pass

    def act(self, state, info=None):
        return np.zeros(NU)

    def act_batch(self, s, idx=None):
        return np.zeros((len(s['t']) if np.ndim(s['t']) else 1, NU))
