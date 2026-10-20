"""mulberry32, bit-identical to rng() in src/softbody/solver.js and src/flight/rng.js (spec/README.md section 7)."""
from __future__ import annotations

import numpy as np

_M = 0xFFFFFFFF
_C = 0x6D2B79F5


def _imul(a: int, b: int) -> int:
    return (a * b) & _M


class Mulberry32:
    def __init__(self, seed: int):
        self.s = (int(seed) & _M) or 1  # JS: seed >>> 0 || 1

    def __call__(self) -> float:
        s = self.s = (self.s + _C) & _M
        t = _imul(s ^ (s >> 15), 1 | s)
        t = ((t + _imul(t ^ (t >> 7), 61 | t)) & _M) ^ t
        return ((t ^ (t >> 14)) & _M) / 4294967296

    def uniform(self, a: float, b: float) -> float:
        return a + (b - a) * self()

    def array(self, n: int) -> np.ndarray:
        """The next n draws at once (numpy), identical to n calls: the state is a counter (s += C per draw) and the
        output a hash of it, so it vectorises with wrapping uint32 arithmetic."""
        s = ((self.s + np.arange(1, n + 1, dtype=np.uint64) * _C) & _M).astype(np.uint32)
        if n:
            self.s = int(s[-1])
        t = (s ^ (s >> np.uint32(15))) * (s | np.uint32(1))
        t = (t + (t ^ (t >> np.uint32(7))) * (t | np.uint32(61))) ^ t
        return (t ^ (t >> np.uint32(14))).astype(np.float64) / 4294967296

    def gauss_array(self, n: int) -> np.ndarray:
        """n standard normals as rng.js gauss(): sum of 12 draws - 6 (exact sums, so any summation order agrees)."""
        return self.array(12 * n).reshape(n, 12).sum(axis=1) - 6.0
