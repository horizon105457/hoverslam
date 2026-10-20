"""mulberry32, bit-identical to rng() in src/softbody/solver.js and src/flight/rng.js (spec/README.md section 7)."""
from __future__ import annotations

_M = 0xFFFFFFFF


def _imul(a: int, b: int) -> int:
    return (a * b) & _M


class Mulberry32:
    def __init__(self, seed: int):
        self.s = (int(seed) & _M) or 1  # JS: seed >>> 0 || 1

    def __call__(self) -> float:
        s = self.s = (self.s + 0x6D2B79F5) & _M
        t = _imul(s ^ (s >> 15), 1 | s)
        t = ((t + _imul(t ^ (t >> 7), 61 | t)) & _M) ^ t
        return ((t ^ (t >> 14)) & _M) / 4294967296

    def uniform(self, a: float, b: float) -> float:
        return a + (b - a) * self()
