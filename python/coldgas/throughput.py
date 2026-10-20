"""Throughput of the Python envs (python -m coldgas.throughput): env steps (control steps, 20 integrator steps each)
per second with random valve commands on a task, including episode resets. Twin-only rates are reported too."""
from __future__ import annotations

import sys
import time

import numpy as np

from .env import ColdGasEnv
from .twin import Twin
from .vecenv import BatchColdGasEnv


def scalar(task, seconds=5.0):
    env = ColdGasEnv(task, seed=0)
    env.reset()
    rng = np.random.default_rng(0)
    n, t0 = 0, time.time()
    while time.time() - t0 < seconds:
        _, _, te, tr, _ = env.step(rng.random(10) * (rng.random(10) < 0.5))
        n += 1
        if te or tr:
            env.reset(n)
    return n / (time.time() - t0)


def batched(task, N, seconds=5.0):
    env = BatchColdGasEnv(task, n=N, seed=0)
    env.reset()
    rng = np.random.default_rng(0)
    n, t0 = 0, time.time()
    while time.time() - t0 < seconds:
        env.step(rng.random((N, 10)) * (rng.random((N, 10)) < 0.5))
        n += N
    return n / (time.time() - t0)


def twin_only(N, steps=50):
    tw = Twin(N)
    tw.reset([{'type': 'state', 'p': [0, 300, 0], 'q': [0, 0, 0, 1], 'v': [0, 0, 0], 'w': [0.3, 0.2, 0.1]}] * N)
    u = np.full((N, 10), 0.2)
    t0 = time.time()
    for _ in range(steps):
        tw.step(u)
    return N * steps / (time.time() - t0)


if __name__ == '__main__':
    task = sys.argv[1] if len(sys.argv) > 1 else 'escape-soft-landing'
    print(f'scalar env       {scalar(task):9.0f} steps/s')
    for N in (64, 256, 1024):
        print(f'batched N={N:<5d} {batched(task, N):9.0f} steps/s   (twin only, in the air: {twin_only(N):9.0f})')
