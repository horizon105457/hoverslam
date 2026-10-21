"""Batched environment: N independent episodes on one numpy twin (MPPI rollouts, PPO data collection).

Lane i runs exactly what ColdGasEnv(task, seed) would (same twin code with N lanes, same tracker and observer):
tests/test_vecenv.py checks equality bit for bit. Episodes auto-reset in the same step (gymnasium "same-step"
mode): the returned obs of a finished lane is the first obs of its next episode; the final obs, metrics and
reason are in info['final'] = {lane: {...}}. Lane i's j-th episode uses seed seed0 + i + j * N.
"""
from __future__ import annotations

import numpy as np

from .contract import NU, NA
from .obs import Observer
from .state import from_twin
from .tasks import load_task, sample_initial, TaskTracker
from .twin import Twin


def _subset(s: dict, idx) -> dict:
    return {k: v[idx] for k, v in s.items()}


class BatchColdGasEnv:
    def __init__(self, task='escape-soft-landing', n: int = 64, seed: int = 0, preset: str | None = None, auto_reset: bool = True,
                 vehicle: dict | None = None):
        self.task = load_task(task) if isinstance(task, str) or 'initial' not in task else task
        self.N, self.seed0, self.preset, self.auto_reset = n, seed, preset, auto_reset
        self.twin = Twin(n, arm=self.task.get('arm', 'start'), vehicle=vehicle)  # vehicle: parameter override (fitting)
        self.trackers = [TaskTracker(self.task) for _ in range(n)]
        self.observer = Observer(self.task, n)
        self.obs_size, self.nu, self.na = self.observer.size, NU, NA  # valves / full action incl. drive (5.13)
        self.episode = np.zeros(n, dtype=np.int64)
        self.seeds = np.zeros(n, dtype=np.int64)
        self.inits = [None] * n

    def _reset_lanes(self, idx):
        inits = []
        for i in idx:
            s = int(self.seed0 + i + self.episode[i] * self.N)
            self.seeds[i] = s
            self.inits[i] = sample_initial(self.task, s, self.preset)
            inits.append(self.inits[i])
        self.twin.reset(inits, idx)
        for i in idx:
            self.trackers[i].reset(self.twin, int(i))
        self.observer.reset(idx)

    def reset(self, seed: int | None = None):
        if seed is not None:
            self.seed0 = seed
        self.episode[:] = 0
        idx = np.arange(self.N)
        self._reset_lanes(idx)
        return self.observer.observe(from_twin(self.twin)), {'seed': self.seeds.copy()}

    def step(self, u):
        self.twin.step(np.asarray(u, dtype=np.float64).reshape(self.N, -1))  # (N, NU) or (N, NA)
        rew = np.zeros(self.N)
        term = np.zeros(self.N, dtype=bool)
        trunc = np.zeros(self.N, dtype=bool)
        final = {}
        for i, tr in enumerate(self.trackers):
            if tr.done:  # only without auto_reset
                continue
            r, te, tu, reason = tr.update(self.twin)
            rew[i], term[i], trunc[i] = r, te, tu
        obs = self.observer.observe(from_twin(self.twin))
        done = np.nonzero(term | trunc)[0]
        for i in done.tolist():
            final[i] = {'obs': obs[i].copy(), 'reason': self.trackers[i].reason, 'metrics': self.trackers[i].metrics(self.twin),
                        'seed': int(self.seeds[i]), 'preset': self.inits[i].get('preset')}
        if self.auto_reset and len(done):
            self.episode[done] += 1
            self._reset_lanes(done)
            obs[done] = self.observer.observe(_subset(from_twin(self.twin), done), done)
        return obs, rew, term, trunc, {'final': final, 'seed': self.seeds.copy()}

    def batch_state(self) -> dict:
        return from_twin(self.twin)
