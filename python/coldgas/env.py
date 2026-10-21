"""Gymnasium-shaped single-vehicle environment, same API and semantics as src/flight/env.js.

    env = ColdGasEnv('escape-soft-landing', seed=0)
    obs, info = env.reset()                 # reset(seed=None, options={'preset': ..., 'init': ...})
    obs, reward, terminated, truncated, info = env.step(u)   # u: (NU,) valve commands in [0, 1], or (NA,) + brake, steer

Physics is coldgas.twin.Twin with N = 1 (so it equals any lane of the batched env bit for bit). If gymnasium is
installed the class is a gymnasium.Env with Box spaces and `register()` adds ColdGas-<task>-v1 ids; otherwise it is
a plain class with the same methods.
"""
from __future__ import annotations

import numpy as np

from .contract import NU, NA
from .obs import Observer
from .state import from_twin
from .tasks import load_task, sample_initial, TaskTracker
from .twin import Twin

try:  # optional
    import gymnasium as _gym
    from gymnasium import spaces as _spaces
except ImportError:  # pragma: no cover
    _gym = None

_Base = _gym.Env if _gym else object


class ColdGasEnv(_Base):
    metadata = {'render_modes': []}

    def __init__(self, task='escape-soft-landing', seed: int = 0, preset: str | None = None, vehicle: dict | None = None):
        self.task = load_task(task) if isinstance(task, str) or 'initial' not in task else task
        self.twin = Twin(1, arm=self.task.get('arm', 'start'), vehicle=vehicle)  # vehicle: parameter override (fitting)
        self.tracker = TaskTracker(self.task)
        self.observer = Observer(self.task, 1)
        self.seed, self.preset, self.init = seed, preset, None
        self.nu, self.na, self.obs_size = NU, NA, self.observer.size  # valves / full action incl. drive (5.13)
        if _gym:
            self.observation_space = _spaces.Box(-np.inf, np.inf, (self.obs_size,), np.float64)
            self.action_space = _spaces.Box(0.0, 1.0, (NU,), np.float64)

    def reset(self, seed: int | None = None, options: dict | None = None):
        options = options or {}
        if seed is not None:
            self.seed = seed
        if _gym:
            super().reset(seed=self.seed)
        init = options.get('init') or sample_initial(self.task, self.seed, options.get('preset', self.preset))
        self.init = init
        self.twin.reset([init])
        self.tracker.reset(self.twin, 0)
        self.observer.reset()
        return self.observer.observe(from_twin(self.twin))[0], self._info()

    def step(self, u):
        self.twin.step(np.asarray(u, dtype=np.float64).reshape(1, -1))  # NU valves or NA with the drive commands
        r, term, trunc, reason = self.tracker.update(self.twin)
        info = self._info()
        if term or trunc:
            info['reason'] = reason
            info['metrics'] = self.tracker.metrics(self.twin)
        return self.observer.observe(from_twin(self.twin))[0], r, term, trunc, info

    @property
    def state(self) -> dict:
        return self.twin.state(0)

    def _info(self) -> dict:
        tw = self.twin
        from .twin import PHASE_NAMES
        return {'t': float(tw.k[0] * tw.dt), 'phase': PHASE_NAMES[tw.phase[0]], 'armed': bool(tw.armed[0]),
                'preset': (self.init or {}).get('preset'), 'seed': self.seed}


def register():
    """Register ColdGas-<task>-v1 with gymnasium (no-op without gymnasium)."""
    if not _gym:
        return []
    from .spec import TASKS
    ids = []
    for tid in TASKS:
        gid = f'ColdGas-{tid}-v1'
        if gid not in _gym.registry:
            _gym.register(id=gid, entry_point='coldgas.env:ColdGasEnv', kwargs={'task': tid})
        ids.append(gid)
    return ids
