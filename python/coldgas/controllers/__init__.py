"""Controller registry, mirror of src/flight/controllers/index.js (same names, configs and defaults).

Interface: reset(task=..., seed=...) once per episode; act(state, info=None) -> u_cmd (NU,) in [0, 1] every
CONTROL_DT, from FlightState fields + task + spec only. Vectorised controllers also offer act_batch(bstate) -> (N, NU)
(coldgas.state batch dicts) and reset_batch(n, task); MPPI uses them for rollouts.
Config: 'zero' | 'pd' | 'replay' | 'mlp' | 'mppi' | {'name': ..., **settings}; pd defaults come from
src/flight/controllers/defaults.json; replay needs traj (dict) or file, mlp policy (dict) or file.
"""
from __future__ import annotations

import json

from ..spec import ROOT

with open(ROOT / 'src' / 'flight' / 'controllers' / 'defaults.json', encoding='utf-8') as _f:
    DEFAULTS = json.load(_f)


def create_controller(name_or_config='zero'):
    cfg = {'name': name_or_config} if isinstance(name_or_config, str) else dict(name_or_config)
    name = cfg.pop('name')
    if name == 'zero':
        from .zero import Zero
        return Zero(**cfg)
    if name == 'pd':
        from .pd import PD
        return PD(**cfg)
    if name == 'replay':
        from .replay import Replay
        return Replay(**cfg)
    if name == 'mlp':
        from .mlp import MLP
        return MLP(**cfg)
    if name == 'mppi':
        from .mppi import MPPI
        return MPPI(**cfg)
    raise KeyError(f'unknown controller {name}')


CONTROLLERS = ('zero', 'pd', 'replay', 'mlp', 'mppi')
