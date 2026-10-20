"""Controllers: pd and mlp agree bit for bit with JS on arbitrary states; replay reproduces an episode."""
import json
import math

import numpy as np

from coldgas.contract import NU, POLICY_FORMAT
from coldgas.controllers import create_controller
from coldgas.episode import run_episode
from coldgas.obs import obs_names
from coldgas.rng import Mulberry32
from coldgas.tasks import load_task


def random_states(n, seed):
    r = Mulberry32(seed)
    out = []
    for k in range(n):
        q = [r() - 0.5 for _ in range(4)]
        l = math.sqrt(sum(x * x for x in q))
        q = [x / l for x in q] if k % 3 else [0.02 * (r() - 0.5), r() - 0.5, 0.02 * (r() - 0.5), 1.0]
        l = math.sqrt(sum(x * x for x in q))
        q = [x / l for x in q]
        m = r() * 125
        out.append({'t': 0.02 * k, 'p': [-30 + 20 * r(), 0.3 + 10 * r(), 12 + 10 * r()], 'q': q,
                    'v': [8 * (r() - 0.5), -14 * r() + 2, 8 * (r() - 0.5)], 'w': [3 * (r() - 0.5) for _ in range(3)],
                    'comB': [0.0, 0.6, -0.42 + 0.1 * r()], 'tank': {'m': m, 'T': 200 + 90 * r(), 'p': 3e7 * (m / 125) ** 1.25},
                    'valve': [r() for _ in range(NU)], 'armed': k % 11 != 0, 'phase': ['air', 'air', 'ground', 'deck'][k % 4]})
    return out


def random_policy(n_in, seed, hidden=16):
    r = Mulberry32(seed)
    mk = lambda o, i: [[(r() - 0.5) * 0.8 for _ in range(i)] for _ in range(o)]
    return {'format': POLICY_FORMAT, 'task': None, 'obs': {'mean': [r() - 0.5 for _ in range(n_in)], 'std': [0.5 + r() for _ in range(n_in)], 'clip': 5},
            'layers': [{'W': mk(hidden, n_in), 'b': [r() - 0.5 for _ in range(hidden)], 'act': 'tanh'},
                       {'W': mk(hidden, hidden), 'b': [r() - 0.5 for _ in range(hidden)], 'act': 'relu'},
                       {'W': mk(NU, hidden), 'b': [r() for _ in range(NU)], 'act': 'linear'}],
            'action': {'transform': 'clip01'}}


def test_pd_and_mlp_match_js(node):
    states = random_states(300, 4)
    cases = {}
    for tid in ('escape-soft-landing', 'hover-hold', 'precision-pad'):
        t = load_task(tid)
        cases[tid] = {'task': t, 'policy': random_policy(len(obs_names(t)), 7)}
    js = node(f"""
import {{ createController }} from './src/flight/controllers/index.js';
const states = {json.dumps(states)}, cases = {json.dumps(cases)}, out = {{}};
for (const [id, c] of Object.entries(cases)) {{
  const pd = createController('pd'), mlp = createController({{ name: 'mlp', policy: c.policy }});
  pd.reset({{ task: c.task }}); mlp.reset({{ task: c.task }});
  out[id] = states.map(s => [Array.from(pd.act(s)), Array.from(mlp.act(s))]);
}}
console.log(JSON.stringify(out));""")
    for tid, c in cases.items():
        pd, mlp = create_controller('pd'), create_controller({'name': 'mlp', 'policy': c['policy']})
        pd.reset(task=c['task'])
        mlp.reset(task=c['task'])
        for s, (upd, umlp) in zip(states, js[tid]):
            assert pd.act(s).tolist() == upd
            assert mlp.act(s).tolist() == umlp
        assert any(any(x > 0 for x in u) for u, _ in js[tid])


def test_replay_reproduces_episode():
    a = run_episode('escape-soft-landing', 'pd', seed=3)
    b = run_episode('escape-soft-landing', {'name': 'replay', 'traj': a['traj']}, seed=3)
    assert a['traj']['state'] == b['traj']['state'] and a['metrics'] == b['metrics']
