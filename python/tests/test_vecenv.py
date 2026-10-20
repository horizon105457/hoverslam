"""The batched env equals the scalar env lane by lane, bit for bit (incl. auto-reset)."""
import numpy as np

from coldgas.env import ColdGasEnv
from coldgas.rng import Mulberry32
from coldgas.vecenv import BatchColdGasEnv


def _actions(seed, n, steps):
    r = Mulberry32(seed)
    return np.array([[[r() if r() < 0.5 else 0.0 for _ in range(10)] for _ in range(n)] for _ in range(steps)])


def test_batched_equals_scalar():
    N, steps = 4, 140
    U = _actions(9, N, steps)
    vec = BatchColdGasEnv('escape-soft-landing', n=N, seed=20)
    obs, info = vec.reset()
    log = [[] for _ in range(N)]
    for k in range(steps):
        o, r, te, tr, inf = vec.step(U[k])
        for i in range(N):
            fin = inf['final'].get(i)
            log[i].append((fin['obs'] if fin else o[i].copy(), r[i], te[i], tr[i], fin['metrics'] if fin else None, o[i].copy()))
    for i in range(N):
        env = ColdGasEnv('escape-soft-landing', seed=20 + i)
        o0, _ = env.reset()
        assert (o0 == obs[i]).all()
        ep = 0
        for k in range(steps):
            o, r, te, tr, inf = env.step(U[k][i])
            fo, fr, fte, ftr, fm, onext = log[i][k]
            assert (o == fo).all() and r == fr and te == fte and tr == ftr, (i, k)
            if te or tr:
                assert inf['metrics'] == fm
                ep += 1
                o, _ = env.reset(20 + i + ep * N)
                assert (o == onext).all()
