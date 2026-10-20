"""PPO on the batched twin (needs the `rl` extra: torch), exporting coldgas-policy@1 for the JS/Python `mlp` controller.

    python -m coldgas.ppo --task hover-hold --steps 3000000 --envs 256 [--out DIR] [--seed 0]

Gaussian policy (tanh MLP mean, state-independent log std), separate value net, GAE, clipped surrogate, running
observation normalisation (frozen into the exported file). The env sees clip(a, 0, 1) and the exported policy uses
transform 'clip01' on the mean action, so evaluation is deterministic and bit-identical in JS and Python.
--bc-steps N warm-starts the policy mean by imitating the pd autopilot on N env steps (pd + exploration noise,
labelled with pd's clean command) before PPO; the export is still a plain MLP.
With BC the observation normalisation is frozen at the BC statistics and the critic is pre-fitted on the BC
rollouts' discounted returns (otherwise PPO tends to wreck the cloned policy within ~50 updates).
Writes config.json, curve.json (one row per update), policy.json (final), policy-best.json (best success rate over
the last 200 training episodes), policy-<update>.json checkpoints.
"""
from __future__ import annotations

import argparse
import json
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

from .contract import NU, POLICY_FORMAT, MODEL
from .obs import OBS_FORMAT
from .spec import ROOT, spec_hash
from .vecenv import BatchColdGasEnv


class RunningMeanStd:
    def __init__(self, n):
        self.mean, self.var, self.count = np.zeros(n), np.ones(n), 1e-4

    def update(self, x):
        bm, bv, bc = x.mean(0), x.var(0), x.shape[0]
        d, tot = bm - self.mean, self.count + bc
        self.mean = self.mean + d * bc / tot
        self.var = (self.var * self.count + bv * bc + d * d * self.count * bc / tot) / tot
        self.count = tot


def mlp(n_in, n_out, hidden):
    layers, n = [], n_in
    for h in hidden:
        layers += [nn.Linear(n, h), nn.Tanh()]
        n = h
    layers.append(nn.Linear(n, n_out))
    return nn.Sequential(*layers)


class Agent(nn.Module):
    def __init__(self, n_obs, hidden=(64, 64), log_std=-0.7, bias0=0.2):
        super().__init__()
        self.pi = mlp(n_obs, NU, hidden)
        self.v = mlp(n_obs, 1, hidden)
        with torch.no_grad():
            self.pi[-1].weight.mul_(0.01)
            self.pi[-1].bias.fill_(bias0)
        self.log_std = nn.Parameter(torch.full((NU,), log_std))

    def dist(self, x):
        return torch.distributions.Normal(self.pi(x), self.log_std.exp())


def export_policy(agent: Agent, rms: RunningMeanStd, task: str, clip: float, meta: dict) -> dict:
    layers = []
    lin = [m for m in agent.pi if isinstance(m, nn.Linear)]
    for k, L in enumerate(lin):
        layers.append({'W': L.weight.detach().double().numpy().tolist(), 'b': L.bias.detach().double().numpy().tolist(),
                       'act': 'linear' if k == len(lin) - 1 else 'tanh'})
    from .obs import obs_names
    from .tasks import load_task
    return {'format': POLICY_FORMAT, 'spec_hash': spec_hash(), 'model': MODEL, 'task': task,
            'obs': {'format': OBS_FORMAT, 'names': obs_names(load_task(task)), 'mean': rms.mean.tolist(),
                    'std': np.sqrt(rms.var + 1e-8).tolist(), 'clip': clip},
            'layers': layers, 'action': {'transform': 'clip01'}, 'meta': meta}


def behaviour_clone(agent, env, obs, rms, norm, steps, noise, rng, out, log, gamma=0.99, epochs=30, batch=4096):
    """Warm start: imitate the pd autopilot (DAgger-style data: execute pd + noise, label with pd's clean command)."""
    from .controllers.pd import PD
    from .state import from_twin
    pd = PD()
    pd.reset_batch(env.N, env.task)
    X, Y, Rw, D = [], [], [], []
    for _ in range(max(1, steps // env.N)):
        u = pd.act_batch(from_twin(env.twin))
        X.append(obs.copy())
        Y.append(u.copy())
        a = np.clip(u + rng.normal(0, noise, u.shape) * (rng.random((env.N, 1)) < 0.7), 0.0, 1.0)
        obs, r, te, tr, info = env.step(a)
        Rw.append(r.copy())
        D.append(te | tr)
        if info['final']:
            pd.reset_batch(env.N, env.task, idx=np.array(sorted(info['final'])))
    # discounted returns of the (noisy) pd rollouts: a value target so PPO does not start from a random critic
    G, Ret = np.zeros(env.N), []
    for r, d in zip(reversed(Rw), reversed(D)):
        G = r + gamma * G * (1 - d)
        Ret.append(G.copy())
    Ret = np.concatenate(Ret[::-1])
    X = np.concatenate(X)
    rms.update(X)
    Xt = torch.as_tensor(norm(X), dtype=torch.float32)
    Yt = torch.as_tensor(np.concatenate(Y), dtype=torch.float32)
    opt = torch.optim.Adam(agent.pi.parameters(), lr=1e-3)
    n = Xt.shape[0]
    for ep in range(epochs):
        perm = torch.as_tensor(rng.permutation(n))
        tot = 0.0
        for s in range(0, n, batch):
            j = perm[s:s + batch]
            loss = ((agent.pi(Xt[j]) - Yt[j]) ** 2).mean()
            opt.zero_grad()
            loss.backward()
            opt.step()
            tot += float(loss) * len(j)
        if log and (ep % 10 == 9 or ep == epochs - 1):
            print(f'bc epoch {ep + 1}/{epochs} mse {tot / n:.5f} ({n} samples)', flush=True)
    Rt = torch.as_tensor(Ret, dtype=torch.float32)
    optv = torch.optim.Adam(agent.v.parameters(), lr=1e-3)
    for ep in range(epochs):
        perm = torch.as_tensor(rng.permutation(n))
        vt = 0.0
        for s in range(0, n, batch):
            j = perm[s:s + batch]
            loss = ((agent.v(Xt[j]).squeeze(-1) - Rt[j]) ** 2).mean()
            optv.zero_grad()
            loss.backward()
            optv.step()
            vt += float(loss) * len(j)
    if log:
        print(f'bc value fit mse {vt / n:.4f}', flush=True)
    (out / 'bc.json').write_text(json.dumps({'samples': int(n), 'mse': tot / n, 'value_mse': vt / n, 'noise': noise}))
    obs, _ = env.reset()
    return obs


def train(task='hover-hold', steps=3_000_000, envs=256, horizon=32, epochs=5, minibatch=4096, lr=3e-4, gamma=0.99,
          lam=0.95, clip=0.2, ent=0.0, vf=0.5, obs_clip=10.0, seed=0, out=None, threads=2, checkpoints=4, log=True,
          bc_steps=0, bc_noise=0.15, log_std=None, freeze_obs=None):
    torch.manual_seed(seed)
    torch.set_num_threads(threads)
    rng = np.random.default_rng(seed)
    stamp = datetime.now().strftime('%Y%m%d-%H%M%S')
    out = Path(out) if out else ROOT / 'experiments' / 'runs' / f'ppo-{task}' / stamp
    out.mkdir(parents=True, exist_ok=True)
    cfg = dict(task=task, steps=steps, envs=envs, horizon=horizon, epochs=epochs, minibatch=minibatch, lr=lr, gamma=gamma,
               lam=lam, clip=clip, ent=ent, vf=vf, obs_clip=obs_clip, seed=seed, bc_steps=bc_steps, bc_noise=bc_noise,
               log_std=log_std,
               spec_hash=spec_hash())
    (out / 'config.json').write_text(json.dumps(cfg, indent=1))
    env = BatchColdGasEnv(task, n=envs, seed=1_000_000 * (seed + 1))
    obs, _ = env.reset()
    rms = RunningMeanStd(env.obs_size)
    rms.update(obs)
    agent = Agent(env.obs_size, log_std=(-0.7 if log_std is None else log_std))
    opt = torch.optim.Adam(agent.parameters(), lr=lr, eps=1e-5)
    norm = lambda o: np.clip((o - rms.mean) / np.sqrt(rms.var + 1e-8), -obs_clip, obs_clip)
    freeze_obs = bool(bc_steps) if freeze_obs is None else freeze_obs  # keep the cloned policy's input scaling
    if bc_steps:
        obs = behaviour_clone(agent, env, obs, rms, norm, bc_steps, bc_noise, rng, out, log, gamma)
    n_upd = steps // (envs * horizon)
    curve, t0, done_eps, best = [], time.time(), [], None
    ck = set(np.linspace(n_upd // checkpoints, n_upd, checkpoints, dtype=int).tolist()) if checkpoints else set()
    for upd in range(1, n_upd + 1):
        frac = 1 - (upd - 1) / n_upd
        for g in opt.param_groups:
            g['lr'] = lr * frac
        B_obs = np.zeros((horizon, envs, env.obs_size), np.float32)
        B_act = np.zeros((horizon, envs, NU), np.float32)
        B_logp = np.zeros((horizon, envs), np.float32)
        B_rew = np.zeros((horizon, envs), np.float32)
        B_done = np.zeros((horizon, envs), np.float32)
        B_val = np.zeros((horizon + 1, envs), np.float32)
        B_boot = np.zeros((horizon, envs), np.float32)  # value of the final obs of truncated episodes
        raw = []
        for t in range(horizon):
            x = torch.as_tensor(norm(obs), dtype=torch.float32)
            with torch.no_grad():
                d = agent.dist(x)
                a = d.sample()
                B_logp[t] = d.log_prob(a).sum(-1).numpy()
                B_val[t] = agent.v(x).squeeze(-1).numpy()
            B_obs[t], B_act[t] = x.numpy(), a.numpy()
            raw.append(obs)
            obs, r, te, tr, info = env.step(np.clip(a.numpy().astype(np.float64), 0.0, 1.0))
            B_rew[t], B_done[t] = r, (te | tr)
            for i, f in info['final'].items():
                done_eps.append((f['metrics']['return'], bool(f['metrics']['success']), f['reason']))
                if tr[i]:
                    with torch.no_grad():
                        B_boot[t, i] = agent.v(torch.as_tensor(norm(f['obs'][None]), dtype=torch.float32)).item()
        if not freeze_obs:
            rms.update(np.concatenate(raw))
        with torch.no_grad():
            B_val[horizon] = agent.v(torch.as_tensor(norm(obs), dtype=torch.float32)).squeeze(-1).numpy()
        adv = np.zeros((horizon, envs), np.float32)
        last = np.zeros(envs, np.float32)
        for t in reversed(range(horizon)):
            nxt = B_val[t + 1] * (1 - B_done[t]) + B_boot[t]
            delta = B_rew[t] + gamma * nxt - B_val[t]
            last = delta + gamma * lam * (1 - B_done[t]) * last
            adv[t] = last
        ret = adv + B_val[:horizon]
        X = torch.as_tensor(B_obs.reshape(-1, env.obs_size))
        A = torch.as_tensor(B_act.reshape(-1, NU))
        LP = torch.as_tensor(B_logp.reshape(-1))
        ADV = torch.as_tensor(adv.reshape(-1))
        RET = torch.as_tensor(ret.reshape(-1))
        n = X.shape[0]
        for _ in range(epochs):
            perm = torch.as_tensor(rng.permutation(n))
            for s in range(0, n, minibatch):
                j = perm[s:s + minibatch]
                d = agent.dist(X[j])
                lp = d.log_prob(A[j]).sum(-1)
                ratio = (lp - LP[j]).exp()
                ad = (ADV[j] - ADV[j].mean()) / (ADV[j].std() + 1e-8)
                pl = -torch.min(ratio * ad, ratio.clamp(1 - clip, 1 + clip) * ad).mean()
                vl = ((agent.v(X[j]).squeeze(-1) - RET[j]) ** 2).mean()
                loss = pl + vf * vl - ent * d.entropy().sum(-1).mean()
                opt.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(agent.parameters(), 0.5)
                opt.step()
        recent = done_eps[-200:]
        row = {'update': upd, 'steps': upd * envs * horizon, 'wall_s': time.time() - t0, 'episodes': len(done_eps),
               'return_mean': float(np.mean([e[0] for e in recent])) if recent else None,
               'success_rate': float(np.mean([e[1] for e in recent])) if recent else None,
               'reasons': {k: sum(1 for e in recent if e[2] == k) for k in {e[2] for e in recent}},
               'std': float(agent.log_std.detach().exp().mean())}
        curve.append(row)
        if log and (upd % 10 == 0 or upd == n_upd):
            print(f"upd {upd}/{n_upd} steps {row['steps']} ret {row['return_mean']} succ {row['success_rate']} "
                  f"std {row['std']:.3f} {row['steps'] / row['wall_s']:.0f} steps/s {row['reasons']}", flush=True)
        if upd in ck:
            pol = export_policy(agent, rms, task, obs_clip, {'algo': 'ppo', 'update': upd, 'steps': row['steps'], **cfg})
            (out / f'policy-{upd:05d}.json').write_text(json.dumps(pol))
        # best policy so far (success rate over the last 200 episodes, then return), judged once per 5 updates
        if recent and len(recent) >= 100 and upd % 5 == 0:
            score = (row['success_rate'], row['return_mean'])
            if best is None or score > best[0]:
                best = (score, upd)
                pol = export_policy(agent, rms, task, obs_clip, {'algo': 'ppo', 'update': upd, 'steps': row['steps'], 'selected': 'best',
                                                                 'success_rate_train': row['success_rate'], **cfg})
                (out / 'policy-best.json').write_text(json.dumps(pol))
        (out / 'curve.json').write_text(json.dumps(curve))
    pol = export_policy(agent, rms, task, obs_clip, {'algo': 'ppo', 'update': n_upd, 'steps': n_upd * envs * horizon, **cfg})
    (out / 'policy.json').write_text(json.dumps(pol))
    torch.save({'agent': agent.state_dict(), 'rms': (rms.mean, rms.var, rms.count)}, out / 'agent.pt')
    latest = out.parent / 'latest'  # experiments/configs can point at runs/ppo-<task>/latest/policy.json
    if latest.is_symlink() or not latest.exists():
        if latest.is_symlink():
            latest.unlink()
        latest.symlink_to(out.name)
    if (out / 'policy-best.json').exists():  # the evaluation configs read policy.json: make it the selected one
        (out / 'policy-final.json').write_text((out / 'policy.json').read_text())
        (out / 'policy.json').write_text((out / 'policy-best.json').read_text())
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(prog='python -m coldgas.ppo')
    ap.add_argument('--task', default='hover-hold')
    ap.add_argument('--steps', type=int, default=3_000_000)
    ap.add_argument('--envs', type=int, default=256)
    ap.add_argument('--horizon', type=int, default=32)
    ap.add_argument('--lr', type=float, default=3e-4)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--threads', type=int, default=2)
    ap.add_argument('--bc-steps', type=int, default=0, help='behaviour-cloning warm start from pd (env steps of data)')
    ap.add_argument('--bc-noise', type=float, default=0.15)
    ap.add_argument('--log-std', type=float, default=None)
    ap.add_argument('--out')
    a = ap.parse_args(argv)
    out = train(a.task, a.steps, a.envs, a.horizon, lr=a.lr, seed=a.seed, out=a.out, threads=a.threads,
                bc_steps=a.bc_steps, bc_noise=a.bc_noise, log_std=a.log_std)
    print(f'-> {out}')


if __name__ == '__main__':
    main()
