"""另一个项目 ~/Downloads/ppo 的 LTV-MPC（沿上次预测线性化 + 每步一次凸 QP，PIQP 内点法；MuJoCo Roadster，
只判硬着陆 / 车身触地）上做 FALL 文案的两个实验：w_fuel = 0（代价里没有用气项）vs 0.5（线性用气项，默认）。
同一批回合（坡道冲出 + 种子 100000 起的随机初态，名义动力学），记录用气、触地时刻、最大下沉速度、底部推力 / 车重。
不改 ppo 目录：只 import 它的模块；安全下沉曲线的减速度 DECEL 是模块常量，用 decel= 覆盖。
报告：experiments/reports/gas-two-costs（补充 B）。
cd ~/Downloads/ppo && .venv/bin/python ~/Downloads/车辆破坏仿真/tools/flight/gas/twocost_ltv.py [--episodes 30]
    [--fuels 0,0.5] [--set decel=5.5 w_sink=0.2 ...] [--out <json>]"""
import argparse, json, multiprocessing as mp, sys
from pathlib import Path
sys.path.insert(0, str(Path.home() / "Downloads/ppo"))
import gymnasium as gym
import numpy as np
import roadster_env  # noqa: F401
from roadster_env import GAS_KG
from export_trajectories import RAMP_START
from mpc_controller import MPCController, nominal_model

def episode(args):
    seed, options, kw, params = args
    import mpc_controller
    kw = dict(kw); mpc_controller.DECEL = kw.pop("decel", 2.5)   # 安全下沉曲线的减速度（模块常量）
    env = gym.make("RoadsterLanding-v0", randomize_dynamics=False)
    u = env.unwrapped
    obs, _ = env.reset(seed=seed, options=options)
    ctrl = MPCController(params, **kw); ctrl.reset()
    rows, done, info = [], False, {}
    while not done:
        a = ctrl(obs)
        h, vz = obs[9] * 20.0, obs[10] * 10.0
        obs, _, term, trunc, info = env.step(a)
        F = float((u.data.act[:4] * u._max_force[:4]).sum())   # 4 个底部喷口：滤波后油门 × 满推力 (N)
        rows.append((h, vz, F, info["gas"]))
        done = term or trunc
    R = np.array(rows); W = u.model.body_mass.sum() * 9.81
    td = int(np.argmax(R[:, 0] < 0.15)) if (R[:, 0] < 0.15).any() else len(R) - 1
    air = R[:td]
    return dict(seed=seed, outcome=info["outcome"], gas=GAS_KG - info["gas"], gas_td=GAS_KG - R[td, 3], t_td=td * 0.02,
                vmax=float(-air[:, 1].min()) if len(air) else 0.0, vz_td=float(R[td, 1]), fb_w=float(air[:, 2].mean() / W) if len(air) else 0.0,
                hold=float((air[:, 2] >= 0.5 * W).mean()) if len(air) else 0.0, h0=float(R[0, 0]),
                F=[round(x / W, 3) for x in R[:, 2]], h=[round(x, 2) for x in R[:, 0]], vz=[round(x, 2) for x in R[:, 1]])

if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--episodes", type=int, default=30); ap.add_argument("--out", default="ltv.json")
    ap.add_argument("--set", nargs="*", default=[]); ap.add_argument("--fuels", default="0,0.5"); a = ap.parse_args()
    base = {k: eval(v) for k, v in (x.split("=") for x in a.set)}
    params = nominal_model()
    fuels = [float(x) for x in a.fuels.split(",")]
    variants = {f"fuel{f:g}": {**base, "w_fuel": f} for f in fuels}
    jobs = [(name, (0, RAMP_START, kw, params)) for name, kw in variants.items()] + [(name, (100_000 + i, None, kw, params)) for name, kw in variants.items() for i in range(a.episodes)]
    with mp.get_context("spawn").Pool(14) as pool:
        res = pool.map(episode, [j[1] for j in jobs])
    out = {}
    for (name, j), r in zip(jobs, res):
        r["ramp"] = j[1] is not None; out.setdefault(name, []).append(r)
    json.dump(dict(base=base, variants=variants, results=out), open(a.out, "w"))
    for name, rs in out.items():
        ok = [r for r in rs if r["outcome"] == "landed"]
        m = lambda k, xs=ok: np.mean([r[k] for r in xs])
        ramp = rs[0]
        print(f"{name:8} landed {len(ok)}/{len(rs)}  gas {m('gas'):.1f} kg (to td {m('gas_td'):.1f})  t_td {m('t_td'):.2f} s  vmax {m('vmax'):.1f}  F/W in air {m('fb_w'):.2f}  hold {m('hold'):.0%}"
              f"  | ramp: {ramp['outcome']} {ramp['gas']:.1f} kg t_td {ramp['t_td']:.2f} vmax {ramp['vmax']:.1f}")
