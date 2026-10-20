# experiments/ — 挑战任务 × 算法对比

- `configs/*.json` — experiment definitions (task, controllers with their settings, presets, seeds). Versioned.
- `runs/<experiment>/<stamp>/` — outputs, git-ignored: `config.json` (the config + spec hash + resolved seeds),
  `episodes.jsonl` (one line of metrics per episode), `summary.json` (aggregates per controller and per preset),
  `traj/<label>__<preset>__s<seed>.json` (`coldgas-traj@1`, see `spec/README.md` §7). PPO runs write
  `runs/ppo-<task>/<stamp>/` with `curve.json`, `policy.json` (`coldgas-policy@1`) and checkpoints.
  Every output records the spec hash; a run is reproducible from (spec hash, task, controller config, seed).

## Running

Python (venv at `python/.venv`, created with `uv venv --python 3.14 python/.venv && uv pip install --python
python/.venv/bin/python -e "python[test,rl]"`):

```sh
python/.venv/bin/python -m coldgas.bench experiments/configs/baseline-soft-landing.json [--workers 6] [--seeds 0:5]
python/.venv/bin/python -m coldgas.ppo --task hover-hold --steps 4000000 --envs 256
python/.venv/bin/python -m coldgas.bench experiments/configs/ppo-hover-eval.json     # evaluate an exported policy
python/.venv/bin/python -m coldgas.sanity       # physical sanity report of the twin
python/.venv/bin/python -m coldgas.throughput   # env steps/s, scalar and batched
python/.venv/bin/python -m pytest python/tests  # unit + parity tests (needs node for the JS side)
```

JS (no build step, Node >= 22):

```sh
node tools/flight/run.mjs task=escape-soft-landing controller=pd seed=0 preset=nose out=experiments/runs/js/nose.json
node tools/flight/run.mjs task=hover-hold controller=mlp policy=experiments/runs/ppo-hover-hold/<stamp>/policy.json
node tools/flight/run.mjs controller=replay traj=<coldgas-traj@1 file> task=... seed=... preset=...
node tools/flight/throughput.mjs
node tests/parity/check.mjs    # JS re-runs the Python goldens (+ compares both golden sets)
node tests/parity/make.mjs     # rewrite the JS goldens;  python -m coldgas.parity make|check for the other side
```

## Config schema (`coldgas.bench`)

```jsonc
{
  "name": "baseline-soft-landing",          // output folder under runs/ (default: file name)
  "description": "...",                     // free text
  "task": "escape-soft-landing",            // spec/tasks/<id>.json (extends resolved)
  "controllers": [                          // each: registry name + settings (+ optional "label", default = name)
    { "name": "zero" },
    { "name": "pd", "kv": 8.0 },
    { "name": "mppi", "samples": 96, "horizon": 1.5 },
    { "name": "mlp", "label": "ppo", "policy": "runs/ppo-hover-hold/<stamp>/policy.json" }  // path: config dir or repo root
  ],
  "presets": ["flat", "nose"],              // launch presets to force; null = let the task draw them from the seed
  "seeds": { "start": 0, "count": 3 },      // or an explicit list [0, 1, 2]
  "record": "all",                          // trajectories: "all" | "first" (first seed per preset) | "none"
  "workers": 6                              // processes (episodes are independent)
}
```

Episodes are (controller × preset × seed); the task's randomisation (speed, angle, kick strength, …) still applies
per seed (spec/README.md §8.2). `summary.json` holds success / hull-strike rates, termination reasons, mean /
median / min / max of the touchdown metrics, gas, return and per-preset success; the runner prints a markdown table.

## Controller registry

Same names, settings and defaults in JS (`src/flight/controllers/index.js`) and Python (`coldgas.controllers`);
defaults in `src/flight/controllers/defaults.json`. Interface: `reset({task, seed})`, `act(FlightState, info) →
u_cmd[10] ∈ [0,1]` every 20 ms, FlightState fields only (so they also run on a state estimated from the soft body).

| name | languages | settings | what |
|---|---|---|---|
| `zero` | JS, Py | – | no thrust (the original crash) |
| `pd` | JS, Py (bit-identical) | `att_kp att_kd td_speed flare_height brake_frac kv tilt_cut hover_kh hover_vmax` | rights the car with differential bottom thrust (min-norm allocation), free-falls and then follows the braking profile `v_ref(h) = −√(td² + 2·brake_frac·a_max·(h − flare)⁺)` with feed-forward; hover tasks hold the target height, then land |
| `replay` | JS, Py | `traj` (object) or `file` | plays `u_cmd` of a `coldgas-traj@1` |
| `mlp` | JS, Py (bit-identical) | `policy` (object) or `file` | runs a `coldgas-policy@1` (PPO export) |
| `mppi` | Py | `samples horizon knot replan sigma temperature plan_dt base seed w_*` | sampling MPC on the batched twin, residual on `pd` by default; planning cost in `coldgas/controllers/mppi.py` |

## Results so far

`baseline-soft-landing` (escape-soft-landing, 6 presets × 3 seeds, spec 2026-10-20):

| controller | n | success | hull strike | vy_td mean (m/s) | tilt_td mean (°) | gas (kg) | return | s/episode |
|---|---|---|---|---|---|---|---|---|
| zero | 18 | 0 % | 100 % | 12.65 | 150.4 | 0.0 | −32.96 | 1.0 |
| pd | 18 | 61 % | 39 % | 0.80 | 0.01 | 49.2 | −0.14 | 2.2 |
| mppi | 18 | 100 % | 0 % | 0.73 | 3.37 | 56.7 | 6.98 | 32.0 |

pd fails on `roof` and `corner` (and one `nose` seed): its slow powered descent carries the car onto the sidewalk
side of the crossing and it rolls into a kerb within the 1 s upright hold; MPPI sees the kerb in its rollouts and
steers / times the landing into the kerb-free crossing.

`pd-precision-pad`, `pd-tumble-recovery` (12 task-drawn seeds each): pd 0 % on the pad task (it does not aim: its
long powered descent lands 21–36 m past the pad), 25 % on tumble-recovery (it rights every car within ~0.5 m of
leaving the deck, then rolls into a kerb); zero thrust 0 % on both.

`ppo-hover-hold` (`python -m coldgas.ppo --task hover-hold --steps 3000000 --envs 256 --bc-steps 204800 --log-std -2
--lr 1e-4`): from scratch PPO (4–5 M steps) never succeeded (first it learned to hop and land early, which the task
did not penalise until `terminal_timeout` was extended to `landed_early`, then it crashed every episode); warm-started
by imitating pd (MSE 4e-4) it reached 98.5–99.5 % training success within 25–50 updates (0.2–0.4 M steps) and then
destabilised (success back to 0 by update ~100: every episode `landed_early`), so the run keeps the best checkpoint
(`policy-best.json`, copied to `policy.json`). `ppo-hover-eval` (20 held-out seeds, deterministic): zero 0 %, pd 100 %
(0.80 m/s, 83.6 kg, return 0.32), PPO 100 % (0.88 m/s, 80.8 kg, return 0.80). The same policy file in the JS `mlp`
controller (`node tools/flight/run.mjs task=hover-hold controller=mlp policy=...`) gives a bit-identical trajectory.

`ppo-escape-soft-landing` (same recipe, lr 3e-5, log std −2.3, stopped at 1 M steps): not solved. The cloned policy
keeps the car level (touchdown tilt ~1°) but starts braking too late (touchdown ~5.5 m/s, hull strike on every
preset); the sharp "free-fall, then brake" switch of pd is not captured by the 64-64 tanh MLP. Next steps: longer
runs, a time-to-impact feature in the observation, or a residual policy on pd.
