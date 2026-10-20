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
node tools/flight/bench.mjs experiments/configs/mppi-soft-landing.json [workers=16] [seeds=1:11] [only=mppi-soft]
                               # same config schema / outputs as coldgas.bench, episodes on worker_threads,
                               # writes runs/<name>-js/<stamp>/
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
| `mppi` | JS, Py (bit-identical) | `samples horizon knot replan sigma temperature plan_dt base seed w_* v_ok curb_*` + soft-body options `w_slip w_td_slip w_gnd_slip w_gnd_yaw curb_steps curb_margin bottom_gate base_pd` | sampling MPC (`src/flight/controllers/mppi.js` = `coldgas/controllers/mppi.py`): K noisy residual sequences on top of `pd`, rolled out on a twin loaded from the FlightState, softmax-weighted; settings / cost documented in `defaults.json` `mppi`. JS: ~2.5 s per escape episode (Python ~18 s); runs closed loop on the soft body |

## MPPI on the soft body (closed loop)

The JS `mppi` runs inside the soft-body loop: `ThrusterRig` calls `act()` every 20 ms with its lattice state
estimate, MPPI loads it into a rigid planning twin (`RigidTwin.loadState`, `plan_dt` 2 ms) and replans every
`replan` steps. Cost: 96 samples x 1.5 s ≈ 33 ms per plan, ~3.2 s per 6.5 s flight (96 plans), i.e. ~2 % of the
~150-170 s soft-body run (ap: 146 s, mppi: 147 s side by side); no worker threads needed.

```sh
C='mppi:{"samples":96,"horizon":1.5,"knot":0.15,"replan":3,"curb_lookahead":2,"curb_steps":8,"curb_margin":1.5,"w_td_tilt":400,"w_td_rate":10,"w_slip":0.3,"w_td_slip":10,"w_gnd_slip":5,"w_gnd_yaw":1,"bottom_gate":3,"base_pd":{"td_speed":0.5}}'
node tools/sim/flight.mjs preset=nose controller="$C" out=mppi-nose dur=6.5     # -> public/sims/flight-mppi-nose.sim
node tools/sim/flight.mjs preset=roof controller="$C" out=mppi-roof dur=6.5
node tools/sim/flight.mjs preset=roof seed=1 controller="$C" out=x dur=6.5 outDir=experiments/runs/soft  # scratch
```

`outDir=` keeps scratch runs out of `public/sims` (writes `.sim` + `.json` with `meta.flight`); registry controllers
given as JSON are recorded as their short name with the settings in `meta.flight.controllerConfig`, MPPI's plan
count / time in `meta.flight.controllerStats`. Soft-body runs are deterministic for (preset, seed, controller).

Why the extra settings (all opt-in, default = the original Python cost, implemented in both languages): with the
baseline settings MPPI lands softly but strikes the hull 0.3-0.8 s later on both nose and roof. Causes found on the
soft body, none of which the twin reproduces: (1) landing with 10-30° slip makes the tyres roll the car onto a
sill (`w_slip`, `w_td_slip`); (2) a 1-2° nose-up touchdown, harmless on the twin, lands the 70 % rear-heavy car on
its rear axle and the rear floor (rest clearance 7 cm) bottoms out, where the twin's 400 kN/m bump stop leaves
3 cm to spare (`bottom_gate`: the residual on the bottom nozzles fades out below 3 m so pd flies a level touchdown;
`base_pd.td_speed` 0.5 m/s); (3) the rear-heavy car spins out ~1 s after landing (`w_gnd_slip`, `w_gnd_yaw`: the
closed loop pays for yaw damping with the rear jets); (4) the roof preset drifts across the road at 7-11 m/s and the
1 s COM probe sees the sidewalk corner only after the side jets can still stop it (`curb_lookahead` 2 s, 8 probes,
1.5 m footprint margin). A clearance-margin cost on the twin's hull points was tried and dropped (the planner
hovered instead of landing).

Soft body, escape-soft-landing criteria (`ThrusterRig.summary`), lattice of 2026-10-20 13:25 (hood latch):

| run | controller | success | vy_td m/s | tilt_td ° | rate_td rad/s | max tilt in hold ° | hull strike | gas kg | parts off |
|---|---|---|---|---|---|---|---|---|---|
| nose | ap (`flight-soft`) | ✓ | 0.91 | 0.06 | 0.04 | 1.3 | – | 60.4 | 0 |
| nose | pd (`flight-pd-nose`) | × | 0.80 | 0.09 | 0.15 | 60.2 | 3.76 s | 47.4 | 10 |
| nose | replay of Python MPPI (`flight-replay-mppi-nose`) | × | 6.69 | 50.7 | 0.78 | 176.9 | 2.46 s | 77.9 | 13 |
| nose | mppi, baseline settings | × | 0.95 | 4.6 | 0.34 | 4.6 | 3.64 s | 67.1 | 7 |
| nose | **mppi-soft** (`flight-mppi-nose`) | ✓ | 0.51 | 0.64 | 0.20 | 1.7 | – | 64.1 | 0 |
| roof | ap (`flight-recover`) | ✓ | 0.96 | 0.05 | 0.11 | 1.2 | – | 79.3 | 0 |
| roof | pd (`flight-pd-roof`) | × | 0.80 | 0.07 | 0.13 | 50.9 | 3.78 s | 50.7 | 10 |
| roof | mppi, baseline settings | × | 0.63 | 7.9 | 0.14 | 7.9 | 4.07 s | 59.6 | 5 |
| roof | **mppi-soft** (`flight-mppi-roof`) | ✓ | 0.61 | 0.64 | 0.19 | 3.3 | – | 63.1 | 0 |

mppi-soft on further soft-body cases (outDir runs): nose seed 1 ✓ 1.28 m/s, nose seed 2 ✓ 0.59, roof seed 1 ✓ 0.94
(hits something 1.1 s after touchdown, after the hold), roof seed 2 ✓ 0.68, flat ✓ 1.11, rear ✓ 0.52, side ✓ 0.51,
corner × (0.46 m/s, hull strike 0.23 s after touchdown): 9/10 + the two recordings above.

Twin (`mppi-soft-landing`, 6 presets x seeds 1..10, JS and Python give identical episodes):

| controller | n | success | hull strike | vy_td | tilt_td | gas kg | return | corner | flat | nose | rear | roof | side | s/ep JS / Py |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| pd | 60 | 42% | 58% | 0.80 | 0.01 | 48.75 | -3.94 | 0% | 60% | 40% | 80% | 10% | 60% | 0.0 / 1.1 |
| mppi | 60 | 90% | 0% | 0.82 | 3.85 | 55.42 | 5.51 | 100% | 100% | 90% | 100% | 70% | 80% | 2.7 / 24 |
| mppi-soft | 60 | 93% | 3% | 0.71 | 1.00 | 51.61 | 6.13 | 100% | 100% | 80% | 100% | 90% | 90% | 2.4 / 33 |

(`baseline-soft-landing` with the shared noise stream: mppi 15/18 (83 %, failures: nose s2 hovers into the
timeout, roof s1 / side s2 land at z ≈ 26-28 by the kerb line and tilt past 12° in the hold); the 18/18 in the table below used numpy's normal RNG, which is gone: with 3 seeds per preset
(and seeds 0 and 1 drawing the same initial condition) the spread between noise streams is that large, hence the
10-seed config. Python s/ep measured with 16 workers on 18 cores.)

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
