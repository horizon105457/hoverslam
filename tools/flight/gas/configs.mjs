// Writes the gas-study experiment configs experiments/configs/gas-*.json (explicit controller lists, so every run is
// readable and reproducible without this script). Re-run after changing a matrix here.
// usage: node tools/flight/gas/configs.mjs
import path from 'path';
import { ROOT, writeJSON } from './lib.mjs';

const P6 = ['flat', 'nose', 'rear', 'side', 'roof', 'corner'], S5 = [1, 2, 3, 4, 5], S3 = [1, 2, 3];
const base = { name: 'mpc', objective: 'gas' };
const r1 = x => +x.toFixed(2);
const C = {};
// late-burn cold start (heave only): closed until 1.0 s after arming, full 1.05-1.45 s (the vertical optimum of the
// reference ignites ~1.05 s after arming and burns ~0.6 s), closed after
const LATE = { type: 'xinit', schedule: { heave: [[0, 0], [1.0, 0], [1.05, 1], [1.45, 1], [1.5, 0]] } };
const HOVER = [0.42, 0, 0, 0, 0, 0, 0], EARLY = [0.7, 0, 0, 0, 0, 0, 0]; // heave 0.42 ≈ weight (pitch-free 26.6 kN max)

C['gas-objective'] = {
  description: 'Objective bias: the lead\'s land objective vs the unbiased gas objective (same CEM, defaults), plus land without its w_stop term and gas with land-like in-air attitude shaping.',
  base, controllers: [
    { label: 'land', objective: 'land' },
    { label: 'land-nostop', objective: { name: 'land', w_stop: 0 } },
    { label: 'gas' },
    { label: 'gas-airshape', objective: { name: 'gas', air_tilt: 3, air_rate: 0.05 } },
  ], presets: P6, seeds: S5, record: 'all',
};

const CO = [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2];
C['gas-coast'] = {
  description: 'Core sweep: forced free fall (every valve closed, plan and execution) for `coast` s after arming, then free CEM MPC with the gas objective.',
  base, controllers: CO.map(t => ({ label: `coast-${r1(t)}`, coast: t })), presets: P6, seeds: S5, record: 'all',
};
C['gas-hcoast'] = {
  description: 'Heave-only coast: the collective (heave channel) held at 0 until the first knot at or after arming + t (xmask), attitude channels free from arming; CEM, gas objective.',
  base, controllers: CO.map(t => ({ label: `hcoast-${r1(t)}`, optimizer: [{ type: 'xmask', heave_until: t }, { type: 'cem' }] })), presets: P6, seeds: S5, record: 'all',
};

// optimizer families at ~equal rollout budget per plan (~400), four cold starts
const FAM = {
  cem: [{ type: 'cem' }],
  mppi: [{ type: 'mppi' }],
  grad: [{ type: 'grad', iters: 2 }],
  xlbfgs: [{ type: 'xlbfgs', iters: 2 }],
  'cem>xlbfgs': [{ type: 'cem', iters: 2 }, { type: 'xlbfgs', iters: 1 }],
  'xlbfgs>cem': [{ type: 'xlbfgs', iters: 1 }, { type: 'cem', iters: 1 }],
  'mppi>grad': [{ type: 'mppi', iters: 2 }, { type: 'grad', iters: 1 }],
};
const INITS = { zero: {}, hover: { init: HOVER }, early: { init: EARLY }, late: { pre: LATE } };
C['gas-optim'] = {
  description: 'Optimizer families (sampling cem / mppi, gradient grad / xlbfgs, hybrids) at ~400 rollouts per plan, from four cold starts: zero, hover-ish constant heave 0.42, early constant heave 0.7, late burn (xinit). Gas objective.',
  base, controllers: Object.entries(FAM).flatMap(([f, st]) => Object.entries(INITS).map(([i, v]) => ({ label: `${f}@${i}`, optimizer: v.pre ? [v.pre, ...st] : st, ...(v.init ? { init: v.init } : {}) }))),
  presets: P6, seeds: S3, record: 'first',
};

// convergence vs rollout budget (zero cold start)
C['gas-budget'] = {
  description: 'Gas and success vs rollout budget per plan for each optimizer family (zero cold start, gas objective).',
  base, controllers: [
    ...[16, 32, 64, 128, 256, 512].map(k => ({ label: `cem-${k}x3`, optimizer: 'cem', cem: { samples: k } })),
    ...[16, 32, 64, 128, 256, 512].map(k => ({ label: `mppi-${k}x3`, optimizer: 'mppi', mppi: { samples: k } })),
    ...[1, 2, 4, 8].map(k => ({ label: `grad-${k}`, optimizer: 'grad', grad: { iters: k } })),
    ...[1, 2, 4, 8].map(k => ({ label: `xlbfgs-${k}`, optimizer: { type: 'xlbfgs', iters: k } })),
    ...[[64, 1], [128, 1], [128, 2], [256, 4]].map(([k, l]) => ({ label: `cem${k}x2>xlbfgs${l}`, optimizer: [{ type: 'cem', iters: 2, samples: k }, { type: 'xlbfgs', iters: l }] })),
    ...[4, 16].map(k => ({ label: `grad-spsa8-${k}`, optimizer: 'grad', grad: { iters: k, fd: 'spsa', spsa_samples: 8 } })),
    { label: 'xlbfgs-2-central', optimizer: { type: 'xlbfgs', iters: 2, fd: 'central' } },
    { label: 'xlbfgs-4-coarse', optimizer: { type: 'xlbfgs', iters: 4 }, knot: 0.1, knot_growth: 1.15 },
    { label: 'cem-128x3-coarse', knot: 0.1, knot_growth: 1.15 },
  ], presets: P6, seeds: S3, record: 'none',
};

C['gas-umax'] = {
  description: 'Thrust ceiling of the bottom group (umax.bottom): long gentle vs short hard burn. CEM, gas objective.',
  base, controllers: [0.5, 0.6, 0.7, 0.8, 0.9, 1.0].map(u => ({ label: `umax-${u}`, umax: { bottom: u } })), presets: P6, seeds: S5, record: 'first',
};
C['gas-vcap'] = {
  description: 'Touchdown-speed cap of the gas objective (the "smooth" requirement): gas vs touchdown speed (Pareto). CEM.',
  base, controllers: [0.5, 0.75, 1.0, 1.25, 1.5, 1.75].map(v => ({ label: `vcap-${v}`, objective: { name: 'gas', v_cap: v } })), presets: P6, seeds: S5, record: 'first',
};

// is the CEM's late righting (roll to 60-70°, right the car during the main burn) a local optimum? In-air rate / tilt
// shaping as a search aid only (the result is judged on gas and the task criteria), with and without the heave coast
const HC = h => ({ optimizer: [{ type: 'xmask', heave_until: h }, { type: 'cem' }] });
C['gas-attitude'] = {
  description: 'Search aid for early attitude correction: in-air angular-rate / tilt shaping (air_rate per (rad/s)² per step, air_tilt per (1 - cos)) on the gas objective, free and with the heave-only coast to 0.9 s (xmask). Judged on gas and success.',
  base, controllers: [
    ...[0.3, 1, 3].map(r => ({ label: `free-rate${r}`, objective: { name: 'gas', air_rate: r } })),
    ...[0, 0.3, 1, 3].map(r => ({ label: `hc0.9-rate${r}`, ...HC(0.9), objective: { name: 'gas', air_rate: r } })),
    ...[10, 30].map(t => ({ label: `hc0.9-tilt${t}`, ...HC(0.9), objective: { name: 'gas', air_tilt: t } })),
    { label: 'hc0.9-tilt10-rate1', ...HC(0.9), objective: { name: 'gas', air_tilt: 10, air_rate: 1 } },
    ...[0.7, 1.0].map(h => ({ label: `hc${h}-rate1`, ...HC(h), objective: { name: 'gas', air_rate: 1 } })),
  ], presets: P6, seeds: S5, record: 'first',
};

// can the MPC find pd's attitude-first solution? more torque exploration, more passes, more feedback (replan 1)
const CS1 = { channel_scale: { pitch: 1, roll: 1, pitch_r: 0.2, yaw: 0.2, surge: 0.2, sway: 0.2 } };
C['gas-explore'] = {
  description: 'MPC search settings that might find the attitude-first solution of pd: torque-channel exploration x2 (channel_scale pitch/roll 1), CEM sigma 0.2, 256 samples and 30 cold passes, replanning every step; free and with the heave-only coast to 0.9 s.',
  base, controllers: [
    { label: 'free-cs1', ...CS1 },
    { label: 'free-cs1-s0.2', ...CS1, cem: { sigma: 0.2 } },
    { label: 'hc0.9-cs1', ...HC(0.9), ...CS1 },
    { label: 'hc0.9-cs1-s0.2', ...HC(0.9), ...CS1, cem: { sigma: 0.2 } },
    { label: 'hc0.9-cs1-s0.2-256-fp30', ...HC(0.9), ...CS1, cem: { sigma: 0.2, samples: 256 }, first_passes: 30 },
    { label: 'hc0.9-replan1', ...HC(0.9), replan: 1 },
    { label: 'hc0.9-cs1-replan1', ...HC(0.9), ...CS1, replan: 1 },
  ], presets: P6, seeds: S5, record: 'first',
};

// non-MPC reference: the registry's pd pilot is the hand-written form of the hypothesis (right the car at once, fall
// freely, brake along v_ref(h) = -sqrt(td² + 2 brake_frac a_max (h - flare)+)); brake_frac -> later, harder braking
C['gas-pd'] = {
  description: 'Reference only (not MPC): the pd pilot (attitude first, free fall, late braking profile) over brake_frac x td_speed.',
  base: { name: 'pd' }, controllers: [0.4, 0.5, 0.6, 0.7, 0.8, 0.9].flatMap(b => [0.8, 1.5].map(v => ({ label: `pd-b${b}-td${v}`, brake_frac: b, td_speed: v }))),
  presets: P6, seeds: S5, record: 'first',
};

// robustness of the late burn: strategies x plan/plant mismatches the planner does not know about
const STRAT = {
  gas: {}, 'gas-hc0.9': { optimizer: [{ type: 'xmask', heave_until: 0.9 }, { type: 'cem' }] }, 'gas-vcap1.0': { objective: { name: 'gas', v_cap: 1.0 } },
  'gas-reserve0.8': { objective: { name: 'gas', reserve: 0.8 } }, land: { objective: 'land' },
};
const MIS = {
  nominal: {}, 'eta0.95': { exec: { eta: { bottom: 0.95 } } }, 'eta0.9': { exec: { eta: { bottom: 0.9 } } }, 'eta0.85': { exec: { eta: { bottom: 0.85 } } },
  delay1: { exec: { delay: 1 } }, delay2: { exec: { delay: 2 } }, 'plan_dt4ms': { plan_dt: 0.004 }, 'plan_dt1ms': { plan_dt: 0.001 },
};
C['gas-robust'] = {
  description: 'Robustness: strategies (free gas CEM; heave-only coast to 0.9 s; touchdown cap 1.0 m/s; braking reserve 0.8; land) x mismatches unknown to the planner (bottom thrust x0.95/0.9/0.85, 1-2 control steps of command delay, planning step 4 ms; 1 ms = exact model as control). tools/flight/gas/bench.mjs exec block.',
  base, controllers: Object.entries(STRAT).flatMap(([sn, sv]) => Object.entries(MIS).map(([mn, mv]) => ({ label: `${sn}|${mn}`, ...sv, ...mv }))), presets: P6, seeds: S3, record: 'none',
};

// second round: a reserve that the plan actually respects (stronger weight), a derated planning model, a lower cap
const STRAT2 = {
  gas: {}, 'gas-reserve0.8-w20': { objective: { name: 'gas', reserve: 0.8, w_reserve: 20 } }, 'gas-reserve0.7-w20': { objective: { name: 'gas', reserve: 0.7, w_reserve: 20 } },
  'gas-reserve0.8-w100': { objective: { name: 'gas', reserve: 0.8, w_reserve: 100 } }, 'gas-derate0.85': { umax: { bottom: 0.85 }, exec: { gain: { bottom: 1 / 0.85 } } },
  'gas-hc0.9-derate0.85': { ...HC(0.9), umax: { bottom: 0.85 }, exec: { gain: { bottom: 1 / 0.85 } } }, 'gas-vcap0.5': { objective: { name: 'gas', v_cap: 0.5 } }, land: { objective: 'land' },
};
const MIS2 = { nominal: {}, 'eta0.9': { eta: { bottom: 0.9 } }, 'eta0.85': { eta: { bottom: 0.85 } } };
C['gas-robust2'] = {
  description: 'Robustness against a thrust shortfall: stronger braking reserve (w_reserve 20 / 100), a derated planning model (plan with umax.bottom 0.85, execute x1/0.85: full thrust stays available to replanning), cap 0.5 m/s, land; bottom thrust x1 / 0.9 / 0.85 unknown to the planner.',
  base, controllers: Object.entries(STRAT2).flatMap(([sn, sv]) => Object.entries(MIS2).map(([mn, mv]) => ({ label: `${sn}|${mn}`, ...sv, exec: { ...(sv.exec || {}), ...mv } }))), presets: P6, seeds: S3, record: 'none',
};

for (const [name, c] of Object.entries(C)) writeJSON(path.join(ROOT, 'experiments/configs', `${name}.json`), { name, task: 'escape-soft-landing', ...c }, 1);
console.log(Object.entries(C).map(([n, c]) => `${n}: ${c.controllers.length} controllers x ${c.presets.length} presets x ${c.seeds.length} seeds = ${c.controllers.length * c.presets.length * c.seeds.length}`).join('\n'));
