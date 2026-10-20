// Open-loop convergence of the MPC optimizers on the first plan of an episode: the state at arming (zero thrust until
// the car has left the deck edge), the MPC's own schedule / planner / gas objective, one optimizer called again and
// again on the same problem (no receding horizon, no execution), recording after every call: rollouts used, cost,
// cost parts, and the planned trajectory's gas, touchdown speed / tilt and ignition time. Answers "where does each
// family converge, from which cold start, after how many rollouts" without the closed loop's warm-start effects.
// usage: node tools/flight/gas/openloop.mjs [budget=40000] [presets=flat,...] [seeds=1,2] [only=cem,...] [workers=8]
//        [out=experiments/runs/gas-openloop/<stamp>]
import fs from 'fs';
import path from 'path';
import { Worker, isMainThread, parentPort, workerData } from 'worker_threads';
import { fileURLToPath } from 'url';

const HERE = fileURLToPath(import.meta.url);
// optimizer families (as in configs.mjs gas-optim) and cold starts
const LATE = { type: 'xinit', schedule: { heave: [[0, 0], [1.0, 0], [1.05, 1], [1.45, 1], [1.5, 0]] } };
export const FAMILIES = {
  cem: [{ type: 'cem' }], mppi: [{ type: 'mppi' }], grad: [{ type: 'grad', iters: 2 }], xlbfgs: [{ type: 'xlbfgs', iters: 2 }],
  'cem>xlbfgs': [{ type: 'cem', iters: 2 }, { type: 'xlbfgs', iters: 1 }], 'xlbfgs>cem': [{ type: 'xlbfgs', iters: 1 }, { type: 'cem', iters: 1 }],
  'mppi>grad': [{ type: 'mppi', iters: 2 }, { type: 'grad', iters: 1 }],
};
export const INITS = { zero: {}, hover: { init: [0.42, 0, 0, 0, 0, 0, 0] }, early: { init: [0.7, 0, 0, 0, 0, 0, 0] }, late: { pre: LATE } };

if (!isMainThread) { // ------------------------------------------------------------------------------ worker
  const { createMPC } = await import('../../../src/flight/mpc/index.js');
  const { coastTo, FB_NOM, BOTTOM } = await import('./lib.mjs');
  const { WORLD } = await import('../../../src/core/spec.js');
  parentPort.on('message', job => {
    const env = coastTo('escape-soft-landing', job.seed, job.preset, tw => tw.armed), s = env.state;
    const fam = FAMILIES[job.family], ini = INITS[job.init];
    const ctl = createMPC({ objective: 'gas', optimizer: ini.pre ? [ini.pre, ...fam] : fam, ...(ini.init ? { init: ini.init } : {}) });
    ctl.reset({ task: env.task, seed: job.seed });
    const P = ctl.planner, opt = ctl.optimizer, th = ctl.theta;
    P.begin(s, s.t);
    const probe = () => { // the plan's own rollout: gas, touchdown, ignition (first step >= 60 % of the bottom nominal)
      let ign = null, gas0 = s.tank.m, td = null;
      const cost = P.rollout(th, (h, tw) => {
        let fb = 0; for (const i of BOTTOM) fb += tw.prop.thrust[i];
        if (ign === null && !td && fb >= 0.6 * FB_NOM) ign = (h + 1) * 0.02;
        if (tw.touchdown && !td) td = { t: tw.touchdown.t - s.t, vy: Math.abs(tw.touchdown.v[1]), tilt: Math.acos(Math.min(1, 1 - 2 * (tw.touchdown.q[0] ** 2 + tw.touchdown.q[2] ** 2))) * 180 / Math.PI };
      });
      P.evals--;
      const L = ctl.objective.lastLane;
      return { cost, gas: gas0 - L.m, parts: { ...L.parts }, td, ign, feasible: L.parts.vtd + L.parts.tilt + L.parts.rate + L.parts.hold + L.parts.crash + L.parts.air + L.parts.late < 1e-9 };
    };
    const curve = [], t0 = performance.now();
    let best = null;
    while (P.evals < job.budget) {
      const r = opt.solve(th, P);
      const pr = probe();
      if (!best || pr.cost < best.cost) best = { ...pr, evals: P.evals };
      curve.push([P.evals, pr.cost, pr.gas, pr.ign, pr.td?.t ?? null, pr.td?.vy ?? null, pr.feasible ? 1 : 0]);
      if (!Number.isFinite(r.cost) && !Number.isFinite(pr.cost)) break;
    }
    void WORLD;
    parentPort.postMessage({ ...job, t_arm: s.t, curve, cols: ['evals', 'cost', 'gas', 'ign_after_arm', 'td_after_arm', 'vtd', 'feasible'], final: probe(), best, wall_s: (performance.now() - t0) / 1000, theta: Array.from(th) });
  });
} else { // ---------------------------------------------------------------------------------------- main
  const { parseArgs } = await import('../../lib/args.mjs');
  const { ROOT, stamp } = await import('./lib.mjs');
  const { specHash } = await import('../../../src/flight/spechash.js');
  const a = parseArgs(), budget = +(a.budget || 40000);
  const presets = (a.presets || 'flat,nose,rear,side,roof,corner').split(','), seeds = (a.seeds || '1,2').split(',').map(Number);
  const fams = a.only ? a.only.split(',') : Object.keys(FAMILIES), inits = a.inits ? a.inits.split(',') : Object.keys(INITS);
  const out = path.resolve(a.out || path.join(ROOT, 'experiments/runs/gas-openloop', stamp()));
  fs.mkdirSync(out, { recursive: true });
  const hash = await specHash();
  const jobs = [];
  for (const family of fams) for (const init of inits) for (const preset of presets) for (const seed of seeds) jobs.push({ family, init, preset, seed, budget });
  fs.writeFileSync(path.join(out, 'config.json'), JSON.stringify({ name: 'gas-openloop', budget, presets, seeds, families: Object.fromEntries(fams.map(f => [f, FAMILIES[f]])), inits: Object.fromEntries(inits.map(i => [i, INITS[i]])), spec_hash: hash, objective: 'gas (defaults)', mpc: 'defaults (horizon 3, knot 0.1 linear, modes, plan_dt 2 ms)' }, null, 1));
  const fj = fs.openSync(path.join(out, 'runs.jsonl'), 'w'), nw = Math.min(+(a.workers || 8), jobs.length), t0 = performance.now();
  let n = 0;
  await new Promise((done, fail) => {
    let next = 0, live = nw;
    for (let w = 0; w < nw; w++) {
      const wk = new Worker(HERE, { workerData: { hash } });
      const feed = () => { if (next < jobs.length) wk.postMessage(jobs[next++]); else { wk.terminate(); if (--live === 0) done(); } };
      wk.on('message', r => { fs.writeSync(fj, JSON.stringify(r) + '\n'); n++; process.stderr.write(`\r${n}/${jobs.length} ${((performance.now() - t0) / 1000).toFixed(0)} s`); feed(); });
      wk.on('error', fail);
      feed();
    }
  });
  fs.closeSync(fj);
  console.log(`\n-> ${path.relative(ROOT, out)}`);
}
