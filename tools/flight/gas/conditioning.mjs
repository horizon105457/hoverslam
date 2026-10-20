// Why gradient shooting struggles here: sensitivity of the plan cost to each schedule element at a good plan.
// For (preset, seed) and a few plan times, run the CEM MPC closed loop up to that time, take its current plan θ, then
// per element e of θ (knot × channel): central difference g_e and second difference H_ee with step h·(box width),
// plus how many elements are exactly flat (a closed valve below the 2 % dead band, an element past the touchdown).
// Writes experiments/runs/gas-conditioning/<stamp>/conditioning.json: per plan time, per channel |g| and |H| by knot.
// usage: node tools/flight/gas/conditioning.mjs [presets=flat,roof] [seed=1] [times=0,0.4,0.8] [h=0.04]
import path from 'path';
import { parseArgs } from '../../lib/args.mjs';
import { ColdGasEnv } from '../../../src/flight/env.js';
import { createMPC } from '../../../src/flight/mpc/index.js';
import { ROOT, stamp, writeJSON, fmt } from './lib.mjs';

const a = parseArgs(), presets = (a.presets || 'flat,roof').split(','), seed = +(a.seed || 1), times = (a.times || '0,0.4,0.8').split(',').map(Number), hf = +(a.h || 0.04);
const out = path.resolve(a.out || path.join(ROOT, 'experiments/runs/gas-conditioning', stamp()));
const res = [];
for (const preset of presets) {
  const env = new ColdGasEnv('escape-soft-landing', { seed, preset });
  env.reset(seed);
  const ctl = createMPC({ objective: 'gas' });
  ctl.reset({ task: env.task, seed });
  let tArm = null, info = env.info();
  for (const tq of times) {
    while (true) { // closed loop until the next plan at or after arming + tq
      const s = env.state;
      if (s.armed && tArm === null) tArm = s.t;
      if (tArm !== null && s.t >= tArm + tq - 1e-9 && Math.round((s.t - tArm) / 0.02) % ctl.config.replan === 0) break;
      const u = ctl.act(s, info); const r = env.step(u); info = r[4];
    }
    const s = env.state;
    const u0 = Float64Array.from(ctl.act(s, info)); // a plan step: θ is now the fresh plan from exactly this state
    const P = ctl.planner, th = Float64Array.from(ctl.theta), par = ctl.param;
    P.begin(s, -Infinity);
    const J0 = P.cost(th), nv = par.nv, nk = par.nk, x = Float64Array.from(th);
    const G = [], H = [];
    let flat = 0;
    for (let e = 0; e < P.dim; e++) {
      const w = P.hi[e] - P.lo[e], h = hf * w, v = th[e];
      const ap = Math.min(P.hi[e], v + h), am = Math.max(P.lo[e], v - h);
      x[e] = ap; const Jp = P.cost(x); x[e] = am; const Jm = P.cost(x); x[e] = v;
      const g = (Jp - Jm) / (ap - am), hh = ap - v > 0 && v - am > 0 ? (Jp - 2 * J0 + Jm) / ((ap - v) * (v - am)) : null;
      if (Jp === J0 && Jm === J0) flat++;
      G.push(g); H.push(hh);
    }
    const per = par.names.map((nm, k) => ({ channel: nm, g: Array.from({ length: nk }, (_, j) => Math.abs(G[j * nv + k])), H: Array.from({ length: nk }, (_, j) => H[j * nv + k]) }));
    const posH = H.filter(v => v != null && v > 1e-9).map(Math.abs), absG = G.map(Math.abs).filter(v => v > 0);
    const r = { preset, seed, t_plan_after_arm: +(s.t - tArm).toFixed(3), J0, parts: ctl.objective.lastLane && { ...ctl.objective.lastLane.parts }, dim: P.dim, flat,
      g_max: Math.max(...absG), g_median: absG.sort((p, q) => p - q)[absG.length >> 1], H_max: Math.max(...posH), H_min: Math.min(...posH), H_pos: posH.length, H_neg: H.filter(v => v != null && v < -1e-9).length, per };
    res.push(r);
    console.log(`${preset} t+${r.t_plan_after_arm}: J ${fmt(J0, 1)} dim ${P.dim} flat ${flat} |g| max ${fmt(r.g_max, 1)} median ${fmt(r.g_median, 3)} | H>0: ${r.H_pos} (max/min ${fmt(r.H_max / r.H_min, 0)}) H<0: ${r.H_neg}`);
    for (const c of per) console.log(`   ${c.channel.padEnd(8)} |g| by knot: ${c.g.slice(0, 16).map(v => (v < 0.01 ? '.' : v.toFixed(v < 10 ? 1 : 0))).join(' ')}`);
    { const r2 = env.step(u0); info = r2[4]; } // the closed loop continues with the command of the probed plan
  }
}
writeJSON(path.join(out, 'conditioning.json'), { presets, seed, times, h: hf, res }, 0);
console.log(`-> ${path.relative(ROOT, out)}`);
