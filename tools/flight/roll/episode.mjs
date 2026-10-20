// One rigid-twin episode of a barrel-roll controller with the report metrics (metrics.mjs) and an optional timeline.
// usage: node tools/flight/roll/episode.mjs config=experiments/configs/roll-final.json [label=<controller label>]
//          [seed=1] [preset=flat] [out=<traj.json>] [timeline=0.1] [trace=1] [h_jump=<jump requirement, default 2.0 m>]
//          [nominal=1 [speed=<km/h>]]: the preset's own launch without the task's randomisation (= the soft body's)
//        node tools/flight/roll/episode.mjs controller='{"name":"mpc","objective":{"name":"roll"}}' ...
// The controller comes from a bench config (first controller, or the one with `label`) or from controller=<json>.
import fs from 'fs';
import path from 'path';
import { parseArgs } from '../../lib/args.mjs';
import { runEpisode } from '../../../src/flight/run.js';
import { createController } from '../../../src/flight/controllers/index.js';
import { specHash } from '../../../src/flight/spechash.js';
import { GROUPS } from '../../../src/flight/contract.js';
import { LAUNCH } from '../../../src/core/spec.js';
import { rollMetrics, fromTraj, rollSeries, attitude } from './metrics.mjs';

const a = parseArgs();
let cfg, task = a.task || 'escape-soft-landing';
if (a.config) {
  const C = JSON.parse(fs.readFileSync(a.config, 'utf8'));
  cfg = a.label ? C.controllers.find(x => (x.label ?? x.name) === a.label) : C.controllers[0];
  if (!cfg) throw new Error(`no controller ${a.label} in ${a.config}`);
  task = C.task || task;
} else cfg = JSON.parse(a.controller);
const { label, ...cc } = cfg;
if (a.trace) cc.trace = true;
const ctl = createController(cc);
const seed = +(a.seed ?? 1), t0 = performance.now();
const pre = a.preset ?? 'flat', P = LAUNCH.presets[pre];
const init = a.nominal ? { type: 'launch', speed: +(a.speed ?? P.speed), angle: P.angle, spin: P.spin, lead: P.lead ?? LAUNCH.defaults.lead, preset: pre, draws: {} } : null;
const { traj } = runEpisode({ task, controller: ctl, seed, preset: pre, init, specHash: await specHash() });
traj.controller = cc;
const wall = (performance.now() - t0) / 1000;
const src = fromTraj(traj), m = rollMetrics(src, a.h_jump != null ? { h_jump: +a.h_jump } : {});
if (a.out) { fs.mkdirSync(path.dirname(path.resolve(a.out)), { recursive: true }); fs.writeFileSync(a.out, JSON.stringify(traj)); }
if (a.timeline) {
  const every = Math.max(1, Math.round(+a.timeline / traj.dt)), { phi } = rollSeries(src);
  const grp = (k, g) => GROUPS[g].reduce((s, i) => s + traj.thrust[k][i], 0) / 1000;
  console.log('   t     y     vy     roll   tilt  pitch   wz    gas   bottom  rear  side (kN)');
  for (let k = 0; k < traj.n; k += every) {
    const s = traj.state[k], at = attitude(s.slice(3, 7));
    console.log([traj.t[k].toFixed(2), s[1].toFixed(2), s[8].toFixed(2), Number.isNaN(phi[k]) ? '   -' : (phi[k] * 57.2958).toFixed(0), at.tilt.toFixed(0), at.pitch.toFixed(1), s[12].toFixed(2),
      (traj.tank[0][0] - traj.tank[k][0]).toFixed(1), grp(k, 'bottom').toFixed(1), grp(k, 'rear').toFixed(1), grp(k, 'side').toFixed(1)].map(x => String(x).padStart(6)).join(' '));
  }
}
const tr = ctl.trace, med = xs => xs.slice().sort((p, q) => p - q)[xs.length >> 1];
const trace = tr?.length ? { first_ms: Math.round(tr[0].ms), first_evals: tr[0].evals, median_ms: Math.round(med(tr.slice(1).map(x => x.ms))), max_ms: Math.round(Math.max(...tr.slice(1).map(x => x.ms))), evals_per_plan: med(tr.slice(1).map(x => x.evals)) } : {};
console.log(JSON.stringify({ label: label ?? cc.name, seed, preset: traj.preset, wall_s: +wall.toFixed(1), plans: ctl.plans, plan_s: +(ctl.planMs / 1000).toFixed(2), evals: ctl.evals, ...trace, reason: traj.metrics.reason, ...m }));
