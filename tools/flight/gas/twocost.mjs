// Two-cost experiment runner (experiments/reports/gas-two-costs): closed-loop MPC episodes on the twin, each one
// scored afterwards by every objective in `score` on its realized trajectory (the objective's step() on the real twin
// every control step from arming to the episode's end, then terminal()), so "which flight does cost k prefer" is
// answered on whole flights, not on a cold plan. Per cost, `select` picks per (preset, seed) the cheapest of its
// candidate episodes (e.g. the same MPC from an all-closed and from a hover cold start: the receding horizon stays
// in the basin of its first plan, so the cost alone cannot move it from one strategy to the other).
// Config (experiments/configs/gas-two-*.json):
//   { name, description, task, base: {<mpc settings>}, controllers: [{label, ...}], score: {key: objective},
//     select: {key: [label, ...]} | {name: {score: key, labels}}, presets, seeds, record: 'all'|'first'|'none' }
// Per episode (episodes.jsonl): metrics, lib.mjs burnStats, profile (peak sink speed, flight time, bottom thrust
// over the flight as a fraction of the weight, share of the flight with the bottom nozzles holding >= 1/2 the
// weight), score[key] = {total, gas_kg (arming -> end), rest = total - w_gas * gas_kg}.
// usage: node tools/flight/gas/twocost.mjs experiments/configs/gas-two-costs.json [workers=16] [seeds=1:6] [presets=]
//        [only=label,...] [out=experiments/runs/<name>/<stamp>]
//        node tools/flight/gas/twocost.mjs rescore=experiments/runs/<name>/<stamp> [config.json]   (other costs, same flights)
import fs from 'fs';
import os from 'os';
import path from 'path';
import { Worker, isMainThread, parentPort, workerData } from 'worker_threads';
import { fileURLToPath } from 'url';

const HERE = fileURLToPath(import.meta.url);

const { ColdGasEnv } = await import('../../../src/flight/env.js');
const { createObjective } = await import('../../../src/flight/mpc/objectives/index.js');
const { MassModel } = await import('../../../src/flight/massprops.js');
const { CONTROL_DT, GROUPS, G, NU } = await import('./lib.mjs');
const MM = new MassModel(), weight = m => MM.at(m).M * G;

/** Realized cost of a recorded flight under each objective of `score` ({key: objective config}): the trajectory's
 *  commands replayed on a fresh twin (deterministic: the same flight, checked against the recorded end state),
 *  objective step() every control step from arming on (H = control steps from arming to the task horizon, the
 *  crash weight's clock), terminal() at the end. -> {key: {total, gas_kg (arming -> end), rest = total - w_gas · gas_kg}} */
export function scoreTraj(traj, task, score) {
  const env = new ColdGasEnv(task, { seed: traj.seed, preset: traj.preset });
  env.reset(traj.seed, { init: traj.init });
  const sc = Object.entries(score).map(([key, o]) => { const obj = createObjective(o, { task: env.task }); obj.reset(env.task); return { key, obj, w: obj.config.w_gas ?? 0, L: null, h: 0, H: 0, total: 0, m0: 0 }; });
  const tw = env.twin, u = new Float64Array(env.na);
  for (let k = 0; k < traj.n - 1; k++) {
    const s = env.state;
    for (const x of sc) {
      x.obj.observe(s);
      if (!x.L && s.armed) { x.obj.begin?.(s); x.L = x.obj.start(tw, s); x.H = Math.round((env.task.horizon - s.t) / CONTROL_DT); x.m0 = tw.prop.m; }
    }
    u.fill(0); u.set(traj.u_cmd[k]); if (traj.drive_cmd) u.set(traj.drive_cmd[k], NU);
    env.step(u);
    for (const x of sc) if (x.L && !x.L.dead) { x.total += x.obj.step(x.L, tw, x.h, x.H, tw.u); x.h++; }
  }
  const end = traj.state[traj.n - 1], st = tw.state;
  if (Math.abs(st.p[1] - end[1]) > 1e-9 || Math.abs(st.v[1] - end[8]) > 1e-9) throw new Error(`scoreTraj: replay diverged (${traj.preset} s${traj.seed})`);
  const out = {};
  for (const x of sc) {
    if (x.L) x.total += x.obj.terminal(x.L, tw, x.H);
    const gas = x.m0 - tw.prop.m;
    out[x.key] = { total: x.total, gas_kg: gas, rest: x.total - x.w * gas };
  }
  return out;
}

if (!isMainThread) { // ------------------------------------------------------------------------------ worker
  const { runEpisode } = await import('../../../src/flight/run.js');
  const { createController } = await import('../../../src/flight/controllers/index.js');
  const { burnStats } = await import('./lib.mjs');
  parentPort.on('message', job => {
    const t0 = performance.now(), ctrl = createController(job.controller);
    const { traj, metrics } = runEpisode({ task: job.task, controller: ctrl, seed: job.seed, preset: job.preset, specHash: workerData.hash });
    traj.controller = job.controller;
    if (job.traj) fs.writeFileSync(job.traj, JSON.stringify(traj));
    parentPort.postMessage({ label: job.label, preset: traj.preset, seed: job.seed, metrics, burn: burnStats(traj), profile: profile(traj), score: scoreTraj(traj, job.task, job.score),
      start: ctrl.start ?? null, start_costs: ctrl.startCosts ?? null, plans: ctrl.plans ?? null, evals: ctrl.evals ?? null, wall_s: (performance.now() - t0) / 1000,
      traj: job.traj ? path.relative(job.out, job.traj) : null });
  });
} else { // ---------------------------------------------------------------------------------------- main
  const { parseArgs } = await import('../../lib/args.mjs');
  const { specHash } = await import('../../../src/flight/spechash.js');
  const { ROOT, stamp } = await import('./lib.mjs');
  const { summarize, table } = await import('./summary.mjs');
  const a = parseArgs(), cfgPath = Object.keys(a).find(k => a[k] === true && k.endsWith('.json'));
  if (a.rescore) { // score the recorded flights of a run again (score / select of the given config, else the run's own)
    const dir = path.resolve(a.rescore), cfg = JSON.parse(fs.readFileSync(cfgPath || path.join(dir, 'config.json'), 'utf8'));
    const rows = fs.readFileSync(path.join(dir, 'episodes.jsonl'), 'utf8').split('\n').filter(Boolean).map(l => JSON.parse(l));
    for (const r of rows) r.score = scoreTraj(JSON.parse(fs.readFileSync(path.join(dir, r.traj), 'utf8')), cfg.task, cfg.score);
    fs.writeFileSync(path.join(dir, 'episodes.rescored.jsonl'), rows.map(r => JSON.stringify(r)).join('\n') + '\n');
    fs.writeFileSync(path.join(dir, 'rescore.json'), JSON.stringify({ score: cfg.score, select: cfg.select, selected: select(rows, cfg.select || {}) }, null, 1));
    for (const [k, s] of Object.entries(select(rows, cfg.select || {}))) console.log(`select ${k}: ${JSON.stringify(s.picks)}  gas ${s.gas.toFixed(1)} kg, success ${s.success}/${s.n}`);
    process.exit(0);
  }
  if (!cfgPath) throw new Error('usage: node tools/flight/gas/twocost.mjs <config.json> [workers=] [seeds=] [presets=] [only=] [out=] | rescore=<run dir> [config.json]');
  const cfg = JSON.parse(fs.readFileSync(cfgPath, 'utf8')), name = cfg.name || path.basename(cfgPath, '.json');
  const out = path.resolve(a.out || path.join(ROOT, 'experiments', 'runs', name, stamp()));
  fs.mkdirSync(path.join(out, 'traj'), { recursive: true });
  const hash = await specHash();
  const seedSpec = a.seeds ? (a.seeds.includes(':') ? (([s, e]) => ({ start: s, count: e - s }))(a.seeds.split(':').map(Number)) : a.seeds.split(',').map(Number)) : cfg.seeds ?? [1];
  const seeds = Array.isArray(seedSpec) ? seedSpec : Array.from({ length: seedSpec.count }, (_, i) => (seedSpec.start ?? 0) + i);
  const presets = a.presets ? a.presets.split(',') : cfg.presets?.length ? cfg.presets : [null], record = cfg.record ?? 'all';
  const only = a.only ? new Set(a.only.split(',')) : null;
  const merge = (b, c) => { const r = { ...b }; for (const [k, v] of Object.entries(c)) r[k] = v && typeof v === 'object' && !Array.isArray(v) && b[k] && typeof b[k] === 'object' && !Array.isArray(b[k]) ? { ...b[k], ...v } : v; return r; };
  const ctrls = cfg.controllers.map(c => merge(cfg.base || {}, c)).filter(c => !only || only.has(c.label ?? c.name));
  fs.writeFileSync(path.join(out, 'config.json'), JSON.stringify({ ...cfg, resolved: ctrls, spec_hash: hash, seeds_resolved: seeds, presets_resolved: presets, stamp: path.basename(out), producer: 'js' }, null, 1));
  const jobs = [];
  for (const c of ctrls) {
    const { label: lab0, ...cc } = c, label = lab0 ?? c.name;
    presets.forEach(preset => seeds.forEach((seed, si) => {
      const keep = record === 'all' || (record === 'first' && si === 0);
      jobs.push({ task: cfg.task, controller: cc, label, preset, seed, out, score: cfg.score, traj: keep ? path.join(out, 'traj', `${label}__${preset ?? 'task'}__s${seed}.json`) : null });
    }));
  }
  const nw = Math.min(+(a.workers || cfg.workers || Math.max(1, os.cpus().length - 2)), jobs.length);
  const rows = [], fj = fs.openSync(path.join(out, 'episodes.jsonl'), 'w'), t0 = performance.now();
  await new Promise((done, fail) => {
    let next = 0, live = nw;
    for (let w = 0; w < nw; w++) {
      const wk = new Worker(HERE, { workerData: { hash } });
      const feed = () => { if (next < jobs.length) wk.postMessage(jobs[next++]); else { wk.terminate(); if (--live === 0) done(); } };
      wk.on('message', r => { rows.push(r); fs.writeSync(fj, JSON.stringify(r) + '\n'); process.stderr.write(`\r${rows.length}/${jobs.length} episodes ${((performance.now() - t0) / 1000).toFixed(0)} s`); feed(); });
      wk.on('error', fail);
      feed();
    }
  });
  fs.closeSync(fj);
  process.stderr.write('\n');
  const labels = ctrls.map(c => c.label ?? c.name);
  const selected = select(rows, cfg.select || {});
  const summary = { name, task: cfg.task, spec_hash: hash, stamp: path.basename(out), episodes: rows.length, wall_s: (performance.now() - t0) / 1000, labels, ...summarize(rows, labels), selected };
  fs.writeFileSync(path.join(out, 'summary.json'), JSON.stringify(summary, null, 1));
  console.log(table(summary));
  for (const [k, s] of Object.entries(selected)) console.log(`select ${k}: ${JSON.stringify(s.picks)}  gas ${s.gas.toFixed(1)} kg, success ${s.success}/${s.n}`);
  console.log(`\n-> ${path.relative(ROOT, out)}`);
}

/** Per selection: for every (preset, seed), the candidate episode with the lowest realized score of its cost.
 *  sel = {name: [label, ...]} (the cost is the score key `name`) or {name: {score: key, labels: [...], feasible}};
 *  feasible: true = the task is a hard constraint, only successful episodes compete (the cost ranks feasible plans;
 *  the land objective prices a landing past the deadline or a touchdown just over the task's speed cap only mildly),
 *  the cheapest failure only if no candidate succeeded. */
export function select(rows, sel) {
  const res = {};
  for (const [name, sv] of Object.entries(sel)) {
    const key = Array.isArray(sv) ? name : sv.score, labels = Array.isArray(sv) ? sv : sv.labels, fz = !Array.isArray(sv) && sv.feasible;
    const better = (r, b) => (fz && r.metrics.success !== b.metrics.success ? r.metrics.success : r.score[key].total < b.score[key].total);
    const by = new Map();
    for (const r of rows) if (labels.includes(r.label)) { const id = `${r.preset}|${r.seed}`, b = by.get(id); if (!b || better(r, b)) by.set(id, r); }
    const ps = [...by.values()], picks = {};
    for (const r of ps) picks[r.label] = (picks[r.label] || 0) + 1;
    res[name] = { score: key, n: ps.length, picks, success: ps.filter(r => r.metrics.success).length, gas: ps.reduce((s, r) => s + r.metrics.gas_used_kg, 0) / (ps.length || 1),
      episodes: ps.map(r => ({ preset: r.preset, seed: r.seed, label: r.label, score: r.score[key].total, gas: r.metrics.gas_used_kg, success: r.metrics.success })) };
  }
  return res;
}

/** Flight shape from a trajectory: peak sink speed (m/s), arming -> touchdown (s), mean bottom thrust over it / weight,
 *  share of it with bottom thrust >= half the weight, gas after the touchdown (kg). */
function profile(tr) {
  const ev = tr.events, tArm = ev.find(e => e.type === 'armed')?.t, td = ev.find(e => e.type === 'touchdown');
  if (tArm == null) return null;
  const dt = tr.dt, k0 = Math.round(tArm / dt), k1 = td ? Math.min(tr.n - 1, Math.floor(td.t / dt + 1e-9)) : tr.n - 1;
  let vmax = 0, Fs = 0, hold = 0;
  for (let k = k0; k < k1; k++) {
    const vy = -tr.state[k][8]; if (vy > vmax) vmax = vy;
    let F = 0; const th = tr.thrust[k + 1];
    for (const i of GROUPS.bottom) F += th[i];
    const W = weight(tr.tank[k][0]);
    Fs += F / W; if (F >= 0.5 * W) hold++;
  }
  const n = Math.max(1, k1 - k0);
  return { v_sink_max: vmax, flight_s: (k1 - k0) * dt, fb_over_w: Fs / n, hold_share: hold / n, gas_after_td: td ? tr.tank[k1][0] - tr.tank[tr.n - 1][0] : 0 };
}
