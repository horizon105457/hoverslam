// Gas-study benchmark runner: tools/flight/bench.mjs semantics (task, controllers, presets, seeds, record) plus, per
// episode, the burn analysis of lib.mjs burnStats (ignition time, burn length, gas before ignition, ...) and the
// rollout budget, and per controller an optional `exec` block (lib.mjs wrapExec: thrust scale / delay / held
// commands the planner does not know about). Config: experiments/configs/gas-*.json
//   { name, description, task, base: {<mpc settings shared by all controllers>}, controllers: [{label, ...}],
//     presets, seeds, record: 'all'|'first'|'none' }
// A controller entry is merged over `base` (objects such as objective / cem / grad / exec merged one level deep).
// usage: node tools/flight/gas/bench.mjs experiments/configs/gas-x.json [workers=8] [seeds=1:4|1,2] [presets=flat,nose]
//        [only=label,...] [out=experiments/runs/<name>/<stamp>]
import fs from 'fs';
import os from 'os';
import path from 'path';
import { Worker, isMainThread, parentPort, workerData } from 'worker_threads';
import { fileURLToPath } from 'url';

const HERE = fileURLToPath(import.meta.url);

if (!isMainThread) { // ------------------------------------------------------------------------------ worker
  const { runEpisode } = await import('../../../src/flight/run.js');
  const { createController } = await import('../../../src/flight/controllers/index.js');
  const { wrapExec, burnStats } = await import('./lib.mjs');
  parentPort.on('message', job => {
    const t0 = performance.now();
    const { exec, ...cc } = job.controller;
    const inner = createController(cc), ctrl = exec ? wrapExec(inner, exec) : inner;
    const r = runEpisode({ task: job.task, controller: ctrl, seed: job.seed, preset: job.preset, specHash: workerData.hash });
    const wall = (performance.now() - t0) / 1000;
    if (job.traj) { r.traj.controller = job.controller; fs.writeFileSync(job.traj, JSON.stringify(r.traj)); }
    const um = cc.umax, scale = typeof um === 'number' ? um : um?.bottom ?? 1; // ignition thresholds relative to the ceiling
    parentPort.postMessage({ label: job.label, preset: r.traj.preset, seed: job.seed, metrics: r.metrics, burn: burnStats(r.traj, { scale }), wall_s: wall,
      plans: inner.plans ?? null, plan_s: inner.planMs != null ? inner.planMs / 1000 : null, evals: inner.evals ?? null, start: inner.start ?? null, start_costs: inner.startCosts ?? null, traj: job.traj ? path.relative(job.out, job.traj) : null });
  });
} else { // ---------------------------------------------------------------------------------------- main
  const { parseArgs } = await import('../../lib/args.mjs');
  const { specHash } = await import('../../../src/flight/spechash.js');
  const { ROOT, stamp, summarize, table } = await import('./lib.mjs').then(async m => ({ ...m, ...(await import('./summary.mjs')) }));
  const a = parseArgs(), cfgPath = Object.keys(a).find(k => a[k] === true && k.endsWith('.json'));
  if (!cfgPath) throw new Error('usage: node tools/flight/gas/bench.mjs <config.json> [workers=] [seeds=] [presets=] [only=] [out=]');
  const cfg = JSON.parse(fs.readFileSync(cfgPath, 'utf8')), name = cfg.name || path.basename(cfgPath, '.json');
  const out = path.resolve(a.out || path.join(ROOT, 'experiments', 'runs', name, stamp()));
  fs.mkdirSync(path.join(out, 'traj'), { recursive: true });
  const hash = await specHash();
  const seedSpec = a.seeds ? (a.seeds.includes(':') ? (([s, e]) => ({ start: s, count: e - s }))(a.seeds.split(':').map(Number)) : a.seeds.split(',').map(Number)) : cfg.seeds ?? [1];
  const seeds = Array.isArray(seedSpec) ? seedSpec : Array.from({ length: seedSpec.count }, (_, i) => (seedSpec.start ?? 0) + i);
  const presets = a.presets ? a.presets.split(',') : cfg.presets?.length ? cfg.presets : [null], record = cfg.record ?? 'first';
  const only = a.only ? new Set(a.only.split(',')) : null;
  const merge = (b, c) => { const r = { ...b }; for (const [k, v] of Object.entries(c)) r[k] = v && typeof v === 'object' && !Array.isArray(v) && b[k] && typeof b[k] === 'object' && !Array.isArray(b[k]) ? { ...b[k], ...v } : v; return r; };
  const ctrls = cfg.controllers.map(c => merge(cfg.base || {}, c)).filter(c => !only || only.has(c.label ?? c.name));
  fs.writeFileSync(path.join(out, 'config.json'), JSON.stringify({ ...cfg, resolved: ctrls, spec_hash: hash, seeds_resolved: seeds, presets_resolved: presets, stamp: path.basename(out), producer: 'js' }, null, 1));
  const jobs = [];
  for (const c of ctrls) {
    const { label: lab0, ...cc } = c, label = lab0 ?? c.name;
    presets.forEach(preset => seeds.forEach((seed, si) => {
      const keep = record === 'all' || (record === 'first' && si === 0);
      jobs.push({ task: cfg.task, controller: cc, label, preset, seed, out, traj: keep ? path.join(out, 'traj', `${label}__${preset ?? 'task'}__s${seed}.json`) : null });
    }));
  }
  // longest first (bigger budgets) keeps the pool busy until the end
  const cost = j => (j.controller.cem?.samples ?? 1) * (j.controller.cem?.iters ?? 1) + (j.controller.mppi?.samples ?? 0) + 300 * (JSON.stringify(j.controller).match(/lbfgs|grad/g)?.length ?? 0);
  jobs.sort((p, q) => cost(q) - cost(p));
  const nw = Math.min(+(a.workers || cfg.workers || Math.max(1, os.cpus().length - 1)), jobs.length);
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
  const summary = { name, task: cfg.task, spec_hash: hash, stamp: path.basename(out), episodes: rows.length, wall_s: (performance.now() - t0) / 1000, labels, ...summarize(rows, labels) };
  fs.writeFileSync(path.join(out, 'summary.json'), JSON.stringify(summary, null, 1));
  console.log(table(summary));
  console.log(`\n-> ${path.relative(ROOT, out)}`);
}
