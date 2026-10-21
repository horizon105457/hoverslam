// Fit the twin's touchdown model to the soft-body landing experiments (grey-box identification): CMA-ES over
// params.mjs PARAMS on the FIT (+ ANCHOR) cases of a touchdown-doe run, loss = lib.mjs caseLoss summed over cases /
// total time; the population is evaluated on worker threads (each holds the cases). HOLDOUT cases are never seen
// here (tools/flight/calib/validate.mjs). Writes <out>/fit.json (best x, physical values, vehicle override, history).
// usage: node tools/flight/calib/fit.mjs run=experiments/runs/touchdown-doe/<stamp>[,<stamp2>...] [out=experiments/runs/calib/<tag>]
//        [evals=4000] [sigma=0.2] [seed=1] [workers=12] [splits=fit,anchor] [start=<fit.json>] [restarts=1]
//        [drive=<drive-doe run> drive_w=1 drive_hold=id,...] [set=<json of physical start values>]
// Several touchdown runs are pooled (same lattice physics; they may differ in what breaks, which only moves the window
// ends). drive=: the soft-body drive tests (tools/flight/drive) join the loss (driveLoss: speed, lateral speed, yaw
// rate, heading), weighted by drive_w per unit time like the touchdown cases; drive_hold cases are held out.
import fs from 'fs';
import path from 'path';
import { Worker, isMainThread, parentPort, workerData } from 'worker_threads';
import { fileURLToPath } from 'url';
import { RigidTwin } from '../../../src/flight/twin.js';
import { loadCases, replay, caseLoss } from './lib.mjs';
import { PARAMS, START, toPhys, toUnit, toVehicle } from './params.mjs';
import { loadDrive, replayDrive, driveLoss } from '../drive/compare.mjs';

/** Touchdown cases of one or more runs (comma separated; ids prefixed by the run's index from the second on). */
export function loadRuns(runs, opts) {
  return runs.flatMap((r, i) => loadCases(r, opts).map(c => (i ? { ...c, id: `${i}:${c.id}` } : c)));
}

/** Loss of one parameter vector over the cases (+ per-part sums); Infinity if the twin rejects / blows up. */
export function lossOf(cases, x, drive = null, driveW = 1) {
  let tw;
  try { tw = new RigidTwin({ vehicle: toVehicle(toPhys(x)) }); } catch { return { f: Infinity }; }
  let tot = 0, T = 0;
  const parts = {};
  for (const cs of cases) {
    const L = caseLoss(cs, replay(tw, cs));
    if (!Number.isFinite(L.total)) return { f: Infinity };
    tot += L.total; T += L.T;
    for (const k in L.parts) parts[k] = (parts[k] || 0) + L.parts[k];
  }
  let dT = 0;
  for (const cs of drive || []) {
    const L = driveLoss(cs, replayDrive(tw, cs));
    if (!Number.isFinite(L.total)) return { f: Infinity };
    tot += driveW * L.total; dT += L.T;
    for (const k in L.parts) parts[`d_${k}`] = (parts[`d_${k}`] || 0) + L.parts[k];
  }
  for (const k in parts) parts[k] /= k.startsWith('d_') ? dT : T;
  return { f: tot / (T + driveW * dT), parts };
}

if (!isMainThread) {
  const cases = loadRuns(workerData.runs, { splits: workerData.splits });
  const drive = workerData.drive ? loadDrive(workerData.drive).filter(c => !workerData.driveHold.includes(c.id)) : null;
  parentPort.on('message', ({ id, x }) => parentPort.postMessage({ id, ...lossOf(cases, x, drive, workerData.driveW) }));
} else {
  const { parseArgs } = await import('../../lib/args.mjs');
  const { cmaes } = await import('./cma.mjs');
  const a = parseArgs(), runs = a.run.split(',').map(r => path.resolve(r)), run = runs.join(','), splits = (a.splits || 'fit,anchor').split(',');
  const drive = a.drive ? path.resolve(a.drive) : null, driveW = +(a.drive_w ?? 1), driveHold = a.drive_hold ? a.drive_hold.split(',') : [];
  const out = path.resolve(a.out || `experiments/runs/calib/${new Date().toISOString().slice(0, 16).replace(/[-:T]/g, '')}`);
  fs.mkdirSync(out, { recursive: true });
  const nw = +(a.workers || 12), self = fileURLToPath(import.meta.url);
  const workers = Array.from({ length: nw }, () => new Worker(self, { workerData: { runs, splits, drive, driveW, driveHold } }));
  const pending = new Map();
  let nid = 0;
  for (const w of workers) w.on('message', m => { pending.get(m.id)(m); pending.delete(m.id); });
  const evalOne = (x, wi) => new Promise(res => { const id = nid++; pending.set(id, res); workers[wi % nw].postMessage({ id, x }); });
  const batch = async xs => (await Promise.all(xs.map((x, i) => evalOne(x, i)))).map(r => r.f);
  // start: a previous fit's physical values (parameters it did not have keep START), else START
  // start: START, overlaid by a previous fit's physical values (start=) and explicit values (set='{"slide":0.8}')
  const x0 = toUnit({ ...START, ...(a.start ? JSON.parse(fs.readFileSync(a.start, 'utf8')).phys : {}), ...(a.set ? JSON.parse(a.set) : {}) }).map(v => Math.min(0.999, Math.max(0.001, v)));
  const f0 = (await evalOne(x0, 0));
  console.log(`start loss ${f0.f.toFixed(3)}`, f0.parts && Object.fromEntries(Object.entries(f0.parts).map(([k, v]) => [k, +v.toFixed(3)])));
  const t0 = Date.now();
  let best = null;
  for (let r = 0; r < +(a.restarts || 1); r++) {
    const res = await cmaes(null, best ? best.x : x0, { sigma: +(a.sigma || 0.2) / (r + 1), seed: +(a.seed || 1) + r, maxEvals: +(a.evals || 4000), batch,
      log: g => { if (g.gen % 10 === 0) console.log(`r${r} gen ${g.gen} evals ${g.evals} best ${g.best.toFixed(4)} gen-best ${g.gen_best.toFixed(4)} sigma ${g.sigma.toFixed(4)} ${((Date.now() - t0) / 1000).toFixed(0)} s`); } });
    if (!best || res.f < best.f) best = { ...res, restart: r };
  }
  const fin = await evalOne(best.x, 0), phys = toPhys(best.x);
  const doc = { run, splits, drive, drive_w: driveW, drive_hold: driveHold, loss: fin.f, parts: fin.parts, x: best.x, phys, vehicle: toVehicle(phys), evals: best.evals, history: best.history, seconds: (Date.now() - t0) / 1000,
    params: PARAMS.map(([k, lo, hi, lg], i) => ({ k, lo, hi, log: !!lg, x: best.x[i], v: phys[k], at_bound: best.x[i] < 0.01 || best.x[i] > 0.99 })) };
  fs.writeFileSync(path.join(out, 'fit.json'), JSON.stringify(doc, null, 1));
  console.log(`best loss ${fin.f.toFixed(4)} (start ${f0.f.toFixed(4)}) after ${best.evals} evals, ${doc.seconds.toFixed(0)} s ->`, path.join(out, 'fit.json'));
  for (const p of doc.params) console.log(`  ${p.k.padEnd(12)} ${p.v.toPrecision(4).padStart(10)} ${p.at_bound ? ' (at bound)' : ''}`);
  console.log('  parts', Object.fromEntries(Object.entries(fin.parts).map(([k, v]) => [k, +v.toFixed(3)])));
  for (const w of workers) w.terminate();
}
