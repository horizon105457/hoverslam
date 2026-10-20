// JS side of the parity harness (mirror of python/coldgas/parity.py): scenario runner, comparison, report.
import fs from 'fs';
import zlib from 'zlib';
import path from 'path';
import { fileURLToPath } from 'url';
import { RigidTwin } from '../../src/flight/twin.js';
import { runEpisode } from '../../src/flight/run.js';
import { createController } from '../../src/flight/controllers/index.js';
import { TrajRecorder } from '../../src/flight/traj.js';
import { mulberry32 } from '../../src/flight/rng.js';
import { NU, CONTROL_DT } from '../../src/flight/contract.js';
import { specHash } from '../../src/flight/spechash.js';

export const PARITY = path.dirname(fileURLToPath(import.meta.url));
export const TOL = { free: 1e-9, contact: 1e-6 }; // spec/README.md section 5.9
export const scenarios = () => JSON.parse(fs.readFileSync(path.join(PARITY, 'scenarios.json'), 'utf8')).scenarios;
export const readGz = f => JSON.parse(zlib.gunzipSync(fs.readFileSync(f)).toString('utf8'));
export const writeGz = (f, o) => fs.writeFileSync(f, zlib.gzipSync(Buffer.from(JSON.stringify(o)), { level: 9 }));

/** Scenario controller: mulberry32(seed); per control step and nozzle a = r(); u = a < p_on ? r() : 0. */
export function randomValves({ seed = 1, p_on = 0.5 } = {}) {
  let r;
  return {
    name: 'random', config: { seed, p_on },
    reset() { r = mulberry32(seed); },
    act() { const u = new Float64Array(NU); for (let i = 0; i < NU; i++) { const a = r(); u[i] = a < p_on ? r() : 0; } return u; },
  };
}

export async function runScenario(sc, golden = null) {
  const c = sc.controller;
  const ctrl = c && c.name === 'random' ? (golden ? createController({ name: 'replay', traj: golden }) : randomValves(c)) : createController(c);
  const hash = await specHash();
  let traj;
  if (sc.task) {
    traj = runEpisode({ task: sc.task, controller: ctrl, seed: sc.seed ?? 0, preset: sc.preset ?? null, specHash: hash }).traj;
  } else {
    const tw = new RigidTwin({ arm: 'start' });
    tw.reset(sc.init);
    ctrl.reset({ task: null, seed: sc.seed ?? 0 });
    const rec = new TrajRecorder({ task: null, controller: c, seed: sc.seed ?? null, init: sc.init });
    for (let k = 0, n = Math.round(sc.duration / CONTROL_DT); k < n; k++) {
      rec.sample(tw);
      tw.step(ctrl.act(tw.state));
      rec.applied(tw.u);
    }
    rec.sample(tw);
    traj = rec.finish({ events: tw.events, metrics: {}, spec_hash: hash });
  }
  traj.scenario = sc;
  return traj;
}

export const GROUPS = { p: ['state', 0, 3], q: ['state', 3, 7], v: ['state', 7, 10], w: ['state', 10, 13], comB: ['comB', 0, 3], tank_m: ['tank', 0, 1], tank_T: ['tank', 1, 2], tank_p: ['tank', 2, 3], valve: ['valve', 0, NU], thrust: ['thrust', 0, NU], u_cmd: ['u_cmd', 0, NU] };

export function compare(golden, mine) {
  const n = Math.min(golden.n, mine.n), contact = (golden.contact || []).slice(0, n).map(x => x !== 0);
  const evs = t => JSON.stringify(t.events.map(e => [e.type, e.t]));
  const rep = { rows: [golden.n, mine.n], groups: {}, events_equal: evs(golden) === evs(mine) };
  for (const [g, [key, a, b]] of Object.entries(GROUPS)) {
    const out = { free: { abs: 0, rel: 0 }, contact: { abs: 0, rel: 0 } };
    for (let j = a; j < b; j++) {
      let scale = 1e-300;
      for (let k = 0; k < n; k++) scale = Math.max(scale, Math.abs(golden[key][k][j]));
      for (let k = 0; k < n; k++) {
        const d = Math.abs(golden[key][k][j] - mine[key][k][j]), o = out[contact[k] ? 'contact' : 'free'];
        o.abs = Math.max(o.abs, d); o.rel = Math.max(o.rel, d / scale);
      }
    }
    rep.groups[g] = out;
  }
  let md = 0;
  for (const [k, v] of Object.entries(golden.metrics || {})) {
    const w = mine.metrics?.[k];
    if (typeof v === 'number' && typeof w === 'number') md = Math.max(md, Math.abs(v - w) / Math.max(Math.abs(v), 1e-12));
    else if (Array.isArray(v) && Array.isArray(w)) v.forEach((x, i) => (md = Math.max(md, Math.abs(x - w[i]) / Math.max(Math.abs(x), 1e-12))));
    else if (v !== w) md = Infinity;
  }
  rep.metrics_rel = md;
  rep.ok = rep.rows[0] === rep.rows[1] && rep.events_equal && md <= TOL.contact && Object.values(rep.groups).every(v => v.free.rel <= TOL.free && v.contact.rel <= TOL.contact);
  return rep;
}

export function printReport(reports, title) {
  const e = x => x.toExponential(1).padStart(7);
  console.log(title);
  console.log('scenario'.padEnd(24), 'ok   rows  events ', Object.keys(GROUPS).map(g => g.padStart(15)).join('  '));
  const worst = Object.fromEntries(Object.keys(GROUPS).map(g => [g, { free: [0, 0], contact: [0, 0] }]));
  for (const [id, r] of Object.entries(reports)) {
    const cells = Object.keys(GROUPS).map(g => {
      for (const s of ['free', 'contact']) { worst[g][s][0] = Math.max(worst[g][s][0], r.groups[g][s].abs); worst[g][s][1] = Math.max(worst[g][s][1], r.groups[g][s].rel); }
      return `${e(r.groups[g].free.rel)}/${e(r.groups[g].contact.rel)}`;
    });
    console.log(id.padEnd(24), r.ok ? 'OK ' : 'BAD', String(r.rows[0]).padStart(5), ' ', String(r.events_equal).padEnd(6), cells.join('  '));
  }
  console.log('max abs / max rel per component (contact-free rows | contact rows):');
  for (const [g, w] of Object.entries(worst)) console.log(`  ${g.padEnd(7)} free abs ${w.free[0].toExponential(3)} rel ${w.free[1].toExponential(3)} | contact abs ${w.contact[0].toExponential(3)} rel ${w.contact[1].toExponential(3)}`);
  const ok = Object.values(reports).every(r => r.ok);
  console.log(ok ? 'ALL OK' : 'PARITY FAILURES');
  return ok;
}
