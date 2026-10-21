// Touchdown design of experiments: FIT and HOLDOUT maximin Latin hypercubes over the ten touchdown factors (genuine
// first tyre contacts only) + one-factor-at-a-time ANCHORS around the nominal touchdown, run as parallel drop.mjs
// child processes. Resumable: a case whose cases/<id>.json.gz exists is done.
// usage: node tools/flight/touchdown/doe.mjs [config=experiments/configs/touchdown-doe.json] [run=<run dir>] [workers=10]
//          [only=F00,A00-nominal] [limit=n] [dry=1]      (new run dir experiments/runs/touchdown-doe/<timestamp>)
//        node tools/flight/touchdown/doe.mjs run=<run dir> report=1     (outcome table + per-factor comparison)
// Run dir: manifest.json (design, split labels, settings), cases/<id>.json.gz (drop.mjs, format touchdown-drop@1),
// summary.jsonl (one outcome line per finished case, manifest order, rebuilt at the end), logs/<id>.log, runs.jsonl.
import fs from 'fs';
import path from 'path';
import zlib from 'zlib';
import { spawn } from 'child_process';
import { fileURLToPath } from 'url';
import { parseArgs } from '../../lib/args.mjs';
import { repo } from '../../lib/paths.mjs';
import { MassModel } from '../../../src/flight/massprops.js';
import { Propulsion } from '../../../src/flight/propulsion.js';
import { FACTORS, NOMINAL, SITE, mulberry32, feasible, tyrePoints, specHash } from './lib.mjs';

const args = parseArgs();
const DROP = path.join(path.dirname(fileURLToPath(import.meta.url)), 'drop.mjs');

// ------------------------------------------------------------------------------------------------ design
const pts = tyrePoints(), mm = new MassModel(), m0 = new Propulsion().m0;
function factorsOf(cfg) { return FACTORS.map(F => ({ ...F, lo: cfg.factors?.[F.key]?.[0] ?? F.lo, hi: cfg.factors?.[F.key]?.[1] ?? F.hi })); }
const feasOf = cfg => f => feasible(f, mm.at(f.gas * m0).com, pts, cfg.feasibility || {});

/** Maximin LHS: jittered strata, column swaps by simulated annealing on (infeasible count, phi_p) lexicographically. */
function maximinLHS(n, seed, factors, feas, { iters = 60000, p = 30, t0 = 0.02, t1 = 2e-5 } = {}) {
  const r = mulberry32(seed), d = factors.length;
  const U = Array.from({ length: n }, () => new Float64Array(d));
  for (let a = 0; a < d; a++) {
    const perm = Array.from({ length: n }, (_, i) => i);
    for (let i = n - 1; i > 0; i--) { const j = Math.floor(r() * (i + 1)); [perm[i], perm[j]] = [perm[j], perm[i]]; }
    for (let i = 0; i < n; i++) U[i][a] = (perm[i] + r()) / n;
  }
  const toF = u => Object.fromEntries(factors.map((F, a) => [F.key, F.lo + u[a] * (F.hi - F.lo)]));
  const bad = U.map(u => (feas(toF(u)).ok ? 0 : 1));
  const dist = (i, j) => { let s = 0; for (let a = 0; a < d; a++) { const q = U[i][a] - U[j][a]; s += q * q; } return Math.sqrt(s); };
  const D = Array.from({ length: n }, (_, i) => Float64Array.from({ length: n }, (_, j) => (i === j ? 0 : dist(i, j))));
  let S = 0; for (let i = 0; i < n; i++) for (let j = i + 1; j < n; j++) S += D[i][j] ** -p;
  let nbad = bad.reduce((s, b) => s + b, 0);
  const rowSum = i => { let s = 0; for (let j = 0; j < n; j++) if (j !== i) s += D[i][j] ** -p; return s; };
  let best = { U: U.map(u => Float64Array.from(u)), nbad, S };
  for (let it = 0; it < iters; it++) {
    const T = t0 * (t1 / t0) ** (it / iters), a = Math.floor(r() * d), i = Math.floor(r() * n);
    let j = Math.floor(r() * (n - 1)); if (j >= i) j++;
    const old = S - rowSum(i) - rowSum(j) + D[i][j] ** -p; // pair (i, j) is unchanged by the swap
    [U[i][a], U[j][a]] = [U[j][a], U[i][a]];
    const Di = Float64Array.from(D[i]), Dj = Float64Array.from(D[j]);
    for (let k = 0; k < n; k++) if (k !== i && k !== j) { D[i][k] = D[k][i] = dist(i, k); D[j][k] = D[k][j] = dist(j, k); }
    const S2 = old + rowSum(i) + rowSum(j) - D[i][j] ** -p;
    const bi = feas(toF(U[i])).ok ? 0 : 1, bj = feas(toF(U[j])).ok ? 0 : 1, nb2 = nbad - bad[i] - bad[j] + bi + bj;
    const dl = Math.log(S2) - Math.log(S); // phi_p = S^(1/p): compare logs
    if (nb2 < nbad || (nb2 === nbad && (dl < 0 || r() < Math.exp(-dl / p / T)))) {
      S = S2; nbad = nb2; bad[i] = bi; bad[j] = bj;
      if (nbad < best.nbad || (nbad === best.nbad && S < best.S)) best = { U: U.map(u => Float64Array.from(u)), nbad, S };
    } else { // revert
      [U[i][a], U[j][a]] = [U[j][a], U[i][a]];
      for (let k = 0; k < n; k++) { D[i][k] = D[k][i] = Di[k]; D[j][k] = D[k][j] = Dj[k]; }
      D[i][i] = D[j][j] = 0;
    }
  }
  let minD = Infinity; for (let i = 0; i < n; i++) for (let j = i + 1; j < n; j++) { let s = 0; for (let a = 0; a < d; a++) s += (best.U[i][a] - best.U[j][a]) ** 2; minD = Math.min(minD, Math.sqrt(s)); }
  const round = (k, v) => Math.round(v * (k === 'vs' || k === 'gas' || k === 'spin' || k.endsWith('rate') ? 1e4 : 1e3)) / (k === 'vs' || k === 'gas' || k === 'spin' || k.endsWith('rate') ? 1e4 : 1e3);
  return { points: best.U.map(u => Object.fromEntries(Object.entries(toF(u)).map(([k, v]) => [k, round(k, v)]))), stats: { n, seed, infeasible: best.nbad, phi_p: +(best.S ** (1 / p)).toFixed(4), min_dist_unit: +minD.toFixed(4), iters, p } };
}

function buildDesign(cfg) {
  const factors = factorsOf(cfg), feas = feasOf(cfg), nominal = { ...NOMINAL, ...cfg.nominal };
  const fit = maximinLHS(cfg.fit.n, cfg.fit.seed, factors, feas, cfg.maximin);
  const hold = maximinLHS(cfg.holdout.n, cfg.holdout.seed, factors, feas, cfg.maximin);
  const pad = (k, n) => String(k).padStart(String(n - 1).length, '0');
  const cases = [
    ...fit.points.map((f, k) => ({ id: `F${pad(k, fit.points.length)}`, split: 'fit', factors: f })),
    ...hold.points.map((f, k) => ({ id: `H${pad(k, hold.points.length)}`, split: 'holdout', factors: f })),
    ...cfg.anchors.map((a, k) => { const { id, ...o } = a; return { id: `A${pad(k, cfg.anchors.length)}-${id}`, split: 'anchor', factors: { ...nominal, ...o } }; }),
  ];
  for (const c of cases) { const r = feas(c.factors); c.feasibility = { ok: r.ok, approach: +r.approach.toFixed(3), wheel: r.wheel }; }
  return { factors, nominal, cases, stats: { fit: fit.stats, holdout: hold.stats } };
}

// ------------------------------------------------------------------------------------------------ helpers
const readCase = f => JSON.parse(zlib.gunzipSync(fs.readFileSync(f)));
function rebuildSummary(dir, manifest) {
  const lines = [];
  for (const c of manifest.cases) { const f = path.join(dir, 'cases', `${c.id}.json.gz`); if (fs.existsSync(f)) lines.push(JSON.stringify({ split: c.split, ...readCase(f).summary })); }
  fs.writeFileSync(path.join(dir, 'summary.jsonl'), lines.join('\n') + (lines.length ? '\n' : ''));
  return lines.map(l => JSON.parse(l));
}

function report(rows, factors) {
  const outcomes = ['clean', 'hull_strike', 'mount_damage', 'spring_snap', 'arm_break', 'wheel_detach', 'rollover', 'no_touchdown'];
  const splits = ['fit', 'holdout', 'anchor'];
  console.log(`outcome (worst flag)    ${splits.map(s => s.padStart(8)).join('')}   all`);
  for (const o of outcomes) { const c = splits.map(s => rows.filter(r => r.split === s && r.outcome === o).length); if (c.some(Boolean)) console.log(`${o.padEnd(22)} ${c.map(v => String(v).padStart(8)).join('')} ${String(c.reduce((a, b) => a + b, 0)).padStart(5)}`); }
  const fl = k => rows.filter(r => r.flags.includes(k)).length;
  console.log(`flags (any): hull_strike ${fl('hull_strike')}, mount_damage ${fl('mount_damage')}, wheel_detach ${fl('wheel_detach')}, arm_break ${fl('arm_break')}, spring_snap ${fl('spring_snap')}, rollover ${fl('rollover')}, hull_first ${fl('hull_first')}, speed_clamp ${fl('speed_clamp')} of ${rows.length}`);
  const ok = rows.filter(r => r.achieved);
  const errs = factors.map(F => { const e = ok.map(r => Math.abs(r.achieved[F.key] - r.target[F.key])).filter(Number.isFinite); return `${F.key} ${e.length ? Math.max(...e).toPrecision(2) : '-'}`; });
  console.log(`max |achieved - target| at touchdown: ${errs.join(', ')}`);
  // per factor: failure rate by tercile and mean factor value per outcome class (LHS splits only)
  const lhs = rows.filter(r => r.split !== 'anchor');
  const fail = r => r.outcome !== 'clean', mag = new Set(['beta', 'phi', 'yaw_rate', 'roll_rate']);
  console.log(`\nper factor (fit + holdout, n=${lhs.length}): failure share by tercile of the factor (|x| for the signed lateral ones), mean factor by class`);
  for (const F of factors) {
    const val = r => (mag.has(F.key) ? Math.abs(r.target[F.key]) : r.target[F.key]);
    const sorted = [...lhs].sort((a, b) => val(a) - val(b)), n3 = sorted.length / 3;
    const terc = [0, 1, 2].map(k => sorted.slice(Math.round(k * n3), Math.round((k + 1) * n3)));
    const share = (g, p) => (g.length ? `${g.filter(p).length}/${g.length}` : '-');
    const mean = g => (g.length ? (g.reduce((s, r) => s + val(r), 0) / g.length).toFixed(2) : '-');
    const cls = k => lhs.filter(r => r.outcome === k);
    console.log(`${(mag.has(F.key) ? `|${F.key}|` : F.key).padEnd(13)} fail ${terc.map(g => share(g, fail).padStart(6)).join(' ')}  hull ${terc.map(g => share(g, r => r.flags.includes('hull_strike')).padStart(6)).join(' ')}  susp ${terc.map(g => share(g, r => ['wheel_detach', 'arm_break', 'spring_snap', 'mount_damage'].some(k => r.flags.includes(k))).padStart(6)).join(' ')}  | mean clean ${mean(cls('clean'))} hull ${mean(lhs.filter(r => r.flags.includes('hull_strike')))} detach ${mean(lhs.filter(r => r.flags.includes('wheel_detach')))}`);
  }
  const ws = rows.map(r => r.compute_s / r.sim_s).filter(Number.isFinite);
  if (ws.length) console.log(`\ncompute ${(rows.reduce((s, r) => s + r.compute_s, 0) / rows.length).toFixed(0)} s per case (mean), ${(ws.reduce((a, b) => a + b, 0) / ws.length).toFixed(1)} s per simulated second; min box distance ${Math.min(...rows.map(r => r.min_box_dist)).toFixed(1)} m`);
}

// ------------------------------------------------------------------------------------------------ main
const cfgFile = args.config || 'experiments/configs/touchdown-doe.json';
let dir = args.run ? repo(args.run) : null, manifest;
if (dir && fs.existsSync(path.join(dir, 'manifest.json'))) manifest = JSON.parse(fs.readFileSync(path.join(dir, 'manifest.json'), 'utf8'));
else {
  const cfg = JSON.parse(fs.readFileSync(repo(cfgFile), 'utf8'));
  const t = Date.now(), D = buildDesign(cfg);
  const d = new Date(), p2 = v => String(v).padStart(2, '0'); // local time stamp YYYYMMDD-HHMMSS
  dir ||= repo('experiments/runs/touchdown-doe', `${d.getFullYear()}${p2(d.getMonth() + 1)}${p2(d.getDate())}-${p2(d.getHours())}${p2(d.getMinutes())}${p2(d.getSeconds())}`);
  manifest = { format: 'touchdown-doe@1', created: new Date().toISOString(), config_file: cfgFile, config: cfg, spec_hash: specHash(), site: SITE, factors: D.factors.map(({ key, lo, hi, unit, doc }) => ({ key, lo, hi, unit, doc })), nominal: D.nominal, design: D.stats, cases: D.cases };
  if (!args.dry) { fs.mkdirSync(path.join(dir, 'cases'), { recursive: true }); fs.writeFileSync(path.join(dir, 'manifest.json'), JSON.stringify(manifest, null, 1)); }
  console.log(`design in ${((Date.now() - t) / 1000).toFixed(1)} s: fit ${JSON.stringify(D.stats.fit)} holdout ${JSON.stringify(D.stats.holdout)}; infeasible anchors ${D.cases.filter(c => !c.feasibility.ok).map(c => c.id).join(' ') || 'none'}`);
}
console.log(`run dir ${path.relative(repo(), dir)} · ${manifest.cases.length} cases`);
if (args.dry) { for (const c of manifest.cases) console.log(c.id, c.split, JSON.stringify(c.factors), c.feasibility.ok ? '' : 'INFEASIBLE'); process.exit(0); }
if (args.report) { report(rebuildSummary(dir, manifest), manifest.factors); process.exit(0); }

const only = args.only ? new Set(String(args.only).split(',')) : null;
const done = c => fs.existsSync(path.join(dir, 'cases', `${c.id}.json.gz`));
let todo = manifest.cases.filter(c => (!only || only.has(c.id)) && !done(c));
if (args.limit) todo = todo.slice(0, +args.limit);
const workers = Math.max(1, +(args.workers || 10)), drop = manifest.config.drop || {};
fs.mkdirSync(path.join(dir, 'logs'), { recursive: true });
const T0 = Date.now(), children = new Set();
console.log(`${manifest.cases.filter(done).length} done, running ${todo.length} with ${workers} workers`);
const stop = () => { for (const ch of children) ch.kill('SIGTERM'); process.exit(130); }; // only our own children
process.on('SIGINT', stop); process.on('SIGTERM', stop);
let k = 0, finished = 0;
const failed = [];
async function worker() {
  while (k < todo.length) {
    const c = todo[k++], t = Date.now();
    const argv = [DROP, `id=${c.id}`, `outDir=${path.join(dir, 'cases')}`, 'quiet=1', ...Object.entries(c.factors).map(([a, v]) => `${a}=${v}`), ...Object.entries(drop).map(([a, v]) => `${a}=${v}`)];
    const log = fs.openSync(path.join(dir, 'logs', `${c.id}.log`), 'w');
    const code = await new Promise(res => { const ch = spawn(process.execPath, argv, { cwd: repo(), stdio: ['ignore', log, log] }); children.add(ch); ch.on('exit', cd => { children.delete(ch); res(cd); }); });
    fs.closeSync(log);
    finished++;
    if (code === 0 && done(c)) {
      const s = readCase(path.join(dir, 'cases', `${c.id}.json.gz`)).summary;
      fs.appendFileSync(path.join(dir, 'summary.jsonl'), JSON.stringify({ split: c.split, ...s }) + '\n');
      console.log(`[${finished}/${todo.length}] ${c.id} ${s.outcome}${s.flags.length > 1 ? ` [${s.flags.join(',')}]` : ''} hull ${s.hull_strike ? `${s.hull_strike.region}+${s.hull_strike.after_td}` : '-'} detach ${s.detached.join(',') || '-'} · ${((Date.now() - t) / 1000).toFixed(0)} s · elapsed ${((Date.now() - T0) / 60000).toFixed(1)} min`);
    } else { failed.push(c.id); console.log(`[${finished}/${todo.length}] ${c.id} FAILED (exit ${code}), see logs/${c.id}.log`); }
  }
}
await Promise.all(Array.from({ length: Math.min(workers, todo.length) }, worker));
const rows = rebuildSummary(dir, manifest);
fs.appendFileSync(path.join(dir, 'runs.jsonl'), JSON.stringify({ started: new Date(T0).toISOString(), wall_s: Math.round((Date.now() - T0) / 1000), workers, ran: todo.length, failed }) + '\n');
console.log(`\n${todo.length} cases in ${((Date.now() - T0) / 60000).toFixed(1)} min wall (${failed.length} failed); ${rows.length}/${manifest.cases.length} done\n`);
report(rows, manifest.factors);
