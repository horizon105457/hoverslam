// Barrel-roll metrics of a twin bench run (tools/flight/bench.mjs with record "all"): reads episodes.jsonl and every
// trajectory, computes the report metrics (metrics.mjs) and prints a markdown table per controller label; writes
// roll-episodes.jsonl and roll-summary.json next to the bench output.
// usage: node tools/flight/roll/summarize.mjs experiments/runs/roll-<name>-js/<stamp> [h_jump=2.0 | margin=0.5] [quiet]
//   jump requirement: JUMP_REQ (2.0 m), h_jump= (one value for every controller) or margin= (each controller's objective
//   h_jump − margin, for sweeps over the target)
import fs from 'fs';
import path from 'path';
import { parseArgs } from '../../lib/args.mjs';
import { rollMetrics, fromTraj, JUMP_REQ } from './metrics.mjs';

const a = parseArgs(), dir = Object.keys(a).find(k => a[k] === true && fs.existsSync(k) && fs.statSync(k).isDirectory());
if (!dir) throw new Error('usage: node tools/flight/roll/summarize.mjs <bench run dir> [h_jump=]');
const rows = fs.readFileSync(path.join(dir, 'episodes.jsonl'), 'utf8').trim().split('\n').map(l => JSON.parse(l));
const out = [];
for (const r of rows) {
  if (!r.traj) continue;
  const tr = JSON.parse(fs.readFileSync(path.join(dir, r.traj), 'utf8'));
  const ht = tr.controller?.objective?.h_jump ?? 2.5, hj = a.h_jump != null ? +a.h_jump : a.margin != null ? ht - +a.margin : JUMP_REQ;
  out.push({ label: r.label, preset: r.preset, seed: r.seed, jump_req: hj, plans: r.plans, plan_s: r.plan_s, evals: r.evals, wall_s: r.wall_s, ...rollMetrics(fromTraj(tr), { h_jump: hj }) });
}
out.sort((p, q) => (p.label < q.label ? -1 : p.label > q.label ? 1 : 0) || (p.preset < q.preset ? -1 : p.preset > q.preset ? 1 : 0) || p.seed - q.seed);
fs.writeFileSync(path.join(dir, 'roll-episodes.jsonl'), out.map(x => JSON.stringify(x)).join('\n') + '\n');

const mean = xs => { xs = xs.filter(x => typeof x === 'number' && Number.isFinite(x)); return xs.length ? xs.reduce((s, x) => s + x, 0) / xs.length : null; };
const max = xs => { xs = xs.filter(x => typeof x === 'number' && Number.isFinite(x)); return xs.length ? Math.max(...xs) : null; };
const KEYS = ['roll_td_deg', 'climb_m', 'apex_above_deck_m', 'gas_used_kg', 'td_vy', 'td_tilt_deg', 'td_rate', 't_touchdown'];
const summary = {};
for (const lab of [...new Set(out.map(x => x.label))]) {
  const rs = out.filter(x => x.label === lab), per = {};
  for (const r of rs) (per[r.preset ?? 'task'] ||= []).push(r.success);
  summary[lab] = {
    n: rs.length, success_rate: rs.filter(r => r.success).length / rs.length, landing_rate: rs.filter(r => r.landing_success).length / rs.length,
    roll_rate: rs.filter(r => r.roll_ok).length / rs.length, jump_rate: rs.filter(r => r.jump_ok).length / rs.length,
    hull_strike_rate: rs.filter(r => r.hull_strike).length / rs.length,
    mean: Object.fromEntries(KEYS.map(k => [k, mean(rs.map(r => (k === 'roll_td_deg' ? Math.abs(r[k]) : r[k])))])),
    gas_max: max(rs.map(r => r.gas_used_kg)), dropouts: rs.filter(r => r.t_regulator_dropout != null).length,
    ms_per_plan: 1000 * mean(rs.map(r => r.plan_s / r.plans)), s_per_episode: mean(rs.map(r => r.wall_s)),
    per_preset_success: Object.fromEntries(Object.keys(per).sort().map(p => [p, per[p].filter(Boolean).length / per[p].length])),
    failures: rs.filter(r => !r.success).map(r => `${r.preset}/s${r.seed}: ${[!r.landing_success && r.reason, !r.roll_ok && `roll ${r.roll_td_deg?.toFixed(0)}°`, !r.jump_ok && `climb ${r.climb_m?.toFixed(2)} m`].filter(Boolean).join(', ')}`),
  };
}
fs.writeFileSync(path.join(dir, 'roll-summary.json'), JSON.stringify(summary, null, 1));
const pc = x => `${Math.round(100 * x)}%`, f = (x, d = 2) => (x == null ? '-' : x.toFixed(d));
console.log('| controller | n | success | landing | roll 360° | jump | hull strike | abs roll_td ° | climb m | gas kg (max) | dropouts | vy_td | tilt_td ° | rate_td | ms/plan |');
console.log('|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|');
for (const [lab, s] of Object.entries(summary)) console.log(`| ${lab} | ${s.n} | ${pc(s.success_rate)} | ${pc(s.landing_rate)} | ${pc(s.roll_rate)} | ${pc(s.jump_rate)} | ${pc(s.hull_strike_rate)} | ${f(s.mean.roll_td_deg, 0)} | ${f(s.mean.climb_m)} | ${f(s.mean.gas_used_kg, 1)} (${f(s.gas_max, 1)}) | ${s.dropouts} | ${f(s.mean.td_vy)} | ${f(s.mean.td_tilt_deg)} | ${f(s.mean.td_rate)} | ${f(s.ms_per_plan, 0)} |`);
if (!a.quiet) for (const [lab, s] of Object.entries(summary)) if (s.failures.length) console.log(`${lab} failures: ${s.failures.join('; ')}`);
