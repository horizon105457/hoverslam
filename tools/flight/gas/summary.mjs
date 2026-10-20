// Aggregates of gas-study episodes (tools/flight/gas/bench.mjs rows): per label success with its Wilson 95 % interval,
// touchdown metrics, gas (all episodes and successes only), burn timing, rollout budget; plus a markdown table.
import fs from 'fs';
import { mean, sd, median, wilson, fmt } from './lib.mjs';

export const readRows = f => fs.readFileSync(f, 'utf8').split('\n').filter(Boolean).map(l => JSON.parse(l));

/** One label's rows -> aggregate. */
export function aggregate(rs) {
  const ok = rs.filter(r => r.metrics.success), n = rs.length, k = ok.length, M = (xs, f) => xs.map(f);
  const st = (xs, f) => { const v = M(xs, f).filter(x => typeof x === 'number' && Number.isFinite(x)); return { mean: mean(v), sd: sd(v), median: median(v), min: v.length ? Math.min(...v) : null, max: v.length ? Math.max(...v) : null, n: v.length }; };
  const per = {};
  for (const r of rs) (per[r.preset ?? 'task'] ||= []).push(r);
  const reasons = {};
  for (const r of rs) reasons[r.metrics.reason] = (reasons[r.metrics.reason] || 0) + 1;
  return {
    n, success: k / n, success_ci: wilson(k, n), hull_strike: rs.filter(r => r.metrics.hull_strike).length / n, reasons,
    gas: st(rs, r => r.metrics.gas_used_kg), gas_ok: st(ok, r => r.metrics.gas_used_kg),
    vtd: st(rs, r => r.metrics.touchdown_vertical_speed), tilt: st(rs, r => r.metrics.touchdown_tilt_deg), rate: st(rs, r => r.metrics.touchdown_angular_rate),
    peak_g: st(rs, r => r.metrics.peak_accel_g), t_td: st(rs, r => r.metrics.touchdown_t),
    ign_after_arm: st(ok, r => r.burn.ign_after_arm), ign_before_td: st(ok, r => r.burn.ign_before_td), h_ign: st(ok, r => r.burn.h_ign),
    burn_s: st(ok, r => r.burn.burn_s), gas_pre_ign: st(ok, r => r.burn.gas_pre_ign), centroid: st(ok, r => r.burn.centroid_before_td),
    evals: st(rs, r => r.evals), evals_per_plan: st(rs, r => (r.plans ? r.evals / r.plans : null)), wall_s: st(rs, r => r.wall_s),
    per_preset: Object.fromEntries(Object.keys(per).sort().map(p => [p, { n: per[p].length, success: per[p].filter(r => r.metrics.success).length / per[p].length, gas: mean(per[p].map(r => r.metrics.gas_used_kg)) }])),
  };
}

export function summarize(rows, labels) {
  return { controllers: Object.fromEntries(labels.filter(l => rows.some(r => r.label === l)).map(l => [l, aggregate(rows.filter(r => r.label === l))])) };
}

export function table(summary) {
  const C = summary.controllers, presets = [...new Set(Object.values(C).flatMap(c => Object.keys(c.per_preset)))].sort();
  const head = ['controller', 'n', 'success', 'hull', 'vy_td', 'tilt_td', 'gas kg', 'gas ok', 'ign-td s', 'h_ign m', 'pre-ign kg', 'evals/plan', ...presets, 's/ep'];
  const pc = x => `${(100 * x).toFixed(0)}%`;
  return [`| ${head.join(' | ')} |`, `|${'---|'.repeat(head.length)}`, ...Object.entries(C).map(([lab, s]) => `| ${[lab, s.n, pc(s.success), pc(s.hull_strike), fmt(s.vtd.mean), fmt(s.tilt.mean, 1), fmt(s.gas.mean, 1), fmt(s.gas_ok.mean, 1),
    fmt(s.ign_before_td.mean), fmt(s.h_ign.mean), fmt(s.gas_pre_ign.mean, 1), fmt(s.evals_per_plan.mean, 0), ...presets.map(p => (p in s.per_preset ? pc(s.per_preset[p].success) : '-')), fmt(s.wall_s.mean, 1)].join(' | ')} |`)].join('\n');
}
