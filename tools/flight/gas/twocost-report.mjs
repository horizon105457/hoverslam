// Figures, tables and numbers of experiments/reports/gas-two-costs from a tools/flight/gas/twocost.mjs run.
// usage: node tools/flight/gas/twocost-report.mjs [run=experiments/runs/gas-two-costs/<stamp>] [hero=flat:1] [extra=nose:3,nose:5]
//   writes the hero pair (and the extra pairs) as traj/{first,second}-<preset>-s<seed>.json next to the report, for the
//   film (flight program: ?traj=/experiments/reports/gas-two-costs/traj/first-flat-s1.json)
//   hero: the (preset, seed) of the representative pair (default: the upright case with the clearest cost-1 preference
//   for the hover start among those where cost 3 keeps the all-closed start and both flights succeed)
import fs from 'fs';
import path from 'path';
import { parseArgs } from '../../lib/args.mjs';
import { ROOT, G, readJSON, series, mean, sd, wilson, fmt } from './lib.mjs';
import { MassModel } from '../../../src/flight/massprops.js';
import { figure } from './svg.mjs';

const a = parseArgs();
const base = path.join(ROOT, 'experiments', 'runs', 'gas-two-costs');
const run = path.resolve(a.run || path.join(base, fs.readdirSync(base).filter(d => /^\d{8}-\d{6}$/.test(d)).sort().pop()));
const OUT = path.join(ROOT, 'experiments', 'reports', 'gas-two-costs');
fs.mkdirSync(OUT, { recursive: true });
const cfg = readJSON(path.join(run, 'config.json')), stampRun = path.basename(run);
const rows = fs.readFileSync(path.join(run, 'episodes.jsonl'), 'utf8').split('\n').filter(Boolean).map(l => JSON.parse(l));
const UP = ['flat', 'nose', 'rear', 'side'], ALL = ['flat', 'nose', 'rear', 'side', 'roof', 'corner'];
const LAB = { 'c1-zero': '代价 1 · 全关起步', 'c1-hover': '代价 1 · 托住起步', 'c2-zero': '代价 2 · 全关起步', 'c2-hover': '代价 2 · 托住起步', 'c3-zero': '代价 3 · 全关起步', 'c3-hover': '代价 3 · 托住起步' };
const COST = { C1: '代价 1（无用气项）', C2: '代价 2（+0.3 / kg）', C3: '代价 3（+1 / kg）', C1s: '代价 1 + 下落速度项' };
const SEL = { C1: ['c1-zero', 'c1-hover'], C2: ['c2-zero', 'c2-hover'], C3: ['c3-zero', 'c3-hover'], C1s: ['c1-zero', 'c1-hover'] };
const NUM = { run: path.relative(ROOT, run), spec_hash: cfg.spec_hash, stamp: stampRun };
const pc = (k, n) => `${k}/${n}`, pct = x => `${Math.round(100 * x)} %`;
const save = (f, s) => fs.writeFileSync(path.join(OUT, f), s);

// ------------------------------------------------------------------------------------- per controller
const stats = rs => {
  const ok = rs.filter(r => r.metrics.success), M = (xs, f) => { const v = xs.map(f).filter(x => typeof x === 'number' && Number.isFinite(x)); return { mean: mean(v), sd: sd(v), n: v.length }; };
  return { n: rs.length, k: ok.length, ci: wilson(ok.length, rs.length), gas: M(rs, r => r.metrics.gas_used_kg), gas_ok: M(ok, r => r.metrics.gas_used_kg),
    vtd: M(ok, r => r.metrics.touchdown_vertical_speed), tilt: M(ok, r => r.metrics.touchdown_tilt_deg), t_td: M(ok, r => r.metrics.touchdown_t),
    ign: M(ok, r => r.burn.ign_before_td), pre: M(ok, r => r.burn.gas_pre_ign), to_td: M(ok, r => r.burn.gas_to_td), after: M(ok, r => r.profile?.gas_after_td),
    vmax: M(ok, r => r.profile?.v_sink_max), flight: M(ok, r => r.profile?.flight_s), fbw: M(ok, r => r.profile?.fb_over_w), hold: M(ok, r => r.profile?.hold_share),
    reasons: rs.reduce((o, r) => (r.metrics.success ? o : { ...o, [r.metrics.reason]: (o[r.metrics.reason] || 0) + 1 }), {}) };
};
const labels = cfg.controllers.map(c => c.label);
const of = (lab, ps) => rows.filter(r => r.label === lab && ps.includes(r.preset));
NUM.controllers = {};
const T = [];
for (const [ps, tag] of [[UP, '四个"正着飞出"的预设（flat / nose / rear / side）'], [ALL, '全部 6 个预设']]) {
  T.push(`**${tag}**，每格 ${ps.length * cfg.seeds_resolved.length} 回合（种子 ${cfg.seeds_resolved[0]}–${cfg.seeds_resolved.at(-1)}）；成功回合的均值：\n`);
  T.push('| 控制器 | 成功 | 用气（全部） | 用气（成功） | 触地速度 | 触地倾角 | 解锁→触地 | 最大下落速度 | 底部推力 / 重力（飞行中均值） | 托住时间占比 | 点火距触地 | 点火前用气 | 触地后用气 |');
  T.push('|---|---|---|---|---|---|---|---|---|---|---|---|---|');
  for (const l of labels) {
    const s = stats(of(l, ps));
    NUM.controllers[`${l}@${ps === UP ? 'up' : 'all'}`] = s;
    T.push(`| ${LAB[l]} | ${pc(s.k, s.n)} | ${fmt(s.gas.mean, 1)} kg | ${fmt(s.gas_ok.mean, 1)} ± ${fmt(s.gas_ok.sd, 1)} | ${fmt(s.vtd.mean)} m/s | ${fmt(s.tilt.mean, 1)}° | ${fmt(s.flight.mean)} s | ${fmt(s.vmax.mean, 1)} m/s | ${fmt(s.fbw.mean)} | ${pct(s.hold.mean ?? 0)} | ${fmt(s.ign.mean)} s | ${fmt(s.pre.mean, 1)} kg | ${fmt(s.after.mean, 1)} kg |`);
  }
  T.push('');
}
T.push('托住时间占比：解锁到触地之间底部推力 ≥ 车重一半的时间份额。点火距触地：触地前第一个底部推力 ≥ 60 % 满推力、且其后 3 步 ≥ 30 % 的控制步（`burnStats`）；"托住起步"的回合往往一路中等推力、从不达到这个门槛，没有点火时刻时不计入均值。\n');

// ------------------------------------------------------------------------------------- selection per cost
const pick = (key, ps) => {
  const by = new Map();
  for (const r of rows) if (SEL[key].includes(r.label) && ps.includes(r.preset)) { const id = `${r.preset}|${r.seed}`, b = by.get(id); if (!b || r.score[key].total < b.score[key].total) by.set(id, r); }
  return [...by.values()];
};
NUM.select = {};
T.push(`### 两起点 MPC：每个（预设、种子）用同一个代价给两条实际飞行打分，保留总代价低的那条\n`);
T.push('| 代价 | 预设 | 选中"托住" | 选中"全关" | 成功 | 用气（选中的飞行） | 其中"托住"的用气 | 其中"全关"的用气 |');
T.push('|---|---|---|---|---|---|---|---|');
for (const key of ['C1', 'C2', 'C3', 'C1s']) for (const [ps, tag] of [[UP, '4 个正着飞出'], [ALL, '全部 6 个']]) {
  const P = pick(key, ps), h = P.filter(r => r.label.endsWith('hover')), z = P.filter(r => r.label.endsWith('zero')), k = P.filter(r => r.metrics.success).length;
  const g = xs => mean(xs.map(r => r.metrics.gas_used_kg));
  NUM.select[`${key}@${ps === UP ? 'up' : 'all'}`] = { n: P.length, hover: h.length, zero: z.length, success: k, gas: g(P), gas_hover: g(h), gas_zero: g(z),
    margin: mean(P.map(r => { const o = rows.find(x => x.preset === r.preset && x.seed === r.seed && SEL[key].includes(x.label) && x.label !== r.label); return o ? o.score[key].total - r.score[key].total : null; })) };
  T.push(`| ${COST[key]} | ${tag} | ${h.length} | ${z.length} | ${pc(k, P.length)} | ${fmt(g(P), 1)} kg | ${fmt(g(h), 1)} | ${fmt(g(z), 1)} |`);
}
T.push('');
T.push('逐个（预设、种子）的选择（h = 托住起步，z = 全关起步，! = 该回合失败）：\n');
T.push(`| 代价 | ${ALL.join(' | ')} |`); T.push(`|---|${ALL.map(() => '---|').join('')}`);
for (const key of ['C1', 'C2', 'C3', 'C1s']) {
  const P = pick(key, ALL);
  T.push(`| ${COST[key]} | ${ALL.map(p => P.filter(r => r.preset === p).sort((x, y) => x.seed - y.seed).map(r => r.label.split('-')[1][0] + (r.metrics.success ? '' : '!')).join('')).join(' | ')} |`);
}
T.push('');

// ------------------------------------------------------------------------------------- cost decomposition (why the coin flips at 0.3 / kg)
{
  const pair = (key, l1, l2) => UP.flatMap(p => cfg.seeds_resolved.map(s => { const x = rows.find(r => r.label === l1 && r.preset === p && r.seed === s), y = rows.find(r => r.label === l2 && r.preset === p && r.seed === s); return x && y && x.metrics.success && y.metrics.success ? { x, y } : null; })).filter(Boolean);
  const P = pair('C1', 'c1-hover', 'c1-zero'), Q = pair('C2', 'c2-hover', 'c2-zero'), R = pair('C3', 'c3-hover', 'c3-zero');
  const d = (A, key, f) => mean(A.map(({ x, y }) => f(x.score[key]) - f(y.score[key])));
  NUM.decomp = {
    C1: { n: P.length, d_total: d(P, 'C1', s => s.total), d_gas: d(P, 'C1', s => s.gas_kg) },
    C2: { n: Q.length, d_total: d(Q, 'C2', s => s.total), d_rest: d(Q, 'C2', s => s.rest), d_gas: d(Q, 'C2', s => s.gas_kg) },
    C3: { n: R.length, d_total: d(R, 'C3', s => s.total), d_rest: d(R, 'C3', s => s.rest), d_gas: d(R, 'C3', s => s.gas_kg) },
  };
  T.push('### 托住 − 全关：同一代价下两条成功飞行的总代价差（4 个正着飞出的预设，配对均值；负数 = 托住更好）\n');
  T.push('| 代价 | 配对数 | 总代价差 | 其中"平稳"部分 | 其中用气部分 | 用气差 |');
  T.push('|---|---|---|---|---|---|');
  T.push(`| ${COST.C1} | ${P.length} | ${fmt(NUM.decomp.C1.d_total, 1)} | ${fmt(NUM.decomp.C1.d_total, 1)} | 0 | +${fmt(NUM.decomp.C1.d_gas, 1)} kg |`);
  T.push(`| ${COST.C2} | ${Q.length} | ${fmt(NUM.decomp.C2.d_total, 1)} | ${fmt(NUM.decomp.C2.d_rest, 1)} | +${fmt(0.3 * NUM.decomp.C2.d_gas, 1)} | +${fmt(NUM.decomp.C2.d_gas, 1)} kg |`);
  T.push(`| ${COST.C3} | ${R.length} | ${fmt(NUM.decomp.C3.d_total, 1)} | ${fmt(NUM.decomp.C3.d_rest, 1)} | +${fmt(NUM.decomp.C3.d_gas, 1)} | +${fmt(NUM.decomp.C3.d_gas, 1)} kg |`);
  T.push('');
}
save('tables.md', `# 表（由 \`node tools/flight/gas/twocost-report.mjs\` 从 \`${path.relative(ROOT, run)}\` 生成）\n\n${T.join('\n')}\n`);

// ------------------------------------------------------------------------------------- hero pair
const get = (lab, p, s) => rows.find(r => r.label === lab && r.preset === p && r.seed === s);
let hero = a.hero ? a.hero.split(':') : null;
if (hero) hero = [hero[0], +hero[1]];
else {
  let best = -Infinity;
  for (const p of ['flat', 'nose']) for (const s of cfg.seeds_resolved) {
    const h = get('c1-hover', p, s), z = get('c1-zero', p, s), z3 = get('c3-zero', p, s), h3 = get('c3-hover', p, s);
    if (!h || !z || !z3 || !h3 || !h.metrics.success || !z3.metrics.success) continue;
    if (h.score.C1.total >= z.score.C1.total || z3.score.C3.total >= h3.score.C3.total) continue;
    const m = z.score.C1.total - h.score.C1.total;
    if (m > best) { best = m; hero = [p, s]; }
  }
}
const [hp, hs] = hero, H1 = get('c1-hover', hp, hs), H2 = get('c3-zero', hp, hs);
const tr1 = readJSON(path.join(run, H1.traj)), tr2 = readJSON(path.join(run, H2.traj));
fs.mkdirSync(path.join(OUT, 'traj'), { recursive: true });
NUM.film = [];
for (const [p, s] of [[hp, hs], ...(a.extra ? a.extra.split(',').map(x => { const [q, n] = x.split(':'); return [q, +n]; }) : [])]) for (const [lab, kind] of [['c1-hover', 'first'], ['c3-zero', 'second']]) {
  const r = get(lab, p, s), f = `traj/${kind}-${p}-s${s}.json`;
  fs.copyFileSync(path.join(run, r.traj), path.join(OUT, f));
  NUM.film.push({ file: f, from: r.traj, kind, preset: p, seed: s, speed_kmh: readJSON(path.join(run, r.traj)).init.speed, gas: r.metrics.gas_used_kg, success: r.metrics.success, vtd: r.metrics.touchdown_vertical_speed, ign_before_td: r.burn.ign_before_td, gas_pre_ign: r.burn.gas_pre_ign });
}
NUM.hero = { preset: hp, seed: hs, first: { traj: path.relative(ROOT, path.join(run, H1.traj)), metrics: H1.metrics, burn: H1.burn, profile: H1.profile, score: H1.score },
  second: { traj: path.relative(ROOT, path.join(run, H2.traj)), metrics: H2.metrics, burn: H2.burn, profile: H2.profile, score: H2.score } };
{
  const S = [[tr1, H1, '第一种（代价无用气项）：托着往下放'], [tr2, H2, '第二种（代价 + 用气）：自杀式点火']].map(([t, r, name]) => ({ name: `${name}，${fmt(r.metrics.gas_used_kg, 1)} kg`, se: series(t), tA: r.burn.t_arm }));
  const tmax = Math.ceil(Math.max(...S.map(x => x.se.t.at(-1) - x.tA)) * 2) / 2;
  const P = (title, ylabel, key, y, scale = 1) => ({ title, xlabel: '解锁后时间 (s)', ylabel, x: [0, tmax], y,
    series: S.map((x, i) => ({ name: x.name, cls: i, legend: false, pts: x.se.t.map((t, k) => [t - x.tA, x.se[key][k] * scale]).filter(q => q[0] >= 0) })) });
  const W0 = new MassModel().at(tr1.tank[0][0]).M * G; // weight at full tank (N)
  const pa = P('(a) 底部 4 个喷口的推力合计', 'kN', 'Fb', [0, 36], 1e-3), pb = P('(b) 高度（车身原点距路面）', 'm', 'h', [0, 10]);
  const pcv = P('(c) 下落速度', 'm/s', 'vy', [-1, 9], -1), pd = P('(d) 累计用气', 'kg', 'gas', [0, 90]);
  pa.refs = [{ y: 1e-3 * W0, text: `车重（满罐 ${fmt(1e-3 * W0, 1)} kN）`, left: true }];
  for (const [x, cls, tdk] of [[S[0], 0, H1], [S[1], 1, H2]]) if (tdk.metrics.touchdown_t != null) (pd.refs ||= []).push({ x: tdk.metrics.touchdown_t - x.tA, text: `触地 ${fmt(tdk.burn.gas_to_td, 0)} kg`, left: cls === 1, bottom: cls === 0 });
  const pl = { title: '图例（各分图颜色一致）', legendBox: S.map((x, i) => ({ name: x.name, cls: i })), lines: [`回合：${hp} 预设、种子 ${hs}（孪生体，CEM-MPC，同一初始条件）`, '用气 = 整个回合（任务指标 gas_used_kg，满罐 145.7 kg）', `第二种点火距触地 ${fmt(H2.burn.ign_before_td)} s，点火前用气 ${fmt(H2.burn.gas_pre_ign, 1)} kg`] };
  save('fig-1-hero.svg', figure([pa, pb, pcv, pd, pl], { cols: 2, pw: 480, ph: 290, title: `图 1  同一次飞出（${hp} s${hs}），两种代价各自选中的飞行`, caption: `gas-two-costs ${stampRun}` }));
}

// ------------------------------------------------------------------------------------- gas per preset, strategies
{
  const show = ['c1-hover', 'c1-zero', 'c2-zero', 'c3-zero'];
  const xf = i => ALL[i] ?? '';
  const mk = (title, ylabel, y, f, err) => ({ title, xlabel: '发射预设', ylabel, x: [-0.5, ALL.length - 0.5], y, xticks: ALL.map((_, i) => i), xfmt: xf,
    series: show.map((l, j) => ({ name: LAB[l], cls: j, line: false, marker: true,
      pts: ALL.map((p, i) => { const s = stats(of(l, [p])); return [i - 0.24 + 0.16 * j, f(s), `${LAB[l]} @ ${p}: ${fmt(f(s), 2)} (成功 ${s.k}/${s.n})`]; }),
      err: err ? ALL.map((p, i) => { const s = stats(of(l, [p])); return [i - 0.24 + 0.16 * j, ...err(s)]; }).filter(e => e[1] != null) : undefined })) });
  const p1 = mk('(a) 用气（成功回合，均值 ± 标准差）', 'kg', [30, 100], s => s.gas_ok.mean, s => (s.gas_ok.n > 1 ? [s.gas_ok.mean - s.gas_ok.sd, s.gas_ok.mean + s.gas_ok.sd] : [null, null]));
  const p2 = mk('(b) 成功率', '成功率', [0, 1.05], s => s.k / s.n);
  p2.yfmt = v => `${Math.round(100 * v)}%`; p2.series.forEach(x => { x.legend = false; });
  save('fig-2-presets.svg', figure([p1, p2], { cols: 1, pw: 760, ph: 300, title: '图 2  按发射预设：用气与成功率（每点 10 回合）', caption: `gas-two-costs ${stampRun}` }));
}

save('numbers.json', JSON.stringify(NUM, null, 1));
console.log(`hero ${hp} s${hs}: first ${fmt(H1.metrics.gas_used_kg, 1)} kg (${H1.traj}), second ${fmt(H2.metrics.gas_used_kg, 1)} kg (${H2.traj})`);
console.log(`-> ${path.relative(ROOT, OUT)}`);
