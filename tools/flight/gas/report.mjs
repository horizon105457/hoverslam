// Figures (SVG) and tables (markdown) of the gas-study report from the runs under experiments/runs/gas-*:
//   experiments/reports/gas-late-burn/fig-*.svg, tables.md (every number of the README, with its run stamp), numbers.json
// Picks the newest stamp of each experiment unless given: node tools/flight/gas/report.mjs [gas-coast=<stamp> ...]
import fs from 'fs';
import path from 'path';
import { parseArgs } from '../../lib/args.mjs';
import { ROOT, readJSON, mean, sd, median, fmt, wilson, burnStats, series, impulseStats } from './lib.mjs';
import { readRows, aggregate } from './summary.mjs';
import { figure } from './svg.mjs';

const a = parseArgs(), OUT = path.join(ROOT, 'experiments/reports/gas-late-burn');
fs.mkdirSync(OUT, { recursive: true });
const RUNS = path.join(ROOT, 'experiments/runs');
const latest = name => { const d = path.join(RUNS, name); if (!fs.existsSync(d)) return null; const s = a[name] || fs.readdirSync(d).filter(x => /^\d{8}-\d{6}$/.test(x)).sort().pop(); return s ? path.join(d, s) : null; };
const md = [], NUM = {};
const save = (f, svg) => { fs.writeFileSync(path.join(OUT, f), svg); md.push(`![${f}](${f})\n`); };
const pc = x => (x == null ? '–' : `${(100 * x).toFixed(0)}%`);
const runOf = name => { const d = latest(name); return d ? { dir: d, stamp: path.basename(d), rows: readRows(path.join(d, 'episodes.jsonl')), cfg: readJSON(path.join(d, 'config.json')) } : null; };
const group = (rows, key = r => r.label) => { const m = new Map(); for (const r of rows) { const k = key(r); if (!m.has(k)) m.set(k, []); m.get(k).push(r); } return m; };
const PRESETS = ['flat', 'nose', 'rear', 'side', 'roof', 'corner'];

// ------------------------------------------------------------------------------------- F1 vertical theory
function theory() {
  const d = latest('gas-reference');
  if (!d) return;
  const R = readJSON(path.join(d, 'reference.json')), C = R.cases, c0 = C.find(c => c.preset === 'flat' && c.seed === 1);
  // (a) touchdown speed vs ignition time (s after the start), flat s1, 2 ms sweep
  const sw = c0.sweep.map(r => ({ t: r[0], vtd: r[1], ok: r[4], hull: r[6], stop: r[7] }));
  const okR = sw.filter(r => r.ok).map(r => r.t), w0 = Math.min(...okR), w1 = Math.max(...okR);
  const pa = { title: '(a) 满推力 bang-bang：落地速度 vs 点火时刻（flat s1）', xlabel: '点火时刻：起点后 (s)', ylabel: '落地竖直速度 |v_y| (m/s)', x: [0.4, 1.05], y: [0, 14],
    shade: [[0.4, w0 - 0.001, '制动过头：停在空中（不触地）']], refs: [{ y: 2, text: '任务上限 2 m/s' }, { x: 0.5 * (w0 + w1), text: `成功窗口 ≈ ${Math.round(1000 * (w1 - w0 + 0.002))} ms` }],
    series: [{ name: 'flat s1', cls: 0, pts: sw.filter(r => r.vtd != null && r.t >= 0.4).map(r => [r.t, r.vtd]) }],
    notes: ['更晚点火：硬着陆 / 车身触地'] };
  // (b) constant-throttle family at v_td = 1.5: gas vs ignition lead (s before the case's own latest feasible ignition)
  const DL = Array.from({ length: 36 }, (_, i) => +(0.02 * i).toFixed(2));
  const famAt = (c, dl) => { const F = c.family['1.5'], tO = F[F.length - 1][0], t = tO - dl; for (let i = 1; i < F.length; i++) if (F[i][0] >= t - 1e-12) { const [ta, ua, ga] = F[i - 1], [tb, ub, gb] = F[i], l = tb > ta ? (t - ta) / (tb - ta) : 0; return t >= F[0][0] - 1e-12 ? [ga + l * (gb - ga), ua + l * (ub - ua), F[i][5] && F[i - 1][5]] : null; } return null; };
  const famG = DL.map(dl => { const g = C.map(c => famAt(c, dl)).filter(Boolean); return g.length >= C.length / 2 ? [dl, mean(g.map(r => r[0])), sd(g.map(r => r[0])), mean(g.map(r => r[1])), g.filter(r => r[2]).length / g.length, g.length] : null; }).filter(Boolean);
  const gOpt = famG[0][1], tOpt = mean(C.map(c => c.optimum['1.5'].tIgn));
  const pb = { title: '(b) 同一落地速度 1.5 m/s：用气 vs 点火提前量', xlabel: '点火比最晚可行时刻提前 (s)', ylabel: '用气 (kg)', x: [0, 0.7], y: [20, 60],
    series: [{ name: '恒定油门，油门求解使 v_td = 1.5 m/s', cls: 0, pts: famG.map(r => [r[0], r[1], `提前 ${r[0].toFixed(2)} s: ${r[1].toFixed(1)} ± ${r[2].toFixed(1)} kg, 油门 ${r[3].toFixed(2)}, n = ${r[5]}`]), band: famG.map(r => [r[0], r[1] - r[2], r[1] + r[2]]), legend: false, label: '恒定油门（油门求解使 v_td = 1.5）', labelAt: 18, anchor: 'end', dy: -12 },
      { name: '最晚点火 + 满推力 (bang-bang)', cls: 1, pts: [[0, gOpt, `bang-bang: ${gOpt.toFixed(2)} kg`]], marker: true, line: false, label: `${gOpt.toFixed(1)} kg：最晚点火、满推力`, dy: 16, legend: false }],
    notes: ['30 个起点（6 预设 × 5 种子）均值 ± 标准差'] };
  const pc2 = { title: '(c) 所需油门 vs 点火提前量', xlabel: '点火比最晚可行时刻提前 (s)', ylabel: '油门（无俯仰力矩满推力的比例）', x: [0, 0.7], y: [0.3, 1.05],
    series: [{ name: 'throttle', cls: 0, pts: famG.map(r => [r[0], r[3]]) }], refs: [{ y: 0.42, text: '≈ 悬停 (T/W = 1)' }] };
  // (d) ceiling and cap: min gas vs umax for caps 1.5 / 2.0 ; (e) min gas vs cap
  const umaxs = C[0].ceilings['1.5'].map(r => r.umax);
  const ceil = cap => umaxs.map(u => { const g = C.map(c => c.ceilings[cap].find(r => r.umax === u)).filter(r => r && !r.infeasible); return [u, g.length ? mean(g.map(r => r.gas)) : null, g.length ? mean(g.map(r => r.burn_s)) : null]; });
  const c15 = ceil('1.5'), c20 = ceil('2');
  const pd = { title: '(d) 推力上限：短而猛 vs 长而缓', xlabel: '底部推力上限 umax.bottom', ylabel: '最少用气 (kg)', x: [0.45, 1.02], y: [20, 46],
    series: [{ name: 'v_td ≤ 1.5', cls: 0, pts: c15.filter(r => r[1]).map(r => [r[0], r[1], `umax ${r[0]}: ${r[1].toFixed(1)} kg, 燃烧 ${r[2].toFixed(2)} s`]), marker: true, label: true, anchor: 'end', dy: -10 },
      { name: 'v_td ≤ 2.0', cls: 1, pts: c20.filter(r => r[1]).map(r => [r[0], r[1], `umax ${r[0]}: ${r[1].toFixed(1)} kg, 燃烧 ${r[2].toFixed(2)} s`]), marker: true, label: true, anchor: 'end', dy: 18 }],
    notes: ['umax 0.4: T/W < 1，无法止住下落'] };
  save('fig-1-vertical-theory.svg', figure([pa, pb, pc2, pd], { cols: 2, pw: 470, ph: 320, title: '图 1  垂直通道（摆正、无转动的同一弹道起点，真实孪生体物理，1 ms 分辨率）' }));
  // numbers
  const caps = R.caps.map(cap => ({ cap, gas: mean(C.map(c => c.optimum[cap]?.gas)), gas_sd: sd(C.map(c => c.optimum[cap]?.gas)), gas_td: mean(C.map(c => c.optimum[cap]?.gas_td)),
    ign_before_td: mean(C.map(c => c.optimum[cap].t_td - c.optimum[cap].tIgn)), h_ign: null, dvdt: mean(C.map(c => c.optimum[cap].dvtd_dt)), ok: C.filter(c => c.optimum[cap].ok).length }));
  const eff = k => mean(C.map(c => c.effects[k]?.gas));
  NUM.theory = { flight_arm_td: mean(C.map(c => c.t_airborne + c.optimum['1.5'].t_td - c.t_armed)), run: path.basename(d), n: C.length, F_heave: C[0].F_heave, mdot_heave: C[0].mdot_heave, F_all4: C[0].F_all4, caps, family: famG, window_ms: Math.round(1000 * (w1 - w0 + 0.002)), opt_t: tOpt, opt_gas: gOpt,
    effects: { analytic: eff('analytic'), oned_ideal: eff('oned_ideal'), oned_real: eff('oned_real'), twin: eff('twin') }, ceil15: c15, ceil20: c20,
    exec50: { levelled_gas: mean(C.map(c => c.exec50.levelled.gas)), levelled_ok: C.filter(c => c.exec50.levelled.success).length, rotating_ok: C.filter(c => c.exec50.rotating.success).length },
    start: { h0: mean(C.map(c => c.h0)), vy0: mean(C.map(c => c.vy0)), free_fall_s: mean(C.map(c => c.free_fall_s)), v_impact: mean(C.map(c => c.v_impact)), real_rate: mean(C.map(c => c.real_rate)), real_tilt: mean(C.map(c => c.real_tilt_deg)) } };
  md.push(`### 表 T1 垂直通道参考（${path.basename(d)}，${C.length} 个起点）\n\n| v_cap (m/s) | 最少用气 kg (均值 ± sd) | 触地前用气 kg | 点火距触地 s | dv_td/dt_ign (m/s per s) | 满足任务判据 |\n|---|---|---|---|---|---|\n` +
    caps.map(r => `| ${r.cap} | ${fmt(r.gas)} ± ${fmt(r.gas_sd)} | ${fmt(r.gas_td)} | ${fmt(r.ign_before_td, 3)} | ${fmt(r.dvdt, 0)} | ${r.ok}/${C.length} |`).join('\n') + '\n');
  md.push(`恒定油门族（v_td = 1.5）：提前 ${famG[famG.length - 1][0]} s 点火 ${fmt(famG[famG.length - 1][1], 1)} kg（油门 ${fmt(famG[famG.length - 1][3])}），提前 0.3 s ${fmt(famG.find(r => r[0] === 0.3)?.[1], 1)} kg，最晚点火 ${fmt(gOpt, 1)} kg（油门 1）。flat s1 满推力成功窗口 ${Math.round(1000 * (w1 - w0 + 0.002))} ms。推力上限 umax：` + c15.map(r => `${r[0]}: ${fmt(r[1], 1)} kg / ${fmt(r[2])} s`).join('，') + '\n');
}

// ------------------------------------------------------------------------------------- tables of bench runs
function benchTable(run, title, labels = null) {
  const G = group(run.rows), L = labels || run.cfg.controllers.map(c => c.label ?? c.name);
  const head = '| 配置 | n | 成功率 (95% CI) | 触地撞车 | v_td m/s | 倾角 ° | 用气 kg (全部) | 用气 kg (成功) | 点火距触地 s | 点火高度 m | 点火前用气 kg | 每次规划 rollouts | 每回合 rollouts |';
  const rows = L.filter(l => G.has(l)).map(l => {
    const s = aggregate(G.get(l)), k = G.get(l).filter(r => r.metrics.success).length;
    return `| ${l} | ${s.n} | ${pc(s.success)} (${pc(s.success_ci[0])}–${pc(s.success_ci[1])}) | ${pc(s.hull_strike)} | ${fmt(s.vtd.mean)} | ${fmt(s.tilt.mean, 1)} | ${fmt(s.gas.mean, 1)} ± ${fmt(s.gas.sd, 1)} | ${fmt(s.gas_ok.mean, 1)} | ${fmt(s.ign_before_td.mean)} | ${fmt(s.h_ign.mean)} | ${fmt(s.gas_pre_ign.mean, 1)} | ${fmt(s.evals_per_plan.mean, 0)} | ${fmt(s.evals.mean / 1000, 1)}k |`;
  });
  md.push(`### ${title}（${run.cfg.name} / ${run.stamp}）\n\n${head}\n|${'---|'.repeat(13)}\n${rows.join('\n')}\n`);
}

// ------------------------------------------------------------------------------------- F2 coast sweeps
function coastFig() {
  const A = runOf('gas-coast'), B = runOf('gas-hcoast'), d = latest('gas-reference');
  if (!A || !B) return;
  const R = d ? readJSON(path.join(d, 'reference.json')) : null;
  const tV = R ? mean(R.cases.map(c => c.t_airborne + c.optimum['1.5'].tIgn - c.t_armed)) : null, gV = R ? mean(R.cases.map(c => c.optimum['1.5'].gas)) : null;
  const agg = (run, pre) => [...group(run.rows)].map(([l, rs]) => [+l.slice(pre.length), aggregate(rs), rs]).sort((p, q) => p[0] - q[0]);
  const a = agg(A, 'coast-'), b = agg(B, 'hcoast-');
  const ser = (rows, cls, name, f, e) => ({ name, cls, marker: true, pts: rows.map(([x, s]) => [x, f(s), `${name} ${x} s: ${fmt(f(s), 2)}`]), err: e ? rows.map(([x, s]) => [x, ...e(s)]) : undefined, label: name, anchor: 'end', dy: cls ? 14 : -8, labelAt: 0 });
  const refs = tV ? [{ x: tV, text: '垂直通道最优点火时刻', bottom: true, left: true }] : [];
  const p1 = { title: '(a) 成功率', xlabel: '滑行时长：解锁后 (s)', ylabel: '成功率（95% Wilson 区间）', x: [-0.05, 1.25], y: [0, 1.05], yfmt: v => `${Math.round(100 * v)}%`, refs,
    series: [ser(a, 0, '全部阀门关闭', s => s.success, s => s.success_ci), ser(b, 1, '仅升力关闭（姿态自由）', s => s.success, s => s.success_ci)] };
  p1.series.forEach(x => { x.labelAt = 0; x.anchor = undefined; x.label = false; });
  const okA = a.filter(r => r[1].gas_ok.n >= 3), okB = b.filter(r => r[1].gas_ok.n >= 3);
  const p2 = { title: '(b) 用气（仅成功回合，均值 ± 标准差）', xlabel: '滑行时长：解锁后 (s)', ylabel: '用气 (kg)', x: [-0.05, 1.25], y: [20, 55], refs: [...refs, ...(gV ? [{ y: gV, text: `垂直通道下限 ${gV.toFixed(1)} kg（v_td 1.5）` }] : [])],
    series: [ser(okA, 0, '全部阀门关闭', s => s.gas_ok.mean, s => [s.gas_ok.mean - s.gas_ok.sd, s.gas_ok.mean + s.gas_ok.sd]), ser(okB, 1, '仅升力关闭', s => s.gas_ok.mean, s => [s.gas_ok.mean - s.gas_ok.sd, s.gas_ok.mean + s.gas_ok.sd])] };
  p2.series.forEach(x => { x.label = false; });
  const p3 = { title: '(c) 主燃烧点火时刻与触地时刻（成功回合）', xlabel: '滑行时长：解锁后 (s)', ylabel: '解锁后时间 (s)', x: [-0.05, 1.25], y: [0, 2.2], refs,
    series: [{ name: '点火（全部关闭）', cls: 0, marker: true, pts: okA.map(([x, s]) => [x, s.ign_after_arm.mean]) }, { name: '点火（仅升力关闭）', cls: 1, marker: true, pts: okB.map(([x, s]) => [x, s.ign_after_arm.mean]) },
      { name: '触地（仅升力关闭）', cls: 2, marker: true, pts: b.map(([x, s, rs]) => [x, mean(rs.map(r => r.metrics.touchdown_t - r.burn.t_arm))]) }] };
  const p4 = { title: '(d) 点火前用气（姿态修正等），成功回合', xlabel: '滑行时长：解锁后 (s)', ylabel: '点火前用气 (kg)', x: [-0.05, 1.25], y: [0, 25], refs,
    series: [{ name: '全部阀门关闭', cls: 0, marker: true, pts: okA.map(([x, s]) => [x, s.gas_pre_ign.mean]) }, { name: '仅升力关闭', cls: 1, marker: true, pts: okB.map(([x, s]) => [x, s.gas_pre_ign.mean]) }] };
  save('fig-2-coast-sweep.svg', figure([p1, p2, p3, p4], { cols: 2, pw: 470, ph: 320, title: '图 2  强制滑行扫描：CEM-MPC + 无偏用气目标，6 预设 × 5 种子，每点 30 回合', caption: `gas-coast ${A.stamp} · gas-hcoast ${B.stamp}` }));
  NUM.coast = Object.fromEntries(a.map(([x, s]) => [x, { success: s.success, gas_ok: s.gas_ok.mean, gas: s.gas.mean, vtd: s.vtd.mean }]));
  NUM.hcoast = Object.fromEntries(b.map(([x, s]) => [x, { success: s.success, gas_ok: s.gas_ok.mean, sd: s.gas_ok.sd, vtd: s.vtd.mean }]));
  NUM.t_vertical_ign_after_arm = tV;
}

// ------------------------------------------------------------------------------------- F3 profiles of one case
function profiles(preset = 'flat', seed = 1) {
  const pick = [['gas-objective', 'land', 'land 目标（CEM）'], ['gas-objective', 'gas', 'gas 目标（CEM，自由）'], ['gas-hcoast', 'hcoast-0.9', '仅升力滑行至 0.9 s'], ['gas-coast', 'coast-0.6', '全部关闭滑行 0.6 s']];
  const S = [];
  for (const [exp, lab, name] of pick) {
    const d = latest(exp), f = d && path.join(d, 'traj', `${lab}__${preset}__s${seed}.json`);
    if (!f || !fs.existsSync(f)) continue;
    const t = readJSON(f), se = series(t), b = burnStats(t), tA = b.t_arm;
    S.push({ name: `${name}：${fmt(t.metrics.gas_used_kg, 1)} kg${t.metrics.success ? '' : '（失败）'}`, se, tA, b, m: t.metrics });
  }
  if (!S.length) return;
  const P = (title, ylabel, key, y, scale = 1) => ({ title, xlabel: '解锁后时间 (s)', ylabel, x: [0, 2.6], y, legendY: 0,
    series: S.map((x, i) => ({ name: x.name, cls: i, pts: x.se.t.map((t, k) => [t - x.tA, x.se[key][k] * scale]).filter(q => q[0] >= 0 && q[0] <= 2.6) })) });
  const pa = P('(a) 底部推力合计', 'kN', 'Fb', [0, 36], 1e-3), pb = P('(b) 倾角', '°', 'tilt', [0, 100]), pc = P('(c) 高度（车身原点距路面）', 'm', 'h', [0, 10]), pd = P('(d) 竖直速度', 'm/s', 'vy', [-9, 1]), pe = P('(e) 累计用气', 'kg', 'gas', [0, 50]);
  [pa, pb, pc, pd, pe].forEach(p => p.series.forEach(x => { x.legend = false; }));
  const pl = { title: '图例（各分图颜色一致）', legendBox: S.map((x, i) => ({ name: x.name, cls: i })), lines: ['用气 = 整个回合（任务指标 gas_used_kg）', '时间轴：从解锁（车辆质心越过甲板边缘）起'] };
  save('fig-3-profiles.svg', figure([pa, pb, pc, pd, pe, pl], { cols: 2, pw: 470, ph: 290, title: `图 3  同一回合（${preset} s${seed}）的四种策略：推力、姿态、高度、速度、用气` }));
  NUM.profiles = S.map(x => ({ name: x.name, gas: x.m.gas_used_kg, success: x.m.success, vtd: x.m.touchdown_vertical_speed, ign_after_arm: x.b.ign_after_arm, t_td_after_arm: x.m.touchdown_t - x.tA }));
}

// ------------------------------------------------------------------------------------- F4 optimizer families x cold starts
function optimFig() {
  const A = runOf('gas-optim');
  if (!A) return;
  const G = group(A.rows), order = A.cfg.controllers.map(c => c.label), fams = [...new Set(order.map(l => l.split('@')[0]))], inits = [...new Set(order.map(l => l.split('@')[1]))];
  const xf = i => fams[i] ?? '';
  const mk = (title, ylabel, y, f, err) => ({ title, xlabel: '优化器（每组 ≈ 400 rollouts / 规划）', ylabel, x: [-0.5, fams.length - 0.5], y, xticks: fams.map((_, i) => i), xfmt: xf, yfmt: y[1] <= 1.05 ? (v => `${Math.round(100 * v)}%`) : undefined,
    series: inits.map((ini, j) => ({ name: `初值 ${ini}`, cls: j, line: false, marker: true, pts: fams.map((fm, i) => { const rs = G.get(`${fm}@${ini}`); if (!rs) return [i, null]; const sg = aggregate(rs); return [i - 0.24 + 0.16 * j, f(sg), `${fm} @ ${ini}: ${fmt(f(sg), 2)} (n=${sg.n})`]; }),
      err: err ? fams.map((fm, i) => { const rs = G.get(`${fm}@${ini}`); if (!rs) return [i, null, null]; const sg = aggregate(rs); return [i - 0.24 + 0.16 * j, ...err(sg)]; }).filter(e => e[1] != null) : undefined })) });
  const p1 = mk('(a) 成功率', '成功率', [0, 1.05], sg => sg.success);
  const p2 = mk('(b) 用气（成功回合，均值 ± 标准差）', '用气 (kg)', [20, 130], sg => sg.gas_ok.mean, sg => (sg.gas_ok.n > 1 ? [sg.gas_ok.mean - sg.gas_ok.sd, sg.gas_ok.mean + sg.gas_ok.sd] : [null, null]));
  p2.refs = NUM.theory ? [{ y: NUM.theory.caps.find(c => c.cap === 1.5).gas, text: '垂直通道下限（v_td 1.5）' }] : [];
  const p3 = mk('(c) 主燃烧点火：距触地时间（成功回合）', '点火距触地 (s)', [0, 3.2], sg => sg.ign_before_td.mean);
  p3.refs = NUM.theory ? [{ y: NUM.theory.caps.find(c => c.cap === 1.5).ign_before_td, text: '垂直通道最优：0.64 s', left: true, below: true }] : [];
  const p4 = mk('(d) 每回合 rollouts（千）', 'rollouts / 回合 (k)', [0, 80], sg => sg.evals.mean / 1000);
  [p2, p3, p4].forEach(p => p.series.forEach(x => { x.legend = false; }));
  save('fig-4-optimizers.svg', figure([p1, p2, p3, p4], { cols: 2, pw: 520, ph: 320, title: '图 4  优化器族 × 冷启动初值（gas 目标，6 预设 × 3 种子，每点 18 回合）', caption: `gas-optim ${A.stamp}；图例见 (a)` }));
  NUM.optim = Object.fromEntries([...G].map(([l, rs]) => { const sg = aggregate(rs); return [l, { success: sg.success, gas_ok: sg.gas_ok.mean, gas: sg.gas.mean, ign_before_td: sg.ign_before_td.mean, evals: sg.evals.mean, vtd: sg.vtd.mean }]; }));
}

// ------------------------------------------------------------------------------------- F5 budget
function budgetFig() {
  const A = runOf('gas-budget');
  if (!A) return;
  const G = group(A.rows);
  const fam = l => (l.includes('>') ? 'cem>xlbfgs' : l.startsWith('cem') ? 'cem' : l.startsWith('mppi') ? 'mppi' : l.startsWith('grad-spsa') ? 'grad (SPSA)' : l.startsWith('grad') ? 'grad' : 'xlbfgs');
  const F = ['cem', 'mppi', 'grad', 'grad (SPSA)', 'xlbfgs', 'cem>xlbfgs'];
  const pts = (f, key) => [...G].filter(([l]) => fam(l) === f && !l.includes('coarse') && !l.includes('central')).map(([l, rs]) => { const sg = aggregate(rs); return [Math.log10(sg.evals_per_plan.mean), key(sg), `${l}: ${fmt(key(sg), 2)} @ ${fmt(sg.evals_per_plan.mean, 0)} rollouts/规划`]; }).sort((p, q) => p[0] - q[0]);
  const xf = v => (Math.abs(v - Math.round(v)) < 1e-9 ? `${10 ** v}` : `${Math.round(10 ** v)}`);
  const xt = [1.5, 2, 2.5, 3, 3.5].map(v => +v.toFixed(2));
  const P = (title, ylabel, y, key) => ({ title, xlabel: '每次规划 rollouts（对数轴）', ylabel, x: [1.5, 3.6], y, xticks: xt, xfmt: xf, series: F.map((f, i) => ({ name: f, cls: i, marker: true, pts: pts(f, key) })).filter(x => x.pts.length) });
  const p1 = P('(a) 成功率 vs 预算', '成功率', [0, 1.05], sg => sg.success); p1.yfmt = v => `${Math.round(100 * v)}%`;
  const p2 = P('(b) 用气（成功回合）vs 预算', '用气 (kg)', [20, 90], sg => sg.gas_ok.mean);
  p2.series.forEach(x => { x.legend = false; });
  p2.refs = NUM.theory ? [{ y: NUM.theory.caps.find(c => c.cap === 1.5).gas, text: '垂直通道下限' }] : [];
  save('fig-5-budget.svg', figure([p1, p2], { cols: 2, pw: 520, ph: 340, title: '图 5  收敛 vs 计算预算（零初值，gas 目标，6 预设 × 3 种子）', caption: `gas-budget ${A.stamp}；图例见 (a)` }));
  NUM.budget = Object.fromEntries([...G].map(([l, rs]) => { const sg = aggregate(rs); return [l, { success: sg.success, gas_ok: sg.gas_ok.mean, evals_per_plan: sg.evals_per_plan.mean, evals: sg.evals.mean, wall: sg.wall_s.mean }]; }));
}

// ------------------------------------------------------------------------------------- F6 thrust ceiling, touchdown cap
function ceilFig() {
  const U = runOf('gas-umax'), V = runOf('gas-vcap');
  if (!U && !V) return;
  const T = NUM.theory, panels = [];
  if (U) {
    const rows = [...group(U.rows)].map(([l, rs]) => [+l.split('-')[1], aggregate(rs)]).sort((p, q) => p[0] - q[0]);
    panels.push({ title: '(a) 推力上限：用气', xlabel: 'umax.bottom', ylabel: '用气 (kg)', x: [0.45, 1.05], y: [20, 75],
      series: [{ name: 'MPC（成功回合）', cls: 0, marker: true, pts: rows.filter(r => r[1].gas_ok.n).map(([u, sg]) => [u, sg.gas_ok.mean, `umax ${u}: ${fmt(sg.gas_ok.mean, 1)} kg`]), err: rows.filter(r => r[1].gas_ok.n > 1).map(([u, sg]) => [u, sg.gas_ok.mean - sg.gas_ok.sd, sg.gas_ok.mean + sg.gas_ok.sd]) },
        ...(T ? [{ name: '垂直通道下限', cls: 1, marker: true, dash: '5 4', pts: T.ceil15.filter(r => r[1]).map(r => [r[0], r[1]]) }] : [])] });
    panels.push({ title: '(b) 推力上限：成功率', xlabel: 'umax.bottom', ylabel: '成功率', x: [0.45, 1.05], y: [0, 1.05], yfmt: v => `${Math.round(100 * v)}%`,
      series: [{ name: '成功率', cls: 0, marker: true, pts: rows.map(([u, sg]) => [u, sg.success, `umax ${u}: ${pc(sg.success)}, 燃烧 ${fmt(sg.burn_s.mean)} s`]) }] });
    NUM.umax = Object.fromEntries(rows.map(([u, sg]) => [u, { success: sg.success, gas_ok: sg.gas_ok.mean, burn_s: sg.burn_s.mean, ign_before_td: sg.ign_before_td.mean, peak_g: sg.peak_g.mean }]));
  }
  if (V) {
    const rows = [...group(V.rows)].map(([l, rs]) => [+l.split('-')[1], aggregate(rs)]).sort((p, q) => p[0] - q[0]);
    const land = runOf('gas-objective'), L = land && aggregate(land.rows.filter(r => r.label === 'land'));
    const pdr = runOf('gas-pd'), PD = pdr && [...group(pdr.rows)].filter(([l]) => l.endsWith('td0.8')).map(([l, rs]) => { const sg = aggregate(rs); return { b: +l.split('-')[1].slice(1), vtd: sg.vtd.mean, gas: sg.gas_ok.mean, success: sg.success }; }).sort((p, q) => p.vtd - q.vtd);
    panels.push({ title: '(c) 用气 vs 实际落地速度（Pareto）', xlabel: '平均触地竖直速度 (m/s)', ylabel: '用气 (kg)', x: [0, 2.1], y: [20, 60], legendY: 0,
      series: [{ name: 'MPC gas（v_cap 扫描）', cls: 0, marker: true, pts: rows.map(([v, sg]) => [sg.vtd.mean, sg.gas_ok.mean ?? sg.gas.mean, `v_cap ${v}: v_td ${fmt(sg.vtd.mean)} m/s, ${fmt(sg.gas_ok.mean, 1)} kg, 成功 ${pc(sg.success)}`]) },
        ...(T ? [{ name: '垂直下限', cls: 1, marker: true, dash: '5 4', pts: T.caps.map(c => [c.cap, c.gas]).sort((p, q) => p[0] - q[0]) }] : []),
        ...(L ? [{ name: 'MPC land', cls: 2, marker: true, line: false, pts: [[L.vtd.mean, L.gas_ok.mean, `land: ${fmt(L.vtd.mean)} m/s, ${fmt(L.gas_ok.mean, 1)} kg`]] }] : []),
        ...(PD ? [{ name: 'pd 参考', cls: 3, marker: true, dash: '2 3', pts: PD.map(r => [r.vtd, r.gas, `pd brake_frac ${r.b}: v_td ${fmt(r.vtd)} m/s, ${fmt(r.gas, 1)} kg, 成功 ${pc(r.success)}`]) }] : [])] });
    panels.push({ title: '(d) 落地速度上限：成功率', xlabel: 'v_cap (m/s)', ylabel: '成功率', x: [0.4, 1.85], y: [0, 1.05], yfmt: v => `${Math.round(100 * v)}%`,
      series: [{ name: '成功率', cls: 0, marker: true, pts: rows.map(([v, sg]) => [v, sg.success]) }] });
    NUM.vcap = Object.fromEntries(rows.map(([v, sg]) => [v, { success: sg.success, gas_ok: sg.gas_ok.mean, vtd: sg.vtd.mean, ign_before_td: sg.ign_before_td.mean }]));
  }
  save('fig-6-ceiling-cap.svg', figure(panels, { cols: 2, pw: 470, ph: 320, title: '图 6  推力上限（长而缓 vs 短而猛）与落地速度上限（平稳程度）', caption: [U && `gas-umax ${U.stamp}`, V && `gas-vcap ${V.stamp}`].filter(Boolean).join(' · ') }));
}

// ------------------------------------------------------------------------------------- F7 robustness
// colour per strategy across both rounds (variants of a round-1 strategy keep its hue; 8 slots)
const RCOL = { gas: 0, 'gas-hc0.9': 1, 'gas-hc0.9-derate0.85': 1, 'gas-vcap1.0': 2, 'gas-vcap0.5': 2, 'gas-reserve0.8': 3, 'gas-reserve0.8-w20': 3, land: 4, 'gas-reserve0.7-w20': 5, 'gas-reserve0.8-w100': 6, 'gas-derate0.85': 7 };
function robustFig() {
  const A = runOf('gas-robust'), B = runOf('gas-robust2');
  if (!A) return;
  const P = (run, title, ylabel, y, f, legend = true) => {
    const G = group(run.rows), order = run.cfg.controllers.map(c => c.label), strat = [...new Set(order.map(l => l.split('|')[0]))], mis = [...new Set(order.map(l => l.split('|')[1]))];
    const w = 0.8 / strat.length;
    return { title, xlabel: '执行与规划模型的失配（规划器不知道）', ylabel, x: [-0.5, mis.length - 0.5], y, xticks: mis.map((_, i) => i), xfmt: i => mis[i] ?? '', yfmt: y[1] <= 1.05 ? (v => `${Math.round(100 * v)}%`) : undefined,
      series: strat.map((st, j) => ({ name: st, cls: RCOL[st] ?? j, legend, marker: true, line: false, r: 4, pts: mis.map((m, i) => { const rs = G.get(`${st}|${m}`); if (!rs) return [i, null]; const sg = aggregate(rs); return [i - 0.4 + w * (j + 0.5), f(sg), `${st} | ${m}: ${fmt(f(sg), 2)} (n=${sg.n})`]; }) })) };
  };
  const panels = [P(A, '(a) 第一轮：成功率', '成功率', [0, 1.05], sg => sg.success), P(A, '(b) 第一轮：用气（全部回合）', '用气 (kg)', [25, 55], sg => sg.gas.mean, false)];
  if (B) panels.push(P(B, '(c) 第二轮（推力不足）：成功率', '成功率', [0, 1.05], sg => sg.success), P(B, '(d) 第二轮：用气（全部回合）', '用气 (kg)', [25, 55], sg => sg.gas.mean, false));
  save('fig-7-robustness.svg', figure(panels, { cols: 1, pw: 1040, ph: 300, title: '图 7  晚点火的鲁棒性：策略 × 模型失配（6 预设 × 3 种子，每点 18 回合）', caption: [`gas-robust ${A.stamp}`, B && `gas-robust2 ${B.stamp}`].filter(Boolean).join(' · ') }));
  for (const run of [A, B].filter(Boolean)) for (const [l, rs] of group(run.rows)) { const sg = aggregate(rs); (NUM.robust ||= {})[l] = { success: sg.success, gas: sg.gas.mean, gas_ok: sg.gas_ok.mean, vtd: sg.vtd.mean, hull: sg.hull_strike }; }
}

// ------------------------------------------------------------------------------------- F9 pd reference
function pdFig() {
  const A = runOf('gas-pd');
  if (!A) return;
  const rows = [...group(A.rows)].map(([l, rs]) => { const [, b, td] = l.match(/b([\d.]+)-td([\d.]+)/); return { b: +b, td: +td, s: aggregate(rs) }; });
  const ser = (td, cls, f, name) => ({ name, cls, marker: true, pts: rows.filter(r => r.td === td).sort((p, q) => p.b - q.b).map(r => [r.b, f(r.s), `brake_frac ${r.b}, td ${td}: ${fmt(f(r.s), 2)}（成功 ${pc(r.s.success)}，${fmt(r.s.gas_ok.mean, 1)} kg）`]) });
  const p1 = { title: '(a) 成功率', xlabel: 'brake_frac（越大 = 刹得越晚越猛）', ylabel: '成功率', x: [0.35, 0.95], y: [0, 1.05], yfmt: v => `${Math.round(100 * v)}%`,
    series: [ser(0.8, 0, s => s.success, '目标触地 0.8 m/s'), ser(1.5, 1, s => s.success, '目标触地 1.5 m/s')] };
  const p2 = { title: '(b) 用气（成功回合）', xlabel: 'brake_frac', ylabel: '用气 (kg)', x: [0.35, 0.95], y: [20, 55],
    series: [ser(0.8, 0, s => s.gas_ok.mean, '目标触地 0.8 m/s'), ser(1.5, 1, s => s.gas_ok.mean, '目标触地 1.5 m/s')].map(x => ({ ...x, legend: false })),
    refs: NUM.optim ? [{ y: NUM.optim['cem@zero'].gas_ok, text: `自由 CEM-MPC ${fmt(NUM.optim['cem@zero'].gas_ok, 1)} kg`, left: true }, ...(NUM.hcoast ? [{ y: NUM.hcoast['0.9'].gas_ok, text: `仅升力滑行 0.9 s 的 MPC ${fmt(NUM.hcoast['0.9'].gas_ok, 1)} kg`, left: true, below: true }] : []), { y: NUM.theory.caps.find(c => c.cap === 1.5).gas, text: '垂直通道下限', left: true }] : [] };
  save('fig-9-pd-reference.svg', figure([p1, p2], { cols: 2, pw: 470, ph: 320, title: '图 9  非 MPC 参考：pd 驾驶仪（立即扶正 → 自由落体 → 按制动曲线刹车），6 预设 × 5 种子', caption: `gas-pd ${A.stamp}；brake_frac ≥ 0.8 时制动曲线要求的减速度超过无俯仰最大推力，即饱和的最晚满推力制动` }));
}

// ------------------------------------------------------------------------------------- F8 open-loop convergence
function openloopFig() {
  const d = latest('gas-openloop');
  if (!d || !fs.existsSync(path.join(d, 'runs.jsonl'))) return;
  const R = readRows(path.join(d, 'runs.jsonl')), fams = [...new Set(R.map(r => r.family))], inits = [...new Set(R.map(r => r.init))];
  const B = Math.max(...R.map(r => r.budget)), grid = Array.from({ length: 41 }, (_, i) => Math.round(B * (i / 40) ** 2)).slice(1);
  const bestAt = (r, e) => { let b = Infinity; for (const c of r.curve) { if (c[0] > e) break; if (c[1] < b) b = c[1]; } return b; };
  const feasAt = (r, e) => r.curve.some(c => c[0] <= e && c[6]);
  const panels = inits.map((ini, k) => ({ title: `(${'abcd'[k]}) 初值 ${ini}：最优代价中位数`, xlabel: '累计 rollouts（千）', ylabel: 'log10 代价', x: [0, B / 1000], y: [1, 5],
    series: fams.map((fm, i) => ({ name: fm, cls: i, legend: k === 0, pts: grid.map(e => { const v = R.filter(r => r.family === fm && r.init === ini).map(r => Math.log10(Math.max(10, bestAt(r, e)))).sort((p, q) => p - q); return [e / 1000, v.length ? v[v.length >> 1] : null]; }) })),
    refs: NUM.theory ? [{ y: Math.log10(NUM.theory.caps.find(c => c.cap === 1.5).gas), text: '垂直通道下限' }] : [] }));
  save('fig-8-openloop.svg', figure(panels, { cols: 2, pw: 520, ph: 320, title: `图 8  开环收敛：解锁时刻的第一个规划问题，同一目标与参数化，${R.length / fams.length / inits.length} 个起点（中位数）`, caption: `gas-openloop ${path.basename(d)}；代价 < 100 ≈ 可行（无惩罚项，代价 = 用气 kg）；图例见 (a)` }));
  // final plans: gas and planned ignition per family x init
  const rows = [];
  for (const fm of fams) for (const ini of inits) {
    const rs = R.filter(r => r.family === fm && r.init === ini), fe = rs.filter(r => r.best.feasible);
    rows.push({ fm, ini, n: rs.length, feasible: fe.length, gas: mean(fe.map(r => r.best.gas)), ign: mean(fe.map(r => r.best.ign)), td: mean(fe.map(r => r.best.td?.t)), cost: median(rs.map(r => r.best.cost)), evals_feasible: median(rs.map(r => { const c = r.curve.find(c => c[6]); return c ? c[0] : null; })) });
  }
  NUM.openloop = { run: path.basename(d), budget: B, rows };
  md.push(`### 表 T10 开环收敛（gas-openloop / ${path.basename(d)}，每格 ${rows[0].n} 个起点，预算 ${B} rollouts）\n\n| 优化器 | 初值 | 可行 | 可行规划的用气 kg | 规划点火（解锁后 s） | 规划触地（解锁后 s） | 最优代价中位数 | 首次可行的 rollouts（中位数） |\n|---|---|---|---|---|---|---|---|\n` +
    rows.map(r => `| ${r.fm} | ${r.ini} | ${r.feasible}/${r.n} | ${fmt(r.gas, 1)} | ${fmt(r.ign)} | ${fmt(r.td)} | ${fmt(r.cost, 1)} | ${fmt(r.evals_feasible, 0)} |`).join('\n') + '\n');
}

// ------------------------------------------------------------------------------------- soft body
function softTable() {
  const dirs = fs.readdirSync(RUNS).filter(x => x.startsWith('soft-gas') && fs.statSync(path.join(RUNS, x)).isDirectory());
  const rows = [];
  for (const dd of dirs) for (const f of fs.readdirSync(path.join(RUNS, dd)).filter(x => x.endsWith('.json'))) {
    const j = readJSON(path.join(RUNS, dd, f)), fl = j.flight || j.meta?.flight || j;
    if (!fl?.metrics) continue;
    rows.push({ dir: dd, file: f, ...fl });
  }
  if (!rows.length) return;
  NUM.soft = rows.map(r => ({ file: r.file, preset: r.preset, controller: r.controllerConfig, metrics: r.metrics, damage: r.damage, computeSeconds: r.computeSeconds }));
  md.push(`### 表 T11 软体（tools/sim/flight.mjs，experiments/runs/soft-gas*）\n\n| 运行 | 预设 | 成功 | 失败原因 | v_td m/s | 倾角 ° | ω rad/s | 保持段最大倾角 ° | 车身触地 | 用气 kg | 触地前用气 kg | 脱落件 |\n|---|---|---|---|---|---|---|---|---|---|---|---|\n` +
    rows.sort((p, q) => p.file.localeCompare(q.file)).map(r => { const m = r.metrics; return `| ${r.file.replace(/^flight-|\.json$/g, '')} | ${r.preset} | ${m.success ? '✓' : '×'} | ${(m.fails || []).join(', ') || '–'} | ${fmt(m.touchdown_vertical_speed)} | ${fmt(m.touchdown_tilt_deg, 1)} | ${fmt(m.touchdown_angular_rate)} | ${fmt(m.max_tilt_hold_deg, 1)} | ${m.hull_strike ? '是' : '–'} | ${fmt(m.gas_used_kg, 1)} | ${fmt(m.gas_used_touchdown_kg, 1)} | ${r.damage?.detached?.length ?? '–'} |`; }).join('\n') + '\n');
}

// ------------------------------------------------------------------------------------- paired comparisons
const T975 = [12.71, 4.30, 3.18, 2.78, 2.57, 2.45, 2.36, 2.31, 2.26, 2.23, 2.20, 2.18, 2.16, 2.14, 2.13, 2.12, 2.11, 2.10, 2.09, 2.09, 2.08, 2.07, 2.07, 2.06, 2.06, 2.06, 2.05, 2.05, 2.05, 2.04];
/** gas(B) - gas(A) over the (preset, seed) pairs where both succeed: mean, 95 % t interval, wins of B. */
function paired([ea, la], [eb, lb], name) {
  const A = runOf(ea), B = ea === eb ? A : runOf(eb);
  if (!A || !B) return null;
  const key = r => `${r.preset}|${r.seed}`, ma = new Map(A.rows.filter(r => r.label === la).map(r => [key(r), r])), d = [];
  let both = 0, onlyA = 0, onlyB = 0, n = 0;
  for (const r of B.rows.filter(r => r.label === lb)) {
    const q = ma.get(key(r)); if (!q) continue; n++;
    if (q.metrics.success && r.metrics.success) { both++; d.push(r.metrics.gas_used_kg - q.metrics.gas_used_kg); } else if (q.metrics.success) onlyA++; else if (r.metrics.success) onlyB++;
  }
  const m = mean(d), s2 = sd(d), h = d.length > 1 ? T975[Math.min(d.length - 2, 29)] * s2 / Math.sqrt(d.length) : null;
  const out = { name, a: `${ea}:${la}`, b: `${eb}:${lb}`, n, both, onlyA, onlyB, mean: m, ci: h != null ? [m - h, m + h] : null, wins: d.filter(x => x < 0).length };
  (NUM.paired ||= []).push(out);
  return out;
}
function pairedTable() {
  const P = [
    [['gas-objective', 'land'], ['gas-objective', 'gas'], 'gas 目标 − land 目标'],
    [['gas-objective', 'land'], ['gas-objective', 'land-nostop'], 'land(w_stop=0) − land'],
    [['gas-hcoast', 'hcoast-0'], ['gas-hcoast', 'hcoast-0.9'], '仅升力滑行 0.9 s − 自由'],
    [['gas-hcoast', 'hcoast-0'], ['gas-hcoast', 'hcoast-1.2'], '仅升力滑行 1.2 s − 自由'],
    [['gas-coast', 'coast-0'], ['gas-coast', 'coast-0.3'], '全部关闭滑行 0.3 s − 自由'],
    [['gas-coast', 'coast-0'], ['gas-coast', 'coast-0.6'], '全部关闭滑行 0.6 s − 自由'],
    [['gas-hcoast', 'hcoast-0.9'], ['gas-attitude', 'hc0.9-rate1'], '仅升力滑行 0.9 s + 角速度整形 − 无整形'],
    [['gas-hcoast', 'hcoast-0'], ['gas-attitude', 'free-rate1'], '自由 + 角速度整形 − 自由'],
    [['gas-vcap', 'vcap-1.5'], ['gas-vcap', 'vcap-1'], 'v_cap 1.0 − 1.5'],
    [['gas-umax', 'umax-1'], ['gas-umax', 'umax-0.7'], 'umax 0.7 − 1.0'],
  ].map(p => paired(...p)).filter(Boolean);
  if (!P.length) return;
  md.push(`### 表 T12 配对比较（同一预设与种子，两者都成功的回合；用气差 B − A，95% t 区间）\n\n| 比较 | 配对 | 都成功 | 仅 A 成功 | 仅 B 成功 | 用气差 kg | 95% CI | B 更省气的回合 |\n|---|---|---|---|---|---|---|---|\n` +
    P.map(p => `| ${p.name} | ${p.n} | ${p.both} | ${p.onlyA} | ${p.onlyB} | ${fmt(p.mean)} | ${p.ci ? `${fmt(p.ci[0])} … ${fmt(p.ci[1])}` : '–'} | ${p.wins}/${p.both} |`).join('\n') + '\n');
}

// ------------------------------------------------------------------------------------- where the gas goes
function gasBudgetTable() {
  const pick = [['gas-objective', 'land'], ['gas-objective', 'gas'], ['gas-coast', 'coast-0.3'], ['gas-coast', 'coast-0.6'], ['gas-hcoast', 'hcoast-0.5'], ['gas-hcoast', 'hcoast-0.9'], ['gas-hcoast', 'hcoast-1.2'], ['gas-attitude', 'hc0.9-rate1'], ['gas-attitude', 'free-rate1']];
  const out = [];
  for (const [exp, lab] of pick) {
    const d = latest(exp); if (!d) continue;
    const td = path.join(d, 'traj'), fs_ = fs.readdirSync(td).filter(f => f.startsWith(`${lab}__`));
    const S = fs_.map(f => { const t = readJSON(path.join(td, f)); return { ...impulseStats(t), ok: t.metrics.success, preset: t.preset }; }).filter(x => x.flight != null && x.ok);
    if (!S.length) continue;
    out.push({ exp, lab, n: S.length, flight: mean(S.map(x => x.flight)), eta: mean(S.map(x => x.eta_up)), gas: mean(S.map(x => x.gas_arm_td)), gas_up: mean(S.map(x => x.gas_up)), gas_lost: mean(S.map(x => x.gas_lost)),
      flat: mean(S.filter(x => x.preset === 'flat').map(x => x.eta_up)), roof: mean(S.filter(x => x.preset === 'roof').map(x => x.eta_up)) });
  }
  if (!out.length) return;
  NUM.gasbudget = out;
  md.push(`### 表 T13 用气去向（解锁 → 触地，成功回合；用气 ∝ 推力冲量）\n\n| 策略 | 回合 | 解锁→触地 s | 竖直冲量占比 η_up | 用气 kg | 其中竖直（制动）kg | 其中损失（倾斜/水平/纯力矩）kg | η_up flat | η_up roof |\n|---|---|---|---|---|---|---|---|---|\n` +
    out.map(r => `| ${r.exp}:${r.lab} | ${r.n} | ${fmt(r.flight)} | ${fmt(r.eta)} | ${fmt(r.gas, 1)} | ${fmt(r.gas_up, 1)} | ${fmt(r.gas_lost, 1)} | ${fmt(r.flat)} | ${fmt(r.roof)} |`).join('\n') +
    `\n\n垂直通道参考（摆正、无转动，v_td 1.5）：η_up = 1，解锁→触地 ${fmt(NUM.theory?.flight_arm_td)} s，用气 ${fmt(NUM.theory?.caps.find(c => c.cap === 1.5).gas, 1)} kg（T1）。\n`);
}

theory();
coastFig();
profiles();
optimFig();
budgetFig();
ceilFig();
robustFig();
openloopFig();
pdFig();
softTable();
pairedTable();
gasBudgetTable();
for (const [name, title] of [['gas-objective', '表 T2 目标函数'], ['gas-coast', '表 T3 强制自由落体（全部阀门关闭）'], ['gas-hcoast', '表 T4 仅升力滑行（姿态通道自由）'], ['gas-optim', '表 T5 优化器 × 初值'], ['gas-budget', '表 T6 预算'], ['gas-umax', '表 T7 推力上限'], ['gas-vcap', '表 T8 落地速度上限'], ['gas-robust', '表 T9 鲁棒性'], ['gas-robust2', '表 T9b 鲁棒性（推力不足，第二轮）'], ['gas-attitude', '表 T14 空中姿态整形（搜索辅助）'], ['gas-explore', '表 T15 MPC 搜索设置'], ['gas-pd', '表 T16 非 MPC 参考：pd 驾驶仪（先摆正、自由落体、按制动曲线晚刹）']]) {
  const r = runOf(name);
  if (r) benchTable(r, title);
}
fs.writeFileSync(path.join(OUT, 'tables.md'), `<!-- generated by tools/flight/gas/report.mjs; do not edit -->\n\n${md.join('\n')}`);
fs.writeFileSync(path.join(OUT, 'numbers.json'), JSON.stringify(NUM, null, 1));
console.log(`-> ${path.relative(ROOT, OUT)}`);
