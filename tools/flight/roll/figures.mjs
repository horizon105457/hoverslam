// SVG figures of the barrel-roll report (no plotting dependency): small multiples on a shared time axis, one y-scale
// per panel, 2 px lines, hairline grid, event markers, light + dark (prefers-color-scheme) palettes.
// usage: node tools/flight/roll/figures.mjs out=experiments/reports/barrel-roll/fig
//          soft=public/sims/flight-mpc-roll.sim twin=<coldgas-traj@1 json> [cmp=<twin traj of the soft body's launch>]
//          [bench=<bench run dir> label=<controller>]
//          [sweep=<bench run dir of the jump-height sweep>]
//          [softset=<a.sim,b.sim,...> (first highlighted)]
// Writes timeline-soft.svg, timeline-twin.svg, soft-vs-twin.svg, seeds.svg, soft-cases.svg, jump-sweep.svg (each when its
// input is given; soft-vs-twin needs soft= and cmp= (a twin trajectory of the same launch), timeline-twin uses twin=).
import fs from 'fs';
import path from 'path';
import zlib from 'zlib';
import { parseArgs } from '../../lib/args.mjs';
import { unpackRecording } from '../../../src/softbody/recording.js';
import { GROUPS } from '../../../src/flight/contract.js';
import { Propulsion } from '../../../src/flight/propulsion.js';
import { WORLD, PROPULSION } from '../../../src/core/spec.js';
import { fromTraj, fromSoft, rollSeries, rollMetrics } from './metrics.mjs';

const DEG = 180 / Math.PI;
// palette: reference categorical slots 1-3 (validated light / dark), chrome and ink
const CSS = `
.viz{--sf:#fcfcfb;--ink:#0b0b0b;--ink2:#52514e;--mut:#898781;--grid:#e1e0d9;--axis:#c3c2b7;--c1:#2a78d6;--c2:#eb6834;--c3:#1baf7a;--dim:#c3c2b7}
@media (prefers-color-scheme: dark){.viz{--sf:#1a1a19;--ink:#f0efec;--ink2:#c3c2b7;--mut:#898781;--grid:#2c2c2a;--axis:#383835;--c1:#3987e5;--c2:#d95926;--c3:#199e70;--dim:#4a4a46}}
text{font-family:system-ui,-apple-system,"PingFang SC","Noto Sans CJK SC","Segoe UI",sans-serif;fill:var(--ink2);font-size:12px}
.t{font-size:15px;fill:var(--ink);font-weight:600}.st{font-size:12px;fill:var(--ink2)}.ax{font-size:11px;fill:var(--mut);font-variant-numeric:tabular-nums}
.pl{font-size:12px;fill:var(--ink)}.ev{font-size:10.5px;fill:var(--mut)}
.g{stroke:var(--grid);stroke-width:1}.b{stroke:var(--axis);stroke-width:1}.r{stroke:var(--axis);stroke-width:1}
.l{fill:none;stroke-width:2;stroke-linejoin:round;stroke-linecap:round}.c1{stroke:var(--c1)}.c2{stroke:var(--c2)}.c3{stroke:var(--c3)}.dim{stroke:var(--dim);stroke-width:1.25}
.f1{fill:var(--c1)}.f2{fill:var(--c2)}.f3{fill:var(--c3)}.dot{stroke:var(--sf);stroke-width:2}`;
const esc = s => String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;');

/** Clean tick values over [lo, hi]. */
function ticks(lo, hi, n = 4) {
  const span = hi - lo || 1, raw = span / n, mag = 10 ** Math.floor(Math.log10(raw)), f = raw / mag;
  const step = (f < 1.5 ? 1 : f < 3 ? 2 : f < 7 ? 5 : 10) * mag, out = [];
  for (let v = Math.ceil(lo / step - 1e-9) * step; v <= hi + 1e-9; v += step) out.push(+v.toFixed(10));
  return out;
}
const fmt = v => (Math.abs(v) >= 1000 ? `${(v / 1000).toFixed(v % 1000 ? 1 : 0)}K` : String(+v.toFixed(2)));

class Fig {
  constructor(w, title, sub) { this.w = w; this.h = 64; this.parts = []; this.title = title; this.sub = sub; this.L = 64; this.R = 150; }
  /** A panel: {h, yr, ylabel, series [{x[], y[], cls, label, dim}], refs [{y, label}], events [{t, label}], xr, xlabel} */
  panel(p) {
    const y0 = this.h + 22, H = p.h ?? 120, x0 = this.L, W = this.w - this.L - this.R, [a, b] = p.xr, [lo, hi] = p.yr;
    const X = t => x0 + ((t - a) / (b - a)) * W, Y = v => y0 + H - ((v - lo) / (hi - lo)) * H, o = this.parts;
    o.push(`<text class="pl" x="${x0}" y="${y0 - 4}">${esc(p.ylabel)}</text>`);
    for (const v of ticks(lo, hi, p.nt ?? 4)) { o.push(`<line class="g" x1="${x0}" x2="${x0 + W}" y1="${Y(v).toFixed(1)}" y2="${Y(v).toFixed(1)}"/>`, `<text class="ax" x="${x0 - 6}" y="${(Y(v) + 4).toFixed(1)}" text-anchor="end">${fmt(v)}</text>`); }
    o.push(`<line class="b" x1="${x0}" x2="${x0 + W}" y1="${y0 + H}" y2="${y0 + H}"/>`);
    for (const r of p.refs || []) if (r.y >= lo && r.y <= hi) o.push(`<line class="r" x1="${x0}" x2="${x0 + W}" y1="${Y(r.y).toFixed(1)}" y2="${Y(r.y).toFixed(1)}" stroke-width="1.5"/>`, `<text class="ev" x="${x0 + W + 6}" y="${(Y(r.y) + 4).toFixed(1)}">${esc(r.label)}</text>`);
    for (const e of p.events || []) if (e.t >= a && e.t <= b) {
      o.push(`<line class="g" x1="${X(e.t).toFixed(1)}" x2="${X(e.t).toFixed(1)}" y1="${y0}" y2="${y0 + H}"/>`);
      if (p.eventLabels) o.push(`<text class="ev" x="${(X(e.t) + 3).toFixed(1)}" y="${y0 + 10}">${esc(e.label)}</text>`);
    }
    const labels = [];
    for (const s of p.series) {
      let d = '', pen = false;
      for (let i = 0; i < s.x.length; i++) {
        const t = s.x[i], v = s.y[i];
        if (!(t >= a && t <= b) || !Number.isFinite(v)) { pen = false; continue; }
        const vc = v < lo ? lo : v > hi ? hi : v;
        d += `${pen ? 'L' : 'M'}${X(t).toFixed(1)},${Y(vc).toFixed(1)}`; pen = true;
      }
      o.push(`<path class="l ${s.dim ? 'dim' : s.cls}" d="${d}"/>`);
      if (s.label) { // direct label at the line's last visible point (nudged apart only by a few px, see below)
        let i = s.x.length - 1; while (i > 0 && !(s.x[i] <= b && Number.isFinite(s.y[i]))) i--;
        labels.push({ y: Y(Math.max(lo, Math.min(hi, s.y[i]))), x: X(Math.min(b, s.x[i])), text: s.label, cls: s.cls });
      }
    }
    labels.sort((p1, p2) => p1.y - p2.y);
    for (let i = 1; i < labels.length; i++) if (labels[i].y - labels[i - 1].y < 13) labels[i].y = labels[i - 1].y + 13;
    for (const l of labels) o.push(`<circle class="dot f${l.cls.slice(1)}" cx="${(x0 + W + 8).toFixed(1)}" cy="${(l.y).toFixed(1)}" r="4"/>`, `<text class="pl" x="${x0 + W + 16}" y="${(l.y + 4).toFixed(1)}">${esc(l.text)}</text>`);
    if (p.xlabel !== false) for (const t of ticks(a, b, 8)) o.push(`<text class="ax" x="${X(t).toFixed(1)}" y="${y0 + H + 14}" text-anchor="middle">${fmt(t)}</text>`);
    this.h = y0 + H + (p.xlabel !== false ? 22 : 8);
    if (p.xlabel) { o.push(`<text class="ax" x="${x0 + W}" y="${this.h + 6}" text-anchor="end">${esc(p.xlabel)}</text>`); this.h += 12; }
    return this;
  }
  legend(items) { // [{cls, label}] (always present for >= 2 series)
    let x = this.L; const y = this.h + 16;
    for (const it of items) { this.parts.push(`<line class="l ${it.cls}" x1="${x}" x2="${x + 18}" y1="${y - 4}" y2="${y - 4}"/>`, `<text class="pl" x="${x + 24}" y="${y}">${esc(it.label)}</text>`); x += 34 + 12.5 * it.label.length; }
    this.h = y + 2; return this;
  }
  svg() {
    const H = this.h + 14;
    return `<svg xmlns="http://www.w3.org/2000/svg" class="viz" width="${this.w}" height="${H}" viewBox="0 0 ${this.w} ${H}"><style>${CSS}</style>`
      + `<rect width="100%" height="100%" fill="var(--sf)"/><text class="t" x="${this.L}" y="24">${esc(this.title)}</text><text class="st" x="${this.L}" y="44">${esc(this.sub)}</text>${this.parts.join('')}</svg>\n`;
  }
}
Fig.prototype.h0 = 64;

// ------------------------------------------------------------------------------------------------ data
function readSim(file) {
  const raw = zlib.gunzipSync(fs.readFileSync(file));
  return unpackRecording(raw.buffer.slice(raw.byteOffset, raw.byteOffset + raw.length)).meta;
}
/** Gas used (kg) at which the regulator drops out (tank pressure = p_set + dp_min on the blowdown polytrope). */
function dropoutGas() {
  const P = new Propulsion(), n = PROPULSION.tank.polytropic_n, pd = PROPULSION.regulator.p_set + PROPULSION.regulator.dp_min;
  return P.m0 - P.V * (pd / P.K) ** (1 / n);
}
function series(src) {
  const { phi } = rollSeries(src), grp = g => src.thrust.map(r => GROUPS[g].reduce((s, i) => s + r[i], 0) / 1000);
  return { t: src.t, y: src.state.map(s => s[1]), vy: src.state.map(s => s[8]), phi: Array.from(phi, v => v * DEG), gas: src.tank.map(r => src.m0 - r[0]), B: grp('bottom'), Rr: grp('rear'), S: grp('side') };
}
function events(src, m) {
  const ev = [{ t: m.t_edge, label: '越过桥边' }, { t: m.t_apex, label: '顶点' }];
  if (m.t_roll_end != null) ev.push({ t: m.t_roll_end, label: '横滚完成' });
  if (m.t_touchdown != null) ev.push({ t: m.t_touchdown, label: '触地' });
  return ev;
}

function timeline(src, title, sub, xr) {
  const m = rollMetrics(src), S = series(src), ev = events(src, m), dg = dropoutGas(), f = new Fig(860, title, sub);
  const ymax = Math.ceil(Math.max(...S.y) + 1);
  f.panel({ xr, yr: [0, ymax], ylabel: '质心高度 y (m)', series: [{ x: S.t, y: S.y, cls: 'c1' }], refs: [{ y: WORLD.deck.top, label: `桥面 ${WORLD.deck.top} m` }, { y: m.apex_com_y, label: `顶点 ${m.apex_com_y.toFixed(2)} m` }], events: ev, eventLabels: true, xlabel: false });
  f.panel({ xr, yr: [-400, 40], ylabel: '横滚角 φ (°，展开；− = 右侧向下)', series: [{ x: S.t, y: S.phi, cls: 'c1' }], refs: [{ y: 0, label: '0°' }, { y: -360, label: '−360°' }], events: ev, xlabel: false });
  const vlo = Math.floor(Math.min(...S.vy) - 1), vhi = Math.ceil(Math.max(...S.vy) + 1);
  f.panel({ xr, yr: [vlo, vhi], ylabel: '竖直速度 v_y (m/s)', series: [{ x: S.t, y: S.vy, cls: 'c1' }], refs: [{ y: -2, label: '−2 m/s 限值' }], events: ev, xlabel: false });
  // the three groups all end at 0 kN: a legend instead of colliding end labels
  f.legend([{ cls: 'c1', label: '底部 4 个' }, { cls: 'c2', label: '尾部 4 个' }, { cls: 'c3', label: '侧向 2 个' }]);
  f.panel({ xr, yr: [0, 36], ylabel: '推力 (kN，按喷管组)', series: [{ x: S.t, y: S.B, cls: 'c1' }, { x: S.t, y: S.Rr, cls: 'c2' }, { x: S.t, y: S.S, cls: 'c3' }], events: ev, xlabel: false });
  f.panel({ xr, yr: [0, 150], ylabel: '已用气体 (kg)', series: [{ x: S.t, y: S.gas, cls: 'c1' }], refs: [{ y: dg, label: `调压失效 ${dg.toFixed(0)} kg` }, { y: src.m0, label: `满瓶 ${src.m0.toFixed(0)} kg` }], events: ev, xlabel: '时间 t (s)' });
  return f.svg();
}

function compare(a, b, la, lb, title, sub, xr) {
  // the two runs converge (same plan, same physics up to the model gap): legend first, no end labels
  const A = series(a), B = series(b), f = new Fig(860, title, sub).legend([{ cls: 'c1', label: la }, { cls: 'c2', label: lb }]), ma = rollMetrics(a);
  f.panel({ xr, yr: [0, Math.ceil(Math.max(...A.y, ...B.y) + 1)], ylabel: '质心高度 y (m)', series: [{ x: A.t, y: A.y, cls: 'c1' }, { x: B.t, y: B.y, cls: 'c2' }], refs: [{ y: WORLD.deck.top, label: '桥面' }], events: events(a, ma), xlabel: false });
  f.panel({ xr, yr: [-400, 40], ylabel: '横滚角 φ (°)', series: [{ x: A.t, y: A.phi, cls: 'c1' }, { x: B.t, y: B.phi, cls: 'c2' }], refs: [{ y: -360, label: '−360°' }], events: events(a, ma), xlabel: false });
  f.panel({ xr, yr: [Math.floor(Math.min(...A.vy, ...B.vy) - 1), Math.ceil(Math.max(...A.vy, ...B.vy) + 1)], ylabel: '竖直速度 v_y (m/s)', series: [{ x: A.t, y: A.vy, cls: 'c1' }, { x: B.t, y: B.vy, cls: 'c2' }], events: events(a, ma), xlabel: false });
  f.panel({ xr, yr: [0, 150], ylabel: '已用气体 (kg)', series: [{ x: A.t, y: A.gas, cls: 'c1' }, { x: B.t, y: B.gas, cls: 'c2' }], refs: [{ y: dropoutGas(), label: '调压失效' }], events: events(a, ma), xlabel: '时间 t (s)' });
  return f.svg();
}

/** Overlay of several runs (gray) with one highlighted (slot 1): sources [{src, hi}]. */
function overlay(all, title, sub, xr) {
  const f = new Fig(860, title, sub);
  const mk = key => all.map(x => { const S = series(x.src); return { x: S.t, y: S[key], dim: !x.hi, cls: 'c1' }; }).sort((p, q) => (p.dim === q.dim ? 0 : p.dim ? -1 : 1));
  f.panel({ xr, yr: [0, 16], ylabel: '质心高度 y (m)', series: mk('y'), refs: [{ y: WORLD.deck.top, label: '桥面' }], xlabel: false });
  f.panel({ xr, yr: [-400, 40], ylabel: '横滚角 φ (°)', series: mk('phi'), refs: [{ y: -360, label: '−360°' }], xlabel: false });
  f.panel({ xr, yr: [-14, 8], ylabel: '竖直速度 v_y (m/s)', series: mk('vy'), refs: [{ y: -2, label: '−2 m/s' }], xlabel: false });
  f.panel({ xr, yr: [0, 150], ylabel: '已用气体 (kg)', series: mk('gas'), refs: [{ y: dropoutGas(), label: '调压失效' }], xlabel: '时间 t (s)' });
  return f.svg();
}
function seeds(dir, label, title, sub, xr) {
  const rows = fs.readFileSync(path.join(dir, 'episodes.jsonl'), 'utf8').trim().split('\n').map(l => JSON.parse(l)).filter(r => r.label === label && r.traj);
  rows.sort((p, q) => (p.preset < q.preset ? -1 : p.preset > q.preset ? 1 : 0) || p.seed - q.seed);
  const all = rows.map(r => ({ src: fromTraj(JSON.parse(fs.readFileSync(path.join(dir, r.traj), 'utf8'))), hi: r.seed === 1 }));
  return overlay(all, title, sub, xr); // identity is in the title (one highlighted run, the rest gray)
}

/** Jump-height sweep: per controller label (h_jump target), mean / per-seed gas and the success rate. */
function sweep(dir, title, sub) {
  const S = JSON.parse(fs.readFileSync(path.join(dir, 'roll-summary.json'), 'utf8'));
  const E = fs.readFileSync(path.join(dir, 'roll-episodes.jsonl'), 'utf8').trim().split('\n').map(l => JSON.parse(l));
  const labs = Object.keys(S).sort((p, q) => parseFloat(p.slice(1)) - parseFloat(q.slice(1))), f = new Fig(860, title, sub), o = f.parts;
  const x0 = f.L, W = f.w - f.L - f.R, band = W / labs.length, name = lab => (lab === 'h0' ? '0（不跳）' : `${lab.slice(1)} m`);
  const pane = (y0, H, lo, hi, ylabel, val, fmtv, refs = []) => {
    const Y = v => y0 + H - ((v - lo) / (hi - lo)) * H;
    o.push(`<text class="pl" x="${x0}" y="${y0 - 14}">${esc(ylabel)}</text>`);
    for (const v of ticks(lo, hi, 4)) o.push(`<line class="g" x1="${x0}" x2="${x0 + W}" y1="${Y(v).toFixed(1)}" y2="${Y(v).toFixed(1)}"/>`, `<text class="ax" x="${x0 - 6}" y="${(Y(v) + 4).toFixed(1)}" text-anchor="end">${fmt(v)}</text>`);
    for (const r of refs) o.push(`<line class="r" x1="${x0}" x2="${x0 + W}" y1="${Y(r.y).toFixed(1)}" y2="${Y(r.y).toFixed(1)}" stroke-width="1.5"/>`, `<text class="ev" x="${x0 + W + 6}" y="${(Y(r.y) + 4).toFixed(1)}">${esc(r.label)}</text>`);
    labs.forEach((lab, i) => {
      const cx = x0 + band * (i + 0.5), vs = val(lab);
      for (const v of vs.pts || []) o.push(`<circle class="f1" cx="${(cx - 14).toFixed(1)}" cy="${Y(v).toFixed(1)}" r="3" opacity="0.5"/>`);
      o.push(`<circle class="dot f1" cx="${cx.toFixed(1)}" cy="${Y(vs.v).toFixed(1)}" r="5"/>`, `<text class="pl" x="${(cx + 10).toFixed(1)}" y="${(Y(vs.v) - 7).toFixed(1)}">${fmtv(vs.v)}</text>`);
    });
    o.push(`<line class="b" x1="${x0}" x2="${x0 + W}" y1="${y0 + H}" y2="${y0 + H}"/>`);
    return y0 + H;
  };
  let y = pane(f.h + 34, 150, 40, 140, '已用气体 (kg)：大点 = 平均，左侧小点 = 各种子', lab => ({ v: S[lab].mean.gas_used_kg, pts: E.filter(e => e.label === lab).map(e => e.gas_used_kg) }), v => `${v.toFixed(0)} kg`, [{ y: dropoutGas(), label: '调压失效' }]);
  y = pane(y + 50, 120, 0, 120, '成功率 (%)：跳跃 + 360° + 软着陆', lab => ({ v: 100 * S[lab].success_rate }), v => `${v.toFixed(0)}%`); // headroom above 100 % for the labels
  labs.forEach((lab, i) => o.push(`<text class="ax" x="${(x0 + band * (i + 0.5)).toFixed(1)}" y="${y + 16}" text-anchor="middle">${esc(name(lab))}</text>`));
  o.push(`<text class="ax" x="${x0 + W}" y="${y + 34}" text-anchor="end">跳跃目标 h_jump（质心爬升）</text>`);
  f.h = y + 36;
  return f.svg();
}

// ------------------------------------------------------------------------------------------------ main
const a = parseArgs(), out = a.out || 'experiments/reports/barrel-roll/fig';
fs.mkdirSync(out, { recursive: true });
const xr = [0.4, +(a.tmax || 5.2)];
const soft = a.soft ? fromSoft(readSim(a.soft)) : null, twin = a.twin ? fromTraj(JSON.parse(fs.readFileSync(a.twin, 'utf8'))) : null;
const w = (f, s) => { fs.writeFileSync(path.join(out, f), s); console.log(path.join(out, f)); };
if (soft) { const m = rollMetrics(soft); w('timeline-soft.svg', timeline(soft, '软体闭环 · 纯 MPC 跳跃横滚 360°（flat 预设）', `爬升 ${m.climb_m.toFixed(2)} m · 横滚 ${m.roll_td_deg.toFixed(0)}° · 触地 v_y ${m.td_vy?.toFixed(2)} m/s，倾角 ${m.td_tilt_deg?.toFixed(1)}° · 用气 ${m.gas_used_kg?.toFixed(1)} kg`, xr)); }
if (twin) { const m = rollMetrics(twin); w('timeline-twin.svg', timeline(twin, `刚体孪生 · 纯 MPC 跳跃横滚 360°（${twin.preset ?? 'flat'}，种子 ${a.twinSeed ?? 1}）`, `爬升 ${m.climb_m.toFixed(2)} m · 横滚 ${m.roll_td_deg.toFixed(0)}° · 触地 v_y ${m.td_vy?.toFixed(2)} m/s，倾角 ${m.td_tilt_deg?.toFixed(1)}° · 用气 ${m.gas_used_kg?.toFixed(1)} kg`, xr)); }
const cmp = a.cmp ? fromTraj(JSON.parse(fs.readFileSync(a.cmp, 'utf8'))) : twin;
if (soft && cmp) w('soft-vs-twin.svg', compare(soft, cmp, '软体', '刚体孪生', '同一 MPC 配置、同一名义发射（flat 64 km/h，无随机化）：软体闭环 vs 刚体孪生', '软体：每 40 ms 从晶格状态估计重规划；孪生：规划模型本身（控制器噪声种子相同）', xr));
if (a.bench) w('seeds.svg', seeds(a.bench, a.label, '刚体孪生 · 多种子（灰：其余种子，蓝：种子 1）', `${a.label} · ${path.basename(a.bench)}`, xr));
if (a.softset) { // comma-separated .sim files, the first one highlighted
  const fs_ = a.softset.split(',');
  w('soft-cases.svg', overlay(fs_.map((f, i) => ({ src: fromSoft(readSim(f)), hi: i === 0 })), `软体闭环 · ${fs_.length} 个工况（蓝：公开录像 flight-mpc-roll，灰：其余）`, '控制器噪声种子 1 / 2、发射速度 58 / 70 km/h、rear 预设；同一 MPC 配置', xr));
}
if (a.sweep) w('jump-sweep.svg', sweep(a.sweep, '跳跃高度目标 h_jump 的代价', '刚体孪生 flat，多种子；成功 = 实际爬升 ≥ 2.0 m（目标值减 0.5 m 余量）且 360° 且软着陆'));
