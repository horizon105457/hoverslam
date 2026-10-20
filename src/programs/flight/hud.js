// Flight telemetry drawn into a 2D canvas: shown as a DOM overlay in realtime and composited into offline film
// frames (the film tools capture only the WebGL canvas), so both look the same. Pure function of the sampled
// state at sim time t: 10 thruster bars grouped bottom / rear / side in spec order, tank pressure / temperature /
// mass, feed pressure, altitude, vertical speed, tilt, and the touchdown verdict against the task criteria.
import { PROPULSION } from '../../core/spec.js';
import { GROUPS } from '../../flight/contract.js';

const FONT = '-apple-system, "PingFang SC", "Helvetica Neue", "Microsoft YaHei", sans-serif';
const MINT = '#93d3b8', HOT = '#ff8a3d', RED = '#ff5a4e', FG = 'rgba(238,242,240,0.95)', DIM = 'rgba(238,242,240,0.58)';
const GROUP_LABEL = { bottom: '底部', rear: '尾部', side: '侧面' };

export class FlightHUD {
  constructor() {
    this.canvas = document.createElement('canvas');
    this.canvas.className = 'flight-hud';
    this.g = this.canvas.getContext('2d');
    this.ids = PROPULSION.nozzles.map(z => z.id);
  }
  resize(w, h) { if (this.canvas.width !== w || this.canvas.height !== h) { this.canvas.width = w; this.canvas.height = h; } }

  /**
   * @param s {t, thrust[], valve[], nom[], tank:{m,T,p}, m0, pc, pset, alt, vy, tilt, rate, speed, phase, roll (deg or null),
   *           verdict: null | {success, vs, tilt, rate, hull, crit}, title, sub, letterbox}
   */
  draw(s) {
    const c = this.canvas, g = this.g, W = c.width, H = c.height, k = H / 1080;
    g.clearRect(0, 0, W, H);
    const lb = s.letterbox ? H * 0.065 : 0;
    const px = v => v * k;
    g.save();
    g.textBaseline = 'alphabetic';
    // ---- title (top left)
    if (s.title) {
      shadow(g, k);
      g.fillStyle = FG; g.font = `300 ${px(40)}px ${FONT}`; g.fillText(s.title, px(40), lb + px(64));
      g.fillStyle = DIM; g.font = `500 ${px(14)}px ${FONT}`; g.fillText(s.sub || '', px(42), lb + px(90));
      noShadow(g);
    }
    // ---- thruster bars (bottom left)
    const bx = px(40), by = H - lb - px(96), bh = px(92), bw = px(16), gap = px(9), ggap = px(26);
    panel(g, bx - px(18), by - bh - px(52), px(18) * 2 + 10 * bw + 7 * gap + 2 * ggap, bh + px(130), k);
    g.font = `500 ${px(12)}px ${FONT}`; g.fillStyle = DIM; g.fillText('冷气推力器 · 阀门开度', bx, by - bh - px(26));
    let x = bx;
    for (const grp of ['bottom', 'rear', 'side']) {
      const idx = GROUPS[grp];
      g.fillStyle = DIM; g.font = `500 ${px(11)}px ${FONT}`;
      g.fillText(GROUP_LABEL[grp], x, by + px(56));
      for (const i of idx) {
        const v = clamp01(s.valve[i]), f = s.nom[i] > 0 ? clamp01(s.thrust[i] / s.nom[i]) : 0;
        g.fillStyle = 'rgba(255,255,255,0.10)'; g.fillRect(x, by - bh, bw, bh);
        // valve opening (outline) and delivered thrust (fill): they part when the regulator drops out
        g.fillStyle = f > 0.02 ? MINT : 'rgba(147,211,184,0.35)';
        g.fillRect(x, by - bh * f, bw, bh * f);
        g.fillStyle = 'rgba(255,255,255,0.85)'; g.fillRect(x, by - bh * v - px(1.5), bw, px(3));
        g.save(); g.translate(x + bw / 2 + px(3.5), by + px(6)); g.rotate(Math.PI / 2);
        g.fillStyle = f > 0.02 ? FG : DIM; g.font = `500 ${px(9)}px ${FONT}`; g.fillText(this.ids[i].replace('_', ''), 0, 0); g.restore();
        x += bw + gap;
      }
      x += ggap - gap;
    }
    const total = s.thrust.reduce((a, b) => a + b, 0);
    g.fillStyle = FG; g.font = `500 ${px(13)}px ${FONT}`;
    g.fillText(`总推力 ${(total / 1000).toFixed(1)} kN`, bx, by + px(76));
    // ---- telemetry (top right)
    const rows = [
      ['高度', `${s.alt.toFixed(2)} m`],
      ['垂直速度', `${s.vy >= 0 ? '+' : '−'}${Math.abs(s.vy).toFixed(2)} m/s`],
      ['水平速度', `${(s.speed * 3.6).toFixed(0)} km/h`],
      ['倾角', `${s.tilt.toFixed(1)}°`],
      ...(s.roll != null ? [['横滚（累计）', `${s.roll.toFixed(0)}°`]] : []),
      ['角速度', `${s.rate.toFixed(2)} rad/s`],
      ['气瓶压力', `${(s.tank.p / 1e6).toFixed(1)} MPa`],
      ['气体温度', `${s.tank.T.toFixed(0)} K`],
      ['供气压力', s.pc > 0 ? `${(s.pc / 1e6).toFixed(1)} MPa` : '—'],
      ['剩余气体', `${s.tank.m.toFixed(1)} kg`],
    ];
    const tw = px(250), tx = W - tw - px(36), ty = lb + px(36), rh = px(25);
    panel(g, tx - px(16), ty - px(14), tw + px(32), rows.length * rh + px(54), k);
    g.font = `500 ${px(12)}px ${FONT}`; g.fillStyle = DIM; g.fillText(`遥测 · ${phaseLabel(s.phase)}`, tx, ty + px(8));
    rows.forEach(([a, b], r) => {
      const y = ty + px(38) + r * rh;
      g.fillStyle = DIM; g.font = `400 ${px(14)}px ${FONT}`; g.fillText(a, tx, y);
      g.fillStyle = FG; g.font = `500 ${px(15)}px ${FONT}`; g.textAlign = 'right'; g.fillText(b, tx + tw, y); g.textAlign = 'left';
    });
    // gas gauge
    const gy = ty + px(38) + rows.length * rh - px(6);
    g.fillStyle = 'rgba(255,255,255,0.12)'; g.fillRect(tx, gy, tw, px(4));
    const fr = clamp01(s.tank.m / s.m0);
    g.fillStyle = fr > 0.35 ? MINT : HOT; g.fillRect(tx, gy, tw * fr, px(4));
    // ---- verdict (centre) after touchdown
    if (s.verdict) {
      const v = s.verdict, ok = v.success, col = ok ? MINT : RED;
      const cx = W / 2, cy = lb + px(150);
      shadow(g, k);
      g.textAlign = 'center';
      g.fillStyle = col; g.font = `300 ${px(56)}px ${FONT}`; g.fillText(verdictLabel(v), cx, cy);
      g.fillStyle = FG; g.font = `400 ${px(16)}px ${FONT}`;
      const cr = v.crit || {};
      const mark = (val, lim, unit, d = 2) => `${val.toFixed(d)}${unit}${lim !== undefined ? (val <= lim ? ' ✓' : ' ✗') : ''}`;
      g.fillText(`触地垂直速度 ${mark(v.vs, cr.max_vertical_speed, ' m/s')}   ·   倾角 ${mark(v.tilt, cr.max_tilt_deg, '°', 1)}   ·   角速度 ${mark(v.rate, cr.max_angular_rate, ' rad/s')}   ·   ${v.hull ? '车身触地 ✗' : '四轮着地 ✓'}${v.hold !== undefined && cr.upright_hold ? `   ·   ${cr.upright_hold} s 内最大倾角 ${mark(v.hold, cr.max_tilt_deg, '°', 0)}` : ''}`, cx, cy + px(34));
      if (v.note) { g.fillStyle = DIM; g.font = `400 ${px(13)}px ${FONT}`; g.fillText(v.note, cx, cy + px(58)); }
      g.textAlign = 'left';
      noShadow(g);
    }
    g.restore();
  }
}

const clamp01 = v => (v < 0 ? 0 : v > 1 ? 1 : v);
/** 软着陆 = task success; 硬着陆 = the touchdown itself broke a limit; 着陆后翻车 = soft touchdown, then a hull strike / roll-over. */
export function verdictLabel(v) {
  if (v.success) return '软着陆';
  const c = v.crit || {};
  const hardTd = (c.max_vertical_speed !== undefined && v.vs > c.max_vertical_speed) || (c.max_tilt_deg !== undefined && v.tilt > c.max_tilt_deg) || (c.max_angular_rate !== undefined && v.rate > c.max_angular_rate);
  return hardTd ? '硬着陆' : '着陆后翻车';
}
const phaseLabel = p => ({ deck: '桥面', air: '空中', ground: '着地' })[p] || p || '';
function panel(g, x, y, w, h, k) {
  g.save(); g.fillStyle = 'rgba(14,18,20,0.46)'; g.strokeStyle = 'rgba(255,255,255,0.12)'; g.lineWidth = Math.max(1, k);
  const r = 14 * k; g.beginPath(); g.roundRect(x, y, w, h, r); g.fill(); g.stroke(); g.restore();
}
function shadow(g, k) { g.shadowColor = 'rgba(0,0,0,0.55)'; g.shadowBlur = 18 * k; g.shadowOffsetY = 2 * k; }
function noShadow(g) { g.shadowColor = 'transparent'; g.shadowBlur = 0; g.shadowOffsetY = 0; }
