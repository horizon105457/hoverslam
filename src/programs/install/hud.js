// Motion-graphics HUD drawn with Canvas2D and composited onto the WebGL canvas after the post chain, so it is
// part of every exported film frame (tools/film grab the canvas, not the DOM) and stays crisp: no bloom, no
// motion blur, no tone mapping. Callouts project 3D anchors with the un-jittered shot camera.
import * as THREE from 'three';
import { clamp01, outCubic } from '../../cinema/ease.js';

const MONO = '"SF Mono", "Menlo", "Roboto Mono", monospace';
const SANS = '-apple-system, "PingFang SC", "Helvetica Neue", "Microsoft YaHei", sans-serif';
export const COL = { line: 'rgba(236,242,240,0.82)', dim: 'rgba(236,242,240,0.5)', faint: 'rgba(236,242,240,0.18)', mint: '#93d3b8', hot: '#ff8a3d', ice: '#a8e6ff' };

export class Hud {
  constructor(stage) {
    this.stage = stage;
    this.canvas = document.createElement('canvas');
    this.g = this.canvas.getContext('2d');
    this.tex = new THREE.CanvasTexture(this.canvas);
    this.tex.colorSpace = THREE.NoColorSpace; this.tex.minFilter = this.tex.magFilter = THREE.LinearFilter; this.tex.generateMipmaps = false;
    this.quad = new THREE.Mesh(new THREE.PlaneGeometry(2, 2), new THREE.ShaderMaterial({
      uniforms: { tHud: { value: this.tex } },
      vertexShader: 'varying vec2 vUv; void main() { vUv = uv; gl_Position = vec4(position.xy, 0.0, 1.0); }',
      fragmentShader: 'uniform sampler2D tHud; varying vec2 vUv; void main() { gl_FragColor = texture2D(tHud, vUv); }',
      transparent: true, depthTest: false, depthWrite: false, blending: THREE.NormalBlending,
    }));
    this.scene = new THREE.Scene(); this.scene.add(this.quad);
    this.cam2 = new THREE.OrthographicCamera(-1, 1, 1, -1, 0, 1);
    this.cam = new THREE.PerspectiveCamera(40, 1, 0.01, 100); // shot camera for projections
    this._v = new THREE.Vector3();
  }

  /** Start a HUD frame for the shot pose (pos, tgt, fov, roll). */
  begin(pose) {
    const r = this.stage.renderer, s = new THREE.Vector2(); r.getDrawingBufferSize(s);
    if (this.canvas.width !== s.x || this.canvas.height !== s.y) { this.canvas.width = s.x; this.canvas.height = s.y; this.tex.dispose(); }
    this.W = s.x; this.H = s.y; this.k = s.y / 1080;
    const c = this.cam;
    c.aspect = s.x / s.y; c.fov = pose.fov; c.position.copy(pose.pos); c.up.set(0, 1, 0); c.lookAt(pose.tgt);
    if (pose.roll) c.rotateZ(pose.roll);
    c.updateMatrixWorld(); c.updateProjectionMatrix();
    this.g.setTransform(1, 0, 0, 1, 0, 0);
    this.g.clearRect(0, 0, s.x, s.y);
    // soft dark halo behind every stroke and glyph: legible over bright metal and busy x-ray backgrounds
    this.g.shadowColor = 'rgba(0,0,0,0.55)'; this.g.shadowBlur = 6 * this.k; this.g.shadowOffsetX = this.g.shadowOffsetY = 0;
  }

  /** World point -> pixel coordinates; null when behind the camera or off screen (with margin). */
  project(p) {
    const v = this._v.copy(p).project(this.cam);
    if (v.z > 1 || v.z < -1) return null;
    const x = (v.x * 0.5 + 0.5) * this.W, y = (0.5 - v.y * 0.5) * this.H;
    if (x < -50 || x > this.W + 50 || y < -50 || y > this.H + 50) return null;
    return { x, y };
  }

  font(px, weight = 500, fam = SANS) { this.g.font = `${weight} ${Math.round(px * this.k)}px ${fam}`; }
  measure(text, px, weight, fam) { this.font(px, weight, fam); return this.g.measureText(text).width; }

  /** Text revealed character by character (p 0..1). */
  type(text, x, y, p, { align = 'left', color = COL.line, px = 14, weight = 500, fam = SANS, spacing = 0 } = {}) {
    const g = this.g, n = Math.round(text.length * clamp01(p));
    if (n <= 0) return 0;
    this.font(px, weight, fam);
    g.fillStyle = color; g.textAlign = align; g.textBaseline = 'alphabetic';
    if (spacing) g.letterSpacing = `${spacing * this.k}px`;
    g.fillText(text.slice(0, n), x, y);
    const w = g.measureText(text).width;
    g.letterSpacing = '0px';
    return w;
  }

  /**
   * Leader-line callout. at: world point, p: 0..1 build-in (negative values = fade out), side +1 right / -1 left,
   * lines: [{t, px, fam, weight, color}], tag: short mono code above the line.
   */
  callout(at, { p = 1, side = 1, up = -1, dx = 46, dy = 34, len = 170, tag = '', lines = [], color = COL.line, accent = COL.mint, alpha = 1 } = {}) {
    const s = this.project(at);
    if (!s || p <= 0) return null;
    const g = this.g, k = this.k;
    g.save(); g.globalAlpha = alpha;
    // the shelf is as long as its widest text: title + tag above, values below
    const first = lines[0];
    const wTitle = first ? this.measure(first.t, first.px || 17, first.weight || 600, first.fam || SANS) : 0;
    const wTag = tag ? this.measure(tag, 11, 600, MONO) + 12 * k : 0;
    let wMax = wTitle + wTag;
    for (let i = 1; i < lines.length; i++) wMax = Math.max(wMax, this.measure(lines[i].t, lines[i].px || 12, lines[i].weight || 500, lines[i].fam || SANS));
    len = Math.max(len, wMax / k + 8);
    const a = outCubic(clamp01(p / 0.35)), b = outCubic(clamp01((p - 0.2) / 0.35));
    const x1 = s.x + side * dx * k, y1 = s.y + up * dy * k, x2 = x1 + side * len * k;
    // anchor
    g.strokeStyle = accent; g.lineWidth = 1.5 * k;
    g.beginPath(); g.arc(s.x, s.y, 5 * k * a, 0, Math.PI * 2); g.stroke();
    g.fillStyle = accent; g.beginPath(); g.arc(s.x, s.y, 1.8 * k, 0, Math.PI * 2); g.fill();
    // leader
    g.strokeStyle = color; g.lineWidth = 1.2 * k;
    g.beginPath(); g.moveTo(s.x + side * 6 * k * Math.SQRT1_2, s.y + up * 6 * k * Math.SQRT1_2);
    g.lineTo(s.x + (x1 - s.x) * a, s.y + (y1 - s.y) * a);
    if (b > 0) g.lineTo(x1 + (x2 - x1) * b, y1);
    g.stroke();
    if (b > 0) { g.fillStyle = accent; g.fillRect(side > 0 ? x1 : x1 - 18 * k * b, y1 - 1.2 * k, 18 * k * b, 2.4 * k); }
    // text
    const tp = clamp01((p - 0.35) / 0.65), tx = side > 0 ? x1 + 2 * k : x1 - 2 * k, align = side > 0 ? 'left' : 'right';
    let y = y1 - 9 * k;
    if (first) this.type(first.t, tx, y, tp * 1.3, { align, px: first.px || 17, weight: first.weight || 600, fam: first.fam || SANS, color: first.color || COL.line });
    if (tag) this.type(tag, side > 0 ? tx + wTitle + 12 * k : tx - wTitle - 12 * k, y, tp * 1.6, { align, color: accent, px: 11, fam: MONO, weight: 600 });
    y = y1 + 20 * k;
    for (let i = 1; i < lines.length; i++) {
      const L = lines[i];
      this.type(L.t, tx, y, tp >= 1 ? 1 : tp * 1.4 - i * 0.12, { align, px: L.px || 12, weight: L.weight || 500, fam: L.fam || SANS, color: L.color || COL.dim });
      y += (L.px || 12) * 1.5 * k;
    }
    g.restore();
    return s;
  }

  /** Corner brackets + safe-frame marks. */
  frame(alpha = 1) {
    const g = this.g, k = this.k, m = 40 * k, L = 26 * k, W = this.W, H = this.H;
    g.save(); g.globalAlpha = alpha; g.strokeStyle = COL.faint; g.lineWidth = 1.2 * k;
    for (const [x, y, sx, sy] of [[m, m, 1, 1], [W - m, m, -1, 1], [m, H - m, 1, -1], [W - m, H - m, -1, -1]]) {
      g.beginPath(); g.moveTo(x, y + sy * L); g.lineTo(x, y); g.lineTo(x + sx * L, y); g.stroke();
    }
    g.restore();
  }

  /** Header: kit title left, step counter + timecode right. */
  header({ step = '', title = '', sub = '', T = 0, alpha = 1 }) {
    const g = this.g, k = this.k, m = 58 * k;
    g.save(); g.globalAlpha = alpha;
    this.type('COLD GAS THRUSTER KIT', m, m + 8 * k, 1, { px: 12, fam: MONO, weight: 600, color: COL.mint, spacing: 3 });
    this.type('冷气推进套件 · MINI EV', m, m + 30 * k, 1, { px: 13, weight: 500, color: COL.dim, spacing: 2 });
    const fr = Math.floor(T * 30) % 30, s = Math.floor(T);
    this.type(`T+${String(Math.floor(s / 60)).padStart(2, '0')}:${String(s % 60).padStart(2, '0')}:${String(fr).padStart(2, '0')}`, this.W - m, m + 8 * k, 1, { align: 'right', px: 12, fam: MONO, weight: 600, color: COL.dim, spacing: 2 });
    if (step) this.type(step, this.W - m, m + 30 * k, 1, { align: 'right', px: 12, fam: MONO, weight: 600, color: COL.line, spacing: 2 });
    g.restore();
    if (title) this.caption(title, sub, alpha);
  }

  /** Lower-left caption: Chinese title + English sub. */
  caption(title, sub, alpha = 1) {
    const g = this.g, k = this.k, m = 58 * k, y = this.H - 64 * k;
    g.save(); g.globalAlpha = alpha;
    g.fillStyle = COL.mint; g.fillRect(m, y - 30 * k, 3 * k, 44 * k);
    this.type(title, m + 16 * k, y - 6 * k, 1, { px: 22, weight: 500, color: COL.line, spacing: 4 });
    this.type(sub, m + 16 * k, y + 14 * k, 1, { px: 11, fam: MONO, weight: 600, color: COL.dim, spacing: 2.5 });
    g.restore();
  }

  /** Horizontal meter (label, value text, fraction) at the lower right. */
  meter(i, label, value, frac, { color = COL.mint, alpha = 1 } = {}) {
    const g = this.g, k = this.k, w = 260 * k, x = this.W - 58 * k - w, y = this.H - 70 * k - i * 46 * k;
    g.save(); g.globalAlpha = alpha;
    this.type(label, x, y - 10 * k, 1, { px: 11, fam: MONO, weight: 600, color: COL.dim, spacing: 2 });
    this.type(value, x + w, y - 10 * k, 1, { align: 'right', px: 13, fam: MONO, weight: 600, color: COL.line });
    g.fillStyle = COL.faint; g.fillRect(x, y, w, 2 * k);
    g.fillStyle = color; g.fillRect(x, y - 1 * k, w * clamp01(frac), 4 * k);
    for (let t = 0; t <= 10; t++) g.fillRect(x + (w * t) / 10, y + 5 * k, 1 * k, (t % 5 ? 3 : 6) * k);
    g.restore();
  }

  /** Text block at pixel position (multi-line, typed). */
  block(x, y, lines, p = 1, { align = 'left', alpha = 1 } = {}) {
    const g = this.g, k = this.k;
    g.save(); g.globalAlpha = alpha;
    // baseline-to-baseline: previous descent + this line's ascent (+ its own leading `lh`)
    let yy = y, prev = 0;
    lines.forEach((L, i) => {
      const px = L.px || 13;
      if (i) yy += (prev * 0.42 + px * (L.lh || 1.15)) * k;
      this.type(L.t, x, yy, p >= 1 ? 1 : p * 1.3 - i * 0.1, { align, px, fam: L.fam || SANS, weight: L.weight || 500, color: L.color || COL.line, spacing: L.spacing || 0 });
      prev = px;
    });
    g.restore();
  }

  /** Thin reticle (crosshair ring) around a world point. */
  reticle(at, p = 1, r = 26, color = COL.mint) {
    const s = this.project(at);
    if (!s || p <= 0) return;
    const g = this.g, k = this.k, R = r * k * (1.6 - 0.6 * outCubic(clamp01(p)));
    g.save(); g.globalAlpha = clamp01(p * 2); g.strokeStyle = color; g.lineWidth = 1.2 * k;
    for (let q = 0; q < 4; q++) { const a0 = q * Math.PI / 2 + 0.25; g.beginPath(); g.arc(s.x, s.y, R, a0, a0 + Math.PI / 2 - 0.5); g.stroke(); }
    for (const [dx, dy] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) { g.beginPath(); g.moveTo(s.x + dx * (R + 4 * k), s.y + dy * (R + 4 * k)); g.lineTo(s.x + dx * (R + 12 * k), s.y + dy * (R + 12 * k)); g.stroke(); }
    g.restore();
  }

  /** Letterbox bars (fraction of height each) and a global fade to black. */
  bars(f, fade = 0) {
    const g = this.g;
    g.shadowBlur = 0;
    if (f > 0) { g.fillStyle = '#000'; const h = this.H * f; g.fillRect(0, 0, this.W, h); g.fillRect(0, this.H - h, this.W, h); }
    if (fade > 0) { g.fillStyle = `rgba(0,0,0,${clamp01(fade)})`; g.fillRect(0, 0, this.W, this.H); }
  }

  /** Composite onto the canvas (after the post chain has written it). */
  end() {
    const r = this.stage.renderer;
    this.tex.needsUpdate = true;
    const ac = r.autoClear; r.autoClear = false;
    r.setRenderTarget(null);
    r.render(this.scene, this.cam2);
    r.autoClear = ac;
  }
}
