// Program "valves" (直到用计算机模拟 段落/02_冷气推进): "这一次我们打开冷气推进系统". 01's slam is rewound (2D, in the
// film's compositor) back to the car just off the deck edge, in the air; this shot picks up on that frame (slam film
// time `sf`, the slam's own camera there) and freezes the world. The car's shell turns into an x-ray ghost (a scan plane
// sweeps from the tail to the nose; the solid shell is clipped on the other side), the cold-gas kit inside shows
// (six COPV bottles, manifold, feed lines, ten nozzle modules) while the camera dives under the car (looking up at its
// floor against the sky, the deck slab cutting across), the ten nozzles light up one by one (a pulse down each feed
// line, then the status ring), the camera rises to the car's outer side, all close and all open with the
// compositor's valve grid; then a cut to the street looking back up at the edge, and a ghost of the car drops to a spot
// part of the way out to 01's crash site, the camera tilting down with it and pushing in, the ghost slowing
// down and levelling out ("落地时下落得尽量慢，车身尽量端正"). The world stays frozen throughout, nothing is darkened:
// the shell ghost is a tinted glass that reads on the daylit scene.
//
// Builds on the slam program (same world, recording, kit parts, ramp), so its frame at `sf` is the slam's frame.
// The compositor's render script passes the word timings (段落/02_冷气推进/tools/render_xray.py, from out/vo.json):
//   node tools/film/film.mjs program=valves port=<p> q=<timings> out=<…/renders/xray_1080p30.mp4> samples=16 marks=1
// Film time T = the segment's time (s): the compositor plays the rewind until `th`, this render from there on.
// film.marks(T): nozzle exits, the car's centre, the drop ghost (centre, ground point, axes, path) in screen px, for
// the compositor's tags, glows, arrows and the dashed path.
// Look-dev: pose=az,d,h,fy,fov,fs (camera, see KEYS), vkeys=<KEYS as JSON> and x=<0..1> (scan) override the timeline.
// only=drop: the drop ghost alone on black (段落/03_代价 lays it over its graphics board, tools/render_ghost.py).
import * as THREE from 'three';
import SlamProgram from '../slam/index.js';
import { renderFilmFrame } from '../../cinema/film.js';
import { fitFrame } from '../../softbody/actuators.js';
import { PROPULSION } from '../../core/spec.js';
import { smoother } from '../../cinema/ease.js';

const v3 = (x, y, z) => new THREE.Vector3(x, y, z);
const P = new URLSearchParams(location.search);
const num = (k, d) => (P.has(k) ? +P.get(k) : d);
const clamp = (x, a = 0, b = 1) => Math.min(b, Math.max(a, x));
const rng = (t, a, b) => clamp((t - a) / (b - a));
const smooth = u => (u = clamp(u), u * u * (3 - 2 * u));
const GOLD = new THREE.Color(0xffd24a);

// ------------------------------------------------------------------ timeline (segment s; defaults = out/vo.json 2026-10-22)
const S_FREEZE = num('sf', 0.9);      // slam film time the rewind lands on (the car just off the deck edge, in the air)
const TL = {
  hand: num('th', 0.55),              // the cut from the rewind to this render
  scan: [num('x0', 0.62), num('x1', 1.5)],     // x-ray scan, tail -> nose ("冷气推进系统")
  lit: num('lit', 2.58), dl: num('dl', 0.09),  // nozzle i lights at lit + i·dl ("10 个喷气阀门")
  off: [num('off0', 5.46), num('off1', 5.88)], // all closed ("全关")
  on: [num('on0', 6.17), num('on1', 6.59)],    // all open ("全开")
  cut: num('cut', 6.59),                       // cut to the street, looking back up at the edge ("让车落地时")
  drop: [num('g0', 6.75), num('g1', 9.45)],    // the ghost leaves the edge .. touches down ("让车落地时 … 端正")
  level: [num('lv0', 8.35), num('lv1', 9.35)], // its tilt goes to zero ("车身尽量端正")
  total: num('total', 9.95),
};

// ------------------------------------------------------------------ camera keys (segment time)
// Rig around the frozen car's centre A in the launch heading frame (as the slam): azimuth 0 = behind, 90 = the outer
// side; d horizontal distance, h height over A, fy focus height over A, fs focus shift to the camera's right (m:
// positive puts the car left of centre, room for the valve grid on the right). Row 0 is replaced by the slam's pose.
// From the cut on, dropCam() (below) takes over.
const KEYS = [
  // T,     az,   d,    h,    fy,    fov, fs
  [TL.hand,  0,   7.0, -0.6,  0.0,   54,  0],     // (the slam's pose at the freeze, set in init)
  [1.65,    22,   4.2, -2.6,  0.15,  52,  0.2],    // dives under the car with the scan: looking up at the floor, the kit
  [3.20,    34,   3.8, -2.6,  0.15,  50,  0.9],    //   against the sky, the deck slab cutting across, pushing in (the nozzles light)
  [4.10,    76,   6.8, -1.9,  0.35,  42,  1.75],   // rises to its outer side, a little below it, car on the left (the
  [6.60,    80,   6.6, -1.8,  0.3,   42,  1.8],    //   valve grid comes in on the right)
];
if (P.has('vkeys')) KEYS.splice(0, KEYS.length, ...JSON.parse(P.get('vkeys')));   // look-dev (not `keys`: the slam reads that)

function keyed(s, c) {   // monotone cubic through the keys (as the slam)
  const K = KEYS, n = K.length;
  if (s <= K[0][0]) return K[0][c];
  if (s >= K[n - 1][0]) return K[n - 1][c];
  let i = 0; while (K[i + 1][0] < s) i++;
  const h = k => K[k + 1][0] - K[k][0], d = k => (K[k + 1][c] - K[k][c]) / h(k);
  const m = k => { if (k === 0 || k === n - 1) return 0; const a = d(k - 1), b = d(k); return a * b <= 0 ? 0 : 3 * (h(k - 1) + h(k)) / ((2 * h(k) + h(k - 1)) / a + (h(k) + 2 * h(k - 1)) / b); };
  const u = (s - K[i][0]) / h(i), u2 = u * u, u3 = u2 * u;
  return (2 * u3 - 3 * u2 + 1) * K[i][c] + (u3 - 2 * u2 + u) * h(i) * m(i) + (-2 * u3 + 3 * u2) * K[i + 1][c] + (u3 - u2) * h(i) * m(i + 1);
}

// ------------------------------------------------------------------ x-ray ghost of the shell
// Tinted glass rather than the install's additive glow (on a daylit street an additive ghost washes out). Body-frame
// scan plane (the car is not axis aligned here): ghost where (z_body - uScan) * uDir > 0.
const GHOST_VS = /* glsl */`
varying vec3 vW; varying vec3 vN; varying vec3 vB;
void main() {
  vB = position;
  vec4 w = modelMatrix * vec4(position, 1.0);
  vW = w.xyz; vN = normalize(mat3(modelMatrix) * normal);
  gl_Position = projectionMatrix * viewMatrix * w;
}`;
const GHOST_FS = /* glsl */`
uniform vec3 uTint, uRim, uEdge; uniform float uAmt, uFill, uScan, uDir, uBand, uLines, uRimK;
varying vec3 vW; varying vec3 vN; varying vec3 vB;
void main() {
  float s = (vB.z - uScan) * uDir;
  if (s < 0.0) discard;
  vec3 V = normalize(cameraPosition - vW);
  float f = 1.0 - abs(dot(normalize(vN), V));
  float rim = pow(f, 2.2);
  float ln = 0.82 + 0.18 * smoothstep(0.3, 0.5, abs(fract(vB.y * uLines) - 0.5));
  float band = exp(-s * s / (uBand * uBand));
#if MODE == 1
  gl_FragColor = vec4(uTint * ln, uFill * ln * uAmt);   // the tinted fill alone (front-most surface only, see buildGhosts)
#else
  float a = clamp((MODE == 2 ? 0.0 : uFill) * ln + rim * uRimK * ln + band * 0.9, 0.0, 0.95) * uAmt;
  vec3 c = mix(uTint, uRim, clamp(rim * 1.4, 0.0, 1.0)) * ln + uEdge * band * 2.5;
  gl_FragColor = vec4(c, a);
#endif
}`;
const KEEP = /^(Paint|Glass|Trim|Rubber|Wheel|Fascia|Headlamp • (black|bright)|Optics • (deep red|ruby)|Mirror|Underfloor)/;

export default class ValvesProgram extends SlamProgram {
  static meta = { id: 'valves', title: '02 冷气推进' };

  async init() {
    await super.init();
    const { stage } = this.ctx;
    this.tF = this.simT(S_FREEZE);
    this.setScene(this.tF);
    this.A = this.anchorAt(S_FREEZE);
    // the slam's pose at the freeze, as the first camera key (same rig parameters; its lens basis differs only in
    // the `tgt` used for the aperture offsets)
    const p0 = (this.pose0 = super.evaluate(S_FREEZE)), rel = p0.pos.clone().sub(this.A);
    KEYS[0] = [TL.hand, Math.atan2(rel.dot(this.r), -rel.dot(this.f)) * 180 / Math.PI, Math.hypot(rel.dot(this.r), rel.dot(this.f)), rel.y, p0.focus.y - this.A.y, p0.fov, 0];
    // rigid frame of the body at the freeze (rest -> world), fitted to all body nodes
    const car = this.car, rest = car.nodes.rest, X = this.view.player.pos, body = [];
    for (const ch of car.chunks) if (ch.piece === 'body') for (let n = ch.nodeStart; n < ch.nodeStart + ch.nodeCount; n++) body.push(n);
    const rc = [0, 0, 0]; for (const n of body) for (let a = 0; a < 3; a++) rc[a] += rest[n * 3 + a] / body.length;
    const q = [...this.log.q(this.tF)], c = [0, 0, 0];
    fitFrame(X, rest, Int32Array.from(body), new Float64Array(body.length).fill(1), rc, q, c, 16);
    const Q = new THREE.Quaternion(q[0], q[1], q[2], q[3]), o = v3(...rc).applyQuaternion(Q);
    this.M = new THREE.Matrix4().compose(v3(c[0] - o.x, c[1] - o.y, c[2] - o.z), Q, v3(1, 1, 1));
    this.Q = Q;
    this.buildGhosts();
    this.buildDrop();
    stage.renderer.localClippingEnabled = true;
    this.clip = new THREE.Plane(v3(0, 0, 1), 1e3);
    for (const m of this.view.carView.materials) { m.clippingPlanes = [this.clip]; m.clipShadows = false; }
    for (const n of this.kit.nozzles) n.bandMat.emissive.copy(GOLD);
    for (const h of this.kit.hoses) h.pulse.material.uniforms.uColor.value.copy(GOLD);
    this.film = {
      total: TL.total,
      render: (T, o) => this.renderV(T, o),
      marks: T => this.marksV(T),
      describe: () => ({ total: TL.total, sFreeze: S_FREEZE, tFreeze: +this.tF.toFixed(4), TL }),
    };
  }

  // ------------------------------------------------------------------ ghosts
  /** The shell's meshes in rest pose, once per pass: [material, renderOrder] pairs. */
  ghostMeshes(passes) {
    const cv = this.view.carView, car = this.car, R = car.nodes.rest, TN = car.render.tetNodes;
    const g = new THREE.Group(); g.matrixAutoUpdate = false;
    this._geo ||= new Map();
    for (const m of cv.group.children) {
      if (!KEEP.test(m.name)) continue;
      if (!this._geo.has(m)) {   // rest-pose positions through the embedding (as install/xray.js)
        const src = m.geometry, tet = src.attributes.aTet.array, bar = src.attributes.aBary.array, n = tet.length, pos = new Float32Array(n * 3);
        for (let i = 0; i < n; i++) {
          const t = Math.round(tet[i]) * 4, b1 = bar[i * 3], b2 = bar[i * 3 + 1], b3 = bar[i * 3 + 2], b0 = 1 - b1 - b2 - b3;
          const a = TN[t] * 3, b = TN[t + 1] * 3, c = TN[t + 2] * 3, d = TN[t + 3] * 3;
          for (let k = 0; k < 3; k++) pos[i * 3 + k] = R[a + k] * b0 + R[b + k] * b1 + R[c + k] * b2 + R[d + k] * b3;
          pos[i * 3 + 1] = Math.max(pos[i * 3 + 1], 0.002);
        }
        const geo = new THREE.BufferGeometry();
        geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
        geo.setAttribute('normal', src.attributes.normal);
        geo.setIndex(src.index);
        this._geo.set(m, geo);
      }
      for (const [mat, order] of passes) {
        const mesh = new THREE.Mesh(this._geo.get(m), mat);
        mesh.frustumCulled = false; mesh.userData.noAO = true; mesh.renderOrder = order;
        g.add(mesh);
      }
    }
    this.ctx.stage.scene.add(g);
    return g;
  }
  ghostMat(o, mode = 0, uniforms = null) {
    const u = uniforms || {
      uTint: { value: new THREE.Color(o.tint) }, uRim: { value: new THREE.Color(o.rim) }, uEdge: { value: new THREE.Color(0x9fe8d0) },
      uAmt: { value: 1 }, uFill: { value: o.fill }, uRimK: { value: o.rimK }, uScan: { value: -10 }, uDir: { value: -1 }, uBand: { value: 0.03 }, uLines: { value: 90 },
    };
    return new THREE.ShaderMaterial({ uniforms: u, defines: { MODE: mode }, vertexShader: GHOST_VS, fragmentShader: GHOST_FS, transparent: true, depthWrite: false, side: THREE.DoubleSide, fog: false });
  }
  buildGhosts() {
    this.xmat = this.ghostMat({ tint: 0x10262c, rim: 0x8fe6ff, fill: num('gfill', 0.22), rimK: num('grim', 0.75) });
    this.ghost = this.ghostMeshes([[this.xmat, 3]]); this.ghost.matrix.copy(this.M);
    // the drop ghost: mint (the car's colour), no scan. Its dark fill must not stack: seen head-on, the tail, floor and
    // nose panels line up and their fills summed to an opaque black patch next to the see-through corners. So the rims
    // and scan lines of every layer first, then a depth-only pass, then the fill once on the front-most surface
    // (after the x-ray ghost, which it must not hide)
    const rim = this.ghostMat({ tint: 0x16302a, rim: 0xb8f0d8, fill: 0.2, rimK: 0.85 }, 2);
    this.dmat = rim; this.dmat.uniforms.uScan.value = 1e3;
    const depth = this.ghostMat(null, 0, rim.uniforms), fill = this.ghostMat(null, 1, rim.uniforms);
    depth.colorWrite = false; depth.depthWrite = true;   // same vertex shader as the fill (no z-fighting), transparent (in order)
    this.drop = this.ghostMeshes([[rim, 5], [depth, 6], [fill, 7]]); this.drop.visible = false;
  }

  /** Scan plane at body z = scan; dir -1: ghost behind it (z < scan), the solid shell in front of it. */
  setScan(scan, dir = -1) {
    const u = this.xmat.uniforms; u.uScan.value = scan; u.uDir.value = dir;
    this.clip.set(v3(0, 0, -dir), scan * dir).applyMatrix4(this.M);   // solid keeps (z - scan)·dir <= 0
    const ghostAll = dir < 0 ? scan > 2.0 : scan < -2.0, solidAll = dir < 0 ? scan < -2.0 : scan > 2.0;
    this.ghost.visible = !solidAll;
    this.view.carView.group.visible = !ghostAll;
  }

  // ------------------------------------------------------------------ the drop (goal): edge -> the street, slowing, levelling
  buildDrop() {
    const p0 = v3().setFromMatrixPosition(this.M);   // rest origin (ground under the car centre) in the world
    const L = this.log.p(this.tTouch);               // the slam's touchdown: 01's crash site
    const fwd = v3(0, 0, 1).applyQuaternion(this.Q); fwd.y = 0; fwd.normalize();
    const k = num('dk', 0.45);                        // part of the way out to the crash site (open street, frames well)
    this.dropP0 = p0; this.dropP1 = v3(p0.x + (L[0] - p0.x) * k, 0, p0.z + (L[2] - p0.z) * k);
    this.fwd = fwd;
    this.dropQ1 = new THREE.Quaternion().setFromUnitVectors(v3(0, 0, 1), fwd);
    // tilted on the way down (rolled towards the outer side, nose down a little), level at the touchdown
    this.dropTilt = new THREE.Quaternion().setFromEuler(new THREE.Euler(num('dpitch', 0.14), 0, num('droll', -0.30), 'YXZ'));
    // vertical profile over tau 0..1: accelerates off the edge, brakes hard, settles slowly (sink speed: peak
    // ~0.25, small from ~0.6 on); horizontal: constant speed
    const n = 400, v = [], y = [0];
    for (let i = 0; i <= n; i++) { const t = i / n; v.push(t < 0.22 ? smooth(t / 0.22) : 0.12 + 0.88 * (1 - smooth((t - 0.22) / 0.4))); }
    for (let i = 1; i <= n; i++) y.push(y[i - 1] + (v[i] + v[i - 1]) / 2 / n);
    this.dropY = y.map(a => a / y[n]); this.dropV = v;
  }
  /** Drop ghost pose at segment time T: {tau, pos (rest origin), q, sink (0..1 of its peak), tilt (0..1)}. */
  dropAt(T) {
    const tau = rng(T, TL.drop[0], TL.drop[1]), i = Math.min(399, Math.floor(tau * 400)), a = tau * 400 - i;
    const fy = this.dropY[i] * (1 - a) + this.dropY[i + 1] * a, sink = tau >= 1 ? 0 : this.dropV[i] * (1 - a) + this.dropV[i + 1] * a;
    const pos = this.dropP0.clone().lerp(this.dropP1, tau); pos.y = this.dropP0.y * (1 - fy);
    const tilt = 1 - smooth(rng(T, TL.level[0], TL.level[1]));
    // leave the edge in the frozen car's own attitude, tip into the tilted pose, level out from TL.level
    const qT = this.dropQ1.clone().multiply(this.dropTilt);
    const q = this.Q.clone().slerp(qT, smooth(tau / 0.3)).slerp(this.dropQ1, 1 - tilt);
    return { tau, pos, q, sink, tilt };
  }

  // ------------------------------------------------------------------ valves
  /** Status-ring glow and feed-line pulse of nozzle i at T: lights at its turn, dims on "全关", flares on "全开". */
  valveFx(i, T) {
    const tl = TL.lit + i * TL.dl;
    let ring = T < tl ? 0 : 0.7 + 2.2 * Math.exp(-(T - tl) * 7);
    ring *= 1 - 0.85 * smooth(rng(T, TL.off[0], TL.off[0] + 0.12)) * (1 - smooth(rng(T, TL.on[0], TL.on[0] + 0.08)));
    if (T > TL.on[0]) ring += 2.6 * smooth(rng(T, TL.on[0], TL.on[0] + 0.1)) * (1 - 0.7 * smooth(rng(T, TL.on[1] + 0.1, TL.on[1] + 0.6)));
    // pulse: manifold (uP 1) -> nozzle (uP 0) over 0.22 s, arriving as the ring lights; all together on "全开"
    const pulse = (t1, k) => { const u = rng(T, t1 - 0.22, t1 + 0.04); return u > 0 && u < 1 ? { p: 1 - u * 1.08, amt: k } : null; };
    return { ring, pulse: pulse(tl, 1.4) || pulse(TL.on[0] + 0.04, 1.8) };
  }

  // ------------------------------------------------------------------ frame
  evaluateV(T) {
    if (T <= TL.hand) return { ...this.pose0, t: this.tF };
    if (T >= TL.cut && !P.has('pose')) return this.dropCam(T);
    let az, d, h, fy, fov, fs;
    if (P.has('pose')) [az, d, h, fy, fov, fs] = P.get('pose').split(',').map(Number);
    else [az, d, h, fy, fov, fs] = [1, 2, 3, 4, 5, 6].map(c => keyed(T, c));
    const A = this.A, a = az * Math.PI / 180;
    const dir = this.f.clone().multiplyScalar(-Math.cos(a)).addScaledVector(this.r, Math.sin(a));
    const right = v3(dir.z, 0, -dir.x);   // camera right (looking along -dir)
    const pos = A.clone().addScaledVector(dir, d).add(v3(0, h, 0));
    const focus = A.clone().add(v3(0, fy, 0)).addScaledVector(right, fs || 0);
    return { t: this.tF, pos, tgt: focus, focus, fov, aperture: 0.002, shake: 0, roll: 0 };
  }

  /** After the cut: on the street ahead of the landing spot, looking back up at the edge. It opens on a long lens on the
   *  car (the ghost about to leave it fills a good part of the frame), tracks the ghost down a beat behind it (it leads
   *  downwards in frame) and pushes in while the lens widens, so that it lands big in frame.
   *  dc0 (at the cut) = ahead, lateral (outer side), height, fov; dc1 (at the touchdown) = ahead, lateral, height,
   *  focus lift over the ghost's centre, fov; lag = how far (s) the tracking runs behind the ghost. */
  dropCam(T) {
    const a = (P.get('dc0') || '15,1.5,1.8,21').split(',').map(Number), b = (P.get('dc1') || '8.2,1.1,1.45,0.12,33').split(',').map(Number);
    const L = this.dropP1, side = v3(this.fwd.z, 0, -this.fwd.x);
    const u = smooth(rng(T, TL.cut + num('dcp0', 0.25), TL.drop[1] + num('dcp1', 0.25)));   // push in, lens widening
    const lerp = (x, y) => x + (y - x) * u;
    const pos = L.clone().addScaledVector(this.fwd, lerp(a[0], b[0])).addScaledVector(side, lerp(a[1], b[1])); pos.y = lerp(a[2], b[2]);
    const D = this.dropAt(T - num('lag', 0.14)), gc = v3(0, 0.62 + b[3] * u, 0).applyQuaternion(D.q).add(D.pos);
    return { t: this.tF, pos, tgt: gc, focus: gc, fov: lerp(a[3], b[4]), aperture: 0.002, shake: 0, roll: 0 };
  }

  pose(T) {
    const x = P.has('x') ? +P.get('x') : smoother(rng(T, TL.scan[0], TL.scan[1]));
    this.setScan(-2.1 + 4.2 * x, -1);
    // bottles: a cool pressure front runs along them with the scan, then a faint glow stays
    this.kit.setTank(smooth(rng(T, TL.scan[0] + 0.2, TL.scan[1] + 0.3)), 0.8 * smooth(rng(T, TL.scan[0], TL.scan[0] + 0.3)));
    this.kit.nozzles.forEach((n, i) => {
      const fx = this.valveFx(i, T);
      n.bandMat.emissiveIntensity = fx.ring;
      this.kit.setHose(i, 1, fx.pulse ? fx.pulse.p : -1, fx.pulse ? fx.pulse.amt : 0, 0.07);
    });
    const D = this.dropAt(T);
    this.drop.visible = T > TL.drop[0] - 0.05;
    this.dmat.uniforms.uAmt.value = smooth(rng(T, TL.drop[0] - 0.05, TL.drop[0] + 0.25));
    this.drop.matrix.compose(D.pos, D.q, v3(1, 1, 1));
  }

  renderV(T, { samples = 16, shutter = 0.5 / 30 } = {}) {
    this.pose(T);
    if (P.get('only') === 'drop') this.onlyDrop();
    renderFilmFrame(this.ctx.stage, T, x => this.evaluateV(x), t => this.setScene(t), { samples, shutter, aperture: 0.002 });
  }

  /** only=drop: the drop ghost alone on black (no sky, fog, grain or vignette), an additive layer for 03 (the street
   *  dissolves to the graphics board, the ghost stays). */
  onlyDrop() {
    const st = this.ctx.stage;
    for (const o of st.scene.children) if (!o.isLight) o.visible = o === this.drop;
    st.scene.background = new THREE.Color(0); st.scene.fog = null;
    for (const k of ['uVignette', 'uGrain', 'uAberration']) st.finish.uniforms[k].value = 0;
    if (st.bloom) st.bloom.strength = 0;
  }

  marksV(T) {
    const ev = this.evaluateV(T), el = this.ctx.stage.renderer.domElement, cam = (this._mc ||= new THREE.PerspectiveCamera());
    cam.aspect = el.width / el.height; cam.fov = ev.fov; cam.near = 0.05; cam.far = 4000;
    cam.position.copy(ev.pos); cam.lookAt(ev.focus); cam.updateMatrixWorld(); cam.updateProjectionMatrix();
    const scr = p => { const v = p.clone().project(cam); return [+((v.x + 1) / 2 * el.width).toFixed(1), +((1 - v.y) / 2 * el.height).toFixed(1), v.z < 1 ? 1 : 0]; };
    const D = this.dropAt(T), DM = new THREE.Matrix4().compose(D.pos, D.q, v3(1, 1, 1));
    const body = (x, y, z) => scr(v3(x, y, z).applyMatrix4(DM));
    const path = [];
    for (let k = 0; k <= 40; k++) { const d = this.dropAt(TL.drop[0] + (TL.drop[1] - TL.drop[0]) * k / 40); path.push(scr(v3(0, 0.62, 0).applyQuaternion(d.q).add(d.pos))); }
    return {
      T, nozzles: PROPULSION.nozzles.map(z => scr(v3(...z.exit).applyMatrix4(this.M))), car: scr(v3(0, 0.62, 0).applyMatrix4(this.M)),
      // 0.35 m down each exhaust (the direction of a jet on screen), the bottle bank's centre
      jets: PROPULSION.nozzles.map(z => scr(v3(...z.exit).addScaledVector(v3(...z.exhaust), 0.35).applyMatrix4(this.M))),
      bank: scr(v3(0, 0.62, -0.78).applyMatrix4(this.M)),
      drop: { tau: +D.tau.toFixed(4), sink: +D.sink.toFixed(4), tilt: +D.tilt.toFixed(4), c: body(0, 0.62, 0), bottom: body(0, 0.05, 0), left: body(-0.75, 0.62, 0), right: body(0.75, 0.62, 0),
        ground: scr(v3(D.pos.x, 0, D.pos.z)), land: scr(this.dropP1.clone()), path },
    };
  }
}
