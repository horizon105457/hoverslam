// Program "finale" (直到用计算机模拟 段落/11_结尾, "让我们浪费一下"): the pure-MPC barrel roll of the film's car
// (soft-body recording public/sims/flight-mpc-roll.sim, report experiments/reports/barrel-roll; re-recorded on the
// current kit with braking, experiments/reports/drive-brakes-steering §9) as one continuous move: a chase down the
// deck to the gap, the jets fire on the music's drop and the car jumps instead of falling, the whole 360° roll about
// its long axis in a slow-motion hang, back to real time for the descent on the jets, the touchdown and the stop.
//   node tools/film/film.mjs program=finale port=<p> q=<bgm.py --query> from=1.1 out=<file.mp4> w=1920 h=1080 fps=30 samples=16 marks=1
//   (the film's 段落/11_结尾/tools/render_3d.sh does exactly this)
// Film time T = the segment's time (s, sample 0 = the segment's first frame), total = the segment length.
// Music sync (段落/11_结尾/tools/bgm.py writes these): bpm, drop = segment time of the drop (= the ignition), and
// tk = beat:event knots, beats after the drop where the recording reaches an event: ign (the jets fire), r<deg> (the
// unwrapped roll |φ| passes <deg>, e.g. r15 the roll starts, r180 upside down, r345 complete), td (touchdown). Sim time
// is real time before the first knot and after the last, a monotone C¹ ramp through the knots in between (default:
// the slowest stretch is the first half of the roll). Camera keys are in beats after the drop too, so a new tempo
// stretches the whole move with the music.
// Other params: total (s), keys=<JSON camera table> (look-dev), devt=<sim s> (frozen sim time), run=<flight run>.
// film.marks(T): sim time, beat, roll |φ| (the barrel-roll report's definition, held from the touchdown: 360.1°), gas used
// and the car's screen point for the compositor.
import * as THREE from 'three';
import { Program } from '../../app/program.js';
import { PlaybackView } from '../../playback/view.js';
import { FlightLog } from '../flight/timeline.js';
import { rollRaw, wrapPi } from '../../flight/mpc/objectives/roll.js';
import { Kit } from '../install/kit.js';
import { Plumes, trackFromRecording } from '../../render/plumes.js';
import { renderFilmFrame } from '../../cinema/film.js';
import { applyPose } from '../../cinema/sequence.js';
import { fitFrame } from '../../softbody/actuators.js';
import { smoother } from '../../cinema/ease.js';

const v3 = (x, y, z) => new THREE.Vector3(x, y, z);
const P = new URLSearchParams(location.search);
const num = (k, d) => (P.has(k) ? +P.get(k) : d);

const BPM = num('bpm', 120), BEAT = 60 / BPM, DROP = num('drop', 2.56), TOTAL = num('total', 10.6);
const TK = (P.get('tk') || '0:ign,1:r15,2.5:r90,4:r180,5:r270,6:r345,8:td').split(',').map(x => x.split(':')).map(([b, e]) => [+b, e]);

// ------------------------------------------------------------------ camera keys (u = beats after the drop)
// Rig around the car's centre of mass A (smoothed once it is on the street) in the launch heading frame: azimuth 0 =
// behind, 90 = its outer side (away from the viaduct), 180 = in front; distance d, height h over A, focus fy over A,
// ox: the look point moved ox m to the camera's right (the car sits left of centre; negative: right of centre).
const KEYS = [
  // u,    az,   d,    h,     fy,    fov, ox
  [-3.0,   10,   6.4,  1.00,  0.30,  52,  0],
  [-0.5,   16,   6.0,  0.70,  0.30,  52,  0],
  [ 0.6,   40,   6.6, -0.40,  0.30,  50,  0],
  [ 2.0,   85,   7.0, -0.10,  0.10,  46,  0],
  // round to the front for the second half of the roll: the car turns like a propeller and the jets fan out
  // sideways (from the outer side the bottom jets point into the lens around 250-270° and white out the frame)
  [ 3.5,  130,   7.4,  0.30,  0.00,  44,  0],
  [ 4.6,  160,   7.8,  0.50,  0.00,  42,  0],
  [ 5.6,  168,   8.2,  0.70, -0.10,  41,  0],
  [ 7.0,  160,   8.4,  0.80, -0.10,  40,  0],
  [ 8.0,  155,   8.2,  0.70,  0.00,  40,  0],
  // after the stop: a slow push and a little orbit, the car right of centre (the end card sits on the left)
  [12.0,  145,   6.6,  0.45,  0.05,  36, -1.9],
  [16.5,  134,   6.0,  0.35,  0.05,  34, -2.1],
];
if (P.has('keys')) KEYS.splice(0, KEYS.length, ...JSON.parse(P.get('keys')));
const CAM_MIN_Y = 0.35;   // never below the street (the descent pulls the rig down with the car)

/** Monotone cubic (Fritsch-Carlson) through table rows, column c over column 0, flat ends. */
function keyed(K, s, c) {
  const n = K.length;
  if (s <= K[0][0]) return K[0][c];
  if (s >= K[n - 1][0]) return K[n - 1][c];
  let i = 0; while (K[i + 1][0] < s) i++;
  const h = k => K[k + 1][0] - K[k][0], d = k => (K[k + 1][c] - K[k][c]) / h(k);
  const m = k => { if (k === 0 || k === n - 1) return 0; const a = d(k - 1), b = d(k); return a * b <= 0 ? 0 : 3 * (h(k - 1) + h(k)) / ((2 * h(k) + h(k - 1)) / a + (h(k) + 2 * h(k - 1)) / b); };
  const u = (s - K[i][0]) / h(i), u2 = u * u, u3 = u2 * u;
  return (2 * u3 - 3 * u2 + 1) * K[i][c] + (u3 - 2 * u2 + u) * h(i) * m(i) + (-2 * u3 + 3 * u2) * K[i + 1][c] + (u3 - u2) * h(i) * m(i + 1);
}

export default class FinaleProgram extends Program {
  static meta = { id: 'finale', title: '11 结尾 · 横滚 360°' };

  async init() {
    const { stage, ui, assets } = this.ctx;
    ui.loadText('载入天空'); const env = await assets.sky();
    ui.loadText('搭建高架场景'); this.world = assets.world();
    ui.loadText('构建软体晶格'); const car = (this.car = await assets.car());
    ui.loadText('载入软体记录');
    const run = P.get('run') || 'mpc-roll';
    const rec = await assets.sim(`flight-${run}`);
    if (!rec) throw new Error(`no recording public/sims/flight-${run}.sim`);
    const { meta, frames } = rec;
    await this.world.ready;

    this.view = new PlaybackView(this.ctx, { car, world: this.world, env });
    const player = this.view.setRecording(meta, frames);
    const L = (this.log = new FlightLog(meta.propulsion, { flight: meta.flight, duration: player.duration }));
    const m = (this.m = L.moments());
    this.tIgn = m.tIgn ?? m.tEdge;
    this.buildRoll(meta.flight.events.find(e => e.type === 'armed')?.t ?? m.tEdge);
    if (this.rollAt(m.tTouch) < 300) throw new Error(`flight-${run}: no full roll in the recording`);
    const roll = (this.roll = { t0: this.rollT(15), t1: this.rollT(345), tApex: m.tEdge });   // apex: highest COM in the air
    { let best = -1e9; for (let k = 0; k < L.n; k++) if (L.t[k] > m.tEdge && L.t[k] < m.tTouch && L.log.state[k][1] > best) { best = L.log.state[k][1]; roll.tApex = L.t[k]; } }
    this.tInv = this.rollT(180);
    this.gas0 = L.log.tank[0][0];

    // launch heading and the car's outer side (away from the viaduct)
    const vh = L.v(m.tEdge - 0.2);
    this.f = v3(vh[0], 0, vh[2]).normalize(); this.r = v3(-this.f.z, 0, this.f.x); if (this.r.z < 0) this.r.negate();
    this.buildRamp();
    this.buildAnchor();

    // jets: the thrust log on the nozzles' mount clusters (as the flight program)
    this.plumes = new Plumes(stage.scene, stage);
    this.plumes.setTrack(trackFromRecording(player, meta.propulsion));
    // kit hardware on the car (as 01 砸地): the nozzle modules ride their mount clusters, the bottle bank, manifold
    // and lines ride one frame fitted to the body nodes around the bank
    const kit = new Kit({ heroId: 'R_UR' });
    const rest = car.nodes.rest, mounts = meta.propulsion.mounts;
    const part = (nodes, share) => {
      const c = [0, 0, 0]; let W = 0;
      nodes.forEach((n, k) => { W += share[k]; for (let ax = 0; ax < 3; ax++) c[ax] += share[k] * rest[n * 3 + ax]; });
      const g = new THREE.Group(); g.matrixAutoUpdate = false; stage.scene.add(g);
      return { g, nodes: Int32Array.from(nodes), share: Float64Array.from(share), rc: c.map(v => v / W), q: [0, 0, 0, 1], c: [0, 0, 0] };
    };
    this.parts = kit.nozzles.map((mod, i) => { const p = part(mounts[i].nodes, mounts[i].share); p.g.add(mod.group); return p; });
    const bc = kit.bank.getCenter(new THREE.Vector3()), body = [];
    for (const ch of car.chunks) if (ch.piece === 'body') for (let n = ch.nodeStart; n < ch.nodeStart + ch.nodeCount; n++) body.push(n);
    const d2 = n => (rest[n * 3] - bc.x) ** 2 + (rest[n * 3 + 1] - bc.y) ** 2 + (rest[n * 3 + 2] - bc.z) ** 2;
    const near = body.sort((x, y) => d2(x) - d2(y)).slice(0, 160);
    const bank = part(near, near.map(() => 1));
    for (const o of [...kit.group.children]) bank.g.add(o);
    this.parts.push(bank);
    for (const p of this.parts) p.g.traverse(o => { if (o.isMesh) { o.castShadow = true; o.receiveShadow = true; } });
    this.qq = new THREE.Quaternion();

    // reflections: on the deck where it leaves, on the street where it lands (jets and kit kept out)
    this.view.extraProbeHidden = [...this.plumes.objects, ...this.parts.map(p => p.g)];
    const E = L.p(m.tEdge);
    this.view.placeProbes(E[0], v3(m.L[0], 0, m.L[2]));
    stage.bloom.strength = num('bloom', 0.14);   // as 01 / the drive
    this.wind = this.world.group.userData.wind;

    const self = this, at = t => +self.sOf(t).toFixed(3);
    this.film = {
      total: TOTAL,
      render: (T, o) => this.render(T, o),
      marks: T => this.marks(T),
      describe: () => ({ total: TOTAL, run, bpm: BPM, drop: DROP, tk: TK.map(k => k.join(':')).join(','),
        sim: { edge: m.tEdge, ignition: self.tIgn, roll0: roll.t0, apex: roll.tApex, inverted: +self.tInv.toFixed(3), roll1: roll.t1, touchdown: m.tTouch, stopped: meta.flight.events.find(e => e.type === 'stopped')?.t ?? null },
        // segment times of the same moments
        film: { edge: at(m.tEdge), ignition: at(self.tIgn), roll0: at(roll.t0), apex: at(roll.tApex), inverted: at(self.tInv), roll1: at(roll.t1), touchdown: at(m.tTouch),
          stopped: at(meta.flight.events.find(e => e.type === 'stopped')?.t ?? m.end) },
        gas: { tank: this.gas0, touchdown: meta.flight.metrics.gas_used_touchdown_kg, total: meta.flight.metrics.gas_used_kg } }),
    };
    this.T = +(P.get('T') || DROP - 1);
  }

  /** Roll φ as the barrel-roll report measures it (tools/flight/roll/metrics.mjs rollSeries, same functions as the
   *  MPC objective): the geometric angle of world up in the body cross-section from the first armed log row,
   *  unwrapped row by row; |φ| in degrees, 0 before arming, held at its touchdown value afterwards (φ_td −360.1°). */
  buildRoll(armedAt) {
    const L = this.log, n = L.n, phi = (this.phi = new Float64Array(n));
    let raw = 0, ph = 0, on = false;
    for (let k = 0; k < n; k++) {
      if (L.t[k] < armedAt - 1e-9) continue;
      const s = L.log.state[k], r = rollRaw([s[3], s[4], s[5], s[6]]);
      if (!on) { raw = ph = r; on = true; } else { ph += wrapPi(r - raw); raw = r; }
      phi[k] = Math.abs(ph) * 180 / Math.PI;
    }
  }
  rollAt(t) {
    t = Math.min(t, this.m.tTouch);
    if (t <= this.log.t[0]) return 0;
    const [k, a] = this.log.locate(t); return this.phi[k] * (1 - a) + this.phi[k + 1] * a;
  }
  /** Sim time where |φ| first passes deg (between the edge and the touchdown). */
  rollT(deg) {
    let a = this.m.tEdge, b = this.m.tTouch;
    for (let k = 0; k < 60; k++) { const c = (a + b) / 2; if (this.rollAt(c) < deg) a = c; else b = c; }
    return (a + b) / 2;
  }
  eventT(e) {
    if (e === 'ign') return this.tIgn;
    if (e === 'edge') return this.m.tEdge;
    if (e === 'td') return this.m.tTouch;
    if (e[0] === 'r') return this.rollT(+e.slice(1));
    throw new Error(`tk: unknown event ${e}`);
  }

  // ------------------------------------------------------------------ time: segment s -> sim t
  // Real time before the drop and after the touchdown; in between a cubic Hermite through (beat, sim time) knots with
  // Fritsch-Carlson slopes and the real-time slope at both ends: speed is continuous, time never runs backwards.
  buildRamp() {
    const K = TK.map(([b, e]) => [b, this.eventT(e)]);
    const n = K.length, h = k => K[k + 1][0] - K[k][0], d = k => (K[k + 1][1] - K[k][1]) / h(k);
    const M = K.map((_, k) => { if (k === 0 || k === n - 1) return BEAT; const a = d(k - 1), b = d(k); return a * b <= 0 ? 0 : 3 * (h(k - 1) + h(k)) / ((2 * h(k) + h(k - 1)) / a + (h(k) + 2 * h(k - 1)) / b); });
    for (let k = 0; k < n - 1; k++) { // keep each piece monotone (alpha² + beta² <= 9)
      const al = M[k] / d(k), be = M[k + 1] / d(k), s = al * al + be * be;
      if (s > 9) { const tau = 3 / Math.sqrt(s); M[k] = tau * al * d(k); M[k + 1] = tau * be * d(k); }
    }
    this.K = K; this.M = M;
  }
  simT(s) {
    const u = (s - DROP) / BEAT, K = this.K, M = this.M, n = K.length;
    if (u <= K[0][0]) return K[0][1] + (u - K[0][0]) * BEAT;
    if (u >= K[n - 1][0]) return K[n - 1][1] + (u - K[n - 1][0]) * BEAT;
    let i = 0; while (K[i + 1][0] < u) i++;
    const hh = K[i + 1][0] - K[i][0], x = (u - K[i][0]) / hh, x2 = x * x, x3 = x2 * x;
    return (2 * x3 - 3 * x2 + 1) * K[i][1] + (x3 - 2 * x2 + x) * hh * M[i] + (-2 * x3 + 3 * x2) * K[i + 1][1] + (x3 - x2) * hh * M[i + 1];
  }
  sOf(t) { let a = -5, b = TOTAL + 5; for (let k = 0; k < 60; k++) { const c = (a + b) / 2; if (this.simT(c) < t) a = c; else b = c; } return (a + b) / 2; }

  /** Camera anchor per segment time: the centre of mass, smoothed (Gaussian in film time) once the car is on the street. */
  buildAnchor() {
    const h = 1 / 240, s0 = -0.5, n = Math.ceil((TOTAL + 1 - s0) / h), raw = [], sTouch = this.sOf(this.m.tTouch);
    for (let i = 0; i <= n; i++) raw.push(this.log.p(this.simT(s0 + i * h)));
    const sig = s => 0.3 * smoother((s - sTouch + 0.1) / 0.6);
    this.anchor = raw.map((_, i) => {
      const sg = sig(s0 + i * h);
      if (sg < 1e-3) return raw[i];
      const w = Math.ceil(3 * sg / h), out = [0, 0, 0]; let W = 0;
      for (let j = Math.max(0, i - w); j <= Math.min(n, i + w); j++) { const k = Math.exp(-0.5 * ((j - i) * h / sg) ** 2); W += k; for (let ax = 0; ax < 3; ax++) out[ax] += k * raw[j][ax]; }
      return out.map(v => v / W);
    });
    this.anchorH = h; this.anchorS0 = s0;
  }
  anchorAt(s) {
    const f = Math.max(0, s - this.anchorS0) / this.anchorH, i = Math.min(this.anchor.length - 2, Math.floor(f)), a = f - i, A = this.anchor;
    return v3(A[i][0] * (1 - a) + A[i + 1][0] * a, A[i][1] * (1 - a) + A[i + 1][1] * a, A[i][2] * (1 - a) + A[i + 1][2] * a);
  }

  // ------------------------------------------------------------------ camera
  evaluate(s) {
    const t = P.has('devt') ? +P.get('devt') : this.simT(s), u = (s - DROP) / BEAT;
    const A = this.anchorAt(s), az = keyed(KEYS, u, 1) * Math.PI / 180, d = keyed(KEYS, u, 2);
    const dir = this.f.clone().multiplyScalar(-Math.cos(az)).addScaledVector(this.r, Math.sin(az));
    const pos = A.clone().addScaledVector(dir, d).add(v3(0, keyed(KEYS, u, 3), 0));
    pos.y = Math.max(pos.y, CAM_MIN_Y);
    const side = v3(0, 1, 0).cross(pos.clone().sub(A)).setY(0).normalize();   // the camera's right
    const focus = A.clone().add(v3(0, keyed(KEYS, u, 4), 0)).addScaledVector(side, KEYS[0].length > 6 ? keyed(KEYS, u, 6) : 0);
    return { t, pos, tgt: focus, focus, fov: keyed(KEYS, u, 5), aperture: 0.004, shake: 0, roll: 0 };
  }

  // ------------------------------------------------------------------ scene at sim time t
  setScene(t) {
    this.view.apply(t);
    const X = this.view.player.pos, q = this.log.q(Math.max(0, t));
    // big jumps in time (look-dev seeks): re-seed the nozzle mount fits from the logged body orientation
    const jump = this._lastT === undefined || Math.abs(t - this._lastT) > 0.06;
    if (jump && this.plumes.track?.fitter) this.plumes.track.fitter.seed(q);
    this.plumes.update(t, t >= 0 ? X : null, jump ? 12 : 3);
    this._lastT = t;
    for (const p of this.parts) {
      p.q.splice(0, 4, ...q);
      fitFrame(X, this.car.nodes.rest, p.nodes, p.share, p.rc, p.q, p.c, 12);
      this.qq.set(p.q[0], p.q[1], p.q[2], p.q[3]);
      const o = v3(...p.rc).applyQuaternion(this.qq);
      p.g.matrix.compose(v3(p.c[0] - o.x, p.c[1] - o.y, p.c[2] - o.z), this.qq, v3(1, 1, 1));
      p.g.matrixWorldNeedsUpdate = true;
    }
    if (this.wind) this.wind.value = t;
  }

  render(T, { samples = 16, shutter = 0.5 / 30 } = {}) {
    renderFilmFrame(this.ctx.stage, T, x => this.evaluate(x), t => this.setScene(t), { samples, shutter, aperture: 0.004 });
  }

  /** Screen points (pixels of the render) and readouts for the compositor, from the centre sample's camera. */
  marks(T) {
    const ev = this.evaluate(T), el = this.ctx.stage.renderer.domElement, cam = (this._mc ||= new THREE.PerspectiveCamera());
    cam.aspect = el.width / el.height; cam.fov = ev.fov; cam.near = 0.1; cam.far = 4000;
    cam.position.copy(ev.pos); cam.lookAt(ev.focus); cam.updateMatrixWorld(); cam.updateProjectionMatrix();
    const scr = p => { const v = p.clone().project(cam); return [+((v.x + 1) / 2 * el.width).toFixed(1), +((1 - v.y) / 2 * el.height).toFixed(1), v.z < 1 ? 1 : 0]; };
    const t = ev.t, c = v3(...this.log.p(t)), tk = this.log.tank(Math.max(0, t));
    return { T, t: +t.toFixed(4), beat: +((T - DROP) / BEAT).toFixed(3), roll: +this.rollAt(t).toFixed(2), gas: +(this.gas0 - tk.m).toFixed(3),
      car: scr(c), carTop: scr(c.clone().add(v3(0, 1.3, 0))), cam: ev.pos.toArray().map(v => +v.toFixed(3)) };
  }

  preview(T) {
    const ev = this.evaluate(T);
    applyPose(this.ctx.stage.camera, ev);
    this.setScene(ev.t);
    this.ctx.stage.render(1 / 60);
  }

  frame(dt) {
    if (!this.film) return;
    if (!P.has('paused')) this.T = (this.T + dt) % this.film.total;
    this.preview(this.T);
  }
}
