// Program "twocost" (直到用计算机模拟 段落/04_MPC稳妥 + 05_加用气项): the two soft-body MPC flights of
// experiments/reports/gas-two-costs §11 (nose s3: the same launch, cost 1 lowers the car on its jets, cost 2 coasts
// and burns late) as designed shots: a speed ramp (film s -> sim s, monotone, so bullet time at the deck edge and
// before the burn) and a keyed camera rig, one shot per `shot=`; film.marks(T) gives the compositor the camera
// (view-projection matrix) per frame, film.describe() the flight's path and key times.
//   ?program=twocost&run=two-costs-first-nose3&shot=hold     04: one take, edge in bullet time -> real-time descent
//   ?program=twocost&run=two-costs-second-nose3&shot=launch  05b: the edge, the puff, the coast
//   ?program=twocost&run=two-costs-second-nose3&shot=nozzle  05b: under the sill, frozen before the burn, the valves slam open
//   ?program=twocost&run=two-costs-second-nose3&shot=slam    05b: low tracking camera, the brake, the touchdown
//   node tools/film/film.mjs program=twocost port=<p> q="run=...&shot=hold" out=<.mp4> w=1920 h=1080 fps=30 samples=16 marks=1
// Look-dev (all optional): ramp=[[T,t],...] (film s, sim s) and keys=[[T,...],...] (the shot's camera table, JSON)
// replace the shot's own; devt=<sim s> freezes sim time; total=<s>; nocar=1 hides the car, kit and jets (a clean plate
// of the same camera, for difference mattes); nojets=1 hides only the jets.
// Camera rigs (columns after T):
//   car   az, d, h, fy, fov, vf, ahead: around the car's centre of mass A (smoothed once it is on the street) in the
//         launch frame (az 0 = behind, 90 = the outer side, away from the viaduct; d m out, h m up), the camera height
//         following A's height by vf (0 = stays at the deck-edge height, 1 = rides with the car), looking at A + fy up,
//         `ahead` m along the launch heading and (optional 9th column) `side` m to the outer side
//   body  px, py, pz, lx, ly, lz, fov: rigidly on the car (body frame, spec/README §1: x left/right, y up, z forward;
//         camera at p, looking at l, rolling with the car)
//   world a, b, y, la, lb, ly, fov, w: fixed camera at E + a·f + b·r (+ y m above the street), E the deck-edge point,
//         f the launch heading, r the outer side; it looks at the blend (by w) of the fixed point (la, lb, ly) and
//         the car's centre of mass
import * as THREE from 'three';
import { Program } from '../../app/program.js';
import { PlaybackView } from '../../playback/view.js';
import { FlightLog } from '../flight/timeline.js';
import { Kit } from '../install/kit.js';
import { Plumes, trackFromRecording } from '../../render/plumes.js';
import { renderFilmFrame } from '../../cinema/film.js';
import { applyPose } from '../../cinema/sequence.js';
import { DECK } from '../../scene/layout.js';
import { fitFrame } from '../../softbody/actuators.js';
import { smoother } from '../../cinema/ease.js';
import { attachedRoll } from '../flight/edit.js';

const v3 = (x, y, z) => new THREE.Vector3(x, y, z);
const P = new URLSearchParams(location.search);
const num = (k, d) => (P.has(k) ? +P.get(k) : d);

// ------------------------------------------------------------------ shots
// ramp: [film s, sim s] keys (monotone cubic through them; the slope at the ends is the end segment's);
// rig + keys: the camera table. tEdge = 0.76, touchdown 4.62 (first) / 2.81 (second).
const SHOTS = {
  // 04 (cost 1, run two-costs-first-nose3), one take, film T = segment time: bullet time on the deck while the plan is
  // drawn (frame 0 = the chase at the gap), the centre of mass over the edge on "冲出" (4.62), then about 0.85x down
  // to the touchdown (sim 4.62) on "放" (9.2) and 0.4 s on the ground. Camera: from the rear quarter on the outer side
  // round behind the car as it goes over, on to the viaduct side and down with it, side-on at the end.
  hold: {
    total: 9.6, rig: 'car',
    ramp: [[0, 0.30], [3.7, 0.50], [4.62, 0.76], [5.4, 1.40], [9.2, 4.62], [9.6, 4.85]],
    keys: [
      // T,   az,  d,    h,    fy,   fov, vf,  ahead, side
      [0.0,   48,  11.0, 5.5,  -1.0, 44,  1.0, 7,     0],
      [3.4,   30,  11.5, 5.0,  -1.2, 44,  1.0, 3,     0],
      [4.62,  10,  12.5, 4.5,  -1.5, 42,  1.0, 2,     0],
      [7.0,   -40, 14.0, 3.5,  -1.0, 40,  0.5, 6,     0],
      [8.5,   -62, 11.0, 2.0,  -0.6, 40,  1.0, 3,     0],
      [9.6,   -75, 9.5,  1.5,  -0.3, 40,  1.0, 2,     0],
    ],
  },
  // 05b part 1 (cost 2, run two-costs-second-nose3), film T 0 = segment 5.46: the same gap as 04 from high on its outer
  // side (the street the two plans dive into in frame), the edge (8.86), the roll-stopping puff slowed right down on
  // "一小口气", the coast speeding up while the camera stays up at deck height and the car drops away (cut at 13.38)
  launch: {
    total: 8.1, rig: 'car',
    ramp: [[0, 0.30], [3.0, 0.72], [3.4, 0.765], [4.2, 0.83], [5.3, 1.0], [6.4, 1.25], [7.92, 1.62], [8.1, 1.655]],
    keys: [
      // T,   az,  d,    h,    fy,   fov, vf,  ahead, side
      [0.0,   78,  24.0, 7.0,  -3.0, 52,  1.0, 7,     0],
      [3.4,   55,  15.0, 4.5,  -1.5, 48,  1.0, 0,     0],
      [5.3,   35,  12.0, 4.0,  -1.0, 44,  0.5, -1,    -1.5],
      [8.1,   30,  11.0, 5.0,  -0.5, 46,  0.0, -1,    -1.0],
    ],
  },
  // 05b part 2, film T 0 = segment 13.38 ("等到落地前最后 0.7 秒"): rigidly under the car's right sill (the flight
  // program's nozzle cam, mirrored), the bottom nozzles idling over the street, the viaduct behind; almost frozen just
  // before the burn (sim 1.62 -> 1.84 at 16.2), then the valves slam open on "猛地开到最大" (sim 1.84 -> 2.02 by 16.9)
  nozzle: {
    total: 3.6, rig: 'body',
    ramp: [[0, 1.62], [1.27, 1.765], [2.0, 1.80], [2.82, 1.84], [3.52, 2.02], [3.6, 2.04]],
    keys: [
      // T,  px,   py,    pz,   lx,   ly,    lz,    fov
      [0.0,  1.45, -0.30, 1.61, 0.35, -0.88, -0.24, 72],
    ],
  },
  // 05b part 3, film T 0 = segment 16.9: a low tracking camera on the outer front quarter, the viaduct behind the car,
  // the brake on full jets, touchdown (sim 2.813) on "刹住" (T 1.99), the car rolling on (into 05c)
  slam: {
    total: 3.4, rig: 'car',
    ramp: [[0, 2.02], [1.99, 2.813], [2.7, 3.25], [4.0, 4.1]],
    keys: [
      // T,   az,  d,    h,     fy,   fov, vf,  ahead
      [0.0,   120, 7.7,  -8.95, -1.2, 50,  0.0, 0],
      [2.0,   122, 7.2,  -9.0,  -0.6, 48,  0.0, 0],
      [4.0,   126, 7.6,  -8.9,  -0.5, 46,  0.0, 0],
    ],
  },
};

/** Monotone cubic (Fritsch-Carlson) through key rows K at column c, as a function of column 0. */
function mono(K, s, c) {
  const n = K.length;
  if (n === 1) return K[0][c];
  const h = k => K[k + 1][0] - K[k][0], d = k => (K[k + 1][c] - K[k][c]) / h(k);
  const m = k => {
    if (k === 0) return d(0);
    if (k === n - 1) return d(n - 2);
    const a = d(k - 1), b = d(k); return a * b <= 0 ? 0 : 3 * (h(k - 1) + h(k)) / ((2 * h(k) + h(k - 1)) / a + (h(k) + 2 * h(k - 1)) / b);
  };
  if (s <= K[0][0]) return K[0][c] + m(0) * (s - K[0][0]);
  if (s >= K[n - 1][0]) return K[n - 1][c] + m(n - 1) * (s - K[n - 1][0]);
  let i = 0; while (K[i + 1][0] < s) i++;
  const u = (s - K[i][0]) / h(i), u2 = u * u, u3 = u2 * u;
  return (2 * u3 - 3 * u2 + 1) * K[i][c] + (u3 - 2 * u2 + u) * h(i) * m(i) + (-2 * u3 + 3 * u2) * K[i + 1][c] + (u3 - u2) * h(i) * m(i + 1);
}
/** Camera keys: flat ends (holds), monotone in between. */
function keyed(K, s, c) {
  if (s <= K[0][0]) return K[0][c];
  if (s >= K[K.length - 1][0]) return K[K.length - 1][c];
  const n = K.length, h = k => K[k + 1][0] - K[k][0], d = k => (K[k + 1][c] - K[k][c]) / h(k);
  const m = k => { if (k === 0 || k === n - 1) return 0; const a = d(k - 1), b = d(k); return a * b <= 0 ? 0 : 3 * (h(k - 1) + h(k)) / ((2 * h(k) + h(k - 1)) / a + (h(k) + 2 * h(k - 1)) / b); };
  let i = 0; while (K[i + 1][0] < s) i++;
  const u = (s - K[i][0]) / h(i), u2 = u * u, u3 = u2 * u;
  return (2 * u3 - 3 * u2 + 1) * K[i][c] + (u3 - 2 * u2 + u) * h(i) * m(i) + (-2 * u3 + 3 * u2) * K[i + 1][c] + (u3 - u2) * h(i) * m(i + 1);
}

export default class TwoCostProgram extends Program {
  static meta = { id: 'twocost', title: '04 / 05 两种代价' };

  async init() {
    const { stage, ui, assets } = this.ctx;
    const name = P.get('shot') || 'hold', S = SHOTS[name];
    if (!S) throw new Error(`twocost: unknown shot=${name} (${Object.keys(SHOTS).join(', ')})`);
    this.shot = {
      name, rig: P.get('rig') || S.rig, total: num('total', S.total),
      ramp: P.has('ramp') ? JSON.parse(P.get('ramp')) : S.ramp, keys: P.has('keys') ? JSON.parse(P.get('keys')) : S.keys,
    };
    ui.loadText('载入天空'); const env = await assets.sky();
    ui.loadText('搭建高架场景'); this.world = assets.world();
    ui.loadText('构建软体晶格'); const car = (this.car = await assets.car());
    ui.loadText('载入飞行记录');
    const run = P.get('run') || 'two-costs-first-nose3';
    const rec = await assets.sim(`flight-${run}`);
    if (!rec) throw new Error(`no recording public/sims/flight-${run}.sim`);
    const { meta, frames } = rec;
    await this.world.ready;

    this.view = new PlaybackView(this.ctx, { car, world: this.world, env });
    const player = this.view.setRecording(meta, frames);
    const L = (this.log = new FlightLog(meta.propulsion, { flight: meta.flight, duration: player.duration }));
    const mo = (this.mo = L.moments());
    this.metrics = meta.flight?.metrics || {};
    // launch frame: heading from the deck edge to the touchdown point (horizontal), outer side away from the viaduct
    const E = (this.E = v3(...L.p(mo.tEdge)));
    this.f = v3(mo.L[0] - E.x, 0, mo.L[2] - E.z).normalize();
    this.r = v3(-this.f.z, 0, this.f.x); if (this.r.z < 0) this.r.negate();
    this.yEdge = E.y;

    // jets: poses from the recording's mount clusters, thrust from the log (as the flight program)
    this.plumes = new Plumes(stage.scene, stage);
    this.view.extraProbeHidden = this.plumes.objects;
    this.plumes.setTrack(trackFromRecording(player, meta.propulsion));
    this._lastPlumeT = null;

    // kit hardware riding the mount clusters (as slam): nozzle modules and the bottle bank
    const kit = new Kit({ heroId: 'R_UR' }), rest = car.nodes.rest, mounts = meta.propulsion.mounts;
    const part = (nodes, share) => {
      const c = [0, 0, 0]; let W = 0;
      nodes.forEach((n, k) => { W += share[k]; for (let a = 0; a < 3; a++) c[a] += share[k] * rest[n * 3 + a]; });
      const g = new THREE.Group(); g.matrixAutoUpdate = false; stage.scene.add(g);
      return { g, nodes: Int32Array.from(nodes), share: Float64Array.from(share), rc: c.map(v => v / W), q: [0, 0, 0, 1], c: [0, 0, 0] };
    };
    this.parts = kit.nozzles.map((m, i) => { const p = part(mounts[i].nodes, mounts[i].share); p.g.add(m.group); return p; });
    const bc = kit.bank.getCenter(new THREE.Vector3()), body = [];
    for (const ch of car.chunks) if (ch.piece === 'body') for (let n = ch.nodeStart; n < ch.nodeStart + ch.nodeCount; n++) body.push(n);
    const d2 = n => (rest[n * 3] - bc.x) ** 2 + (rest[n * 3 + 1] - bc.y) ** 2 + (rest[n * 3 + 2] - bc.z) ** 2;
    const near = body.sort((a, b) => d2(a) - d2(b)).slice(0, 160);
    const bank = part(near, near.map(() => 1));
    for (const o of [...kit.group.children]) bank.g.add(o);
    this.parts.push(bank);
    for (const p of this.parts) p.g.traverse(o => { if (o.isMesh) { o.castShadow = true; o.receiveShadow = true; } });
    this.qq = new THREE.Quaternion();

    // reflections: on the deck before the edge, on the street where it lands
    this.view.placeProbes(E.x, v3(mo.L[0], 0, mo.L[2]));
    if (P.has('bloom')) stage.bloom.strength = +P.get('bloom');
    this.wind = this.world.group.userData.wind;
    this.hide = { car: P.get('nocar') === '1', jets: P.get('nocar') === '1' || P.get('nojets') === '1' };
    if (this.hide.car) { this.view.carView.group.visible = false; for (const p of this.parts) p.g.visible = false; }
    if (this.hide.jets) this.plumes.visible = false;

    // the first flight passes the signal gantry at the intersection's near corner within half a metre and lands on
    // the sidewalk beside it; the sim has no colliders there, so a tracking camera only gets a pole wiping through the
    // car. Clear the corner's street furniture (gantry, signal heads, guide sign, stop sign, mirror) in every shot of
    // this program (props=1 keeps them).
    if (P.get('props') !== '1') {
      const box = new THREE.Box3(), c = new THREE.Vector3();
      for (const m of this.world.group.children) {
        if (!m.isMesh || !m.geometry) continue;
        box.setFromObject(m); box.getCenter(c);
        if (c.x > -68 && c.x < -62 && c.z > 9 && c.z < 31.5 && box.max.y > 1.0 && box.max.y < 9 && box.max.x - box.min.x < 3) m.visible = false;
      }
    }

    // the hedge along the outer green belt is not a collider in the sim either: the first flight touches down on the
    // sidewalk's edge right on its line and would end up half inside it. Clear the hedge boxes where the car lands and
    // rolls on (degenerate triangles in the merged mesh; hedge=1 keeps them)
    if (P.get('hedge') !== '1') {
      const m = this.world.group.children.find(o => o.isMesh && o.material?.color?.getHex?.() === 0xbad0a0 && o.material.roughness === 0.95);
      if (m) {
        const g = m.geometry = m.geometry.clone(), X = g.attributes.position;
        for (let i = 0; i < X.count; i += 3) {
          const cx = (X.getX(i) + X.getX(i + 1) + X.getX(i + 2)) / 3, cz = (X.getZ(i) + X.getZ(i + 1) + X.getZ(i + 2)) / 3;
          if (cx > -118 && cx < -66 && cz > 29.5 && cz < 33.5) for (let k = 0; k < 3; k++) X.setXYZ(i + k, 0, -10, 0);
        }
        X.needsUpdate = true; g.computeBoundingSphere();
      }
    }

    this.buildAnchor();
    const self = this;
    this.film = {
      total: this.shot.total,
      render: (T, o) => this.render(T, o),
      marks: T => this.marks(T),
      describe: () => self.describe(),
    };
    this.T = +(P.get('T') || 0);
  }

  // ------------------------------------------------------------------ time
  simT(T) { return P.has('devt') ? +P.get('devt') : mono(this.shot.ramp, Math.max(0, T), 1); }
  /** Film time at which the ramp reaches sim time t (bisection; the ramp is monotone). */
  filmT(t) {
    let lo = -5, hi = this.shot.total + 5;
    for (let k = 0; k < 60; k++) { const m = (lo + hi) / 2; if (mono(this.shot.ramp, m, 1) < t) lo = m; else hi = m; }
    return (lo + hi) / 2;
  }

  /** Camera anchor: the centre of mass, Gaussian-smoothed in sim time once the car is on the street (bounces). */
  buildAnchor() {
    const L = this.log, h = 1 / 240, t0 = -3, t1 = L.duration, n = Math.ceil((t1 - t0) / h), tT = this.mo.tTouch;
    const raw = []; for (let i = 0; i <= n; i++) raw.push(L.p(t0 + i * h));
    const sig = t => 0.12 * smoother((t - tT + 0.05) / 0.4);
    this.anchor = raw.map((_, i) => {
      const sg = sig(t0 + i * h); if (sg < 1e-3) return raw[i];
      const w = Math.ceil(3 * sg / h), out = [0, 0, 0]; let W = 0;
      for (let j = Math.max(0, i - w); j <= Math.min(n, i + w); j++) { const k = Math.exp(-0.5 * ((j - i) * h / sg) ** 2); W += k; for (let a = 0; a < 3; a++) out[a] += k * raw[j][a]; }
      return out.map(v => v / W);
    });
    this.anchorT0 = t0; this.anchorH = h;
  }
  anchorAt(t) {
    const f = Math.max(0, (t - this.anchorT0) / this.anchorH), A = this.anchor, i = Math.min(A.length - 2, Math.floor(f)), a = Math.min(1, f - i);
    return v3(A[i][0] * (1 - a) + A[i + 1][0] * a, A[i][1] * (1 - a) + A[i + 1][1] * a, A[i][2] * (1 - a) + A[i + 1][2] * a);
  }

  // ------------------------------------------------------------------ camera
  evaluate(T) {
    const t = this.simT(T), K = this.shot.keys, k = c => keyed(K, T, c), A = this.anchorAt(t);
    if (this.shot.rig === 'body') {
      const L = this.log, pos = v3(...L.bodyPoint(t, [k(1), k(2), k(3)])), tgt = v3(...L.bodyPoint(t, [k(4), k(5), k(6)]));
      return { t, pos, tgt, focus: tgt, fov: k(7), aperture: num('ap', 0.002), shake: 0, roll: attachedRoll(pos, tgt, v3(...L.bodyDir(t, [0, 1, 0]))) };
    }
    if (this.shot.rig === 'world') {
      const E = this.E, f = this.f, r = this.r;
      const pos = v3(E.x, 0, E.z).addScaledVector(f, k(1)).addScaledVector(r, k(2)).setY(k(3));
      const fix = v3(E.x, 0, E.z).addScaledVector(f, k(4)).addScaledVector(r, k(5)).setY(k(6));
      const focus = fix.lerp(A, k(8));
      return { t, pos, tgt: focus, focus, fov: k(7), aperture: num('ap', 0.003), shake: 0, roll: 0 };
    }
    const az = k(1) * Math.PI / 180, d = k(2), vf = k(6);
    const dir = this.f.clone().multiplyScalar(-Math.cos(az)).addScaledVector(this.r, Math.sin(az));
    const base = v3(A.x, this.yEdge + (A.y - this.yEdge) * vf, A.z);
    const pos = base.addScaledVector(dir, d).add(v3(0, k(3), 0));
    const focus = A.clone().add(v3(0, k(4), 0)).addScaledVector(this.f, k(7)).addScaledVector(this.r, K[0].length > 8 ? k(8) : 0);
    return { t, pos, tgt: focus, focus, fov: k(5), aperture: num('ap', 0.003), shake: 0, roll: 0 };
  }

  // ------------------------------------------------------------------ scene at sim time t
  setScene(t) {
    const c = this.view.apply(t), X = this.view.player.pos;
    const jump = this._lastPlumeT === null || Math.abs(t - this._lastPlumeT) > 0.06;
    if (jump && this.plumes.track?.fitter) this.plumes.track.fitter.seed(this.log.q(Math.max(0, t)));
    this.plumes.update(t, t >= 0 ? X : null, jump ? 12 : 3);
    this._lastPlumeT = t;
    // sun shadow frustum: on the deck while the car is up there, then with the car
    this.ctx.stage.focus.set(c[0], c[1] > DECK.top ? DECK.top : Math.max(0, c[1] - 0.5), c[2]);
    const q = this.log.q(Math.max(0, t));
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
    renderFilmFrame(this.ctx.stage, T, x => this.evaluate(x), t => this.setScene(t), { samples, shutter, aperture: 0.003 });
  }

  /** Per frame for the compositor: sim time and the centre sample's view-projection (column-major, world -> clip). */
  marks(T) {
    const ev = this.evaluate(T), el = this.ctx.stage.renderer.domElement, cam = (this._mc ||= new THREE.PerspectiveCamera());
    cam.aspect = el.width / el.height; cam.fov = ev.fov; cam.near = 0.1; cam.far = 4000;
    cam.position.copy(ev.pos); cam.lookAt(ev.focus); if (ev.roll) cam.rotateZ(ev.roll); cam.updateMatrixWorld(); cam.updateProjectionMatrix();
    const M = new THREE.Matrix4().multiplyMatrices(cam.projectionMatrix, cam.matrixWorldInverse);
    const c = this.log.p(ev.t);
    return { T, t: +ev.t.toFixed(5), vp: M.elements.map(x => +x.toPrecision(8)), cam: ev.pos.toArray().map(x => +x.toFixed(4)), com: c.map(x => +x.toFixed(4)) };
  }

  describe() {
    const L = this.log, mo = this.mo, path = [];
    for (let t = 0; t <= L.duration + 1e-9; t += 1 / 120) { const p = L.p(t); path.push([+t.toFixed(4), ...p.map(x => +x.toFixed(4))]); }
    return {
      shot: this.shot, tEdge: mo.tEdge, tTouch: mo.tTouch, tBrake: mo.tBrake, metrics: this.metrics,
      E: this.E.toArray(), f: this.f.toArray(), r: this.r.toArray(), deck: { top: DECK.top, halfWidth: DECK.halfWidth, gap: DECK.gap },
      sEdge: +this.filmT(mo.tEdge).toFixed(4), sTouch: +this.filmT(mo.tTouch).toFixed(4), path,
    };
  }

  preview(T) {
    const ev = this.evaluate(T);
    applyPose(this.ctx.stage.camera, { ...ev, tgt: ev.focus });
    this.setScene(ev.t);
    this.ctx.stage.render(1 / 60);
  }

  frame(dt) {
    if (!this.film) return;
    if (!P.has('paused')) this.T = (this.T + dt) % this.film.total;
    this.preview(this.T);
  }
}
