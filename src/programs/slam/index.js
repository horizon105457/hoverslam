// Program "slam" (直到用计算机模拟 段落/01_砸地): the kit car of the opening's drive-in leaves the viaduct gap with the
// thrusters off and slams into the street. Soft-body recording public/slam/flight-slam-<run>.sim:
//   node tools/sim/flight.mjs launch=<the drive's trajectory> controller=zero spin=0,0,0 out=slam-free outDir=public/slam
// (run=free: no kerb kick, the car just drives off the edge; run=kick: the launch's own kick, it lands on its roof).
// The shot picks up on the frame after the opening's last one: the chase rig of opening/drive.js, with the soft car
// put where the drive's rigid car was. Over the edge time ramps down into bullet time while the camera swings out to
// the car's outer side and down (the drop to the street opens up below it), ramps back up to real time for the fall
// so that the touchdown lands on film time `simp`, and stays with the wreck as it slides to rest.
//   node tools/film/film.mjs program=slam out=../段落/01_砸地/renders/slam_1080p30.mp4 w=1920 h=1080 fps=30 samples=16 marks=1
//   (overlay points only, after changing marks(): node tools/film/marks.mjs program=slam out=<same .mp4 path> to=6.5)
// film.marks(T): screen points for the compositor (段落/01_砸地/tools/compose.py): the deck edge's height line, the car.
// Look-dev: devt=<sim s> freezes sim time; every ramp / camera key below is a query parameter.
import * as THREE from 'three';
import { Program } from '../../app/program.js';
import { PlaybackView } from '../../playback/view.js';
import { FlightLog } from '../flight/timeline.js';
import { Kit } from '../install/kit.js';
import { renderFilmFrame } from '../../cinema/film.js';
import { applyPose } from '../../cinema/sequence.js';
import { DECK } from '../../scene/layout.js';
import { unpackRecording } from '../../softbody/recording.js';
import { fitFrame } from '../../softbody/actuators.js';
import { smoother } from '../../cinema/ease.js';

const v3 = (x, y, z) => new THREE.Vector3(x, y, z);
const P = new URLSearchParams(location.search);
const num = (k, d) => (P.has(k) ? +P.get(k) : d);

// sim time of the drive's rigid car on the frame after the opening's last one: opening/drive.js t = t0 + 1.15 T,
// t0 = 0.5 - 4.2 * 1.15; compose.py plays drive T = DRIVE_T0 + (t - B4) = 0.30 + (637 / 30 - 17.387)
const T_CUT = num('tcut', 0.4383);

// ------------------------------------------------------------------ speed ramp: film s -> sim t
// The car keeps the drive's on-screen speed (F0, set in init) over the edge until A, eases into bullet time (SLOW) by
// A1, holds it, and from B eases back up to real time (FAST) over UP; B is solved so that the touchdown is on SIMP.
const RAMP = { A: num('ra', 0.25), A1: num('ra1', 0.5), SLOW: num('slow', 0.1), UP: num('rup', 0.4), FAST: num('fast', 1.0), SIMP: num('simp', 3.24), total: num('total', 6.6) };
const DT = 1 / 2000;

// ------------------------------------------------------------------ camera keys (film s)
// Rig around the car's centre of mass A (smoothed after the touchdown) in the launch heading frame: azimuth 0 =
// behind, 90 = its outer side (+z, away from the viaduct), distance d, height h over A, focus fy over A. The first key
// is the drive's chase pose (4.2 back, 1.15 up, focus 0.6 up, fov 60).
const KEYS = [
  // s,    az,  d,    h,     fy,    fov
  [0.00,   0,   4.2,  1.15,  0.6,   60],
  [0.22,   0,   4.4,  1.15,  0.5,   60],
  [1.30,  64,  10.0, -1.4,  -2.8,   50],
  [2.30,  92,  13.5, -3.0,  -4.3,   48],   // the street under the deck edge in frame from ~1.6 (the height line's foot)
  [3.24, 118,   9.5,  0.8,  -0.2,   46],
  [4.60, 138,   7.6,  0.8,   0.0,   44],
  [6.60, 150,   7.0,  0.8,   0.0,   42],
];
if (P.has('keys')) KEYS.splice(0, KEYS.length, ...JSON.parse(P.get('keys')));   // look-dev: the whole table as JSON

/** Monotone cubic (Fritsch-Carlson) through the keys, column c: smooth moves through the keys, no overshoot. */
function keyed(s, c) {
  const K = KEYS, n = K.length;
  if (s <= K[0][0]) return K[0][c];
  if (s >= K[n - 1][0]) return K[n - 1][c];
  let i = 0; while (K[i + 1][0] < s) i++;
  const h = k => K[k + 1][0] - K[k][0], d = k => (K[k + 1][c] - K[k][c]) / h(k);
  const m = k => { if (k === 0 || k === n - 1) return 0; const a = d(k - 1), b = d(k); return a * b <= 0 ? 0 : 3 * (h(k - 1) + h(k)) / ((2 * h(k) + h(k - 1)) / a + (h(k) + 2 * h(k - 1)) / b); };
  const u = (s - K[i][0]) / h(i), u2 = u * u, u3 = u2 * u;
  return (2 * u3 - 3 * u2 + 1) * K[i][c] + (u3 - 2 * u2 + u) * h(i) * m(i) + (-2 * u3 + 3 * u2) * K[i + 1][c] + (u3 - u2) * h(i) * m(i + 1);
}

export default class SlamProgram extends Program {
  static meta = { id: 'slam', title: '01 砸地' };

  async init() {
    const { stage, ui, assets } = this.ctx;
    ui.loadText('载入天空'); const env = await assets.sky();
    ui.loadText('搭建高架场景'); this.world = assets.world();
    ui.loadText('构建软体晶格'); const car = (this.car = await assets.car());
    ui.loadText('载入软体记录');
    const run = P.get('run') || 'free';
    const res = await fetch(`/slam/flight-slam-${run}.sim`);
    if (!res.ok || res.headers.get('content-type')?.includes('text/html')) throw new Error(`no recording public/slam/flight-slam-${run}.sim`);
    const { meta, frames } = unpackRecording(await new Response(res.body.pipeThrough(new DecompressionStream('gzip'))).arrayBuffer());
    // the drive's car: the fastest successful launch of the ghost ensemble (same pick as opening/drive.js)
    const all = await fetch('/opening/ghosts.json').then(r => r.json());
    const twin = new FlightLog(all.filter(d => d.success).sort((a, b) => Math.hypot(b.state[0][7], b.state[0][9]) - Math.hypot(a.state[0][7], a.state[0][9]))[0]);
    await this.world.ready;

    this.view = new PlaybackView(this.ctx, { car, world: this.world, env });
    const player = this.view.setRecording(meta, frames);
    const L = (this.log = new FlightLog(meta.propulsion, { flight: meta.flight, duration: player.duration }));
    const ev = t => meta.flight.events.find(e => e.type === t)?.t;
    this.tEdge = ev('edge'); this.tTouch = ev('touchdown');

    // launch heading (the drive camera's lagged heading on the straight run) and the car's outer side
    const vh = L.v(this.tEdge - 0.3);
    this.f = v3(vh[0], 0, vh[2]).normalize(); this.r = v3(this.f.z, 0, -this.f.x);
    // start where the drive's car was on the cut (the soft car rolls a little slower on the deck: same place, and the
    // same on-screen speed until the ramp)
    const pt = v3(...twin.p(T_CUT)), along = t => v3(...L.p(t)).sub(pt).dot(this.f);
    let lo = 0, hi = this.tEdge;
    for (let k = 0; k < 50; k++) { const m = (lo + hi) / 2; if (along(m) < 0) lo = m; else hi = m; }
    this.t0 = lo;
    const vT = twin.v(T_CUT), vS = L.v(this.t0);
    this.F0 = 1.15 * Math.hypot(vT[0], vT[2]) / Math.hypot(vS[0], vS[2]);
    this.buildRamp();
    this.buildAnchor();

    // kit: the nozzle modules ride their mount clusters (the thrust log's node sets, as the plumes do); the bottle
    // bank, manifold and lines ride one frame fitted to the body nodes around the bank
    const kit = (this.kit = new Kit({ heroId: 'R_UR' }));
    const rest = car.nodes.rest, mounts = meta.propulsion.mounts;
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
    this.mat = new THREE.Matrix4(); this.qq = new THREE.Quaternion();

    // reflections: on the deck where the drive's probe was (half way along its run), on the street where it lands
    const mid = twin.p(-4.33 + 4.2 * 1.15 * 0.5), land = L.p(this.tTouch);
    const hide = this.view.probeHidden().concat(this.parts.map(p => p.g));
    this.view.probes = [stage.captureProbe(v3(mid[0], DECK.top + 1.0, mid[2]), hide), stage.captureProbe(v3(land[0], 0.9, land[2]), hide)];
    this.view._probe = -1; this.view.pickProbe(DECK.top);
    stage.bloom.strength = num('bloom', 0.14);   // as the drive (a sun glint on the mirror glass blooms into a lamp)
    this.wind = this.world.group.userData.wind;

    const self = this;
    this.film = {
      total: RAMP.total,
      render: (T, o) => this.render(T, o),
      marks: T => this.marks(T),
      describe: () => ({ total: RAMP.total, run, t0: +self.t0.toFixed(4), F0: +self.F0.toFixed(3), tEdge: self.tEdge, tTouch: self.tTouch,
        // film times: COM over the edge, into bullet time, back up to real time, touchdown
        sEdge: +self.sOf(self.tEdge).toFixed(3), slow: [RAMP.A, RAMP.A1], up: [+self.B.toFixed(3), +(self.B + RAMP.UP).toFixed(3)], sTouch: +self.sOf(self.tTouch).toFixed(3) }),
    };
    this.T = +(P.get('T') || 0);
  }

  // ------------------------------------------------------------------ time
  buildRamp() {
    const { A, A1, SLOW, UP, FAST, SIMP, total } = RAMP, F0 = this.F0;
    const speed = (s, B) => s < A ? F0 : s < A1 ? F0 + (SLOW - F0) * smoother((s - A) / (A1 - A)) : s < B ? SLOW : FAST + (SLOW - FAST) * (1 - smoother((s - B) / UP));
    const table = B => {
      const n = Math.ceil((total + 1) / DT), a = new Float64Array(n + 1); a[0] = this.t0;
      for (let i = 1; i <= n; i++) a[i] = a[i - 1] + speed((i - 0.5) * DT, B) * DT;
      return a;
    };
    let lo = A1, hi = SIMP;   // later B -> less sim time by SIMP
    for (let k = 0; k < 40; k++) { const m = (lo + hi) / 2; if (table(m)[Math.round(SIMP / DT)] > this.tTouch) lo = m; else hi = m; }
    this.B = (lo + hi) / 2; this.tab = table(this.B);
  }
  simT(s) { const f = Math.max(0, s) / DT, i = Math.min(this.tab.length - 2, Math.floor(f)), a = f - i; return this.tab[i] * (1 - a) + this.tab[i + 1] * a; }
  sOf(t) { let i = 0; while (i < this.tab.length - 1 && this.tab[i + 1] < t) i++; return i * DT; }

  /** Camera anchor per film time: the centre of mass, smoothed (Gaussian in film time) once the car hits the street. */
  buildAnchor() {
    const h = 1 / 240, n = Math.ceil((RAMP.total + 0.5) / h), raw = [], sTouch = this.sOf(this.tTouch);
    for (let i = 0; i <= n; i++) raw.push(this.log.p(this.simT(i * h)));
    const sig = s => 0.25 * smoother((s - sTouch + 0.05) / 0.5);
    this.anchor = raw.map((_, i) => {
      const sg = sig(i * h);
      if (sg < 1e-3) return raw[i];
      const w = Math.ceil(3 * sg / h), out = [0, 0, 0]; let W = 0;
      for (let j = Math.max(0, i - w); j <= Math.min(n, i + w); j++) { const k = Math.exp(-0.5 * ((j - i) * h / sg) ** 2); W += k; for (let a = 0; a < 3; a++) out[a] += k * raw[j][a]; }
      return out.map(v => v / W);
    });
    this.anchorH = h;
  }
  anchorAt(s) {
    const f = Math.max(0, s) / this.anchorH, i = Math.min(this.anchor.length - 2, Math.floor(f)), a = f - i, A = this.anchor;
    return v3(A[i][0] * (1 - a) + A[i + 1][0] * a, A[i][1] * (1 - a) + A[i + 1][1] * a, A[i][2] * (1 - a) + A[i + 1][2] * a);
  }

  // ------------------------------------------------------------------ camera
  evaluate(s) {
    const t = P.has('devt') ? +P.get('devt') : this.simT(s);
    const A = this.anchorAt(s), az = keyed(s, 1) * Math.PI / 180, d = keyed(s, 2);
    const dir = this.f.clone().multiplyScalar(-Math.cos(az)).addScaledVector(this.r, Math.sin(az));
    const pos = A.clone().addScaledVector(dir, d).add(v3(0, keyed(s, 3), 0));
    const focus = A.clone().add(v3(0, keyed(s, 4), 0));
    // lens basis as the drive (its look point 9 m ahead of the car) while it is still chasing
    const tgt = focus.clone().addScaledVector(dir, -9);
    return { t, pos, tgt, focus, fov: keyed(s, 5), aperture: 0.003, shake: 0.25, roll: 0 };
  }

  // ------------------------------------------------------------------ scene at sim time t
  setScene(t, T) {
    const c = this.view.apply(t), X = this.view.player.pos, q = this.log.q(Math.max(0, t));
    // sun shadow frustum: on the deck where the drive had it, then with the car
    this.ctx.stage.focus.set(c[0], c[1] > DECK.top ? DECK.top : Math.max(0, c[1] - 0.5), c[2]);
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

  /** Screen points (pixels of the render) for the compositor, from the centre sample's camera. */
  marks(T) {
    const ev = this.evaluate(T), el = this.ctx.stage.renderer.domElement, cam = (this._mc ||= new THREE.PerspectiveCamera());
    cam.aspect = el.width / el.height; cam.fov = ev.fov; cam.near = 0.1; cam.far = 4000;
    cam.position.copy(ev.pos); cam.lookAt(ev.focus); cam.updateMatrixWorld(); cam.updateProjectionMatrix();
    const scr = p => { const v = p.clone().project(cam); return [+((v.x + 1) / 2 * el.width).toFixed(1), +((1 - v.y) / 2 * el.height).toFixed(1), v.z < 1 ? 1 : 0]; };
    // the height line hangs at the deck edge, travelling along it `dimx` m behind the car (to its right on screen)
    const x = this.log.p(ev.t)[0] - this.f.x * num('dimx', 2.5), z = DECK.halfWidth;
    return { T, t: ev.t, flight: ev.t - this.tEdge, edgeTop: scr(v3(x, DECK.top, z)), edgeBot: scr(v3(x, 0, z)), car: scr(v3(...this.log.p(ev.t))), sTouch: this.sOf(this.tTouch) };
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
