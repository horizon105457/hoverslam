// Shot "ghosts": a few hundred rigid-twin launches (public/opening/ghosts.json, packed by tools/pack_ghosts.py from
// the experiment runs: every preset, zero / pd / mpc / mppi, many seeds) leave the viaduct at the same moment, drawn
// on top of each other: translucent ghost cars (each its own single translucent layer: per-car depth pre-pass, then
// colour with depthFunc <=) with faint jets, thin centre-of-mass trails, and one solid hero car with its real jets.
// For the shot every run is turned about the vertical through its deck-edge crossing so that it leaves the gap nearly
// square (`hd`, the runs themselves leave at 13-20°), and shifted in time and along the edge so that all of them cross
// at the same point and instant: one car goes through the gap firing its jets and splits into the ensemble. A turn
// about the vertical leaves the free flight exact; the gap is moved over the crossing road (`gx`) so the runs land on
// open asphalt as they did in the experiments. The camera starts close in front of the car at deck height, then pulls
// back and arcs round to its side while the ensemble fans out in slow motion.
// Film time runs through a speed ramp: fast over the edge and through the split, then bullet time to the end of the
// shot (the film cuts away to the Roadster while time still crawls).
import * as THREE from 'three';
import { Line2 } from 'three/addons/lines/Line2.js';
import { LineGeometry } from 'three/addons/lines/LineGeometry.js';
import { LineMaterial } from 'three/addons/lines/LineMaterial.js';
import { CarRenderer } from '../../render/carRender.js';
import { Plumes, trackFromTrajectory } from '../../render/plumes.js';
import { FlightLog } from '../flight/timeline.js';
import { renderFilmFrame } from '../../cinema/film.js';
import { applyPose } from '../../cinema/sequence.js';
import { DECK } from '../../scene/layout.js';
import { nozzleNominal } from '../../render/plumes.js';
import { PROPULSION } from '../../core/spec.js';
import { clamp01, smooth, smoother } from '../../cinema/ease.js';

const v3 = (x, y, z) => new THREE.Vector3(x, y, z);
const P = new URLSearchParams(location.search);
const num = (k, d) => (P.has(k) ? +P.get(k) : d);

// ------------------------------------------------------------------ launch geometry of the shot
// GX: centre of the parapet gap (moved over the crossing road, x -64..-16); HD: exit heading off the deck edge
// (0 = along the lane, 90 = square to the edge); T_EDGE: sim time at which every run crosses the deck edge.
const GX = num('gx', -40), HD = num('hd', 72) * Math.PI / 180, T_EDGE = num('tedge', 0.76);
const E = v3(-Math.cos(HD), 0, Math.sin(HD)), N = v3(Math.sin(HD), 0, Math.cos(HD)); // exit direction, its left-hand side

// ------------------------------------------------------------------ speed ramp: film T -> sim t
// speed s(T): FAST until A, eased down to SLOW by B, held to the end. t = T0 + ∫ s dT (numeric table).
// The first frame has the car's nose at the edge; it crosses at FAST speed in the first frames and the ensemble splits
// off it (the cars overlap for ~0.3 s after the edge, are ±1 m apart 0.45 s after it, ±2.5 m at 0.85 s, ±4 m at
// 1.3 s). By B the fan is open and time crawls, so it keeps opening slowly for the rest of the shot.
const RAMP = { T0: num('t0', T_EDGE - 0.08), FAST: num('fast', 2.0), A: num('ra', 0.25), B: num('rb', 0.75), SLOW: num('slow', 0.2), total: num('total', 3.3) };
function speed(T) {
  const { A, B, FAST, SLOW } = RAMP;
  if (T < A) return FAST;
  if (T < B) return FAST + (SLOW - FAST) * smoother((T - A) / (B - A));
  return SLOW;
}
// camera: follows the (smoothed) centroid of the ghosts. First frame: close in front of the car at deck height (its
// nose on the lip, coming at the lens, the street below); then back and round towards its N side (azimuth 0 = behind,
// 90 = its N side, 180 = in front) and down to a little below the cars, looking up at them as it goes down with them,
// easing out (power `ease`) over o0..o1 and still moving at the cut: the burst comes at the lens and the fan of trails
// from the gap opens up sideways. Distance eases out faster (`dease`) so the opening fan stays in frame.
const CAM = { o0: num('o0', 0), o1: num('o1', 3.3), az: [num('az0', 180), num('az1', 100)], el: [num('el0', 2), num('el1', -4)],
  d: [num('d0', 5), num('d1', 22)], fov: [num('fov0', 46), num('fov1', 38)], ease: num('ease', 1.5), dease: num('dease', 3),
  roll: num('roll', 4) };
const RAMP_DT = 1 / 2000, RAMP_TAB = (() => {
  const n = Math.ceil((RAMP.total + 1) / RAMP_DT), a = new Float64Array(n + 1);
  a[0] = RAMP.T0;
  for (let i = 1; i <= n; i++) a[i] = a[i - 1] + speed((i - 0.5) * RAMP_DT) * RAMP_DT;
  return a;
})();
export function simTime(T) {
  if (T <= 0) return RAMP.T0 + T;
  const f = T / RAMP_DT, i = Math.min(RAMP_TAB.length - 2, Math.floor(f)), a = f - i;
  return RAMP_TAB[i] * (1 - a) + RAMP_TAB[i + 1] * a;
}

/**
 * Turn every run about the vertical through its deck-edge crossing to the common exit heading (mean heading -> HD,
 * the spread between runs kept), move the crossing to (GX, ·, edge) and shift its clock so it crosses at T_EDGE.
 * In place on the packed logs: state rows [p, q, v, ω_B] (ω is in body axes and stays), t.
 */
function turnRuns(all) {
  const hw = DECK.halfWidth;
  const cross = all.map(d => {
    const S = d.state; let k = S.findIndex(r => r[2] >= hw); if (k < 1) k = 1;
    const a = (hw - S[k - 1][2]) / (S[k][2] - S[k - 1][2]), at = c => S[k - 1][c] * (1 - a) + S[k][c] * a;
    return { t: d.t[k - 1] * (1 - a) + d.t[k] * a, x: at(0), z: at(2), h: Math.atan2(at(9), -at(7)) };
  });
  const phi = HD - cross.reduce((s, c) => s + c.h, 0) / cross.length, c = Math.cos(phi), s = Math.sin(phi);
  const qy = [0, Math.sin(phi / 2), 0, Math.cos(phi / 2)];
  all.forEach((d, i) => {
    const X = cross[i];
    d.t = d.t.map(t => t - X.t + T_EDGE);
    for (const r of d.state) {
      const dx = r[0] - X.x, dz = r[2] - X.z;
      r[0] = GX + c * dx + s * dz; r[2] = hw - s * dx + c * dz;
      const vx = r[7], vz = r[9]; r[7] = c * vx + s * vz; r[9] = -s * vx + c * vz;
      const [x, y, z, w] = [r[3], r[4], r[5], r[6]], [a, b, cc, e] = qy; // qy ⊗ q
      r[3] = e * x + a * w + b * z - cc * y; r[4] = e * y - a * z + b * w + cc * x; r[5] = e * z + a * y - b * x + cc * w; r[6] = e * w - a * x - b * y - cc * z;
    }
  });
  return phi;
}

export default class GhostsShot {
  constructor(ctx) { this.ctx = ctx; }

  async init() {
    const { stage, ui, assets } = this.ctx;
    DECK.gap = [GX - 8, GX + 8]; // before the set is built: parapet, kerb and the construction props follow it
    ui.loadText('载入天空'); const env = await assets.sky();
    ui.loadText('搭建高架场景'); this.world = assets.world();
    ui.loadText('构建车模'); const car = (this.car = await assets.car());
    ui.loadText('载入几百次模拟');
    const all = await fetch('/opening/ghosts.json').then(r => r.json());
    this.turn = turnRuns(all);
    await this.world.ready;
    this.logs = all.map(d => ({ d, log: new FlightLog(d) }));

    // the hero fires its jets the moment it leaves the deck (a short lift pulse), coasts, then brakes hard for a
    // soft landing (mppi, flat preset: the steadiest attitude of the successes)
    const heroId = P.get('hero') || '20261020-123158/traj/mppi__flat__s7';
    const heroPick = (heroId && this.logs.find(g => g.d.id.includes(heroId))) || this.pickHero();
    this.hero = heroPick;
    // ghost cars: a subset (every preset and controller); every launch gets a trail
    const NCAR = num('ncar', 24);
    this.ghosts = this.diverse(this.logs.filter(g => g !== this.hero), NCAR);

    // car meshes: one CarRenderer at its rest pose only builds the merged geometry + tuned materials; every car
    // here is rigid, so plain meshes with the rest-space geometry under a posed group do the rest
    const src = new CarRenderer(car, { envMap: env });
    this.src = src;
    this.parts = src.group.children.map(m => ({ geo: m.geometry, mat: m.material, glass: /Glass|clear cover|optical clear|Optics/.test(m.material.name || '') || m.material.transmission > 0.3 }));
    this.alpha = num('alpha', 0.13);
    this.fade = { value: 0 }; // ghosts appear as the hero crosses the edge
    this.depthMat = new THREE.MeshBasicMaterial({ colorWrite: false, depthWrite: true, transparent: true });
    const ghostMats = rimRGB => this.parts.map(p => {
      const m = p.mat.clone();
      m.transparent = true; m.opacity = this.alpha * (p.glass ? 0.5 : 1); m.depthWrite = false; m.depthFunc = THREE.LessEqualDepth;
      m.envMap = env; m.transmission = 0;
      const rimK = num('rim', 1.0), alpha = m.opacity;
      // the rim term is clamped: |dot| rounds past 1 on faces square to the lens, pow() of a negative is NaN, and the
      // bloom spreads one NaN pixel over the whole sub-sample, which comes out black (a one-frame flicker in the film)
      m.onBeforeCompile = sh => {
        sh.uniforms.uFade = this.fade;
        sh.fragmentShader = 'uniform float uFade;\n' + sh.fragmentShader.replace('#include <dithering_fragment>', `#include <dithering_fragment>
          { float fr = pow(clamp(1.0 - abs(dot(normalize(normal), normalize(vViewPosition))), 0.0, 1.0), 2.5);
            gl_FragColor.rgb += vec3(${rimRGB}) * fr * ${rimK.toFixed(3)};
            gl_FragColor.a = mix(${alpha.toFixed(3)}, min(1.0, ${alpha.toFixed(3)} * 4.0), fr) * uFade; }`);
      };
      m.customProgramCacheKey = () => 'ghost-rim-' + rimK + '-' + alpha + '-' + rimRGB;
      return m;
    });
    // rim tint by outcome, like the trails: soft landing cyan, crash orange
    const okMats = ghostMats('0.55, 0.95, 1.0'), badMats = ghostMats('1.0, 0.62, 0.38');
    const heroMats = this.parts.map(p => { const m = p.mat.clone(); m.envMap = env; return m; });
    this.mats = { ghost: okMats, bad: badMats, hero: heroMats };
    const mk = (mats, ghost) => {
      const g = new THREE.Group(); g.matrixAutoUpdate = false;
      this.parts.forEach((p, i) => {
        if (ghost) {
          const d = new THREE.Mesh(p.geo, this.depthMat); d.userData.noAO = true; d.frustumCulled = false; d.userData.depth = true; g.add(d);
        }
        const c = new THREE.Mesh(p.geo, mats[i]); c.frustumCulled = false;
        if (ghost) c.userData.noAO = true; else { c.castShadow = !p.glass; c.receiveShadow = true; if (p.glass) c.userData.noAO = true; }
        g.add(c);
      });
      stage.scene.add(g);
      return g;
    };
    for (const g of this.ghosts) g.group = mk(g.d.success ? okMats : badMats, true);
    this.hero.group = mk(heroMats, false);
    this.heroOn = P.get('heroOn') !== '0';
    this.hero.group.visible = this.heroOn;

    // jets of the hero car
    this.plumes = new Plumes(stage.scene, stage);
    this.plumes.setTrack(trackFromTrajectory(this.hero.d));
    this.ghostJets = new GhostJets(stage.scene, this.ghosts.length);

    // trails: centre of mass from the deck edge on
    this.trailMat = new LineMaterial({ color: 0xffffff, linewidth: num('lw', 1.6), vertexColors: true, transparent: true, opacity: num('trailA', 0.35), blending: P.get('blend') === 'add' ? THREE.AdditiveBlending : THREE.NormalBlending, depthWrite: false, worldUnits: false });
    this.trailMat.resolution.set(stage.renderer.domElement.width, stage.renderer.domElement.height);
    this.trails = [];
    for (const g of this.logs) {
      // from just before the deck edge on
      let t0 = -0.2; while (t0 < 2 && g.log.p(t0)[2] < DECK.halfWidth - 1.2) t0 += 0.02;
      const TR = { dt: 0.02, t0, t1: 3.9 };
      const n = Math.round((TR.t1 - TR.t0) / TR.dt) + 1, pos = new Float32Array(n * 3), col = new Float32Array(n * 3);
      const c = new THREE.Color(g.d.success ? 0x8fe6ff : 0xff9a6b);
      for (let k = 0; k < n; k++) {
        const t = TR.t0 + k * TR.dt, p = g.log.p(t);
        pos.set(p, k * 3);
        col.set([c.r, c.g, c.b], k * 3);
      }
      const geo = new LineGeometry(); geo.setPositions(pos); geo.setColors(col);
      const line = new Line2(geo, this.trailMat); line.frustumCulled = false; line.userData.noAO = true; line.renderOrder = 5;
      line.computeLineDistances();
      stage.scene.add(line);
      this.trails.push({ line, geo, n, TR, g });
    }

    // reflections: the set seen from the deck edge
    const hide = [...this.ghosts.map(g => g.group), this.hero.group, ...this.plumes.objects, this.ghostJets.mesh, ...this.trails.map(t => t.line)];
    const probe = stage.captureProbe(v3(GX, DECK.top + 1.2, DECK.halfWidth - 2), hide);
    for (const m of [...okMats, ...badMats, ...heroMats]) m.envMap = probe.texture;
    stage.focus.set(GX, 4, DECK.halfWidth + 14); // shadow frustum: the gap and the landing zone

    this.wind = this.world.group.userData.wind;
    this.film = { total: RAMP.total, render: (T, o) => this.render(T, o), describe: () => this.describe() };
  }

  /** Farthest-point subset: positions and body axes at a few sim times, so the ghosts cover the fan. */
  diverse(list, n) {
    const ts = [1.4, 1.8, 2.2], f = list.map(g => ts.flatMap(t => { const p = g.log.p(t), up = g.log.bodyDir(t, [0, 1, 0]), fw = g.log.bodyDir(t, [0, 0, 1]); return [...p, ...up.map(x => x * 2.5), ...fw.map(x => x * 2.5)]; }));
    const d2 = (a, b) => a.reduce((s, x, i) => s + (x - b[i]) ** 2, 0);
    const out = [0], best = f.map(x => d2(x, f[0]));
    while (out.length < Math.min(n, list.length)) {
      let k = 0; for (let i = 1; i < best.length; i++) if (best[i] > best[k]) k = i;
      out.push(k); for (let i = 0; i < best.length; i++) best[i] = Math.min(best[i], d2(f[i], f[k]));
    }
    return out.map(i => list[i]);
  }

  /** Mean centre of mass of the ghost cars at sim time t (the camera follows it, smoothed in time). */
  centroid(t) {
    if (!this._cen) {
      const n = 400, a = new Float64Array(n * 3), L = this.ghosts.map(g => g.log);
      for (let k = 0; k < n; k++) {
        const tt = -0.5 + k * 0.01; let x = 0, y = 0, z = 0;
        for (const l of L) { const p = l.p(tt); x += p[0]; y += p[1]; z += p[2]; }
        a[k * 3] = x / L.length; a[k * 3 + 1] = y / L.length; a[k * 3 + 2] = z / L.length;
      }
      // box-smooth over 0.3 s so cuts in the mean (cars leaving the log) do not jolt the camera
      const b = new Float64Array(n * 3), w = 15;
      for (let k = 0; k < n; k++) for (let c = 0; c < 3; c++) { let s = 0, m = 0; for (let j = Math.max(0, k - w); j <= Math.min(n - 1, k + w); j++) { s += a[j * 3 + c]; m++; } b[k * 3 + c] = s / m; }
      this._cen = b;
    }
    const f = Math.max(0, Math.min(398.999, (t + 0.5) / 0.01)), k = Math.floor(f), a = f - k, B = this._cen;
    return v3(B[k * 3] * (1 - a) + B[k * 3 + 3] * a, B[k * 3 + 1] * (1 - a) + B[k * 3 + 4] * a, B[k * 3 + 2] * (1 - a) + B[k * 3 + 5] * a);
  }

  pickHero() {
    // a successful gas-study run with the latest ignition (late burn = suicide burn), else any success
    const ign = g => { const th = g.d.thrust; if (!th) return 99; const k = th.findIndex(r => r.reduce((a, b) => a + b, 0) > 3000); return k < 0 ? 99 : g.d.t[k]; };
    const ok = this.logs.filter(g => g.d.success && !g.d.hull && g.d.thrust);
    ok.sort((a, b) => ign(b) - ign(a));
    const late = ok.filter(g => ign(g) < 5 && g.d.preset === 'flat');
    return late[0] || ok[0] || this.logs[0];
  }

  describe() {
    return { total: RAMP.total, ramp: RAMP, hero: this.hero.d.id, ghosts: this.ghosts.length, trails: this.trails.length, t: [0, 0.25, 0.5, 0.75, 1, 1.5, 2, 2.5, 3, RAMP.total].map(T => +simTime(T).toFixed(3)) };
  }

  // ------------------------------------------------------------------ scene at sim time t
  poseGroup(g, t) {
    const L = g.log, p = L.p(t), q = L.q(t), c = L.comB(t);
    const Q = (this._q ||= new THREE.Quaternion()).set(q[0], q[1], q[2], q[3]);
    const off = (this._o ||= v3(0, 0, 0)).set(c[0], c[1], c[2]).applyQuaternion(Q);
    g.group.matrix.compose(v3(p[0] - off.x, p[1] - off.y, p[2] - off.z), Q, v3(1, 1, 1));
    g.group.matrixWorldNeedsUpdate = true;
    return p;
  }

  setScene(t, cam) {
    // ghosts back to front, each: depth pre-pass then colour; they split off the hero as it crosses the edge
    const fade = (this.fade.value = smooth((t - T_EDGE + 0.02) / num('fadeDur', 0.22)));
    const order = this.ghosts.map(g => { const p = this.poseGroup(g, t); g.group.visible = fade > 0; return { g, d: cam.distanceToSquared(v3(p[0], p[1], p[2])) }; }).sort((a, b) => b.d - a.d);
    order.forEach(({ g }, r) => g.group.children.forEach(m => (m.renderOrder = 100 + r * 2 + (m.userData.depth ? 0 : 1))));
    this.ghostJets.update(this.ghosts, t, fade);
    if (this.heroOn) this.poseGroup(this.hero, t);
    this.plumes.visible = this.heroOn;
    this.plumes.update(t);
    // trails up to t (from the deck edge on: a line along the lane would only clutter the deck)
    for (const tr of this.trails) {
      const k = Math.floor((t - tr.TR.t0) / tr.TR.dt);
      tr.line.visible = k >= 1;
      tr.geo.instanceCount = Math.max(1, Math.min(tr.n - 1, k));
    }
    if (this.wind) this.wind.value = t;
  }

  camera(T, t) {
    const dc = P.get('devcam');
    if (dc) { const v = dc.split(',').map(Number); return { pos: v3(v[0], v[1], v[2]), tgt: v3(v[3], v[4], v[5]), fov: v[6] || 40, aperture: 0.002, roll: 0 }; }
    // close in front at the lip, then back, round to the side and down while time crawls
    const u = clamp01((T - CAM.o0) / (CAM.o1 - CAM.o0)), a = 1 - (1 - u) ** CAM.ease, b = 1 - (1 - u) ** CAM.dease;
    const k = ([x0, x1]) => x0 + (x1 - x0) * a, deg = Math.PI / 180, c = this.centroid(t), az = k(CAM.az) * deg, el = k(CAM.el) * deg;
    const dir = E.clone().multiplyScalar(-Math.cos(az)).addScaledVector(N, Math.sin(az)).multiplyScalar(Math.cos(el)).add(v3(0, Math.sin(el), 0));
    const pos = c.clone().addScaledVector(dir, CAM.d[0] + (CAM.d[1] - CAM.d[0]) * b);
    return { pos, tgt: c, fov: k(CAM.fov), aperture: 0.004, roll: CAM.roll * deg * Math.sin(Math.PI * a) };
  }

  evaluate(T) {
    const t = P.has('devt') ? +P.get('devt') : simTime(T);
    return { ...this.camera(T, t), t, shake: 0 };
  }

  render(T, { samples = 16, shutter = 0.5 / 30, aperture } = {}) {
    const stage = this.ctx.stage;
    this.trailMat.resolution.set(stage.renderer.domElement.width, stage.renderer.domElement.height);
    renderFilmFrame(stage, T, x => this.evaluate(x), (t, _T, ev) => this.setScene(t, ev.pos), { samples, shutter, aperture: aperture ?? 0.004 });
  }

  preview(T) {
    const stage = this.ctx.stage, ev = this.evaluate(T);
    applyPose(stage.camera, ev);
    this.setScene(ev.t, ev.pos);
    stage.render(1 / 60);
  }
}

// ------------------------------------------------------------------ faint jets of the ghost cars
// Per ghost and nozzle an additive cone from the exit along the exhaust, length and brightness from the logged
// thrust (relative to the nominal full-open thrust), tinted like the trails (success cyan, failure orange).
const NZ = PROPULSION.nozzles.length, DOWN = v3(0, -1, 0);
class GhostJets {
  constructor(scene, n) {
    const geo = new THREE.ConeGeometry(1, 1, 20, 1, true); geo.translate(0, -0.5, 0); // apex (nozzle) at 0, opens to -y
    const mat = new THREE.ShaderMaterial({
      vertexShader: `varying float vS; varying vec3 vC, vN, vV;
        void main() {
          vS = -position.y;
          #ifdef USE_INSTANCING_COLOR
            vC = instanceColor;
          #else
            vC = vec3(1.0);
          #endif
          mat4 m = modelMatrix * instanceMatrix;
          vec4 mv = viewMatrix * m * vec4(position, 1.0);
          vN = normalize(mat3(viewMatrix) * mat3(m) * normal); vV = -mv.xyz;
          gl_Position = projectionMatrix * mv;
        }`,
      fragmentShader: `varying float vS; varying vec3 vC, vN, vV;
        void main() {
          float facing = abs(dot(normalize(vN), normalize(vV)));
          gl_FragColor = vec4(vC * pow(clamp(1.0 - vS, 0.0, 1.0), 1.8) * (0.2 + 0.8 * facing * facing), 1.0);
        }`,
      transparent: true, blending: THREE.AdditiveBlending, depthWrite: false, side: THREE.DoubleSide,
    });
    this.mesh = new THREE.InstancedMesh(geo, mat, n * NZ);
    this.mesh.frustumCulled = false; this.mesh.userData.noAO = true; this.mesh.renderOrder = 1000;
    for (let i = 0; i < n * NZ; i++) this.mesh.setColorAt(i, new THREE.Color(0, 0, 0));
    scene.add(this.mesh);
    this.nom = nozzleNominal().F;
    this.k = num('jetK', 0.5); this.len = num('jetL', 0.75);
    this._m = new THREE.Matrix4(); this._q = new THREE.Quaternion(); this._p = v3(0, 0, 0); this._d = v3(0, 0, 0); this._s = v3(0, 0, 0); this._c = new THREE.Color();
    this.ok = new THREE.Color(0x8fe6ff); this.bad = new THREE.Color(0xff9a6b);
  }
  update(ghosts, t, fade) {
    const Z = PROPULSION.nozzles;
    ghosts.forEach((g, gi) => {
      const M = g.group.matrix, tint = g.d.success ? this.ok : this.bad;
      for (let i = 0; i < NZ; i++) {
        const j = gi * NZ + i, f = g.log.thrust(t, i) / this.nom[i];
        if (fade <= 0 || f < 0.02) { this.mesh.setMatrixAt(j, this._m.makeScale(0, 0, 0)); continue; }
        const r = Math.sqrt(Math.min(f, 1.5));
        this._p.set(Z[i].exit[0], Z[i].exit[1], Z[i].exit[2]).applyMatrix4(M);
        this._d.set(Z[i].exhaust[0], Z[i].exhaust[1], Z[i].exhaust[2]).transformDirection(M);
        this._q.setFromUnitVectors(DOWN, this._d);
        const L = (0.5 + 2.6 * r) * this.len, R = 0.08 + 0.2 * r;
        this.mesh.setMatrixAt(j, this._m.compose(this._p, this._q, this._s.set(R, L, R)));
        this.mesh.setColorAt(j, this._c.copy(tint).multiplyScalar(this.k * r * fade));
      }
    });
    this.mesh.instanceMatrix.needsUpdate = true; this.mesh.instanceColor.needsUpdate = true;
  }
}
