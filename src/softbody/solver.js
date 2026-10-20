// Soft-body crash solver: XPBD node/beam lattice with force-limited (perfectly plastic + hardening) beams,
// brittle welds, breakable joints, node/node self-contact, static obstacle contact with Coulomb friction,
// tempered/laminated glass failure and a rigid shard/debris particle system. Deterministic for a given seed.
// Runs in a Web Worker in the app and in plain Node for tuning.

import { BEAM, SPRING } from '../car/carPrep.js';
import { DECK, KICK, staticColliders, launchOrigin } from '../scene/layout.js';
import { LAUNCH } from '../core/spec.js';

const G = -9.81;
const REST_V1 = 0.8, REST_V0 = 0.15, REST_K = 0.003; // m/s, m/s, per-substep velocity loss at full rest damping
const SHARD_STRIDE = 7;

function rng(seed) {
  let s = seed >>> 0 || 1;
  return () => { s |= 0; s = (s + 0x6D2B79F5) | 0; let t = Math.imul(s ^ (s >>> 15), 1 | s); t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t; return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
}

export class CrashSim {
  // own cell + 13 forward neighbours: every unordered cell pair is visited once
  static HALF_SHELL = (() => {
    const o = [0, 0, 0];
    for (let dx = -1; dx <= 1; dx++) for (let dy = -1; dy <= 1; dy++) for (let dz = -1; dz <= 1; dz++) {
      if (dx > 0 || (dx === 0 && (dy > 0 || (dy === 0 && dz > 0)))) o.push(dx, dy, dz);
    }
    return Int32Array.from(o);
  })();

  /**
   * @param {object} car  output of buildCar (typed arrays; render meshes not needed)
   * @param {object} sc   scenario {speed, angle, spin:[p,y,r], seed, strength, substepHz, recordHz, duration, level}
   *                      level: levelled suspension + free wheels keep their spin (levelSprings, keepWheelSpin);
   *                      opt-in model update, off by default so existing recordings and results stay valid
   */
  constructor(car, sc) {
    this.car = car;
    this.sc = Object.assign({ speed: 70, angle: 15, spin: [0, 0, 0], seed: 1, strength: 1, joints: 1, substepHz: 8000, recordHz: 100, duration: 7, lead: LAUNCH.defaults.lead }, sc);
    this.rand = rng(this.sc.seed * 7919 + 13);
    const N = (this.N = car.nodes.count);
    this.h = car.h;
    this.x = new Float64Array(N * 3);
    this.p = new Float64Array(N * 3); // position at substep start
    this.v = new Float64Array(N * 3);
    this.w = new Float64Array(N);
    this.alive = new Uint8Array(N).fill(1);
    this.chunk = car.nodes.chunk;
    this.restX = car.nodes.rest; // car-local rest (collision filter, embedding)
    this.allow = car.nodes.allow;
    this.damage = new Float32Array(N);
    this.scrape = new Float32Array(N);
    this.impactV = new Float32Array(N);
    this.contact = new Uint8Array(N);
    const chunks = car.chunks;
    this.nodePiece = new Uint8Array(N);
    const pieces = car.pieces;
    this.pieceIndex = new Map(pieces.map((p, i) => [p, i]));
    for (const c of chunks) for (let n = c.nodeStart; n < c.nodeStart + c.nodeCount; n++) this.nodePiece[n] = this.pieceIndex.get(c.piece);
    this.mu = new Float32Array(N).fill(0.55);
    this.isWheel = new Uint8Array(N);
    this.isMetal = new Uint8Array(N);
    const metal = /^(body|door_|hood|hatch|fender_)/;
    for (const c of chunks) for (let n = c.nodeStart; n < c.nodeStart + c.nodeCount; n++) {
      if (c.wheel) { this.mu[n] = 1.0; this.isWheel[n] = 1; }
      if (metal.test(c.piece)) this.isMetal[n] = 1;
    }
    for (let n = 0; n < N; n++) this.w[n] = 1 / car.nodes.mass[n];

    // beams
    const B = car.beams, NB = (this.NB = B.count), S = this.sc.strength;
    this.bi = B.i; this.bj = B.j; this.kind = B.kind; this.comp = B.comp;
    // connector give scales with the strength setting (weaker = parts come off sooner)
    this.tear = Float32Array.from(B.tear);
    const J = this.sc.joints ?? 1;
    for (let b = 0; b < NB; b++) { const k = B.kind[b]; if (k === BEAM.JOINT || k === BEAM.WELD || k === BEAM.SUSP) this.tear[b] *= J; }
    this.L0 = Float64Array.from(B.L0);
    this.L0o = B.L0;
    this.fy = new Float32Array(NB);
    for (let b = 0; b < NB; b++) this.fy[b] = B.fy[b] * (B.kind[b] === BEAM.SPOKE ? 1 : S);
    // live beam count per node: nodes that lose every beam leave the simulation
    this.nodeBeams = new Int32Array(N);
    for (let b = 0; b < NB; b++) { this.nodeBeams[B.i[b]]++; this.nodeBeams[B.j[b]]++; }
    this.plastic = new Float32Array(NB);
    this.stretchF = new Float32Array(NB);
    this.pre = new Float64Array(NB); // coil-over preload (m beyond the modelled length), see levelSprings
    this.level = !!this.sc.level;
    this.broken = new Uint8Array(NB);
    // tets for anti-inversion
    const T = car.tets, NT = (this.NT = T.length / 4);
    this.tets = T;
    this.tetV0 = new Float32Array(NT);
    const R = this.restX;
    for (let t = 0; t < NT; t++) this.tetV0[t] = this.vol(R, T[t * 4], T[t * 4 + 1], T[t * 4 + 2], T[t * 4 + 3]);
    // per-chunk joint bookkeeping (detachment)
    this.chunkPiece = chunks.map(c => c.piece);
    this.pieceDetached = new Uint8Array(pieces.length);
    this.pieceLinks = new Int32Array(pieces.length);
    for (let b = 0; b < NB; b++) {
      const k = B.kind[b];
      if (k === BEAM.JOINT || k === BEAM.SUSP) { const pa = this.nodePiece[B.i[b]], pb = this.nodePiece[B.j[b]]; if (pa !== pb) { this.pieceLinks[pa]++; this.pieceLinks[pb]++; } }
    }
    // render tet edges -> tear times
    const RT = car.render.tetCount, tn = car.render.tetNodes;
    this.tearTime = new Float32Array(RT).fill(1e9);
    this.edgeTets = new Map();
    for (let r = 0; r < RT; r++) for (let a = 0; a < 4; a++) for (let c = a + 1; c < 4; c++) {
      const u = tn[r * 4 + a], q = tn[r * 4 + c], key = Math.min(u, q) * 33554432 + Math.max(u, q);
      let l = this.edgeTets.get(key); if (!l) this.edgeTets.set(key, (l = [])); l.push(r);
    }
    // glass / lens groups
    this.groups = car.groups.map((g, i) => ({ ...g, id: i, broken: i === 0, breakT: 1e9, crackT: 1e9, crackAt: null }));
    this.groupTime = new Float32Array(this.groups.length).fill(1e9);
    this.latches = (car.latches || []).map(l => ({ ...l, open: false }));
    this.pieceGroup = new Map();
    for (const g of this.groups) if (g.kind === 'tempered' || g.kind === 'laminated') this.pieceGroup.set(g.piece, g);

    // particles (shards / debris)
    const PM = (this.PMAX = 9000);
    this.np = 0;
    this.pp = new Float64Array(PM * 3); this.pv = new Float64Array(PM * 3);
    this.pq = new Float32Array(PM * 4); this.pw = new Float32Array(PM * 3);
    this.pr = new Float32Array(PM); this.pk = new Uint8Array(PM); this.pt = new Float32Array(PM); this.psleep = new Uint8Array(PM);
    this.pSize = new Float32Array(PM * 3);

    this.events = { dust: [], sparks: [], scrape: [], breaks: [], tears: [] };
    this.lastDust = new Map(); this.lastSpark = new Map(); this.lastScrape = new Map();
    this.boxes = staticColliders();
    this.time = 0;
    this.dt = 1 / this.sc.substepHz;
    this.step = 0;
    this.pairs = new Int32Array(1 << 20); this.npairs = 0;
    this.hashSize = 1 << 17;
    this.hashCount = new Int32Array(this.hashSize + 1);
    this.hashFill = new Int32Array(this.hashSize);
    this.hashItems = new Int32Array(N);
    this.cellOf = new Int32Array(N);
    this.cellI = new Int32Array(N * 3);
    this.margin = 0.05;
    this.broadEvery = 20;
    this.activeBoxes = [];
    this.kick = null;
    this.reverse = false;
    /**
     * External actuators (e.g. cold-gas thrusters, src/softbody/actuators.js): objects with
     * `apply(sim, dt)`, called once per substep before gravity/prediction; they may change `sim.v`
     * (velocity impulses) of live nodes. Not called while settling. Order of the array = order of application.
     */
    this.actuators = [];
  }

  vol(X, a, b, c, d) {
    const ax = X[a * 3], ay = X[a * 3 + 1], az = X[a * 3 + 2];
    const ux = X[b * 3] - ax, uy = X[b * 3 + 1] - ay, uz = X[b * 3 + 2] - az;
    const vx = X[c * 3] - ax, vy = X[c * 3 + 1] - ay, vz = X[c * 3 + 2] - az;
    const wx = X[d * 3] - ax, wy = X[d * 3 + 1] - ay, wz = X[d * 3 + 2] - az;
    return (ux * (vy * wz - vz * wy) - uy * (vx * wz - vz * wx) + uz * (vx * wy - vy * wx)) / 6;
  }

  // ------------------------------------------------------------------------------------------------
  /**
   * Levelled suspension (`level`): coil-over preload for the current masses (bare car, or with the thruster kit and
   * its gas), so that the static load holds the car at its modelled ride height instead of sagging 5-11 cm with the
   * tyres inside the arches. Axle loads from the sprung centre of mass (lever rule front/rear, then left/right),
   * shared by the wheel's springs along their rest direction. Without `level` the springs are unloaded (pre = 0).
   */
  levelSprings() {
    if (!this.level) { this.pre.fill(0); return; }
    const car = this.car, R = this.restX, w = this.w, N = this.N, B = car.beams;
    let M = 0, mx = 0, mz = 0;
    for (let n = 0; n < N; n++) {
      if (this.isWheel[n] || !w[n]) continue;
      const m = 1 / w[n]; M += m; mx += m * R[n * 3]; mz += m * R[n * 3 + 2];
    }
    mx /= M; mz /= M;
    const W = car.wheels, wheelOf = new Map(W.map((wh, i) => [wh.chunk, i]));
    const front = W.filter(wh => wh.centre[2] > 0), rear = W.filter(wh => wh.centre[2] <= 0);
    const zf = front.reduce((s, wh) => s + wh.centre[2], 0) / front.length, zr = rear.reduce((s, wh) => s + wh.centre[2], 0) / rear.length;
    const load = W.map(wh => {
      const axle = wh.centre[2] > 0 ? front : rear, other = axle.find(o => o !== wh);
      const Fa = -G * M * (wh.centre[2] > 0 ? mz - zr : zf - mz) / (zf - zr);
      return other ? Fa * (mx - other.centre[0]) / (wh.centre[0] - other.centre[0]) : Fa;
    });
    const springs = [], count = new Array(W.length).fill(0);
    for (let b = 0; b < this.NB; b++) {
      if (B.kind[b] !== BEAM.SPRING) continue;
      const i = B.i[b], j = B.j[b], wi = wheelOf.get(this.isWheel[i] ? this.chunk[i] : this.chunk[j]);
      springs.push([b, wi]); count[wi]++;
    }
    for (const [b, wi] of springs) {
      const i = B.i[b] * 3, j = B.j[b] * 3, cosv = Math.abs(R[j + 1] - R[i + 1]) / B.L0[b];
      this.pre[b] = (load[wi] / count[wi] / cosv) * B.comp[b];
    }
  }

  /** Let the car sag onto its springs on flat ground (deterministic; cacheable). Returns node positions. */
  settle() {
    const N = this.N, x = this.x, R = this.restX, sc = this.sc;
    this.levelSprings();
    let ymin = Infinity;
    for (let n = 0; n < N; n++) ymin = Math.min(ymin, R[n * 3 + 1]);
    for (let n = 0; n < N * 3; n++) x[n] = R[n];
    for (let n = 0; n < N; n++) x[n * 3 + 1] += 0.004 - ymin;
    this.settling = true;
    const saveBoxes = this.boxes; this.boxes = [];
    const steps = Math.round(0.9 * sc.substepHz);
    for (let s = 0; s < steps; s++) { this.substep(); const k = s < steps * 0.7 ? 0.9985 : 0.99; for (let i = 0; i < N * 3; i++) this.v[i] *= k; }
    this.boxes = saveBoxes;
    this.settling = false;
    this.step = 0;
    return Float32Array.from(x);
  }

  /** Place the settled car on the deck with launch velocity. `settled` = cached result of settle(). */
  setup(settled) {
    const N = this.N, x = this.x, sc = this.sc;
    this.levelSprings();
    if (settled && settled.length === N * 3) for (let i = 0; i < N * 3; i++) x[i] = settled[i];
    else this.settle();
    this.v.fill(0);
    for (let b = 0; b < this.NB; b++) { this.plastic[b] = 0; }
    this.damage.fill(0); this.scrape.fill(0);
    this.events = { dust: [], sparks: [], scrape: [], breaks: [], tears: [] };
    this.tearTime.fill(1e9);
    // re-centre horizontally on the settled centre of mass
    let mx = 0, mz = 0, mt = 0;
    for (let n = 0; n < N; n++) { const m = 1 / this.w[n]; mx += x[n * 3] * m; mz += x[n * 3 + 2] * m; mt += m; }
    mx /= mt; mz /= mt;
    // world placement
    const th = (sc.angle * Math.PI) / 180, speed = sc.speed / 3.6;
    // heading -X, veering right (+Z) towards the gap: car +Z (forward) -> f, car +X (right) -> r = up x f
    const f = [-Math.cos(th), 0, Math.sin(th)], r = [Math.sin(th), 0, Math.cos(th)];
    const vz = speed * f[2], vx = speed * f[0];
    // centre of mass crosses the edge `lead` seconds after start, at the middle of the gap
    const [xc, zc] = launchOrigin([vx, 0, vz], sc.lead);
    for (let n = 0; n < N; n++) {
      const lx = x[n * 3] - mx, ly = x[n * 3 + 1], lz = x[n * 3 + 2] - mz;
      x[n * 3] = xc + lx * r[0] + lz * f[0];
      x[n * 3 + 1] = DECK.top + ly;
      x[n * 3 + 2] = zc + lx * r[2] + lz * f[2];
      this.v[n * 3] = vx; this.v[n * 3 + 2] = vz;
    }
    // spinning wheels
    this.wheelSpin = [];
    for (const wh of this.car.wheels) {
      const a0 = wh.axle[0], a1 = wh.axle[1];
      const ax = [x[a1 * 3] - x[a0 * 3], x[a1 * 3 + 1] - x[a0 * 3 + 1], x[a1 * 3 + 2] - x[a0 * 3 + 2]];
      const L = Math.hypot(...ax); ax[0] /= L; ax[1] /= L; ax[2] /= L; // = car +X (right)
      const omega = speed / wh.radius; // rolling forward: spin about +right
      const cx = (x[a0 * 3] + x[a1 * 3]) / 2, cy = (x[a0 * 3 + 1] + x[a1 * 3 + 1]) / 2, cz = (x[a0 * 3 + 2] + x[a1 * 3 + 2]) / 2;
      const ch = this.car.chunks[wh.chunk];
      for (let n = ch.nodeStart; n < ch.nodeStart + ch.nodeCount; n++) {
        const dx = x[n * 3] - cx, dy = x[n * 3 + 1] - cy, dz = x[n * 3 + 2] - cz;
        this.v[n * 3] += omega * (ax[1] * dz - ax[2] * dy);
        this.v[n * 3 + 1] += omega * (ax[2] * dx - ax[0] * dz);
        this.v[n * 3 + 2] += omega * (ax[0] * dy - ax[1] * dx);
      }
      this.wheelSpin.push({ chunk: wh.chunk, axle: [a0, a1], omega });
    }
    this.launch = { velocity: [vx, 0, vz], forward: f, right: r, speed };
    this.time = 0;
    this.kick = { done: false, t0: -1, dur: KICK.dur, spin: sc.spin, f, r };
    this.updateActiveBoxes();
  }

  // ------------------------------------------------------------------------------------------------
  updateActiveBoxes() {
    const N = this.N, x = this.x;
    let lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
    for (let n = 0; n < N; n++) {
      if (!this.alive[n]) continue;
      for (let a = 0; a < 3; a++) { const q = x[n * 3 + a]; if (q < lo[a]) lo[a] = q; if (q > hi[a]) hi[a] = q; }
    }
    this.bounds = [lo, hi];
    const m = 1.0;
    this.activeBoxes = this.boxes.filter(B => {
      const rx = Math.abs(Math.cos(B.yaw)) * B.e[0] + Math.abs(Math.sin(B.yaw)) * B.e[2];
      const rz = Math.abs(Math.sin(B.yaw)) * B.e[0] + Math.abs(Math.cos(B.yaw)) * B.e[2];
      return B.c[0] + rx > lo[0] - m && B.c[0] - rx < hi[0] + m && B.c[1] + B.e[1] > lo[1] - m && B.c[1] - B.e[1] < hi[1] + m && B.c[2] + rz > lo[2] - m && B.c[2] - rz < hi[2] + m;
    });
  }

  broadphase() {
    const N = this.N, x = this.x, h = this.h, alive = this.alive, R = this.restX;
    const reach = 0.84 * h + this.margin, reach2 = reach * reach;
    const filt2 = (1.9 * h) ** 2;
    const inv = 1 / reach, HS = this.hashSize, mask = HS - 1;
    const cnt = this.hashCount, items = this.hashItems, cellOf = this.cellOf, fill = this.hashFill;
    const ci = this.cellI;
    cnt.fill(0);
    for (let n = 0; n < N; n++) {
      if (!alive[n]) { cellOf[n] = -1; continue; }
      const ix = Math.floor(x[n * 3] * inv), iy = Math.floor(x[n * 3 + 1] * inv), iz = Math.floor(x[n * 3 + 2] * inv);
      ci[n * 3] = ix; ci[n * 3 + 1] = iy; ci[n * 3 + 2] = iz;
      const hsh = (Math.imul(ix, 73856093) ^ Math.imul(iy, 19349663) ^ Math.imul(iz, 83492791)) & mask;
      cellOf[n] = hsh; cnt[hsh + 1]++;
    }
    for (let i = 0; i < HS; i++) cnt[i + 1] += cnt[i];
    fill.set(cnt.subarray(0, HS));
    for (let n = 0; n < N; n++) if (cellOf[n] >= 0) items[fill[cellOf[n]]++] = n;
    let np = 0;
    const pairs = this.pairs, maxp = pairs.length / 2;
    const wheel = this.isWheel, piece = this.nodePiece, det = this.pieceDetached;
    const OFF = CrashSim.HALF_SHELL;
    for (let a = 0; a < N; a++) {
      if (cellOf[a] < 0) continue;
      const ax = x[a * 3], ay = x[a * 3 + 1], az = x[a * 3 + 2];
      const ix = ci[a * 3], iy = ci[a * 3 + 1], iz = ci[a * 3 + 2];
      const ra = a * 3, Rx = R[ra], Ry = R[ra + 1], Rz = R[ra + 2];
      for (let o = 0; o < 14; o++) {
        const ox = OFF[o * 3], oy = OFF[o * 3 + 1], oz = OFF[o * 3 + 2];
        const tx = ix + ox, ty = iy + oy, tz = iz + oz;
        const hsh = (Math.imul(tx, 73856093) ^ Math.imul(ty, 19349663) ^ Math.imul(tz, 83492791)) & mask;
        for (let k = cnt[hsh]; k < cnt[hsh + 1]; k++) {
          const b = items[k];
          if (o === 0 && b <= a) continue;
          const ib = b * 3;
          if (ci[ib] !== tx || ci[ib + 1] !== ty || ci[ib + 2] !== tz) continue; // hash collision
          const ex = x[ib] - ax, ey = x[ib + 1] - ay, ez = x[ib + 2] - az;
          if (ex * ex + ey * ey + ez * ez > reach2) continue;
          const rx = R[ib] - Rx, ry = R[ib + 1] - Ry, rz = R[ib + 2] - Rz;
          if (rx * rx + ry * ry + rz * rz < filt2) continue;
          // attached wheels only touch other things once they have come off
          if (wheel[a] | wheel[b]) {
            if (piece[a] === piece[b]) continue;
            if (!det[piece[a]] && !det[piece[b]]) continue;
          }
          if (np < maxp) { pairs[np * 2] = a; pairs[np * 2 + 1] = b; np++; }
        }
      }
    }
    this.npairs = np;
  }

  // ------------------------------------------------------------------------------------------------
  substep() {
    const N = this.N, x = this.x, p = this.p, v = this.v, w = this.w, alive = this.alive, dt = this.dt;
    const gdt = G * dt;
    if (this.step % this.broadEvery === 0) { this.broadphase(); if (this.step % 48 === 0) this.updateActiveBoxes(); }
    // kick: the broken kerb throws the car into a spin as it leaves the deck
    const K = this.kick;
    if (K && !K.done && !this.settling) {
      let mz = 0, mt = 0;
      if (K.t0 < 0) {
        if (this.step % 16 === 0) {
          for (let n = 0; n < N; n += 7) { if (!w[n]) continue; const m = 1 / w[n]; mz += x[n * 3 + 2] * m; mt += m; }
          if (mz / mt > DECK.halfWidth - KICK.trigger) K.t0 = this.time;
        }
      } else if (this.time - K.t0 < K.dur) this.applySpin(dt / K.dur);
      else K.done = true;
    }
    if (!this.settling) for (let k = 0; k < this.actuators.length; k++) this.actuators[k].apply(this, dt);
    this.keepWheelSpin(false);
    for (let n = 0; n < N; n++) {
      const i = n * 3;
      p[i] = x[i]; p[i + 1] = x[i + 1]; p[i + 2] = x[i + 2];
      if (!alive[n]) continue;
      v[i + 1] += gdt;
      x[i] += v[i] * dt; x[i + 1] += v[i + 1] * dt; x[i + 2] += v[i + 2] * dt;
    }
    this.solveBeams(dt);
    if (this.step % 3 === 0) this.solveTets();
    this.solvePairs();
    this.solveStatic(dt);
    const idt = 1 / dt;
    for (let n = 0; n < N; n++) {
      const i = n * 3;
      let vx = (x[i] - p[i]) * idt, vy = (x[i + 1] - p[i + 1]) * idt, vz = (x[i + 2] - p[i + 2]) * idt;
      const s2 = vx * vx + vy * vy + vz * vz;
      if (s2 > 1600) { const k = 40 / Math.sqrt(s2); vx *= k; vy *= k; vz *= k; } // safety net: nothing here exceeds ~25 m/s
      v[i] = vx; v[i + 1] = vy; v[i + 2] = vz;
    }
    if (this.step % 2 === 0) this.dampBeams(0.06);
    this.keepWheelSpin(true);
    if (!this.settling) this.restDamping();
    if (this.step % 8 === 0 && !this.settling) this.checkGroups();
    if (this.pendingShatter) { const g = this.pendingShatter; this.pendingShatter = null; if (!g.broken) this.shatterGroup(g); }
    this.time += dt;
    this.step++;
  }

  /**
   * Free-spinning wheels keep their spin. Projecting the rim's lengths every substep takes a little of each rim
   * node's tangential speed (~(ω·dt)²/2 per substep): over a 2.5 s flight the wheels lost half their spin, so at
   * touchdown the tyres had to be spun up again from ~35 to 69 rad/s by friction alone and that shock nearly
   * snapped control arms. Before the solve (`restore` false) the spin about each axle is measured; after it, wheels
   * that touch nothing get it back (contact flags of this solve: a wheel in contact is left to friction). `level` only.
   */
  keepWheelSpin(restore) {
    if (!this.level) return;
    const W = this.car.wheels, x = this.x, v = this.v, w = this.w, contact = this.contact, chunks = this.car.chunks;
    const om = (this._spin ||= new Float64Array(W.length));
    for (let k = 0; k < W.length; k++) {
      const wh = W[k], a0 = wh.axle[0] * 3, a1 = wh.axle[1] * 3, ch = chunks[wh.chunk];
      if (!w[wh.axle[0]] || !w[wh.axle[1]]) continue;
      let ax = x[a1] - x[a0], ay = x[a1 + 1] - x[a0 + 1], az = x[a1 + 2] - x[a0 + 2];
      const al = Math.hypot(ax, ay, az); ax /= al; ay /= al; az /= al;
      const cx = (x[a0] + x[a1]) / 2, cy = (x[a0 + 1] + x[a1 + 1]) / 2, cz = (x[a0 + 2] + x[a1 + 2]) / 2;
      const ux = (v[a0] + v[a1]) / 2, uy = (v[a0 + 1] + v[a1 + 1]) / 2, uz = (v[a0 + 2] + v[a1 + 2]) / 2;
      let L = 0, I = 0, touch = 0;
      const n0 = ch.nodeStart, n1 = n0 + ch.nodeCount;
      for (let n = n0; n < n1; n++) {
        if (!w[n]) continue;
        const i = n * 3, m = 1 / w[n];
        let rx = x[i] - cx, ry = x[i + 1] - cy, rz = x[i + 2] - cz;
        const d = rx * ax + ry * ay + rz * az; rx -= d * ax; ry -= d * ay; rz -= d * az;
        const vx = v[i] - ux, vy = v[i + 1] - uy, vz = v[i + 2] - uz;
        L += m * (ax * (ry * vz - rz * vy) + ay * (rz * vx - rx * vz) + az * (rx * vy - ry * vx));
        I += m * (rx * rx + ry * ry + rz * rz);
        touch |= contact[n];
      }
      if (!I) continue;
      if (!restore) { om[k] = L / I; continue; }
      if (touch || this.settling) continue;
      const dw = om[k] - L / I;
      for (let n = n0; n < n1; n++) {
        if (!w[n]) continue;
        const i = n * 3;
        let rx = x[i] - cx, ry = x[i + 1] - cy, rz = x[i + 2] - cz;
        const d = rx * ax + ry * ay + rz * az; rx -= d * ax; ry -= d * ay; rz -= d * az;
        v[i] += dw * (ay * rz - az * ry); v[i + 1] += dw * (az * rx - ax * rz); v[i + 2] += dw * (ax * ry - ay * rx);
      }
    }
  }

  /**
   * Coming to rest. The lattice has far less internal friction/hysteresis than crushed sheet metal, so a
   * wreck lying on the road kept rocking and creeping (~0.3 m/s, sinking 6 cm over the last second) and
   * light fragments buzzed on their contacts. Chunks that touch the ground and move slowly (mean node
   * speed below REST_V1) get velocity damping that ramps to full strength at REST_V0 (time constant
   * ~40 ms): the usual "sleeping" treatment of rigid-body engines, applied smoothly. Once the body itself
   * rests, slow parts still hanging off it (a torn bumper dangling on its last clip) settle the same way.
   * Free wheels may spin on.
   */
  restDamping() {
    const chunks = this.car.chunks, v = this.v, alive = this.alive, contact = this.contact;
    const C = chunks.length;
    if (!this.restK) this.restK = new Float32Array(C);
    const K = this.restK;
    if (this.step % 16 === 0) {
      // chunk 0 is the body shell: evaluate it first so hanging parts can follow it to rest
      for (let c = 0; c < C; c++) {
        const ch = chunks[c];
        K[c] = 0;
        if (ch.wheel || !ch.nodeCount) continue;
        let s = 0, k = 0, touch = 0;
        for (let n = ch.nodeStart; n < ch.nodeStart + ch.nodeCount; n++) {
          if (!alive[n]) continue;
          const i = n * 3;
          s += Math.sqrt(v[i] * v[i] + v[i + 1] * v[i + 1] + v[i + 2] * v[i + 2]); k++;
          touch |= contact[n];
        }
        if (!k || !(touch || K[0] > 0)) continue;
        const m = s / k, u = Math.min(1, Math.max(0, (REST_V1 - m) / (REST_V1 - REST_V0)));
        K[c] = REST_K * u * u * (3 - 2 * u);
      }
    }
    for (let c = 0; c < C; c++) {
      const k = K[c];
      if (!k) continue;
      const ch = chunks[c], f = 1 - k;
      for (let i = ch.nodeStart * 3, e = (ch.nodeStart + ch.nodeCount) * 3; i < e; i++) v[i] *= f;
    }
  }

  applySpin(frac) {
    const N = this.N, x = this.x, v = this.v, w = this.w, K = this.kick;
    let cx = 0, cy = 0, cz = 0, mt = 0;
    // nodes that left the simulation have w = 0 (infinite "mass"): skip them
    for (let n = 0; n < N; n++) { if (!w[n]) continue; const m = 1 / w[n]; cx += x[n * 3] * m; cy += x[n * 3 + 1] * m; cz += x[n * 3 + 2] * m; mt += m; }
    cx /= mt; cy /= mt; cz /= mt;
    // spin in car axes -> world: pitch about right, yaw about up, roll about forward
    const [sp, sy, sr] = K.spin, r = K.r, f = K.f;
    const om = [(sp * r[0] + sr * f[0]) * frac, sy * frac, (sp * r[2] + sr * f[2]) * frac];
    for (let n = 0; n < N; n++) {
      const dx = x[n * 3] - cx, dy = x[n * 3 + 1] - cy, dz = x[n * 3 + 2] - cz;
      // move positions too so that the velocity rebuild keeps the impulse
      const ux = om[1] * dz - om[2] * dy, uy = om[2] * dx - om[0] * dz, uz = om[0] * dy - om[1] * dx;
      v[n * 3] += ux; v[n * 3 + 1] += uy; v[n * 3 + 2] += uz;
      this.p[n * 3] -= ux * this.dt; this.p[n * 3 + 1] -= uy * this.dt; this.p[n * 3 + 2] -= uz * this.dt;
    }
  }

  solveBeams(dt) {
    const x = this.x, w = this.w, bi = this.bi, bj = this.bj, L0 = this.L0, L0o = this.L0o, fy = this.fy, kind = this.kind;
    const comp = this.comp, plastic = this.plastic, broken = this.broken, tear = this.tear, damage = this.damage;
    const dt2 = dt * dt, idt2 = 1 / dt2, NB = this.NB;
    const sf = this.stretchF, filt = Math.min(1, dt / 0.003);
    // Always sweep in the same order: alternating directions between substeps pumps energy into the lattice.
    for (let b = 0; b < NB; b++) {
      if (broken[b]) continue;
      const a = bi[b], c = bj[b];
      const wa = w[a], wc = w[c], ws = wa + wc;
      if (ws === 0) continue;
      const ia = a * 3, ic = c * 3;
      let dx = x[ic] - x[ia], dy = x[ic + 1] - x[ia + 1], dz = x[ic + 2] - x[ia + 2];
      const L = Math.sqrt(dx * dx + dy * dy + dz * dz);
      if (L < 1e-9) continue;
      const C = L - L0[b];
      const at = comp[b] * idt2;
      let dl = -C / (ws + at);
      const F = dl * idt2; // N, negative = tension
      const k = kind[b];
      if (k === BEAM.STRUCT || k === BEAM.SPOKE) {
        // strain-based rigid-plasticity: whatever this substep's motion stretched/squeezed the beam
        // beyond its (hardening) yield strain becomes permanent; only the elastic part is pushed back.
        const lo = L0o[b], e = C / lo;
        const ey = fy[b] * (1 + 2.5 * plastic[b]);
        if (e > ey || e < -ey) {
          const keep = (e > 0 ? ey : -ey) * lo;
          let nl = L0[b] + (C - keep);
          if (nl < lo * 0.3) nl = lo * 0.3;
          const dp = Math.abs(nl - L0[b]) / lo;
          L0[b] = nl;
          plastic[b] += dp;
          damage[a] += dp; damage[c] += dp;
          if (nl > lo * (1 + tear[b])) { this.breakBeam(b); continue; }
          dl = -(L - nl) / (ws + at);
        }
      } else if (k === BEAM.SPRING) {
        // elastic about the free length L0 + pre; bump / rebound stops (from L0) are rigid and snap if overloaded
        const pre = this.pre[b], bump = this.level ? SPRING.bump : tear[b], reb = this.level ? SPRING.rebound : tear[b];
        dl = -(C - pre) / (ws + at);
        if (C < -bump || C > reb) {
          const Ce = C < 0 ? C + bump : C - reb;
          dl = -Ce / ws;
          if (Math.abs(dl * idt2) > fy[b]) { this.breakBeam(b); continue; }
          dl += -(C - Ce - pre) / (ws + at);
        }
      } else {
        // compliant welds / bolts / hinges / control arms let go once their (low-passed, ~1.5 ms)
        // stretch exceeds the give: lattice stress-wave spikes alone do not snap them
        const ac = C < 0 ? -C : C;
        const f = (sf[b] += (ac - sf[b]) * filt);
        if (f > tear[b]) { this.breakBeam(b); continue; }
      }
      const s = dl / L;
      dx *= s; dy *= s; dz *= s;
      x[ia] -= wa * dx; x[ia + 1] -= wa * dy; x[ia + 2] -= wa * dz;
      x[ic] += wc * dx; x[ic + 1] += wc * dy; x[ic + 2] += wc * dz;
    }
  }

  dampBeams(k) {
    const v = this.v, w = this.w, x = this.x, bi = this.bi, bj = this.bj, broken = this.broken, kind = this.kind, NB = this.NB;
    const dtd = this.dt * 2;
    for (let b = 0; b < NB; b++) {
      const kb = kind[b];
      if (broken[b] || kb === BEAM.SUSP) continue;
      const a = bi[b], c = bj[b], wa = w[a], wc = w[c], ws = wa + wc;
      if (ws === 0) continue;
      // coil-over dampers (~2.5 kNs/m); everything else: light structural damping of beam-axis velocity
      const kk = kb === BEAM.SPRING ? Math.min(0.8, 3000 * dtd * ws) : kb === BEAM.SPOKE ? 0.3 : k;
      const ia = a * 3, ic = c * 3;
      let nx = x[ic] - x[ia], ny = x[ic + 1] - x[ia + 1], nz = x[ic + 2] - x[ia + 2];
      const L = Math.sqrt(nx * nx + ny * ny + nz * nz); if (L < 1e-9) continue;
      nx /= L; ny /= L; nz /= L;
      const rv = (v[ic] - v[ia]) * nx + (v[ic + 1] - v[ia + 1]) * ny + (v[ic + 2] - v[ia + 2]) * nz;
      const j = (rv * kk) / ws;
      v[ia] += wa * j * nx; v[ia + 1] += wa * j * ny; v[ia + 2] += wa * j * nz;
      v[ic] -= wc * j * nx; v[ic + 1] -= wc * j * ny; v[ic + 2] -= wc * j * nz;
    }
  }

  breakBeam(b) {
    if (this.broken[b]) return;
    this.broken[b] = 1;
    const k = this.kind[b], a = this.bi[b], c = this.bj[b], x = this.x;
    if (--this.nodeBeams[a] <= 0) { this.alive[a] = 0; this.w[a] = 0; }
    if (--this.nodeBeams[c] <= 0) { this.alive[c] = 0; this.w[c] = 0; }
    if (this.settling) return;
    const pa = this.nodePiece[a], pc = this.nodePiece[c];
    if ((k === BEAM.JOINT || k === BEAM.SUSP) && pa !== pc) {
      for (const pi of [pa, pc]) {
        if (--this.pieceLinks[pi] <= 0 && !this.pieceDetached[pi]) {
          this.pieceDetached[pi] = 1;
          const name = this.car.pieces[pi];
          this.events.breaks.push({ t: this.time, type: 'detach', piece: name, x: x[a * 3], y: x[a * 3 + 1], z: x[a * 3 + 2] });
          // a tempered pane torn out of its frame bursts on the spot
          const g = this.pieceGroup.get(name);
          if (g && g.kind === 'tempered' && !g.broken) this.pendingShatter = g;
        }
      }
      if (this.rand() < 0.5) this.spawnChip(a, this.isMetal[a] ? 3 : 4);
    }
    if (k === BEAM.WELD) { if (this.rand() < 0.35) this.spawnChip(a, 4); }
    if (k === BEAM.STRUCT) {
      const key = Math.min(a, c) * 33554432 + Math.max(a, c);
      const l = this.edgeTets.get(key);
      if (l) for (const r of l) if (this.tearTime[r] > 1e8) this.tearTime[r] = this.time;
      this.events.tears.push(this.time);
      if (this.isMetal[a] && this.rand() < 0.25) this.spawnChip(a, 3);
    }
  }

  solveTets() {
    const T = this.tets, NT = this.NT, x = this.x, w = this.w, V0 = this.tetV0;
    for (let t = 0; t < NT; t++) {
      const a = T[t * 4], b = T[t * 4 + 1], c = T[t * 4 + 2], d = T[t * 4 + 3];
      const ia = a * 3, ib = b * 3, ic = c * 3, id = d * 3;
      const ux = x[ib] - x[ia], uy = x[ib + 1] - x[ia + 1], uz = x[ib + 2] - x[ia + 2];
      const vx = x[ic] - x[ia], vy = x[ic + 1] - x[ia + 1], vz = x[ic + 2] - x[ia + 2];
      const qx = x[id] - x[ia], qy = x[id + 1] - x[ia + 1], qz = x[id + 2] - x[ia + 2];
      const V = (ux * (vy * qz - vz * qy) - uy * (vx * qz - vz * qx) + uz * (vx * qy - vy * qx)) / 6;
      const v0 = V0[t], target = 0.18 * v0;
      // same sign convention as rest; only act when collapsing/inverting
      if (v0 > 0 ? V >= target : V <= target) continue;
      // gradients
      const g1x = (vy * qz - vz * qy) / 6, g1y = (vz * qx - vx * qz) / 6, g1z = (vx * qy - vy * qx) / 6;
      const g2x = (qy * uz - qz * uy) / 6, g2y = (qz * ux - qx * uz) / 6, g2z = (qx * uy - qy * ux) / 6;
      const g3x = (uy * vz - uz * vy) / 6, g3y = (uz * vx - ux * vz) / 6, g3z = (ux * vy - uy * vx) / 6;
      const g0x = -g1x - g2x - g3x, g0y = -g1y - g2y - g3y, g0z = -g1z - g2z - g3z;
      const den = w[a] * (g0x * g0x + g0y * g0y + g0z * g0z) + w[b] * (g1x * g1x + g1y * g1y + g1z * g1z) + w[c] * (g2x * g2x + g2y * g2y + g2z * g2z) + w[d] * (g3x * g3x + g3y * g3y + g3z * g3z);
      if (den < 1e-20) continue;
      let l = -(V - target) / den;
      const lim = 2e-4 / Math.sqrt(den / (w[a] + w[b] + w[c] + w[d]) + 1e-30); // cap node moves (~0.2 mm)
      if (l > lim) l = lim; else if (l < -lim) l = -lim;
      x[ia] += w[a] * l * g0x; x[ia + 1] += w[a] * l * g0y; x[ia + 2] += w[a] * l * g0z;
      x[ib] += w[b] * l * g1x; x[ib + 1] += w[b] * l * g1y; x[ib + 2] += w[b] * l * g1z;
      x[ic] += w[c] * l * g2x; x[ic + 1] += w[c] * l * g2y; x[ic + 2] += w[c] * l * g2z;
      x[id] += w[d] * l * g3x; x[id + 1] += w[d] * l * g3y; x[id + 2] += w[d] * l * g3z;
    }
  }

  solvePairs() {
    const x = this.x, w = this.w, pairs = this.pairs, np = this.npairs, alive = this.alive;
    const d0 = 0.84 * this.h, d02 = d0 * d0;
    for (let k = 0; k < np; k++) {
      const a = pairs[k * 2], b = pairs[k * 2 + 1];
      if (!alive[a] || !alive[b]) continue;
      const ia = a * 3, ib = b * 3;
      let dx = x[ib] - x[ia], dy = x[ib + 1] - x[ia + 1], dz = x[ib + 2] - x[ia + 2];
      const d2 = dx * dx + dy * dy + dz * dz;
      if (d2 >= d02 || d2 < 1e-14) continue;
      const d = Math.sqrt(d2), ws = w[a] + w[b];
      // resolve deep overlaps gradually: at most 0.3 mm per substep (~2.4 m/s separation)
      const pen = d0 - d < 3e-4 ? d0 - d : 3e-4;
      const s = pen / (d * ws);
      dx *= s; dy *= s; dz *= s;
      x[ia] -= w[a] * dx; x[ia + 1] -= w[a] * dy; x[ia + 2] -= w[a] * dz;
      x[ib] += w[b] * dx; x[ib + 1] += w[b] * dy; x[ib + 2] += w[b] * dz;
    }
  }

  solveStatic(dt) {
    const N = this.N, x = this.x, p = this.p, alive = this.alive, allow = this.allow, mu = this.mu;
    const boxes = this.activeBoxes, nb = boxes.length;
    const contact = this.contact, impactV = this.impactV;
    const idt = 1 / dt;
    for (let n = 0; n < N; n++) {
      contact[n] = 0;
      if (!alive[n]) continue;
      const i = n * 3;
      const al = allow[n];
      // ground plane
      if (x[i + 1] < -al) this.resolveContact(n, 0, 1, 0, -al - x[i + 1], mu[n], idt);
      for (let k = 0; k < nb; k++) {
        const B = boxes[k];
        const cy = Math.cos(B.yaw), sy = Math.sin(B.yaw);
        const rx = x[i] - B.c[0], ry = x[i + 1] - B.c[1], rz = x[i + 2] - B.c[2];
        const lx = cy * rx - sy * rz, lz = sy * rx + cy * rz;
        const ex = B.e[0] - al, ey = B.e[1] - al, ez = B.e[2] - al;
        const px = ex - Math.abs(lx), py = ey - Math.abs(ry), pz = ez - Math.abs(lz);
        if (px <= 0 || py <= 0 || pz <= 0) continue;
        // push out along the axis of least penetration (prefer the top face for nodes coming from above)
        const prevY = p[i + 1] - B.c[1];
        if (py <= px && py <= pz || (prevY > ey - 0.02 && ry > 0)) {
          this.resolveContact(n, 0, ry > 0 ? 1 : -1, 0, py, B.mu * (mu[n] > 0.9 ? 1.25 : 1), idt);
        } else if (px <= pz) {
          const s = lx > 0 ? 1 : -1; // local x axis -> world
          this.resolveContact(n, cy * s, 0, -sy * s, px, B.mu, idt);
        } else {
          const s = lz > 0 ? 1 : -1;
          this.resolveContact(n, sy * s, 0, cy * s, pz, B.mu, idt);
        }
      }
    }
  }

  resolveContact(n, nx, ny, nz, depth, mu, idt) {
    const x = this.x, p = this.p, i = n * 3;
    // approach speed (for impact events / glass)
    const vn = ((x[i] - p[i]) * nx + (x[i + 1] - p[i + 1]) * ny + (x[i + 2] - p[i + 2]) * nz) * idt;
    x[i] += nx * depth; x[i + 1] += ny * depth; x[i + 2] += nz * depth;
    // Coulomb friction on the substep displacement
    let tx = x[i] - p[i], ty = x[i + 1] - p[i + 1], tz = x[i + 2] - p[i + 2];
    const dn = tx * nx + ty * ny + tz * nz;
    tx -= dn * nx; ty -= dn * ny; tz -= dn * nz;
    const tl = Math.sqrt(tx * tx + ty * ty + tz * tz);
    if (tl > 1e-12) {
      const s = tl < mu * depth * 1.3 ? 1 : (mu * depth) / tl;
      x[i] -= tx * s; x[i + 1] -= ty * s; x[i + 2] -= tz * s;
    }
    this.contact[n] = 1;
    const imp = -vn;
    if (imp > this.impactV[n]) this.impactV[n] = imp;
    if (this.settling) return;
    // effects bookkeeping
    const slip = tl * idt;
    if (imp > 3.5) this.emitDust(n, imp);
    if (slip > 2.5) {
      this.scrape[n] += tl * 0.5; // ~0.5 per metre slid
      if (this.isMetal[n]) this.emitSpark(n, slip, tx / tl, ty / tl, tz / tl);
      this.emitScrape(n, slip);
    }
  }

  emitDust(n, s) {
    const x = this.x, i = n * 3, key = `${Math.floor(x[i] / 0.5)},${Math.floor(x[i + 2] / 0.5)}`;
    const last = this.lastDust.get(key);
    if (last !== undefined && this.time - last < 0.08) return;
    this.lastDust.set(key, this.time);
    const v = this.v;
    this.events.dust.push(this.time, x[i], x[i + 1], x[i + 2], v[i] * 0.3, Math.min(s, 15), v[i + 2] * 0.3);
  }
  emitSpark(n, s, tx, ty, tz) {
    const x = this.x, i = n * 3, key = `${Math.floor(x[i] / 0.25)},${Math.floor(x[i + 2] / 0.25)}`;
    const last = this.lastSpark.get(key);
    if (last !== undefined && this.time - last < 0.02) return;
    this.lastSpark.set(key, this.time);
    this.events.sparks.push(this.time, x[i], x[i + 1], x[i + 2], tx * s, ty * s, tz * s);
  }
  emitScrape(n, s) {
    const x = this.x, i = n * 3, key = `${Math.floor(x[i] / 0.06)},${Math.floor(x[i + 2] / 0.06)}`;
    if (this.lastScrape.has(key)) return;
    this.lastScrape.set(key, this.time);
    this.events.scrape.push(this.time, x[i], x[i + 1], x[i + 2], s, this.isMetal[n]);
  }

  // ------------------------------------------------------------------------------------------------
  checkGroups() {
    const x = this.x;
    // latches pop when the opening distorts
    for (const L of this.latches) {
      if (L.open) continue;
      const P = L.pairs;
      for (let k = 0; k < P.length; k += 3) {
        const a = P[k], b = P[k + 1];
        const d = Math.hypot(x[a * 3] - x[b * 3], x[a * 3 + 1] - x[b * 3 + 1], x[a * 3 + 2] - x[b * 3 + 2]);
        if (Math.abs(d / P[k + 2] - 1) > L.strain * (this.sc.joints ?? 1) * 1.4) {
          L.open = true;
          for (let bb = L.beams[0]; bb < L.beams[1]; bb++) this.breakBeam(bb);
          this.events.breaks.push({ t: this.time, type: 'latch', piece: L.piece, x: x[a * 3], y: x[a * 3 + 1], z: x[a * 3 + 2] });
          break;
        }
      }
    }
    for (const g of this.groups) {
      if (g.broken || !g.pairs) continue;
      // distortion of the host lattice
      let maxStrain = 0;
      const P = g.pairs;
      for (let k = 0; k < P.length; k += 3) {
        const a = P[k], b = P[k + 1], d0 = P[k + 2];
        const d = Math.hypot(x[a * 3] - x[b * 3], x[a * 3 + 1] - x[b * 3 + 1], x[a * 3 + 2] - x[b * 3 + 2]);
        maxStrain = Math.max(maxStrain, Math.abs(d / d0 - 1));
      }
      let maxImp = 0, at = -1;
      for (const n of g.hostNodes) if (this.impactV[n] > maxImp) { maxImp = this.impactV[n]; at = n; }
      const kind = g.kind;
      const strainLim = kind === 'tempered' ? 0.012 : kind === 'laminated' ? 0.008 : 0.03;
      const impLim = kind === 'tempered' ? 4.5 : kind === 'laminated' ? 5.5 : 6;
      if (maxStrain > strainLim || maxImp > impLim) {
        if (kind === 'laminated') {
          if (g.crackT > 1e8) {
            g.crackT = this.time;
            const n = at >= 0 && maxImp > impLim ? at : g.hostNodes[0];
            g.crackAt = [this.restX[n * 3], this.restX[n * 3 + 1], this.restX[n * 3 + 2]];
            this.events.breaks.push({ t: this.time, type: 'crack', group: g.name, x: x[n * 3], y: x[n * 3 + 1], z: x[n * 3 + 2], at: g.crackAt });
            this.shatter(g, 70, 0.6);
            // cracked laminated glass turns into a floppy sheet
            for (let b = 0; b < this.NB; b++) if (this.chunkPiece[this.chunk[this.bi[b]]] === 'windshield' && this.kind[b] === BEAM.STRUCT) this.fy[b] *= 0.2;
          }
          if (maxStrain > 0.12) { g.broken = true; }
          continue;
        }
        this.shatterGroup(g);
      }
    }
    this.impactV.fill(0);
  }

  shatterGroup(g) {
    g.broken = true;
    g.breakT = this.time;
    this.groupTime[g.id] = this.time;
    const x = this.x;
    const c = g.hostNodes[0];
    this.events.breaks.push({ t: this.time, type: g.kind === 'tempered' ? 'glass' : 'lens', group: g.name, x: x[c * 3], y: x[c * 3 + 1], z: x[c * 3 + 2] });
    this.shatter(g, g.count, 1);
    if (g.kind === 'tempered') {
      // the pane is gone: remove its lattice from the simulation
      const ci = this.car.chunks.findIndex(ch => ch.piece === g.piece);
      const ch = this.car.chunks[ci];
      for (let n = ch.nodeStart; n < ch.nodeStart + ch.nodeCount; n++) { this.alive[n] = 0; this.w[n] = 0; }
      for (let b = 0; b < this.NB; b++) if (!this.broken[b] && (this.chunk[this.bi[b]] === ci || this.chunk[this.bj[b]] === ci)) this.broken[b] = 1;
    }
  }

  /** Spawn shards from the group's surface samples (positions follow the deformed host lattice). */
  shatter(g, count, burst) {
    const S = g.samples, x = this.x, v = this.v, r = this.rand;
    const n = Math.min(count, S.length / SHARD_STRIDE);
    for (let s = 0; s < n; s++) {
      const o = s * SHARD_STRIDE;
      const a = S[o], b = S[o + 1], c = S[o + 2], d = S[o + 3], u = S[o + 4], vv = S[o + 5], ww = S[o + 6], t0 = 1 - u - vv - ww;
      const pos = [0, 0, 0], vel = [0, 0, 0];
      for (let k = 0; k < 3; k++) {
        pos[k] = x[a * 3 + k] * t0 + x[b * 3 + k] * u + x[c * 3 + k] * vv + x[d * 3 + k] * ww;
        vel[k] = v[a * 3 + k] * t0 + v[b * 3 + k] * u + v[c * 3 + k] * vv + v[d * 3 + k] * ww;
      }
      const sp = burst * (0.6 + r() * 2.6);
      const th = r() * Math.PI * 2, ph = Math.acos(2 * r() - 1);
      vel[0] += sp * Math.sin(ph) * Math.cos(th); vel[1] += sp * Math.cos(ph) * 0.7 + burst * 0.8; vel[2] += sp * Math.sin(ph) * Math.sin(th);
      const kind = g.shard;
      const size = kind === 6 ? 0.008 + r() * 0.018 : kind === 5 ? 0.01 + r() * 0.03 : 0.006 + r() * 0.03;
      this.addParticle(pos, vel, kind, size);
    }
  }

  spawnChip(n, kind) {
    const x = this.x, v = this.v, r = this.rand, i = n * 3;
    const cnt = 1 + Math.floor(r() * 3);
    for (let k = 0; k < cnt; k++) {
      const vel = [v[i] + (r() - 0.5) * 3, v[i + 1] + r() * 2.5, v[i + 2] + (r() - 0.5) * 3];
      this.addParticle([x[i], x[i + 1], x[i + 2]], vel, kind, 0.008 + r() * 0.03);
    }
  }

  addParticle(pos, vel, kind, size) {
    if (this.np >= this.PMAX) return;
    const k = this.np++, r = this.rand;
    this.pp.set(pos, k * 3); this.pv.set(vel, k * 3);
    // random orientation
    const u1 = r(), u2 = r(), u3 = r();
    const s1 = Math.sqrt(1 - u1), s2 = Math.sqrt(u1);
    this.pq.set([s1 * Math.sin(2 * Math.PI * u2), s1 * Math.cos(2 * Math.PI * u2), s2 * Math.sin(2 * Math.PI * u3), s2 * Math.cos(2 * Math.PI * u3)], k * 4);
    this.pw.set([(r() - 0.5) * 60, (r() - 0.5) * 60, (r() - 0.5) * 60], k * 3);
    this.pr[k] = size * 0.5; this.pk[k] = kind; this.pt[k] = this.time;
    // flat-ish shards: per-axis scale
    const flat = kind === 6 ? 0.55 + r() * 0.4 : 0.12 + r() * 0.2;
    this.pSize.set([size * (0.7 + r() * 0.6), size * flat, size * (0.7 + r() * 0.6)], k * 3);
  }

  stepParticles(dt) {
    const np = this.np, pp = this.pp, pv = this.pv, pq = this.pq, pw = this.pw, pr = this.pr, sl = this.psleep;
    const boxes = this.boxes;
    for (let k = 0; k < np; k++) {
      if (sl[k]) continue;
      const i = k * 3;
      pv[i + 1] += G * dt;
      // light air drag (shards tumble)
      const drag = 1 - 0.25 * dt;
      pv[i] *= drag; pv[i + 1] *= drag; pv[i + 2] *= drag;
      pp[i] += pv[i] * dt; pp[i + 1] += pv[i + 1] * dt; pp[i + 2] += pv[i + 2] * dt;
      const r = pr[k] * 0.6;
      let floor = r, hit = false;
      // ground + boxes (top faces only for particles)
      for (const B of boxes) {
        const top = B.c[1] + B.e[1] + r;
        if (pp[i + 1] < top && pp[i + 1] > top - 0.3 && Math.abs(pp[i] - B.c[0]) < B.e[0] && Math.abs(pp[i + 2] - B.c[2]) < B.e[2]) floor = Math.max(floor, top);
      }
      if (pp[i + 1] < floor) {
        pp[i + 1] = floor; hit = true;
        if (pv[i + 1] < 0) pv[i + 1] *= -0.28;
        pv[i] *= 0.82; pv[i + 2] *= 0.82;
        pw[i] *= 0.7; pw[i + 1] *= 0.7; pw[i + 2] *= 0.7;
      }
      // integrate orientation
      const q = k * 4, wx = pw[i] * dt * 0.5, wy = pw[i + 1] * dt * 0.5, wz = pw[i + 2] * dt * 0.5;
      const qx = pq[q], qy = pq[q + 1], qz = pq[q + 2], qw = pq[q + 3];
      let nx = qx + wx * qw + wy * qz - wz * qy, ny = qy + wy * qw + wz * qx - wx * qz, nz = qz + wz * qw + wx * qy - wy * qx, nw = qw - wx * qx - wy * qy - wz * qz;
      const L = 1 / Math.sqrt(nx * nx + ny * ny + nz * nz + nw * nw);
      pq[q] = nx * L; pq[q + 1] = ny * L; pq[q + 2] = nz * L; pq[q + 3] = nw * L;
      if (hit) {
        const sp = pv[i] * pv[i] + pv[i + 1] * pv[i + 1] + pv[i + 2] * pv[i + 2];
        if (sp < 0.02) {
          sl[k] = 1; pv[i] = pv[i + 1] = pv[i + 2] = 0;
          // lie flat: snap to the nearest "flat" orientation about Y
          const yaw = Math.atan2(2 * (qw * qy + qx * qz), 1 - 2 * (qy * qy + qx * qx));
          pq[q] = 0; pq[q + 1] = Math.sin(yaw / 2); pq[q + 2] = 0; pq[q + 3] = Math.cos(yaw / 2);
          pp[i + 1] = floor - r + this.pSize[k * 3 + 1] * 0.5;
        }
      }
    }
  }

  // ------------------------------------------------------------------------------------------------
  /** Advance by `seconds`; calls onFrame at the record rate. */
  run(seconds, onFrame) {
    const sub = this.sc.substepHz, rec = this.sc.recordHz, perRec = Math.round(sub / rec), perPart = 8;
    const steps = Math.round(seconds * sub);
    for (let s = 0; s < steps; s++) {
      this.substep();
      if (this.step % perPart === 0) this.stepParticles(perPart * this.dt);
      if (this.step % perRec === 0 && onFrame) onFrame(this);
    }
  }

  /** Centre of mass of the main body (for cameras). */
  bodyCentre() {
    const ch = this.car.chunks.find(c => c.piece === 'body');
    let cx = 0, cy = 0, cz = 0;
    for (let n = ch.nodeStart; n < ch.nodeStart + ch.nodeCount; n += 5) { cx += this.x[n * 3]; cy += this.x[n * 3 + 1]; cz += this.x[n * 3 + 2]; }
    const k = Math.ceil(ch.nodeCount / 5);
    return [cx / k, cy / k, cz / k];
  }
}
