// Compact simulation recording.
// Every lattice chunk (body, door, wheel, fragment ...) is stored as a rigid frame (centre + quaternion,
// found by shape matching) plus Int16 local node offsets at 0.25 mm. Rigid flight/spin interpolates exactly
// (slerp) and local offsets only change while metal actually deforms, so recordings compress very well.
import { quatToMat, quatMul, quatNorm, slerp } from '../core/quat.js';
import { LAUNCH } from '../core/spec.js';
import { driveIn, launchOrigin } from '../scene/layout.js';
export { slerp };

const LOCAL_SCALE = 4000; // 1 unit = 0.25 mm
const PART_SCALE = 500;   // particles: 2 mm
export const REC_VERSION = 3;

/**
 * Rotation extraction (Müller et al. 2016): iteratively rotate q so that it best aligns rest -> current.
 * A is the 3x3 (row-major) covariance sum (x - c)(X - C)^T.
 */
function extractRotation(A, q, iters = 12) {
  const R = new Float64Array(9);
  for (let it = 0; it < iters; it++) {
    quatToMat(q, R);
    // omega = sum_i r_i x a_i / (|sum r_i . a_i| + eps), with r_i columns of R and a_i columns of A
    const r0 = [R[0], R[3], R[6]], r1 = [R[1], R[4], R[7]], r2 = [R[2], R[5], R[8]];
    const a0 = [A[0], A[3], A[6]], a1 = [A[1], A[4], A[7]], a2 = [A[2], A[5], A[8]];
    const cr = (u, v) => [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
    const c0 = cr(r0, a0), c1 = cr(r1, a1), c2 = cr(r2, a2);
    const den = Math.abs(r0[0] * a0[0] + r0[1] * a0[1] + r0[2] * a0[2] + r1[0] * a1[0] + r1[1] * a1[1] + r1[2] * a1[2] + r2[0] * a2[0] + r2[1] * a2[1] + r2[2] * a2[2]) + 1e-9;
    const wx = (c0[0] + c1[0] + c2[0]) / den, wy = (c0[1] + c1[1] + c2[1]) / den, wz = (c0[2] + c1[2] + c2[2]) / den;
    const ang = Math.hypot(wx, wy, wz);
    if (ang < 1e-9) break;
    const s = Math.sin(ang / 2) / ang;
    quatMul([wx * s, wy * s, wz * s, Math.cos(ang / 2)], q, q);
    quatNorm(q);
  }
  return q;
}

// ------------------------------------------------------------------------------------------------
export class RecordingEncoder {
  constructor(car) {
    this.N = car.nodes.count;
    this.chunks = car.chunks.map(c => ({ start: c.nodeStart, count: c.nodeCount }));
    this.C = this.chunks.length;
    // rest centroid + rest local coords per chunk
    const R = car.nodes.rest;
    this.restC = new Float64Array(this.C * 3);
    this.restLocal = new Float64Array(this.N * 3);
    this.q = [];
    for (let c = 0; c < this.C; c++) {
      const { start, count } = this.chunks[c];
      let x = 0, y = 0, z = 0;
      for (let n = start; n < start + count; n++) { x += R[n * 3]; y += R[n * 3 + 1]; z += R[n * 3 + 2]; }
      x /= count || 1; y /= count || 1; z /= count || 1;
      this.restC.set([x, y, z], c * 3);
      for (let n = start; n < start + count; n++) this.restLocal.set([R[n * 3] - x, R[n * 3 + 1] - y, R[n * 3 + 2] - z], n * 3);
      this.q.push([0, 0, 0, 1]);
    }
  }

  /** Encode the current simulator state into a transferable frame. */
  encode(sim, withDamage) {
    const N = this.N, C = this.C, x = sim.x, alive = sim.alive;
    const xf = new Float32Array(C * 7);
    const local = new Int16Array(N * 3);
    const A = new Float64Array(9), m = new Float64Array(9);
    for (let c = 0; c < C; c++) {
      const { start, count } = this.chunks[c];
      let cx = 0, cy = 0, cz = 0, k = 0;
      for (let n = start; n < start + count; n++) { if (!alive[n]) continue; cx += x[n * 3]; cy += x[n * 3 + 1]; cz += x[n * 3 + 2]; k++; }
      if (!k) { for (let n = start; n < start + count; n++) { cx += x[n * 3]; cy += x[n * 3 + 1]; cz += x[n * 3 + 2]; } k = count || 1; }
      cx /= k; cy /= k; cz /= k;
      A.fill(0);
      const RL = this.restLocal;
      for (let n = start; n < start + count; n++) {
        if (!alive[n]) continue;
        const px = x[n * 3] - cx, py = x[n * 3 + 1] - cy, pz = x[n * 3 + 2] - cz;
        const qx = RL[n * 3], qy = RL[n * 3 + 1], qz = RL[n * 3 + 2];
        A[0] += px * qx; A[1] += px * qy; A[2] += px * qz;
        A[3] += py * qx; A[4] += py * qy; A[5] += py * qz;
        A[6] += pz * qx; A[7] += pz * qy; A[8] += pz * qz;
      }
      const q = extractRotation(A, this.q[c]);
      quatToMat(q, m);
      xf.set([cx, cy, cz, q[0], q[1], q[2], q[3]], c * 7);
      for (let n = start; n < start + count; n++) {
        const px = x[n * 3] - cx, py = x[n * 3 + 1] - cy, pz = x[n * 3 + 2] - cz;
        // R^T p
        const lx = m[0] * px + m[3] * py + m[6] * pz, ly = m[1] * px + m[4] * py + m[7] * pz, lz = m[2] * px + m[5] * py + m[8] * pz;
        local[n * 3] = clamp16(lx * LOCAL_SCALE); local[n * 3 + 1] = clamp16(ly * LOCAL_SCALE); local[n * 3 + 2] = clamp16(lz * LOCAL_SCALE);
      }
    }
    // particles relative to a snapped origin
    const np = sim.np, bc = sim.bodyCentre();
    const origin = new Float32Array([Math.round(bc[0]), Math.round(bc[1]), Math.round(bc[2])]);
    const pp = new Int16Array(np * 3), pq = new Int8Array(np * 4);
    for (let k = 0; k < np; k++) {
      for (let a = 0; a < 3; a++) pp[k * 3 + a] = clamp16((sim.pp[k * 3 + a] - origin[a]) * PART_SCALE);
      for (let a = 0; a < 4; a++) pq[k * 4 + a] = Math.round(sim.pq[k * 4 + a] * 127);
    }
    const frame = { t: sim.time, xf, local, origin, np, pp, pq, body: Float32Array.from(bc) };
    if (withDamage) {
      const dmg = new Uint8Array(N), scr = new Uint8Array(N);
      for (let n = 0; n < N; n++) { dmg[n] = Math.min(255, sim.damage[n] * 60); scr[n] = Math.min(255, sim.scrape[n] * 255); }
      frame.damage = dmg; frame.scrape = scr;
    }
    return frame;
  }
}
const clamp16 = v => (v > 32767 ? 32767 : v < -32767 ? -32767 : Math.round(v));

/** Everything that is decided once per run (static data + event logs) – produced at the end. */
export function finalMeta(sim, encoder, extra = {}) {
  return {
    version: REC_VERSION,
    N: sim.N, C: encoder.C,
    recordHz: sim.sc.recordHz,
    scenario: sim.sc,
    launch: sim.launch, wheelSpin: sim.wheelSpin,
    restC: Array.from(encoder.restC),
    particles: {
      count: sim.np,
      kind: Array.from(sim.pk.subarray(0, sim.np)),
      spawn: Array.from(sim.pt.subarray(0, sim.np)),
      size: Array.from(sim.pSize.subarray(0, sim.np * 3)),
    },
    groupTime: Array.from(sim.groupTime),
    groups: sim.groups.map(g => ({ name: g.name, kind: g.kind, breakT: g.breakT, crackT: g.crackT, crackAt: g.crackAt })),
    tearTime: Array.from(sim.tearTime),
    events: {
      dust: Array.from(sim.events.dust), sparks: Array.from(sim.events.sparks), scrape: Array.from(sim.events.scrape),
      breaks: sim.events.breaks,
    },
    ...extra,
  };
}

// ------------------------------------------------------------------------------------------------
/**
 * Binary container: [u32 jsonLen][json][u32 frames][frames...]; each frame = 48 byte header + payload.
 * Node offsets and particle positions are stored as deltas to the previous frame with low/high bytes
 * split into separate planes, which makes the mostly-static data gzip down to a fraction.
 */
export function packRecording(meta, frames) {
  const enc = new TextEncoder();
  const metaBytes = enc.encode(JSON.stringify(meta));
  const N = meta.N, C = meta.C;
  let size = pad4(4 + metaBytes.length) + 4;
  for (const f of frames) size += frameBytes(f, N, C);
  const buf = new ArrayBuffer(size);
  const dv = new DataView(buf);
  dv.setUint32(0, metaBytes.length, true);
  new Uint8Array(buf, 4, metaBytes.length).set(metaBytes);
  let o = pad4(4 + metaBytes.length);
  dv.setUint32(o, frames.length, true); o += 4;
  let prev = null, prevD = null;
  for (const f of frames) { o = writeFrame(buf, o, f, N, C, prev, prevD); prev = f; if (f.damage) prevD = f; }
  return buf;
}
function frameBytes(f, N, C) {
  return pad4(48 + C * 28 + N * 6) + pad4(f.np * 6) + pad4(f.np * 4) + (f.damage ? pad4(N) * 2 : 0);
}
const pad4 = n => (n + 3) & ~3;
function putPlanes16(buf, o, cur, prev) {
  const n = cur.length, u = new Uint8Array(buf, o, n * 2);
  for (let i = 0; i < n; i++) {
    const d = (cur[i] - (prev && i < prev.length ? prev[i] : 0)) & 0xffff;
    u[i] = d & 0xff; u[n + i] = d >> 8;
  }
}
function getPlanes16(buf, o, n, prev) {
  const u = new Uint8Array(buf, o, n * 2), out = new Int16Array(n);
  for (let i = 0; i < n; i++) out[i] = ((u[i] | (u[n + i] << 8)) + (prev && i < prev.length ? prev[i] : 0)) << 16 >> 16;
  return out;
}
function putDelta8(buf, o, cur, prev) {
  const u = new Uint8Array(buf, o, cur.length);
  for (let i = 0; i < cur.length; i++) u[i] = (cur[i] - (prev ? prev[i] : 0)) & 0xff;
}
function getDelta8(buf, o, n, prev) {
  const u = new Uint8Array(buf, o, n), out = new Uint8Array(n);
  for (let i = 0; i < n; i++) out[i] = (u[i] + (prev ? prev[i] : 0)) & 0xff;
  return out;
}
function writeFrame(buf, o, f, N, C, prev, prevD) {
  const dv = new DataView(buf);
  dv.setFloat32(o, f.t, true); dv.setUint32(o + 4, f.np, true); dv.setUint32(o + 8, f.damage ? 1 : 0, true);
  dv.setFloat32(o + 12, f.origin[0], true); dv.setFloat32(o + 16, f.origin[1], true); dv.setFloat32(o + 20, f.origin[2], true);
  dv.setFloat32(o + 24, f.body[0], true); dv.setFloat32(o + 28, f.body[1], true); dv.setFloat32(o + 32, f.body[2], true);
  o += 48;
  new Float32Array(buf, o, C * 7).set(f.xf); o += C * 28;
  putPlanes16(buf, o, f.local, prev?.local); o = pad4(o + N * 6);
  // particle positions are relative to a per-frame origin: delta only when the origin did not move
  const samePO = prev && prev.origin[0] === f.origin[0] && prev.origin[1] === f.origin[1] && prev.origin[2] === f.origin[2];
  putPlanes16(buf, o, f.pp, samePO ? prev.pp : null); o = pad4(o + f.np * 6);
  new Uint8Array(buf, o, f.np * 4).set(new Uint8Array(f.pq.buffer, f.pq.byteOffset, f.np * 4)); o = pad4(o + f.np * 4);
  if (f.damage) {
    putDelta8(buf, o, f.damage, prevD?.damage); o = pad4(o + N);
    putDelta8(buf, o, f.scrape, prevD?.scrape); o = pad4(o + N);
  }
  return o;
}
export function unpackRecording(buf) {
  const dv = new DataView(buf);
  const ml = dv.getUint32(0, true);
  const meta = JSON.parse(new TextDecoder().decode(new Uint8Array(buf, 4, ml)));
  let o = pad4(4 + ml);
  const nf = dv.getUint32(o, true); o += 4;
  const N = meta.N, C = meta.C, frames = [];
  let prev = null, prevD = null;
  for (let i = 0; i < nf; i++) {
    const t = dv.getFloat32(o, true), np = dv.getUint32(o + 4, true), hasD = dv.getUint32(o + 8, true);
    const origin = new Float32Array([dv.getFloat32(o + 12, true), dv.getFloat32(o + 16, true), dv.getFloat32(o + 20, true)]);
    const body = new Float32Array([dv.getFloat32(o + 24, true), dv.getFloat32(o + 28, true), dv.getFloat32(o + 32, true)]);
    o += 48;
    const xf = new Float32Array(buf.slice(o, o + C * 28)); o += C * 28;
    const local = getPlanes16(buf, o, N * 3, prev?.local); o = pad4(o + N * 6);
    const samePO = prev && prev.origin[0] === origin[0] && prev.origin[1] === origin[1] && prev.origin[2] === origin[2];
    const pp = getPlanes16(buf, o, np * 3, samePO ? prev.pp : null); o = pad4(o + np * 6);
    const pq = new Int8Array(buf.slice(o, o + np * 4)); o = pad4(o + np * 4);
    const f = { t, np, origin, body, xf, local, pp, pq };
    if (hasD) { f.damage = getDelta8(buf, o, N, prevD?.damage); o = pad4(o + N); f.scrape = getDelta8(buf, o, N, prevD?.scrape); o = pad4(o + N); prevD = f; }
    frames.push(f);
    prev = f;
  }
  return { meta, frames };
}

// ------------------------------------------------------------------------------------------------
/** Samples node / particle positions at arbitrary times (incl. the drive-in before the sim started: the first
 *  frame moved rigidly along `driveIn`, wheels rolling). */
export class RecordingPlayer {
  constructor(car, meta, frames = []) {
    this.car = car; this.meta = meta; this.frames = frames;
    const v = meta.launch?.velocity || [0, 0, 0];
    this.drive = driveIn(launchOrigin(v, meta.scenario?.lead ?? LAUNCH.defaults.lead), v);
    this.N = meta.N; this.C = meta.C;
    this.chunks = car.chunks.map(c => ({ start: c.nodeStart, count: c.nodeCount }));
    this.pos = new Float32Array(this.N * 3);
    this.q0 = [0, 0, 0, 1]; this.q1 = [0, 0, 0, 1]; this.qi = [0, 0, 0, 1];
    this.m = new Float64Array(9);
    this.wheelChunk = new Map((meta.wheelSpin || []).map(w => [w.chunk, w]));
  }
  get duration() { return this.frames.length ? this.frames[this.frames.length - 1].t : 0; }
  get ready() { return this.frames.length > 0; }

  frameIndex(t) {
    const F = this.frames;
    let lo = 0, hi = F.length - 1;
    if (t <= F[0].t) return [0, 0, 0];
    if (t >= F[hi].t) return [hi, hi, 0];
    while (hi - lo > 1) { const mid = (lo + hi) >> 1; if (F[mid].t <= t) lo = mid; else hi = mid; }
    return [lo, hi, (t - F[lo].t) / (F[hi].t - F[lo].t)];
  }

  /** Fill this.pos with interpolated node positions at time t (seconds of sim time; t<0 = driving in). */
  sample(t) {
    const F = this.frames, pos = this.pos, m = this.m;
    if (!F.length) return pos;
    let [i0, i1, a] = this.frameIndex(t);
    const f0 = F[i0], f1 = F[i1];
    const D = t < F[0].t ? this.drive(t - F[0].t) : null;
    for (let c = 0; c < this.C; c++) {
      const { start, count } = this.chunks[c];
      if (!count) continue;
      const o = c * 7;
      const X0 = f0.xf, X1 = f1.xf;
      let cx = X0[o] + (X1[o] - X0[o]) * a, cy = X0[o + 1] + (X1[o + 1] - X0[o + 1]) * a, cz = X0[o + 2] + (X1[o + 2] - X0[o + 2]) * a;
      this.q0[0] = X0[o + 3]; this.q0[1] = X0[o + 4]; this.q0[2] = X0[o + 5]; this.q0[3] = X0[o + 6];
      this.q1[0] = X1[o + 3]; this.q1[1] = X1[o + 4]; this.q1[2] = X1[o + 5]; this.q1[3] = X1[o + 6];
      const q = slerp(this.q0, this.q1, a, this.qi);
      if (D) {
        const x0 = cx; cx = D.c * x0 + D.s * cz + D.dx; cz = -D.s * x0 + D.c * cz + D.dz;
        const w = this.wheelChunk.get(c);
        if (w) { // roll the wheel backwards about its axle at t = 0 (constant speed: angle ∝ time)
          const ax = this.axleDir(w, f0), ang = w.omega * (t - F[0].t), s = Math.sin(ang / 2);
          quatMul([ax[0] * s, ax[1] * s, ax[2] * s, Math.cos(ang / 2)], q, q);
        }
        quatMul(D.q, q, q);
      }
      quatToMat(q, m);
      const L0 = f0.local, L1 = f1.local;
      for (let n = start; n < start + count; n++) {
        const k = n * 3;
        const lx = (L0[k] + (L1[k] - L0[k]) * a) / LOCAL_SCALE, ly = (L0[k + 1] + (L1[k + 1] - L0[k + 1]) * a) / LOCAL_SCALE, lz = (L0[k + 2] + (L1[k + 2] - L0[k + 2]) * a) / LOCAL_SCALE;
        pos[k] = cx + m[0] * lx + m[1] * ly + m[2] * lz;
        pos[k + 1] = cy + m[3] * lx + m[4] * ly + m[5] * lz;
        pos[k + 2] = cz + m[6] * lx + m[7] * ly + m[8] * lz;
      }
    }
    return pos;
  }

  axleDir(w, f0) {
    // axle direction from the two axle nodes of frame 0
    const c = w.chunk, o = c * 7, L = f0.local, X = f0.xf, m = quatToMat([X[o + 3], X[o + 4], X[o + 5], X[o + 6]], new Float64Array(9));
    const a0 = w.axle[0] * 3, a1 = w.axle[1] * 3;
    const d = [(L[a1] - L[a0]) / LOCAL_SCALE, (L[a1 + 1] - L[a0 + 1]) / LOCAL_SCALE, (L[a1 + 2] - L[a0 + 2]) / LOCAL_SCALE];
    const wx = m[0] * d[0] + m[1] * d[1] + m[2] * d[2], wy = m[3] * d[0] + m[4] * d[1] + m[5] * d[2], wz = m[6] * d[0] + m[7] * d[1] + m[8] * d[2];
    const l = Math.hypot(wx, wy, wz) || 1;
    return [wx / l, wy / l, wz / l];
  }

  /** Particles alive at time t: returns count and fills out (pos xyz, quat xyzw) arrays. */
  sampleParticles(t, outPos, outQuat) {
    const F = this.frames;
    if (!F.length) return 0;
    const [i0, i1, a] = this.frameIndex(t);
    const f0 = F[i0], f1 = F[i1];
    const n = f1.np;
    for (let k = 0; k < n; k++) {
      const has0 = k < f0.np;
      for (let c = 0; c < 3; c++) {
        const p1 = f1.pp[k * 3 + c] / PART_SCALE + f1.origin[c];
        const p0 = has0 ? f0.pp[k * 3 + c] / PART_SCALE + f0.origin[c] : p1;
        outPos[k * 3 + c] = p0 + (p1 - p0) * a;
      }
      const q1 = [f1.pq[k * 4] / 127, f1.pq[k * 4 + 1] / 127, f1.pq[k * 4 + 2] / 127, f1.pq[k * 4 + 3] / 127];
      const q0 = has0 ? [f0.pq[k * 4] / 127, f0.pq[k * 4 + 1] / 127, f0.pq[k * 4 + 2] / 127, f0.pq[k * 4 + 3] / 127] : q1;
      const q = slerp(quatNorm(q0), quatNorm(q1), a, [0, 0, 0, 1]);
      outQuat.set(q, k * 4);
    }
    return n;
  }

  /** Per-node damage/scrape (0..255) from the closest damage frame at or before t. */
  sampleDamage(t) {
    const F = this.frames;
    let [i0] = this.frameIndex(t);
    for (let i = i0; i >= 0; i--) if (F[i].damage) return F[i];
    return null;
  }

  bodyCentre(t) {
    const F = this.frames;
    if (!F.length) return [0, 0, 0];
    const [i0, i1, a] = this.frameIndex(t);
    const b0 = F[i0].body, b1 = F[i1].body, b = [b0[0] + (b1[0] - b0[0]) * a, b0[1] + (b1[1] - b0[1]) * a, b0[2] + (b1[2] - b0[2]) * a];
    if (t >= F[0].t) return b;
    const D = this.drive(t - F[0].t);
    return [D.c * b[0] + D.s * b[2] + D.dx, b[1], -D.s * b[0] + D.c * b[2] + D.dz];
  }
}
