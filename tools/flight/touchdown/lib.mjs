// Shared pieces of the soft-body touchdown rig (drop.mjs) and its experimental design (doe.mjs): factor table,
// touchdown-state conventions, rigid (gyrostat) ballistic propagation and the cheap tyre-geometry feasibility test.
// Conventions (also in README.md): body B = lattice rest frame (+X right, +Y up, +Z forward), q body->world [x,y,z,w].
//   heading psi: forward (sin psi, 0, cos psi); attitude q = yaw(psi) * rotX(-theta) * rotZ(-phi), so
//   theta = asin(fwd_W.y) (nose up +), phi = atan2(-right_W.y, up_W.y) (right side down +), tilt = acos(up_W.y).
//   u = v . h, v_lat = v . r_h with h = (sin psi, 0, cos psi), r_h = (cos psi, 0, -sin psi) (horizontal right),
//   beta = atan2(v_lat, u) (+ = moving toward the car's right side), vs = -v_y (sinking +).
//   yaw_rate = w_B.y (+ = nose turning right), pitch_rate = -w_B.x (= d theta/dt), roll_rate = -w_B.z (= d phi/dt).
//   spin = wheel rim speed / u (omega_wheel = spin * u / r, about the axle = body +X, + = rolling forward).
import fs from 'fs';
import path from 'path';
import { quatToMat, quatMul, quatNorm } from '../../../src/core/quat.js';
import { GENERATED } from '../../../src/core/spec.js';
import { repo } from '../../lib/paths.mjs';

export const deg = 180 / Math.PI, rad = Math.PI / 180, G = 9.81;
export const SUBSTEP_HZ = 8000; // CrashSim default (substepHz)

/** The ten design factors: target state at the instant the first tyre touches the street. */
export const FACTORS = [
  { key: 'vs', lo: 0.3, hi: 3.0, unit: 'm/s', doc: 'sink speed of the COM (downward +)' },
  { key: 'u', lo: 0, hi: 22, unit: 'm/s', doc: 'forward speed along the heading' },
  { key: 'beta', lo: -30, hi: 30, unit: 'deg', doc: 'slip angle atan2(v.r_h, v.h), + = moving toward the right side' },
  { key: 'theta', lo: -8, hi: 8, unit: 'deg', doc: 'pitch, nose up +' },
  { key: 'phi', lo: -8, hi: 8, unit: 'deg', doc: 'roll, right side (body +X) down +' },
  { key: 'yaw_rate', lo: -0.6, hi: 0.6, unit: 'rad/s', doc: 'w_B.y, + = nose turning right' },
  { key: 'pitch_rate', lo: -0.4, hi: 0.4, unit: 'rad/s', doc: '-w_B.x, + = nose rising' },
  { key: 'roll_rate', lo: -0.4, hi: 0.4, unit: 'rad/s', doc: '-w_B.z, + = right side dropping' },
  { key: 'gas', lo: 0.25, hi: 1.0, unit: '1', doc: 'gas fill, fraction of a full tank (145.7 kg)' },
  { key: 'spin', lo: 0, hi: 1.1, unit: '1', doc: 'wheel spin ratio omega*r/u (1 = free rolling)' },
];
// spin 0.57: median of omega*r/u just before touchdown over 9 soft-body flight recordings (0.40-0.86; the wheels
// leave the deck rolling and PBD projection bleeds ~45 % of the spin in a 2.5-3.4 s flight, see README)
export const NOMINAL = { vs: 1.25, u: 15, beta: 0, theta: 0, phi: 0, yaw_rate: 0, pitch_rate: 0, roll_rate: 0, gas: 0.4, spin: 0.57 };
// Touchdown site: COM (x, z) at first contact, horizontal velocity along +Z. The cross street x in [-64, -16] has no
// sidewalk / median boxes; piers (z = 0) and the deck (y > 8.4, |z| < 8.6) are > 6 m away for z > 14.
export const SITE = { x: -40, z: 22 };

export function mulberry32(seed) {
  let s = seed >>> 0 || 1;
  return () => { s |= 0; s = (s + 0x6D2B79F5) | 0; let t = Math.imul(s ^ (s >>> 15), 1 | s); t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t; return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
}

const axisQuat = (ax, ay, az, a) => { const s = Math.sin(a / 2); return [ax * s, ay * s, az * s, Math.cos(a / 2)]; };
/** Attitude from heading psi, pitch theta (nose up +), roll phi (right down +), radians. */
export function attitude(psi, theta, phi) {
  const q = quatMul(axisQuat(0, 1, 0, psi), axisQuat(1, 0, 0, -theta), [0, 0, 0, 1]);
  return quatNorm(quatMul(q, axisQuat(0, 0, 1, -phi), [0, 0, 0, 1]));
}
export const rotate = (m, v) => [m[0] * v[0] + m[1] * v[1] + m[2] * v[2], m[3] * v[0] + m[4] * v[1] + m[5] * v[2], m[6] * v[0] + m[7] * v[1] + m[8] * v[2]];
export const rotateT = (m, v) => [m[0] * v[0] + m[3] * v[1] + m[6] * v[2], m[1] * v[0] + m[4] * v[1] + m[7] * v[2], m[2] * v[0] + m[5] * v[1] + m[8] * v[2]];
export const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];

/** Touchdown parameters (the factor convention) of a rigid state {q, v, w} (+ tilt, heading, horizontal speed). */
export function stateParams(S) {
  const m = quatToMat(S.q, new Float64Array(9));
  const fwd = [m[2], m[5], m[8]], right = [m[0], m[3], m[6]], up = [m[1], m[4], m[7]];
  const psi = Math.atan2(fwd[0], fwd[2]), h = [Math.sin(psi), Math.cos(psi)], rh = [Math.cos(psi), -Math.sin(psi)];
  const u = S.v[0] * h[0] + S.v[2] * h[1], vl = S.v[0] * rh[0] + S.v[2] * rh[1];
  return {
    vs: -S.v[1], u, beta: Math.atan2(vl, u) * deg, theta: Math.asin(Math.max(-1, Math.min(1, fwd[1]))) * deg,
    phi: Math.atan2(-right[1], up[1]) * deg, yaw_rate: S.w[1], pitch_rate: -S.w[0], roll_rate: -S.w[2],
    tilt: Math.acos(Math.max(-1, Math.min(1, up[1]))) * deg, heading: psi * deg, hs: Math.hypot(S.v[0], S.v[2]),
  };
}

/** Target touchdown state (COM height still free: p[1] = 0) of a factor set at the site. Velocity along +Z, psi = -beta. */
export function targetState(f, site = SITE) {
  const b = f.beta * rad, psi = -b;
  return {
    p: [site.x, 0, site.z], q: attitude(psi, f.theta * rad, f.phi * rad),
    v: [0, -f.vs, f.u / Math.cos(b)], w: [-f.pitch_rate, f.yaw_rate, -f.roll_rate], psi,
  };
}

// symmetric / general 3x3 helpers (row-major)
const mul3 = (I, w) => [I[0] * w[0] + I[1] * w[1] + I[2] * w[2], I[3] * w[0] + I[4] * w[1] + I[5] * w[2], I[6] * w[0] + I[7] * w[1] + I[8] * w[2]];
export function inv3(a) {
  const [a0, a1, a2, a3, a4, a5, a6, a7, a8] = a, c0 = a4 * a8 - a5 * a7, c1 = a5 * a6 - a3 * a8, c2 = a3 * a7 - a4 * a6;
  const d = a0 * c0 + a1 * c1 + a2 * c2;
  return [c0 / d, (a2 * a7 - a1 * a8) / d, (a1 * a5 - a2 * a4) / d, c1 / d, (a0 * a8 - a2 * a6) / d, (a2 * a3 - a0 * a5) / d, c2 / d, (a1 * a6 - a0 * a7) / d, (a0 * a4 - a1 * a3) / d];
}

/**
 * Torque-free ballistic flight of a rigid gyrostat (inertia I about the COM in body axes, row-major 3x3, constant
 * internal angular momentum h = the spinning wheels, body axes) over `dt_total` seconds (negative = backwards):
 * I dw/dt = -w x (I w + h), dq/dt = q (w, 0) / 2 (RK4, step `h0`), COM parabola under gravity.
 * Returns {p, q, v, w} (new arrays).
 */
export function propagate(S, I, h, dtTotal, h0 = 1e-4) {
  const Ii = inv3(I), n = Math.max(1, Math.ceil(Math.abs(dtTotal) / h0)), dt = dtTotal / n;
  const f = (q, w) => {
    const L = mul3(I, w); L[0] += h[0]; L[1] += h[1]; L[2] += h[2];
    const tq = cross(w, L), dw = mul3(Ii, [-tq[0], -tq[1], -tq[2]]);
    const dq = quatMul(q, [w[0], w[1], w[2], 0], [0, 0, 0, 0]);
    return [dq.map(c => c / 2), dw];
  };
  let q = [...S.q], w = [...S.w];
  const add = (a, b, s) => a.map((x, i) => x + b[i] * s);
  for (let k = 0; k < n; k++) {
    const [q1, w1] = f(q, w), [q2, w2] = f(add(q, q1, dt / 2), add(w, w1, dt / 2));
    const [q3, w3] = f(add(q, q2, dt / 2), add(w, w2, dt / 2)), [q4, w4] = f(add(q, q3, dt), add(w, w3, dt));
    q = q.map((x, i) => x + dt / 6 * (q1[i] + 2 * q2[i] + 2 * q3[i] + q4[i]));
    w = w.map((x, i) => x + dt / 6 * (w1[i] + 2 * w2[i] + 2 * w3[i] + w4[i]));
    quatNorm(q);
  }
  const T = dtTotal;
  return { p: [S.p[0] + S.v[0] * T, S.p[1] + S.v[1] * T - 0.5 * G * T * T, S.p[2] + S.v[2] * T], q, v: [S.v[0], S.v[1] - G * T, S.v[2]], w };
}

/**
 * Spin bleed of free wheels in the solver (PBD projection of the rim, see CrashSim.keepWheelSpin): d(1/omega^2)/dt =
 * SPIN_BLEED, measured on this lattice in free flight (30 rad/s: -3 rad/s^2, 57 rad/s: -21 rad/s^2; the solver comment's
 * (omega dt)^2/2 per substep is 1/1.84 of it). Returns the spin to give at release so that `omega` remains after `tau`
 * seconds, and the wheel's rotation angle over that time.
 */
export const SPIN_BLEED = 2.3e-4;
export function spinBleed(omega, tau, k = SPIN_BLEED) {
  if (!omega) return { omega0: 0, angle: 0 };
  const a = 1 / (omega * omega) - k * tau;
  const omega0 = a > 0 ? Math.sign(omega) / Math.sqrt(a) : omega;
  const A = 1 / (omega0 * omega0), angle = Math.sign(omega) * 2 / k * (Math.sqrt(A + k * tau) - Math.sqrt(A));
  return { omega0, angle };
}
// The solver clamps every node to 40 m/s (CrashSim.substep safety net). A tyre's top node moves at ~|v_h| + omega r, so
// spin + speed beyond that is destroyed within milliseconds (and jerks the body): the design keeps tyre nodes below VTOP.
export const VCLAMP = 40, VTOP = 38;

// ---------------------------------------------------------------------------------------- design feasibility
// Tyre nodes of the lattice (carPrep wheels: two rings of 24 nodes at +-halfWidth, radius R, rest phase = a node at
// the bottom), from the generated mass properties, so the design check needs no lattice.
const SEG = 24;
export function tyrePoints(wheels = GENERATED.massprops.wheels) {
  const pts = [];
  wheels.forEach((wh, j) => { for (const sx of [-1, 1]) for (let s = 0; s < SEG; s++) { const a = s / SEG * Math.PI * 2; pts.push([wh.centre[0] + sx * wh.halfWidth, wh.centre[1] + wh.radius * Math.sin(a), wh.centre[2] + wh.radius * Math.cos(a), j]); } });
  return pts;
}

/**
 * Is the factor set a genuine first tyre contact the solver can represent? Places the tyre rings at the target attitude
 * with the lowest node on the ground and flies the rigid body back over `window` s (constant body rates: < 2 deg in
 * 60 ms): no tyre node may be below the street before the contact, the touching node must approach at >= `vmin` m/s,
 * and no tyre node may move faster than `vtop` (rigid motion + wheel spin; VCLAMP). comB: body-frame COM (fill
 * dependent), radius: wheel radius for the spin. Returns {ok, approach, minClear, vtyre, wheel}.
 */
export function feasible(f, comB, pts, { window = 0.06, vmin = 0.1, vtop = VTOP, step = 0.005, radius = GENERATED.massprops.wheels[0].radius, wheels = GENERATED.massprops.wheels } = {}) {
  const S = targetState(f), m = quatToMat(S.q, new Float64Array(9));
  const rel = pts.map(P => rotate(m, [P[0] - comB[0], P[1] - comB[1], P[2] - comB[2]]));
  let lo = Infinity, k0 = 0;
  rel.forEach((r, k) => { if (r[1] < lo) { lo = r[1]; k0 = k; } });
  const wW = rotate(m, S.w), vy = k => S.v[1] + cross(wW, rel[k])[1];
  const approach = -vy(k0);
  // tyre node speeds: rigid motion + spin about the axle (a_W x (node - hub))
  const om = f.spin * f.u / radius, aW = [m[0], m[3], m[6]];
  let vtyre = 0;
  rel.forEach((r, k) => {
    const c = wheels[pts[k][3]].centre, hub = rotate(m, [c[0] - comB[0], c[1] - comB[1], c[2] - comB[2]]);
    const u = cross(wW, r), s = cross(aW, [r[0] - hub[0], r[1] - hub[1], r[2] - hub[2]]);
    vtyre = Math.max(vtyre, Math.hypot(S.v[0] + u[0] + om * s[0], S.v[1] + u[1] + om * s[1], S.v[2] + u[2] + om * s[2]));
  });
  let minClear = Infinity;
  for (let t = step; t <= window + 1e-9; t += step) {
    const qd = quatMul(S.q, [S.w[0] * -t / 2, S.w[1] * -t / 2, S.w[2] * -t / 2, 1], [0, 0, 0, 0]); quatNorm(qd); // q(-t)
    const mt = quatToMat(qd, new Float64Array(9)), dy = -S.v[1] * t - 0.5 * G * t * t; // COM height gain going back
    for (const P of pts) { const y = rotate(mt, [P[0] - comB[0], P[1] - comB[1], P[2] - comB[2]])[1] - lo + dy; if (y < minClear) minClear = y; }
  }
  return { ok: approach >= vmin && minClear > 0 && vtyre <= vtop, approach, minClear, vtyre, wheel: pts[k0][3] };
}

/** Spec hash of this checkout (spec/README.md section 7: FNV-1a 64 over every .json under spec/, sorted by relative path). */
export function specHash(dir = repo('spec')) {
  const files = [];
  const walk = d => { for (const e of fs.readdirSync(d, { withFileTypes: true })) { const p = path.join(d, e.name); if (e.isDirectory()) walk(p); else if (e.name.endsWith('.json')) files.push(p); } };
  walk(dir);
  const rel = files.map(p => path.relative(dir, p).split(path.sep).join('/')).sort((a, b) => (a < b ? -1 : a > b ? 1 : 0));
  let h = 0xcbf29ce484222325n; const P = 0x100000001b3n, MASK = (1n << 64n) - 1n;
  const feed = bytes => { for (const b of bytes) h = ((h ^ BigInt(b)) * P) & MASK; };
  for (const r of rel) { feed(Buffer.from(r, 'utf8')); feed([0]); feed(fs.readFileSync(path.join(dir, r))); feed([0]); }
  return h.toString(16).padStart(16, '0');
}
