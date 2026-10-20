// Shared helpers of the gas study (is "free fall, then burn late" the gas-optimal soft landing?): initial states,
// the bang-bang reference controller, burn analysis of a coldgas-traj@1, execution wrappers (model mismatch the
// planner does not know about), statistics. Plain Node, no dependencies; everything deterministic.
import fs from 'fs';
import path from 'path';
import { fileURLToPath } from 'url';
import { ColdGasEnv } from '../../../src/flight/env.js';
import { GROUPS, NU, CONTROL_DT } from '../../../src/flight/contract.js';
import { tiltOf, heightOf } from '../../../src/flight/tasks.js';
import { bottomThrustNominal } from '../../../src/flight/mpc/objectives/land.js';
import { WORLD, PROPULSION } from '../../../src/core/spec.js';

export const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../../..');
export const DEG = 180 / Math.PI, G = WORLD.gravity;
export const FB_NOM = bottomThrustNominal(); // ≈ 34.9 kN: all four bottom valves open, regulated feed, full tank
export const BOTTOM = GROUPS.bottom;
// The kit is rear-heavy: the COM (z ≈ -0.43 m full, -0.34 m empty) lies well behind the centroid of the bottom
// nozzles (z = 0.55 / -0.95 -> -0.20), so all four valves open pitch the car at ~9 rad/s². Pitch-free heave keeps
// the rear pair open and the front pair at r = (c_z - z_rear) / (z_front - c_z) (≈ 0.53 full): the largest
// vertical thrust that leaves the attitude alone is FB_NOM (1 + r) / 2 ≈ 26.6 kN (T/W ≈ 2.4), not 34.9 kN.
const ZB = BOTTOM.map(i => PROPULSION.nozzles[i].exit[2]), ZF = Math.max(...ZB), ZR = Math.min(...ZB);
export const FRONT = BOTTOM.filter(i => PROPULSION.nozzles[i].exit[2] === ZF), REAR = BOTTOM.filter(i => PROPULSION.nozzles[i].exit[2] === ZR);
export const heaveRatio = cz => Math.min(1, Math.max(0, (cz - ZR) / (ZF - cz)));
/** Pitch-free heave at level u about the COM z cz: valves into out (others untouched). */
export function heaveInto(out, u, cz) { const r = heaveRatio(cz); for (const i of REAR) out[i] = u; for (const i of FRONT) out[i] = u * r; return out; }

export const stamp = () => { const d = new Date(), p = x => String(x).padStart(2, '0'); return `${d.getFullYear()}${p(d.getMonth() + 1)}${p(d.getDate())}-${p(d.getHours())}${p(d.getMinutes())}${p(d.getSeconds())}`; };
export const readJSON = f => JSON.parse(fs.readFileSync(f, 'utf8'));
export const writeJSON = (f, x, ind) => { fs.mkdirSync(path.dirname(f), { recursive: true }); fs.writeFileSync(f, JSON.stringify(x, null, ind)); };

/** Zero-thrust episode until stop(twin) (e.g. the 'airborne' event): returns the env (twin at that state). */
export function coastTo(task, seed, preset, stop) {
  const env = new ColdGasEnv(task, { seed, preset });
  env.reset(seed);
  const u = new Float64Array(NU);
  while (!stop(env.twin)) env.step(u);
  return env;
}

/** Yaw-only quaternion with the same heading (body +Z projected on the horizontal plane). */
export function levelQuat(q) {
  const [x, y, z, w] = q, fx = 2 * (x * z + y * w), fz = 1 - 2 * (x * x + y * y), ps = Math.atan2(fx, fz);
  return [0, Math.sin(ps / 2), 0, Math.cos(ps / 2)];
}

/** Bottom-group burn per control step from a trajectory: thrust sum (N) and gas share by group (ṁ ∝ F: every
 *  nozzle has the same expansion ratio, feed pressure and temperature, so the same exhaust speed). */
export function burnStats(traj, { fon = 0.6, foff = 0.3, scale = 1 } = {}) { // scale: the bottom ceiling (umax.bottom)
  fon *= scale; foff *= scale;
  const ev = traj.events, tArm = ev.find(e => e.type === 'armed')?.t ?? null, td = ev.find(e => e.type === 'touchdown') ?? null;
  const n = traj.n, dt = traj.dt, Fb = new Float64Array(n), Fall = new Float64Array(n);
  // thrust[k] belongs to the step that ended at t_k: attribute it to [t_{k-1}, t_k)
  for (let k = 1; k < n; k++) { let b = 0, a = 0; for (let i = 0; i < NU; i++) { const f = traj.thrust[k][i]; a += f; if (BOTTOM.includes(i)) b += f; } Fb[k - 1] = b; Fall[k - 1] = a; }
  const tTD = td ? td.t : null, kTD = td ? Math.min(n - 1, Math.floor(tTD / dt + 1e-9)) : n - 1;
  const gas = k => traj.tank[0][0] - traj.tank[k][0];
  // ignition: first step (before touchdown) with bottom thrust ≥ fon·F_nom that starts a burn of ≥ 3 steps ≥ foff·F_nom
  let kIgn = -1;
  for (let k = 0; k < kTD; k++) if (Fb[k] >= fon * FB_NOM) {
    let ok = true;
    for (let j = k; j < Math.min(k + 3, kTD); j++) if (Fb[j] < foff * FB_NOM) ok = false;
    if (ok) { kIgn = k; break; }
  }
  let Ib = 0, It = 0, Ia = 0, nOn = 0;
  for (let k = 0; k < kTD; k++) { Ib += Fb[k] * dt; It += Fb[k] * dt * (k + 0.5) * dt; Ia += Fall[k] * dt; if (Fb[k] >= foff * FB_NOM) nOn++; }
  const sIgn = kIgn >= 0 ? traj.state[kIgn] : null;
  const m = traj.metrics || {};
  return {
    t_arm: tArm, t_td: tTD, t_ign: kIgn >= 0 ? kIgn * dt : null,
    ign_after_arm: kIgn >= 0 && tArm != null ? kIgn * dt - tArm : null,
    ign_before_td: kIgn >= 0 && tTD != null ? tTD - kIgn * dt : null,
    h_ign: sIgn ? heightOf(sIgn.slice(0, 3), sIgn.slice(3, 7), traj.comB[kIgn]) : null,
    vy_ign: sIgn ? sIgn[8] : null,
    burn_s: nOn * dt, // steps with bottom thrust ≥ foff·F_nom before touchdown
    gas_pre_ign: kIgn >= 0 ? gas(kIgn) : gas(kTD), gas_to_td: gas(kTD), gas_total: gas(n - 1),
    bottom_share: Ia > 0 ? Ib / Ia : null, // fraction of all thrust impulse (= gas) before touchdown that is bottom
    centroid_before_td: Ib > 0 && tTD != null ? tTD - It / Ib : null, // impulse-weighted mean time-to-touchdown of the bottom burn
    peak_g: m.peak_accel_g ?? null,
  };
}

// unit thrust direction per nozzle (body frame), -exhaust
const DIRS = PROPULSION.nozzles.map(z => { const e = z.exhaust, l = Math.hypot(e[0], e[1], e[2]); return [-e[0] / l, -e[1] / l, -e[2] / l]; });
/**
 * Where the gas goes (from arming to touchdown): gas ∝ thrust impulse (same exhaust speed for every nozzle), so
 *   I_tot   Σ|F| dt (all nozzles)            I_up   world-vertical component of the thrust impulse (braking)
 *   eta_up  I_up / I_tot (cosine / horizontal / pure-torque losses)
 *   flight  arming -> touchdown (s); grav = g ∫M dt over it: the impulse gravity adds (the "gravity loss")
 *   need    M (|v_y,arm| ... ) bookkeeping: I_up = M Δv_y + grav (+ drag, contact before airborne)
 */
export function impulseStats(traj) {
  const ev = traj.events, tArm = ev.find(e => e.type === 'armed')?.t, td = ev.find(e => e.type === 'touchdown');
  if (tArm == null || !td) return null;
  const dt = traj.dt, k0 = Math.round(tArm / dt), k1 = Math.min(traj.n - 1, Math.floor(td.t / dt + 1e-9));
  let It = 0, Iu = 0, Ib = 0, Ibu = 0, grav = 0;
  for (let k = k0; k < k1; k++) {
    const q = traj.state[k].slice(3, 7), [x, y, z, w] = q, r3 = 2 * (x * y + z * w), r4 = 1 - 2 * (x * x + z * z), r5 = 2 * (y * z - x * w);
    const th = traj.thrust[k + 1], M = 1001.857 + traj.tank[k][0];
    for (let i = 0; i < NU; i++) {
      const F = th[i], d = DIRS[i], up = F * (r3 * d[0] + r4 * d[1] + r5 * d[2]);
      It += F * dt; Iu += up * dt;
      if (BOTTOM.includes(i)) { Ib += F * dt; Ibu += up * dt; }
    }
    grav += M * G * dt;
  }
  const gas = traj.tank[k0][0] - traj.tank[k1][0];
  return { flight: k1 * dt - tArm, I_tot: It, I_up: Iu, eta_up: It > 0 ? Iu / It : null, eta_up_bottom: Ib > 0 ? Ibu / Ib : null, bottom_share: It > 0 ? Ib / It : null, grav, gas_arm_td: gas,
    gas_up: It > 0 ? gas * Iu / It : null, gas_lost: It > 0 ? gas * (1 - Iu / It) : null, vy_arm: traj.state[k0][8], vy_td: td ? null : null };
}

/** Per-step series for plots: t, height, v_y, tilt, bottom/rear/side thrust, gas used. */
export function series(traj) {
  const out = { t: [], h: [], vy: [], tilt: [], Fb: [], Fr: [], Fs: [], gas: [] }, n = traj.n;
  for (let k = 0; k < n; k++) {
    const s = traj.state[k], th = traj.thrust[Math.min(k + 1, n - 1)];
    let b = 0, r = 0, sd = 0;
    for (let i = 0; i < NU; i++) { if (GROUPS.bottom.includes(i)) b += th[i]; else if (GROUPS.rear.includes(i)) r += th[i]; else sd += th[i]; }
    out.t.push(traj.t[k]); out.h.push(heightOf(s.slice(0, 3), s.slice(3, 7), traj.comB[k])); out.vy.push(s[8]);
    out.tilt.push(tiltOf(s.slice(3, 7)) * DEG); out.Fb.push(b); out.Fr.push(r); out.Fs.push(sd); out.gas.push(traj.tank[0][0] - traj.tank[k][0]);
  }
  return out;
}

/**
 * Execution wrapper: what the controller commands is not exactly what the plant gets (the planner does not know).
 *   eta     {bottom, rear, side} or number: valve openings scaled (thrust ∝ opening: a weaker engine / partly
 *           blocked nozzle, an un-modelled thrust error)
 *   delay   control steps of transport delay between the plan and the valves (commands held in a FIFO)
 *   hold    commands are only updated every `hold` control steps (a slower actuator bus)
 *   gain    {group: k} commands multiplied by k before eta (clamped to 1): with umax.bottom = 1/k the planner
 *           believes the bottom nozzles are 1/k weaker than they are (a derated planning model, thrust in reserve)
 */
export function wrapExec(inner, { eta = 1, delay = 0, hold = 1, gain = null } = {}) {
  const sc = new Float64Array(NU).fill(1), gk = new Float64Array(NU).fill(1);
  for (const g of ['bottom', 'rear', 'side']) for (const i of GROUPS[g]) { sc[i] = typeof eta === 'number' ? eta : eta[g] ?? 1; gk[i] = gain?.[g] ?? 1; }
  let fifo = [], k = 0, last = new Float64Array(NU);
  const w = Object.create(inner);
  w.reset = (o) => { fifo = []; k = 0; last = new Float64Array(NU); return inner.reset(o); };
  w.act = (s, info) => {
    const u = Float64Array.from(inner.act(s, info));
    for (let i = 0; i < NU; i++) { const v = u[i] * gk[i]; u[i] = (v > 1 ? 1 : v) * sc[i]; }
    fifo.push(u);
    const d = fifo.length > delay ? fifo.shift() : new Float64Array(NU);
    if (k++ % hold === 0) last = d;
    return last;
  };
  return w;
}

/** Bang-bang reference at the 50 Hz control rate: closed until t_arm + t_ign, then pitch-free heave at u (rear pair
 *  at u, front pair at u·r for the current COM) until the first street contact (then closed). */
export function burnController({ tIgn, u = 1, cut = true }) {
  let tArm = null;
  return {
    name: 'burn', config: { tIgn, u },
    reset() { tArm = null; },
    act(s) {
      const out = new Float64Array(NU);
      if (!s.armed) return out;
      if (tArm === null) tArm = s.t;
      if (cut && s.phase === 'ground') return out;
      if (s.t >= tArm + tIgn - 1e-9) heaveInto(out, u, s.comB[2]);
      return out;
    },
  };
}

// ------------------------------------------------------------------------------------------- statistics
export const mean = xs => { const a = xs.filter(Number.isFinite); return a.length ? a.reduce((s, x) => s + x, 0) / a.length : null; };
export const sd = xs => { const a = xs.filter(Number.isFinite), m = mean(a); return a.length > 1 ? Math.sqrt(a.reduce((s, x) => s + (x - m) ** 2, 0) / (a.length - 1)) : null; };
export const median = xs => { const a = xs.filter(Number.isFinite).sort((p, q) => p - q); return a.length ? (a.length % 2 ? a[a.length >> 1] : 0.5 * (a[a.length / 2 - 1] + a[a.length / 2])) : null; };
/** Wilson 95 % interval of a success rate. */
export function wilson(k, n, z = 1.96) {
  if (!n) return [null, null];
  const p = k / n, d = 1 + z * z / n, c = (p + z * z / (2 * n)) / d, h = (z * Math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))) / d;
  return [Math.max(0, c - h), Math.min(1, c + h)];
}
export const fmt = (x, d = 2) => (x == null || !Number.isFinite(x) ? '–' : x.toFixed(d));
export { CONTROL_DT, NU, GROUPS };
