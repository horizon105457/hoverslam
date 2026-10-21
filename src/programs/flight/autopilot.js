// Interim landing autopilot (Controller interface of src/flight/contract.js) used by the flight-viewer until the
// flight-twin's controller registry exists, and kept as the soft-body reference pilot ("ap"). Pure numbers, no
// three.js, deterministic: identical results in Node (tools/sim/flight.mjs) and in the browser.
//
//   attitude   level the car: body rate command ω_d = Kp·θ·axis (axis/angle taking body-up to world-up, yaw rate
//              0), torque τ = I·Kd·(ω_d − ω), with I the rigid inertia of car + kit about comB (spec/README §4)
//   descent    vertical-speed schedule v_t(h) = −sqrt(v_td² + 2·a_b·h) over the height h of the wheels above the
//              street, F_up = m·(g + kv·(v_t − v_y) + a_ff), a_ff = a_b·v_y/v_t (feed-forward along the curve),
//              projected onto the body-up thrusters (nothing while the car points more than ~75° away from up)
//   road       (film nicety, not a task requirement) side nozzles null the velocity across the road (world z)
//              and the rear pairs yaw the heading onto the travel direction, so the car lands rolling along
//              the street instead of drifting into the kerb at 14 deg
//   allocation box-constrained weighted least squares over the 10 nozzles for the body wrench
//              [F (3), τ (3)] with full-open thrust from the current tank state (thrust is linear in the valve
//              opening, spec/README §3); torque first, heave second, lateral forces barely penalised
//   touchdown  heave / pitch / roll thrust off at the first street contact (the springs take it from there);
//              with the kit's 312 kg behind the rear axle the free-rolling car is yaw-unstable (oversteer: it
//              spins out ~1.5 s after landing at 70 km/h), so a ground mode keeps damping yaw and holding the
//              heading on the travel direction with the side / rear jets (ESP by thrust) while it rolls fast;
//              a hard touchdown (crash sensor: > crashVs m/s down or > crashTilt deg) shuts every valve for good
//   brakes     (spec/drive.json, since 2026-10-21) ground mode brakes fully from brakeAfter s after the touchdown on
//              (the car's ABS keeps the wheels rolling); straight braking is yaw-stable on the soft body (it stops
//              from 20 m/s in ~2.4 s / 31 m, heading within 4°), so the steering stays centred
import { PROPULSION, GENERATED, WORLD } from '../../core/spec.js';
import { Propulsion } from '../../flight/propulsion.js';
import { NU, NA, DRIVE_IDX } from '../../flight/contract.js';
import { quatToMat } from '../../core/quat.js';

const G = WORLD.gravity;

/** Rigid mass properties of car + kit for a gas mass mg: {m, com: B, I: 3x3 about com (body axes)}. */
export function vehicleMassProps(mg, spec = PROPULSION) {
  const L = GENERATED.massprops.lattice;
  const parts = [{ m: L.mass, c: L.com, I: L.inertia }];
  const V = spec.tank.bottles.reduce((s, b) => s + b.volume, 0);
  for (const b of spec.tank.bottles) {
    const m = b.dry_mass + mg * b.volume / V, r = b.diameter / 2, ax = b.axis.map(Math.abs);
    const ia = 0.5 * m * r * r, ip = m * (3 * r * r + b.length * b.length) / 12; // solid cylinder
    parts.push({ m, c: b.center, I: [0, 1, 2].map(i => [0, 1, 2].map(j => (i === j ? ip + (ia - ip) * ax[i] : 0))) });
  }
  for (const z of spec.nozzles) parts.push({ m: z.mass, c: z.mount });
  for (const h of spec.hardware) parts.push({ m: h.mass, c: h.center });
  const m = parts.reduce((s, p) => s + p.m, 0);
  const com = [0, 1, 2].map(a => parts.reduce((s, p) => s + p.m * p.c[a], 0) / m);
  const I = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  for (const p of parts) {
    const d = [p.c[0] - com[0], p.c[1] - com[1], p.c[2] - com[2]], d2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) I[i][j] += (p.I ? p.I[i][j] : 0) + p.m * ((i === j ? d2 : 0) - d[i] * d[j]);
  }
  return { m, com, I };
}

export class Autopilot {
  static DEFAULTS = {
    kp: 5.0, kd: 9.0, wmax: 4.0, kv: 2.5, ab: 4.5, vtd: 0.8, tiltCut: 1.3,
    wF: 1.0, wT: 3.0, wYaw: 1.0, wLat: 0.03, wFwd: 0.3, reg: 0.002,
    road: true, klat: 1.5, kpsi: 2.5, wSide: 0.6,
    esp: true, kgy: 5.0, kgpsi: 3.0, crashVs: 3.5, crashTilt: 30,
    brakes: true, brakeAfter: 0.15,
  };
  /** @param opts gains (see DEFAULTS) */
  constructor(opts = {}) {
    this.name = opts.name || 'ap';
    this.o = { ...Autopilot.DEFAULTS, ...opts };
    this.spec = PROPULSION;
    this.prop = new Propulsion(this.spec);
    const Z = this.spec.nozzles;
    this.d = Z.map(z => z.exhaust.map(v => -v));       // thrust axis (body)
    this.x = Z.map(z => z.exit);
    this.f = new Float64Array(NU);
    this.u = new Float64Array(NA); // valves, then brake / steer (DRIVE_IDX)
    this.F1 = new Float64Array(NU);
    this._R = new Float64Array(9);
  }
  reset({ task } = {}) { this.task = task; this.touched = false; this.crashed = false; this.tTouch = null; this.f.fill(0); this.u.fill(0); }

  /** Full-open thrust of every nozzle for the current tank state (model v1: F = s·F1). */
  fullThrust(tank) {
    const P = this.prop;
    P.m = tank.m; P.T = tank.T; P.p = tank.p;
    const pc = P.feedPressure(), Tc = tank.T;
    for (let i = 0; i < NU; i++) {
      if (pc <= 0) { this.F1[i] = 0; continue; }
      const md = P.At[i] * pc * P.choke / Math.sqrt(Tc), ve = P.Me[i] * Math.sqrt(P.g * P.R * Tc * P.TeRatio[i]);
      this.F1[i] = Math.max(0, md * ve + (pc * P.peRatio[i] - P.pa) * P.Ae[i]);
    }
    return this.F1;
  }

  /** Signed yaw angle (about world up) from the body heading to the horizontal travel direction. */
  slip(s, R) {
    const vh = Math.hypot(s.v[0], s.v[2]), fx = R[2], fz = R[8], fl = Math.hypot(fx, fz) || 1, dx = s.v[0] / vh, dz = s.v[2] / vh;
    return Math.atan2((fz * dx - fx * dz) / fl, (fx * dx + fz * dz) / fl);
  }

  act(s) {
    const o = this.o;
    const R = quatToMat(s.q, this._R);
    const gB = [R[3], R[4], R[5]]; // world up in body axes (row 1 of R)
    if (s.phase === 'ground' && !this.touched) {
      this.touched = true; this.tTouch = s.t;
      this.crashed = -s.v[1] > o.crashVs || Math.acos(Math.max(-1, Math.min(1, gB[1]))) * 180 / Math.PI > o.crashTilt;
    }
    const mp = vehicleMassProps(s.tank.m), I = mp.I, m = mp.m;
    const vh = Math.hypot(s.v[0], s.v[2]);
    if (this.touched) {
      // ground mode: brakes, yaw damping + heading hold only, wheels down and upright
      this.u[DRIVE_IDX.brake] = o.brakes && s.t >= this.tTouch + o.brakeAfter ? 1 : 0;
      if (!o.esp || this.crashed || vh < 3 || gB[1] < 0.8) { this.u.fill(0, 0, NU); this.f.fill(0); return this.u; }
      const ty = I[1][1] * (o.kgpsi * this.slip(s, R) - o.kgy * s.w[1]);
      return this.allocate(s, [0, 0, 0, 0, ty, 0], [o.wLat, o.wF, o.wFwd, 0, o.wYaw, 0]);
    }
    // attitude: rotate body-up onto world-up; near inverted keep turning the way the car already rolls
    const ang = Math.acos(Math.max(-1, Math.min(1, gB[1])));
    let ex = gB[2], ez = -gB[0];
    const el = Math.hypot(ex, ez);
    if (el < 1e-3) { const sg = s.w[2] >= 0 ? 1 : -1; ex = 0; ez = sg; } else { ex /= el; ez /= el; }
    const wd = [o.kp * ang * ex, 0, o.kp * ang * ez];
    // road mode: heading onto the travel direction, velocity across the road (world z) -> 0 by the side jets
    let fxB = 0;
    if (o.road && gB[1] > 0.8 && vh > 2) {
      wd[1] = o.kpsi * this.slip(s, R);
      const Fz = m * o.klat * -s.v[2];
      fxB = Math.abs(R[6]) > 0.3 ? Fz / R[6] : 0;
    }
    const wn = Math.hypot(wd[0], wd[2]); if (wn > o.wmax) { wd[0] *= o.wmax / wn; wd[2] *= o.wmax / wn; }
    const e = [wd[0] - s.w[0], wd[1] - s.w[1], wd[2] - s.w[2]];
    const tau = [0, 1, 2].map(i => o.kd * (I[i][0] * e[0] + I[i][1] * e[1] + I[i][2] * e[2]));
    // descent: wheels' height above the street
    const h = Math.max(0, s.p[1] - s.comB[1]);
    const vt = -Math.sqrt(o.vtd * o.vtd + 2 * o.ab * h), vy = s.v[1];
    const aff = h > 0.05 ? o.ab * Math.max(0, Math.min(1.5, vy / vt)) : 0;
    const Fup = Math.max(0, m * (G + o.kv * (vt - vy) + aff));
    const fy = gB[1] > Math.cos(o.tiltCut) ? Fup / Math.max(0.35, gB[1]) : 0;
    return this.allocate(s, [fxB, fy, 0, tau[0], tau[1], tau[2]], [fxB ? o.wSide : o.wLat, o.wF, o.wFwd, o.wT, o.wYaw, o.wT]);
  }

  /**
   * Body wrench w = [F (N), τ (N m)] -> openings: min Σ W_k (A f − w)_k² + reg |f|² over 0 <= f <= F1 (kN units),
   * projected Gauss-Seidel warm-started from the previous period.
   */
  allocate(s, w, W) {
    const o = this.o, u = this.u, F1 = this.fullThrust(s.tank), c = s.comB, b = w.map(v => v / 1000);
    const A = this.x.map((x, i) => { const r = [x[0] - c[0], x[1] - c[1], x[2] - c[2]], d = this.d[i]; return [d[0], d[1], d[2], r[1] * d[2] - r[2] * d[1], r[2] * d[0] - r[0] * d[2], r[0] * d[1] - r[1] * d[0]]; });
    const H = A.map(ai => A.map(aj => ai.reduce((sum, v, k) => sum + W[k] * v * aj[k], 0)));
    const g = A.map(ai => ai.reduce((sum, v, k) => sum + W[k] * v * b[k], 0));
    for (let i = 0; i < NU; i++) H[i][i] += o.reg;
    const f = this.f, fmax = Array.from(F1, v => v / 1000);
    for (let i = 0; i < NU; i++) f[i] = Math.min(f[i], fmax[i]);
    for (let it = 0; it < 80; it++) {
      for (let i = 0; i < NU; i++) {
        let r = g[i];
        for (let j = 0; j < NU; j++) if (j !== i) r -= H[i][j] * f[j];
        const v = r / H[i][i];
        f[i] = v < 0 ? 0 : v > fmax[i] ? fmax[i] : v;
      }
    }
    for (let i = 0; i < NU; i++) u[i] = fmax[i] > 0 ? f[i] / fmax[i] : 0;
    return u;
  }
}
