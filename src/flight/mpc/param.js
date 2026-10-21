// Control parameterisation of the MPC (src/flight/mpc): decision vector θ = nk knots × nv channels (knot-major)
// -> valve commands u ∈ [0, umax]^NU at any time τ (s since the plan started).
//  * knot times τ_j: spacing `knot` growing by the factor `knot_growth` per knot (1 = uniform; > 1 puts the
//    decision variables where they matter, near term, and keeps the far end coarse), until the horizon is covered;
//    'zoh' holds knot j on [τ_j, τ_j+1), 'linear' interpolates between knot times (with a knot at the horizon);
//  * mixer 'raw': one channel per valve (nv = NU, θ_i ∈ [0, umax_i]); 'modes': 7 wrench channels per nozzle group
//    (bottom heave / pitch / roll, rear surge / pitch / yaw, side sway), columns of the min-norm allocation
//    pinv(A) of each group's wrench matrix about the full-tank COM, scaled to a peak entry of 1; u = clamp(Mθ).
//    Heave and surge are in [0, 1], the differential channels in [-1, 1]. Fewer, decoupled dimensions make
//    sampling far more efficient; the clamp makes M θ piecewise linear, which the optimizers tolerate.
//  * drive (opt-in, `drive: true`): two more channels after the mixer's, `brake` ∈ [0, 1] and `steer` ∈ [-1, 1]
//    (spec/drive.json, action indices NU, NU + 1), passed through unmixed; commands are then NA long.
// No feedback anywhere: θ is an open-loop command schedule that the receding horizon re-optimises.
import { NU, NA, CONTROL_DT, GROUPS, DRIVE_IDX } from '../contract.js';
import { PROPULSION } from '../../core/spec.js';
import { MassModel } from '../massprops.js';
import { Propulsion } from '../propulsion.js';
import { pyRound, solve3 } from '../twin.js';

/** Wrench rows per group: index into (Fx, Fy, Fz, τx, τy, τz) (body frame, about the COM). */
const MODES = [
  { group: 'bottom', rows: [1, 3, 5], names: ['heave', 'pitch', 'roll'], signed: [false, true, true] },
  { group: 'rear', rows: [2, 3, 4], names: ['surge', 'pitch_r', 'yaw'], signed: [false, true, true] },
  { group: 'side', rows: [0], names: ['sway'], signed: [true] },
];

/** NU × 7 mixing matrix (row-major, nozzle i, channel j) and the channel names / signedness. */
export function modeMatrix() {
  const com = new MassModel().at(new Propulsion().m0).com;
  const M = new Float64Array(NU * 7), names = [], signed = [];
  let j0 = 0;
  for (const md of MODES) {
    const idx = GROUPS[md.group], n = idx.length, r = md.rows.length;
    // 6 x n wrench of unit thrust per nozzle: (d, (e - c) × d)
    const W = idx.map(i => {
      const z = PROPULSION.nozzles[i], e = z.exhaust, l = Math.hypot(e[0], e[1], e[2]);
      const d = [-e[0] / l, -e[1] / l, -e[2] / l], x = [z.exit[0] - com[0], z.exit[1] - com[1], z.exit[2] - com[2]];
      return [d[0], d[1], d[2], x[1] * d[2] - x[2] * d[1], x[2] * d[0] - x[0] * d[2], x[0] * d[1] - x[1] * d[0]];
    });
    const A = md.rows.map(row => W.map(w => w[row])); // r × n
    // pinv(A) = Aᵀ (A Aᵀ)⁻¹, column by column: solve (A Aᵀ) y = e_k, column = Aᵀ y
    const G = A.map(a => A.map(b => a.reduce((s, v, k) => s + v * b[k], 0)));
    for (let k = 0; k < r; k++) {
      const ek = md.rows.map((_, i) => (i === k ? 1 : 0));
      const y = r === 1 ? [1 / G[0][0]] : solve3(G.flat(), ek);
      const col = idx.map((_, m) => A.reduce((s, a, i) => s + a[m] * y[i], 0));
      const peak = Math.max(...col.map(Math.abs));
      idx.forEach((i, m) => { M[i * 7 + j0 + k] = col[m] / peak; });
      names.push(md.names[k]); signed.push(md.signed[k]);
    }
    j0 += r;
  }
  return { M, names, signed };
}

export class ControlParam {
  /** c: MPC settings (horizon, knot, interp, mixer, umax). */
  constructor(c) {
    this.H = Math.max(1, pyRound(c.horizon / CONTROL_DT)); // control steps per rollout
    this.linear = c.interp === 'linear';
    const tk = [0], gr = c.knot_growth ?? 1; // knot start times
    for (let d = c.knot; tk[tk.length - 1] + d < c.horizon - 1e-9; d *= gr) tk.push(tk[tk.length - 1] + d);
    if (this.linear) tk.push(c.horizon);
    this.tk = Float64Array.from(tk); this.nk = tk.length;
    this.umax = new Float64Array(NU);
    for (let i = 0; i < NU; i++) {
      const g = PROPULSION.nozzles[i].group;
      this.umax[i] = typeof c.umax === 'number' ? c.umax : c.umax?.[g] ?? 1;
    }
    if (c.mixer === 'modes') {
      const { M, names, signed } = modeMatrix();
      this.M = M; this.nv = 7; this.names = names;
      this.clo = Float64Array.from(signed, s => (s ? -1 : 0)); this.chi = new Float64Array(7).fill(1);
    } else if (c.mixer === 'raw') {
      this.M = null; this.nv = NU; this.names = PROPULSION.nozzles.map(z => z.id);
      this.clo = new Float64Array(NU); this.chi = Float64Array.from(this.umax);
    } else throw new Error(`mpc: unknown mixer ${c.mixer}`);
    this.nm = this.nv; // mixer channels; the drive channels follow them
    this.drive = !!c.drive; this.na = this.drive ? NA : NU;
    if (this.drive) {
      this.nv += 2; this.names = [...this.names, 'brake', 'steer'];
      this.clo = Float64Array.from([...this.clo, 0, -1]); this.chi = Float64Array.from([...this.chi, 1, 1]);
      this.driveInit = c.drive_init ?? [0, 0];
    }
    // exploration scale per channel (optimizers multiply their sigma by it): {name: factor} or an nv array
    const cs = c.channel_scale || {};
    this.cscale = Float64Array.from(this.names, (nm, k) => (Array.isArray(cs) ? cs[k] : cs[nm]) ?? 1);
    this.dim = this.nk * this.nv;
    this.scale = new Float64Array(this.dim);
    // and per knot: noise_decay^j (< 1 explores the near term, where the next plans cannot fix a bad choice)
    for (let j = 0; j < this.nk; j++) for (let k = 0; k < this.nv; k++) this.scale[j * this.nv + k] = this.cscale[k] * (c.noise_decay ?? 1) ** j;
    this.lo = new Float64Array(this.dim); this.hi = new Float64Array(this.dim);
    for (let j = 0; j < this.nk; j++) { this.lo.set(this.clo, j * this.nv); this.hi.set(this.chi, j * this.nv); }
    this.ch = new Float64Array(this.nv); this.tmp = new Float64Array(this.dim);
  }

  /** Channel values at τ s after the plan start (beyond the last knot: held). */
  channels(theta, tau, out = this.ch) {
    const nv = this.nv, tk = this.tk, n = this.nk, t = tau + 1e-9;
    let j = 0;
    while (j < n - 1 && tk[j + 1] <= t) j++;
    if (!this.linear || j >= n - 1) {
      const o = j * nv;
      for (let k = 0; k < nv; k++) out[k] = theta[o + k];
    } else {
      const a = (tau - tk[j]) / (tk[j + 1] - tk[j]), o = j * nv;
      for (let k = 0; k < nv; k++) out[k] = theta[o + k] + (theta[o + nv + k] - theta[o + k]) * a;
    }
    return out;
  }

  /** Commands at τ into u: NU valve openings (+ brake, steer at NU, NU + 1 with drive; u must be NA long then). */
  command(theta, tau, u) {
    const x = this.channels(theta, tau), um = this.umax;
    if (this.drive) {
      const b = x[this.nm], s = x[this.nm + 1];
      u[DRIVE_IDX.brake] = b < 0 ? 0 : b > 1 ? 1 : b; u[DRIVE_IDX.steer] = s < -1 ? -1 : s > 1 ? 1 : s;
    }
    if (!this.M) { for (let i = 0; i < NU; i++) { const v = x[i]; u[i] = v < 0 ? 0 : v > um[i] ? um[i] : v; } return u; }
    const M = this.M;
    for (let i = 0; i < NU; i++) {
      let v = 0;
      for (let k = 0; k < 7; k++) v += M[i * 7 + k] * x[k];
      u[i] = v < 0 ? 0 : v > um[i] ? um[i] : v;
    }
    return u;
  }

  /** Warm start: re-sample the schedule dt s later (zoh at the new knots' midpoints, linear at the knot times). */
  shift(theta, dt) {
    const T = this.tmp, nv = this.nv, tk = this.tk, n = this.nk;
    for (let j = 0; j < n; j++) {
      const at = this.linear || j === n - 1 ? tk[j] : 0.5 * (tk[j] + tk[j + 1]);
      T.set(this.channels(theta, at + dt), j * nv);
    }
    theta.set(T);
    return theta;
  }

  /** Initial schedule: a number (every mixer channel; clamped into its box) or an array of nv channel values; the
   *  drive channels start at drive_init (default [0, 0]) unless the array covers them. */
  initial(init = 0, theta = new Float64Array(this.dim)) {
    for (let j = 0; j < this.nk; j++) for (let k = 0; k < this.nv; k++) {
      const v = Array.isArray(init) && k < init.length ? init[k] : k >= this.nm ? this.driveInit[k - this.nm] : init, e = j * this.nv + k;
      theta[e] = v < this.lo[e] ? this.lo[e] : v > this.hi[e] ? this.hi[e] : v;
    }
    return theta;
  }

  /** Clamp θ into the box (in place). */
  project(theta) {
    const lo = this.lo, hi = this.hi;
    for (let e = 0; e < this.dim; e++) { const v = theta[e]; theta[e] = v < lo[e] ? lo[e] : v > hi[e] ? hi[e] : v; }
    return theta;
  }
}
