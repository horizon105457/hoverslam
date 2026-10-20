// Heuristic attitude + descent autopilot on the four bottom nozzles, identical to python/coldgas/controllers/pd.py
// (parity-tested). Uses only FlightState fields + task + spec, so it runs unchanged on a state estimated from
// the soft body. Only + - * / sqrt min max: no libm differences between the languages.
//  - attitude: alpha = kp e - kd w (e: rotation taking body up to world up, |e| = sin tilt, -> 2 when inverted),
//    torque = diag(I) alpha, realised by a min-norm split over the bottom nozzles, shifted along the zero-torque
//    direction until no nozzle pulls, then scaled into the available thrust;
//  - vertical: hover tasks hold task.target.height, then land; escape tasks follow the braking profile
//    v_ref(h) = -sqrt(td^2 + 2 brake_frac a_max (h - flare)+), i.e. fall freely and burn as late as is safe.
import defaults from './defaults.json' with { type: 'json' };
import { PROPULSION, WORLD } from '../../core/spec.js';
import { Propulsion } from '../propulsion.js';
import { MassModel, invSym } from '../massprops.js';
import { GROUPS, NU, CONTROL_DT } from '../contract.js';
import { heightOf } from '../tasks.js';

export function createPD(cfg = {}) {
  const c = { ...defaults.pd, ...cfg };
  delete c._doc;
  const P = new Propulsion(PROPULSION, WORLD.air.pressure); // constants only
  const mass = new MassModel(), B = GROUPS.bottom, nb = B.length;
  const ex = B.map(i => PROPULSION.nozzles[i].exit);
  const mp = { M: 0, com: new Float64Array(3), I: new Float64Array(6), Iinv: new Float64Array(6) };
  const F1 = new Float64Array(nb), F = new Float64Array(nb), xs = new Float64Array(nb), zs = new Float64Array(nb);
  const S6 = new Float64Array(6), Ai = new Float64Array(6);
  const g = WORLD.gravity, reg = PROPULSION.regulator;
  let task = null, holdT = 0, landing = true;
  return {
    name: 'pd', config: c,
    reset({ task: t } = {}) { task = t || null; holdT = 0; landing = !(task && task.target); },
    /** Episode memory (hover hold timer, landing flag): MPPI copies the real pilot's into its rollout pilot. */
    get memory() { return { holdT, landing }; },
    set memory(m) { holdT = m.holdT; landing = m.landing; },
    act(s) {
      const u = new Float64Array(NU);
      if (!s.armed) return u;
      const qx = s.q[0], qy = s.q[1], qz = s.q[2], qw = s.q[3];
      const ux = 2 * (qx * qy + qz * qw), uy = 1 - 2 * (qx * qx + qz * qz), uz = 2 * (qy * qz - qx * qw); // world up, body axes
      const h = heightOf(s.p, s.q, s.comB);
      if (!landing) { // hover: hold the target until the task's hold time is done (same rule as the tracker)
        const err = h - task.target.height;
        holdT = Math.abs(err) <= task.target.tolerance ? holdT + CONTROL_DT : 0;
        if (holdT >= task.target.hold - 1e-9) landing = true;
      }
      if (landing && s.phase === 'ground') return u;
      // full-open thrust of each bottom nozzle at the current tank state (propulsion model with s = 1)
      let pc = Math.min(reg.p_set, s.tank.p - reg.dp_min);
      pc = pc > 2 * P.pa ? pc : 0;
      if (pc <= 0) return u;
      const Tc = s.tank.T;
      let Fa = 0;
      for (let k = 0; k < nb; k++) {
        const i = B[k];
        const md = P.At[i] * pc * P.choke / Math.sqrt(Tc), ve = P.Me[i] * Math.sqrt(P.g * P.R * Tc * P.TeRatio[i]);
        F1[k] = md * ve + (pc * P.peRatio[i] - P.pa) * P.Ae[i];
        if (!(F1[k] > 0)) return u; // over-expanded at a nearly empty tank: the nozzles no longer push
        Fa += F1[k];
      }
      mass.at(s.tank.m, mp);
      const M = mp.M;
      // attitude
      const sn = Math.sqrt(ux * ux + uz * uz);
      let kk = 1;
      if (uy < 0) kk = sn > 1e-6 ? (2 - sn) / sn : 0;
      let ea = uz * kk, eb = -ux * kk;
      if (uy < 0 && sn <= 1e-6) { ea = 0; eb = 2; }
      const tx = mp.I[0] * (c.att_kp * ea - c.att_kd * s.w[0]), tz = mp.I[2] * (c.att_kp * eb - c.att_kd * s.w[2]);
      // collective
      let Fc = 0;
      if (uy > c.tilt_cut && !(landing && h <= 0)) {
        let vref, aff = 0;
        if (!landing) {
          vref = c.hover_kh * (task.target.height - h);
          vref = vref > c.hover_vmax ? c.hover_vmax : vref < -c.hover_vmax ? -c.hover_vmax : vref;
        } else {
          const amax = Fa * uy / M - g, ab = c.brake_frac * (amax > 0.5 ? amax : 0.5), hh = h - c.flare_height;
          vref = -Math.sqrt(c.td_speed * c.td_speed + 2 * ab * (hh > 0 ? hh : 0));
          // feed-forward of the profile's own deceleration d(v_ref)/dt = ab v_y / v_ref while above the flare
          if (hh > 0 && s.v[1] < 0) { aff = (ab * s.v[1]) / vref; aff = aff > 2 * ab ? 2 * ab : aff; }
        }
        const a = g + aff + c.kv * (vref - s.v[1]);
        Fc = (M * (a > 0 ? a : 0)) / uy;
      }
      // allocation: rows (1, -z, x) for (collective, torque x, torque z)
      let sz = 0, szz = 0, sx = 0, sxx = 0, sxz = 0;
      for (let k = 0; k < nb; k++) {
        const x = ex[k][0] - s.comB[0], z = ex[k][2] - s.comB[2];
        xs[k] = x; zs[k] = z;
        sz += z; szz += z * z; sx += x; sxx += x * x; sxz += x * z;
      }
      S6[0] = nb; S6[1] = szz; S6[2] = sxx; S6[3] = -sz; S6[4] = sx; S6[5] = -sxz;
      invSym(S6, Ai);
      const y0 = Ai[0] * Fc + Ai[3] * tx + Ai[4] * tz, y1 = Ai[3] * Fc + Ai[1] * tx + Ai[5] * tz, y2 = Ai[4] * Fc + Ai[5] * tx + Ai[2] * tz;
      let lam = 0;
      for (let k = 0; k < nb; k++) {
        F[k] = y0 - zs[k] * y1 + xs[k] * y2;
        const w = Ai[0] - zs[k] * Ai[3] + xs[k] * Ai[4];
        if (F[k] < 0 && w > 0) { const l = -F[k] / w; if (l > lam) lam = l; }
      }
      let sc = 1;
      for (let k = 0; k < nb; k++) {
        F[k] += lam * (Ai[0] - zs[k] * Ai[3] + xs[k] * Ai[4]);
        if (F[k] > F1[k]) { const r = F1[k] / F[k]; if (r < sc) sc = r; }
      }
      for (let k = 0; k < nb; k++) {
        const v = (F[k] * sc) / F1[k];
        u[B[k]] = v > 1 ? 1 : v < 0 ? 0 : v;
      }
      return u;
    },
  };
}
