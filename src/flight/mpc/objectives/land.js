// Soft-landing objective of the pure MPC (no base pilot: attitude, braking and steering all come from the plan).
// Interface shared by every objective (objectives/index.js):
//   reset(task)        episode start               observe(s)   every real control step (episode memory)
//   begin(s)           plan start                  start(tw, s) -> lane state L (after loadState)
//   step(L, tw, h, H, u) -> cost after control step h of a rollout (may set L.dead: the rollout stops)
//   terminal(L, tw, H) -> cost at the rollout's end
// Costs (weights below, all per control step unless 'once' / 'terminal'; tilt terms use 1 - cos(tilt)):
//   gas        w_gas · kg used
//   airborne   w_tilt · (1 - cos tilt) + w_rate · |ω|² (+ w_slip · sin²(slip), soft body)
//   crash      once at the first hull strike: w_crash · (2 - h/H) (earlier is worse), then the lane is dead
//   touchdown  once at the first street contact: w_vy · max(0, |v_y| - v_ok)² + w_vy2 · v_y² + w_td_tilt · (1 - cos)
//              + w_td_rate · |ω|² (+ w_td_slip · sin²(slip), w_pad · pad distance² on pad tasks)
//   ground     after touchdown: w_gnd_tilt · (1 - cos tilt) + w_gnd_rate · |ω|² (+ w_gnd_slip, w_gnd_yaw · ω_y²)
//   terminal   still airborne: w_air + w_air_h · h (the plan has to land inside the horizon) + w_stop · the excess of the
//              deceleration a stop needs over stop_frac of the bottom thrust + w_deadline · s past the task's
//              landing deadline; kerbs where the COM rolls within curb_lookahead s (curb_steps probes, curb_margin
//              m footprint): w_curb; pad tasks: ballistic landing-point distance² · w_pad / 2.
// w_air / w_stop shape plans whose horizon ends in the air; with a horizon that reaches the touchdown they only
// rule out hovering. w_stop prices "late" states (a bias towards braking earlier) but with the default 3 s horizon
// it never fires: the gas study (experiments/reports/gas-late-burn) got bit-identical episodes with it at 0.
// bottomThrustNominal() is the full-open sum (34.9 kN); the largest pitch-free bottom thrust is ~26.6 kN (rear-heavy
// kit, spec/README.md §2).
import { heightOf } from '../../tasks.js';
import { streetLevel, GROUND } from '../../world.js';
import { slip2 } from '../../controllers/mppi.js';
import { GROUPS, NU } from '../../contract.js';
import { Propulsion } from '../../propulsion.js';
import { WORLD, PROPULSION } from '../../../core/spec.js';

// Tuned for the pure MPC on the twin (escape-soft-landing, 6 presets x 3 seeds: 94 % success, 47 kg, vs pd 72 %,
// 48 kg): without a base pilot the attitude terms have to be much stronger than in controllers/mppi.js (40 / 3 /
// 0.3 there), and a plan that ends in the air must cost more than any landing (w_air) or it learns to hover.
export const LAND_DEFAULTS = {
  w_gas: 0.3, w_tilt: 3, w_rate: 0.05, w_slip: 0,
  w_crash: 60, w_vy: 6, v_ok: 1.0, w_vy2: 0, w_td_tilt: 400, w_td_rate: 10, w_td_slip: 0, w_pad: 2,
  w_gnd_tilt: 20, w_gnd_rate: 0.05, w_gnd_slip: 0, w_gnd_yaw: 0,
  w_air: 50, w_air_h: 20, w_stop: 0.5, stop_frac: 0.7, w_deadline: 20,
  w_curb: 8, curb_lookahead: 1.0, curb_steps: 2, curb_margin: 0,
};

/** Full-open thrust of the bottom group at the regulated feed pressure, full tank (N). */
export function bottomThrustNominal() {
  const P = new Propulsion(), u = new Float64Array(NU).fill(1);
  P.valve.fill(1);
  const T = P.step(u, 1e-9);
  let F = 0;
  for (const i of GROUPS.bottom) F += T[i];
  return F;
}

const cosTilt = q => 1 - 2 * (q[0] * q[0] + q[2] * q[2]);

export function createLand(cfg = {}) {
  const c = { ...LAND_DEFAULTS, ...cfg };
  const FbNom = bottomThrustNominal(), g = WORLD.gravity, pSet = PROPULSION.regulator.p_set;
  let task = null;
  const obj = {
    name: 'land', config: c,
    reset(t) { task = t || null; },
    observe() {},
    start(tw) { return { dead: false, td0: tw.touchdown !== null, td: tw.touchdown !== null, m: tw.prop.m }; },

    step(L, tw, h, H) {
      const q = tw.q, w = tw.w, ct = cosTilt(q), r2 = w[0] * w[0] + w[1] * w[1] + w[2] * w[2];
      let cs = c.w_gas * (L.m - tw.prop.m);
      L.m = tw.prop.m;
      const td = tw.touchdown;
      if (td && !L.td) { // first street contact of this rollout (priced before a crash in the same step: a harder
        // impact must cost more, or every crashing sample looks alike and the cheapest-gas one wins)
        L.td = true;
        const vy = Math.abs(td.v[1]), ex = vy - c.v_ok > 0 ? vy - c.v_ok : 0, tq = td.q, tw2 = td.w;
        cs += c.w_vy * ex * ex + c.w_vy2 * vy * vy + c.w_td_tilt * (1 - cosTilt(tq)) + c.w_td_rate * (tw2[0] * tw2[0] + tw2[1] * tw2[1] + tw2[2] * tw2[2]);
        if (c.w_td_slip) cs += c.w_td_slip * slip2(tq, td.v);
        if (task?.pad && c.w_pad) { const dx = td.p[0] - task.pad.center[0], dz = td.p[2] - task.pad.center[2]; cs += c.w_pad * (dx * dx + dz * dz); }
      }
      if (tw.hullStrike !== null) { L.dead = true; return cs + c.w_crash * (2 - h / H); }
      if (!L.td) {
        cs += c.w_tilt * (1 - ct) + c.w_rate * r2;
        if (c.w_slip) cs += c.w_slip * slip2(q, tw.v);
      } else {
        cs += c.w_gnd_tilt * (1 - ct) + c.w_gnd_rate * r2;
        if (c.w_gnd_slip || c.w_gnd_yaw) cs += c.w_gnd_slip * slip2(q, tw.v) + c.w_gnd_yaw * w[1] * w[1];
      }
      return cs;
    },

    terminal(L, tw) {
      if (L.dead) return 0;
      const p = tw.p, v = tw.v, q = tw.q;
      let cst = 0;
      if (!L.td) {
        const hb = heightOf(p, q, tw.mp.com), hp = hb > 0 ? hb : 0;
        cst += c.w_air + c.w_air_h * hp;
        if (c.w_stop) { // can the remaining descent still be stopped with stop_frac of the bottom thrust?
          const r = tw.prop.pc / pSet, Fb = FbNom * (r < 1 ? r : 1), ct = cosTilt(q);
          const aAv = c.stop_frac * Fb * (ct > 0 ? ct : 0) / tw.mp.M - g;
          const hm = hb - 0.3 > 0.05 ? hb - 0.3 : 0.05, aReq = v[1] < 0 ? v[1] * v[1] / (2 * hm) : 0, ex = aReq - aAv;
          cst += c.w_stop * (ex > 0 ? ex : 0);
        }
        const tNeed = (task?.horizon ?? 1e9) - (task?.success?.upright_hold ?? 0) - 0.3, late = tw.k * tw.dt - tNeed;
        if (late > 0) cst += c.w_deadline * late;
        if (task?.pad && c.w_pad) { // ballistic landing point
          const nv = -v[1] > 1 ? -v[1] : 1, tHit = v[1] < 0 ? hp / nv : 1;
          const dx = p[0] + v[0] * tHit - task.pad.center[0], dz = p[2] + v[2] * tHit - task.pad.center[2];
          cst += 0.5 * c.w_pad * (dx * dx + dz * dz);
        }
      }
      if (c.w_curb && curbAhead(p, v, c)) cst += c.w_curb;
      return cst;
    },
  };
  return obj;
}

/** Will the COM (± curb_margin) cross a raised street box within curb_lookahead s of flying / rolling straight? */
export function curbAhead(p, v, c) {
  const la = c.curb_lookahead, n = c.curb_steps, m = c.curb_margin;
  for (let i = n; i >= 1; i--) {
    const f = la * i / n, x = p[0] + v[0] * f, z = p[2] + v[2] * f;
    if (streetLevel(x, z) > GROUND.y) return true;
    if (m > 0 && (streetLevel(x + m, z) > GROUND.y || streetLevel(x - m, z) > GROUND.y || streetLevel(x, z + m) > GROUND.y || streetLevel(x, z - m) > GROUND.y)) return true;
  }
  return false;
}
