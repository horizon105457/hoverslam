// Gas-study objective (gas-study work stream, 2026-10-20): the cheapest soft landing. Gas is the only quantity priced
// inside the feasible set; the landing requirements are near-hard hinge penalties that are exactly zero inside their
// caps, so no term rewards braking early or late for its own sake (cf. land.js: its w_stop prices "late" states and
// its w_vy / w_td_tilt are paid inside the task limits, both of which pull the plan towards early, gentle braking).
// Interface: header of objectives/land.js. Costs, in kg-equivalents (w_gas = 1 per kg):
//   gas        w_gas · kg used during the rollout (the task counts every kg of the episode)
//   touchdown  once, at the first street contact of the rollout, priced before a crash in the same step (a harder
//              impact must cost more than a softer one, else every crashing sample looks alike and the
//              cheapest-gas one wins): hinge(|v_y| - v_cap) + hinge(tilt - tilt_cap) + hinge(|ω| - rate_cap)
//              with hinge(e) = w1·e + w2·e² for e > 0, else 0; touchdown after the task deadline: w_late per s
//   hold       on the ground until hold s after the touchdown (the task's upright window): w_hold per rad of tilt
//              over hold_tilt_cap per control step; a hull strike before the hold ends: w_crash, lane dead; once the
//              hold is over nothing is priced and the rollout stops (the episode ends there)
//   end        still airborne at the horizon end: w_air + w_gas · the bang-bang gas-to-go of the vertical channel
//              from the end state (latest full pitch-free burn to v_cap, constant mass: the value function of the
//              1-D problem, so ending in the air is neither a shortcut nor a shaping term) + the impact hinge if even
//              a full burn from there cannot meet v_cap, + w_late per s past the deadline
//   optional   air_tilt · (1 - cos tilt) + air_rate · |ω|² per airborne step (shaping; 0 by default),
//              w_curb (land.js curbAhead) at the end (0 by default: the rollout's ground roll sees kerbs itself),
//              reserve (robustness, 0 by default): w_reserve · max(0, a_req - reserve · a_max) per airborne step, with
//              a_req = (v_y² - v_cap²) / 2h the deceleration still needed to reach v_cap at the street and a_max the
//              net deceleration of a full pitch-free burn; unlike umax.bottom (a ceiling on the command itself) the
//              plan brakes with reserve·a_max but a replan may still command full thrust when the plant falls short.
// Remaining bias, by construction: the caps (v_cap below the task's 2 m/s, the twin strikes its rear hull from
// ~1.7-1.9 m/s), the horizon (a landing beyond it is priced by the 1-D gas-to-go, which ignores attitude) and the
// planning model (plan_dt, rigid twin). lastLane (diagnostics): the parts of the most recent rollout.
import { heightOf } from '../../tasks.js';
import { curbAhead } from './land.js';
import { MassModel } from '../../massprops.js';
import { Propulsion } from '../../propulsion.js';
import { GROUPS, NU } from '../../contract.js';
import { WORLD, PROPULSION } from '../../../core/spec.js';

export const GAS_DEFAULTS = {
  w_gas: 1,
  v_cap: 1.5, w_v1: 30, w_v2: 300,
  tilt_cap_deg: 6, w_tilt1: 200, w_tilt2: 2000,
  rate_cap: 0.5, w_rate1: 30, w_rate2: 300,
  hold: null, hold_tilt_cap_deg: 8, w_hold: 50,
  w_crash: 1000, w_air: 100, w_late: 100,
  air_tilt: 0, air_rate: 0, reserve: 0, w_reserve: 2,
  w_curb: 0, curb_lookahead: 1.0, curb_steps: 2, curb_margin: 0,
};

const cosTilt = q => 1 - 2 * (q[0] * q[0] + q[2] * q[2]);
const tiltAng = q => { const c = cosTilt(q); return Math.acos(c > 1 ? 1 : c < -1 ? -1 : c); };
const hinge = (e, w1, w2) => (e > 0 ? w1 * e + w2 * e * e : 0);

/** Pitch-free heave of the bottom group at full opening, full tank: thrust (N) and mass flow (kg/s). The pair with
 *  the longer arm about the COM is throttled: full tank, the front one to (c_z - z_r) / (z_f - c_z) ≈ 0.94 (tools/flight/gas). */
export function heaveFull() {
  const P = new Propulsion(), cz = new MassModel().at(P.m0).com[2], B = GROUPS.bottom, z = B.map(i => PROPULSION.nozzles[i].exit[2]);
  const zf = Math.max(...z), zr = Math.min(...z), u = new Float64Array(NU);
  const f = Math.min(1, (cz - zr) / (zf - cz)), r = Math.min(1, (zf - cz) / (cz - zr));
  B.forEach((i, k) => { u[i] = z[k] === zf ? f : r; });
  P.valve.set(u);
  const T = P.step(u, 1e-9);
  let F = 0, md = 0;
  for (const i of B) { F += T[i]; md += P.mdot[i]; }
  return { F, mdot: md };
}

export function createGas(cfg = {}, ctx = {}) {
  const c = { ...GAS_DEFAULTS, ...cfg }, g = WORLD.gravity, D = Math.PI / 180;
  const tiltCap = c.tilt_cap_deg * D, holdCap = c.hold_tilt_cap_deg * D, HF = heaveFull();
  let task = ctx.task || null, hold = 1, tLate = Infinity, tdReal = null, prevPhase = null;

  /** Bang-bang gas-to-go of the vertical channel from height h (above the street) with vertical speed vy, mass M:
   *  [gas, impact hinge]; constant mass and thrust (a lower bound of the real burn, see tools/flight/gas). */
  const toGo = (h, vy, M) => {
    const a = HF.F / M - g, v0 = -vy, e2 = v0 * v0 + 2 * g * (h > 0 ? h : 0), cap2 = c.v_cap * c.v_cap;
    if (e2 <= cap2) return [0, 0]; // lands below the cap in free fall
    const hi = (e2 - cap2) / (2 * (g + a));
    if (hi > h) { // already too late for a full burn: burn all the way, price the impact it still has
      const vi = v0 > 0 ? v0 : 0, vImp2 = vi * vi - 2 * a * (h > 0 ? h : 0), vImp = vImp2 > 0 ? Math.sqrt(vImp2) : 0;
      return [HF.mdot * Math.max(0, vi - vImp) / a, hinge(vImp - c.v_cap, c.w_v1, c.w_v2)];
    }
    const vi = Math.sqrt(cap2 + 2 * a * hi);
    return [HF.mdot * (vi - c.v_cap) / a, 0];
  };

  const obj = {
    name: 'gas', config: c, lastLane: null,
    reset(t) {
      task = t || null; tdReal = null; prevPhase = null;
      hold = c.hold ?? task?.success?.upright_hold ?? 1;
      tLate = (task?.horizon ?? Infinity) - hold - 0.3; // latest touchdown that still completes the hold in the episode
    },
    observe(s) { // the real touchdown time (the planning twin's loadState forgets it): first 'ground' after 'air'
      if (tdReal === null && s.phase === 'ground' && prevPhase === 'air') tdReal = s.t;
      prevPhase = s.phase;
    },
    start(tw) {
      const td0 = tw.touchdown !== null;
      const L = { dead: false, td: td0, tTD: td0 ? (tdReal ?? tw.k * tw.dt) : null, m: tw.prop.m,
        parts: { gas: 0, vtd: 0, tilt: 0, rate: 0, hold: 0, crash: 0, air: 0, late: 0, shape: 0 } };
      obj.lastLane = L;
      return L;
    },

    step(L, tw, h, H) {
      const P = L.parts, dm = L.m - tw.prop.m;
      let cs = c.w_gas * dm;
      P.gas += c.w_gas * dm; L.m = tw.prop.m;
      const td = tw.touchdown;
      if (td && !L.td) {
        L.td = true; L.tTD = td.t;
        const cv = hinge(Math.abs(td.v[1]) - c.v_cap, c.w_v1, c.w_v2), ct = hinge(tiltAng(td.q) - tiltCap, c.w_tilt1, c.w_tilt2);
        const w = td.w, cr = hinge(Math.sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]) - c.rate_cap, c.w_rate1, c.w_rate2);
        const cl = td.t > tLate ? c.w_late * (td.t - tLate) : 0;
        P.vtd += cv; P.tilt += ct; P.rate += cr; P.late += cl;
        cs += cv + ct + cr + cl;
      }
      const t = tw.k * tw.dt;
      if (L.td && t > L.tTD + hold + 1e-9) { L.dead = true; return cs; } // hold over: the episode has ended
      if (tw.hullStrike !== null) { L.dead = true; P.crash += c.w_crash; return cs + c.w_crash; }
      if (L.td) {
        const e = tiltAng(tw.q) - holdCap;
        if (e > 0) { P.hold += c.w_hold * e; cs += c.w_hold * e; }
      } else {
        if (c.air_tilt || c.air_rate) {
          const w = tw.w, sh = c.air_tilt * (1 - cosTilt(tw.q)) + c.air_rate * (w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
          P.shape += sh; cs += sh;
        }
        if (c.reserve && tw.v[1] < -c.v_cap) {
          const hb = heightOf(tw.p, tw.q, tw.mp.com), hm = hb > 0.05 ? hb : 0.05, vy = tw.v[1];
          const ex = (vy * vy - c.v_cap * c.v_cap) / (2 * hm) - c.reserve * (HF.F / tw.mp.M - g);
          if (ex > 0) { P.shape += c.w_reserve * ex; cs += c.w_reserve * ex; }
        }
      }
      return cs;
    },

    terminal(L, tw) {
      if (L.dead) return 0;
      let cst = 0;
      const P = L.parts;
      if (!L.td) {
        const hb = heightOf(tw.p, tw.q, tw.mp.com), [gg, imp] = toGo(hb, tw.v[1], tw.mp.M), t = tw.k * tw.dt;
        const cl = t > tLate ? c.w_late * (t - tLate) : 0;
        cst += c.w_air + c.w_gas * gg + imp + cl;
        P.air += c.w_air + c.w_gas * gg + imp; P.late += cl;
      }
      if (c.w_curb && curbAhead(tw.p, tw.v, c)) cst += c.w_curb;
      return cst;
    },
  };
  obj.reset(task);
  return obj;
}
