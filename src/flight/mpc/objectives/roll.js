// Barrel-roll objective of the pure MPC (barrel-roll work stream, 2026-10-20 round): leave the viaduct, JUMP (climb
// above the deck edge), ROLL 360° about the car's long axis (body +Z) and land softly. No feedback law anywhere: the
// terms below only price rollouts of the open-loop schedule; closed-loop behaviour comes from re-planning.
// Interface: header of objectives/land.js. The landing part is delegated to a `land` objective (gas, touchdown,
// crash, ground, kerbs, end-in-the-air) whose airborne attitude terms are switched off: this file prices the
// attitude itself, because during the roll "upright" is the wrong reference.
//
// Measurements (same definitions in tools/flight/roll/metrics.mjs, so the report measures what is optimised):
//   roll φ   unwrapped angle of world up seen in the body cross-section, φ = atan2(u_x, u_y), u = Rᵀŷ: for a pure
//            rotation by φ about body +Z, u = (sin φ, cos φ, 0); + = right side up. Unwrapped step by step (a 20 ms
//            control step turns < 0.3 rad even at 15 rad/s, far below π); if the long axis points within ~13° of
//            vertical the angle is undefined and the step falls back to ω_z·dt. Geometric, so it does not drift on
//            the soft body's noisy ω estimate. Episode memory (observe) counts from the first armed state, every
//            rollout continues from it. The roll to do: e = 2π·turns − dir·φ (dir −1: right side down, the way the
//            deck edge already throws the car, ~−2.3 rad/s on `flat`; dir 0: either way, e = 2π·turns − |φ|).
//   jump     climb = max COM height over the episode and the rollout minus the COM height at arming (the edge).
// Costs (per control step unless 'terminal'):
//   delegated  land with w_tilt = w_rate = w_slip = 0: gas, touchdown (vy, tilt, rate, slip), crash, ground terms,
//              end in the air / stop / deadline / kerb terms (objectives/land.js header)
//   airborne   blend (1 − λ)·A_roll + λ·A_land with λ = smoothstep over the last roll_blend_deg of the roll (λ = 1 once
//              e ≤ 0): A_roll = w_pitch·u_z² (the long axis stays level: a roll, not a cartwheel) + w_xyrate·(ω_x² + ω_y²)
//              + w_roll_run·e⁺ (a little hurry against putting the roll off; 0 by default: it did not help);
//              A_land = land's w_tilt·(1 − cos tilt) + w_rate·|ω|² (+ w_slip). The blend makes the switch from "roll"
//              to "come upright and stop rotating" continuous: a hard switch made plans avoid finishing the roll.
//   inverted   w_inv·Σu_bottom·(−cos tilt)⁺: bottom thrust while upside down pushes the car at the street and has to be
//              braked again (gas alone did not stop it: 19-24 kg of a 100 kg flight went into it, w_gas 0.3-1.5 alike)
//   wrong way  w_wrong·(roll against dir since arming, rad)²: a counter-roll (seen in cold first plans as +50° before
//              the −360°) tilts the jump thrust back towards the viaduct
//   deck       w_deck·(halfWidth + deck_margin − z)⁺² while the COM is less than deck_below under the deck top: the car
//              leaves on the +Z side and must not drift back into the parapet / slab edge while it is up there (the jump
//              keeps it at deck height ~1.5 s longer than a plain fall; a hull strike alone is priced too late)
//   climb      w_climb·(y_edge + h_jump − y)⁺ while the climb is not reached, during the first climb_window s after
//              arming: a dense version of the jump term (a cold first plan's samples with a little heave never climb,
//              so the terminal jump term alone gives them all the same cost and no direction)
//   clearance  (opt-in, 0) w_clear·(clear_margin − h)⁺² (h in cm: the lowest of the rear hull points clear_points above the street)
//              for clear_window s after the real touchdown (soft body: the rear-heavy car's rear floor bottoms out
//              ~0.2 s after a touchdown the twin calls soft; the twin rests at 7.7 cm and dips to 4.5 cm at 1.2 m/s)
//   slip       (opt-in, 0) land's w_td_slip / w_gnd_slip / w_gnd_yaw, and w_slip airborne once the roll is done (λ):
//              the roll's sideways thrust turns the velocity by 10-30° while the heading stays, the soft body's tyres
//              then load sideways at touchdown (a wheel came off at 32° of slip); the twin does not mind. Both terms
//              fixed the soft-body case they target but failed others and cost the twin 10-20 % (report §5.2).
//   terminal   w_roll·(|e_td| − roll_tol)⁺² with e at the first touchdown (or the crash / horizon end): the roll must
//              be complete (and not overdone) when the wheels arrive; w_jump·(h_jump − climb)⁺²; with jump_first,
//              w_order·(roll turned before the jump was committed − order_free, rad)²: committed = the ballistic apex
//              y + v_y²/2g reaches y_edge + h_jump, i.e. the jump burn is over; the roll may then go on during the climb.
import { createLand, LAND_DEFAULTS } from './land.js';
import { slip2 } from '../../controllers/mppi.js';
import { GROUPS } from '../../contract.js';
import { WORLD, VEHICLE } from '../../../core/spec.js';
import { streetLevel } from '../../world.js';

// Tuned on the twin (flat, experiments/configs/roll-tune*.json, report experiments/reports/barrel-roll): with the MPC
// settings of experiments/configs/roll-final.json 10/10 flat seeds (19/20 on 1-20) jump >= 2 m, roll 360° and land
// (~90 kg of gas); soft body: public/sims/flight-mpc-roll.sim succeeded, 4/6 over the report's soft-body cases, on the
// 2026-10-20 nozzle layout; since the bottom pairs moved (2026-10-21) it strikes the rear floor after touchdown
// (twin 9/10; heave exploration x1.3: twin 10/10, soft body 0/6), see the report's 2026-10-21 note.
// Why these values: w_jump 200 (at 40 a climb 0.1-0.5 m short was cheaper than the gas), w_climb 3 (cold start),
// jump_first (else the plan rolls at once and the thrust never points up long enough to jump), w_wrong 20 / w_deck 2
// (the counter-roll / drift back into the parapet), w_inv 2 (upside-down thrust), w_roll_run 0 (the hurry term did
// not help), w_gas 0.6 (land: 0.3). Land terms not listed here keep LAND_DEFAULTS (v_ok 1 m/s: softer touchdowns
// cost ~10 kg more and hit the regulator dropout).
export const ROLL_DEFAULTS = {
  ...LAND_DEFAULTS, w_gas: 0.6,
  dir: -1, turns: 1,
  w_roll: 40, roll_tol_deg: 5, roll_blend_deg: 60, w_roll_run: 0,
  w_pitch: 3, w_xyrate: 0.05,
  h_jump: 2.5, w_jump: 200, w_climb: 3, climb_window: 1.0,
  jump_first: true, order_free_deg: 90, w_order: 5,
  w_inv: 2, w_wrong: 20, w_deck: 2, deck_margin: 2.0, deck_below: 2.0,
  w_clear: 0, clear_margin: 0.07, clear_window: 0.5, clear_points: [2, 3, 19],
};

const TAU = 2 * Math.PI;
/** Roll angle of q about body +Z relative to world up (rad, (−π, π]); + = right side up. */
export const rollRaw = q => Math.atan2(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[0] * q[0] + q[2] * q[2]));
/** Wrap an angle difference into [−π, π). */
export const wrapPi = d => d - TAU * Math.floor((d + Math.PI) / TAU);
const cosTilt = q => 1 - 2 * (q[0] * q[0] + q[2] * q[2]);
const G = WORLD.gravity, BOTTOM = GROUPS.bottom, DECK = WORLD.deck;
/** Ballistic apex of the COM from (y, v_y): where a coast from here tops out. */
const apexOf = (y, vy) => (vy > 0 ? y + (vy * vy) / (2 * G) : y);

/** Advance the unwrapped roll of a holder {phi, raw} to orientation q (body roll rate wz over dt as fallback). */
export function rollAdvance(o, q, wz, dt) {
  const ux = 2 * (q[0] * q[1] + q[2] * q[3]), uy = 1 - 2 * (q[0] * q[0] + q[2] * q[2]);
  const r = Math.atan2(ux, uy);
  o.phi += ux * ux + uy * uy > 0.05 ? wrapPi(r - o.raw) : wz * dt;
  o.raw = r;
  return o.phi;
}

export function createRoll(cfg = {}, ctx = {}) {
  const c = { ...ROLL_DEFAULTS, ...cfg };
  const inner = createLand({ ...c, w_tilt: 0, w_rate: 0, w_slip: 0 }, ctx);
  const target = TAU * c.turns, tol = (c.roll_tol_deg * Math.PI) / 180, blend = (c.roll_blend_deg * Math.PI) / 180;
  const free = (c.order_free_deg * Math.PI) / 180, HP = c.clear_points.map(j => VEHICLE.hull.points[j]);
  // episode memory (real states only)
  const ep = { phi: 0, raw: 0, t: null, t0: null, phi0: 0, yEdge: null, apex: -Infinity, orderPhi: null, tTd: null };
  const left = phi => target - (c.dir ? c.dir * phi : Math.abs(phi)); // roll still to do (rad)
  const obj = {
    name: 'roll', config: c, episode: ep,
    reset(t) { inner.reset(t); ep.phi = ep.raw = ep.phi0 = 0; ep.t = ep.t0 = null; ep.yEdge = null; ep.apex = -Infinity; ep.orderPhi = null; ep.tTd = null; },

    observe(s) {
      inner.observe(s);
      if (!s.armed) return;
      if (ep.tTd === null && s.phase === 'ground') ep.tTd = s.t; // the real touchdown (a plan from the ground starts touched down)
      if (ep.t === null) { ep.raw = ep.phi = ep.phi0 = rollRaw(s.q); ep.yEdge = s.p[1]; ep.t0 = s.t; }
      else rollAdvance(ep, s.q, s.w[2], s.t - ep.t);
      ep.t = s.t;
      if (s.p[1] > ep.apex) ep.apex = s.p[1];
      // roll done before the jump was committed (for jump_first): frozen once a coast would reach the climb
      if (ep.orderPhi === null && apexOf(s.p[1], s.v[1]) - ep.yEdge >= c.h_jump) ep.orderPhi = ep.phi;
    },

    begin(s) { inner.begin?.(s); },

    start(tw, s) {
      const L = inner.start(tw, s);
      L.phi = ep.phi; L.raw = ep.raw; L.apex = ep.apex; L.phiEnd = null; L.orderPhi = ep.orderPhi;
      L.yEdge = ep.yEdge ?? s.p[1];
      return L;
    },

    step(L, tw, h, H, u) {
      let cs = inner.step(L, tw, h, H, u);
      const q = tw.q, w = tw.w;
      rollAdvance(L, q, w[2], tw.dt * tw.nsub);
      if (c.w_climb && L.apex - L.yEdge < c.h_jump && tw.k * tw.dt - ep.t0 <= c.climb_window) { // dense jump shaping
        const d = L.yEdge + c.h_jump - tw.p[1];
        if (d > 0) cs += c.w_climb * d;
      }
      if (tw.p[1] > L.apex) L.apex = tw.p[1];
      if (L.orderPhi === null && apexOf(tw.p[1], tw.v[1]) - L.yEdge >= c.h_jump) L.orderPhi = L.phi;
      if ((L.td || L.dead) && L.phiEnd === null) L.phiEnd = L.phi; // the roll counts until the first contact
      if (c.w_clear && L.td && !L.dead) { // rear floor clearance just after the touchdown
        const t = tw.k * tw.dt, t0 = ep.tTd ?? tw.touchdown.t;
        if (t - t0 <= c.clear_window) {
          const q = tw.q, cm = tw.mp.com, r3 = 2 * (q[0] * q[1] + q[2] * q[3]), r4 = 1 - 2 * (q[0] * q[0] + q[2] * q[2]), r5 = 2 * (q[1] * q[2] - q[0] * q[3]);
          const ys = streetLevel(tw.p[0], tw.p[2]);
          let hm = Infinity;
          for (const H of HP) { const y = tw.p[1] + r3 * (H[0] - cm[0]) + r4 * (H[1] - cm[1]) + r5 * (H[2] - cm[2]) - ys; if (y < hm) hm = y; }
          const d = (c.clear_margin - hm) * 100; // cm
          if (d > 0) cs += c.w_clear * d * d;
        }
      }
      if (!L.td && !L.dead) {
        const e = left(L.phi), ct = cosTilt(q);
        const x = e <= 0 ? 1 : e >= blend ? 0 : 1 - e / blend, lam = x * x * (3 - 2 * x);
        const uz = 2 * (q[1] * q[2] - q[0] * q[3]);
        let a = 0;
        if (lam < 1) a += (1 - lam) * (c.w_pitch * uz * uz + c.w_xyrate * (w[0] * w[0] + w[1] * w[1]) + c.w_roll_run * (e > 0 ? e : 0));
        if (lam > 0) {
          let al = c.w_tilt * (1 - ct) + c.w_rate * (w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
          if (c.w_slip) al += c.w_slip * slip2(q, tw.v);
          a += lam * al;
        }
        if (c.w_wrong && c.dir) { const wr = c.dir * (ep.phi0 - L.phi); if (wr > 0) a += c.w_wrong * wr * wr; } // counter-roll
        if (c.w_deck && tw.p[1] > DECK.top - c.deck_below) { const d = DECK.halfWidth + c.deck_margin - tw.p[2]; if (d > 0) a += c.w_deck * d * d; }
        if (c.w_inv && ct < 0) { let ub = 0; for (const i of BOTTOM) ub += u[i]; a += c.w_inv * ub * -ct; } // bottom jets pushing the car down
        cs += a;
      }
      return cs;
    },

    terminal(L, tw, H) {
      let cs = inner.terminal(L, tw, H);
      const e = left(L.phiEnd ?? L.phi), ex = Math.abs(e) - tol;
      if (ex > 0) cs += c.w_roll * ex * ex;
      const dj = c.h_jump - (L.apex - L.yEdge);
      if (dj > 0) cs += c.w_jump * dj * dj;
      if (c.jump_first) { // roll turned before the jump was committed, beyond a free allowance
        const pre = Math.abs((L.orderPhi ?? L.phi) - ep.phi0) - free;
        if (pre > 0) cs += c.w_order * pre * pre;
      }
      return cs;
    },
  };
  return obj;
}
