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
//   crash      also at the first wheel overload (twin failure thresholds, spec/vehicle.json failure: a control arm /
//              spring the soft body would break; the rigid twin cannot model the break, only its load)
//   sideslip   touchdown: v_lat = the largest contact-patch speed across a wheel (lateral axle direction, wheel
//              bottoms, at the first street contact): w_td_lat · (v_lat − td_lat_ok)⁺², a crash once v_lat >
//              td_lat_crash; ground, from gnd_lat_after s after the real touchdown on: per wheel e = EMA (½ per control
//              step) of |F_lat|, w_gnd_lat · Σ ((e − gnd_lat_ok)/1 kN)⁺², a crash once any e > gnd_lat_crash. The
//              thresholds are soft-body failure envelopes as the planning twin (2 ms, per control step) sees them.
//              coldgas-rigid@3 (2026-10-21, kit rear arms x2, flight connector strength 0.55; tools/flight/calib/
//              criteria.mjs, experiments/reports/drive-brakes-steering): a control arm (now mostly a front one) breaks
//              at impact when the tyres must kill a sideways contact speed (best split 2.1 m/s on 72 designed
//              landings, held out 0.77; crash at 2.0, penalty from 1.0 = 90 % recall), and later when yaw
//              divergence or cornering loads a wheel sideways (soft-body drive tests: every broken arm at an EMA
//              >= 3.17 kN, intact braking / steering <= 2.25 kN; crash 3.2 kN, penalty from 2.2; landings: 4.3 kN).
//              Before (coldgas-rigid@2, stock rear arms, the DOE at connector strength 1): 5.8 / 4.0 / 1.2 m/s and
//              2.9 / 1.7 kN. The v1 twin (Coulomb tyres) took sideways landings for free: the soft-body MPC landed
//              at 14-89° of slip and lost a wheel in 0.1-0.4 s.
//   margins    (opt-in, 0) for floor_window s after the rollout's touchdown: w_floor · ((floor_margin − h)/1 cm)⁺² per
//              step, h = lowest underbody point (hull points below body y 0.35) above the street under the COM;
//              terminal w_lat · Σ_wheels (peak |F_lat| / wheel_lat_max − lat_frac)⁺² over the rollout. The margins
//              cover the twin's prediction error measured on held-out soft-body landings (tools/flight/calib)
//   terminal   still airborne: w_air + w_air_h · h (the plan has to land inside the horizon) + w_stop · the excess of the
//              deceleration a stop needs over stop_frac of the bottom thrust + w_deadline · s past the task's
//              landing deadline; kerbs where the COM rolls within curb_lookahead s (curb_steps probes, curb_margin
//              m footprint): w_curb; pad tasks: ballistic landing-point distance² · w_pad / 2.
//   drive      (opt-in, 0 / false; with the MPC's brake / steer channels, `drive: true`) after the rollout's touchdown:
//              kerb_crash: a wheel standing on a sidewalk / median top (the twin's kerb event, spec/README.md 5.13) is a
//              crash (the soft body breaks a control arm on a 16 cm kerb at speed); w_kerb · Σ_wheels ((kerb_margin −
//              d)/kerb_margin)⁺² per step, d = horizontal distance of the wheel centre from the nearest raised street
//              box; w_dist · metres travelled per step (brake early: the kit car is yaw-unstable while rolling fast);
//              terminal w_stop_kerb · (v²/(2 a_stop) + kerb_margin − d_ray)⁺² with d_ray the distance from the COM along
//              its velocity to the first raised box (the stop must fit in front of the kerb; also priced for plans that end
//              in the air when stop_kerb_air, so the landing is placed where the stop fits), w_v_end · v²; a crash after
//              the touchdown adds w_crash_v · v² (an unavoidable kerb hit still pays for every m/s braked off);
//              w_kerb_reach · the same overlap² per control step (· CONTROL_DT), in the air too: dense, so plans that die
//              at a kerb inside the horizon still learn to shift the landing with the side jets; stop tasks
//              (success.stop_speed): w_stop_late · (v − stop_speed)⁺ per step from stop_lead s before the task's
//              stop_within after the touchdown (rolling on into the deadline must cost more than any braking).
// w_air / w_stop shape plans whose horizon ends in the air; with a horizon that reaches the touchdown they only
// rule out hovering. w_stop prices "late" states (a bias towards braking earlier) but with the default 3 s horizon
// it never fires: the gas study (experiments/reports/gas-late-burn) got bit-identical episodes with it at 0.
// bottomThrustNominal() is the full-open sum (34.9 kN); the largest pitch-free bottom thrust is 33.7-34.9 kN over
// the burn (the bottom pairs bracket the moving COM, spec/README.md §2).
import { heightOf } from '../../tasks.js';
import { streetLevel, GROUND, BOXES, KIND } from '../../world.js';
import { slip2 } from '../../controllers/mppi.js';
import { GROUPS, NU, CONTROL_DT } from '../../contract.js';
import { Propulsion } from '../../propulsion.js';
import { WORLD, PROPULSION, VEHICLE, GENERATED } from '../../../core/spec.js';

// Tuned for the pure MPC on the twin (escape-soft-landing, 6 presets x 3 seeds: 94 % success, 47 kg, vs pd 72 %,
// 48 kg): without a base pilot the attitude terms have to be much stronger than in controllers/mppi.js (40 / 3 /
// 0.3 there), and a plan that ends in the air must cost more than any landing (w_air) or it learns to hover.
export const LAND_DEFAULTS = {
  w_gas: 0.3, w_tilt: 3, w_rate: 0.05, w_slip: 0,
  w_crash: 60, w_vy: 6, v_ok: 1.0, w_vy2: 0, w_td_tilt: 400, w_td_rate: 10, w_td_slip: 0, w_pad: 2,
  w_gnd_tilt: 20, w_gnd_rate: 0.05, w_gnd_slip: 0, w_gnd_yaw: 0,
  w_air: 50, w_air_h: 20, w_stop: 0.5, stop_frac: 0.7, w_deadline: 20,
  w_curb: 8, curb_lookahead: 1.0, curb_steps: 2, curb_margin: 0,
  w_floor: 0, floor_margin: 0.03, floor_window: 2.0, w_lat: 0, lat_frac: 0.8,
  w_td_lat: 10, td_lat_ok: 1.0, td_lat_crash: 2.0, w_gnd_lat: 1, gnd_lat_ok: 2200, gnd_lat_crash: 3200, gnd_lat_after: 0.3,
  kerb_crash: false, w_kerb: 0, kerb_margin: 0.5, w_dist: 0, w_stop_kerb: 0, stop_kerb_air: false, a_stop: 6, w_v_end: 0, w_stop_late: 0, stop_lead: 3, w_crash_v: 0, w_kerb_reach: 0,
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
  const FbNom = bottomThrustNominal(), g = WORLD.gravity, pSet = PROPULSION.regulator.p_set, latMax = VEHICLE.failure?.wheel_lat_max ?? Infinity;
  let task = null, tTd = null; // tTd: real touchdown time (episode memory), so ground plans keep counting from it
  const obj = {
    name: 'land', config: c,
    reset(t) { task = t || null; tTd = null; },
    observe(s) { if (tTd === null && s.phase === 'ground') tTd = s.t; },
    start(tw) { return { dead: false, td0: tw.touchdown !== null, td: tw.touchdown !== null, m: tw.prop.m, ema: [0, 0, 0, 0], wxz: new Float64Array(8) }; },

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
        if (c.w_td_lat || c.td_lat_crash < Infinity) {
          const vl = contactLateralSpeed(tq, td.v, tw2, tw.mp.com), e = vl - c.td_lat_ok;
          if (e > 0) cs += c.w_td_lat * e * e;
          if (vl > c.td_lat_crash) { L.dead = true; return cs + c.w_crash * (2 - h / H); }
        }
      }
      if (tw.hullStrike !== null || tw.wheelOverload || (c.kerb_crash && tw.kerb)) {
        L.dead = true;
        if (c.w_crash_v && L.td) cs += c.w_crash_v * (tw.v[0] * tw.v[0] + tw.v[2] * tw.v[2]);
        return cs + c.w_crash * (2 - h / H);
      }
      if (c.w_kerb_reach) { // dense: the stop along the current velocity has to fit in front of the next kerb
        const v = tw.v, v2 = v[0] * v[0] + v[2] * v[2], vh = Math.sqrt(v2);
        if (vh > 0.1) { const e = v2 / (2 * c.a_stop) + c.kerb_margin - kerbRay(tw.p[0], tw.p[2], v[0] / vh, v[2] / vh, 60); if (e > 0) cs += c.w_kerb_reach * e * e * CONTROL_DT; }
      }
      if (L.td && (c.w_kerb || c.w_dist || c.w_stop_late)) {
        const v = tw.v, vh = Math.sqrt(v[0] * v[0] + v[2] * v[2]);
        if (c.w_dist) cs += c.w_dist * vh * CONTROL_DT;
        if (c.w_stop_late && task?.success?.stop_speed !== undefined && vh > task.success.stop_speed
          && tw.k * tw.dt - (tTd ?? tw.touchdown.t) > (task.success.stop_within ?? 8) - c.stop_lead) cs += c.w_stop_late * (vh - task.success.stop_speed);
        if (c.w_kerb) { const W = wheelsXZ(tw, L.wxz); for (let j = 0; j < 4; j++) { const e = 1 - kerbDistance(W[2 * j], W[2 * j + 1]) / c.kerb_margin; if (e > 0) cs += c.w_kerb * e * e; } }
      }
      if (L.td && (c.w_gnd_lat || c.gnd_lat_crash < Infinity)) { // wheel lateral load on the ground (yaw divergence)
        const late = tw.k * tw.dt - (tTd ?? tw.touchdown.t) >= c.gnd_lat_after, W = tw.wheels;
        for (let j = 0; j < 4; j++) {
          const f = W[j].lat, e = (L.ema[j] = 0.5 * L.ema[j] + 0.5 * (f < 0 ? -f : f));
          if (!late) continue;
          if (e > c.gnd_lat_crash) { L.dead = true; return cs + c.w_crash * (2 - h / H); }
          const x = (e - c.gnd_lat_ok) * 1e-3;
          if (x > 0) cs += c.w_gnd_lat * x * x;
        }
      }
      if (c.w_floor && L.td && tw.k * tw.dt - tw.touchdown.t < c.floor_window) {
        const e = c.floor_margin - underbodyClearance(tw);
        if (e > 0) cs += c.w_floor * (e * 100) * (e * 100);
      }
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
      if ((L.td || c.stop_kerb_air) && (c.w_stop_kerb || c.w_v_end)) { // the rest of the stop has to fit in front of the next kerb
        const v2 = v[0] * v[0] + v[2] * v[2], vh = Math.sqrt(v2);
        if (L.td) cst += c.w_v_end * v2;
        if (c.w_stop_kerb && vh > 0.1) { const e = v2 / (2 * c.a_stop) + c.kerb_margin - kerbRay(p[0], p[2], v[0] / vh, v[2] / vh, 60); if (e > 0) cst += c.w_stop_kerb * e * e; }
      }
      if (c.w_lat && latMax < Infinity) {
        const pk = tw.wheelPeak.lat;
        for (let j = 0; j < 4; j++) { const e = pk[j] / latMax - c.lat_frac; if (e > 0) cst += c.w_lat * e * e; }
      }
      return cst;
    },
  };
  return obj;
}

const WHEELS = GENERATED.massprops.wheels;
/** Largest horizontal speed across the axle (lateral tyre direction) of the four wheel bottoms (body-fixed points)
 *  for a state q, v, ω_B, COM com (m/s): what the tyres must absorb sideways at the touchdown. */
export function contactLateralSpeed(q, v, w, com) {
  const x = q[0], y = q[1], z = q[2], s = q[3];
  const R0 = 1 - 2 * (y * y + z * z), R1 = 2 * (x * y - z * s), R2 = 2 * (x * z + y * s), R3 = 2 * (x * y + z * s), R4 = 1 - 2 * (x * x + z * z);
  const R5 = 2 * (y * z - x * s), R6 = 2 * (x * z - y * s), R7 = 2 * (y * z + x * s), R8 = 1 - 2 * (x * x + y * y);
  const wx = R0 * w[0] + R1 * w[1] + R2 * w[2], wy = R3 * w[0] + R4 * w[1] + R5 * w[2], wz = R6 * w[0] + R7 * w[1] + R8 * w[2];
  const h = Math.sqrt(R0 * R0 + R6 * R6) || 1, ax = R0 / h, az = R6 / h;
  let m = 0;
  for (const W of WHEELS) {
    const bx = W.centre[0] - com[0], by = W.centre[1] - W.radius - com[1], bz = W.centre[2] - com[2];
    const rx = R0 * bx + R1 * by + R2 * bz, ry = R3 * bx + R4 * by + R5 * bz, rz = R6 * bx + R7 * by + R8 * bz;
    const vl = (v[0] + wy * rz - wz * ry) * ax + (v[2] + wx * ry - wy * rx) * az;
    if ((vl < 0 ? -vl : vl) > m) m = vl < 0 ? -vl : vl;
  }
  return m;
}

/** Lowest underbody point (twin hull points below body y 0.35) above the street level under the COM (m). */
export function underbodyClearance(tw) {
  const q = tw.q, p = tw.p, c = tw.mp.com;
  // R of the current q (tw.R holds the last substep's start): rows 3..5 suffice for heights
  const x = q[0], y = q[1], z = q[2], w = q[3];
  const r3 = 2 * (x * y + z * w), r4 = 1 - 2 * (x * x + z * z), r5 = 2 * (y * z - x * w);
  let h = Infinity;
  for (const P of tw.hull) if (P[1] < 0.35) { const yy = p[1] + r3 * (P[0] - c[0]) + r4 * (P[1] - c[1]) + r5 * (P[2] - c[2]); if (yy < h) h = yy; }
  return h - streetLevel(p[0], p[2]);
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

// raised street boxes (sidewalks, median): the kerbs of the drive terms
const KERBS = BOXES.filter(b => b.kind === KIND.street);
/** World (x, z) of the four wheel centres of the twin's current pose into out (8). */
export function wheelsXZ(tw, out) {
  const q = tw.q, p = tw.p, c = tw.mp.com, x = q[0], y = q[1], z = q[2], w = q[3];
  const R0 = 1 - 2 * (y * y + z * z), R1 = 2 * (x * y - z * w), R2 = 2 * (x * z + y * w), R6 = 2 * (x * z - y * w), R7 = 2 * (y * z + x * w), R8 = 1 - 2 * (x * x + y * y);
  for (let j = 0; j < 4; j++) {
    const C = WHEELS[j].centre, bx = C[0] - c[0], by = C[1] - c[1], bz = C[2] - c[2];
    out[2 * j] = p[0] + R0 * bx + R1 * by + R2 * bz; out[2 * j + 1] = p[2] + R6 * bx + R7 * by + R8 * bz;
  }
  return out;
}
/** Horizontal distance from (x, z) to the nearest raised street box footprint (0 inside). */
export function kerbDistance(x, z) {
  let d = Infinity;
  for (const b of KERBS) {
    const dx = x - b.c[0], dz = z - b.c[2], lx = b.cy * dx - b.sy * dz, lz = b.sy * dx + b.cy * dz;
    const ex = (lx < 0 ? -lx : lx) - b.e[0], ez = (lz < 0 ? -lz : lz) - b.e[2];
    const k = ex > 0 ? (ez > 0 ? Math.sqrt(ex * ex + ez * ez) : ex) : ez > 0 ? ez : 0;
    if (k < d) d = k;
  }
  return d;
}
/** Distance along the unit horizontal direction (dx, dz) from (x, z) to the first raised street box (cap tmax). */
export function kerbRay(x, z, dx, dz, tmax) {
  let t = tmax;
  for (const b of KERBS) {
    const ox = x - b.c[0], oz = z - b.c[2], lx = b.cy * ox - b.sy * oz, lz = b.sy * ox + b.cy * oz, ux = b.cy * dx - b.sy * dz, uz = b.sy * dx + b.cy * dz;
    let t0 = 0, t1 = t, ok = true;
    for (const [o, u, e] of [[lx, ux, b.e[0]], [lz, uz, b.e[2]]]) {
      if (Math.abs(u) < 1e-12) { if (o < -e || o > e) { ok = false; break; } continue; }
      let a = (-e - o) / u, c2 = (e - o) / u; if (a > c2) { const s2 = a; a = c2; c2 = s2; }
      if (a > t0) t0 = a; if (c2 < t1) t1 = c2; if (t0 > t1) { ok = false; break; }
    }
    if (ok && t0 < t) t = t0;
  }
  return t;
}
