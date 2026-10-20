// Barrel-roll report metrics from a recorded flight: a rigid-twin trajectory (coldgas-traj@1) or a soft-body
// recording's thrust log (meta.propulsion, coldgas-log@1, + meta.flight). Same definitions as the MPC objective
// (src/flight/mpc/objectives/roll.js), so what the planner optimises is what the report measures:
//   roll φ    unwrapped angle of world up in the body cross-section, atan2(u_x, u_y) with u = Rᵀŷ (+ = right side
//             up, a rotation about body +Z); counted from the first sample after arming (the soft body's controller
//             sees nothing earlier); cross-check: ∫ω_z dt over the same samples (drifts with pitch / yaw, the
//             geometric angle does not)
//   jump      climb = apex of the COM height minus the COM height at arming (= the deck edge for arm 'edge');
//             apex_above_deck = COM apex minus the deck road surface (the COM rides ~0.5 m above it at rest)
//   timeline  t_edge (arming), t_apex, t_roll_start (|φ − φ_arm| ≥ 10°), t_roll_end (|φ| ≥ 360° − 10°), touchdown
//   success   manoeuvre = task landing success ∧ |φ_td| ≥ 360° − max_tilt ∧ climb ≥ h_jump (JUMP_REQ = 2.0 m by default)
import { WORLD, PROPULSION } from '../../../src/core/spec.js';
import { rollRaw, wrapPi } from '../../../src/flight/mpc/objectives/roll.js';

const DEG = 180 / Math.PI, REG = PROPULSION.regulator;
/** Jump requirement of the report (m of COM climb): the objective aims at h_jump = 2.5 m, 0.5 m of margin above it. */
export const JUMP_REQ = 2.0;

/** Common input: {t[], state[][13], tank[][3] (m, T, p), thrust[][10], events [{t, type}], metrics, armedAt}. */
export function rollSeries({ t, state, armedAt }) {
  const n = t.length, phi = new Float64Array(n), phiW = new Float64Array(n);
  let k0 = t.findIndex(x => x >= armedAt - 1e-9);
  if (k0 < 0) k0 = n;
  let raw = 0, ph = 0, iw = 0;
  for (let k = 0; k < n; k++) {
    const s = state[k], q = [s[3], s[4], s[5], s[6]];
    if (k < k0) { phi[k] = NaN; phiW[k] = NaN; continue; }
    const r = rollRaw(q);
    if (k === k0) { raw = r; ph = r; iw = r; } else { ph += wrapPi(r - raw); raw = r; iw += 0.5 * (s[12] + state[k - 1][12]) * (t[k] - t[k - 1]); }
    phi[k] = ph; phiW[k] = iw;
  }
  return { phi, phiW, k0 };
}

/** Pitch of the long axis above the horizon (deg) and tilt of body up from world up (deg). */
export function attitude(q) {
  const uz = 2 * (q[1] * q[2] - q[0] * q[3]), uy = 1 - 2 * (q[0] * q[0] + q[2] * q[2]);
  return { pitch: Math.asin(Math.max(-1, Math.min(1, uz))) * DEG, tilt: Math.acos(Math.max(-1, Math.min(1, uy))) * DEG };
}

/**
 * Report metrics. src: {t, state, tank, events, metrics (landing metrics of the run), armedAt, tdT (touchdown t or
 * null), m0 (full gas, kg)}; opts {h_jump (m), max_tilt (deg), turns}.
 */
export function rollMetrics(src, { h_jump = JUMP_REQ, max_tilt = 12, turns = 1 } = {}) {
  const { t, state, tank } = src, n = t.length;
  const { phi, phiW, k0 } = rollSeries(src);
  const kTd = src.tdT == null ? n - 1 : Math.max(k0, t.findIndex(x => x >= src.tdT - 1e-9));
  const yEdge = state[k0]?.[1] ?? NaN;
  let kApex = k0, maxPhi = 0, kStart = null, kEnd = null, wzMax = 0, pitchMax = 0, wyMax = 0;
  for (let k = k0; k <= kTd && k < n; k++) {
    const s = state[k];
    if (s[1] > state[kApex][1]) kApex = k;
    const a = Math.abs(phi[k]);
    if (a > maxPhi) maxPhi = a;
    if (kStart === null && Math.abs(phi[k] - phi[k0]) >= 10 / DEG) kStart = k;
    if (kEnd === null && a >= (360 * turns - 10) / DEG) kEnd = k;
    wzMax = Math.max(wzMax, Math.abs(s[12])); wyMax = Math.max(wyMax, Math.abs(s[11]));
    pitchMax = Math.max(pitchMax, Math.abs(attitude([s[3], s[4], s[5], s[6]]).pitch));
  }
  const m0 = src.m0 ?? null;
  const gasAt = k => (m0 != null && tank?.[k] ? m0 - tank[k][0] : null);
  let kDrop = null;
  if (tank) for (let k = 0; k < n; k++) if (tank[k][2] - REG.dp_min < REG.p_set) { kDrop = k; break; }
  const lm = src.metrics || {}, climb = state[kApex][1] - yEdge, phiTd = phi[kTd] * DEG;
  const rollOk = Math.abs(phiTd) >= 360 * turns - max_tilt, jumpOk = climb >= h_jump;
  const r3 = x => (x == null || !Number.isFinite(x) ? null : Math.round(x * 1000) / 1000);
  return {
    success: !!lm.success && rollOk && jumpOk, landing_success: !!lm.success, roll_ok: rollOk, jump_ok: jumpOk,
    roll_td_deg: r3(phiTd), roll_max_deg: r3(maxPhi * DEG), roll_wz_integral_td_deg: r3(phiW[kTd] * DEG),
    roll_rate_max: r3(wzMax), yaw_rate_max: r3(wyMax), pitch_max_deg: r3(pitchMax),
    t_edge: r3(t[k0]), t_apex: r3(t[kApex]), t_roll_start: kStart === null ? null : r3(t[kStart]), t_roll_end: kEnd === null ? null : r3(t[kEnd]),
    t_touchdown: r3(src.tdT), roll_at_apex_deg: r3(phi[kApex] * DEG),
    climb_m: r3(climb), apex_com_y: r3(state[kApex][1]), apex_above_deck_m: r3(state[kApex][1] - WORLD.deck.top),
    gas_touchdown_kg: r3(gasAt(kTd)), gas_used_kg: r3(lm.gas_used_kg ?? gasAt(n - 1)), t_regulator_dropout: kDrop === null ? null : r3(t[kDrop]),
    td_vy: r3(lm.touchdown_vertical_speed), td_tilt_deg: r3(lm.touchdown_tilt_deg), td_rate: r3(lm.touchdown_angular_rate),
    td_hspeed: r3(lm.touchdown_horizontal_speed), td_slip_deg: src.tdT == null ? null : r3(slipDeg(state[kTd])), hull_strike: lm.hull_strike ?? null, reason: lm.reason ?? (lm.fails ? lm.fails.join(',') || 'landed' : null),
  };
}

/** From a coldgas-traj@1 trajectory. */
export function fromTraj(tr) {
  const ev = tr.events || [], arm = ev.find(e => e.type === 'armed'), td = ev.find(e => e.type === 'touchdown');
  return { t: tr.t, state: tr.state, tank: tr.tank, thrust: tr.thrust, u: tr.u_cmd, events: ev, metrics: tr.metrics, armedAt: arm ? arm.t : 0, tdT: td ? td.t : null, m0: tr.tank[0][0], kind: 'twin' };
}

/** From a soft-body recording's meta (meta.propulsion + meta.flight). */
export function fromSoft(meta) {
  const L = meta.propulsion, F = meta.flight, ev = F.events || [];
  const k = L.armed.findIndex(a => a);
  const td = ev.find(e => e.type === 'touchdown');
  return { t: L.t, state: L.state, tank: L.tank, thrust: L.thrust, u: L.u_cmd, events: ev, metrics: F.metrics, armedAt: k < 0 ? Infinity : L.t[k], tdT: td ? td.t : null, m0: L.kit?.gas0 ?? L.tank[0][0], kind: 'soft' };
}

/** Slip angle (deg) between the body's horizontal forward axis and the horizontal velocity (0 = rolling straight). */
export function slipDeg(s) {
  const q = [s[3], s[4], s[5], s[6]], fx = 2 * (q[0] * q[2] + q[1] * q[3]), fz = 1 - 2 * (q[0] * q[0] + q[1] * q[1]);
  const vx = s[7], vz = s[9], c = (fx * vx + fz * vz) / (Math.hypot(fx, fz) * Math.hypot(vx, vz) || 1);
  return Math.acos(Math.max(-1, Math.min(1, c))) * DEG;
}
