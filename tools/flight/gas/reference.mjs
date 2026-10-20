// Theory / reference of the gas study: the vertical channel of the soft landing in isolation.
// Start = the real ballistic state of each (preset, seed) once airborne and 1.5 m clear of the deck edge (zero thrust
// until then), levelled (heading kept, pitch / roll removed) and with ω = 0, so only the vertical problem
// remains. On that start, with the real twin physics (valve lag, mass and COM changing, Isp falling as the tank
// cools, regulated feed, suspension and hull contacts), at 1 ms resolution (integrator steps, not the 50 Hz grid):
//   1. bang-bang: closed, then all four bottom valves at u until the first street contact (then closed): sweep the
//      ignition time -> touchdown speed, gas, success; the latest ignition meeting v_td <= v_cap is the
//      Meditch / Pontryagin minimum-gas landing for that cap;
//   2. constant-throttle family at fixed touchdown speed: for each ignition time the throttle u that lands at
//      exactly v_cap (bisection) and its gas -> gas vs ignition time at equal touchdown speed (the question itself);
//   3. throttle ceilings (umax.bottom): the latest-ignition optimum per ceiling (short hard vs long gentle burn);
//   4. 1-D point-mass model and the analytic constant-mass / constant-Isp bound, to separate the model effects.
// usage: node tools/flight/gas/reference.mjs [presets=flat,nose,...] [seeds=1,2,3] [out=experiments/runs/gas-reference/<stamp>]
import path from 'path';
import { parseArgs } from '../../lib/args.mjs';
import { RigidTwin } from '../../../src/flight/twin.js';
import { Propulsion } from '../../../src/flight/propulsion.js';
import { MassModel } from '../../../src/flight/massprops.js';
import { tiltOf, heightOf } from '../../../src/flight/tasks.js';
import { specHash } from '../../../src/flight/spechash.js';
import { VEHICLE, WORLD, GENERATED } from '../../../src/core/spec.js';
import { streetLevel } from '../../../src/flight/world.js';
import { runEpisode } from '../../../src/flight/run.js';
import { CONTROL_DT } from '../../../src/flight/contract.js';
import { ROOT, BOTTOM, NU, G, DEG, FB_NOM, heaveInto, burnController, coastTo, levelQuat, writeJSON, stamp, mean, fmt } from './lib.mjs';

const a = parseArgs();
const presets = (a.presets || 'flat,nose,rear,side,roof,corner').split(',');
const seeds = (a.seeds || '1,2,3').split(',').map(Number);
const out = path.resolve(a.out || path.join(ROOT, 'experiments/runs/gas-reference', stamp()));
const CAPS = [2.0, 1.5, 1.0, 0.5], UMAX = [0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0];
const HOLD = 1.0, DT = VEHICLE.integrator.dt, CTRL = 20; // integrator steps per control step
// pitch-free heave at full rear valves, full tank: thrust and mass flow (the bound of the vertical channel)
const [FH, MDH] = (() => { const Q = new Propulsion(), U = heaveInto(new Float64Array(NU), 1, new MassModel().at(Q.m0).com[2]); Q.valve.set(U); const T = Q.step(U, 1e-9); return [BOTTOM.reduce((s, i) => s + T[i], 0), BOTTOM.reduce((s, i) => s + Q.mdot[i], 0)]; })();

/** One drop at integrator resolution: bottom valves at u from tIgn (s after start) until touchdown. */
function drop(st, { tIgn, u = 1 }) {
  const tw = new RigidTwin({ arm: 'start' });
  tw.reset({ type: 'state', p: st.p, q: st.q, v: st.v, w: [0, 0, 0], gas: st.gas, valve: new Array(NU).fill(0), leftDeck: true });
  tw.airborne = true; // in the air (the start is the 'airborne' state); otherwise no touchdown is ever recorded
  const U0 = new Float64Array(NU), U1 = new Float64Array(NU);
  const nIgn = Math.round(tIgn / DT), m0 = tw.prop.m;
  let k = 0, kTD = null, gasTD = null, holdTilt = 0, pk = 0;
  while (k < 8000) {
    heaveInto(U1, u, tw.mp.com[2]); // pitch-free allocation about the current COM (it moves forward as gas is used)
    tw.substep(k >= nIgn && !tw.touchdown ? U1 : U0); k++;
    // over-braked: the burn stopped the descent above the street (it would climb away): the family's v_td is 0 there
    if (!tw.touchdown && k > nIgn && tw.v[1] > 0) return { tIgn, u, stopped: true, vtd: 0, gas: m0 - tw.prop.m, ok: false, h_stop: tw.p[1] };
    if (tw.touchdown && kTD === null) { kTD = k; gasTD = m0 - tw.prop.m; tw.peakSf = 0; }
    if (kTD !== null && (k - kTD) % CTRL === 0) holdTilt = Math.max(holdTilt, tiltOf(tw.q));
    if (tw.hullStrike || (kTD !== null && k - kTD >= HOLD / DT)) break;
    if (kTD !== null) pk = Math.max(pk, tw.peakSf);
  }
  const td = tw.touchdown;
  if (!td) return { tIgn, u, vtd: null, gas: m0 - tw.prop.m, ok: false };
  const vtd = Math.abs(td.v[1]), tilt = tiltOf(td.q), rate = Math.hypot(...td.w);
  // task criteria (escape-soft-landing) incl. its 6 s horizon: touchdown + 1 s hold must fit (episode clock = t0 + t)
  const fail = tw.hullStrike ? 'hull' : vtd > 2 ? 'vtd' : tilt > 12 / DEG || holdTilt > 12 / DEG ? 'tilt' : rate > 1 ? 'rate' : st.t0 + td.t + HOLD > 6 ? 'late' : null;
  const ok = !fail;
  return { fail, tIgn, u, t_td: td.t, street: streetLevel(td.p[0], td.p[2]), vtd, tilt_deg: tilt * DEG, hold_tilt_deg: holdTilt * DEG, rate, gas_td: gasTD, gas: m0 - tw.prop.m, hull: !!tw.hullStrike, ok, peak_g_after_td: pk / G };
}

/** Latest ignition with v_td <= cap at throttle u. Ignition is quantised to the 1 ms integrator step, where v_td
 *  moves by up to ~0.4 m/s, so: integer bisection for the last step k with v_td <= cap (over-braked counts as 0),
 *  then gas / time interpolated linearly in v_td between steps k and k+1 to the exact cap. */
function latest(st, cap, u = 1) {
  const f = k => drop(st, { tIgn: k * DT, u });
  let lo = 0, hi = Math.round(st.tFree / DT);
  const r0 = f(lo);
  if (!(r0.vtd != null && r0.vtd <= cap)) return null; // even an immediate burn cannot meet the cap with this ceiling
  while (hi - lo > 1) { const m = (lo + hi) >> 1, r = f(m); if (r.vtd != null && r.vtd <= cap) lo = m; else hi = m; }
  const a = f(lo), b = f(hi), va = a.vtd ?? 0, l = b.vtd > va ? (cap - va) / (b.vtd - va) : 0;
  const ip = (x, y) => (x == null || y == null ? null : x + l * (y - x));
  const ga = a.stopped ? a.gas : a.gas_td; // over-braked bracket: the gas at the stop (a ~0 m/s landing)
  return { ...(a.stopped ? b : a), tIgn: ip(lo * DT, hi * DT), vtd: cap, gas_td: ip(ga, b.gas_td), gas: ip(a.stopped ? a.gas : a.gas, b.gas), t_td: a.stopped ? b.t_td : ip(a.t_td, b.t_td),
    bracket: [{ k: lo, vtd: va, gas_td: ga, ok: a.ok, fail: a.fail ?? null, stopped: !!a.stopped }, { k: hi, vtd: b.vtd, gas_td: b.gas_td, ok: b.ok, fail: b.fail }],
    dvtd_dt: (b.vtd - va) / DT, ok: a.stopped ? b.vtd <= 2 && !b.hull : a.ok };
}

/** Throttle that lands at exactly cap after igniting at tIgn (null if even u = 1 is too weak). */
function throttleFor(st, tIgn, cap) {
  const r1 = drop(st, { tIgn, u: 1 });
  if (!(r1.vtd != null && r1.vtd <= cap)) return null;
  let lo = 0.05, hi = 1;
  for (let i = 0; i < 22; i++) { const m = 0.5 * (lo + hi), r = drop(st, { tIgn, u: m }); if (r.vtd != null && r.vtd <= cap) hi = m; else lo = m; }
  return drop(st, { tIgn, u: hi });
}

/** 1-D point mass from the same start (height of the wheel bottoms above the street, v_y), drag on y:
 *  real = the propulsion model (valve lag, tank blowdown, Isp(T)) and the mass model; ideal = no lag, constant
 *  mass M0, constant thrust and mass flow of the full tank. */
function drop1D(st, { tIgn, u = 1, real = true }) {
  const P = new Propulsion(), MM = new MassModel(), ka = 0.5 * WORLD.air.density * VEHICLE.aero.CdA[1];
  P.m = st.gas; { const rho = P.m / P.V; P.p = P.K * rho ** P.spec.tank.polytropic_n; P.T = P.p / (rho * P.R); }
  const U = new Float64Array(NU), nI = Math.round(tIgn / DT);
  let h = st.h, v = st.v[1], k = 0, m = st.gas, M = MM.at(m).M;
  while (h > 0 && k < 8000) {
    const on = k >= nI;
    let F;
    if (real) { heaveInto(U, on ? u : 0, MM.at(m).com[2]); const T = P.step(U, DT); F = BOTTOM.reduce((s, i) => s + T[i], 0); }
    else { F = on ? u * FH : 0; m -= on ? u * MDH * DT : 0; }
    v += DT * ((F - ka * Math.abs(v) * v) / M - G); h += DT * v; k++;
    if (real) { m = P.m; M = MM.at(m).M; }
    if (on && v > 0) return { tIgn, vtd: 0, stopped: true, gas: st.gas - m };
  }
  return { tIgn, vtd: -v, gas: st.gas - m };
}
function latest1D(st, cap, opt) { // same integer-step bisection + interpolation to the exact cap as latest()
  const f = k => drop1D(st, { ...opt, tIgn: k * DT });
  let lo = 0, hi = Math.round(st.tFree / DT);
  while (hi - lo > 1) { const m = (lo + hi) >> 1; if (f(m).vtd <= cap) lo = m; else hi = m; }
  const a = f(lo), b = f(hi), l = b.vtd > a.vtd ? (cap - a.vtd) / (b.vtd - a.vtd) : 0;
  return { tIgn: (lo + l) * DT, vtd: cap, gas: a.gas + l * (b.gas - a.gas) };
}

/** Analytic: constant mass M0, constant thrust F, gravity only (no drag, no lag): latest ignition and its gas. */
function analytic(st, cap, F, mdot) {
  const M = st.M, acc = F / M - G, v0 = -st.v[1], h0 = st.h;
  const hi = (v0 * v0 + 2 * G * h0 - cap * cap) / (2 * (G + acc)), vi = Math.sqrt(cap * cap + 2 * acc * hi), tb = (vi - cap) / acc;
  const tff = (-v0 + Math.sqrt(v0 * v0 + 2 * G * (h0 - hi))) / G; // free fall from h0 to hi (v0 downward positive)
  return { h_ign: hi, v_ign: vi, burn_s: tb, t_ign: tff, gas: mdot * tb, impulse: F * tb, dv: vi - cap + 0, gravity_loss: G * tb };
}

const pick = m => ({ success: m.success, reason: m.reason, vtd: m.touchdown_vertical_speed, tilt_deg: m.touchdown_tilt_deg, rate: m.touchdown_angular_rate, gas: m.gas_used_kg, hull: m.hull_strike, peak_g: m.peak_accel_g });
const hash = await specHash();
const cases = [];
const wheelLow = Math.min(...GENERATED.massprops.wheels.map(w => w.centre[1] - w.radius)); // body-frame wheel bottom
for (const preset of presets) for (const seed of seeds) {
  // start: the zero-thrust car once airborne and 1.5 m clear of the deck edge (the levelled body must not overlap the deck)
  const env = coastTo('escape-soft-landing', seed, preset, tw => tw.airborne && tw.p[2] > WORLD.deck.halfWidth + 1.5);
  const tw = env.twin, s = tw.state;
  const q = levelQuat(s.q);
  // origin height of the levelled car above the touchdown point (COM height minus the levelled COM offset)
  const MM = new MassModel(), mp = MM.at(s.tank.m);
  const h0 = s.p[1] - mp.com[1] + wheelLow; // street level at the landing point is the ground plane (street_td below)
  const st = { preset, seed, t0: s.t, p: Array.from(s.p), q, v: Array.from(s.v), gas: s.tank.m, h: h0, M: mp.M };
  // free-fall touchdown time from this state (upper end of every ignition sweep)
  const z = drop(st, { tIgn: 99 });
  st.tFree = z.t_td; st.vFree = z.vtd; // the drop's clock starts at 0
  const rec = { preset, seed, t_airborne: s.t, t_armed: env.twin.events.find(e => e.type === 'armed').t, h0, vy0: s.v[1], vh0: Math.hypot(s.v[0], s.v[2]), M0: mp.M,
    real_tilt_deg: tiltOf(s.q) * DEG, real_rate: Math.hypot(...s.w), free_fall_s: st.tFree, v_impact: st.vFree, street_td: z.street, F_heave: FH, mdot_heave: MDH, F_all4: FB_NOM };
  // 1. bang-bang ignition sweep (2 ms)
  rec.sweep = [];
  for (let t = 0; t <= st.tFree + 1e-9; t += 0.002) { const r = drop(st, { tIgn: t }); rec.sweep.push([+t.toFixed(3), r.stopped ? null : r.vtd, r.gas, r.gas_td ?? null, r.ok ? 1 : 0, r.tilt_deg ?? null, r.hull ? 1 : 0, r.stopped ? 1 : 0]); }
  rec.sweep_cols = ['t_ign', 'vtd', 'gas', 'gas_td', 'ok', 'tilt_deg', 'hull', 'stopped'];
  rec.optimum = {};
  for (const cap of CAPS) {
    const r = latest(st, cap);
    const L = r && { ...r, h_ign: null };
    if (L) { const tr = drop1D(st, { tIgn: r.tIgn }); L.vtd_1d = tr.vtd; }
    rec.optimum[cap] = L;
  }
  // 2. constant throttle at fixed touchdown speed (v_cap = 1.5 and 2.0): gas vs ignition time
  rec.family = {};
  for (const cap of [2.0, 1.5]) {
    const L = rec.optimum[cap], fam = [];
    for (let t = 0; L && t <= L.tIgn + 1e-9; t += 0.02) { const r = throttleFor(st, t, cap); if (r) fam.push([+t.toFixed(3), r.u, r.gas, r.gas_td, r.vtd, r.ok ? 1 : 0]); }
    if (L) fam.push([+L.tIgn.toFixed(4), 1, L.gas, L.gas_td, L.vtd, L.ok ? 1 : 0]);
    rec.family[cap] = fam;
  }
  rec.family_cols = ['t_ign', 'u', 'gas', 'gas_td', 'vtd', 'ok'];
  // 3. throttle ceilings at v_cap 1.5 / 2.0
  rec.ceilings = {};
  for (const cap of [2.0, 1.5]) rec.ceilings[cap] = UMAX.map(u => { const r = latest(st, cap, u); return r ? { umax: u, tIgn: r.tIgn, gas: r.gas, gas_td: r.gas_td, vtd: r.vtd, burn_s: r.t_td - r.tIgn, ok: r.ok } : { umax: u, infeasible: true }; });
  // 4. model effects at v_cap 1.5: analytic, 1-D ideal (no lag, constant Isp), 1-D real propulsion, twin
  const an = analytic(st, 1.5, FH, MDH);
  rec.effects = {
    analytic: an,
    oned_ideal: latest1D(st, 1.5, { real: false }),
    oned_real: latest1D(st, 1.5, { real: true }),
    twin: rec.optimum[1.5] && { tIgn: rec.optimum[1.5].tIgn, gas: rec.optimum[1.5].gas, gas_td: rec.optimum[1.5].gas_td, vtd: rec.optimum[1.5].vtd },
  };
  // 5. executable at 50 Hz on the task (TaskTracker metrics): ignition at the last control boundary before the
  //    cap-1.5 optimum, throttle trimmed (bisection) to land at 1.5 m/s; (a) levelled start, (b) the real, rotating
  //    car (no attitude control at all: shows why the full problem needs more than the vertical burn)
  const o15 = rec.optimum[1.5];
  if (o15) {
    const kArm = Math.round(s.t / CONTROL_DT), tg = Math.floor((o15.tIgn - 1e-9) / CONTROL_DT) * CONTROL_DT;
    const lvl = { type: 'state', p: st.p, q: st.q, v: st.v, w: [0, 0, 0], gas: st.gas, leftDeck: true };
    const ep = (init, u, tIgn) => runEpisode({ task: 'escape-soft-landing', controller: burnController({ tIgn, u }), seed, preset, init, record: false }).metrics;
    let lo = 0.5, hi = 1;
    for (let i = 0; i < 20; i++) { const m = 0.5 * (lo + hi), r = ep(lvl, m, tg); if (r.touchdown_vertical_speed == null || r.touchdown_vertical_speed <= 1.5) hi = m; else lo = m; } // no touchdown = over-braked (climbs away)
    const ml = ep(lvl, hi, tg);
    rec.exec50 = { t_ign: tg, u: hi, levelled: pick(ml), rotating: pick(ep(null, hi, tg + (s.t - rec.t_armed))) };
    void kArm;
  }
  cases.push(rec);
  const o = rec.optimum;
  console.log(`${preset} s${seed}: airborne ${s.t.toFixed(2)} s h0 ${h0.toFixed(2)} vy0 ${s.v[1].toFixed(2)} free fall ${st.tFree.toFixed(3)} s -> ${st.vFree.toFixed(2)} m/s | gas@cap 2.0 ${fmt(o[2]?.gas)} (ign +${fmt(o[2]?.tIgn, 3)} s) 1.5 ${fmt(o[1.5]?.gas)} 1.0 ${fmt(o[1]?.gas)} 0.5 ${fmt(o[0.5]?.gas)} | analytic ${an.gas.toFixed(2)} 1D ${rec.effects.oned_real.gas.toFixed(2)} ideal ${rec.effects.oned_ideal.gas.toFixed(2)}`);
}
const summary = { spec_hash: hash, presets, seeds, caps: CAPS };
for (const cap of CAPS) summary[`gas_min_cap${cap}`] = mean(cases.map(c => c.optimum[cap]?.gas));
for (const cap of CAPS) summary[`ign_before_td_cap${cap}`] = mean(cases.map(c => (c.optimum[cap] ? c.optimum[cap].t_td - c.optimum[cap].tIgn : NaN)));
writeJSON(path.join(out, 'reference.json'), { format: 'gas-reference@1', spec_hash: hash, presets, seeds, caps: CAPS, cases, summary }, 0);
console.log(JSON.stringify(summary, null, 1), `\n-> ${path.relative(ROOT, out)}`);
