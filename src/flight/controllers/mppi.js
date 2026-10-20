// MPPI (model-predictive path integral control) on the rigid twin, mirror of python/coldgas/controllers/mppi.py
// (same settings, defaults (defaults.json `mppi`), noise stream and cost; parity-tested on full episodes).
// A Controller like the others: act(FlightState) -> u_cmd. Every `replan` control steps the state is loaded into a
// planning twin (RigidTwin.loadState), K perturbed command sequences are rolled out for `horizon` seconds and the
// nominal sequence moves to their cost-weighted mean (softmax, temperature relative to the cost spread). Commands
// are piecewise constant over `knot` seconds. With base 'pd' the samples are residuals on top of the pd autopilot
// evaluated inside each rollout (pd keeps the car upright, MPPI shapes the burn, uses rear/side nozzles to steer the
// landing point away from kerbs, saves gas); base 'none' plans raw valve commands.
// The planning model may use a coarser integrator step (plan_dt) than the real twin: model mismatch on purpose
// (speed); the executed physics is the reference twin, or the soft body (ThrusterRig's state estimate).
// Lanes are rolled out one after the other on one twin (lanes are independent: a lane of the batched numpy twin
// equals the scalar twin bit for bit). Reductions are sequential sums in lane / knot / nozzle order and the softmax
// uses the portable exp, so a plan is bit-identical to Python's.
// Soft-body additions (w_slip, w_td_slip, w_gnd_slip, w_gnd_yaw, curb_steps, curb_margin, bottom_gate; defaults = the
// original algorithm) in both languages: see defaults.json `_doc_soft`, `_doc_gate` and experiments/README.md.
import defaults from './defaults.json' with { type: 'json' };
import { createPD } from './pd.js';
import { RigidTwin, pyRound } from '../twin.js';
import { Propulsion } from '../propulsion.js';
import { NU, CONTROL_DT, GROUPS } from '../contract.js';
import { heightOf } from '../tasks.js';
import { streetLevel, GROUND } from '../world.js';
import { mulberry32 } from '../rng.js';
import { exp } from '../fmath.js';
import { PROPULSION, WORLD } from '../../core/spec.js';

export function createMPPI(cfg = {}) {
  const c = { ...defaults.mppi, ...cfg };
  for (const k of Object.keys(c)) if (k[0] === '_') delete c[k];
  const K = Math.trunc(c.samples), H = Math.max(1, pyRound(c.horizon / CONTROL_DT)), knot = Math.max(1, pyRound(c.knot / CONTROL_DT));
  const nk = Math.ceil(H / knot), g = WORLD.gravity, pSet = PROPULSION.regulator.p_set;
  const U = new Float64Array(nk * NU), V = new Float64Array(K * nk * NU), cost = new Float64Array(K), wts = new Float64Array(K);
  const u = new Float64Array(NU), zero = new Float64Array(NU);
  // nominal full-open bottom thrust (regulated feed, full tank) for the stoppability cost
  const P = new Propulsion();
  P.valve.fill(1);
  const T1 = P.step(new Float64Array(NU).fill(1), 1e-9);
  let FbNom = 0;
  for (const i of GROUPS.bottom) FbNom += T1[i];
  const isB = Array.from({ length: NU }, (_, i) => GROUPS.bottom.includes(i));
  /** Residual weight on the bottom nozzles: fades linearly from 1 at bottom_gate metres to 0 on the street, so the
   *  base pilot alone flies the touchdown (level, no residual heave/pitch/roll thrust on the ground); 0 = off. */
  const gate = (p, q, comB) => {
    if (!c.bottom_gate) return 1;
    const g = heightOf(p, q, comB) / c.bottom_gate;
    return g > 1 ? 1 : g > 0 ? g : 0;
  };
  let task = null, rng = null, tw = null, base = null, baseReal = null, since = 1e9;
  const ctl = { name: 'mppi', config: c, plans: 0, lastCost: null, planMs: 0 };

  ctl.reset = ({ task: t, seed } = {}) => {
    task = t || null;
    // + 1: mulberry32 maps seed 0 to 1, so episode seeds 0 and 1 would share a noise stream
    rng = mulberry32((seed == null ? c.seed : c.seed * 1000003 + seed) + 1);
    tw = new RigidTwin({ arm: task?.arm || 'start', dt: c.plan_dt });
    base = c.base === 'pd' ? createPD(c.base_pd) : null;
    baseReal = c.base === 'pd' ? createPD(c.base_pd) : null;
    if (base) { base.reset({ task }); baseReal.reset({ task }); }
    U.fill(0); since = 1e9; ctl.plans = 0; ctl.planMs = 0;
  };

  ctl.act = (s) => {
    const ub = base ? baseReal.act(s) : zero; // keeps the real pd memory in step
    const out = new Float64Array(NU);
    if (!s.armed) return out;
    if (since >= c.replan) {
      if (since < 1e9) { // shift the nominal by the whole knots executed since the last plan
        const sh = Math.floor(since / knot);
        if (sh) { U.copyWithin(0, Math.min(sh, nk) * NU); U.fill(0, Math.max(nk - sh, 0) * NU); }
      }
      const t0 = performance.now();
      plan(s);
      ctl.planMs += performance.now() - t0;
      since = 0;
    }
    const j = Math.min(Math.floor(since / knot), nk - 1), gb = gate(s.p, s.q, s.comB);
    since++;
    for (let i = 0; i < NU; i++) { const x = ub[i] + (isB[i] && gb !== 1 ? U[j * NU + i] * gb : U[j * NU + i]); out[i] = x < 0 ? 0 : x > 1 ? 1 : x; }
    return out;
  };

  // ------------------------------------------------------------------------------------------------- plan
  function plan(s) {
    const n = nk * NU;
    for (let k = 0; k < K; k++) { // noise drawn for every lane (lane 0 = the current nominal, its draws discarded)
      for (let e = 0; e < n; e++) { const z = rng.gauss() * c.sigma; V[k * n + e] = U[e] + (k === 0 ? 0 : z); }
      let a = 0;
      for (let e = 0; e < n; e++) a += V[k * n + e] * V[k * n + e];
      cost[k] = c.w_res * (a / n); // residual effort (keeps MPPI close to the base when it can)
    }
    const mem = base ? baseReal.memory : null;
    for (let k = 0; k < K; k++) cost[k] = rollout(s, k, mem, cost[k]);
    // softmax weights with a temperature relative to the spread of the costs
    let cmin = Infinity, sum = 0;
    for (let k = 0; k < K; k++) { if (cost[k] < cmin) cmin = cost[k]; sum += cost[k]; }
    const d = sum / K - cmin, lam = c.temperature * (d > 1e-6 ? d : 1e-6);
    let ws = 0;
    for (let k = 0; k < K; k++) { const x = -(cost[k] - cmin) / lam; wts[k] = exp(x > -700 ? x : -700); ws += wts[k]; }
    for (let k = 0; k < K; k++) wts[k] = wts[k] / ws;
    for (let e = 0; e < n; e++) {
      let a = 0;
      for (let k = 0; k < K; k++) a += wts[k] * V[k * n + e];
      U[e] = a < -1 ? -1 : a > 1 ? 1 : a;
    }
    ctl.plans++;
    ctl.lastCost = [cost[0], cmin];
  }

  /** Total cost of lane k: load the state, roll the perturbed sequence out for H steps, accumulating onto c0 (the
   *  residual term) step by step in Python's order. */
  function rollout(s, k, mem, c0) {
    tw.loadState(s);
    if (base) base.memory = mem;
    const view = { t: 0, p: tw.p, q: tw.q, v: tw.v, w: tw.w, comB: tw.mp.com, tank: { m: 0, T: 0, p: 0 }, valve: tw.prop.valve, armed: tw.armed, phase: tw.phase };
    const L = { td0: tw.touchdown !== null, alive: true };
    let mPrev = tw.prop.m, total = c0;
    for (let h = 0; h < H; h++) {
      let ub = zero;
      if (base) {
        const P = tw.prop;
        view.t = tw.k * tw.dt; view.comB = tw.mp.com; view.tank.m = P.m; view.tank.T = P.T; view.tank.p = P.p; view.armed = tw.armed; view.phase = tw.phase;
        ub = base.act(view);
      }
      const o = (k * nk + Math.min(Math.floor(h / knot), nk - 1)) * NU, gb = gate(tw.p, tw.q, tw.mp.com);
      for (let i = 0; i < NU; i++) { const x = ub[i] + (isB[i] && gb !== 1 ? V[o + i] * gb : V[o + i]); u[i] = x < 0 ? 0 : x > 1 ? 1 : x; }
      tw.step(u);
      total += stepCost(mPrev, L, h);
      mPrev = tw.prop.m;
    }
    return total + terminalCost(L.alive);
  }

  function stepCost(mPrev, L, h) {
    const q = tw.q, w = tw.w, alive = L.alive;
    let cs = c.w_gas * (mPrev - tw.prop.m);
    const tiltC = 2 * (q[0] * q[0] + q[2] * q[2]); // 1 - cos(tilt)
    const rate2 = w[0] * w[0] + w[1] * w[1] + w[2] * w[2];
    if (alive) cs = cs + (c.w_tilt * tiltC + c.w_rate * rate2);
    const crash = alive && tw.hullStrike !== null;
    if (crash) cs = cs + c.w_crash * (2 - h / H); // earlier is worse
    const td = tw.touchdown;
    if (alive && td && !L.td0) {
      const vy = Math.abs(td.v[1]), tt = 2 * (td.q[0] * td.q[0] + td.q[2] * td.q[2]), rr = td.w[0] * td.w[0] + td.w[1] * td.w[1] + td.w[2] * td.w[2];
      const ex = vy - c.v_ok > 0 ? vy - c.v_ok : 0;
      let ctd = c.w_vy * (ex * ex) + c.w_td_tilt * tt + c.w_td_rate * rr;
      if (c.w_td_slip) ctd = ctd + c.w_td_slip * slip2(td.q, td.v);
      if (task?.pad) { const dx = td.p[0] - task.pad.center[0], dz = td.p[2] - task.pad.center[2]; ctd = ctd + c.w_pad * (dx * dx + dz * dz); }
      cs = cs + ctd;
      L.td0 = true;
    }
    const hb = heightOf(tw.p, tw.q, tw.mp.com), hp = hb > 0 ? hb : 0;
    if (task?.target && base) {
      const landing = base.memory.landing;
      if (alive && !landing) cs = cs + c.w_height * Math.abs(hb - task.target.height);
      if (alive && landing && !td) cs = cs + c.w_alt * hp;
    } else if (alive && !td) cs = cs + c.w_alt * hp; // descent progress: hovering out the horizon is never optimal
    if (c.w_slip && alive) cs = cs + c.w_slip * slip2(q, tw.v); // heading on the travel direction (soft body)
    if ((c.w_gnd_slip || c.w_gnd_yaw) && alive && td) // on the wheels: ESP by thrust against the soft body's spin-out
      cs = cs + (c.w_gnd_slip * slip2(q, tw.v) + c.w_gnd_yaw * (w[1] * w[1]));
    if (crash) L.alive = false;
    return cs;
  }

  function terminalCost(alive) {
    if (!alive) return 0;
    const p = tw.p, v = tw.v, q = tw.q, td = tw.touchdown !== null;
    // deadline: still airborne when the task needs the car on its wheels (horizon minus the upright hold)
    const tNeed = (task?.horizon ?? 1e9) - (task?.success?.upright_hold ?? 0) - 0.3;
    const lt = tw.k * tw.dt - tNeed, late = lt > 0 ? lt : 0;
    const hb = heightOf(p, q, tw.mp.com), vy = v[1];
    let cst = 0;
    if (!td) { // can the remaining descent still be stopped with ~70 % of the bottom thrust?
      const r = tw.prop.pc / pSet, Fb = FbNom * (r < 1 ? r : 1);
      const cosT = 1 - 2 * (q[0] * q[0] + q[2] * q[2]);
      const aAv = 0.7 * Fb * (cosT > 0 ? cosT : 0) / tw.mp.M - g;
      const hm = hb - 0.3 > 0.05 ? hb - 0.3 : 0.05;
      const aReq = vy < 0 ? vy * vy / (2 * hm) : 0, ex = aReq - aAv;
      cst = c.w_stop * (ex > 0 ? ex : 0) + c.w_deadline * late;
    }
    // kerbs ahead: where will the COM be after curb_lookahead seconds of rolling / flying straight on?
    // (probes at la*i/n, each with a footprint margin m; n = 2, m = 0 is the original COM check at la/2 and la)
    const la = c.curb_lookahead, n = c.curb_steps, m = c.curb_margin;
    let onCurb = false;
    for (let i = n; i >= 1 && !onCurb; i--) {
      const f = la * i / n, x = p[0] + v[0] * f, z = p[2] + v[2] * f;
      onCurb = streetLevel(x, z) > GROUND.y || (m > 0 && (streetLevel(x + m, z) > GROUND.y || streetLevel(x - m, z) > GROUND.y || streetLevel(x, z + m) > GROUND.y || streetLevel(x, z - m) > GROUND.y));
    }
    if (onCurb) cst = cst + c.w_curb;
    if (task?.pad && c.w_pad && !td) { // ballistic landing-point estimate for vehicles still in the air
      const nv = -vy > 1 ? -vy : 1, tHit = vy < 0 ? (hb > 0 ? hb : 0) / nv : 1;
      const dx = p[0] + v[0] * tHit - task.pad.center[0], dz = p[2] + v[2] * tHit - task.pad.center[2];
      cst = cst + 0.5 * c.w_pad * (dx * dx + dz * dz);
    }
    return cst;
  }

  return ctl;
}

/** sin^2 of the slip angle (body heading vs horizontal velocity), 0 below 2 m/s or with the nose near vertical.
 *  Only + - * / (parity). */
export function slip2(q, v) {
  const fx = 2 * (q[0] * q[2] + q[1] * q[3]), fz = 1 - 2 * (q[0] * q[0] + q[1] * q[1]);
  const vv = v[0] * v[0] + v[2] * v[2], ff = fx * fx + fz * fz;
  if (!(vv > 4 && ff > 0.25)) return 0;
  const cr = fz * v[0] - fx * v[2];
  return cr * cr / (ff * vv);
}
