// Pure model-predictive control on the rigid twin (JS only; registry name `mpc`). Unlike controllers/mppi.js there
// is no base pilot (no pd): the valves follow an open-loop command schedule θ (param.js) that is re-optimised
// every `replan` control steps from the current FlightState, so attitude, braking, steering and any manoeuvre
// come from the objective alone. Pieces, each swappable by config:
//   param.js       schedule: knots, interpolation, mixer (raw valves | wrench modes), umax, warm-start shift
//   planner.js     the problem: θ -> cost of one rollout on a planning twin loaded from the state (plan_dt)
//   optim/*.js     sampling (mppi, cem), gradient (grad: finite-difference projected descent), hybrid stages
//   objectives/*.js  land (soft landing), roll (jump + barrel roll), gas (gas studies)
// A Controller like the others (src/flight/contract.js): reset({task, seed}), act(FlightState) -> u_cmd, so it runs
// on the twin (tools/flight/run.mjs, bench.mjs) and closed loop on the soft body (tools/sim/flight.mjs,
// controller=mpc:{...}). Deterministic for (config, task, seed): noise from mulberry32(seed·1000003 + episode seed + 1).
// Stats: plans, evals (rollouts), planMs, lastCost; with trace: true, ctl.trace = [{t, cost, evals, ms}] per plan.
import defaults from './defaults.json' with { type: 'json' };
import { ControlParam } from './param.js';
import { Planner } from './planner.js';
import { createOptimizer } from './optim/index.js';
import { createObjective } from './objectives/index.js';
import { NU, CONTROL_DT } from '../contract.js';
import { mulberry32 } from '../rng.js';

const BLOCKS = ['mppi', 'cem', 'grad', 'hybrid'];

/** Defaults merged with cfg (optimizer blocks key by key), `_` keys dropped. */
export function mpcSettings(cfg = {}) {
  const c = { ...defaults, ...cfg };
  for (const b of BLOCKS) c[b] = { ...defaults[b], ...(cfg[b] || {}) };
  for (const k of Object.keys(c)) if (k[0] === '_') delete c[k];
  return c;
}

export function createMPC(cfg = {}) {
  const c = mpcSettings(cfg);
  const param = new ControlParam(c), theta = new Float64Array(param.dim);
  let obj = null, planner = null, opt = null, since = 1e9, tArm = null;
  const ctl = { name: 'mpc', config: c, param, plans: 0, evals: 0, planMs: 0, lastCost: null, trace: null, theta };

  ctl.reset = ({ task, seed } = {}) => {
    const rng = mulberry32((seed == null ? c.seed : c.seed * 1000003 + seed) + 1);
    obj = createObjective(c.objective, { task: task || null });
    obj.reset(task || null);
    planner = new Planner(c, param, obj, task || null);
    opt = createOptimizer(c.optimizer, c, { rng, nv: param.nv });
    param.initial(c.init, theta);
    since = 1e9; tArm = null;
    ctl.objective = obj; ctl.planner = planner; ctl.optimizer = opt;
    ctl.plans = 0; ctl.evals = 0; ctl.planMs = 0; ctl.lastCost = null; ctl.trace = c.trace ? [] : null;
  };

  ctl.act = (s) => {
    if (!obj) ctl.reset({});
    obj.observe(s);
    const out = new Float64Array(NU);
    if (!s.armed) return out; // never plan on the deck: the planning twin would arm at once
    if (tArm === null) tArm = s.t;
    if (since >= c.replan) {
      if (since < 1e9) param.shift(theta, since * CONTROL_DT);
      const t0 = performance.now(), e0 = planner.evals;
      planner.begin(s, tArm + c.coast);
      let r = opt.solve(theta, planner);
      // cold start: the first plan has no warm start worth the name, give it extra passes
      for (let k = 1; k < (ctl.plans ? 1 : c.first_passes); k++) r = opt.solve(theta, planner);
      const ms = performance.now() - t0;
      ctl.plans++; ctl.planMs += ms; ctl.evals = planner.evals; ctl.lastCost = r.cost;
      if (ctl.trace) ctl.trace.push({ t: s.t, cost: r.cost, evals: planner.evals - e0, ms });
      since = 0;
    }
    param.command(theta, since * CONTROL_DT, out);
    if (s.t < tArm + c.coast - 1e-9) out.fill(0);
    since++;
    return out;
  };

  /** Predicted rollout of the current plan from state s (analysis / plots): per control step t, p, q, v, w, u, gas. */
  ctl.predict = (s) => {
    const rows = [];
    planner.begin(s, tArm === null ? -Infinity : tArm + c.coast);
    planner.rollout(theta, (h, tw, u, cost) => rows.push({ t: tw.k * tw.dt, p: Array.from(tw.p), q: Array.from(tw.q), v: Array.from(tw.v), w: Array.from(tw.w), u: Array.from(u), gas: tw.prop.m, cost }));
    planner.evals--; // not part of the controller's budget
    return rows;
  };

  return ctl;
}
