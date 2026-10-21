// The optimisation problem of one MPC plan: θ -> total cost of one rollout on a planning twin loaded from the
// current FlightState (RigidTwin.loadState, integrator step plan_dt). Optimizers only see this interface:
//   P.dim, P.lo, P.hi (box on θ), P.scale (exploration scale per element), P.cost(θ) -> number (one rollout,
//   counted in P.evals),
//   P.rollout(θ, rec) (same, with a per-step recorder for plots / analysis).
// The objective (objectives/*.js) prices every control step of the rollout and its end; the rollout stops early
// once the objective marks the lane dead (e.g. a hull strike). Rollouts are deterministic: same θ, same cost.
// Drive channels (param.drive): brake / steer commands stay 0 until the rollout's touchdown (or from the start if the
// plan state is on the ground), unless drive_air: wheels braked in the air arrive locked, a steered wheel sideways.
import { RigidTwin } from '../twin.js';
import { NU, CONTROL_DT, GROUPS, DRIVE_IDX } from '../contract.js';

export class Planner {
  constructor(c, param, objective, task) {
    this.c = c; this.param = param; this.obj = objective;
    this.tw = new RigidTwin({ arm: task?.arm || 'start', dt: c.plan_dt });
    this.dim = param.dim; this.lo = param.lo; this.hi = param.hi; this.scale = param.scale; this.H = param.H;
    this.u = new Float64Array(param.na);
    this.evals = 0; this.s = null; this.coastEnd = -Infinity; this.cutAt = null;
  }

  /** Plan from FlightState s; commands are forced to 0 before coastEnd (episode time), the bottom valves from cutAt
   *  on (episode time; null: from ground_cut s after the rollout's own touchdown, if ground_cut is set). */
  begin(s, coastEnd = -Infinity, cutAt = null) { this.s = s; this.coastEnd = coastEnd; this.cutAt = cutAt; this.obj.begin?.(s); return this; }

  cost(theta) { return this.rollout(theta, null); }

  /** Roll θ out from the plan state; rec(h, tw, u, costSoFar) after every control step if given. */
  rollout(theta, rec) {
    const s = this.s, tw = this.tw.loadState(s), P = this.param, obj = this.obj, u = this.u, H = this.H;
    const L = obj.start(tw, s), gc = this.c.ground_cut;
    let total = 0, cutAt = this.cutAt;
    for (let h = 0; h < H; h++) {
      P.command(theta, h * CONTROL_DT, u);
      if (s.t + h * CONTROL_DT < this.coastEnd - 1e-9) u.fill(0);
      if (cutAt !== null && s.t + h * CONTROL_DT >= cutAt - 1e-9) for (const i of GROUPS.bottom) u[i] = 0;
      if (P.drive && !this.c.drive_air && tw.touchdown === null) { u[DRIVE_IDX.brake] = 0; u[DRIVE_IDX.steer] = 0; }
      tw.step(u);
      if (cutAt === null && gc != null && tw.touchdown) cutAt = s.t + (h + 1) * CONTROL_DT + gc;
      total += obj.step(L, tw, h, H, u);
      if (rec) rec(h, tw, u, total);
      if (L.dead) break;
    }
    total += obj.terminal(L, tw, H);
    this.evals++;
    return total;
  }
}
