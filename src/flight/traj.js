// coldgas-traj@1 writer (spec/README.md section 7), same layout as python/coldgas/traj.py. Row k is the state at
// t_k = k CONTROL_DT; u_cmd[k] is the command applied over [t_k, t_k+1) (after arming); valve/thrust are the values
// of the last integrator step before t_k (zeros at k = 0). The last row is the final state with u_cmd = 0.
// Optional contact[k]: bit mask of the contacts during the step that ended at t_k (1 any, 2 street, 4 deck,
// 8 parapet/pier, 16 hull point on a non-deck surface).
import { TRAJ_FORMAT, MODEL, NU, CONTROL_DT } from './contract.js';

export class TrajRecorder {
  constructor(meta = {}) {
    this.meta = meta;
    Object.assign(this, { t: [], state: [], comB: [], tank: [], u_cmd: [], valve: [], thrust: [], contact: [] });
  }
  /** Record the boundary state of the twin (before the step). */
  sample(tw) {
    const P = tw.prop;
    this.t.push(tw.k * tw.dt);
    this.state.push(Array.from(tw.flat()));
    this.comB.push(Array.from(tw.mp.com));
    this.tank.push([P.m, P.T, P.p]);
    this.valve.push(Array.from(P.valve));
    this.thrust.push(Array.from(P.thrust));
    this.u_cmd.push(new Array(NU).fill(0));
    const F = tw.stepFlags; // contacts during the control step that ended at t_k (0 at k = 0)
    this.contact.push(tw.k === 0 ? 0 : (F.any ? 1 : 0) | (F.street ? 2 : 0) | (F.deck ? 4 : 0) | (F.other ? 8 : 0) | (F.hull ? 16 : 0));
  }
  /** Command actually applied in the step that followed the last sample. */
  applied(u) { this.u_cmd[this.u_cmd.length - 1] = Array.from(u); }
  finish({ events = [], metrics = {}, spec_hash = null } = {}) {
    const m = this.meta;
    return {
      format: TRAJ_FORMAT, spec_hash, model: MODEL, producer: 'js', task: m.task ?? null, controller: m.controller ?? null,
      seed: m.seed ?? null, preset: m.preset ?? null, init: m.init ?? null, dt: CONTROL_DT, n: this.t.length,
      t: this.t, state: this.state, comB: this.comB, tank: this.tank, u_cmd: this.u_cmd, valve: this.valve, thrust: this.thrust,
      contact: this.contact, events, metrics,
    };
  }
}
