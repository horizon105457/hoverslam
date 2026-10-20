// Plays u_cmd from a coldgas-traj@1 trajectory: row k is applied at t = k * CONTROL_DT (k = round(t / dt), so a
// soft-body clock with a different integrator step still lines up); zeros after the end.
import { NU, CONTROL_DT, TRAJ_FORMAT } from '../contract.js';
export function createReplay({ traj } = {}) {
  if (!traj || traj.format !== TRAJ_FORMAT) throw new Error(`replay: need a ${TRAJ_FORMAT} object (config.traj)`);
  const rows = traj.u_cmd, dt = traj.dt || CONTROL_DT;
  return {
    name: 'replay', config: { source: traj.controller, seed: traj.seed, task: traj.task },
    reset() {},
    act(s) {
      const k = Math.round(s.t / dt), r = rows[k], u = new Float64Array(NU);
      if (r) for (let i = 0; i < NU; i++) u[i] = r[i];
      return u;
    },
  };
}
