// Plays u_cmd from a coldgas-traj@1 trajectory: row k is applied at t = k * CONTROL_DT (k = round(t / dt), so a
// soft-body clock with a different integrator step still lines up); zeros after the end. If the trajectory has
// drive_cmd (5.13) its rows follow as action entries NU.. (an NA-long action), else the action is NU long.
import { NU, ND, NA, CONTROL_DT, TRAJ_FORMAT } from '../contract.js';
export function createReplay({ traj } = {}) {
  if (!traj || traj.format !== TRAJ_FORMAT) throw new Error(`replay: need a ${TRAJ_FORMAT} object (config.traj)`);
  const rows = traj.u_cmd, drows = traj.drive_cmd || null, dt = traj.dt || CONTROL_DT;
  return {
    name: 'replay', config: { source: traj.controller, seed: traj.seed, task: traj.task },
    reset() {},
    act(s) {
      const k = Math.round(s.t / dt), r = rows[k], u = new Float64Array(drows ? NA : NU);
      if (r) for (let i = 0; i < NU; i++) u[i] = r[i];
      const d = drows && drows[k];
      if (d) for (let i = 0; i < ND; i++) u[NU + i] = d[i];
      return u;
    },
  };
}
