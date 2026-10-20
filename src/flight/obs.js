// Default observation vector (spec/README.md section 8, `coldgas-obs@1`), identical to python/coldgas/obs.py.
// Built from FlightState fields + task only (works on soft-body estimates), no libm functions.
// Base (24): world up in body axes (3), body-frame velocity (3), body rates (3), world vertical speed, height of the
// body origin above the street, gas fraction, feed-pressure ratio pc/p_set, armed, valve openings (10).
// Task extras, appended in this order when the task has them: target -> [height_error, hold_progress],
// pad -> [pad_dx, pad_dz], recover_by_height -> [recover_margin].
import { PROPULSION, WORLD } from '../core/spec.js';
import { NOZZLE_IDS, CONTROL_DT } from './contract.js';
import { heightOf } from './tasks.js';
import { streetLevel } from './world.js';
import { Propulsion } from './propulsion.js';

export const OBS_FORMAT = 'coldgas-obs@1';
export const OBS_BASE = ['up_x', 'up_y', 'up_z', 'vb_x', 'vb_y', 'vb_z', 'w_x', 'w_y', 'w_z', 'v_y', 'height', 'gas_frac', 'pc_ratio', 'armed', ...NOZZLE_IDS.map(id => `valve_${id}`)];
export function obsNames(task) {
  const n = [...OBS_BASE];
  if (task?.target) n.push('height_error', 'hold_progress');
  if (task?.pad) n.push('pad_dx', 'pad_dz');
  if (task?.recover_by_height !== undefined) n.push('recover_margin');
  return n;
}

const m0 = new Propulsion(PROPULSION, WORLD.air.pressure).m0, reg = PROPULSION.regulator, pa = WORLD.air.pressure;

export class Observer {
  constructor(task) { this.task = task; this.names = obsNames(task); this.size = this.names.length; this.reset(); }
  reset() { this.holdT = 0; }
  /** @returns {Float64Array} observation for a FlightState (call once per control step, in order) */
  observe(s, out = new Float64Array(this.size)) {
    const [x, y, z, w] = s.q, v = s.v;
    const R0 = 1 - 2 * (y * y + z * z), R1 = 2 * (x * y - z * w), R2 = 2 * (x * z + y * w);
    const R3 = 2 * (x * y + z * w), R4 = 1 - 2 * (x * x + z * z), R5 = 2 * (y * z - x * w);
    const R6 = 2 * (x * z - y * w), R7 = 2 * (y * z + x * w), R8 = 1 - 2 * (x * x + y * y);
    out[0] = R3; out[1] = R4; out[2] = R5;
    out[3] = R0 * v[0] + R3 * v[1] + R6 * v[2]; out[4] = R1 * v[0] + R4 * v[1] + R7 * v[2]; out[5] = R2 * v[0] + R5 * v[1] + R8 * v[2];
    out[6] = s.w[0]; out[7] = s.w[1]; out[8] = s.w[2]; out[9] = v[1];
    const h = heightOf(s.p, s.q, s.comB);
    out[10] = h; out[11] = s.tank.m / m0;
    let pc = Math.min(reg.p_set, s.tank.p - reg.dp_min);
    pc = pc > 2 * pa ? pc : 0;
    out[12] = pc / reg.p_set; out[13] = s.armed ? 1 : 0;
    for (let i = 0; i < NOZZLE_IDS.length; i++) out[14 + i] = s.valve[i];
    let k = 14 + NOZZLE_IDS.length;
    const T = this.task;
    if (T?.target) {
      const err = h - T.target.height;
      this.holdT = Math.abs(err) <= T.target.tolerance ? this.holdT + CONTROL_DT : 0;
      const pr = this.holdT / T.target.hold;
      out[k++] = err; out[k++] = pr > 1 ? 1 : pr;
    }
    if (T?.pad) { out[k++] = T.pad.center[0] - s.p[0]; out[k++] = T.pad.center[2] - s.p[2]; }
    if (T?.recover_by_height !== undefined) out[k++] = (s.p[1] - streetLevel(s.p[0], s.p[2])) - T.recover_by_height;
    return out;
  }
}
