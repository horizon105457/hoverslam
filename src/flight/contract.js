// Cross-module contract of the cold-gas flight stack (see spec/README.md for the authoritative prose).
// Consumers: the JS rigid twin (src/flight), the soft-body thruster integration (src/softbody/actuators.js),
// viewers (src/programs/*) and, mirrored in Python, python/coldgas. Changing anything here is a contract change:
// bump the version string and update spec/README.md, the Python mirror and the parity goldens together.
import { PROPULSION, NOZZLE_IDS, VEHICLE, DRIVE } from '../core/spec.js';
import { MODEL_ID } from './model.js';

export const CONTRACT_VERSION = 'coldgas-contract@2';
/** Number of thrusters = length of every valve / thrust vector, in spec/propulsion.json order (action indices 0..NU-1). */
export const NU = PROPULSION.nozzles.length;
export { NOZZLE_IDS };
/**
 * Drive channels (spec/drive.json, spec/README.md 5.13): brake and steer, appended after the nozzles. An action is
 * NU long (valves only, drive commands 0) or NA = NU + ND long; DRIVE_IDX gives each channel's action index.
 */
export const DRIVE_CHANNELS = DRIVE.channels;
export const ND = DRIVE_CHANNELS.length;
export const NA = NU + ND;
export const DRIVE_IDX = Object.fromEntries(DRIVE_CHANNELS.map((c, i) => [c, NU + i]));
/** Indices of the nozzle groups in the action vector. */
export const GROUPS = Object.fromEntries(['bottom', 'rear', 'side'].map(g => [g, PROPULSION.nozzles.flatMap((n, i) => (n.group === g ? [i] : []))]));
/** Control period (s): commands are held for this long. Same for the twins and the soft body. */
export const CONTROL_DT = PROPULSION.control.dt;

/** Flat rigid state layout (Float64Array(13)) used in trajectories and batched code. */
export const STATE = { p: 0, q: 3, v: 7, w: 10, size: 13 };
export const TRAJ_FORMAT = 'coldgas-traj@1';
/** Policy file format (MLP weights + observation normalisation), spec/README.md section 7. */
export const POLICY_FORMAT = 'coldgas-policy@1';
/** Rigid-twin model id written into trajectories (`model`); bumped whenever twin physics change. */
export const MODEL = MODEL_ID;
/** Integrator step (s) and integrator steps per control step. */
export const DT = VEHICLE.integrator.dt;
export const SUBSTEPS = Math.round(CONTROL_DT / DT);
export const PHASES = ['deck', 'air', 'ground'];

/**
 * @typedef {object} FlightState  vehicle state seen by controllers (rigid twin or soft-body estimate)
 * @property {number} t              s since episode start
 * @property {Float64Array} p        world position of the centre of mass (m)
 * @property {Float64Array} q        orientation body->world, [x, y, z, w]
 * @property {Float64Array} v        world velocity of the centre of mass (m/s)
 * @property {Float64Array} w        angular velocity in BODY axes (rad/s)
 * @property {Float64Array} comB     centre of mass in the body frame (moves as gas is used)
 * @property {{m:number, T:number, p:number}} tank  gas mass (kg), temperature (K), pressure (Pa)
 * @property {Float64Array} valve    actual valve openings 0..1 (NU)
 * @property {boolean} armed         thrusters enabled (task rule, e.g. after leaving the deck)
 * @property {string} phase          'deck' | 'air' | 'ground'
 * @property {{brake:number, steer:number}} [drive]  brake pressure 0..1 and road-wheel steer angle (rad) (5.13);
 *           the twins and the soft-body rig report it, controllers must not need it (missing = 0 for loadState)
 * Optional (twin only, controllers must not need them): pc (feed pressure, Pa), thrust (N per nozzle), step.
 *
 * @typedef {object} Controller      implemented identically in JS (src/flight/controllers) and Python
 * @property {(opts:{task:object, seed:number, spec?:object}) => void} reset
 * @property {(state:FlightState, info?:object) => Float64Array} act   commanded openings u_cmd in [0,1]^NU,
 *           optionally followed by the drive commands (length NA: brake in [0,1], steer in [-1,1]); called every
 *           CONTROL_DT; ignored (forced to 0) while not armed
 */
