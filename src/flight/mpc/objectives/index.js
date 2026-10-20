// Objective registry of the MPC (interface: header of objectives/land.js). A config is a name or {name, ...weights};
// weights default to the objective's own DEFAULTS. Each objective file is owned by one work stream:
//   land  soft landing (lead)     roll  jump + 360° barrel roll + landing (barrel-roll stream)
//   gas   gas-optimal landing studies (gas-study stream)
import { createLand } from './land.js';
import { createRoll } from './roll.js';
import { createGas } from './gas.js';

export const OBJECTIVES = { land: createLand, roll: createRoll, gas: createGas };

export function createObjective(spec = 'land', ctx = {}) {
  const { name, ...cfg } = typeof spec === 'string' ? { name: spec } : spec;
  const make = OBJECTIVES[name];
  if (!make) throw new Error(`mpc: unknown objective ${name} (have ${Object.keys(OBJECTIVES).join(', ')})`);
  return make(cfg, ctx);
}
