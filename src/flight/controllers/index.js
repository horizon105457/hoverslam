// Controller registry (same names and semantics in python/coldgas/controllers). Interface (src/flight/contract.js):
//   reset({task, seed}) once per episode, act(state, info) -> u_cmd (Float64Array(NU), each in [0, 1]) every
//   CONTROL_DT; act sees only FlightState fields (twin or soft-body estimate), the task and the spec.
// Config: a name ('zero' | 'pd' | 'replay' | 'mlp') or {name, ...settings}; pd settings default to defaults.json;
// replay needs {traj: coldgas-traj@1 object}, mlp {policy: coldgas-policy@1 object}. loadController() also
// resolves {file} (Node) or {url} (browser) to that object first.
import { createZero } from './zero.js';
import { createPD } from './pd.js';
import { createReplay } from './replay.js';
import { createMLP } from './mlp.js';
import defaults from './defaults.json' with { type: 'json' };

export const CONTROLLERS = { zero: createZero, pd: createPD, replay: createReplay, mlp: createMLP };
export const CONTROLLER_DEFAULTS = defaults;

export function createController(nameOrConfig = 'zero') {
  const cfg = typeof nameOrConfig === 'string' ? { name: nameOrConfig } : { ...nameOrConfig };
  const make = CONTROLLERS[cfg.name];
  if (!make) throw new Error(`unknown controller ${cfg.name} (have ${Object.keys(CONTROLLERS).join(', ')})`);
  const { name, ...settings } = cfg;
  return make(settings);
}

/** Async variant: {name:'replay', file|url} -> traj, {name:'mlp', file|url} -> policy, then createController. */
export async function loadController(nameOrConfig) {
  const cfg = typeof nameOrConfig === 'string' ? { name: nameOrConfig } : { ...nameOrConfig };
  const key = cfg.name === 'replay' ? 'traj' : cfg.name === 'mlp' ? 'policy' : null;
  if (key && !cfg[key] && (cfg.file || cfg.url)) {
    if (cfg.url) cfg[key] = await (await fetch(cfg.url)).json();
    else { const fs = await import(/* @vite-ignore */ ['f', 's'].join('')); cfg[key] = JSON.parse(fs.readFileSync(cfg.file, 'utf8')); }
  }
  return createController(cfg);
}
