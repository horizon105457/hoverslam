// Optimizer registry of the MPC. An optimizer is created once per episode, `create(settings, {rng, nv})`, and
// improves the warm-started schedule in place: solve(θ, P) -> {cost} with P = Planner (dim, lo, hi, cost(θ)).
//   mppi, cem         sampling            (optim/mppi.js, optim/cem.js)
//   grad              finite-difference projected descent  (optim/grad.js)
//   hybrid            stages run in sequence on the same θ, e.g. [{type:'cem', iters:2}, {type:'grad', iters:2}]
// Settings of a type come from the MPC config's block of that name (mpc.cem, mpc.grad, ...), stage overrides on
// top. Additional optimizers: optim/extra.js (exports `optimizers = {name: create}`), merged in below.
import { createMPPIOpt } from './mppi.js';
import { createCEM } from './cem.js';
import { createGrad } from './grad.js';
import { optimizers as EXTRA } from './extra.js';

export const OPTIMIZERS = { mppi: createMPPIOpt, cem: createCEM, grad: createGrad, ...EXTRA };

/** spec: 'cem' | {type, ...overrides} | [stage, ...] | 'hybrid' (= c.hybrid.stages); c: full MPC settings. */
export function createOptimizer(spec, c, ctx) {
  if (spec === 'hybrid') spec = c.hybrid.stages;
  if (Array.isArray(spec)) {
    const stages = spec.map(s => createOptimizer(s, c, ctx));
    return {
      name: `hybrid(${stages.map(s => s.name).join('>')})`, stages,
      solve(theta, P) { let r = null; for (const s of stages) r = s.solve(theta, P); return r; },
    };
  }
  const { type, ...over } = typeof spec === 'string' ? { type: spec } : spec;
  const make = OPTIMIZERS[type];
  if (!make) throw new Error(`mpc: unknown optimizer ${type} (have ${Object.keys(OPTIMIZERS).join(', ')}, hybrid)`);
  return make({ ...(c[type] || {}), ...over }, ctx);
}
