// Additional MPC optimizers (owned by the gas-study work stream, 2026-10-20 round): export them here as
// `optimizers = { name: (settings, {rng, nv}) => ({name, solve(θ, P) -> {cost}}) }`; they join the registry in
// optim/index.js and can be used by name or as hybrid stages. Settings come from the MPC config block of the same
// name (merged over each optimizer's own defaults; these blocks are not in defaults.json).
//   xlbfgs  projected L-BFGS with finite-difference gradients (x-lbfgs.js)
//   xinit   stage: time-varying cold-start schedule (x-stages.js)
//   xmask   stage: heave-only coast, the collective held at 0 until arming + heave_until (x-stages.js)
import { createLBFGS } from './x-lbfgs.js';
import { createInit, createMask } from './x-stages.js';

export const optimizers = { xlbfgs: createLBFGS, xinit: createInit, xmask: createMask };
