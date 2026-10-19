import { loadCarNode } from './load-car-node.mjs';
import { CrashSim } from '../src/sim/solver.js';
const car = await loadCarNode({ h: 0.065 });
for (const off of ['none', 'damp', 'tets', 'both']) {
  const sim = new CrashSim(car, { substepHz: 8000 });
  if (off === 'damp' || off === 'both') sim.dampBeams = () => {};
  if (off === 'tets' || off === 'both') sim.solveTets = () => {};
  const N = sim.N, x = sim.x, R = car.nodes.rest;
  let ymin = 1e9; for (let n = 0; n < N; n++) ymin = Math.min(ymin, R[n*3+1]);
  for (let i = 0; i < N * 3; i++) x[i] = R[i];
  for (let n = 0; n < N; n++) x[n*3+1] += 0.004 - ymin;
  sim.settling = true; sim.boxes = [];
  for (let s = 0; s < 120; s++) sim.substep();
  let mv = 0; for (let n = 0; n < N; n++) mv = Math.max(mv, Math.hypot(sim.v[n*3], sim.v[n*3+1], sim.v[n*3+2]));
  console.log(off, 'max v after 120 steps', mv.toFixed(3));
}
