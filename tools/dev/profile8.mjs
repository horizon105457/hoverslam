import { loadCarNode } from '../lib/car.mjs';
import { CrashSim } from '../../src/softbody/solver.js';
const car = await loadCarNode({ h: 0.065 });
const fc = car.chunks.findIndex(c => c.name === 'fender_front_L');
for (const mode of ['noJoints', 'noPlastic', 'noJointsNoPlastic', 'noGravity']) {
  const sim = new CrashSim(car, { substepHz: 8000 });
  const N = sim.N, x = sim.x, R = car.nodes.rest;
  for (let i = 0; i < N * 3; i++) x[i] = R[i];
  for (let n = 0; n < N; n++) if (car.nodes.chunk[n] !== fc) sim.w[n] = 0;
  if (mode.includes('noJoints')) for (let b = 0; b < sim.NB; b++) if (sim.kind[b] === 2) sim.broken[b] = 1;
  if (mode.includes('noPlastic')) sim.fy.fill(1e12);
  sim.settling = true; sim.boxes = [];
  if (mode === 'noGravity') { for (let s = 0; s < 200; s++) { sim.substep(); for (let n = 0; n < N; n++) sim.v[n*3+1] += 9.81 * sim.dt; } }
  else for (let s = 0; s < 200; s++) sim.substep();
  let mv = 0; for (let n = 0; n < N; n++) mv = Math.max(mv, Math.hypot(sim.v[n*3], sim.v[n*3+1], sim.v[n*3+2]));
  console.log(mode, 'max v', mv.toFixed(4));
}
