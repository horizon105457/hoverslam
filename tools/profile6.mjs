import { loadCarNode } from './load-car-node.mjs';
import { CrashSim } from '../src/sim/solver.js';
import { BEAM } from '../src/car/carPrep.js';
const car = await loadCarNode({ h: 0.065 });
const sim = new CrashSim(car, { substepHz: 8000 });
const N = sim.N, x = sim.x, R = car.nodes.rest;
let mm = 1e9; for (const m of car.nodes.mass) mm = Math.min(mm, m); console.log('min mass', mm);
let ymin = 1e9; for (let n = 0; n < N; n++) ymin = Math.min(ymin, R[n*3+1]);
for (let i = 0; i < N * 3; i++) x[i] = R[i];
for (let n = 0; n < N; n++) x[n*3+1] += 0.004 - ymin;
sim.settling = true; sim.boxes = [];
const fc = car.chunks.findIndex(c => c.name === 'fender_front_L');
for (let s = 0; s < 76; s++) {
  sim.substep();
  if (s < 40) continue;
  // worst violated beam touching fender
  let worst = 0, wb = -1;
  for (let b = 0; b < sim.NB; b++) { const a = sim.bi[b], c = sim.bj[b]; if (car.nodes.chunk[a] !== fc && car.nodes.chunk[c] !== fc) continue; const L = Math.hypot(x[a*3]-x[c*3], x[a*3+1]-x[c*3+1], x[a*3+2]-x[c*3+2]); const e = Math.abs(L - sim.L0[b]); if (e > worst) { worst = e; wb = b; } }
  const a = sim.bi[wb], c = sim.bj[wb];
  console.log(s, 'worst', worst.toExponential(2), 'kind', sim.kind[wb], car.chunks[car.nodes.chunk[a]].name, car.chunks[car.nodes.chunk[c]].name, 'contact', sim.contact[a], sim.contact[c], 'y', x[a*3+1].toFixed(3), x[c*3+1].toFixed(3), 'allow', car.nodes.allow[a].toFixed(3), car.nodes.allow[c].toFixed(3));
}
