import { loadCarNode } from '../lib/car.mjs';
import { CrashSim } from '../../src/softbody/solver.js';
const car = await loadCarNode({ h: 0.065 });
const fc = car.chunks.findIndex(c => c.name === 'fender_front_L');
const sim = new CrashSim(car, { substepHz: 8000 });
const N = sim.N, x = sim.x, R = car.nodes.rest;
for (let i = 0; i < N * 3; i++) x[i] = R[i];
for (let n = 0; n < N; n++) if (car.nodes.chunk[n] !== fc) sim.w[n] = 0;
const drift = () => { let m = 0; for (let i = 0; i < N*3; i++) m = Math.max(m, Math.abs(x[i] - R[i])); return m; };
sim.solveBeams(sim.dt); console.log('after beams', drift());
sim.solveTets(); console.log('after tets', drift());
sim.solveStatic(sim.dt); console.log('after static', drift());
// perturb one node slightly and iterate beams only
x[car.chunks[fc].nodeStart * 3 + 1] += 1e-6;
for (let it = 0; it < 50; it++) { sim.solveBeams(sim.dt); if (it % 10 === 9) console.log('iter', it, 'drift', drift()); }
