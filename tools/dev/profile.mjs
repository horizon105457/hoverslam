import { loadCarNode } from '../lib/car.mjs';
import { CrashSim } from '../../src/softbody/solver.js';
const car = await loadCarNode({ h: 0.065 });
const sim = new CrashSim(car, { substepHz: 8000 });
const N = sim.N, x = sim.x, R = car.nodes.rest;
for (let i = 0; i < N * 3; i++) x[i] = R[i];
for (let n = 0; n < N; n++) x[n*3+1] += 0.01;
const T = {};
const time = (k, f) => { const t = performance.now(); f(); T[k] = (T[k] || 0) + performance.now() - t; };
for (let s = 0; s < 200; s++) {
  time('broad', () => { if (s % 8 === 0) sim.broadphase(); });
  time('beams', () => sim.solveBeams(sim.dt));
  time('tets', () => { if (s % 3 === 0) sim.solveTets(); });
  time('pairs', () => sim.solvePairs());
  time('static', () => sim.solveStatic(sim.dt));
  time('damp', () => { if (s % 2 === 0) sim.dampBeams(0.06); });
  time('groups', () => { if (s % 8 === 0) sim.checkGroups(); });
}
for (const k in T) console.log(k, (T[k] / 200).toFixed(3), 'ms/substep');
console.log('pairs', sim.npairs);
