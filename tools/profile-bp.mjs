import { loadCarNode } from './load-car-node.mjs';
import { CrashSim } from '../src/sim/solver.js';
const car = await loadCarNode({ h: 0.065 });
const sim = new CrashSim(car, { substepHz: 8000 });
const N = sim.N; sim.x.set(car.nodes.rest);
for (let k = 0; k < 5; k++) sim.broadphase();
let t = performance.now(); for (let k = 0; k < 30; k++) sim.broadphase(); console.log('broadphase ms', ((performance.now() - t) / 30).toFixed(2));
// count candidate checks
const inv = 1 / (0.84 * sim.h + sim.margin); const cells = new Map();
for (let n = 0; n < N; n++) { const k = Math.floor(sim.x[n*3]*inv)+','+Math.floor(sim.x[n*3+1]*inv)+','+Math.floor(sim.x[n*3+2]*inv); cells.set(k, (cells.get(k)||0)+1); }
let s = 0; for (const c of cells.values()) s += c*c; console.log('cells', cells.size, 'avg occupancy', (N/cells.size).toFixed(2), 'sum sq', s);
