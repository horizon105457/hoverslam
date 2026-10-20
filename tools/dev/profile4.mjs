import { loadCarNode } from '../lib/car.mjs';
import { CrashSim } from '../../src/softbody/solver.js';
const car = await loadCarNode({ h: 0.065 });
const sim = new CrashSim(car, { substepHz: 8000 });
const N = sim.N, x = sim.x, R = car.nodes.rest;
let ymin = 1e9; for (let n = 0; n < N; n++) ymin = Math.min(ymin, R[n*3+1]);
for (let i = 0; i < N * 3; i++) x[i] = R[i];
for (let n = 0; n < N; n++) x[n*3+1] += 0.004 - ymin;
sim.settling = true; sim.boxes = [];
for (let s = 0; s < 66; s++) sim.substep();
let mv = 0, mn = -1; for (let n = 0; n < N; n++) { const v = Math.hypot(sim.v[n*3], sim.v[n*3+1], sim.v[n*3+2]); if (v > mv) {mv = v; mn=n;} }
console.log('node', mn, 'mass', car.nodes.mass[mn], 'allow', car.nodes.allow[mn], 'v', mv);
for (let b = 0; b < sim.NB; b++) if (sim.bi[b] === mn || sim.bj[b] === mn) { const o = sim.bi[b] === mn ? sim.bj[b] : sim.bi[b]; const L = Math.hypot(x[mn*3]-x[o*3], x[mn*3+1]-x[o*3+1], x[mn*3+2]-x[o*3+2]); console.log(' beam', b, 'kind', sim.kind[b], 'other', car.chunks[car.nodes.chunk[o]].name, 'mass', car.nodes.mass[o].toFixed(4), 'L', L.toFixed(5), 'L0', sim.L0[b].toFixed(5), 'L0o', sim.L0o[b].toFixed(5), 'comp', sim.comp[b], 'fy', sim.fy[b].toFixed(0), 'pl', sim.plastic[b].toFixed(4), 'broken', sim.broken[b]); }
console.log('pairs', sim.npairs); for (let k = 0; k < sim.npairs; k++) if (sim.pairs[k*2] === mn || sim.pairs[k*2+1] === mn) console.log(' pair', sim.pairs[k*2], sim.pairs[k*2+1]);
