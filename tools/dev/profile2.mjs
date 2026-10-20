import { loadCarNode } from '../lib/car.mjs';
import { CrashSim } from '../../src/softbody/solver.js';
const car = await loadCarNode({ h: 0.065 });
const sim = new CrashSim(car, { substepHz: 8000 });
const N = sim.N, x = sim.x, R = car.nodes.rest;
let ymin = 1e9; for (let n = 0; n < N; n++) ymin = Math.min(ymin, R[n*3+1]);
for (let i = 0; i < N * 3; i++) x[i] = R[i];
for (let n = 0; n < N; n++) x[n*3+1] += 0.004 - ymin;
sim.settling = true; sim.boxes = [];
let t = performance.now();
for (let s = 0; s < 400; s++) { sim.substep(); if (s % 50 === 49) { let br = 0; for (const b of sim.broken) br += b; console.log(s, (performance.now() - t).toFixed(0), 'ms pairs', sim.npairs, 'broken', br); t = performance.now(); } }
