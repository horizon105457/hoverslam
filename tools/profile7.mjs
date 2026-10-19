import { loadCarNode } from './load-car-node.mjs';
import { CrashSim } from '../src/sim/solver.js';
const car = await loadCarNode({ h: 0.065 });
const fc = car.chunks.findIndex(c => c.name === 'fender_front_L');
// duplicate / degenerate beams in fender
const seen = new Map(); let dup = 0, self = 0;
for (let b = 0; b < car.beams.count; b++) { const a = car.beams.i[b], c = car.beams.j[b]; if (a === c) self++; const k = Math.min(a,c)+'_'+Math.max(a,c); if (seen.has(k)) dup++; seen.set(k, b); }
console.log('dup beams', dup, 'self beams', self);
const sim = new CrashSim(car, { substepHz: 8000 });
const N = sim.N, x = sim.x, R = car.nodes.rest;
for (let i = 0; i < N * 3; i++) x[i] = R[i];
for (let n = 0; n < N; n++) if (car.nodes.chunk[n] !== fc) sim.w[n] = 0;
sim.settling = true; sim.boxes = [];
for (let s = 0; s < 200; s++) sim.substep();
let mv = 0, mn = 0; for (let n = 0; n < N; n++) { const v = Math.hypot(sim.v[n*3], sim.v[n*3+1], sim.v[n*3+2]); if (v > mv) { mv = v; mn = n; } }
console.log('fender only: max v', mv, 'node', mn);
// count beams of fender kind
const kinds = {}; for (let b = 0; b < car.beams.count; b++) if (car.nodes.chunk[car.beams.i[b]] === fc || car.nodes.chunk[car.beams.j[b]] === fc) kinds[car.beams.kind[b]] = (kinds[car.beams.kind[b]]||0)+1; console.log(kinds);
