import { loadCarNode } from '../lib/car.mjs';
import { CrashSim } from '../../src/softbody/solver.js';
const car = await loadCarNode({ h: 0.065 });
const sim = new CrashSim(car, { substepHz: 8000 });
const N = sim.N, x = sim.x, v = sim.v, R = car.nodes.rest;
let ymin = 1e9; for (let n = 0; n < N; n++) ymin = Math.min(ymin, R[n*3+1]);
for (let i = 0; i < N*3; i++) x[i] = R[i]; for (let n = 0; n < N; n++) x[n*3+1] += 0.004 - ymin;
sim.settling = true; sim.boxes = [];
for (let s = 0; s < 2400; s++) { sim.substep(); if (s % 4 === 0) for (let i = 0; i < N*3; i++) v[i] *= 0.9; }
sim.settling = false; v.fill(0);
for (let n = 0; n < N; n++) x[n*3+1] += 0.3;
const sel = []; for (let b = 0; b < sim.NB; b++) if (sim.kind[b] === 2 && car.chunks[car.nodes.chunk[sim.bi[b]]].piece === 'bumper_rear') sel.push(b);
for (let s = 0; s < 2470; s++) sim.substep();
let best = sel[0]; for (const b of sel) if (sim.stretchF[b] > sim.stretchF[best]) best = b;
const a = sim.bi[best], c = sim.bj[best];
const L = Math.hypot(x[a*3]-x[c*3], x[a*3+1]-x[c*3+1], x[a*3+2]-x[c*3+2]);
console.log('beam', best, 'L', L.toFixed(4), 'L0', sim.L0[best].toFixed(4), 'sf', sim.stretchF[best], 'comp', sim.comp[best], 'fy', sim.fy[best]);
for (const n of [a, c]) console.log(car.chunks[car.nodes.chunk[n]].name, 'mass', car.nodes.mass[n].toFixed(4), 'v', [v[n*3], v[n*3+1], v[n*3+2]].map(q=>q.toFixed(2)), 'contact', sim.contact[n], 'y', x[n*3+1].toFixed(3), 'allow', car.nodes.allow[n].toFixed(3));
// all beams on node a
for (let b = 0; b < sim.NB; b++) if ((sim.bi[b] === a || sim.bj[b] === a) && sim.kind[b] !== 0) { const o = sim.bi[b] === a ? sim.bj[b] : sim.bi[b]; console.log('  kind', sim.kind[b], 'to', car.chunks[car.nodes.chunk[o]].name, 'C', (Math.hypot(x[a*3]-x[o*3], x[a*3+1]-x[o*3+1], x[a*3+2]-x[o*3+2]) - sim.L0[b]).toFixed(4), 'broken', sim.broken[b]); }
