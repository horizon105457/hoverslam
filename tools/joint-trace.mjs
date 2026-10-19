import { loadCarNode } from './load-car-node.mjs';
import { CrashSim } from '../src/sim/solver.js';
const car = await loadCarNode({ h: 0.065 });
const sim = new CrashSim(car, { substepHz: 8000 });
const N = sim.N, x = sim.x, v = sim.v, R = car.nodes.rest;
let ymin = 1e9; for (let n = 0; n < N; n++) ymin = Math.min(ymin, R[n*3+1]);
for (let i = 0; i < N*3; i++) x[i] = R[i]; for (let n = 0; n < N; n++) x[n*3+1] += 0.004 - ymin;
sim.settling = true; sim.boxes = [];
for (let s = 0; s < 2400; s++) { sim.substep(); if (s % 4 === 0) for (let i = 0; i < N*3; i++) v[i] *= 0.9; }
sim.settling = false; v.fill(0); if (process.argv[2] === "nopairs") sim.solvePairs = () => {}; if (process.argv[2] === "notets") sim.solveTets = () => {};
for (let n = 0; n < N; n++) x[n*3+1] += 0.3;
const sel = []; for (let b = 0; b < sim.NB; b++) if (sim.kind[b] === 2 && car.chunks[car.nodes.chunk[sim.bi[b]]].piece === 'bumper_rear') sel.push(b);
const springs = []; for (let b = 0; b < sim.NB; b++) if (sim.kind[b] === 5) springs.push(b);
const body = car.chunks.find(c => c.piece === 'body');
for (let s = 0; s < 3200; s++) {
  sim.substep();
  if (s % 80 === 0) {
    let mx = 0, br = 0; for (const b of sel) { if (sim.broken[b]) br++; else mx = Math.max(mx, sim.stretchF[b]); }
    let sc = 0; for (const b of springs) { const a = sim.bi[b], c = sim.bj[b]; sc += Math.hypot(x[a*3]-x[c*3], x[a*3+1]-x[c*3+1], x[a*3+2]-x[c*3+2]) - sim.L0[b]; }
    let vy = 0; for (let n = body.nodeStart; n < body.nodeStart + body.nodeCount; n += 10) vy += v[n*3+1]; vy /= Math.ceil(body.nodeCount/10);
    console.log((s/8000).toFixed(3), 'bodyVy', vy.toFixed(2), 'spring avg C', (sc/springs.length).toFixed(3), 'bumper joint max sf', mx.toFixed(4), 'broken', br, '/', sel.length);
  }
}
