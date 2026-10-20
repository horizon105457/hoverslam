import { loadCarNode } from '../lib/car.mjs';
import { CrashSim } from '../../src/softbody/solver.js';
const car = await loadCarNode({ h: 0.065 });
const fc = car.chunks.findIndex(c => c.name === 'fender_front_L');
const sim = new CrashSim(car, { substepHz: 8000 });
const N = sim.N, x = sim.x, p = sim.p, v = sim.v, R = car.nodes.rest, dt = sim.dt;
for (let i = 0; i < N * 3; i++) x[i] = R[i];
const ch = car.chunks[fc];
for (let n = 0; n < N; n++) if (car.nodes.chunk[n] !== fc) sim.w[n] = 0;
for (let s = 0; s < 14; s++) {
  for (let n = 0; n < N; n++) { const i = n*3; p[i]=x[i];p[i+1]=x[i+1];p[i+2]=x[i+2]; if (sim.w[n] === 0) continue; v[i+1] -= 9.81*dt; x[i]+=v[i]*dt;x[i+1]+=v[i+1]*dt;x[i+2]+=v[i+2]*dt; }
  sim.solveBeams(dt);
  for (let n = 0; n < N; n++) { const i = n*3; v[i]=(x[i]-p[i])/dt; v[i+1]=(x[i+1]-p[i+1])/dt; v[i+2]=(x[i+2]-p[i+2])/dt; }
}
let mv = 0, mn = 0; for (let n = ch.nodeStart; n < ch.nodeStart + ch.nodeCount; n++) { const s = Math.hypot(v[n*3], v[n*3+1], v[n*3+2]); if (s > mv) { mv = s; mn = n; } }
console.log('fastest', mn, mv, 'kinds of its beams:');
for (let b = 0; b < sim.NB; b++) if (sim.bi[b] === mn || sim.bj[b] === mn) { const o = sim.bi[b] === mn ? sim.bj[b] : sim.bi[b]; console.log(' kind', sim.kind[b], 'other', o, car.chunks[car.nodes.chunk[o]].name, 'w', sim.w[o].toFixed(1), 'L0', sim.L0[b].toFixed(4), 'comp', sim.comp[b], 'fy', sim.fy[b]); }
