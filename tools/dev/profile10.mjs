import { loadCarNode } from '../lib/car.mjs';
import { CrashSim } from '../../src/softbody/solver.js';
const car = await loadCarNode({ h: 0.065 });
const fc = car.chunks.findIndex(c => c.name === 'fender_front_L');
const sim = new CrashSim(car, { substepHz: 8000 });
const N = sim.N, x = sim.x, p = sim.p, v = sim.v, R = car.nodes.rest, dt = sim.dt;
for (let i = 0; i < N * 3; i++) x[i] = R[i];
const ch = car.chunks[fc];
for (let n = 0; n < N; n++) if (car.nodes.chunk[n] !== fc) sim.w[n] = 0;
sim.settling = true;
const maxv = () => { let m = 0; for (let n = ch.nodeStart; n < ch.nodeStart + ch.nodeCount; n++) m = Math.max(m, Math.hypot(v[n*3], v[n*3+1], v[n*3+2])); return m; };
for (let s = 0; s < 120; s++) {
  for (let n = 0; n < N; n++) { const i = n*3; p[i]=x[i];p[i+1]=x[i+1];p[i+2]=x[i+2]; if (sim.w[n] === 0) continue; v[i+1] -= 9.81*dt; x[i]+=v[i]*dt;x[i+1]+=v[i+1]*dt;x[i+2]+=v[i+2]*dt; }
  sim.solveBeams(dt);
  for (let n = 0; n < N; n++) { const i = n*3; v[i]=(x[i]-p[i])/dt; v[i+1]=(x[i+1]-p[i+1])/dt; v[i+2]=(x[i+2]-p[i+2])/dt; }
  if (s % 10 === 0) console.log(s, maxv().toExponential(3));
}
// find nodes with highest speed and their valence
let mv = 0, mn = 0; for (let n = ch.nodeStart; n < ch.nodeStart + ch.nodeCount; n++) { const s = Math.hypot(v[n*3], v[n*3+1], v[n*3+2]); if (s > mv) { mv = s; mn = n; } }
const val = new Map(); for (let b = 0; b < sim.NB; b++) { for (const q of [sim.bi[b], sim.bj[b]]) val.set(q, (val.get(q)||0)+1); }
console.log('fastest', mn, 'valence', val.get(mn), 'rest', [...R.slice(mn*3, mn*3+3)]);
let low = 0; for (let n = ch.nodeStart; n < ch.nodeStart + ch.nodeCount; n++) if ((val.get(n)||0) < 4) low++; console.log('nodes with valence<4:', low);
const hist = {}; for (let n = ch.nodeStart; n < ch.nodeStart + ch.nodeCount; n++) { const k = val.get(n)||0; hist[k]=(hist[k]||0)+1; } console.log(hist);
