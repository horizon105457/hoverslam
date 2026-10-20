import { loadCarNode } from '../lib/car.mjs';
import { CrashSim } from '../../src/softbody/solver.js';
const car = await loadCarNode({ h: 0.065 });
const sim = new CrashSim(car, { substepHz: 8000 });
const N = sim.N, x = sim.x, R = car.nodes.rest;
let ymin = 1e9; for (let n = 0; n < N; n++) ymin = Math.min(ymin, R[n*3+1]);
for (let i = 0; i < N * 3; i++) x[i] = R[i];
for (let n = 0; n < N; n++) x[n*3+1] += 0.004 - ymin;
sim.settling = true; sim.boxes = [];
const orig = sim.breakBeam.bind(sim); let logged = 0;
sim.breakBeam = (b) => { if (logged++ < 15) { const a = sim.bi[b], c = sim.bj[b]; console.log('step', sim.step, 'kind', sim.kind[b], car.chunks[car.nodes.chunk[a]].name, car.chunks[car.nodes.chunk[c]].name, 'L0', sim.L0[b].toFixed(4), 'L0o', sim.L0o[b].toFixed(4), 'fy', sim.fy[b], 'plastic', sim.plastic[b]); } orig(b); };
let maxv = 0;
for (let s = 0; s < 100; s++) { sim.substep(); let mv = 0, mn=-1; for (let n = 0; n < N; n++) { const v = Math.hypot(sim.v[n*3], sim.v[n*3+1], sim.v[n*3+2]); if (v > mv) {mv = v; mn=n;} } if (s % 5 === 0 || mv > 1) console.log(s, 'max v', mv.toFixed(3), car.chunks[car.nodes.chunk[mn]].name, [...R.slice(mn*3,mn*3+3)].map(v=>v.toFixed(2))); if (mv > 5) break; }
