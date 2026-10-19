// Controlled drop: settle, rotate the car (pitch, roll in degrees), drop from height H onto flat ground.
import { loadCarNode } from './load-car-node.mjs';
import { CrashSim } from '../src/sim/solver.js';
const args = Object.fromEntries(process.argv.slice(2).map(a => a.split('=')));
const car = await loadCarNode({ h: +(args.h || 0.065) });
const sim = new CrashSim(car, { substepHz: +(args.sub || 8000), strength: +(args.s || 1) });
const N = sim.N, x = sim.x, v = sim.v, R = car.nodes.rest;
let ymin = 1e9; for (let n = 0; n < N; n++) ymin = Math.min(ymin, R[n*3+1]);
for (let i = 0; i < N*3; i++) x[i] = R[i]; for (let n = 0; n < N; n++) x[n*3+1] += 0.004 - ymin;
sim.settling = true; sim.boxes = [];
const t0 = Date.now();
{ const steps = args.settle ? +args.settle : 7200; for (let s = 0; s < steps; s++) { sim.substep(); const k = s < steps * 0.7 ? 0.9985 : 0.99; for (let i = 0; i < N*3; i++) v[i] *= k; } }
{ let sc = 0, ns = 0; for (let b = 0; b < sim.NB; b++) if (sim.kind[b] === 5) { const a = sim.bi[b], c = sim.bj[b]; sc += Math.hypot(x[a*3]-x[c*3], x[a*3+1]-x[c*3+1], x[a*3+2]-x[c*3+2]) - sim.L0[b]; ns++; } console.log('spring sag', (sc/ns).toFixed(3)); }
{ let nb = 0; for (let b = 0; b < sim.NB; b++) nb += sim.broken[b]; console.log('broken after settle', nb, 'settle s', ((Date.now()-t0)/1000).toFixed(1)); }
sim.settling = false; v.fill(0); sim.plastic.fill(0); sim.damage.fill(0);
const pitch = (+(args.pitch || 0)) * Math.PI / 180, roll = (+(args.roll || 0)) * Math.PI / 180, H = +(args.H || 9);
let cx = 0, cy = 0, cz = 0, mt = 0; for (let n = 0; n < N; n++) { const m = 1/sim.w[n]; cx += x[n*3]*m; cy += x[n*3+1]*m; cz += x[n*3+2]*m; mt += m; } cx/=mt; cy/=mt; cz/=mt;
const cp = Math.cos(pitch), sp = Math.sin(pitch), cr = Math.cos(roll), sr = Math.sin(roll);
let lo = 1e9;
for (let n = 0; n < N; n++) {
  let px = x[n*3]-cx, py = x[n*3+1]-cy, pz = x[n*3+2]-cz;
  // pitch about X (nose down positive), then roll about Z
  let y1 = py*cp - pz*sp, z1 = py*sp + pz*cp; py = y1; pz = z1;
  let x2 = px*cr - py*sr, y2 = px*sr + py*cr; px = x2; py = y2;
  x[n*3] = px; x[n*3+1] = py; x[n*3+2] = pz; lo = Math.min(lo, py);
}
for (let n = 0; n < N; n++) { x[n*3+1] += H - lo; v[n*3+1] = 0; v[n*3+2] = +(args.vz || 0); }
const roofFloor = []; const bodyCh = car.chunks.find(c => c.piece === 'body');
const roof = [], floor = [];
for (let n = bodyCh.nodeStart; n < bodyCh.nodeStart + bodyCh.nodeCount; n++) { if (R[n*3+1] > 1.55 && Math.abs(R[n*3]) < 0.4 && R[n*3+2] > -1.0 && R[n*3+2] < 0.2) roof.push(n); if (R[n*3+1] < 0.35 && Math.abs(R[n*3]) < 0.4 && R[n*3+2] > -1.0 && R[n*3+2] < 0.2) floor.push(n); }
const front = [], rear = []; for (let n = bodyCh.nodeStart; n < bodyCh.nodeStart + bodyCh.nodeCount; n++) { if (R[n*3+2] > 1.15) front.push(n); if (R[n*3+2] < -1.2) rear.push(n); }
const left = [], right = []; for (let n = bodyCh.nodeStart; n < bodyCh.nodeStart + bodyCh.nodeCount; n++) { if (R[n*3] < -0.7 && R[n*3+1] > 0.6) left.push(n); if (R[n*3] > 0.7 && R[n*3+1] > 0.6) right.push(n); }
const mk = (A, B) => { const o = []; for (let i = 0; i < 80; i++) o.push([A[(i*7919)%A.length], B[(i*104729)%B.length]]); return o; };
const P1 = mk(roof, floor), P2 = mk(front, rear), P3 = mk(left, right);
const avg = P => P.reduce((s,[a,b]) => s + Math.hypot(x[a*3]-x[b*3], x[a*3+1]-x[b*3+1], x[a*3+2]-x[b*3+2]), 0) / P.length;
const h0 = avg(P1), l0 = avg(P2), w0 = avg(P3);
const dur = +(args.dur || 2.2);
const t1 = Date.now();
for (let t = 0; t < dur - 1e-6; t += 0.1) {
  sim.run(0.1);
  let ke = 0, maxv = 0, mn = 0; const keP = {}; for (let n = 0; n < N; n++) { if (!sim.alive[n]) continue; const s2 = v[n*3]**2+v[n*3+1]**2+v[n*3+2]**2; ke += 0.5*s2/sim.w[n]; const pc = car.chunks[car.nodes.chunk[n]].piece; keP[pc] = (keP[pc]||0) + 0.5*s2/sim.w[n]; if (s2 > maxv*maxv) { maxv = Math.sqrt(s2); mn = n; } }
  if (args.verbose) console.log('   fastest', car.chunks[car.nodes.chunk[mn]].name, 'KE by piece', Object.entries(keP).sort((a,b)=>b[1]-a[1]).slice(0,5).map(([k,v])=>k+':'+v.toFixed(0)).join(' '));
  const kinds = {}; for (let b = 0; b < sim.NB; b++) if (sim.broken[b]) kinds[sim.kind[b]] = (kinds[sim.kind[b]]||0)+1;
  const c = sim.bodyCentre();
  console.log(`t=${sim.time.toFixed(2)} y=${c[1].toFixed(2)} KE=${(ke/1000).toFixed(1)}kJ vmax=${maxv.toFixed(1)} H=${(avg(P1)/h0).toFixed(3)} L=${(avg(P2)/l0).toFixed(3)} W=${(avg(P3)/w0).toFixed(3)} broken=${JSON.stringify(kinds)} p=${sim.np} pairs=${sim.npairs}`);
}
{ const bp = {}; for (let b = 0; b < sim.NB; b++) if (sim.broken[b] && sim.kind[b] !== 0) { const k = car.chunks[car.nodes.chunk[sim.bi[b]]].piece + '-' + car.chunks[car.nodes.chunk[sim.bj[b]]].piece + ':' + sim.kind[b]; bp[k] = (bp[k]||0)+1; } console.log('broken connectors', JSON.stringify(bp)); }
{ const bp = {}; for (let b = 0; b < sim.NB; b++) if (sim.broken[b] && sim.kind[b] === 0) { const k = car.chunks[car.nodes.chunk[sim.bi[b]]].piece; bp[k] = (bp[k]||0)+1; } console.log('torn STRUCT by piece', JSON.stringify(bp)); }
{ const pp = {}; for (let b = 0; b < sim.NB; b++) if (sim.kind[b] === 0 && !sim.broken[b]) { const k = car.chunks[car.nodes.chunk[sim.bi[b]]].piece; pp[k] = Math.max(pp[k]||0, sim.plastic[b]); } console.log('max plastic by piece', Object.entries(pp).map(([k,v])=>k+':'+v.toFixed(2)).join(' ')); }
console.log('detached', sim.events.breaks.filter(e=>e.type==='detach').map(e=>e.piece+'@'+e.t.toFixed(2)).join(' '));
console.log('glass', sim.events.breaks.filter(e=>e.type!=='detach').map(e=>e.group+'@'+e.t.toFixed(2)).join(' '));
console.log('wall', ((Date.now()-t1)/1000).toFixed(1), 's');
if (args.dump) { const fs = await import('fs'); fs.writeFileSync(args.dump, Buffer.from(Float32Array.from(x).buffer)); }
if (args.pairs) {
  const d0 = 0.84 * sim.h; const hist = {};
  for (let k = 0; k < sim.npairs; k++) { const a = sim.pairs[k*2], b = sim.pairs[k*2+1]; const d = Math.hypot(x[a*3]-x[b*3], x[a*3+1]-x[b*3+1], x[a*3+2]-x[b*3+2]); if (d >= d0) continue; const rd = Math.hypot(R[a*3]-R[b*3], R[a*3+1]-R[b*3+1], R[a*3+2]-R[b*3+2]); const key = car.chunks[car.nodes.chunk[a]].piece + '|' + car.chunks[car.nodes.chunk[b]].piece; (hist[key] ||= []).push([d.toFixed(3), rd.toFixed(3)]); }
  for (const [k, l] of Object.entries(hist)) console.log(k, l.length, JSON.stringify(l.slice(0, 4)));
  // fastest node
  let mv = 0, mn = 0; for (let n = 0; n < N; n++) { const s = Math.hypot(v[n*3], v[n*3+1], v[n*3+2]); if (s > mv) { mv = s; mn = n; } }
  console.log('fastest', mn, car.chunks[car.nodes.chunk[mn]].name, mv.toFixed(2), 'rest', [...R.slice(mn*3, mn*3+3)].map(q=>q.toFixed(2)), 'contact', sim.contact[mn]);
  for (let b = 0; b < sim.NB; b++) if ((sim.bi[b] === mn || sim.bj[b] === mn) && sim.kind[b] !== 0) console.log('  beam kind', sim.kind[b], 'broken', sim.broken[b]);
}
