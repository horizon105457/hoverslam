// Headless solver run: prints performance and damage statistics, optionally dumps a recording.
import fs from 'fs';
import { loadCarNode } from '../lib/car.mjs';
import { CrashSim } from '../../src/softbody/solver.js';
import { PRESETS } from '../../src/scene/layout.js';
const args = Object.fromEntries(process.argv.slice(2).map(a => a.split('=')));
const car = await loadCarNode({ h: +(args.h || 0.065) });
const preset = PRESETS[args.preset || 'roof'];
const sim = new CrashSim(car, { ...preset, substepHz: +(args.sub || 8000), strength: +(args.s || 1), duration: +(args.dur || 5) });
let t0 = Date.now();
sim.setup();
console.log('settle ms', Date.now() - t0, 'nodes', sim.N, 'beams', sim.NB);
// rest-height of the body after settling
const bodyCh = car.chunks.find(c => c.piece === 'body');
function bodyExtent() {
  // extent along the car's own up axis is hard after rotation; use pairwise roof-floor distances instead
  let s = 0, k = 0;
  for (const [a, b] of sampled) { s += Math.hypot(sim.x[a*3]-sim.x[b*3], sim.x[a*3+1]-sim.x[b*3+1], sim.x[a*3+2]-sim.x[b*3+2]); k++; }
  return s / k;
}
const R = car.nodes.rest; const roof = [], floor = [];
for (let n = bodyCh.nodeStart; n < bodyCh.nodeStart + bodyCh.nodeCount; n++) { if (R[n*3+1] > 1.55 && Math.abs(R[n*3]) < 0.4 && R[n*3+2] > -1.0 && R[n*3+2] < 0.2) roof.push(n); if (R[n*3+1] < 0.35 && Math.abs(R[n*3]) < 0.4 && R[n*3+2] > -1.0 && R[n*3+2] < 0.2) floor.push(n); }
const sampled = []; for (let i = 0; i < 60; i++) sampled.push([roof[(i*7919)%roof.length], floor[(i*104729)%floor.length]]);
const front = [], rear = [];
for (let n = bodyCh.nodeStart; n < bodyCh.nodeStart + bodyCh.nodeCount; n++) { if (R[n*3+2] > 1.15) front.push(n); if (R[n*3+2] < -1.2) rear.push(n); }
const fr = []; for (let i = 0; i < 60; i++) fr.push([front[(i*7919)%front.length], rear[(i*104729)%rear.length]]);
const avgDist = (pairs) => pairs.reduce((s,[a,b]) => s + Math.hypot(sim.x[a*3]-sim.x[b*3], sim.x[a*3+1]-sim.x[b*3+1], sim.x[a*3+2]-sim.x[b*3+2]), 0) / pairs.length;
const h0 = avgDist(sampled), l0 = avgDist(fr);
t0 = Date.now();
const dur = +(args.dur || 5);
let frames = 0;
const rec = [];
for (let t = 0; t < dur; t += 0.25) {
  sim.run(0.25, s => { frames++; if (args.dump) rec.push(Float32Array.from(s.x)); });
  const c = sim.bodyCentre();
  const kinds = {}; for (let b = 0; b < sim.NB; b++) if (sim.broken[b]) kinds[sim.kind[b]] = (kinds[sim.kind[b]]||0)+1;
  console.log(`t=${sim.time.toFixed(2)} com=${c.map(v=>v.toFixed(2)).join(',')} height=${(avgDist(sampled)/h0).toFixed(3)} length=${(avgDist(fr)/l0).toFixed(3)} broken=${JSON.stringify(kinds)} parts=${sim.np} pairs=${sim.npairs} wall=${((Date.now()-t0)/1000).toFixed(1)}s`);
}
console.log('detached', sim.events.breaks.filter(e=>e.type==='detach').map(e=>e.piece+'@'+e.t.toFixed(2)).join(' '));
console.log('glass', sim.events.breaks.filter(e=>e.type!=='detach').map(e=>e.group+':'+e.type+'@'+e.t.toFixed(2)).join(' '));
console.log('dust', sim.events.dust.length/7, 'sparks', sim.events.sparks.length/7, 'scrape', sim.events.scrape.length/5, 'tears', sim.events.tears.length);
let maxp = 0; for (const p of sim.plastic) maxp = Math.max(maxp, p); console.log('max plastic', maxp.toFixed(3));
if (args.dump) { const out = new Float32Array(rec.length * sim.N * 3); rec.forEach((r, i) => out.set(r, i * sim.N * 3)); fs.writeFileSync(args.dump, Buffer.from(out.buffer)); console.log('dumped', rec.length, 'frames'); }
