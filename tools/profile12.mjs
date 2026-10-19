import { loadCarNode } from './load-car-node.mjs';
import { CrashSim } from '../src/sim/solver.js';
const car = await loadCarNode({ h: 0.065 });
const fc = car.chunks.findIndex(c => c.name === 'fender_front_L');
const ch = car.chunks[fc];
// connected components of the fender through STRUCT beams, and "weak" connectivity (articulation via single nodes)
const adj = new Map();
for (let b = 0; b < car.beams.count; b++) { if (car.beams.kind[b] !== 0) continue; const a = car.beams.i[b], c = car.beams.j[b]; if (car.nodes.chunk[a] !== fc) continue; (adj.get(a) || adj.set(a, []).get(a)).push(c); (adj.get(c) || adj.set(c, []).get(c)).push(a); }
const seen = new Set(); const comps = [];
for (let n = ch.nodeStart; n < ch.nodeStart + ch.nodeCount; n++) { if (seen.has(n)) continue; const st = [n]; seen.add(n); let cnt = 0; while (st.length) { const q = st.pop(); cnt++; for (const o of adj.get(q) || []) if (!seen.has(o)) { seen.add(o); st.push(o); } } comps.push(cnt); }
console.log('fender STRUCT components', comps);
