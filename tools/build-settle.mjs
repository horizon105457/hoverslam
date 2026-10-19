// Precompute the settled pose (car resting on its springs) so every run can skip ~20 s of settling.
import fs from 'fs';
import { loadCarNode } from './load-car-node.mjs';
import { CrashSim } from '../src/sim/solver.js';
const car = await loadCarNode({ h: 0.065 });
const sim = new CrashSim(car, {});
const t = Date.now();
const pos = sim.settle();
const head = new Uint32Array([car.nodes.count, car.beams.count]);
fs.writeFileSync(new URL('../public/assets/settled.bin', import.meta.url), Buffer.concat([Buffer.from(head.buffer), Buffer.from(pos.buffer)]));
console.log('settled', car.nodes.count, 'nodes in', Date.now() - t, 'ms');
