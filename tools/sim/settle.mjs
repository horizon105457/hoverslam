// Precompute the settled pose (car resting on its springs) so every run can skip ~20 s of settling.
// usage: node tools/sim/settle.mjs [out=public/assets/settled.bin]
import { loadCarNode } from '../lib/car.mjs';
import { parseArgs } from '../lib/args.mjs';
import { writeSettled } from '../lib/sims.mjs';
import { CrashSim } from '../../src/softbody/solver.js';
const args = parseArgs();
const car = await loadCarNode({ h: 0.065 });
const sim = new CrashSim(car, {});
const t = Date.now();
const pos = sim.settle();
writeSettled(args.out || 'public/assets/settled.bin', car, pos);
console.log('settled', car.nodes.count, 'nodes in', Date.now() - t, 'ms');
