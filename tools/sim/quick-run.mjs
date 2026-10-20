// Short headless run for A/B checks: prints break events. usage: node tools/sim/quick-run.mjs preset=roof dur=0.4 [norest=1]
import { loadCarNode } from '../lib/car.mjs';
import { parseArgs } from '../lib/args.mjs';
import { readSettled } from '../lib/sims.mjs';
import { CrashSim } from '../../src/softbody/solver.js';
import { PRESETS } from '../../src/scene/layout.js';
const args = parseArgs();
const name = args.preset || 'roof';
const car = await loadCarNode({ h: 0.065 });
const settled = readSettled('public/assets/settled.bin', car);
const sc = { ...PRESETS[name], recordHz: 100, duration: +(args.dur || 0.4) };
const sim = new CrashSim(car, sc);
if (args.norest) sim.restDamping = () => {};
sim.setup(settled);
sim.run(sc.duration, () => {});
console.log(sim.events.breaks.map(e => e.type + ':' + (e.piece || e.group) + '@' + e.t.toFixed(3)).join(' ') || 'no breaks');
