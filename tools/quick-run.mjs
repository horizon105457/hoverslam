// Short headless run for A/B checks: prints break events. usage: node tools/quick-run.mjs preset=roof dur=0.4 [norest=1]
import fs from 'fs';
import { loadCarNode } from './load-car-node.mjs';
import { CrashSim } from '../src/sim/solver.js';
import { PRESETS } from '../src/scene/layout.js';
const args = Object.fromEntries(process.argv.slice(2).map(a => a.split('=')));
const name = args.preset || 'roof';
const car = await loadCarNode({ h: 0.065 });
const buf = fs.readFileSync(new URL('../public/assets/settled.bin', import.meta.url));
const settled = new Float32Array(buf.buffer.slice(buf.byteOffset + 8, buf.byteOffset + buf.length));
const sc = { ...PRESETS[name], recordHz: 100, duration: +(args.dur || 0.4) };
const sim = new CrashSim(car, sc);
if (args.norest) sim.restDamping = () => {};
sim.setup(settled);
sim.run(sc.duration, s => {});
console.log(sim.events.breaks.map(e => e.type + ':' + (e.piece || e.group) + '@' + e.t.toFixed(3)).join(' ') || 'no breaks');
