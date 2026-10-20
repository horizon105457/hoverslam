// Run a crash scenario headless and write public/sims/<name>.sim for instant playback in the app.
// usage: node tools/sim/precompute.mjs preset=roof [dur=7 hz=120 speed= s= j=]
import { loadCarNode } from '../lib/car.mjs';
import { parseArgs } from '../lib/args.mjs';
import { readSettled, writeSim } from '../lib/sims.mjs';
import { CrashSim } from '../../src/softbody/solver.js';
import { RecordingEncoder, finalMeta } from '../../src/softbody/recording.js';
import { PRESETS } from '../../src/scene/layout.js';
const args = parseArgs();
const name = args.preset || 'roof';
const P = PRESETS[name];
const car = await loadCarNode({ h: 0.065 });
const settled = readSettled('public/assets/settled.bin', car);
const sc = { ...P, ...(args.speed ? { speed: +args.speed } : {}), strength: +(args.s || P.strength || 1), joints: +(args.j || P.joints || 1), recordHz: +(args.hz || 120), duration: +(args.dur || 7) };
const sim = new CrashSim(car, sc);
sim.setup(settled);
const enc = new RecordingEncoder(car);
const frames = [];
let k = 0;
const t0 = Date.now();
frames.push(enc.encode(sim, true));
sim.run(sc.duration, s => { k++; frames.push(enc.encode(s, k % 4 === 0)); if (k % 60 === 0) process.stdout.write(`\r${name} t=${s.time.toFixed(2)} ${((Date.now() - t0) / 1000).toFixed(0)}s`); });
const meta = finalMeta(sim, enc, { name, label: P.label });
const size = writeSim(name, meta, frames);
console.log(`\n${name}: ${frames.length} frames raw ${(size.raw / 1e6).toFixed(1)}MB gz ${(size.gz / 1e6).toFixed(1)}MB in ${((Date.now() - t0) / 1000).toFixed(0)}s`);
console.log('detached', sim.events.breaks.filter(e => e.type === 'detach').map(e => e.piece + '@' + e.t.toFixed(2)).join(' '));
