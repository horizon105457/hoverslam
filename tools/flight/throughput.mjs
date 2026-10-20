// JS env throughput: env (control) steps per second with random valve commands, resets included.
// usage: node tools/flight/throughput.mjs [task=escape-soft-landing] [seconds=5]
import { parseArgs } from '../lib/args.mjs';
import { ColdGasEnv } from '../../src/flight/env.js';
import { mulberry32 } from '../../src/flight/rng.js';
const a = parseArgs(), task = a.task || 'escape-soft-landing', secs = +(a.seconds || 5);
const env = new ColdGasEnv(task, { seed: 0 }), r = mulberry32(1), u = new Float64Array(10);
env.reset(0);
let n = 0, eps = 0;
const t0 = performance.now();
while (performance.now() - t0 < secs * 1000) {
  for (let i = 0; i < 10; i++) u[i] = r() < 0.5 ? r() : 0;
  const [, , te, tr] = env.step(u);
  n++;
  if (te || tr) env.reset(++eps);
}
console.log(`JS env ${task}: ${(n / ((performance.now() - t0) / 1000)).toFixed(0)} steps/s (${eps} episodes)`);
