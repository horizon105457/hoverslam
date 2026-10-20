// Soft-body runs of the gas study (the "real world" proxy): tools/sim/flight.mjs per (strategy, preset), at most
// `par` processes at a time (each is single-threaded, ~2-3 min), outputs experiments/runs/soft-gas/flight-<name>.{sim,json}.
// usage: node tools/flight/gas/soft.mjs [presets=flat,nose,roof] [only=gas,land] [par=4] [dur=5] [outDir=experiments/runs/soft-gas]
import { spawn } from 'child_process';
import fs from 'fs';
import path from 'path';
import { parseArgs } from '../../lib/args.mjs';
import { ROOT } from './lib.mjs';

// strategies = controller specs of tools/sim/flight.mjs: MPC configs (registry `mpc`, as in experiments/configs/gas-*.json)
// and the pd reference (not MPC)
const mpc = c => `mpc:${JSON.stringify(c)}`;
export const STRATEGIES = {
  gas: mpc({ objective: 'gas' }),
  'gas-hc0.9': mpc({ objective: 'gas', optimizer: [{ type: 'xmask', heave_until: 0.9 }, { type: 'cem' }] }),
  'gas-vcap1.0': mpc({ objective: { name: 'gas', v_cap: 1.0 } }),
  'gas-reserve0.8-w20': mpc({ objective: { name: 'gas', reserve: 0.8, w_reserve: 20 } }),
  land: mpc({ objective: 'land' }),
  'pd-b0.9': 'pd:{"brake_frac":0.9}',
};

const a = parseArgs(), presets = (a.presets || 'flat,nose,roof').split(','), only = a.only ? a.only.split(',') : Object.keys(STRATEGIES);
const par = +(a.par || 4), dur = a.dur || '5', outDir = a.outDir || 'experiments/runs/soft-gas';
const jobs = only.flatMap(s => presets.map(p => ({ s, p, name: `${s}-${p}` })));
fs.mkdirSync(path.join(ROOT, outDir), { recursive: true });
let next = 0, live = 0;
const t0 = Date.now();
await new Promise(done => {
  const go = () => {
    while (live < par && next < jobs.length) {
      const j = jobs[next++], ctl = STRATEGIES[j.s];
      const args = ['tools/sim/flight.mjs', `preset=${j.p}`, `controller=${ctl}`, `out=${j.name}`, `dur=${dur}`, `outDir=${outDir}`];
      const log = fs.openSync(path.join(ROOT, outDir, `${j.name}.log`), 'w');
      fs.writeSync(log, `node ${args.map(x => (x.includes('{') ? `'${x}'` : x)).join(' ')}\n`);
      const ch = spawn('node', args, { cwd: ROOT, stdio: ['ignore', log, log] });
      live++;
      ch.on('exit', code => { live--; fs.closeSync(log); console.log(`${j.name}: exit ${code} (${((Date.now() - t0) / 1000).toFixed(0)} s)`); if (next >= jobs.length && live === 0) done(); else go(); });
    }
  };
  go();
});
