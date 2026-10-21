// Soft-body drive design: coasting, straight braking, steer steps / reversals and combined brake + steer at several
// speeds (tools/flight/drive/test.mjs per case, parallel child processes, resumable). The reference for the rigid twins'
// drive model (spec/README.md 5.13) and its calibration (tools/flight/drive/compare.mjs).
// usage: node tools/flight/drive/doe.mjs [run=experiments/runs/drive-doe/<stamp>] [workers=10] [only=id,...]
import fs from 'fs';
import path from 'path';
import { spawn } from 'child_process';
import { fileURLToPath } from 'url';
import { parseArgs } from '../../lib/args.mjs';
import { repo } from '../../lib/paths.mjs';

const TEST = path.join(path.dirname(fileURLToPath(import.meta.url)), 'test.mjs');
// [id, u (m/s), script [[t, brake, steer], ...], dur (s), gas fill]
export const CASES = [
  ['coast10', 10, [], 3.5], ['coast15', 15, [], 3.5], ['coast20', 20, [], 3.5],
  ['brake20', 20, [[0.4, 1, 0]], 3.5], ['brake20h', 20, [[0.4, 0.4, 0]], 3.5], ['brake12', 12, [[0.4, 1, 0]], 2.5], ['brake20e', 20, [[0.4, 1, 0]], 3.5, 0.3],
  ['steer15p', 15, [[0.4, 0, 0.06]], 3], ['steer15m', 15, [[0.4, 0, -0.06]], 3], ['steer20', 20, [[0.4, 0, 0.04]], 3],
  ['steer10', 10, [[0.4, 0, 0.15]], 3], ['steer6', 6, [[0.4, 0, 0.5]], 3.5],
  ['weave15', 15, [[0.4, 0, 0.08], [1.0, 0, -0.08], [1.6, 0, 0]], 3],
  ['bsteer20', 20, [[0.4, 1, 0.1]], 3.5], ['bsteer18h', 18, [[0.4, 0.5, -0.08]], 3.5], ['braketurn20', 20, [[0.4, 1, 0], [1.4, 1, 0.3]], 3.5],
];

const a = parseArgs();
const dir = path.resolve(a.run || repo(`experiments/runs/drive-doe/${new Date().toISOString().slice(0, 19).replace(/[-:]/g, '').replace('T', '-')}`));
fs.mkdirSync(path.join(dir, 'logs'), { recursive: true });
const only = a.only ? new Set(a.only.split(',')) : null;
const todo = CASES.filter(c => (!only || only.has(c[0])) && !fs.existsSync(path.join(dir, `${c[0]}.json.gz`)));
fs.writeFileSync(path.join(dir, 'manifest.json'), JSON.stringify({ cases: CASES.map(([id, u, script, dur, gas]) => ({ id, u, script, dur, gas: gas ?? 0.65 })) }, null, 1));
console.log(`run dir ${dir} · ${todo.length} of ${CASES.length} cases to run`);
const nw = +(a.workers || 10), t0 = Date.now();
let next = 0, live = 0;
await new Promise(fin => {
  if (!todo.length) return fin();
  const go = () => {
    while (live < nw && next < todo.length) {
      const [id, u, script, dur, gas] = todo[next++];
      const log = fs.openSync(path.join(dir, 'logs', `${id}.log`), 'w');
      const ch = spawn('node', [TEST, `u=${u}`, `script=${JSON.stringify(script)}`, `dur=${dur}`, `gas=${gas ?? 0.65}`, `id=${id}`, `outDir=${dir}`], { cwd: repo('.'), stdio: ['ignore', log, log] });
      live++;
      ch.on('exit', code => { live--; fs.closeSync(log); console.log(`${id}: exit ${code} (${((Date.now() - t0) / 1000).toFixed(0)} s) ${fs.readFileSync(path.join(dir, 'logs', `${id}.log`), 'utf8').trim().split('\n').pop()}`); if (next >= todo.length && live === 0) fin(); else go(); });
    }
  };
  go();
});
