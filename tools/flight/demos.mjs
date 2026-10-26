// Reproduce the two supported demos; recordings remain local and Git-ignored.
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { spawnSync } from 'node:child_process';
import { parseArgs } from '../lib/args.mjs';
const args = parseArgs(), root = fileURLToPath(new URL('../../', import.meta.url));
const names = ['nonconvex-stop', 'nonconvex-pad'];
if (args.only && !names.includes(args.only)) throw new Error(`only must be ${names.join(' or ')}`);
const scratch = fs.mkdtempSync(path.join(os.tmpdir(), 'jetcar-demos-'));
for (const name of names.filter(n => !args.only || args.only === n)) {
  const config = JSON.parse(fs.readFileSync(path.join(root, `experiments/configs/${name}.json`), 'utf8'));
  const controller = JSON.stringify(config.controllers[0]), rigid = path.join(scratch, `${name}.json`);
  for (const argv of [
    ['tools/flight/run.mjs', `task=${config.task}`, 'preset=flat', 'seed=0', `controller=${controller}`, `out=${rigid}`],
    ['tools/sim/flight.mjs', `task=${config.task}`, 'preset=flat', `launch=${rigid}`, `controller=${controller}`, `out=${name}`, 'dur=6', 'hz=100'],
  ]) {
    const result = spawnSync(process.execPath, argv, { cwd: root, stdio: 'inherit' });
    if (result.error) throw result.error;
    if (result.status !== 0) throw new Error(`${argv[0]} failed: ${result.signal || result.status}`);
  }
}
console.log(`Rigid launch records retained in ${scratch}`);
