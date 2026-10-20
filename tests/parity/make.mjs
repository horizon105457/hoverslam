// Write the JS goldens: node tests/parity/make.mjs [outDir=tests/parity/goldens/js]
import fs from 'fs';
import path from 'path';
import { PARITY, scenarios, runScenario, writeGz } from './lib.mjs';
const out = process.argv[2] || path.join(PARITY, 'goldens', 'js');
fs.mkdirSync(out, { recursive: true });
for (const sc of scenarios()) {
  const traj = await runScenario(sc);
  writeGz(path.join(out, `${sc.id}.json.gz`), traj);
  console.log(`wrote ${sc.id}: ${traj.n} rows, events ${traj.events.map(e => e.type).join(',')}`);
}
