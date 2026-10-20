// Re-run the Python goldens in the JS twin and report deviations: node tests/parity/check.mjs [dir=goldens/py]
// Also cross-checks the two golden sets against each other (both producers ran the same scenarios).
import fs from 'fs';
import path from 'path';
import { PARITY, readGz, runScenario, compare, printReport } from './lib.mjs';
const src = process.argv[2] || path.join(PARITY, 'goldens', 'py');
const reports = {};
for (const f of fs.readdirSync(src).filter(f => f.endsWith('.json.gz')).sort()) {
  const golden = readGz(path.join(src, f));
  reports[golden.scenario.id] = compare(golden, await runScenario(golden.scenario, golden));
}
let ok = printReport(reports, `JS re-run of ${path.relative(path.join(PARITY, '../..'), src)}`);
const other = path.join(PARITY, 'goldens', 'js');
if (!process.argv[2] && fs.existsSync(other)) {
  const cross = {};
  for (const f of fs.readdirSync(src).filter(f => f.endsWith('.json.gz') && fs.existsSync(path.join(other, f))).sort()) {
    const a = readGz(path.join(src, f)), b = readGz(path.join(other, f));
    cross[a.scenario.id] = compare(a, b);
  }
  ok = printReport(cross, '\nPython goldens vs JS goldens (independent runs)') && ok;
}
process.exit(ok ? 0 : 1);
