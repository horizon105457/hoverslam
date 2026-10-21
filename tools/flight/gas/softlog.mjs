// Telemetry of a soft-body flight recording for film overlays (gas gauge, thrust bars): the thrust log meta.propulsion
// resampled at `fps` from t0, plus the landing summary. Columns: t (sim s), y (world height of the body origin, m; at rest
// on the street ≈ 0.56), vy (vertical speed, m/s, + up), Fb (bottom group thrust, N), F (all nozzles, N), gas (kg used
// since t = 0), tank (kg left), valve_bottom (largest bottom opening 0..1).
// usage: node tools/flight/gas/softlog.mjs public/sims/flight-<run>.sim out=<file.json> [fps=30 t0=0 t1=end]
import fs from 'fs';
import zlib from 'zlib';
import { parseArgs } from '../../lib/args.mjs';
import { unpackRecording } from '../../../src/softbody/recording.js';
import { GROUPS, NU } from '../../../src/flight/contract.js';

const a = parseArgs(), file = Object.keys(a).find(k => a[k] === true && k.endsWith('.sim'));
if (!file || !a.out) throw new Error('usage: node tools/flight/gas/softlog.mjs <flight-*.sim> out=<json> [fps=30 t0=0 t1=end]');
const raw = zlib.gunzipSync(fs.readFileSync(file));
const { meta } = unpackRecording(raw.buffer.slice(raw.byteOffset, raw.byteOffset + raw.byteLength));
const L = meta.propulsion, fps = +(a.fps || 30), t0 = +(a.t0 || 0), t1 = +(a.t1 ?? L.t[L.t.length - 1]);
const at = (col, t, f) => { // linear in time between log rows
  const T = L.t; let k = Math.min(T.length - 2, Math.max(0, Math.floor((t - T[0]) * L.hz)));
  while (k < T.length - 2 && T[k + 1] < t) k++;
  const u = Math.min(1, Math.max(0, (t - T[k]) / (T[k + 1] - T[k]))), x = f(col[k]), y = f(col[k + 1]);
  return x + (y - x) * u;
};
const sum = (r, idx) => idx.reduce((s, i) => s + r[i], 0), all = [...Array(NU).keys()];
const rows = [];
for (let t = t0; t <= t1 + 1e-9; t += 1 / fps) rows.push({
  t: +t.toFixed(4), y: +at(L.state, t, s => s[1]).toFixed(3), vy: +at(L.state, t, s => s[8]).toFixed(3),
  Fb: Math.round(at(L.thrust, t, r => sum(r, GROUPS.bottom))), F: Math.round(at(L.thrust, t, r => sum(r, all))),
  gas: +(L.tank[0][0] - at(L.tank, t, r => r[0])).toFixed(2), tank: +at(L.tank, t, r => r[0]).toFixed(2),
  valve_bottom: +at(L.valve, t, r => Math.max(...GROUPS.bottom.map(i => r[i]))).toFixed(3),
});
const out = { format: 'fall-telemetry@1', source: file, fps, t0, controller: meta.flight.controller, controllerConfig: meta.flight.controllerConfig ?? null,
  metrics: meta.flight.metrics, events: meta.flight.events, gas0: L.tank[0][0], rows };
fs.writeFileSync(a.out, JSON.stringify(out));
console.log(`${rows.length} rows -> ${a.out} (gas ${meta.flight.metrics.gas_used_kg.toFixed(1)} kg, touchdown ${meta.flight.metrics.touchdown_t?.toFixed(2)} s)`);
