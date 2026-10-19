// High-frequency jitter at rest: mean |p(t+dt) - 2p(t) + p(t-dt)| per chunk (mm), recorded frames only.
import fs from 'fs'; import zlib from 'zlib';
import { loadCarNode } from './load-car-node.mjs';
import { unpackRecording, RecordingPlayer } from '../src/sim/recording.js';
const name = process.argv[2] || 'roof';
const car = await loadCarNode({ h: 0.065 });
const raw = zlib.gunzipSync(fs.readFileSync(`public/sims/${name}.sim`));
const { meta, frames } = unpackRecording(raw.buffer.slice(raw.byteOffset, raw.byteOffset + raw.length));
const P = new RecordingPlayer(car, meta, frames);
const t0 = +(process.argv[3] || P.duration - 1);
const dt = frames[1].t - frames[0].t;
const S = [];
for (let t = t0; t <= P.duration - 1e-6; t += dt) S.push(Float32Array.from(P.sample(t)));
const disp = {}, hf = {};
car.chunks.forEach(c => {
  let a = 0, d = 0, k = 0;
  for (let s = 1; s < S.length - 1; s++) for (let n = c.nodeStart; n < c.nodeStart + c.nodeCount; n += 3) {
    let e = 0, f = 0;
    for (let q = 0; q < 3; q++) { const i = n * 3 + q; e += (S[s + 1][i] - 2 * S[s][i] + S[s - 1][i]) ** 2; f += (S[s + 1][i] - S[s][i]) ** 2; }
    a += Math.sqrt(e); d += Math.sqrt(f); k++;
  }
  hf[c.name] = (1000 * a / k).toFixed(2); disp[c.name] = (1000 * d / k).toFixed(2);
});
console.log('frame dt', dt.toFixed(4), 'from', t0);
for (const k in hf) console.log(k.padEnd(18), 'step mm', disp[k].padStart(7), ' 2nd-diff mm', hf[k].padStart(7));
