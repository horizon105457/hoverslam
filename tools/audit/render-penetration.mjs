// Render-surface ground penetration: lowest visible render vertex per time (y < 0 = sunk into asphalt).
import fs from 'fs'; import zlib from 'zlib';
import { loadCarNode } from '../lib/car.mjs';
import { unpackRecording, RecordingPlayer } from '../../src/softbody/recording.js';
const name = process.argv[2] || 'roof';
const car = await loadCarNode({ h: 0.065 });
const raw = zlib.gunzipSync(fs.readFileSync(`public/sims/${name}.sim`));
const { meta, frames } = unpackRecording(raw.buffer.slice(raw.byteOffset, raw.byteOffset + raw.length));
const P = new RecordingPlayer(car, meta, frames);
const tn = car.render.tetNodes, tt = meta.tearTime, gt = meta.groupTime || [];
const hist = [];
for (let t = 1.9; t <= P.duration; t += 0.05) {
  const pos = P.sample(t);
  let lo = 1e9, below = 0, tot = 0, who = '';
  for (const m of car.render.meshes) {
    if (m.group > 0 && gt[m.group] <= t) continue;
    for (let v = 0; v < m.aTet.length; v++) {
      const r = m.aTet[v]; if (tt && tt[r] <= t) continue;
      const b1 = m.aBary[v * 3], b2 = m.aBary[v * 3 + 1], b3 = m.aBary[v * 3 + 2], b0 = 1 - b1 - b2 - b3;
      const y = pos[tn[r * 4] * 3 + 1] * b0 + pos[tn[r * 4 + 1] * 3 + 1] * b1 + pos[tn[r * 4 + 2] * 3 + 1] * b2 + pos[tn[r * 4 + 3] * 3 + 1] * b3;
      tot++; if (y < -0.01) below++;
      if (y < lo) { lo = y; who = m.name; }
    }
  }
  hist.push(`t=${t.toFixed(2)} lowest=${lo.toFixed(3)} (${who}) below-1cm=${below}/${tot}`);
}
console.log(hist.join('\n'));
